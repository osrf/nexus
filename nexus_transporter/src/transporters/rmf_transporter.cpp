/*
 * Copyright (C) 2025 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nexus_orchestrator_msgs/msg/workcell_state.hpp>
#include <nexus_orchestrator_msgs/msg/work_order_state.hpp>
#include <nexus_transporter/Itinerary.hpp>
#include <nexus_transporter/Transporter.hpp>
#include <nexus_transporter_msgs/msg/destination.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rmf_building_map_msgs/msg/building_map.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_request.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_result.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>
#include <std_msgs/msg/string.hpp>
#include <rmf_task_ros2/bidding/Auctioneer.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace nexus_transporter {

using ApiRequest = rmf_task_msgs::msg::ApiRequest;
using ApiResponse = rmf_task_msgs::msg::ApiResponse;
using BuildingMap = rmf_building_map_msgs::msg::BuildingMap;
using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
using IngestorRequest = rmf_ingestor_msgs::msg::IngestorRequest;
using IngestorResult = rmf_ingestor_msgs::msg::IngestorResult;
using TaskStateUpdate = std_msgs::msg::String;
using WorkOrderState = nexus_orchestrator_msgs::msg::WorkOrderState;

class RmfTransporter : public Transporter
{
public:

  struct ItineraryQuery
  {
    std::string job_id;

    std::vector<Destination> destinations;

    Transporter::ItineraryQueryCompleted completed_cb;

    std::optional<rmf_task_ros2::bidding::Response::Proposal> winner =
      std::nullopt;
  };

  struct OngoingItinerary
  {
    Itinerary itinerary;

    Transporter::TransporterState transporter_state;

    Transporter::TransportFeedback feedback_cb;

    Transporter::TransportCompleted completed_cb;
  };

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;

  rclcpp::Node::SharedPtr _internal_node;

  std::shared_ptr<std::thread> _spin_thread;

  bool _ready = false;

  int _bidding_time_window_seconds = 2;
  int _itinerary_expiration_seconds = 60;

  std::shared_ptr<rmf_task_ros2::bidding::Auctioneer> _auctioneer = nullptr;

  // Used for bidding only
  std::unordered_map<std::string, ItineraryQuery>
  _rmf_task_id_to_itinerary_query = {};

  // Used for transportation requests
  std::unordered_map<std::string, OngoingItinerary>
  _itinerary_id_to_unconfirmed_itineraries = {};
  std::unordered_map<std::string, OngoingItinerary>
  _rmf_task_id_to_ongoing_itinerary = {};

  // Used to signal RMF dispensers / ingestors
  std::unordered_set<std::string> _pending_ingestor_task_ids;

  // Used for cancellation only
  std::unordered_map<std::string, std::string>
  _cancellation_request_id_to_rmf_task_id = {};

  // Used for re-using past bid results
  // TODO: cleanup expired itineraries when new itineraries are requested
  std::unordered_map<std::string, Itinerary> _job_id_to_itinerary = {};

  // RMF interface
  BuildingMap::SharedPtr _building_map = nullptr;
  std::unordered_set<std::string> _waypoints = {};
  rclcpp::Subscription<rmf_building_map_msgs::msg::BuildingMap>::SharedPtr
    _building_map_sub = nullptr;

  // Task interface
  rclcpp::Publisher<ApiRequest>::SharedPtr _api_request_pub = nullptr;
  rclcpp::Subscription<ApiResponse>::SharedPtr _api_response_sub = nullptr;
  rclcpp::Subscription<TaskStateUpdate>::SharedPtr _task_state_sub = nullptr;

  // Dispenser and ingestor interface
  // Used for signaling transporter completion. The transporter will report completion
  // as soon as the robot reaches its last destination and publishes an ingestor request,
  // to request the workcell to pickup its item.
  // A mock dispenser interface is optionally provided for dispensing from workcells,
  // useful for validating transportation if the logic to dispense item on robots
  // from workcells is not implemented
  rclcpp::Subscription<IngestorRequest>::SharedPtr _ingestor_request_sub =
    nullptr;
  rclcpp::Publisher<IngestorResult>::SharedPtr _ingestor_result_pub = nullptr;
  rclcpp::Subscription<DispenserRequest>::SharedPtr
    _mock_dispenser_request_sub = nullptr;
  rclcpp::Publisher<DispenserResult>::SharedPtr _mock_dispenser_result_pub =
    nullptr;

  // Subscription to work order states to cancel any ongoing dispenser or
  // ingestor requests
  rclcpp::Subscription<WorkOrderState>::SharedPtr _wo_state_sub =
    nullptr;
  std::unordered_map<std::string, WorkOrderState::ConstSharedPtr>
  _cancelled_wo = {};

  // TODO(ac): support ACTION_TRANSIT with a basic go-to-place.
  std::optional<std::string> _action_to_activity_category(uint8_t action)
  {
    switch (action)
    {
      case Destination::ACTION_PICKUP:
        return "pickup";
      case Destination::ACTION_DROPOFF:
        return "dropoff";
      default:
        return std::nullopt;
    }
  }

  std::optional<nlohmann::json> _generate_dispatch_task_request_json(
    const std::vector<Destination>& destinations)
  {
    auto n = this->_node.lock();
    if (!n)
    {
      std::cerr
        << "RmfTransporter::_generate_dispatch_task_request_json - invalid node"
        << std::endl;
      return std::nullopt;
    }

    nlohmann::json r;
    r["unix_millis_request_time"] = 0;
    r["requester"] = n->get_name();
    r["category"] = "compose";
    nlohmann::json d;
    d["category"] = "multi_delivery";
    d["phases"] = nlohmann::json::array();
    nlohmann::json activity;
    activity["category"] = "sequence";
    activity["description"]["activities"] = nlohmann::json::array();
    for (const auto& d : destinations)
    {
      nlohmann::json a;
      const auto category = _action_to_activity_category(d.action);
      if (!category.has_value())
      {
        RCLCPP_ERROR(
          n->get_logger(),
          "Invalid action [%hhu] assigned for destination [%s], only PICKUP "
          "and DROPOFF are supported.",
          d.action, d.name.c_str());
        continue;
      }
      a["category"] = *category;
      nlohmann::json p;
      p["place"] = d.name;
      // TODO(luca) We should assign a handler that is related to the workcell.
      // For now the assumption is that a location has only one handler
      p["handler"] = d.name;
      p["payload"] = nlohmann::json::array();
      a["description"] = p;
      activity["description"]["activities"].push_back(a);

      for (const auto& item : d.items)
      {
        nlohmann::json payload;
        payload["sku"] = item.item_id;
        payload["quantity"] = 1;
        p["payload"].push_back(payload);
      }
    }
    nlohmann::json act_obj;
    act_obj["activity"] = activity;
    d["phases"].push_back(act_obj);
    r["description"] = d;

    return r;
  }

  std::optional<ApiRequest> _generate_dispatch_api_message(
    const Itinerary& itinerary)
  {
    auto n = this->_node.lock();
    if (!n)
    {
      std::cerr
        << "RmfTransporter::_generate_dispatch_api_message - "
        << "invalid node"
        << std::endl;
      return std::nullopt;
    }

    const auto request_json =
      _generate_dispatch_task_request_json(itinerary.destinations());
    if (!request_json.has_value())
    {
      RCLCPP_ERROR(
        n->get_logger(), "failed to generate dispatch task request json");
      return std::nullopt;
    }

    const auto fleet_name = itinerary.metadata("fleet_name");
    const auto robot_name = itinerary.metadata("robot_name");

    nlohmann::json j;
    if (!fleet_name.has_value() && !robot_name.has_value())
    {
      j["type"] = "dispatch_task_request";
    }
    else
    {
      j["type"] = "robot_task_request";
      j["fleet"] = fleet_name.has_value() ? fleet_name.value() : nullptr;
      j["robot"] = robot_name.has_value() ? robot_name.value() : nullptr;
    }
    j["request"] = request_json.value();

    RCLCPP_DEBUG(n->get_logger(), "%s", j.dump(4).c_str());

    ApiRequest msg;
    msg.json_msg = j.dump();

    const auto work_order_id = _get_work_order_id_from_rmf_task_id(
      itinerary.id());
    if (!work_order_id.has_value())
    {
      msg.request_id = _add_hex_string_to_work_order_id(itinerary.id());
    }
    else
    {
      msg.request_id = _add_hex_string_to_work_order_id(*work_order_id);
    }
    return msg;
  }

  // From rmf_ros2/rmf_task_ros2/src/rmf_task_ros2/Dispatcher.cpp
  std::string _generate_random_hex_string()
  {
    std::stringstream ss;
    // TODO(ac): parameterize length of hex string
    for (std::size_t i = 0; i < 5; ++i)
    {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> dis(0, 255);
      const auto random_char = dis(gen);
      std::stringstream hexstream;
      hexstream << std::hex << random_char;
      auto hex = hexstream.str();
      ss << (hex.length() < 2 ? '0' + hex : hex);
    }
    return ss.str();
  }

  std::string _add_hex_string_to_work_order_id(
    const std::string& work_order_id)
  {
    return work_order_id + "-" + _generate_random_hex_string();
  }

  std::optional<std::string> _get_work_order_id_from_rmf_task_id(
    const std::string& rmf_task_id)
  {
    // TODO(ac): parameterize length of separator + hex string
    int work_order_id_length = rmf_task_id.size() - 11;
    if (work_order_id_length < 1)
    {
      return std::nullopt;
    }

    return rmf_task_id.substr(0, work_order_id_length);
  }

  void _conclude_bid(
    const std::string& rmf_task_id,
    const std::optional<rmf_task_ros2::bidding::Response::Proposal> winner,
    const std::vector<std::string>& errors)
  {
    auto n = this->_node.lock();
    if (!n)
    {
      std::cerr << "RmfTransporter::_conclude_bid - invalid node" << std::endl;
      return;
    }

    if (!errors.empty())
    {
      RCLCPP_ERROR(
        n->get_logger(),
        "Errors occurred during auctioneer bidding:");
      for (const auto& e : errors)
      {
        RCLCPP_ERROR(n->get_logger(), e.c_str());
      }
    }

    auto it = _rmf_task_id_to_itinerary_query.find(rmf_task_id);
    if (it == _rmf_task_id_to_itinerary_query.end())
    {
      RCLCPP_ERROR(
        n->get_logger(),
        "Nexus RMF Transporter: unable to complete itinerary query for bid "
        "[%s], as itinerary query completion callback was not found. "
        "Skipping...",
        rmf_task_id.c_str());
      _auctioneer->ready_for_next_bid();
      return;
    }

    if (!winner.has_value())
    {
      RCLCPP_INFO(
        n->get_logger(),
        "Nexus RMF Transporter Bidding Result: no suitable transporter found "
        "for bid [%s].",
        rmf_task_id.c_str());
      it->second.completed_cb(std::nullopt);
      _auctioneer->ready_for_next_bid();
      return;
    }

    RCLCPP_INFO(
      n->get_logger(),
      "Nexus RMF Transporter: found suitable transporter for bid [%s], fleet "
      "[%s], robot [%s].",
      rmf_task_id.c_str(),
      winner->fleet_name.c_str(),
      winner->expected_robot_name.c_str());
    it->second.winner = winner;

    // TODO(ac): Use job_id instead of rmf_task_id, once we have made sure that
    // job_id is always unique. At the moment, job_id is just the work order ID
    // which could comprise of multiple transportation requests.
    const rclcpp::Time expiration_time = rmf_traffic_ros2::to_ros2(
      rmf_traffic::time::apply_offset(
        std::chrono::steady_clock::now(), _itinerary_expiration_seconds));

    auto itinerary = Itinerary(
      rmf_task_id,
      it->second.destinations,
      n->get_name(),
      rmf_traffic_ros2::to_ros2(winner->finish_time),
      expiration_time);
    itinerary
      .metadata("fleet_name", winner->fleet_name)
      .metadata("robot_name", winner->expected_robot_name);
    it->second.completed_cb(itinerary);

    // This itinerary may have been generated via get_itinerary, and contains
    // the designated fleet or robot names.
    _job_id_to_itinerary.insert({itinerary.id(), itinerary});

    // Since we will be using direct dispatching, we don't need to keep this
    // query anymore
    _rmf_task_id_to_itinerary_query.erase(it);

    _auctioneer->ready_for_next_bid();
  }

  ApiRequest _generate_rmf_cancellation_api_request(
    const std::string& rmf_task_id,
    const std::string& request_id)
  {
    nlohmann::json cancel_json;
    cancel_json["type"] = "cancel_task_request";
    cancel_json["task_id"] = rmf_task_id;

    return rmf_task_msgs::build<ApiRequest>()
      .json_msg(cancel_json.dump())
      .request_id(request_id);
  }

  void _cancel_rmf_task(
    const std::string& rmf_task_id,
    const std::string& request_id)
  {
    _api_request_pub->publish(
      _generate_rmf_cancellation_api_request(rmf_task_id, request_id));
    _cancellation_request_id_to_rmf_task_id.insert({request_id, rmf_task_id});
    _rmf_task_id_to_ongoing_itinerary.erase(rmf_task_id);
  }

public:

  bool configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node) final
  {
    auto n = node.lock();
    if (!n)
    {
      std::cerr << "RmfTransporter::configure - invalid node" << std::endl;
      return false;
    }
    _node = n;

    // Set the bidding time window duration
    _bidding_time_window_seconds = n->declare_parameter(
      "bidding_time_window_seconds",
      2);
    RCLCPP_INFO(
      n->get_logger(),
      "RmfTransporter bidding_time_window_seconds set to [%d].",
      _bidding_time_window_seconds
    );

    // Set the itinerary expiration duration
    _itinerary_expiration_seconds = n->declare_parameter(
      "itinerary_expiration_seconds",
      60);
    RCLCPP_INFO(
      n->get_logger(),
      "RmfTransporter itinerary_expiration_seconds set to [%d].",
      _itinerary_expiration_seconds
    );

    const bool mock_dispenser = n->declare_parameter(
      "mock_dispenser_interface",
      true);
    RCLCPP_INFO(
      n->get_logger(),
      "RmfTransporter mock_dispenser_interface set to [%s].",
      mock_dispenser ? "true" : "false"
    );

    // The internal node is used to run any interactions with RMF without
    // being blocked by the transporter node's actions and services
    _internal_node = rclcpp::Node::make_shared("rmf_transporter_internal_node");

    _spin_thread = std::make_shared<std::thread>([&]()
      {
        rclcpp::experimental::executors::EventsExecutor executor;
        executor.add_node(_internal_node);
        executor.spin();
      });

    _auctioneer = rmf_task_ros2::bidding::Auctioneer::make(
      _internal_node,
      [this](
        const std::string& rmf_task_id,
        const std::optional<rmf_task_ros2::bidding::Response::Proposal> winner,
        const std::vector<std::string>& errors)
      {
        this->_conclude_bid(rmf_task_id, std::move(winner), errors);
      },
      std::make_shared<rmf_task_ros2::bidding::QuickestFinishEvaluator>());

    const auto reliable_qos =
      rclcpp::SystemDefaultsQoS().keep_last(10).reliable();
    const auto transient_qos =
      rclcpp::SystemDefaultsQoS().transient_local().keep_last(10).reliable();

    if (mock_dispenser)
    {
      _mock_dispenser_result_pub = _internal_node->create_publisher<DispenserResult>(
        "/dispenser_results",
        reliable_qos);

      _mock_dispenser_request_sub = _internal_node->create_subscription<DispenserRequest>(
        "/dispenser_requests",
        reliable_qos,
        [&](DispenserRequest::SharedPtr msg)
        {
          auto n = _node.lock();
          if (!n)
          {
            std::cerr << "RmfTransporter::_mock_dispenser_request_sub - invalid node"

                      << std::endl;
            return;
          }

          // Check if the AMR is processing a cancelled work order.
          const auto work_order_id = _get_work_order_id_from_rmf_task_id(
            msg->request_guid);
          if (work_order_id.has_value() &&
            _cancelled_wo.count(work_order_id.value()) > 0)
          {
            RCLCPP_INFO(
              n->get_logger(),
              "Received DispenserRequest message from %s for request_guid %s "
              "and target_guid %s while the work order %s for this task has "
              "been cancelled. Cancelling RMF task...",
              msg->transporter_type.c_str(),
              msg->request_guid.c_str(),
              msg->target_guid.c_str(),
              work_order_id.value().c_str());

            DispenserResult dispenser_result_msg;
            dispenser_result_msg.request_guid = msg->request_guid;
            dispenser_result_msg.source_guid = msg->target_guid;
            dispenser_result_msg.status = DispenserResult::FAILED;
            _mock_dispenser_result_pub->publish(
              std::move(dispenser_result_msg));

            // Also cancel the RMF task.
            auto now = std::chrono::steady_clock::now().time_since_epoch();
            std::stringstream ss;
            ss << std::chrono::duration_cast<std::chrono::nanoseconds>
              (now).count();
            _cancel_rmf_task(msg->request_guid, ss.str());
            _cancelled_wo.erase(work_order_id.value());
            return;
          }

          auto it = _rmf_task_id_to_ongoing_itinerary.find(msg->request_guid);
          if (it == _rmf_task_id_to_ongoing_itinerary.end())
          {
            RCLCPP_WARN(
              n->get_logger(),
              "Dispenser request received for RMF task [%s] but it is not in an itinerary.",
              msg->request_guid.c_str());
            return;
          }
          if (it->second.itinerary.destinations().empty())
          {
            RCLCPP_WARN(
              n->get_logger(),
              "Itinerary with id [%s] has no destinations",
              it->second.itinerary.id().c_str());
            return;
          }
          RCLCPP_INFO(
            n->get_logger(),
            "Dispenser request received for RMF task [%s] publishing a mock successful result.",
            msg->request_guid.c_str());
          _pending_ingestor_task_ids.insert(msg->request_guid);

          DispenserResult res;
          res.status = DispenserResult::SUCCESS;
          res.request_guid = msg->request_guid;
          res.source_guid = n->get_name();
          _mock_dispenser_result_pub->publish(res);
        });
    }

    _api_request_pub = _internal_node->create_publisher<ApiRequest>(
      "/task_api_requests",
      transient_qos);

    _ingestor_result_pub = _internal_node->create_publisher<IngestorResult>(
      "/ingestor_results",
      reliable_qos);

    _api_response_sub = _internal_node->create_subscription<ApiResponse>(
      "/task_api_responses",
      transient_qos,
      [&](ApiResponse::UniquePtr msg)
      {
        auto n = _node.lock();
        if (!n)
        {
          std::cerr << "RmfTransporter::_api_response_sub - invalid node"
                    << std::endl;
          return;
        }

        if (msg->type != msg->TYPE_RESPONDING)
        {
          // Ignore non-responding messages
          return;
        }

        // Check for task cancellation first
        auto cancellation_it =
        _cancellation_request_id_to_rmf_task_id.find(msg->request_id);
        if (cancellation_it != _cancellation_request_id_to_rmf_task_id.end())
        {
          auto c = nlohmann::json::parse(msg->json_msg, nullptr, false);
          if (c.is_discarded() || !c.contains("success"))
          {
            RCLCPP_ERROR(
              n->get_logger(),
              "Invalid JSON in cancellation API response, RMF cancellation "
              "[%s] for RMF task [%s] failed",
              msg->request_id.c_str(),
              cancellation_it->second.c_str());
            // Note(ac): assume that some form of manual intervention or manual
            // cancellation is required if cancellation from API fails, and
            // there is nothing Nexus can do about it.
            return;
          }

          if (c["success"] == false)
          {
            // Note(ac): Same assumption as the note above regarding manual
            // cancellation or intervention.
            RCLCPP_ERROR(
              n->get_logger(),
              "RMF cancellation [%s] for RMF task [%s] failed",
              msg->request_id.c_str(),
              cancellation_it->second.c_str());
            return;
          }

          // Cancellation was successful
          _cancellation_request_id_to_rmf_task_id.erase(cancellation_it);
          return;
        }

        // Warning about pending or failed cancellations
        if (!_cancellation_request_id_to_rmf_task_id.empty())
        {
          std::stringstream ss;
          for (auto it = _cancellation_request_id_to_rmf_task_id.begin();
          it != _cancellation_request_id_to_rmf_task_id.end(); it++)
          {
            ss << "RMF task: [" << it->second << "], cancellation request: ["
              << it->first << "]," << std::endl;
          }
          RCLCPP_WARN(
            n->get_logger(),
            "Pending or failed transporter cancellations: \n%s\n",
            ss.str().c_str());
        }

        auto it =
        _itinerary_id_to_unconfirmed_itineraries.find(msg->request_id);
        if (it == _itinerary_id_to_unconfirmed_itineraries.end())
        {
          // Ignore API responses that are not for this transporter
          return;
        }

        auto j = nlohmann::json::parse(msg->json_msg, nullptr, false);
        // TODO(ac): use schema validation instead
        if (j.is_discarded() ||
        !j.contains("success") ||
        !j.contains("state") ||
        !j["state"].contains("booking") ||
        !j["state"]["booking"].contains("id"))
        {
          RCLCPP_ERROR(
            n->get_logger(),
            "Invalid JSON in API response, itinerary [%s] failed.",
            it->second.itinerary.id().c_str());
          it->second.completed_cb(false);
          _itinerary_id_to_unconfirmed_itineraries.erase(it);
          return;
        }

        if (j["success"] == false)
        {
          RCLCPP_ERROR(
            n->get_logger(),
            "RMF task dispatch request for itinerary [%s] rejected.",
            it->second.itinerary.id().c_str());
          it->second.completed_cb(false);
          _itinerary_id_to_unconfirmed_itineraries.erase(it);
          return;
        }

        RCLCPP_DEBUG(
          n->get_logger(),
          "RMF task dispatch request for itinerary [%s] accepted,\n%s",
          it->second.itinerary.id().c_str(),
          j.dump(4).c_str());

        std::string rmf_task_id = j["state"]["booking"]["id"];
        it->second.transporter_state.task_id = rmf_task_id;
        _rmf_task_id_to_ongoing_itinerary.insert(
          {std::move(rmf_task_id), std::move(it->second)});
        _itinerary_id_to_unconfirmed_itineraries.erase(it);
      });

    _task_state_sub = _internal_node->create_subscription<TaskStateUpdate>(
      "/task_state_update",
      transient_qos,
      [&](TaskStateUpdate::UniquePtr msg)
      {
        auto n = _node.lock();
        if (!n)
        {
          std::cerr << "RmfTransporter::_task_state_sub - invalid node"
                    << std::endl;
          return;
        }

        auto j = nlohmann::json::parse(msg->data, nullptr, false);
        // TODO(ac): use schema validation instead
        if (j.is_discarded() ||
        !j.contains("data") ||
        !j["data"].contains("status") ||
        !j["data"].contains("booking") ||
        !j["data"]["booking"].contains("id"))
        {
          RCLCPP_ERROR(
            n->get_logger(),
            "Ignoring invalid JSON in task state update\n%s",
            msg->data.c_str());
          return;
        }

        const std::string rmf_task_id = j["data"]["booking"]["id"];

        auto it = _rmf_task_id_to_ongoing_itinerary.find(rmf_task_id);
        if (it == _rmf_task_id_to_ongoing_itinerary.end())
        {
          // Ignore task state updates that are not for this transporter
          return;
        }

        const std::string status = j["data"]["status"];
        if (status == "failed" || status == "canceled" || status == "killed")
        {
          RCLCPP_ERROR(
            n->get_logger(),
            "RMF task [%s] has status [%s], transporter itinerary [%s] failed.",
            rmf_task_id.c_str(),
            status.c_str(),
            it->second.itinerary.id().c_str());
          it->second.completed_cb(false);
          _rmf_task_id_to_ongoing_itinerary.erase(it);
          return;
        }
        else if (status == "completed")
        {
          RCLCPP_INFO(
            n->get_logger(),
            "RMF task [%s] completed, transporter itinerary [%s] completed.",
            rmf_task_id.c_str(),
            it->second.itinerary.id().c_str());
          // it->second.completed_cb(true);
          // _rmf_task_id_to_ongoing_itinerary.erase(it);
          return;
        }
        else
        {
          RCLCPP_DEBUG(
            n->get_logger(),
            "RMF task [%s] has status [%s].",
            rmf_task_id.c_str(),
            status.c_str());
          // TODO(ac): modify transporter state
          it->second.feedback_cb(it->second.transporter_state);
        }
      });

    _ingestor_request_sub =
      _internal_node->create_subscription<IngestorRequest>(
      "/ingestor_requests",
      reliable_qos,
      [&](IngestorRequest::SharedPtr msg)
      {
        auto n = _node.lock();
        if (!n)
        {
          std::cerr << "RmfTransporter::_ingestor_request_sub - invalid node"

                    << std::endl;
          return;
        }

        // Check if the AMR is processing a cancelled work order.
        const auto work_order_id = _get_work_order_id_from_rmf_task_id(
          msg->request_guid);
        if (work_order_id.has_value() &&
          _cancelled_wo.count(work_order_id.value()) > 0)
        {
          RCLCPP_INFO(
            n->get_logger(),
            "Received IngestorRequest message from %s for request_guid %s and "
            "target_guid %s while the work order %s for this task has been "
            "cancelled. Cancelling RMF task...",
            msg->transporter_type.c_str(),
            msg->request_guid.c_str(),
            msg->target_guid.c_str(),
            work_order_id.value().c_str());

          IngestorResult ingestor_result_msg;
          ingestor_result_msg.request_guid = msg->request_guid;
          ingestor_result_msg.source_guid = msg->target_guid;
          ingestor_result_msg.status = IngestorResult::FAILED;
          _ingestor_result_pub->publish(std::move(ingestor_result_msg));

          // Also cancel the RMF task.
          auto now = std::chrono::steady_clock::now().time_since_epoch();
          std::stringstream ss;
          ss << std::chrono::duration_cast<std::chrono::nanoseconds>
            (now).count();
          _cancel_rmf_task(msg->request_guid, ss.str());
          _cancelled_wo.erase(work_order_id.value());
          return;
        }

        auto it = _rmf_task_id_to_ongoing_itinerary.find(msg->request_guid);
        if (it == _rmf_task_id_to_ongoing_itinerary.end())
        {
          RCLCPP_WARN(
            n->get_logger(),
            "Ingestor request received for RMF task [%s] but it is not in an itinerary.",
            msg->request_guid.c_str());
          return;
        }
        if (it->second.itinerary.destinations().empty())
        {
          RCLCPP_WARN(
            n->get_logger(),
            "Itinerary with id [%s] has no destinations",
            it->second.itinerary.id().c_str());
          return;
        }
        const auto& last_destination =
        it->second.itinerary.destinations().back();
        if (last_destination.action != Destination::ACTION_DROPOFF)
        {
          RCLCPP_WARN(
            n->get_logger(),
            "Itinerary with id [%s] does not end in a dropoff but a dropoff was requested, it ends with [%d] instead.",
            it->second.itinerary.id().c_str(),
            (int)last_destination.action);
          return;
        }
        RCLCPP_INFO(
          n->get_logger(),
          "Ingestor request received for RMF task [%s] which is the last step in the itinerary, marking task as completed.",
          msg->request_guid.c_str());
        // We are at the last step and it is marked as a pickup, this means we arrived
        it->second.completed_cb(true);
        _rmf_task_id_to_ongoing_itinerary.erase(it);
        _pending_ingestor_task_ids.insert(msg->request_guid);
      });

    _building_map_sub = _internal_node->create_subscription<BuildingMap>(
      "/map",
      transient_qos,
      [&](BuildingMap::SharedPtr msg)
      {
        _building_map = msg;
        for (const auto& level : _building_map->levels)
        {
          for (const auto& graph : level.nav_graphs)
          {
            for (const auto& v : graph.vertices)
            {
              if (!v.name.empty())
              {
                _waypoints.insert(v.name);
              }
            }
          }
        }
      });

    _wo_state_sub = _internal_node->create_subscription<WorkOrderState>(
      "/work_order_states",
      reliable_qos,
      [&](WorkOrderState::ConstSharedPtr msg)
      {
        auto n = _node.lock();
        if (!n)
        {
          std::cerr << "RmfTransporter::_wo_state_sub - invalid node"
                    << std::endl;
          return;
        }

        if (msg->id.empty())
        {
          return;
        }
        if (msg->state == WorkOrderState::STATE_CANCELLED ||
          msg->state == WorkOrderState::STATE_FAILED)
        {
          _cancelled_wo[msg->id] = msg;
          RCLCPP_INFO(
            n->get_logger(),
            "Adding work_order with id %s to cancelled list.",
            msg->id.c_str()
          );
        }
      }
    );

    _ready = true;
    RCLCPP_INFO(
      n->get_logger(),
      "Finished configuring RmfTransporter!"
    );
    return true;
  }

  bool ready() const final
  {
    return _ready;
  }

  void get_itinerary(
    const std::string& job_id,
    const std::vector<Destination>& destinations,
    Transporter::ItineraryQueryCompleted completed_cb) final
  {
    auto n = _node.lock();
    if (!n)
    {
      std::cerr << "RmfTransporter::get_itinerary - invalid node" << std::endl;
      completed_cb(std::nullopt);
      return;
    }

    std::stringstream ss;
    for (const auto& d : destinations)
    {
      ss << d.name << ",";
    }
    RCLCPP_INFO(
      n->get_logger(),
      "Received itinerary request with id [%s] for destinations [%s]",
      job_id.c_str(),
      ss.str().c_str()
    );

    if (!_building_map)
    {
      RCLCPP_ERROR(
        n->get_logger(),
        "Building map not yet received");
      completed_cb(std::nullopt);
      return;
    }

    // Checks through building map first
    for (const auto& d : destinations)
    {
      if (_waypoints.find(d.name) == _waypoints.end())
      {
        RCLCPP_ERROR(
          n->get_logger(),
          "Destination name [%s] not available.",
          d.name.c_str());
        completed_cb(std::nullopt);
        return;
      }
    }

    const auto request = _generate_dispatch_task_request_json(destinations);
    if (!request.has_value())
    {
      RCLCPP_ERROR(
        n->get_logger(),
        "Failed to generate dispatch task request json");
      completed_cb(std::nullopt);
      return;
    }

    RCLCPP_DEBUG(n->get_logger(), "%s", request.value().dump(4).c_str());
    // TODO(ac): perform schema validation.

    const auto rmf_task_id = _add_hex_string_to_work_order_id(job_id);

    const auto bid_notice =
      rmf_task_msgs::build<rmf_task_msgs::msg::BidNotice>()
      .request(request.value().dump())
      .task_id(rmf_task_id)
      .time_window(
        rmf_traffic_ros2::convert(rmf_traffic::time::from_seconds(2.0)))
      .dry_run(true);
    _auctioneer->request_bid(bid_notice);

    _rmf_task_id_to_itinerary_query[rmf_task_id] = ItineraryQuery{
      job_id, destinations, std::move(completed_cb), std::nullopt};
  }

  void transport_to_destination(
    Itinerary itinerary,
    Transporter::TransportFeedback feedback_cb,
    Transporter::TransportCompleted completed_cb) final
  {
    auto n = _node.lock();
    if (!n)
    {
      std::cerr << "RmfTransporter::transport_to_destination - invalid node"
                << std::endl;
      completed_cb(false);
      return;
    }

    RCLCPP_INFO(
      n->get_logger(),
      "RmfTransporter::transport_to_destination, got a request for an itinerary!"
    );

    Itinerary itinerary_to_use = itinerary;
    const auto saved_it = _job_id_to_itinerary.find(itinerary.id());
    if (saved_it != _job_id_to_itinerary.end())
    {
      // TODO(ac): check that the destinations match
      itinerary_to_use = saved_it->second;
      _job_id_to_itinerary.erase(saved_it);
    }

    const auto api_request_msg =
      _generate_dispatch_api_message(itinerary_to_use);

    if (!api_request_msg.has_value())
    {
      completed_cb(false);
      return;
    }
    _api_request_pub->publish(api_request_msg.value());

    nexus_transporter_msgs::msg::TransporterState transporter_state;
    transporter_state.transporter = itinerary.transporter_name();
    transporter_state.state =
      nexus_transporter_msgs::msg::TransporterState::STATE_UNAVAILABLE;
    transporter_state.estimated_finish_time =
      itinerary.estimated_finish_time() - n->get_clock()->now();

    _itinerary_id_to_unconfirmed_itineraries.insert(
      {
        api_request_msg.value().request_id,
        {
          std::move(itinerary),
          std::move(transporter_state),
          std::move(feedback_cb),
          std::move(completed_cb)
        }
      });
  }

  bool cancel(Itinerary itinerary) final
  {
    auto n = _node.lock();
    if (!n)
    {
      std::cerr << "RmfTransporter::cancel - invalid node" << std::endl;
      return false;
    }

    auto it = _rmf_task_id_to_ongoing_itinerary.begin();
    for (; it != _rmf_task_id_to_ongoing_itinerary.end(); it++)
    {
      if (it->second.itinerary.id() != itinerary.id())
      {
        continue;
      }

      std::stringstream ss;
      ss << "cancellation.nexus-" << itinerary.id() << "-" << it->first;
      _cancel_rmf_task(itinerary.id(), ss.str());
      return true;
    }

    RCLCPP_ERROR(
      n->get_logger(),
      "No ongoing RMF task for itinerary [%s] found",
      itinerary.id().c_str());
    return false;
  }

  bool handle_signal(std::string task_id, std::string signal) final
  {
    auto n = _node.lock();
    if (!n)
    {
      std::cerr << "RmfTransporter::cancel - invalid node" << std::endl;
      return false;
    }

    auto it = _pending_ingestor_task_ids.find(task_id);
    if (it == _pending_ingestor_task_ids.end())
    {
      RCLCPP_WARN(
        n->get_logger(),
        "Received signal [%s] for task [%s] that is not waiting for it, "
        "ignoring",
        signal.c_str(), task_id.c_str());
      return false;
    }

    if (signal != "dropoff")
    {
      RCLCPP_WARN(
        n->get_logger(),
        "Received unsupported signal [%s] for task [%s]",
        signal.c_str(), task_id.c_str());
      return false;
    }


    // Publish the ingestor result
    IngestorResult res;
    res.status = IngestorResult::SUCCESS;
    res.request_guid = task_id;
    res.source_guid = n->get_name();
    _ingestor_result_pub->publish(res);
    _pending_ingestor_task_ids.erase(it);
    return true;
  }

  ~RmfTransporter()
  {
    if (_spin_thread && _spin_thread->joinable())
    {
      _spin_thread->join();
    }
  };
};

}  // namespace nexus_transporter

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus_transporter::RmfTransporter, nexus_transporter::Transporter)
