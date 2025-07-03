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

#include <nexus_transporter/Itinerary.hpp>
#include <nexus_transporter/Transporter.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rmf_building_map_msgs/msg/building_map.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>
#include <std_msgs/msg/string.hpp>
#include <rmf_task_ros2/bidding/Auctioneer.hpp>
#include <rmf_traffic_ros2/Time.hpp>

#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace nexus_transporter {

using ApiRequest = rmf_task_msgs::msg::ApiRequest;
using ApiResponse = rmf_task_msgs::msg::ApiResponse;
using TaskStateUpdate = std_msgs::msg::String;
using BuildingMap = rmf_building_map_msgs::msg::BuildingMap;

class RmfTransporter : public Transporter
{
public:

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

  bool _ready = false;

  // TODO(ac): parameterize these time windows
  builtin_interfaces::msg::Duration _bidding_time_window =
    rmf_traffic_ros2::convert(rmf_traffic::time::from_seconds(2.0));
  double _itinerary_expiration_seconds = 60;

  std::mutex _mutex;

  // Used for transportation requests
  std::unordered_map<std::string, OngoingItinerary>
  _itinerary_id_to_unconfirmed_itineraries = {};
  std::unordered_map<std::string, OngoingItinerary>
  _rmf_task_id_to_ongoing_itinerary = {};

  // Used for cancellation only
  std::unordered_map<std::string, std::string>
  _cancellation_rmf_id_to_itinerary_id = {};

  // RMF interface
  BuildingMap::SharedPtr _building_map = nullptr;
  std::unordered_set<std::string> _waypoints = {};
  rclcpp::Subscription<rmf_building_map_msgs::msg::BuildingMap>::SharedPtr
    _building_map_sub = nullptr;

  // Task interface
  rclcpp::Publisher<ApiRequest>::SharedPtr _api_request_pub = nullptr;
  rclcpp::Subscription<ApiResponse>::SharedPtr _api_response_sub = nullptr;
  rclcpp::Subscription<TaskStateUpdate>::SharedPtr _task_state_sub = nullptr;

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

    nlohmann::json j;
    j["type"] = "dispatch_task_request";
    j["request"] = request_json.value();

    ApiRequest msg;
    msg.json_msg = j.dump();
    std::stringstream ss;
    ss << "compose.nexus-delivery" << itinerary.id() << "-" << _generate_random_hex_string(5);
    msg.request_id = ss.str();
    return msg;
  }

  // From rmf_ros2/rmf_task_ros2/src/rmf_task_ros2/Dispatcher.cpp
  std::string _generate_random_hex_string(const std::size_t length = 3)
  {
    std::stringstream ss;
    for (std::size_t i = 0; i < length; ++i)
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

  std::string _generate_rmf_bidding_task_id(const std::string& job_id)
  {
    std::stringstream ss;
    ss << "compose.nexus-bidding-" << job_id << "-"
      << _generate_random_hex_string(5);
    return ss.str();
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
    // TODO(luca) get RMF parameters here

    const auto transient_qos =
      rclcpp::SystemDefaultsQoS().transient_local().keep_last(10).reliable();

    _api_request_pub = n->create_publisher<ApiRequest>(
      "/task_api_requests",
      transient_qos);

    _api_response_sub = n->create_subscription<ApiResponse>(
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
          _cancellation_rmf_id_to_itinerary_id.find(msg->request_id);
        if (cancellation_it != _cancellation_rmf_id_to_itinerary_id.end())
        {
          auto c = nlohmann::json::parse(msg->json_msg, nullptr, false);
          if (c.is_discarded() || !c.contains("success"))
          {
            RCLCPP_ERROR(
              n->get_logger(),
              "Invalid JSON in cancellation API response, RMF cancellation "
              "[%s] for itinerary [%s] failed",
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
              "RMF cancellation [%s] for itinerary [%s] failed",
              msg->request_id.c_str(),
              cancellation_it->second.c_str());
            return;
          }

          // Cancellation was successful
          _cancellation_rmf_id_to_itinerary_id.erase(cancellation_it);
          return;
        }

        // Warning about pending or failed cancellations
        if (!_cancellation_rmf_id_to_itinerary_id.empty())
        {
          std::stringstream ss;
          for (auto it = _cancellation_rmf_id_to_itinerary_id.begin();
            it != _cancellation_rmf_id_to_itinerary_id.end(); it++)
          {
            ss << "itinerary: [" << it->second << "], RMF: [" << it->first
              << "]," << std::endl;
          }
          RCLCPP_WARN(
            n->get_logger(),
            "Pending or failed transporter cancellations: \n%s\n",
            ss.str().c_str());
        }

        std::lock_guard<std::mutex> lock(_mutex);
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

        RCLCPP_INFO(
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

    _task_state_sub = n->create_subscription<TaskStateUpdate>(
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

        std::lock_guard<std::mutex> lock(_mutex);
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
          it->second.completed_cb(true);
          _rmf_task_id_to_ongoing_itinerary.erase(it);
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

    _building_map_sub = n->create_subscription<BuildingMap>(
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

    // Naively just checks if the waypoints exist on the building map.
    // TODO(ac): use auctioneer for proper bidding
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

    // TODO(ac): use estimated finishing time from auctioneer. For now we just
    // naively set the completion and expiration to 60 seconds from now.
    auto itinerary = Itinerary(
      job_id,
      destinations,
      n->get_name(),
      n->get_clock()->now() + rclcpp::Duration(std::chrono::seconds(60)),
      n->get_clock()->now() + rclcpp::Duration(std::chrono::seconds(60)));
    completed_cb(itinerary);
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

    const auto api_request_msg =
      _generate_dispatch_api_message(itinerary);

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

    std::lock_guard<std::mutex> lock(_mutex);
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

      nlohmann::json c;
      c["type"] = "cancel_task_request";
      c["task_id"] = it->first;

      std::stringstream ss;
      ss << "cancellation.nexus-" << itinerary.id() << "-" << it->first;

      _cancellation_rmf_id_to_itinerary_id.insert({ss.str(), itinerary.id()});

      ApiRequest msg;
      msg.json_msg = c.dump();
      msg.request_id = ss.str();
      _api_request_pub->publish(msg);
      return true;
    }

    RCLCPP_ERROR(
      n->get_logger(),
      "No ongoing RMF task for itinerary [%s] found",
      itinerary.id().c_str());
    return false;
  }

  ~RmfTransporter() = default;
};

}  // namespace nexus_transporter

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus_transporter::RmfTransporter, nexus_transporter::Transporter)
