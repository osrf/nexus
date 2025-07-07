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

#include "rmf_request.hpp"

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

namespace nexus::capabilities {

namespace rmf {

BT::NodeStatus DispatchRequest::onStart()
{
  const auto destinations = this->getInput<std::deque<Destination>>("destinations");
  if (!destinations)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [destinations] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto transient_qos =
    rclcpp::SystemDefaultsQoS().transient_local().keep_last(10).reliable();
  this->_api_request_pub = this->_node->create_publisher<ApiRequest>("/task_api_requests", transient_qos);
  // TODO(luca) publish request here with unique UUID
  this->_api_response_sub = this->_node->create_subscription<ApiResponse>("/task_api_responses", transient_qos,
      [&](ApiResponse::UniquePtr msg)
      {
        this->api_response_cb(*msg);
      });
  this->submit_itinerary(*destinations);
  return BT::NodeStatus::RUNNING;
}

void DispatchRequest::submit_itinerary(const std::deque<Destination>& destinations)
{
  nlohmann::json j;
  j["type"] = "dispatch_task_request";
  nlohmann::json r;
  r["unix_millis_request_time"] = 0;
  r["unix_millis_earliest_start_time"] = 0;
  r["requester"] = this->_node->get_name();
  r["category"] = "compose";
  nlohmann::json d;
  d["category"] = "multi_delivery";
  d["phases"] = nlohmann::json::array();
  nlohmann::json activity;
  activity["category"] = "sequence";
  activity["description"]["activities"] = nlohmann::json::array();
  for (const auto& destination : destinations)
  {
    nlohmann::json a;
    a["category"] = destination.action;
    nlohmann::json p;
    // TODO(luca) exception safety for wrong types? Checking for pickup only since we don't do ingestors yet?
    p["place"] = destination.workcell;
    // TODO(luca) We should assign a handler that is related to the workcell.
    // For now the assumption is that a location has only one handler
    p["handler"] = destination.workcell;
    p["payload"] = nlohmann::json::array();
    a["description"] = p;
    activity["description"]["activities"].push_back(a);
  }
  nlohmann::json act_obj;
  act_obj["activity"] = activity;
  d["phases"].push_back(act_obj);
  r["description"] = d;
  j["request"] = r;
  ApiRequest msg;
  msg.json_msg = j.dump();
  // Time since epoch as a unique id
  auto now = std::chrono::steady_clock::now().time_since_epoch();
  msg.request_id = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  this->requested_id = msg.request_id;
  this->_api_request_pub->publish(msg);
}

void DispatchRequest::api_response_cb(const ApiResponse& msg)
{
  // Receive response, populate hashmaps
  if (msg.type != msg.TYPE_RESPONDING)
  {
    return;
  }
  if (msg.request_id != this->requested_id)
  {
    return;
  }
  auto j = nlohmann::json::parse(msg.json_msg, nullptr, false);
  if (j.is_discarded())
  {
    RCLCPP_ERROR(this->_node->get_logger(), "Invalid json in api response");
    return;
  }
  // TODO(luca) exception safety for missing fields, return FAILURE if
  // submission failed
  if (j["success"] == false)
  {
    RCLCPP_ERROR(this->_node->get_logger(), "Task submission failed");
    return;
  }
  // Task cancellations don't have a state field
  if (!j.contains("state"))
    return;
  this->rmf_task_id = j["state"]["booking"]["id"];
}

BT::NodeStatus DispatchRequest::onRunning()
{
  if (rmf_task_id.has_value())
  {
    this->setOutput("rmf_task_id", *rmf_task_id);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SignalAmr::tick()
{
  const auto workcell = this->getInput<std::string>("workcell");
  if (!workcell)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [workcell] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const auto rmf_task_id = this->getInput<std::string>("rmf_task_id");
  if (!rmf_task_id)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [rmf_task_id] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const auto action_type = this->getInput<std::string>("action_type");
  if (!action_type)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [action_type] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (action_type == "pickup")
  {
    this->_dispenser_result_pub = this->_node->create_publisher<DispenserResult>(
        "/dispenser_results",
        rclcpp::SystemDefaultsQoS().keep_last(10).reliable());
    DispenserResult msg;
    msg.request_guid = *rmf_task_id;
    msg.source_guid = *workcell;
    msg.status = DispenserResult::SUCCESS;
    this->_dispenser_result_pub->publish(msg);
  }
  else if (action_type == "dropoff")
  {
    this->_ingestor_result_pub = this->_node->create_publisher<IngestorResult>(
        "/ingestor_results",
        rclcpp::SystemDefaultsQoS().keep_last(10).reliable());
    IngestorResult msg;
    msg.request_guid = *rmf_task_id;
    msg.source_guid = *workcell;
    msg.status = DispenserResult::SUCCESS;
    this->_ingestor_result_pub->publish(msg);
  }
  else
  {
    RCLCPP_ERROR(
      this->_node->get_logger(),
      "%s: [action_type] port only accepts \"pickup\" or \"dropoff\"",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SendSignal::tick()
{
  const auto ctx = this->_ctx_mgr->current_context();
  auto signal = this->getInput<std::string>("signal");
  if (!signal)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: port [signal] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto workcell = this->getInput<std::string>("workcell");
  if (!workcell)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: port [workcell] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto workcell_task_id = this->getInput<std::string>("workcell_task_id");
  if (!workcell_task_id)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: port [workcell_task_id] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  this->_signal_client = std::make_unique<common::SyncServiceClient<endpoints::
      SignalWorkcellService::ServiceType>>(
        this->_node, endpoints::SignalWorkcellService::service_name(*workcell));

  auto req =
    std::make_shared<endpoints::SignalWorkcellService::ServiceType::Request>();
  req->task_id = *workcell_task_id;
  req->signal = *signal;
  auto resp = this->_signal_client->send_request(req);
  if (!resp->success)
  {
    // TODO(luca) implement a queueing mechanism if the request fails?
    RCLCPP_WARN(
      this->_node->get_logger(),
      "%s: Workcell [%s] is not able to accept [%s] signal now. Skipping signaling, error: [%s]",
      this->_node->get_name(), workcell->c_str(), signal->c_str(), resp->message.c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

}
}
