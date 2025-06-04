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
  RCLCPP_INFO(
    this->_node->get_logger(),
    "[DispatchRequest::api_response_cb] Received task response with rmf_task_id %s.",
    rmf_task_id.value().c_str()
  );
}

BT::NodeStatus DispatchRequest::onRunning()
{
  if (this->rmf_task_id.has_value())
  {
    this->setOutput("rmf_task_id", *rmf_task_id);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void DispatchRequest::onHalted()
{
  // Cancel any on-going RMF task.
  if (!rmf_task_id.has_value())
  {
    return;
  }
  RCLCPP_ERROR(
    this->_node->get_logger(),
    "[DispatchRequest::onHalted] Cancelling rmf_task with task_id %s.",
    this->rmf_task_id.value().c_str()
  );
  nlohmann::json cancel_json;
  cancel_json["type"] = "cancel_task_request";
  cancel_json["task_id"] = this->rmf_task_id.value();

  ApiRequest msg;
  // Time since epoch as a unique id.
  auto now = std::chrono::steady_clock::now().time_since_epoch();
  msg.request_id = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  msg.json_msg = cancel_json.dump();
  this->_api_request_pub->publish(std::move(msg));
}

BT::NodeStatus ExtractDestinations::tick()
{
  const auto ctx = this->_ctx_mgr->current_context();
  const auto& task = ctx->task;
  std::deque<Destination> destinations;
  for (const auto& node : task.data)
  {
    if (node["type"] && node["destination"] && node["workcell_task_id"])
    {
      auto type = node["type"].as<std::string>();
      auto destination = node["destination"].as<std::string>();
      auto workcell_task_id = node["workcell_task_id"].as<std::string>();
      destinations.push_back(Destination {type, destination, workcell_task_id});
    }
    else
    {
      RCLCPP_ERROR(
        this->_node->get_logger(),
        "Order element did not contain \"type\" and \"destination\" fields");
      return BT::NodeStatus::FAILURE;
    }
  }
  this->setOutput("destinations", destinations);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UnpackDestinationData::tick()
{
  const auto destination = this->getInput<Destination>("destination");
  if (!destination)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [destination] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  this->setOutput("workcell", destination->workcell);
  this->setOutput("type", destination->action);
  this->setOutput("workcell_task_id", destination->workcell_task_id);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SignalAmr::onStart()
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

  this->_workcell = *workcell;
  this->_rmf_task_id = *rmf_task_id;
  this->_dispenser_result_pub = this->_node->create_publisher<DispenserResult>("/dispenser_results", 10);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SignalAmr::onRunning()
{
  DispenserResult msg;
  msg.request_guid = _rmf_task_id;
  msg.source_guid = _workcell;
  msg.status = DispenserResult::SUCCESS;
  this->_dispenser_result_pub->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

void SignalAmr::onHalted()
{
  DispenserResult msg;
  msg.request_guid = _rmf_task_id;
  msg.source_guid = _workcell;
  msg.status = DispenserResult::FAILED;
  this->_dispenser_result_pub->publish(msg);
}

BT::NodeStatus LoopDestinations::tick()
{
  if (!this->_initialized)
  {
    auto queue = this->getInput<std::deque<Destination>>("queue");
    if (!queue)
    {
      RCLCPP_ERROR(
        this->_node->get_logger(), "%s: [queue] port is required",
        this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    this->_queue = *queue;
    this->_initialized = true;
  }

  if (this->_queue.size() == 0)
  {
    return BT::NodeStatus::SUCCESS;
  }

  this->setStatus(BT::NodeStatus::RUNNING);

  while (this->_queue.size() > 0)
  {
    auto current_value = this->_queue.front();
    this->setOutput("value", current_value);

    this->setStatus(BT::NodeStatus::RUNNING);
    auto child_state = this->child_node_->executeTick();
    switch (child_state)
    {
      case BT::NodeStatus::SUCCESS:
        this->haltChild();
        this->_queue.pop_front();
        break;
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;
      case BT::NodeStatus::FAILURE:
        this->haltChild();
        return BT::NodeStatus::FAILURE;
      case BT::NodeStatus::IDLE:
        throw BT::LogicError("A child node must never return IDLE");
    }
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus WaitForAmr::onStart()
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

  this->_workcell = *workcell;
  this->_rmf_task_id = *rmf_task_id;
  this->_amr_ready = false;

  this->_dispenser_request_sub = this->_node->create_subscription<DispenserRequest>("/dispenser_requests", 10,
      [&](DispenserRequest::UniquePtr msg)
      {
        this->dispenser_request_cb(*msg);
      });
  return BT::NodeStatus::RUNNING;
}

void WaitForAmr::dispenser_request_cb(const DispenserRequest& msg)
{
  if (msg.request_guid == this->_rmf_task_id &&
      msg.target_guid == this->_workcell)
  {
    this->_amr_ready = true;
  }
}

BT::NodeStatus WaitForAmr::onRunning()
{
  if (this->_amr_ready)
  {
    this->_amr_ready = false;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
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
