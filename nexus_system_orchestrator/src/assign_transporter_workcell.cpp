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

#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>

#include <yaml-cpp/yaml.h>

#include "assign_transporter_workcell.hpp"

namespace nexus::system_orchestrator {

BT::PortsList AssignTransporterWorkcell::providedPorts()
{
  return {
    BT::OutputPort<std::string>("transporter_id"),
    BT::OutputPort<WorkcellTask>("transport_task")
  };
}

BT::NodeStatus AssignTransporterWorkcell::onStart()
{
  auto node = this->_w_node.lock();
  if (!node)
  {
    std::cerr <<
      "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES" <<
      std::endl;
    std::terminate();
  }

  // Create a list of destination from the current context tasks
  YAML::Node orders;
  std::vector<std::string> locations;
  std::optional<std::string> prev_output_station = std::nullopt;

  for (const auto& task : this->_ctx->get_tasks())
  {
    const auto assigned_workcell_id =
      this->_ctx->get_workcell_task_assignment(task.task_id);
    if (!assigned_workcell_id.has_value())
    {
      RCLCPP_ERROR(
        node->get_logger(), "%s: Unable to transport, task [%s] was not assigned to a workcell",
        this->name().c_str(), task.task_id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    const auto step =
      YAML::Load(task.payload).as<nexus::common::WorkOrder::Step>();
    // get input and outputs
    // check if previous output matches next input
    // throw error if it doesn't match
    // get the name of the station

    // Multipickup task
    // TODO(luca) Consider encoding where is a pickup and where a dropoff
    // TODO(luca) remove consecutive duplicates (multiple tasks to same workcell that don't need transportation)
    YAML::Node order;
    order["type"] = "pickup";
    order["destination"] = *assigned_workcell_id;
    order["workcell_task_id"] = task.task_id;
    orders.push_back(order);
  }
  this->_transport_task.work_order_id = this->_ctx->get_job_id();
  this->_transport_task.task_id = this->_ctx->get_job_id();
  this->_transport_task.type = "transportation";
  YAML::Emitter out;
  out << orders;
  this->_transport_task.payload = out.c_str();

  // Probe workcells for transportation capability to this set of destinations
  auto req =
    std::make_shared<endpoints::IsTaskDoableService::ServiceType::Request>();
  req->task = this->_transport_task;
  // send request to all transporters in parallel
  for (auto& [workcell_id, session] : this->_ctx->get_workcell_sessions())
  {
    auto fut = session->task_doable_client->async_send_request(req);
    this->_ongoing_requests.emplace(workcell_id,
      OngoingRequest{session->task_doable_client, std::move(fut)});
  }
  this->_time_limit = std::chrono::steady_clock::now() +
    std::chrono::seconds{5};

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AssignTransporterWorkcell::onRunning()
{
  auto result = this->_update_ongoing_requests();
  if (result != BT::NodeStatus::RUNNING)
  {
    // action is done, drop requests that have not received response to avoid callback leak.
    this->_cleanup_pending_requests();
  }
  return result;
}

void AssignTransporterWorkcell::onHalted()
{
  // can't cancel a service call, so we just drop the requests.
  this->_cleanup_pending_requests();
}

void AssignTransporterWorkcell::_cleanup_pending_requests()
{
  for (auto& [_, ongoing_req] : this->_ongoing_requests)
  {
    ongoing_req.client->remove_pending_request(ongoing_req.fut);
  }
}

BT::NodeStatus AssignTransporterWorkcell::_update_ongoing_requests()
{
  auto node = this->_w_node.lock();
  if (!node)
  {
    std::cerr <<
      "FATAL ERROR!!! NODE IS DESTROYED WHILE THERE ARE STILL REFERENCES" <<
      std::endl;
    std::terminate();
  }

  if (std::chrono::steady_clock::now() > this->_time_limit)
  {
    for (const auto& [workcell_id, _] : this->_ongoing_requests)
    {
      RCLCPP_WARN(
        node->get_logger(), "%s: Skipped transporter [%s] (no response)",
        this->name().c_str(),
        workcell_id.c_str());
    }
    RCLCPP_ERROR(
      node->get_logger(), "%s: No transporter is able to perform task",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::vector<std::string> finished_requests;
  for (auto& [workcell_id, ongoing_req] : this->_ongoing_requests)
  {
    if (ongoing_req.fut.wait_for(std::chrono::seconds{0}) ==
      std::future_status::ready)
    {
      finished_requests.emplace_back(workcell_id);
      auto resp = ongoing_req.fut.get();
      // TODO(luca) make this proper bidding
      // use first available transporter
      if (resp->success)
      {
        RCLCPP_INFO(
          node->get_logger(), "[%s]: Bid awarded to [%s]",
          this->name().c_str(),
          workcell_id.c_str());
        this->setOutput("transporter_id", workcell_id);
        this->setOutput("transport_task", this->_transport_task);
        // Update the context
        const auto& task_id = this->_transport_task.task_id;
        this->_ctx->set_workcell_task_assignment(task_id, workcell_id);
        auto task_state = nexus_orchestrator_msgs::msg::TaskState();
        task_state.workcell_id = workcell_id;
        task_state.task_id = task_id;
        this->_ctx->set_task_state(task_id, task_state);
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_DEBUG(
          node->get_logger(), "%s: Transporter [%s] cannot perform task",
          this->name().c_str(), workcell_id.c_str());
      }
    }
  }

  for (const auto& workcell_id : finished_requests)
  {
    this->_ongoing_requests.erase(workcell_id);
  }

  return BT::NodeStatus::RUNNING;
}

}
