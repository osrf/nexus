/*
 * Copyright (C) 2023 Johnson & Johnson
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

#include "send_signal.hpp"

#include <nexus_endpoints.hpp>

namespace nexus::system_orchestrator {

BT::NodeStatus SendSignal::tick()
{
  auto task =
    this->getInput<nexus_orchestrator_msgs::msg::WorkcellTask>("task");
  if (!task)
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(), "%s: port [task] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto signal = this->getInput<std::string>("signal");
  if (!signal)
  {
    RCLCPP_ERROR(
      this->_ctx->get_node().get_logger(), "%s: port [signal] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  try
  {
    const auto workcell_task_assignments = this->_ctx->get_workcell_task_assignments();
    auto it = std::find_if(
      workcell_task_assignments.cbegin(),
      workcell_task_assignments.cend(),
      [&task](const std::pair<std::string, std::string>& p)
      {
        return p.first == task->task_id;
      });
    if (it == workcell_task_assignments.cend())
    {
      RCLCPP_ERROR(
        this->_ctx->get_node().get_logger(), "%s: Unable to find workcell assigned to task [%s]",
        this->name().c_str(), task->task_id.c_str());
      return BT::NodeStatus::FAILURE;
    }
    const auto& workcell = it->second;

    const auto& session = this->_ctx->get_workcell_sessions().at(workcell);
    auto req =
      std::make_shared<endpoints::SignalWorkcellService::ServiceType::Request>();
    req->task_id = task->task_id;
    req->signal = *signal;
    auto resp = session->signal_wc_client->send_request(req);
    RCLCPP_INFO(
      this->_ctx->get_node().get_logger(), "%s: Sent signal [%s] to workcell [%s]",
      this->name().c_str(), signal->c_str(), workcell.c_str());
    if (!resp->success)
    {
      RCLCPP_WARN(
        this->_ctx->get_node().get_logger(),
        "%s: Workcell is not able to accept [%s] signal now. Queuing the signal to be sent on the next task request.",
        this->name().c_str(), signal->c_str());
      this->_ctx->add_queued_signal(task->task_id, *signal);
    }
  }
  catch (const std::out_of_range& e)
  {
    RCLCPP_ERROR(this->_ctx->get_node().get_logger(), "%s: %s",
      this->name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}
