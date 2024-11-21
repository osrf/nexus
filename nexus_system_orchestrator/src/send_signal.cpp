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
  auto task = this->getInput<WorkcellTask>("task");
  auto transporter = this->getInput<std::string>("transporter");
  if ((task && transporter) || (!task && !transporter))
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s only one of ports [task] and [transporter] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto signal = this->getInput<std::string>("signal");
  if (!signal)
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s: port [signal] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (task)
  {
    return signal_task(*task, *signal);
  }
  if (transporter)
  {
    return signal_transporter(*transporter, *signal);
  }
  // Should never reach here because we return early if none is available
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SendSignal::signal_task(const WorkcellTask& task, const std::string& signal)
{
  auto it = std::find_if(
    this->_ctx->workcell_task_assignments.cbegin(),
    this->_ctx->workcell_task_assignments.cend(),
    [&task](const std::pair<std::string, std::string>& p)
    {
      return p.first == task.id;
    });
  if (it == this->_ctx->workcell_task_assignments.cend())
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(),
      "%s: Unable to find workcell assigned to task [%s]",
      this->name().c_str(), task.id.c_str());
    return BT::NodeStatus::FAILURE;
  }
  const auto& workcell = it->second;

  const auto session_it = this->_ctx->workcell_sessions.find(workcell);
  if (session_it == this->_ctx->workcell_sessions.end())
  {
    RCLCPP_ERROR(this->_ctx->node.get_logger(), "%s: Session not found for workcell %s",
      this->name().c_str(), workcell);
    return BT::NodeStatus::FAILURE;
  }
  const auto& session = session_it->second;
  auto req =
    std::make_shared<endpoints::SignalWorkcellService::ServiceType::Request>();
  req->task_id = task.id;
  req->signal = signal;
  auto resp = session->signal_wc_client->send_request(req);
  RCLCPP_INFO(
    this->_ctx->node.get_logger(), "%s: Sent signal [%s] to workcell [%s]",
    this->name().c_str(), signal.c_str(), workcell.c_str());
  if (!resp->success)
  {
    RCLCPP_WARN(
      this->_ctx->node.get_logger(),
      "%s: Workcell is not able to accept [%s] signal now. Queuing the signal to be sent on the next task request.",
      this->name().c_str(), signal.c_str());
    this->_ctx->queued_signals[task.id].emplace_back(signal);
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SendSignal::signal_transporter(const std::string& transporter, const std::string& signal)
{
  auto it = this->_ctx->transporter_sessions.find(transporter);
  if (it == this->_ctx->transporter_sessions.cend())
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(),
      "%s: Unable to find transporter [%s]",
      this->name().c_str(), transporter.c_str());
    return BT::NodeStatus::FAILURE;
  }
  const auto& session = it->second;
  if (!session)
  {
    // TODO(luca) implement a queueing mechanism if the transporter is not available?
    RCLCPP_WARN(
      this->_ctx->node.get_logger(),
      "%s: No session found for transporter [%s], skipping signaling",
      this->name().c_str(), transporter.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  auto req =
    std::make_shared<endpoints::SignalTransporterService::ServiceType::Request>();
  // TODO(luca) Using job id for task id is not scalable, generate uuids?
  req->task_id = this->_ctx->job_id;
  req->signal = signal;
  auto resp = session->signal_client->send_request(req);
  RCLCPP_INFO(
    this->_ctx->node.get_logger(), "%s: Sent signal [%s] to transporter [%s]",
    this->name().c_str(), signal.c_str(), transporter.c_str());
  if (!resp->success)
  {
    // TODO(luca) implement a queueing mechanism if the request fails?
    RCLCPP_WARN(
      this->_ctx->node.get_logger(),
      "%s: Transporter [%s] is not able to accept [%s] signal now. Skipping signaling",
      this->name().c_str(), transporter.c_str(), signal.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::SUCCESS;
}

}
