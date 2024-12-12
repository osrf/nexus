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

#include "signals.hpp"

#include <nexus_endpoints.hpp>

namespace nexus::system_orchestrator {

BT::NodeStatus SendSignal::tick()
{
  auto workcell = this->getInput<std::string>("workcell");
  if (!workcell)
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s [workcell] is required",
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

  return signal_workcell(*workcell, *signal);
}

BT::NodeStatus SendSignal::signal_workcell(const std::string& workcell, const std::string& signal)
{
  auto it = this->_ctx->workcell_sessions.find(workcell);
  if (it == this->_ctx->workcell_sessions.cend())
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(),
      "%s: Unable to find workcell [%s]",
      this->name().c_str(), workcell.c_str());
    return BT::NodeStatus::FAILURE;
  }
  const auto& session = it->second;
  if (!session)
  {
    // TODO(luca) implement a queueing mechanism if the workcell is not available?
    RCLCPP_WARN(
      this->_ctx->node.get_logger(),
      "%s: No session found for workcell [%s], skipping signaling",
      this->name().c_str(), workcell.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  auto req =
    std::make_shared<endpoints::SignalWorkcellService::ServiceType::Request>();
  // TODO(luca) Using job id for task id is not scalable, generate uuids?
  req->task_id = this->_ctx->job_id;
  req->signal = signal;
  auto resp = session->signal_wc_client->send_request(req);
  RCLCPP_INFO(
    this->_ctx->node.get_logger(), "%s: Sent signal [%s] to workcell [%s]",
    this->name().c_str(), signal.c_str(), workcell.c_str());
  if (!resp->success)
  {
    // TODO(luca) implement a queueing mechanism if the request fails?
    RCLCPP_WARN(
      this->_ctx->node.get_logger(),
      "%s: Workcell [%s] is not able to accept [%s] signal now. Skipping signaling, error: [%s]",
      this->name().c_str(), workcell.c_str(), signal.c_str(), resp->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus WaitForSignal::onStart()
{
  const auto signal = this->getInput<std::string>("signal");
  if (!signal)
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s: [signal] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto& signals = this->_ctx->orchestrator_signals;
  if (auto signal_it = std::find(signals.begin(), signals.end(), *signal); signal_it != signals.end())
  {
    RCLCPP_DEBUG(
      this->_ctx->node.get_logger(), "%s: signal [%s] already set",
      this->name().c_str(), signal->c_str());
    auto clear = this->getInput<bool>("clear");
    if (clear && *clear)
    {
      signals.erase(signal_it);
      RCLCPP_INFO(
        this->_ctx->node.get_logger(), "%s: cleared signal [%s]",
        this->name().c_str(), signal->c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForSignal::onRunning()
{
  const auto signal = this->getInput<std::string>("signal");
  if (!signal)
  {
    RCLCPP_ERROR(
      this->_ctx->node.get_logger(), "%s: [signal] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto& signals = this->_ctx->orchestrator_signals;
  for (const auto& signal : signals)
  {
    RCLCPP_INFO(
      this->_ctx->node.get_logger(), "%s: Signal available [%s]",
      this->name().c_str(), signal.c_str());
  }
  if (auto signal_it = std::find(signals.begin(), signals.end(), *signal); signal_it != signals.end())
  {
    RCLCPP_DEBUG(
      this->_ctx->node.get_logger(), "%s: signal [%s] already set",
      this->name().c_str(), signal->c_str());
    auto clear = this->getInput<bool>("clear");
    if (clear && *clear)
    {
      signals.erase(signal_it);
      RCLCPP_INFO(
        this->_ctx->node.get_logger(), "%s: cleared signal [%s]",
        this->name().c_str(), signal->c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

}
