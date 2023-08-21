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

namespace nexus::workcell_orchestrator {

BT::NodeStatus WaitForSignal::onStart()
{
  auto ctx = this->_ctx_mgr->current_context();
  const auto signal = this->getInput<std::string>("signal");
  if (!signal)
  {
    RCLCPP_ERROR(
      ctx->node.get_logger(), "%s: [signal] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto already_set = ctx->signals.count(*signal);
  if (already_set)
  {
    RCLCPP_DEBUG(
      ctx->node.get_logger(), "%s: signal [%s] already set",
      this->name().c_str(), signal->c_str());
    auto clear = this->getInput<bool>("clear");
    if (clear && *clear)
    {
      ctx->signals.erase(*signal);
      RCLCPP_INFO(
        ctx->node.get_logger(), "%s: cleared signal [%s]",
        this->name().c_str(), signal->c_str());
    }
  }
  return already_set ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForSignal::onRunning()
{
  auto ctx = this->_ctx_mgr->current_context();
  const auto signal = this->getInput<std::string>("signal");
  if (!signal)
  {
    RCLCPP_ERROR(
      ctx->node.get_logger(), "%s: [signal] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (ctx->signals.count(*signal))
  {
    auto clear = this->getInput<bool>("clear");
    if (clear && *clear)
    {
      ctx->signals.erase(*signal);
      RCLCPP_INFO(
        ctx->node.get_logger(), "%s: cleared signal [%s]",
        this->name().c_str(), signal->c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetSignal::tick()
{
  auto ctx = this->_ctx_mgr->current_context();
  auto signal = this->getInput<std::string>("signal");
  if (!signal)
  {
    RCLCPP_ERROR(
      ctx->node.get_logger(), "%s: port [signal] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  ctx->signals.emplace(*signal);
  RCLCPP_DEBUG(
    ctx->node.get_logger(), "%s: set signal [%s]",
    this->name().c_str(), signal->c_str());
  return BT::NodeStatus::SUCCESS;
}

}
