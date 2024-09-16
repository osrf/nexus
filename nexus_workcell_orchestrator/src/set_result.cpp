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

#include "set_result.hpp"

namespace nexus::workcell_orchestrator {

BT::NodeStatus SetResult::tick()
{
  auto ctx = this->_ctx_mgr->current_context();

  // Ideally we want to allow any ROS message to be used, but BehaviorTree.CPP
  // does not support weakly typed ports.
  auto key = this->getInput<std::string>("key");
  if (!key)
  {
    RCLCPP_ERROR(
      ctx->node->get_logger(), "%s: port [key] is required",
      this->name().c_str());
  }
  auto val = this->getInput<std::string>("value");
  if (!val)
  {
    RCLCPP_ERROR(
      ctx->node->get_logger(), "%s: port [value] is required",
      this->name().c_str());
  }
  ctx->task.previous_results[*key] = *val;
  return BT::NodeStatus::SUCCESS;
}

}
