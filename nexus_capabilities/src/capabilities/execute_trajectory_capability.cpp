/*
 * Copyright (C) 2022 Johnson & Johnson
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

#include "execute_trajectory_capability.hpp"

#include "execute_trajectory.hpp"

#include <iostream>
#include <memory>
#include <string>

namespace nexus::capabilities {

common::Result<void> ExecuteTrajectoryCapability::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<const ContextManager> /* ctx_mgr */,
  BT::BehaviorTreeFactory& bt_factory)
{
  bt_factory.registerBuilder<ExecuteTrajectory>(
    "execute_trajectory.ExecuteTrajectory",
    [node](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<ExecuteTrajectory>(name, config, node);
    });
  return common::Result<void>();
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus::capabilities::ExecuteTrajectoryCapability,
  nexus::Capability
)
