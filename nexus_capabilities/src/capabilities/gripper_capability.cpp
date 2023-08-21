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

#include "gripper_capability.hpp"

#include "gripper_control.hpp"

namespace nexus::capabilities {

using rcl_interfaces::msg::ParameterDescriptor;

void GripperCapability::declare_params(rclcpp_lifecycle::LifecycleNode& node)
{
  ParameterDescriptor desc;
  desc.read_only = true;
  desc.description = "List of grippers available";
  node.declare_parameter("grippers", std::vector<std::string>{}, desc);
}

void GripperCapability::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<const ContextManager> /* ctx_mgr */,
  BT::BehaviorTreeFactory& bt_factory)
{
  auto grippers = node->get_parameter("grippers").as_string_array();
  if (grippers.empty())
  {
    RCLCPP_WARN(node->get_logger(), "No grippers defined");
  }

  bt_factory.registerBuilder<GripperControl>("gripper.GripperControl",
    [node](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<GripperControl>(name, config, *node);
    });
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus::capabilities::GripperCapability,
  nexus::Capability
)
