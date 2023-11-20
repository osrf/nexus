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

#include "dispense_item_capability.hpp"

#include "dispense_item_task_data.hpp"

#include <yaml-cpp/exceptions.h>

namespace nexus::capabilities {

using rcl_interfaces::msg::ParameterDescriptor;

//==============================================================================
void DispenseItemCapability::declare_params(
  rclcpp_lifecycle::LifecycleNode& node)
{
  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description = "List of dispensers available";
    node.declare_parameter("dispensers", std::vector<std::string>{}, desc);
  }
  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "Dispenser time out in milliseconds.";
    node.declare_parameter("dispenser_timeout", 5000, desc);
  }
}

//==============================================================================
common::Result<void> DispenseItemCapability::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<const ContextManager> /* ctx_mgr */,
  BT::BehaviorTreeFactory& bt_factory)
{
  auto dispenser_ids = node->get_parameter("dispensers").as_string_array();
  if (dispenser_ids.empty())
  {
    RCLCPP_WARN(node->get_logger(), "No dispensers defined");
  }
  for (const auto& id : dispenser_ids)
  {
    this->_dispensers.emplace_back(DispenseItem::DispenserSession{id,
        node->create_client<endpoints::DispenserService::ServiceType>(
          endpoints::DispenserService::service_name(id))});
  }
  auto dispenser_timeout =
    node->get_parameter("dispenser_timeout").as_int();

  bt_factory.registerBuilder<DispenseItem>(
    "dispense_item.DispenseItem",
    [this, w_node = std::weak_ptr{node},
    dispenser_timeout](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      auto node = w_node.lock();
      return std::make_unique<DispenseItem>(name, config,
      node, this->_dispensers, std::chrono::milliseconds{dispenser_timeout});
    });

  return common::Result<void>();
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus::capabilities::DispenseItemCapability,
  nexus::Capability
)
