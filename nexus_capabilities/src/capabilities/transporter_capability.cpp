/*
 * Copyright (C) 2025 Open Source Robotics Foundation.
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


#include "bid_transporter.hpp"
#include "transporter_capability.hpp"

#include <nexus_capabilities/exceptions.hpp>

#include <yaml-cpp/exceptions.h>

namespace nexus::capabilities {

using rcl_interfaces::msg::ParameterDescriptor;

//==============================================================================
void TransporterCapability::declare_params(
  rclcpp_lifecycle::LifecycleNode& node)
{
  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description =
      "Bid transporter timeout in milliseconds.";
    node.declare_parameter("bid_transporter_timeout", 5000, desc);
  }
}

//==============================================================================
void TransporterCapability::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<const ContextManager> /* ctx_mgr */,
  BT::BehaviorTreeFactory& bt_factory)
{
  auto bid_transporter_timeout =
    node->get_parameter("bid_transporter_timeout").as_int();

  bt_factory.registerBuilder<BidTransporter>(
    "transporter.BidTransporter",
    [this, w_node = std::weak_ptr{node}, bid_transporter_timeout](
      const std::string& name,
      const BT::NodeConfiguration& config)
    {
      auto node = w_node.lock();
      return std::make_unique<BidTransporter>(
        name, config, node, std::chrono::milliseconds{bid_transporter_timeout});
    });
}

}  // namespace nexus::capabilities

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus::capabilities::TransporterCapability,
  nexus::Capability
)
