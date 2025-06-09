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

#include "rmf_capability.hpp"
#include "rmf_wait_for_dispenser_request.hpp"

#include <nexus_capabilities/exceptions.hpp>

namespace nexus::capabilities {

using rcl_interfaces::msg::ParameterDescriptor;

//==============================================================================
void RMFCapability::declare_params(rclcpp_lifecycle::LifecycleNode&)
{
  {
    ParameterDescriptor desc;
    desc.read_only = true;
    desc.description = "Waiting timeout in milliseconds.";
    node.declare_parameter("wait_timeout", 5000, desc);
  }
}

//==============================================================================
void RMFCapability::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<const ContextManager> ctx_mgr,
  BT::BehaviorTreeFactory& bt_factory)
{
  bt_factory.registerBuilder<BidTransporter>(
    "transporter.BidTransporter",
    [ctx_mgr, node](
      const std::string& name,
      const BT::NodeConfiguration& config)
    {
      return std::make_unique<BidTransporter>(
        name, config, ctx_mgr, *node);
    });

  bt_factory.registerBuilder<RequestTransporter>(
    "transporter.RequestTransporter",
    [ctx_mgr, node](
      const std::string& name,
      const BT::NodeConfiguration& config)
    {
      return std::make_unique<RequestTransporter>(
        name, config, ctx_mgr, *node);
    });
}

}  // namespace nexus::capabilities

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus::capabilities::RMFCapability,
  nexus::Capability
)
