/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

// #include <behaviortree_cpp/decorators/loop_node.h>

#include "rmf_request.hpp"
#include "rmf_request_capability.hpp"

namespace nexus::capabilities {

using namespace rmf;

void RMFRequestCapability::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<const ContextManager> ctx_mgr,
  BT::BehaviorTreeFactory& bt_factory)
{
  bt_factory.registerBuilder<DispatchRequest>("rmf_request.DispatchRequest",
    [this, node](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<DispatchRequest>(name, config, node);
    });

  bt_factory.registerBuilder<ExtractDestinations>("rmf_request.ExtractDestinations",
    [this, node, ctx_mgr](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<ExtractDestinations>(name, config, ctx_mgr, node);
    });

  bt_factory.registerBuilder<UnpackDestinationData>("rmf_request.UnpackDestinationData",
    [this, node](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<UnpackDestinationData>(name, config, node);
    });

  bt_factory.registerBuilder<SignalAmr>("rmf_request.SignalAmr",
    [this, node](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<SignalAmr>(name, config, node);
    });

  bt_factory.registerBuilder<LoopDestinations>("rmf_request.LoopDestinations",
    [this, node](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<LoopDestinations>(name, config, node);
    });

  bt_factory.registerBuilder<WaitForAmr>("rmf_request.WaitForAmr",
    [this, node](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<WaitForAmr>(name, config, node);
    });

  bt_factory.registerBuilder<SendSignal>("rmf_request.SendSignal",
    [this, node, ctx_mgr](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<SendSignal>(name, config, ctx_mgr, node);
    });
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus::capabilities::RMFRequestCapability,
  nexus::Capability
)
