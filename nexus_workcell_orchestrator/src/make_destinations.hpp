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

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__MAKE_DESTINATIONS_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__MAKE_DESTINATIONS_HPP

#include <nexus_capabilities/context_manager.hpp>

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nexus::workcell_orchestrator
{

class MakeDropoffDestinations : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts();

public: MakeDropoffDestinations(
  const std::string& name,
  const BT::NodeConfiguration& config,
  std::shared_ptr<const ContextManager> ctx_mgr,
  rclcpp_lifecycle::LifecycleNode& node)
: BT::SyncActionNode(name, config), _ctx_mgr{ctx_mgr}, _node(node) {}

public: BT::NodeStatus tick() override;

private: std::shared_ptr<const ContextManager> _ctx_mgr;
private: rclcpp_lifecycle::LifecycleNode& _node;
};

}  // namespace nexus::workcell_orchestrator

#endif  // NEXUS_WORKCELL_ORCHESTRATOR__MAKE_DESTINATIONS_HPP