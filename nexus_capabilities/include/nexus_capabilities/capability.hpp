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

#ifndef NEXUS_CAPABILITIES__CAPABILITY_HPP
#define NEXUS_CAPABILITIES__CAPABILITY_HPP

#include "context_manager.hpp"
#include "task.hpp"

#include <nexus_common/bt_store.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>
#include <string>
#include <tuple>

// TODO(YV): Make this class generic enough to be reused by system and workcells.
namespace nexus {

//==============================================================================
/**
 * An implementation of this class is responsible for setting up required ROS endpoints
 * and registering BehaviorTree nodes needed to perform actions needed by a capability.
 */
class Capability
{
public:
  /**
   * A subclass can implement this static method to declare params required.
   * This will be called by the workcell orchestrator when creating the node.
   */
  virtual void declare_params(rclcpp_lifecycle::LifecycleNode& node) = 0;

  /**
   * Configures the capability. Subclass should create all the ROS endpoints needed
   * and registers all the BT node that is supported here.
   * @param node
   * @param bt_factory Must be valid for the lifetime of the capability.
   * @throws StateTransitionError if transition fails.
   */
  virtual void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<const ContextManager> ctx_mgr,
    BT::BehaviorTreeFactory& bt_factory) = 0;

  /**
   * Activates the capability.
   * @throws StateTransitionError if transition fails.
   */
  virtual void activate() = 0;

  /**
   * Deactivates the capability.
   * @throws StateTransitionError if transition fails.
   */
  virtual void deactivate() = 0;

  virtual ~Capability() = default;
};

} // namespace nexus

#endif // NEXUS_CAPABILITIES__CAPABILITY_HPP
