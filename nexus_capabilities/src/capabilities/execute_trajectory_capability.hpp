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

#ifndef \
  NEXUS_CAPABILITIES__CAPABILITIES__EXECUTE_TRAJECTORY_CAPABILITY_HPP
#define \
  NEXUS_CAPABILITIES__CAPABILITIES__EXECUTE_TRAJECTORY_CAPABILITY_HPP

#include <nexus_capabilities/capability.hpp>
#include <nexus_capabilities/context_manager.hpp>
#include <nexus_endpoints.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>

namespace nexus::capabilities {

/**
 * Capability to execute robot arm trajectories.
 */
class ExecuteTrajectoryCapability : public Capability
{
  /**
   * @copydoc Capability::declare_params
   */
public: void declare_params(rclcpp_lifecycle::LifecycleNode& /* node */) final
  {
  }

  /**
   * @copydoc Capability::configure
   */
public: void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<const ContextManager> ctx_mgr,
    BT::BehaviorTreeFactory& bt_factory) final;

  /**
   * @copydoc Capability::activate
   */
public: void activate() final {}

  /**
   * @copydoc Capability::deactivate
   */
public: void deactivate() final {}
};

}

#endif
