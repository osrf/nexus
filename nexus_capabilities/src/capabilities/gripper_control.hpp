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

#ifndef NEXUS_CAPABILITIES__CAPABILITIES__GRIPPER_CONTROL_HPP
#define NEXUS_CAPABILITIES__CAPABILITIES__GRIPPER_CONTROL_HPP

#include <nexus_common/action_client_bt_node.hpp>
#include <nexus_endpoints.hpp>

#include <behaviortree_cpp_v3/tree_node.h>

#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>
#include <string>

namespace nexus::capabilities {

/**
 * Activates or deactivates the gripper.
 *
 * Input Ports:
 *   gripper |std::string| Gripper id.
 *   position |double| Position of the gripper.
 *   max_effort |double| Max effort.
 */
class GripperControl : public common::
  ActionClientBtNode<rclcpp_lifecycle::LifecycleNode*,
    endpoints::GripperCommandAction::ActionType>
{
public: using ActionType = endpoints::GripperCommandAction::ActionType;

  /**
   * Defines provided ports for the BT node.
   */
public: static BT::PortsList providedPorts();

  /**
   * @param name name of the BT node.
   * @param config config of the BT node.
   * @param logger rclcpp logger to use.
   * @param client_factory a callable that returns the rclcpp action client to use.
   */
public: inline GripperControl(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode& node)
  : common::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode*, ActionType>(
      name, config, &node) {}

protected: std::string get_action_name() const override
  {
    const auto gripper_name = this->getInput<std::string>("gripper");
    return endpoints::GripperCommandAction::action_name(*gripper_name);
  }

protected: std::optional<endpoints::GripperCommandAction::ActionType::Goal>
  make_goal() override;
};

}

#endif
