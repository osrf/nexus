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

#ifndef NEXUS_CAPABILITIES__CAPABILITIES__EXECUTE_TRAJECTORY_HPP
#define NEXUS_CAPABILITIES__CAPABILITIES__EXECUTE_TRAJECTORY_HPP

#include <nexus_common/action_client_bt_node.hpp>
#include <nexus_endpoints.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>

namespace nexus::capabilities {

/**
 * Executes a trajectory on a robot arm.
 *
 * Input Ports:
 *   trajectory |moveit_msgs::msg::RobotTrajectory| The trajectory to execute.
 */
class ExecuteTrajectory : public common
  ::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode::SharedPtr,
    endpoints::ControllerRobotTrajectoryAction::ActionType>
{
public: using ActionType =
    endpoints::ControllerRobotTrajectoryAction::ActionType;

  /**
   * Defines provided ports for the BT node.
   */
public: static BT::PortsList providedPorts();

  /**
   * @param name name of the BT node.
   * @param config config of the BT node.
   * @param logger rclcpp logger to use.
   * @param client The rclcpp action client to use.
   */
public: inline ExecuteTrajectory(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : common::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode::SharedPtr,
      ActionType>(
      name, config, std::move(node)) {}

protected: std::string get_action_name() const
  override
  {
    return endpoints::ControllerRobotTrajectoryAction::action_name();
  }

protected: std::optional<endpoints::ControllerRobotTrajectoryAction::ActionType::Goal>
  make_goal() override;
};

}

#endif
