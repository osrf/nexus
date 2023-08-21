/*
 * Copyright (C) 2023 Johnson & Johnson
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

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__GET_JOINT_CONSTRAINT_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__GET_JOINT_CONSTRAINT_HPP

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nexus::workcell_orchestrator {

/**
 * Extract and convert a TrajectoryPoint from
 * `moveit_msgs::msg::RobotTrajectory` into a set of
 * moveit_msgs::msg::JointConstraints.
 * Input Ports:
 *   trajectory |moveit_msgs::msg::Detection3DArray|
 *   index |OPTIONAL. The index of the Trajectory point to convert.
 *   Defaults to the last index.
 * Output Ports:
 *   joint_constraints |std::vector<moveit_msgs::msg::JointConstraint>|
 */
class GetJointConstraints : public BT::SyncActionNode
{
public: GetJointConstraints(
    const std::string& name,
    const BT::NodeConfiguration& config,
    std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node)
  : BT::SyncActionNode{name, config},
    _node(node)
  {
    // Do nothing
  }

public: static BT::PortsList providedPorts();

public: BT::NodeStatus tick() override;

private: std::weak_ptr<rclcpp_lifecycle::LifecycleNode> _node;
};

} // namespace nexus::workcell_orchestrator

#endif //NEXUS_WORKCELL_ORCHESTRATOR__GET_JOINT_CONSTRAINT_HPP
