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

#include "get_joint_constraints.hpp"

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

//==============================================================================
namespace nexus::workcell_orchestrator {

//==============================================================================
BT::PortsList GetJointConstraints::providedPorts()
{
  return { BT::InputPort<moveit_msgs::msg::RobotTrajectory>("trajectory"),
    BT::InputPort<std::size_t>("index"),
    BT::OutputPort<std::vector<moveit_msgs::msg::JointConstraint>>(
      "joint_constraints") };
}

//==============================================================================
BT::NodeStatus GetJointConstraints::tick()
{
  // Check if trajectory is valid.
  auto trajectory = this->getInput<moveit_msgs::msg::RobotTrajectory>(
    "trajectory");
  if (!trajectory)
  {
    RCLCPP_ERROR(
      this->_node.lock()->get_logger(), "%s: port [trajectory] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const auto& joint_trajectory = trajectory->joint_trajectory;
  if (joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(
      _node.lock()->get_logger(),
      "Provided trajectory cannot be empty."
    );
    return BT::NodeStatus::FAILURE;
  }

  // Check if index is valid.
  auto maybe_index = this->getInput<std::size_t>("index");
  const std::size_t index = maybe_index.has_value() ? maybe_index.value() :
    joint_trajectory.points.size() - 1;
  if (index >= joint_trajectory.points.size())
  {
    RCLCPP_ERROR(
      _node.lock()->get_logger(),
      "Provided index exceeds size of trajectory."
    );
    return BT::NodeStatus::FAILURE;
  }

  // Convert TrajectoryPoint to set of JointConstraints.
  // `index` is checked above so the accesses here should never cause OOB.
  // No need to catch exception.
  std::vector<moveit_msgs::msg::JointConstraint> constraints;
  const auto& final_joint_values = joint_trajectory.points.at(index).positions;
  for (std::size_t i = 0; i < final_joint_values.size(); ++i)
  {
    const auto& joint_value = final_joint_values.at(i);
    const auto& joint_name = joint_trajectory.joint_names.at(i);
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = joint_name;
    constraint.position = joint_value;
    constraints.push_back(std::move(constraint));
  }
  this->setOutput("joint_constraints", constraints);
  return BT::NodeStatus::SUCCESS;
}

} // namespace nexus::workcell_orchestrator
