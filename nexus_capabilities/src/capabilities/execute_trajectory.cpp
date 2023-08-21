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

#include "execute_trajectory.hpp"

namespace nexus::capabilities {

BT::PortsList ExecuteTrajectory::providedPorts()
{
  return { BT::InputPort<moveit_msgs::msg::RobotTrajectory>(
      "trajectory")};
}

std::optional<endpoints::ControllerRobotTrajectoryAction::ActionType::Goal>
ExecuteTrajectory::make_goal()
{
  auto traj = this->getInput<moveit_msgs::msg::RobotTrajectory>("trajectory");
  if (!traj)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: port [trajectory] is required",
      this->name().c_str());
    return std::nullopt;
  }
  endpoints::ControllerRobotTrajectoryAction::ActionType::Goal goal;
  goal.trajectory = traj->joint_trajectory;
  goal.multi_dof_trajectory = traj->multi_dof_joint_trajectory;
  return goal;
}

}
