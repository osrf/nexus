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

#include "gripper_control.hpp"

#include <string>

namespace nexus::capabilities {

BT::PortsList GripperControl::providedPorts()
{
  return { BT::InputPort<std::string>("gripper"),
    BT::InputPort<double>("position"), BT::InputPort<double>("max_effort") };
}

std::optional<endpoints::GripperCommandAction::ActionType::Goal> GripperControl
::make_goal()
{
  endpoints::GripperCommandAction::ActionType::Goal goal;
  auto position = this->getInput<double>("position");
  if (!position.has_value())
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: port [position] is required",
      this->name().c_str());
    return std::nullopt;
  }
  auto max_effort = this->getInput<double>("max_effort");
  if (!max_effort.has_value())
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: port [max_effort] is required",
      this->name().c_str());
    return std::nullopt;
  }
  goal.command.position = *position;
  goal.command.max_effort = *max_effort;
  return goal;
}

}
