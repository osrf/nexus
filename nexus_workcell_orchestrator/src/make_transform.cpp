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

#include "make_transform.hpp"

#include <geometry_msgs/msg/transform.hpp>

namespace nexus::workcell_orchestrator {

BT::PortsList MakeTransform::providedPorts()
{
  return { BT::InputPort<double>("x"), BT::InputPort<double>("y"),
    BT::InputPort<double>("z"), BT::InputPort<double>("qx"),
    BT::InputPort<double>("qy"), BT::InputPort<double>("qz"),
    BT::InputPort<double>("qw"), BT::OutputPort<geometry_msgs::msg::Transform>(
      "pose") };
}

BT::NodeStatus MakeTransform::tick()
{
  geometry_msgs::msg::Transform t;
  t.translation.x = this->getInput<double>("x").value_or(0);
  t.translation.y = this->getInput<double>("y").value_or(0);
  t.translation.z = this->getInput<double>("z").value_or(0);
  t.rotation.x = this->getInput<double>("qx").value_or(0);
  t.rotation.y = this->getInput<double>("qy").value_or(0);
  t.rotation.z = this->getInput<double>("qz").value_or(0);
  t.rotation.w = this->getInput<double>("qw").value_or(1.0);
  this->setOutput("result", t);
  return BT::NodeStatus::SUCCESS;
}

}
