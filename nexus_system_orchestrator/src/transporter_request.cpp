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

#include "transporter_request.hpp"

#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>
#include <nexus_transporter_msgs/msg/destination.hpp>

namespace nexus::system_orchestrator {

BT::PortsList TransporterRequest::providedPorts()
{
  return { BT::InputPort<TransportationRequest>("transport_task"),
    BT::InputPort<std::string>("transporter") };
}

BT::NodeStatus TransporterRequest::onStart()
{
  auto maybe_task = this->getInput<TransportationRequest>("transport_task");
  if (!maybe_task)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [transport_task] param is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  this->_request = maybe_task.value();

  auto maybe_transporter = this->getInput<std::string>("transporter");
  if (!maybe_transporter)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [transporter] param is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  this->_transporter = maybe_transporter.value();

  return common::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode*,
      endpoints::TransportAction::ActionType>::
    onStart();
}

std::string TransporterRequest::get_action_name() const
{
  return endpoints::TransportAction::action_name(this->_transporter);
}

std::optional<endpoints::TransportAction::ActionType::Goal> TransporterRequest::
make_goal()
{
  endpoints::TransportAction::ActionType::Goal goal;
  goal.request = this->_request;
  return goal;
}

}
