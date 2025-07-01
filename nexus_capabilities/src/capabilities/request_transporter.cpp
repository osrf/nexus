/*
 * Copyright (C) 2025 Open Source Robotics Foundation.
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


#include "request_transporter.hpp"

#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>
#include <nexus_transporter_msgs/msg/destination.hpp>

namespace nexus::capabilities {

//==============================================================================
BT::PortsList RequestTransporter::providedPorts()
{
  return {
    BT::InputPort<std::string>("transporter"),
    BT::InputPort<std::vector<nexus_transporter_msgs::msg::Destination>>(
      "destinations"),
    BT::OutputPort<std::string>("transporter_task_id"),
  };
}

//==============================================================================
BT::NodeStatus RequestTransporter::onStart()
{
  auto maybe_transporter = this->getInput<std::string>("transporter");
  if (!maybe_transporter)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [transporter] param is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  this->_transporter = maybe_transporter.value();

  auto maybe_destinations =
    this->getInput<std::vector<nexus_transporter_msgs::msg::Destination>>(
      "destinations");
  if (!maybe_destinations)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [destinations] param is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  this->_destinations = *maybe_destinations;

  return common::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode*,
    endpoints::TransportAction::ActionType>::onStart();
}

//==============================================================================
void RequestTransporter::on_feedback(
  endpoints::TransportAction::ActionType::Feedback::ConstSharedPtr msg)
{
  this->setOutput("transporter_task_id", msg->state.task_id);
}

//==============================================================================
std::string RequestTransporter::get_action_name() const
{
  RCLCPP_INFO(
    this->_node->get_logger(),
    "action name: %s",
    endpoints::TransportAction::action_name(this->_transporter).c_str());
  return endpoints::TransportAction::action_name(this->_transporter);
}

//==============================================================================
std::optional<endpoints::TransportAction::ActionType::Goal>
RequestTransporter::make_goal()
{
  endpoints::TransportAction::ActionType::Goal goal;
  goal.request.id = this->_ctx_mgr->current_context()->task.task_id;
  goal.request.requester = this->_node->get_name();
  goal.request.destinations = this->_destinations;
  return goal;
}

}  // namespace nexus::capabilities
