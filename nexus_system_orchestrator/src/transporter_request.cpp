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

namespace nexus::system_orchestrator {

using Task = nexus_orchestrator_msgs::msg::WorkcellTask;

BT::PortsList TransporterRequest::providedPorts()
{
  return { BT::InputPort<std::string>("transporter"),
    BT::InputPort<std::string>("destination"),
    BT::InputPort<std::string>("source"),
    BT::InputPort<std::string>("signal_destination"),
    BT::InputPort<std::string>("signal_source")
  };
}

BT::NodeStatus TransporterRequest::onStart()
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

  auto maybe_destination = this->getInput<std::string>("destination");
  if (!maybe_destination)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [destination] param is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  this->_destination = maybe_destination.value();
  RCLCPP_ERROR(
    this->_node->get_logger(), "DESTINATION IS %s",
    this->_destination.c_str());

  // The following are not mandatory parameters
  if (auto maybe_source = this->getInput<std::string>("source"))
  {
    this->_source = maybe_source.value();
    RCLCPP_ERROR(
      this->_node->get_logger(), "SOURCE IS %s",
      this->_source.c_str());
  }
  if (auto maybe_signal_source = this->getInput<std::string>("signal_source"))
  {
    this->_signal_source = maybe_signal_source.value();
    RCLCPP_ERROR(
      this->_node->get_logger(), "SIGNAL SOURCE IS %s",
      this->_signal_source.c_str());
  }
  if (auto maybe_signal_destination = this->getInput<std::string>("signal_destination"))
  {
    this->_signal_destination = maybe_signal_destination.value();
    RCLCPP_ERROR(
      this->_node->get_logger(), "SIGNAL DESTINATION IS %s",
      this->_signal_destination.c_str());
  }

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
  goal.request.id = this->_ctx->job_id;
  goal.request.requester = this->_node->get_name();
  goal.request.destination = this->_destination;
  goal.request.source = this->_source;
  goal.request.signal_destination = this->_signal_destination;
  goal.request.signal_source = this->_signal_source;
  return goal;
}

}
