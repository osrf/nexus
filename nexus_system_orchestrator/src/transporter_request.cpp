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

using Task = nexus_orchestrator_msgs::msg::WorkcellTask;

BT::PortsList TransporterRequest::providedPorts()
{
  return { BT::InputPort<std::string>("transporter"),
    BT::InputPort<std::string>("destination"),
    BT::OutputPort<std::string>("transporter_task_id") };
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
  goal.request.id = this->_ctx->get_job_id();
  goal.request.requester = this->_node->get_name();
  // TODO(Yadunund): Parse work order and assign action type and params.
  // See https://github.com/osrf/nexus/issues/68.
  goal.request.destinations.emplace_back(
    nexus_transporter_msgs::build<nexus_transporter_msgs::msg::Destination>()
      .name(this->_destination)
      .action(nexus_transporter_msgs::msg::Destination::ACTION_PICKUP)
      .params("")
  );
  return goal;
}

void TransporterRequest::on_feedback(endpoints::TransportAction::ActionType::Feedback::ConstSharedPtr fb)
{
  this->setOutput("transporter_task_id", fb->state.task_id);
}

}
