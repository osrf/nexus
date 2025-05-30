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

#include <nexus_transporter_msgs/msg/destination.hpp>

#include "bid_transporter.hpp"

namespace nexus::capabilities {

//==============================================================================
BT::PortsList BidTransporter::providedPorts()
{
  return {
    BT::InputPort<std::vector<nexus_transporter_msgs::msg::Destination>>(
      "destinations"),
    BT::OutputPort<std::string>("result")
  };
}

//==============================================================================
BT::NodeStatus BidTransporter::onStart()
{
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
    endpoints::BidTransporterAction::ActionType>::onStart();
}

//==============================================================================
std::string BidTransporter::get_action_name() const
{
  return endpoints::BidTransporterAction::action_name();
}

//==============================================================================
std::optional<endpoints::BidTransporterAction::ActionType::Goal>
BidTransporter::make_goal()
{
  endpoints::BidTransporterAction::ActionType::Goal goal;
  goal.request.requester = this->_node->get_name();
  goal.request.id = this->_ctx_mgr->current_context()->task.task_id;
  goal.request.destinations = this->_destinations;
  return goal;
}

//==============================================================================
bool BidTransporter::on_result(
  const rclcpp_action::ClientGoalHandle<
  endpoints::BidTransporterAction::ActionType>::WrappedResult& result)
{
  if (!result.result->available)
  {
    this->_ctx_mgr->current_context()->errors.push_back(
      "No transporter available to perform request");
    return false;
  }

  RCLCPP_INFO(
    this->_node->get_logger(),
    "Transporter [%s] is available to perform request",
    result.result->transporter.c_str());
  this->setOutput("result", result.result->transporter);
  return true;
}

} // namespace nexus::capabilities
