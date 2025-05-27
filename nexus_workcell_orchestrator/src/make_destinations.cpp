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

#include <string>
#include <vector>

#include "make_destinations.hpp"

namespace nexus::workcell_orchestrator
{

//==============================================================================
BT::PortsList MakeDropoffDestinations::providedPorts()
{
  return {
    BT::InputPort<std::vector<std::string>>("destination_names"),
    BT::OutputPort<std::vector<nexus_transporter_msgs::msg::Destination>>(
      "destinations")
  };
}

//==============================================================================
BT::NodeStatus MakeDropoffDestinations::tick()
{
  auto destination_names = this->getInput<std::vector<std::string>>(
    "destination_names");
  if (!destination_names)
  {
    RCLCPP_ERROR(
      this->_node.get_logger(), "%s: port [destination_names] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::vector<nexus_transporter_msgs::msg::Destination> destinations;
  for (const auto & name : *destination_names)
  {
    // TODO(aaronchong): Set up params based on workcell task inputs
    destinations.emplace_back(
      nexus_transporter_msgs::build<nexus_transporter_msgs::msg::Destination>()
        .name(name)
        .action(nexus_transporter_msgs::msg::Destination::ACTION_DROPOFF)
        .params(""));
  }
  this->setOutput("destinations", destinations);

  return BT::NodeStatus::SUCCESS;
}

//==============================================================================
BT::PortsList MakePickupDestinations::providedPorts()
{
  return {
    BT::InputPort<std::vector<std::string>>("destination_names"),
    BT::OutputPort<std::vector<nexus_transporter_msgs::msg::Destination>>(
      "destinations")
  };
}

//==============================================================================
BT::NodeStatus MakePickupDestinations::tick()
{
  auto destination_names = this->getInput<std::vector<std::string>>(
    "destination_names");
  if (!destination_names)
  {
    RCLCPP_ERROR(
      this->_node.get_logger(), "%s: port [destination_names] is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::vector<nexus_transporter_msgs::msg::Destination> destinations;
  for (const auto & name : *destination_names)
  {
    // TODO(aaronchong): Set up params based on workcell task inputs
    destinations.emplace_back(
      nexus_transporter_msgs::build<nexus_transporter_msgs::msg::Destination>()
        .name(name)
        .action(nexus_transporter_msgs::msg::Destination::ACTION_PICKUP)
        .params(""));
  }
  this->setOutput("destinations", destinations);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nexus::workcell_orchestrator
