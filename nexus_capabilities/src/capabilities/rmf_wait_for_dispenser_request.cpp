/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include "rmf_wait_for_dispenser_request.hpp"

namespace nexus::capabilities {

//==============================================================================
BT::PortList WaitForDispenserRequest::providedPorts()
{
  return {
    BT::InputPort<std::string>("rmf_task_id"),
    BT::InputPort<std::string>("dispenser")
  };
}

//==============================================================================
BT::NodeStatus WaitForDispenserRequest::onStart()
{
  const auto rmf_task_id = this->getInput<std::string>("rmf_task_id");
  if (!rmf_task_id)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [rmf_task_id] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto dispenser = this->getInput<std::string>("dispenser");
  if (!dispenser)
  {
    RCLCPP_ERROR(
      this->_node->get_logger(), "%s: [dispenser] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  this->_rmf_task_id = *rmf_task_id;
  this->_dispenser = *dispenser;
  this->_received_request = false;

  this->_dispenser_request_sub =
    this->_node->create_subscription<DispenserRequest>(
      "/dispenser_requests",
      10,
      [&](DispenserRequest::UniquePtr msg)
      {
        this->dispenser_request_cb(*msg);
      });
  return BT::NodeStatus::RUNNING;
}

//==============================================================================
BT::NodeStatus WaitForDispenserRequest::onRunning()
{
  if (this->_received_request)
  {
    this->_received_request = false;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

//==============================================================================
void WaitForDispenserRequest::onHalted()
{
  // do nothing
}

//==============================================================================
void WaitForDispenserRequest::dispenser_request_cb(const DispenserRequest& msg)
{
  if (msg.request_guid == this->_rmf_task_id &&
      msg.target_guid == this->_workcell)
  {
    this->_received_request = true;
  }
}

}  // namespace nexus::capabilities
