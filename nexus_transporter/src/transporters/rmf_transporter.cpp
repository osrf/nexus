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

#include "rmf_transporter.hpp"

namespace nexus_transporter {

//==============================================================================
bool RMFTransporter::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr& wp_node)
{
  auto node = wp_node.lock();
  if (node == nullptr)
  {
    return false;
  }
  // Create ROS connections to Open-RMF.
  const auto reliable_qos =
    rclcpp::SystemDefaultQoS().keep_last(100).reliable();
  const auto transient_qos =
    rclcpp::SystemDefaultsQoS().transient_local().keep_last(10).reliable();
  this->_fleet_state_sub = node->create_subscription<FleetState>(
    "/fleet_states",
    reliable_qos,
    [this](FleetState::ConstSharedPtr msg)
    {
      this->_fleet_states.insert_or_assign(msg->name, std::move(msg));
    }
  );
  this->_api_response_sub = node->create_subscription<ApiResponse>(
    "/task_api_responses",
    transient_qos,
    [this](ApiResponse::ConstSharedPtr msg)
    {
      void(msg);
    }
  );
  this->_dispenser_request_sub = node->create_subscription<DispenserRequest>(
    "/dispenser_requests",
    reliable_qos,
    [this](DispenserRequest::ConstSharedPtr msg)
    {
      void(msg);
    }
  );
  this->_api_request_pub = node->create_publisher<ApiRequest>(
    "/task_api_requests",
    transient_qos
  );
  this->_dispenser_result_pub = node->create_publisher<DispenserResult>(
    "/dispenser_results",
    reliable_qos
  );
}

//==============================================================================
bool RMFTransporter::ready() final
{
  bool ready = false;
  for (const auto& it : _fleet_states)
  {
    if (!it->seconds->robots.empty())
    {
      ready = true;
      break;
    }
  }
  return ready;
}

//==============================================================================
std::optional<Itinerary> RMFTransporter::get_itinerary(
  const std::string& job_id,
  const std::vector<Destination>& destinations)
{

}

//==============================================================================
void RMFTransporter::transport_to_destination(
  const Itinerary& itinerary,
  TransportFeedback feedback_cb,
  TransportCompleted completed_cb)
{

}


} // namespace nexus_transporter

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus_transporter::MockTransporter, nexus_transporter::Transporter)
