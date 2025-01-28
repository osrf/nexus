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

#ifndef SRC__TRANSPORTERS__RMF_TRANSPORTER_HPP_
#define SRC__TRANSPORTERS__RMF_TRANSPORTER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include <nexus_transporter/Transporter.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>

namespace nexus_transporter {

//==============================================================================
class RMFTransporter : public Transporter
{
public:
  using ApiRequest = rmf_task_msgs::msg::ApiRequest;
  using ApiResponse = rmf_task_msgs::msg::ApiResponse;
  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResponse = rmf_dispenser_msgs::msg::DispenserResponse;
  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using RobotState = rmf_fleet_msgs::msg::RobotState;

  struct AMR
  {

  };


  bool configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& wp_node) final;

  bool ready() final;

  std::optional<Itinerary> get_itinerary(
    const std::string& job_id,
    const std::vector<Destination>& destinations) final;

  void transport_to_destination(
    const Itinerary& itinerary,
    TransportFeedback feedback_cb,
    TransportCompleted completed_cb) final;

  bool cancel(const Itinerary& itinerary);

private:
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Subscription<ApiResponse>::SharedPtr _api_response_sub;
  rcclcpp::Subscription<DispenserRequest>::SharedPtr _dispenser_request_sub;
  rclcpp::Publisher<ApiRequest>::SharedPtr _api_request_pub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _dispenser_result_pub;
  // Map fleet name to its state.
  std::unordered_map<std::string, FleetState::ConstSharedPtr> _fleet_states;
  // Map job_id to a robot state.
};
} // namespace nexus_transporter

#endif  // SRC__TRANSPORTERS__RMF_TRANSPORTER_HPP_
