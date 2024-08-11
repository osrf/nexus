/*
 * Copyright (C) 2023 Johnson & Johnson
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

#ifndef SRC__CALIBRATION_NODE_HPP
#define SRC__CALIBRATION_NODE_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nexus_endpoints.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <tf2/buffer_core.h>

// VRPN headers
#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>

// =============================================================================
namespace nexus {

class CalibrationNode : public rclcpp_lifecycle::LifecycleNode
{

public:

  using CallbackReturn = rclcpp_lifecycle::LifecycleNode::CallbackReturn;
  using ExtrinsicCalibrationService = endpoints::ExtrinsicCalibrationService;
  using ExtrinsicCalibrationServiceType =
    ExtrinsicCalibrationService::ServiceType;
  using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
  using State = rclcpp_lifecycle::State;
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  /// @brief  Constructor
  /// @param options NodeOptions
  CalibrationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// rclcpp_lifecycle::LifecycleNode functions to override
  CallbackReturn on_configure(const State& previous_state) override;
  CallbackReturn on_cleanup(const State& previous_state) override;
  CallbackReturn on_shutdown(const State& previous_state) override;
  CallbackReturn on_activate(const State& previous_state) override;
  CallbackReturn on_deactivate(const State& previous_state) override;
  CallbackReturn on_error(const State& previous_state) override;

  ~CalibrationNode();

private:

  // Forward declare.
  struct Tracker;
  using TrackerPtr = std::shared_ptr<Tracker>;

  // Map tracker name to its Tracker.
  std::unordered_map<std::string, TrackerPtr> _trackers;

  // All trackers will dump their frames into this buffer.
  std::shared_ptr<tf2::BufferCore> _tf_buffer;

  // VRPN connection
  std::shared_ptr<vrpn_Connection> _vrpn_connection;

  // The frame_id in which tracking data is being streamed. Usually world.
  std::string _vrpn_frame_id = "world";

  // The name of the workcell which is being calibrated.
  std::string _workcell_id;

  // Destination of server in ip:port format.
  std::string _server_address;

  // Tracker names
  std::vector<std::string> _tracker_names;

  // Timer to run the vrvpn API's update functions.
  // TODO(YV): Consider disabling this timer once good quality updates
  // are received to minimize CPU load.
  rclcpp::TimerBase::SharedPtr _timer;

  // Rate for timer.
  double _update_rate;

  // Service server
  rclcpp::Service<ExtrinsicCalibrationServiceType>::SharedPtr
    _extrinsic_service;

};

} // namespace nexus


#endif // SRC__CALIBRATION_NODE_HPP
