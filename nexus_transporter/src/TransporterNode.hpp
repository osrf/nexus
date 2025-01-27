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

#ifndef SRC__TRANSPORTERNODE_HPP
#define SRC__TRANSPORTERNODE_HPP

#include <nexus_transporter/Itinerary.hpp>
#include <nexus_transporter/Transporter.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <pluginlib/class_loader.hpp>

#include <nexus_endpoints.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <memory>

//==============================================================================
namespace nexus_transporter {

class TransporterNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::LifecycleNode::CallbackReturn;
  using State = rclcpp_lifecycle::State;
  using IsTransporterAvailableService =
    nexus::endpoints::IsTransporterAvailableService;
  using IsTransporterAvailable = IsTransporterAvailableService::ServiceType;
  using RegisterTransporterService =
    nexus::endpoints::RegisterTransporterService;
  using RegisterTransporter = RegisterTransporterService::ServiceType;
  using TransportAction = nexus::endpoints::TransportAction;
  using ActionType = TransportAction::ActionType;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;

  /// Constructor
  TransporterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// Register the transporter to the system orchestrator.
  bool registration_callback();

  /// rclcpp_lifecycle::LifecycleNode functions to override
  CallbackReturn on_configure(const State& previous_state) override;
  CallbackReturn on_cleanup(const State& previous_state) override;
  CallbackReturn on_shutdown(const State& previous_state) override;
  CallbackReturn on_activate(const State& previous_state) override;
  CallbackReturn on_deactivate(const State& previous_state) override;
  CallbackReturn on_error(const State& previous_state) override;

private:

  // Bundling all member variables into a data struct and capturing a shared_ptr
  // of this struct within lambdas of various callbacks will guarantee thread
  // safety over capturing "this" raw ptr by reference.
  // TODO(Yadunund): Add an internal node and get rid of dependency on
  // multithreaded executor.
  struct Data
  {
    /// pluginlib clasloader to dynamically load the transporter plugin.
    pluginlib::ClassLoader<Transporter> transporter_loader;
    /// The loaded transporter plugin.
    pluginlib::UniquePtr<Transporter> transporter;
    /// The fully qualified name of the plugin to load
    std::string transporter_plugin_name;
    /// Duration timeout to wait for before reporting connection failures with
    /// System Orchestrator.
    std::chrono::nanoseconds connection_timeout;

    /// Service server to process IsTransporterAvailable requests.
    rclcpp::Service<IsTransporterAvailable>::SharedPtr availability_srv;

    // ROS 2 Action server to handle Transport requests.
    rclcpp_action::Server<ActionType>::SharedPtr
      action_srv;

    /// A separate callback group to prevent deadlock during state transitions
    /// and other callbacks that need to execute.
    rclcpp::CallbackGroup::SharedPtr cb_group;

    // Map itinerary id to GoalData.
    std::unordered_map<rclcpp_action::GoalUUID, std::unique_ptr<Itinerary>>
    itineraries;

    // TF broadcaster for transporter poses during action feedback.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // Weakptr to lifecycle node.
    std::weak_ptr<rclcpp_lifecycle::LifecycleNode> w_node;

    Data();
  };

  std::shared_ptr<Data> _data;

  rclcpp::TimerBase::SharedPtr _register_timer;
};

} // namespace nexus_transporter

#endif // SRC__TRANSPORTERNODE_HPP
