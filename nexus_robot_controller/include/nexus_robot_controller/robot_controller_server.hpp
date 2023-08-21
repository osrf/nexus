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

#include <atomic>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "realtime_tools/thread_priority.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

#ifndef NEXUS_ROBOT_CONTROLLER__ROBOT_CONTROLLER_SERVER_HPP_
#define NEXUS_ROBOT_CONTROLLER__ROBOT_CONTROLLER_SERVER_HPP_

namespace nexus::robot_controller_server {
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


/// Lifecycle-managed ros2_control controller manager wrapper
/**
 * This controller server uses the ros2_control controller manager to manage the
 * loading of hardware interface plugins and controllers from the robot_description
 * and associated parameters.
 *
 * The server then manages the controller manager in accordance with node lifecycles.
 * Specifically, it triggers as appropriate at lifecylce transitions:
 * - Loading and unloading of controllers
 * - Switching of controllers
 * - Spinning up or destruction of the controller manager instance
 *
 * The following parameters have to be set:
 * - ~/robot_description
 * - Controller configurations as per ros2_control
 * - ~/managed_controllers (to direct the server to manage the controllers)
 *
 * Furthermore, the controllers managed by this controller server should NOT be
 * manually interacted with using the ros2_control flow, since there is no way for
 * the controller server to check which controllers are active.
 */
class RobotControllerServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  RobotControllerServer(
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string& node_name = "robot_controller_server",
    const std::string& ns = "",
    const std::string& cm_node_name = "controller_manager",
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~RobotControllerServer();

protected:
  // ROS Interfaces ============================================================
  // None

  // Lifecycle =================================================================
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
  struct Implementation;
  std::unique_ptr<Implementation> pimpl_;
};

}  // namespace nexus::robot_controller_server

#endif  // NEXUS_ROBOT_CONTROLLER__ROBOT_CONTROLLER_SERVER_HPP_
