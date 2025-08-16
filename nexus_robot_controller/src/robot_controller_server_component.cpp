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

#include "nexus_robot_controller/robot_controller_server.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace nexus::robot_controller_server {

class RobotControllerServerComponent : public rclcpp::Node
{
public:
  explicit RobotControllerServerComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("robot_controller_server_component", options)
  {
    // Create executor for the robot controller server
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    // Get parameters
    std::string node_name = this->declare_parameter<std::string>("node_name", "robot_controller_server");
    std::string ns = this->declare_parameter<std::string>("namespace", "");
    std::string cm_node_name = this->declare_parameter<std::string>("controller_manager_node_name", "controller_manager");

    RCLCPP_INFO(
      this->get_logger(),
      "Creating Robot Controller Server with node_name: %s, namespace: %s, cm_node_name: %s",
      node_name.c_str(), ns.c_str(), cm_node_name.c_str()
    );

    // Create the robot controller server
    robot_controller_server_ = std::make_shared<RobotControllerServer>(
      executor_,
      node_name,
      ns,
      cm_node_name,
      rclcpp::NodeOptions()
    );

    // Add the robot controller server to the executor
    executor_->add_node(robot_controller_server_->get_node_base_interface());

    // Start executor in a separate thread
    executor_thread_ = std::thread([this]() {
      executor_->spin();
    });

    RCLCPP_INFO(this->get_logger(), "Robot Controller Server Component initialized");
  }

  ~RobotControllerServerComponent()
  {
    if (executor_)
    {
      executor_->cancel();
    }
    
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }

    robot_controller_server_.reset();
    executor_.reset();
  }

private:
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<RobotControllerServer> robot_controller_server_;
  std::thread executor_thread_;
};

}  // namespace nexus::robot_controller_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nexus::robot_controller_server::RobotControllerServerComponent)