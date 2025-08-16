// Copyright 2022 Johnson & Johnson
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <stdio.h>

#include <rclcpp/rclcpp.hpp>

#include "motion_planner_server.hpp"

namespace nexus::motion_planner {

class MotionPlannerServerComponent : public rclcpp::Node
{
public:
  explicit MotionPlannerServerComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("motion_planner_server_component", options)
  {
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    RCLCPP_INFO(this->get_logger(), "Creating Motion Planner Server Component");

    // Create the motion planner server node
    motion_planner_server_ = std::make_shared<MotionPlannerServer>(rclcpp::NodeOptions());

    RCLCPP_INFO(this->get_logger(), "Motion Planner Server Component initialized");
  }

  ~MotionPlannerServerComponent()
  {
    motion_planner_server_.reset();
  }

private:
  std::shared_ptr<MotionPlannerServer> motion_planner_server_;
};

}  // namespace nexus::motion_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nexus::motion_planner::MotionPlannerServerComponent)