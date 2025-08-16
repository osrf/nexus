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
#include <thread>
#include <vector>
#include <string>
#include <chrono>

#include "nexus_lifecycle_manager/lifecycle_manager.hpp"
#include <rclcpp/rclcpp.hpp>

namespace nexus::lifecycle_manager {

class LifecycleManagerNode : public rclcpp::Node
{
public:
  explicit LifecycleManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("lifecycle_manager_node", options)
  {
    // Declare parameters
    std::vector<std::string> default_node_names;
    auto node_names = this->declare_parameter("node_names", default_node_names);
    auto service_request_timeout = this->declare_parameter<int>("service_request_timeout", 10);
    auto autostart = this->declare_parameter<bool>("autostart", false);
    auto activate_services = this->declare_parameter<bool>("activate_services", true);

    RCLCPP_INFO(
      this->get_logger(),
      "Lifecycle Manager Node starting with %zu managed nodes",
      node_names.size()
    );

    // Create the lifecycle manager
    lifecycle_manager_ = std::make_shared<LifecycleManager<>>(
      "nexus",
      node_names,
      activate_services,
      autostart,
      service_request_timeout
    );

    // Add system_orchestrator by default if no nodes specified
    if (node_names.empty())
    {
      lifecycle_manager_->addNodeName("system_orchestrator");
      RCLCPP_INFO(this->get_logger(), "Added default node: system_orchestrator");
    }

    RCLCPP_INFO(this->get_logger(), "Lifecycle Manager Node initialized");
  }

  ~LifecycleManagerNode()
  {
    if (lifecycle_manager_)
    {
      lifecycle_manager_.reset();
    }
  }

private:
  std::shared_ptr<LifecycleManager<>> lifecycle_manager_;
};

}  // namespace nexus::lifecycle_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nexus::lifecycle_manager::LifecycleManagerNode)