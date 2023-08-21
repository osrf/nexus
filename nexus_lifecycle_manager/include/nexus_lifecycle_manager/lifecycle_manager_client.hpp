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
#ifndef NEXUS_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
#define NEXUS_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_

#include <memory>
#include <string>

#include "nexus_lifecycle_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nexus_common/sync_service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace nexus::lifecycle_manager {
/// \enum nexus_lifecycle_manager::SystemStatus
/// \brief Enum class representing the status of the system.
enum class SystemStatus {ACTIVE, INACTIVE, TIMEOUT};

/// \class nexus_lifecycle_manager::LifeCycleMangerClient
/// \brief The LifecycleManagerClient sends requests to the LifecycleManager to
/// control the lifecycle state of the navigation modules.
class LifecycleManagerClient
{
public:
  /// \brief A constructor for LifeCycleMangerClient
  /// \param[in] _name Managed node name
  /// \param[in] _parent_node Node that execute the service calls
  explicit LifecycleManagerClient(
    const std::string& _name,
    std::shared_ptr<rclcpp::Node> _parent_node);

  // Client-side interface to the lifecycle manager
  /// \brief Make start up service call
  /// \return true or false
  bool startup(
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// \brief Make shutdown service call
  /// \return true or false
  bool shutdown(
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// \brief Make pause service call
  /// \return true or false
  bool pause(
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// \brief Make resume service call
  /// \return true or false
  bool resume(
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// \brief Make reset service call
  /// \return true or false
  bool reset(
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  /// \brief Check if lifecycle node manager server is active
  /// \return ACTIVE or INACTIVE or TIMEOUT
  SystemStatus is_active(
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

protected:
  using ManageLifecycleNodes = nexus_lifecycle_msgs::srv::ManageLifecycleNodes;

  /// \brief A generic method used to call startup, shutdown, etc.
  /// \param command
  /// \param timeout
  bool callService(
    uint8_t command,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  // The node to use for the service call
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<common::SyncServiceClient<ManageLifecycleNodes>>
  manager_client_;
  std::shared_ptr<common::SyncServiceClient<std_srvs::srv::Trigger>>
  is_active_client_;
  std::string manage_service_name_;
  std::string active_service_name_;
};

}  // namespace nexus::lifecycle_manager

#endif  // NEXUS_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_CLIENT_HPP_
