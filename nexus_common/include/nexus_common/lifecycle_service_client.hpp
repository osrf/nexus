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

#ifndef NEXUS_COMMON__LIFECYCLE_SERVICE_CLIENT_HPP_
#define NEXUS_COMMON__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nexus_common/sync_service_client.hpp"
#include "nexus_common/node_utils.hpp"

using std::chrono::seconds;
using std::string;
using namespace std::chrono_literals;

namespace nexus::common {

/// Helper functions to interact with a lifecycle node.
template<class NodeType = rclcpp::Node>
class LifecycleServiceClient
{
public:
  explicit LifecycleServiceClient(const std::string& lifecycle_node_name)
  : node_(generate_internal_node(lifecycle_node_name + "_lifecycle_client")),
    change_state_(node_, lifecycle_node_name + "/change_state"),
    get_state_(node_, lifecycle_node_name + "/get_state")
  {
  }

  LifecycleServiceClient(
    const std::string& lifecycle_node_name,
    const std::shared_ptr<NodeType> parent_node)
  : node_(parent_node),
    change_state_(node_, lifecycle_node_name + "/change_state"),
    get_state_(node_, lifecycle_node_name + "/get_state")
  {
  }

  /// Trigger a state change
  ///
  /// \param transition It takes a lifecycle_msgs::msg::Transition id and
  /// try to reach the state
  /// \param timeout Time to wait for the service
  bool change_state(
    const uint8_t transition,  //
    const std::chrono::seconds timeout = std::chrono::seconds(5))
  {
    if (!change_state_.wait_for_service(timeout))
    {
      return false;
    }

    auto request =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    auto response = change_state_.send_request(request, timeout);
    return response.get()->success;
  }

  /// Get the current state as a lifecycle_msgs::msg::State id value
  /// \param timeout Time to wait for the service
  std::optional<uint8_t> get_state(
    const std::chrono::seconds timeout = std::chrono::seconds(2))
  {
    if (!get_state_.wait_for_service(timeout))
    {
      return std::nullopt;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto result = get_state_.send_request(request, timeout);
    return std::optional<uint8_t>(result->current_state.id);
  }

protected:
  const std::shared_ptr<NodeType> node_;
  SyncServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;
  SyncServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};

}  // namespace nexus::common

#endif  // NEXUS_COMMON__LIFECYCLE_SERVICE_CLIENT_HPP_
