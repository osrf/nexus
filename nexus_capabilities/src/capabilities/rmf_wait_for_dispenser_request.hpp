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

#ifndef SRC__CAPABILITIES__RMF_WAIT_FOR_DISPENSER_REQUEST_CPP
#define SRC__CAPABILITIES__RMF_WAIT_FOR_DISPENSER_REQUEST_CPP

#include <nexus_capabilities/context_manager.hpp>
#include <nexus_common/action_client_bt_node.hpp>
#include <nexus_endpoints.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>

#include <behaviortree_cpp_v3/action_node.h>

#include <chrono>

namespace nexus::capabilities {

class WaitForDispenserRequest : public BT::StatefulActionNode
{
public:

  static BT::PortList providedPorts();

  inline WaitForDIspenserRequest(
    const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::chrono::milliseconds timeout)
  : BT::StatefulActionNode(name, config), _node(std::move(node)),
    _timeout(timeout)
  {}

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:

  void dispenser_request_cb(const DispenserRequest& msg);

  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  std::chrono::milliseconds _timeout;
  std::string _rmf_task_id;
  std::string _dispenser;
  rclcpp_Subscription<DispenserRequest>::SharedPtr _dispenser_request_sub;
  std::atomic_bool _received_request = false;
};

}  // namespace nexus::capabilities

#endif  // SRC__CAPABILITIES__RMF_WAIT_FOR_DISPENSER_REQUEST_CPP
