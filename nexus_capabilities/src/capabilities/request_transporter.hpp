/*
 * Copyright (C) 2025 Open Source Robotics Foundation.
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

#ifndef SRC__CAPABILITIES__REQUEST_TRANSPORTER_HPP
#define SRC__CAPABILITIES__REQUEST_TRANSPORTER_HPP

#include <nexus_capabilities/context_manager.hpp>
#include <nexus_common/action_client_bt_node.hpp>
#include <nexus_endpoints.hpp>

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nexus::capabilities {

/**
 * Request a transporter to do some work.
 *
 * Input Ports:
 *   transporter |std::string| The transporter to use.
 *   destinations |std::vector<nexus_transporter_msgs::msg::Destination>|
 *     A vector of destinations to transport to.
 */
class RequestTransporter : public common::ActionClientBtNode<
rclcpp_lifecycle::LifecycleNode*, endpoints::TransportAction::ActionType>
{
public: using ActionType = endpoints::TransportAction::ActionType;

public: static BT::PortsList providedPorts();

public: inline RequestTransporter(
    const std::string& name,
    const BT::NodeConfiguration& config,
    std::shared_ptr<const ContextManager> ctx_mgr,
    rclcpp_lifecycle::LifecycleNode& node)
  : common::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode*,
    endpoints::TransportAction::ActionType>(name, config, &node),
    _ctx_mgr(std::move(ctx_mgr)) {}

public: BT::NodeStatus onStart() override;

protected: void on_feedback(
    endpoints::TransportAction::ActionType::Feedback::ConstSharedPtr msg)
  override;

protected: std::string get_action_name() const override;

protected: std::optional<endpoints::TransportAction::ActionType::Goal>
  make_goal() override;

private: std::shared_ptr<const ContextManager> _ctx_mgr;
private: std::string _transporter;
private: std::vector<nexus_transporter_msgs::msg::Destination> _destinations;
};

}  // namespace nexus::capabilities

#endif  // SRC__CAPABILITIES__REQUEST_TRANSPORTER_HPP
