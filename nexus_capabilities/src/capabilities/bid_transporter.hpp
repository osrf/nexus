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

#ifndef SRC__CAPABILITIES__BID_TRANSPORTER_HPP
#define SRC__CAPABILITIES__BID_TRANSPORTER_HPP

#include <nexus_capabilities/context_manager.hpp>
#include <nexus_common/action_client_bt_node.hpp>
#include <nexus_endpoints.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nexus::capabilities {

class BidTransporter : public common::ActionClientBtNode<
  rclcpp_lifecycle::LifecycleNode*, endpoints::BidTransporterAction::ActionType>
{
public: static BT::PortsList providedPorts();

public: inline BidTransporter(
    const std::string& name,
    const BT::NodeConfiguration& config,
    std::shared_ptr<const ContextManager> ctx_mgr,
    rclcpp_lifecycle::LifecycleNode& node)
  : common::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode*,
    endpoints::BidTransporterAction::ActionType>(name, config, &node),
    _ctx_mgr(std::move(ctx_mgr)) {}

public: BT::NodeStatus onStart() override;

protected: std::string get_action_name() const override;

protected: std::optional<endpoints::BidTransporterAction::ActionType::Goal>
  make_goal() override;

protected: bool on_result(
  const rclcpp_action::ClientGoalHandle<
  endpoints::BidTransporterAction::ActionType>::WrappedResult& result) override;

private: std::shared_ptr<const ContextManager> _ctx_mgr;
private: std::vector<nexus_transporter_msgs::msg::Destination> _destinations;
};

}  // namespace nexus::capabilities

#endif  // SRC__CAPABILITIES__BID_TRANSPORTER_HPP
