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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__TRANSPORTER_REQUEST_HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__TRANSPORTER_REQUEST_HPP

#include "context.hpp"
#include "session.hpp"

#include <nexus_common/action_client_bt_node.hpp>
#include <nexus_endpoints.hpp>

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nexus::system_orchestrator {

/**
 * Request a transporter to do some work.
 *
 * Input Ports:
 *   transporter |std::string| The transporter to use.
 *   destination |std::string| A map between task id and it's transporter session.
 */
class TransporterRequest : public common::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode*,
    endpoints::TransportAction::ActionType>
{
public: using ActionType = endpoints::TransportAction::ActionType;

public: static BT::PortsList providedPorts();

public: inline TransporterRequest(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode& node,
    std::shared_ptr<Context> ctx)
  : common::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode*,
      endpoints::TransportAction::ActionType>(name, config, &node),
    _ctx{std::move(ctx)} {}

public: BT::NodeStatus onStart() override;

protected: std::string get_action_name() const override;

protected: std::optional<endpoints::TransportAction::ActionType::Goal> make_goal()
  override;

private: std::shared_ptr<Context> _ctx;
private: std::string _transporter;
private: std::string _destination;
private: std::string _source;
};

}

#endif
