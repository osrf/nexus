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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__DISPATCH_TRANSPORTER_HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__DISPATCH_TRANSPORTER_HPP

#include "context.hpp"
#include "session.hpp"

#include <nexus_common/models/work_order.hpp>
#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <nexus_endpoints.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace nexus::system_orchestrator {

/**
 * Iterates over all the tasks and sends a request to the transporter to move
 * the item inbetween workcells.
 *
 * Input Ports:
 *   destination |std::string| Destination to transport to.
 * Output Ports:
 *   result |std::string| Id of the transporter assigned.
 */
class DispatchTransporter : public BT::StatefulActionNode
{
public: using IsTaskDoableService =
    endpoints::IsTaskDoableService;

public: using WorkcellTask =
    nexus_orchestrator_msgs::msg::WorkcellTask;

public: static BT::PortsList providedPorts();

public: DispatchTransporter(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::WeakPtr w_node,
    std::shared_ptr<Context> ctx)
  : BT::StatefulActionNode{name, config}, _w_node{w_node}, _ctx{std::move(ctx)}
  {
  }

public: BT::NodeStatus onStart() override;
public: BT::NodeStatus onRunning() override;
public: void onHalted() override;

private: struct OngoingRequest
  {
    rclcpp::Client<IsTaskDoableService::ServiceType>::SharedPtr client;
    rclcpp::Client<IsTaskDoableService::ServiceType>::
    FutureAndRequestId fut;
  };

private: rclcpp_lifecycle::LifecycleNode::WeakPtr _w_node;
private: std::shared_ptr<Context> _ctx;
private: std::unordered_map<std::string, OngoingRequest> _ongoing_requests;
private: std::chrono::steady_clock::time_point _time_limit;
private: WorkcellTask _transport_task;

private: void _cleanup_pending_requests();
private: BT::NodeStatus _update_ongoing_requests();
};

}

#endif
