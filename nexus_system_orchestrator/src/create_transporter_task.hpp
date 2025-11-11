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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__CREATE_TRANSPORTER_TASK_HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__CREATE_TRANSPORTER_TASK_HPP

#include "context.hpp"
#include "session.hpp"

#include <behaviortree_cpp_v3/action_node.h>

#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>
#include <nexus_transporter_msgs/msg/transportation_request.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nexus::system_orchestrator {

/**
 * Create a transporter task, if necessary, to deliver the item to the target workcell
 * It will look at the current context to know where the input items are and create a transport
 * request if it's needed, or std::nullopt if not needed
 *
 * TODO(luca) if we had a struct with both the workcell task and the workcell it is assigned to this would
 * require only one input
 *
 * Input Ports:
 *   workcell_task |WorkcellTask| The task that the workcell is trying to execute, used to get its input items
 */
class CreateTransporterTask : public BT::SyncActionNode
{
public: using WorkcellTask = nexus_orchestrator_msgs::msg::WorkcellTask;
public: using TransportationRequest = nexus_transporter_msgs::msg::TransportationRequest;
public: static BT::PortsList providedPorts()
  {
    return { BT::InputPort<WorkcellTask>("workcell_task",
        "The task that the workcell is trying to execute."),
      BT::OutputPort<std::optional<TransportationRequest>>(
         "result", "The result transportation task, or std::nullopt if it's not needed") };
  }

public: inline CreateTransporterTask(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::WeakPtr w_node,
    std::shared_ptr<Context> ctx)
  : BT::SyncActionNode(name, config),
    _w_node(w_node),
    _ctx{std::move(ctx)} {}

public: BT::NodeStatus tick() override;

private: rclcpp_lifecycle::LifecycleNode::WeakPtr _w_node;
private: std::shared_ptr<Context> _ctx;
};

/**
 * Unpacks the optional transporter task as an input, returning SUCCESS and the content if present
 * or FAILURE if not present.
 *
 * Input Ports:
 *   input |std::optional<TransportationRequest>| The task that the node will unpack
 */
class UnpackTransporterTask : public BT::SyncActionNode
{
public: using TransportationRequest = nexus_transporter_msgs::msg::TransportationRequest;
public: static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::optional<TransportationRequest>>(
         "input", "An optional TransportationRequest"),
      BT::OutputPort<TransportationRequest>(
         "output", "The contained TransportationRequest, if available") };
  }

public: inline UnpackTransporterTask(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode::WeakPtr w_node)
  : BT::SyncActionNode(name, config),
    _w_node(w_node) {}

public: BT::NodeStatus tick() override;

private: rclcpp_lifecycle::LifecycleNode::WeakPtr _w_node;
};

}

#endif

