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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__WORKCELL_REQUEST_HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__WORKCELL_REQUEST_HPP

#include "context.hpp"
#include "session.hpp"

#include <nexus_common/action_client_bt_node.hpp>
#include <nexus_common/models/work_order.hpp>
#include <nexus_endpoints.hpp>
#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nexus::system_orchestrator {

/**
 * Request a workcell to do some work.
 *
 * Input Ports:
 *   workcell |std::string| Workcell to use.
 *   task |nexus_orchestrator_msgs::msg::WorkcellTask| The task to execute.
 */
class WorkcellRequest : public common::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode*,
    endpoints::WorkcellRequestAction::ActionType>
{
public: using ActionType = endpoints::WorkcellRequestAction::ActionType;
public: using Task = nexus_orchestrator_msgs::msg::WorkcellTask;
public: using TaskState = nexus_orchestrator_msgs::msg::TaskState;
public: using OnTaskProgressCb = std::function<void(const std::unordered_map<std::string,
      TaskState>& task_states)>;

public: static BT::PortsList providedPorts();

public: inline WorkcellRequest(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode& node,
    std::shared_ptr<Context> ctx, OnTaskProgressCb on_task_progress_cb)
  : common::ActionClientBtNode<rclcpp_lifecycle::LifecycleNode*,
      endpoints::WorkcellRequestAction::ActionType>(name, config, &node), _ctx(std::move(
        ctx)), _on_task_progress(std::move(on_task_progress_cb)) {}

public: BT::NodeStatus onStart() override;

protected: std::string get_action_name() const override;

protected: std::optional<endpoints::WorkcellRequestAction::ActionType::Goal>
  make_goal() override;

protected: void on_feedback(
    endpoints::WorkcellRequestAction::ActionType::Feedback::ConstSharedPtr msg)
  override;

protected: bool on_result(
    const rclcpp_action::ClientGoalHandle<endpoints::WorkcellRequestAction::ActionType>::WrappedResult& result)
  override;

private: Task _task;
private: std::string _workcell_id;
private: std::shared_ptr<Context> _ctx;
private: OnTaskProgressCb _on_task_progress;
private: std::shared_ptr<WorkcellSession> _session;
};

}

#endif
