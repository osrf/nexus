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

#ifndef NEXUS_SYSTEM_ORCHESTRATOR__CONTEXT_HPP
#define NEXUS_SYSTEM_ORCHESTRATOR__CONTEXT_HPP

#include "session.hpp"

#include <nexus_common/models/work_order.hpp>
#include <nexus_endpoints.hpp>
#include <nexus_orchestrator_msgs/msg/task_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace nexus::system_orchestrator {

class Context
{
public: using WorkOrder = common::WorkOrder;
public: using WorkOrderGoalHandle =
    rclcpp_action::ServerGoalHandle<nexus::endpoints::WorkOrderAction::ActionType>;
public: using WorkcellTask = nexus_orchestrator_msgs::msg::WorkcellTask;
public: using TaskState = nexus_orchestrator_msgs::msg::TaskState;

  // using reference to prevent circular references
public: rclcpp_lifecycle::LifecycleNode& node;
public: std::string job_id;
public: WorkOrder wo;
public: std::vector<WorkcellTask> tasks;
  /**
   * Map of task ids and their assigned workcell ids.
   */
public: std::unordered_map<std::string, std::string> workcell_task_assignments;

  /**
   * Map of workcell ids and it's session.
   */
public: std::unordered_map<std::string,
    std::shared_ptr<WorkcellSession>> workcell_sessions;
  /**
   * Map of transporter ids and it's session.
   */
public: std::unordered_map<std::string,
    std::shared_ptr<TransporterSession>> transporter_sessions;
public: std::unordered_map<std::string, TaskState> task_states;
public: std::shared_ptr<WorkOrderGoalHandle> goal_handle;
public: std::vector<std::string> errors;
public: std::string task_results;

  /**
   * Map of task ids and their queued signals.
   */
public: std::unordered_map<std::string,
    std::vector<std::string>> queued_signals;
};

}

#endif
