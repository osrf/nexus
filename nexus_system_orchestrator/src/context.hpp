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

#include <nexus_common/task_remapper.hpp>
#include <nexus_common/models/work_order.hpp>
#include <nexus_endpoints.hpp>
#include <nexus_orchestrator_msgs/msg/task_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>
#include <memory>
#include <string>

namespace nexus::system_orchestrator {

class Context
{
public:
  using WorkOrder = common::WorkOrder;
  using WorkOrderGoalHandle =
    rclcpp_action::ServerGoalHandle<nexus::endpoints::WorkOrderAction::ActionType>;
  using WorkcellTask = nexus_orchestrator_msgs::msg::WorkcellTask;
  using TaskState = nexus_orchestrator_msgs::msg::TaskState;

  explicit Context(rclcpp_lifecycle::LifecycleNode& node);

  rclcpp_lifecycle::LifecycleNode& get_node() const;

  Context& set_job_id(const std::string& job_id);

  std::string get_job_id() const;

  Context& set_work_order(const WorkOrder& wo);

  WorkOrder get_work_order() const;

  Context& set_tasks(const std::vector<WorkcellTask>& tasks);

  std::vector<WorkcellTask> get_tasks() const;

  std::size_t get_num_tasks() const;

  Context& set_task_remapper(
    const std::shared_ptr<const common::TaskRemapper>& remapper);

  std::shared_ptr<const common::TaskRemapper> get_task_remapper() const;

  Context& set_workcell_task_assignment(
    const std::string& task_id,
    const std::string& workcell_id);

  std::unordered_map<std::string, std::string> get_workcell_task_assignments()
    const;

  std::optional<std::string> get_workcell_task_assignment(
    const std::string& workcell_task_id) const;

  Context& set_workcell_sessions(
    const std::unordered_map<std::string,
    std::shared_ptr<WorkcellSession>>& sessions);

  std::unordered_map<std::string, std::shared_ptr<WorkcellSession>>
    get_workcell_sessions() const;

  std::shared_ptr<WorkcellSession> get_workcell_session(
    const std::string& workcell_id) const;

  Context& set_transporter_sessions(
    const std::unordered_map<std::string,
    std::shared_ptr<TransporterSession>>& sessions);

  std::unordered_map<std::string, std::shared_ptr<TransporterSession>>
    get_transporter_sessions() const;

  std::shared_ptr<TransporterSession> get_transporter_session(
    const std::string& transporter_id) const;

  Context& set_task_state(const std::string& task_id, const TaskState& task_state);

  std::unordered_map<std::string, TaskState> get_task_states() const;

  std::optional<TaskState> get_task_state(const std::string& task_id) const;

  Context& set_goal_handle(
    const std::shared_ptr<WorkOrderGoalHandle>& handle);

  std::shared_ptr<WorkOrderGoalHandle> get_goal_handle() const;

  Context& add_error(const std::string& error);

  std::vector<std::string> get_errors() const;

  Context& set_task_results(const std::string& results);

  std::string get_task_results() const;

  Context& add_queued_signal(const std::string& task_id, const std::string& signal);

  Context& set_task_queued_signals(
    const std::string& task_id,
    const std::vector<std::string>& signals);

  std::optional<std::vector<std::string>> get_task_queued_signals(
    const std::string& task_id) const;

private:
  // using reference to prevent circular references
  rclcpp_lifecycle::LifecycleNode& _node;
  std::string _job_id;
  WorkOrder _wo;
  std::vector<WorkcellTask> _tasks;
  std::shared_ptr<const common::TaskRemapper> _task_remapper;

  /**
   * Map of task ids and their assigned workcell ids.
   */
  std::unordered_map<std::string, std::string> _workcell_task_assignments = {};

  /**
   * Map of workcell ids and it's session.
   */
  std::unordered_map<std::string,
  std::shared_ptr<WorkcellSession>> _workcell_sessions;

  /**
   * Map of transporter ids and it's session.
   */
  std::unordered_map<std::string,
  std::shared_ptr<TransporterSession>> _transporter_sessions;

  std::unordered_map<std::string, TaskState> _task_states = {};
  std::shared_ptr<WorkOrderGoalHandle> _goal_handle = nullptr;
  std::vector<std::string> _errors = {};
  std::string _task_results;

  /**
   * Map of workcell task ids and their queued signals.
   */
  std::unordered_map<std::string, std::vector<std::string>> _queued_signals;

  mutable std::mutex _mutex;
};

} // namespace nexus::system_orchestrator

#endif // NEXUS_SYSTEM_ORCHESTRATOR__CONTEXT_HPP
