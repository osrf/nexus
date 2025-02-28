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

namespace nexus::system_orchestrator {

class Context
{
public:
  using WorkOrder = common::WorkOrder;
  using WorkOrderGoalHandle =
    rclcpp_action::ServerGoalHandle<nexus::endpoints::WorkOrderAction::ActionType>;
  using WorkcellTask = nexus_orchestrator_msgs::msg::WorkcellTask;
  using TaskState = nexus_orchestrator_msgs::msg::TaskState;

  Context(rclcpp_lifecycle::LifecycleNode& node) : _node(node) {}

  rclcpp_lifecycle::LifecycleNode& get_node() const
  {
    std::lock_guard<std::mutex> lock(_node_mutex);
    return _node;
  }

  Context& set_job_id(const std::string& job_id)
  {
    std::lock_guard<std::mutex> lock(_job_id_mutex);
    _job_id = job_id;
    return *this;
  }

  std::string get_job_id() const
  {
    std::lock_guard<std::mutex> lock(_job_id_mutex);
    return _job_id;
  }

  Context& set_work_order(const WorkOrder& wo)
  {
    std::lock_guard<std::mutex> lock(_wo_mutex);
    _wo = wo;
    return *this;
  }

  WorkOrder get_work_order() const
  {
    std::lock_guard<std::mutex> lock(_wo_mutex);
    return _wo;
  }

  Context& set_tasks(const std::vector<WorkcellTask>& tasks)
  {
    std::lock_guard<std::mutex> lock(_tasks_mutex);
    _tasks = tasks;
    return *this;
  }

  std::vector<WorkcellTask> get_tasks() const
  {
    std::lock_guard<std::mutex> lock(_tasks_mutex);
    return _tasks;
  }

  std::size_t get_task_number() const
  {
    std::lock_guard<std::mutex> lock(_tasks_mutex);
    return _tasks.size();
  }

  Context& set_task_remapper(const std::shared_ptr<const common::TaskRemapper>& remapper)
  {
    std::lock_guard<std::mutex> lock(_task_remapper_mutex);
    _task_remapper = remapper;
    return *this;
  }

  std::shared_ptr<const common::TaskRemapper> get_task_remapper() const
  {
    std::lock_guard<std::mutex> lock(_task_remapper_mutex);
    return _task_remapper;
  }

  Context& set_workcell_task_assignments(const std::unordered_map<std::string, std::string>& assignments)
  {
    std::lock_guard<std::mutex> lock(_workcell_task_assignments_mutex);
    _workcell_task_assignments = assignments;
    return *this;
  }

  Context& set_workcell_task_assignment(const std::string& task_id, const std::string& workcell_id)
  {
    std::lock_guard<std::mutex> lock(_workcell_task_assignments_mutex);
    _workcell_task_assignments.emplace(task_id, workcell_id);
    return *this;
  }

  std::unordered_map<std::string, std::string> get_workcell_task_assignments() const
  {
    std::lock_guard<std::mutex> lock(_workcell_task_assignments_mutex);
    return _workcell_task_assignments;
  }

  Context& set_workcell_sessions(const std::unordered_map<std::string, std::shared_ptr<WorkcellSession>>& sessions)
  {
    std::lock_guard<std::mutex> lock(_workcell_sessions_mutex);
    _workcell_sessions = sessions;
    return *this;
  }

  std::unordered_map<std::string, std::shared_ptr<WorkcellSession>> get_workcell_sessions() const
  {
    std::lock_guard<std::mutex> lock(_workcell_sessions_mutex);
    return _workcell_sessions;
  }

  Context& set_transporter_sessions(const std::unordered_map<std::string, std::shared_ptr<TransporterSession>>& sessions)
  {
    std::lock_guard<std::mutex> lock(_transporter_sessions_mutex);
    _transporter_sessions = sessions;
    return *this;
  }

  std::unordered_map<std::string, std::shared_ptr<TransporterSession>> get_transporter_sessions() const
  {
    std::lock_guard<std::mutex> lock(_transporter_sessions_mutex);
    return _transporter_sessions;
  }

  Context& set_task_states(const std::unordered_map<std::string, TaskState>& states)
  {
    std::lock_guard<std::mutex> lock(_task_states_mutex);
    _task_states = states;
    return *this;
  }

  Context& set_task_state(const std::string& task_id, const TaskState& task_state)
  {
    std::lock_guard<std::mutex> lock(_task_states_mutex);
    _task_states.emplace(task_id, task_state);
    return *this;
  }

  std::unordered_map<std::string, TaskState> get_task_states() const
  {
    std::lock_guard<std::mutex> lock(_task_states_mutex);
    return _task_states;
  }

  Context& set_goal_handle(const std::shared_ptr<WorkOrderGoalHandle>& handle)
  {
    std::lock_guard<std::mutex> lock(_goal_handle_mutex);
    _goal_handle = handle;
    return *this;
  }

  std::shared_ptr<WorkOrderGoalHandle> get_goal_handle() const
  {
    std::lock_guard<std::mutex> lock(_goal_handle_mutex);
    return _goal_handle;
  }

  Context& set_errors(const std::vector<std::string>& errors)
  {
    std::lock_guard<std::mutex> lock(_errors_mutex);
    _errors = errors;
    return *this;
  }

  Context& add_error(const std::string& error)
  {
    std::lock_guard<std::mutex> lock(_errors_mutex);
    _errors.emplace_back(error);
    return *this;
  }

  std::vector<std::string> get_errors() const
  {
    std::lock_guard<std::mutex> lock(_errors_mutex);
    return _errors;
  }

  Context& set_task_results(const std::string& results)
  {
    std::lock_guard<std::mutex> lock(_task_results_mutex);
    _task_results = results;
    return *this;
  }

  std::string get_task_results() const
  {
    std::lock_guard<std::mutex> lock(_task_results_mutex);
    return _task_results;
  }

  Context& set_queued_signals(const std::unordered_map<std::string, std::vector<std::string>>& signals)
  {
    std::lock_guard<std::mutex> lock(_queued_signals_mutex);
    _queued_signals = signals;
    return *this;
  }

  Context& add_queued_signal(const std::string& task_id, const std::string& signal) {
    std::lock_guard<std::mutex> lock(_queued_signals_mutex);
    _queued_signals[task_id].emplace_back(signal);
    return *this;
  }

  std::unordered_map<std::string, std::vector<std::string>> get_queued_signals() const
  {
    std::lock_guard<std::mutex> lock(_queued_signals_mutex);
    return _queued_signals;
  }

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

  mutable std::mutex _node_mutex;
  mutable std::mutex _job_id_mutex;
  mutable std::mutex _wo_mutex;
  mutable std::mutex _tasks_mutex;
  mutable std::mutex _task_remapper_mutex;
  mutable std::mutex _workcell_task_assignments_mutex;
  mutable std::mutex _workcell_sessions_mutex;
  mutable std::mutex _transporter_sessions_mutex;
  mutable std::mutex _task_states_mutex;
  mutable std::mutex _goal_handle_mutex;
  mutable std::mutex _errors_mutex;
  mutable std::mutex _task_results_mutex;
  mutable std::mutex _queued_signals_mutex;
};

}

#endif
