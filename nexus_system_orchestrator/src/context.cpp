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

#include "context.hpp"

namespace nexus::system_orchestrator {

Context::Context(rclcpp_lifecycle::LifecycleNode& node) : _node(node) {}

rclcpp_lifecycle::LifecycleNode& Context::get_node() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _node;
}

Context& Context::set_job_id(const std::string& job_id)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _job_id = job_id;
  return *this;
}

std::string Context::get_job_id() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _job_id;
}

Context& Context::set_work_order(const Context::WorkOrder& wo)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _wo = wo;
  return *this;
}

Context::WorkOrder Context::get_work_order() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _wo;
}

Context& Context::set_tasks(const std::vector<Context::WorkcellTask>& tasks)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _tasks = tasks;
  return *this;
}

std::vector<Context::WorkcellTask> Context::get_tasks() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _tasks;
}

std::size_t Context::get_num_tasks() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _tasks.size();
}

Context& Context::set_task_remapper(
  const std::shared_ptr<const common::TaskRemapper>& remapper)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _task_remapper = remapper;
  return *this;
}

std::shared_ptr<const common::TaskRemapper> Context::get_task_remapper() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _task_remapper;
}

Context& Context::set_workcell_task_assignment(
  const std::string& task_id, const std::string& workcell_id)
{
  std::lock_guard<std::mutex> lock(_mutex);
  if (_workcell_task_assignments.count(task_id) > 0)
  {
    RCLCPP_WARN(
      _node.get_logger(),
      "[Context::set_workcell_task_assignment] task_id %s previously "
      "assigned to workcell %s is now reassigned to workcell %s",
      task_id.c_str(),
      _workcell_task_assignments[task_id].c_str(),
      workcell_id.c_str()
    );
  }
  _workcell_task_assignments[task_id] = workcell_id;
  return *this;
}

std::unordered_map<std::string, std::string>
Context::get_workcell_task_assignments() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _workcell_task_assignments;
}

std::optional<std::string> Context::get_workcell_task_assignment(
  const std::string& workcell_task_id) const
{
  std::lock_guard<std::mutex> lock(_mutex);
  auto assignment_it = _workcell_task_assignments.find(workcell_task_id);
  if (assignment_it == _workcell_task_assignments.end())
  {
    return std::nullopt;
  }
  return assignment_it->second;
}

Context& Context::set_workcell_sessions(
  const std::unordered_map<std::string, std::shared_ptr<WorkcellSession>>& sessions)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _workcell_sessions = sessions;
  return *this;
}

std::unordered_map<std::string, std::shared_ptr<WorkcellSession>> Context::get_workcell_sessions() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _workcell_sessions;
}

std::shared_ptr<WorkcellSession> Context::get_workcell_session(
  const std::string& workcell_id) const
{
  std::lock_guard<std::mutex> lock(_mutex);
  auto workcell_session_it = _workcell_sessions.find(workcell_id);
  if (workcell_session_it == _workcell_sessions.end())
  {
    return nullptr;
  }
  return workcell_session_it->second;
}

Context& Context::set_transporter_sessions(
  const std::unordered_map<std::string, std::shared_ptr<TransporterSession>>& sessions)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _transporter_sessions = sessions;
  return *this;
}

std::unordered_map<std::string, std::shared_ptr<TransporterSession>>
Context::get_transporter_sessions() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _transporter_sessions;
}

std::shared_ptr<TransporterSession> Context::get_transporter_session(
  const std::string& transporter_id) const
{
  std::lock_guard<std::mutex> lock(_mutex);
  auto transporter_session_it = _transporter_sessions.find(transporter_id);
  if (transporter_session_it == _transporter_sessions.end())
  {
    return nullptr;
  }
  return transporter_session_it->second;
}

Context& Context::set_task_state(
  const std::string& task_id, const Context::TaskState& task_state)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _task_states[task_id] = task_state;
  return *this;
}

std::unordered_map<std::string, Context::TaskState>
Context::get_task_states() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _task_states;
}

std::optional<Context::TaskState> Context::get_task_state(
  const std::string& task_id) const
{
  std::lock_guard<std::mutex> lock(_mutex);
  const auto it = _task_states.find(task_id);
  if (it == _task_states.end())
  {
    return std::nullopt;
  }
  return it->second;
}

Context& Context::set_goal_handle(
  const std::shared_ptr<WorkOrderGoalHandle>& handle)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _goal_handle = handle;
  return *this;
}

std::shared_ptr<Context::WorkOrderGoalHandle> Context::get_goal_handle() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _goal_handle;
}

Context& Context::add_error(const std::string& error)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _errors.emplace_back(error);
  return *this;
}

std::vector<std::string> Context::get_errors() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _errors;
}

Context& Context::set_task_results(const std::string& results)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _task_results = results;
  return *this;
}

std::string Context::get_task_results() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _task_results;
}

Context& Context::add_queued_signal(
  const std::string& task_id, const std::string& signal)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _queued_signals[task_id].emplace_back(signal);
  return *this;
}

Context& Context::set_task_queued_signals(
  const std::string& task_id, const std::vector<std::string>& signals)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _queued_signals[task_id] = signals;
  return *this;
}

std::optional<std::vector<std::string>> Context::get_task_queued_signals(
  const std::string& task_id) const
{
  std::lock_guard<std::mutex> lock(_mutex);
  const auto it = _queued_signals.find(task_id);
  if (it == _queued_signals.end())
  {
    return std::nullopt;
  }
  return it->second;
}

void Context::set_sku_location(const WorkcellTask& task, const std::string& workcell)
{
  if (task.output_item.size() > 0)
  {
    this->sku_locations[task.output_item] = workcell;
  }
}

std::optional<std::string> Context::get_sku_location(const std::string& sku) const
{
  const auto it = this->sku_locations.find(sku);
  if (it != this->sku_locations.end())
  {
    return it->second;
  }
  return std::nullopt;
}

} // namespace nexus::system_orchestrator
