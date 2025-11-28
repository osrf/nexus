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

#ifndef NEXUS_CAPABILITIES__CONTEXT_HPP
#define NEXUS_CAPABILITIES__CONTEXT_HPP

#include "task.hpp"

#include <nexus_common/logging.hpp>
#include <nexus_endpoints.hpp>
#include <nexus_orchestrator_msgs/msg/task_state.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <rclcpp/logger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <yaml-cpp/yaml.h>

#include <functional>
#include <istream>
#include <unordered_map>

//==============================================================================
namespace nexus {

class Context : std::enable_shared_from_this<Context>
{
private: struct Private{ explicit Private() = default; };
public: using GoalHandlePtr =
    std::shared_ptr<rclcpp_action::ServerGoalHandle<endpoints::WorkcellRequestAction::ActionType>>;
public: using TaskState = nexus_orchestrator_msgs::msg::TaskState;

  // using reference to prevent circular references
public: rclcpp_lifecycle::LifecycleNode& node;
public: BT::Tree bt;
public: GoalHandlePtr goal_handle;
public: Task task;
public: std::vector<std::string> errors;
public: std::unique_ptr<common::BtLogging> bt_logging;
public: std::unordered_set<std::string> signals;
private: uint8_t task_status = TaskState::STATUS_NONE;
public: uint64_t tick_count = 0;
// Used to keep track of the last time the state was changed
public: rclcpp::Time state_transition_time;

public: Context(Private, rclcpp_lifecycle::LifecycleNode& node)
  : node(node), state_transition_time(node.now()) {}
public: static std::shared_ptr<Context> make(rclcpp_lifecycle::LifecycleNode& node) {
    Private pvt;
    return std::make_shared<Context>(pvt, node);
}

public: std::shared_ptr<Context> get_ptr() {
  return shared_from_this();
}

public: void set_task_status(uint8_t status)
{
  this->task_status = status;
  this->state_transition_time = node.now();
}

public: TaskState get_task_state() const
{
  TaskState task_state;
  task_state.task_id = this->task.task_id;
  task_state.workcell_id = this->node.get_name();
  task_state.status = this->task_status;
  return task_state;
}

// bool is C1 < C2, smallest gets on top
public: static bool compare(const std::shared_ptr<Context> c1, const std::shared_ptr<Context> c2) {
  using TaskState = nexus_orchestrator_msgs::msg::TaskState;
  const auto s1 = c1->task_status;
  const auto s2 = c2->task_status;
  if (s1 == s2)
  {
    // For equal task state, prioritize earlier transition time to make sure
    // tasks that started earlier are first in the queue
    return c1->state_transition_time < c2->state_transition_time;
  }
  // An executing task will always have priority over a non executing task
  if (s1 == TaskState::STATUS_RUNNING)
  {
    return true;
  }
  if (s1 == TaskState::STATUS_QUEUED)
  {
    if (s2 == TaskState::STATUS_RUNNING)
    {
      // Prioritize the running task
      return false;
    }
    // Prioritize queued task
    return true;
  }
  if (s1 == TaskState::STATUS_FINISHED || s1 == TaskState::STATUS_FAILED)
  {
    if (s2 == TaskState::STATUS_FINISHED || s2 == TaskState::STATUS_FAILED)
    {
      // Sort by time
      return c1->state_transition_time < c2->state_transition_time;
    }
    // Push to end of queue
    return false;
  }
  if (s1 == TaskState::STATUS_ASSIGNED)
  {
    if (s2 == TaskState::STATUS_RUNNING || s2 == TaskState::STATUS_QUEUED)
    {
      // Prioritize tasks that have been requested or are running already
      return false;
    }
    return true;
  }
  // Undefined, just sort in order of transition time
  return c1->state_transition_time < c2->state_transition_time;
}
};

class ContextSet {
private:
  std::vector<std::shared_ptr<Context>> set;
  // TODO(luca) consider map of task_id to iterator + change the above to a list

  void reorder() {
    std::sort(set.begin(), set.end(), [](const std::shared_ptr<Context>& ctx1, const std::shared_ptr<Context>& ctx2) {
      return Context::compare(ctx1, ctx2);
    });
  }

public:
  ContextSet() = default;

  std::optional<std::shared_ptr<const Context>> get_at(const std::size_t n) const {
    if (n >= this->size()) {
      return std::nullopt;
    }
    return set[n];
  }

  std::optional<std::shared_ptr<Context>> get_task_id(const std::string& task_id) const {
    for (auto it = set.begin(); it != set.end(); ++it) {
      if ((*it)->task.task_id == task_id) {
        return *it;
      }
    }
    return std::nullopt;
  }

  bool modify_at(const std::size_t n, std::function<void(Context&)> modifier) {
    if (n >= this->size()) {
      return false;
    }
    modifier(*set[n]);
    this->reorder();
    return true;
  }

  bool modify_task_id(const std::string& task_id, std::function<void(Context&)> modifier) {
    for (auto it = set.begin(); it != set.end(); ++it) {
      if ((*it)->task.task_id == task_id) {
        modifier(**it);
        this->reorder();
        return true;
      }
    }
    return false;
  }

  std::size_t size() const {
    return set.size();
  }

  bool remove_at(const std::size_t n) {
    if (n >= this->size()) {
      return false;
    }
    set.erase(set.begin() + n);
    return true;
  }

  bool remove(const std::shared_ptr<Context>& ptr) {
    for (auto it = set.begin(); it != set.end(); ++it) {
      if ((*it) == ptr) {
        set.erase(it);
        return true;
      }
    }
    return false;
  }

  bool remove_task_id(const std::string& task_id) {
    for (auto it = set.begin(); it != set.end(); ++it) {
      if ((*it)->task.task_id == task_id) {
        set.erase(it);
        return true;
      }
    }
    return false;
  }

  bool insert(const std::shared_ptr<Context> ptr) {
    const auto it = get_task_id(ptr->task.task_id);
    if (it != std::nullopt) {
      return false;
    }
    set.emplace_back(std::move(ptr));
    this->reorder();
    return true;
  }

  void clear() {
    set.clear();
  }

};

} // namespace nexus

#endif // NEXUS_CAPABILITIES__CONTEXT_HPP
