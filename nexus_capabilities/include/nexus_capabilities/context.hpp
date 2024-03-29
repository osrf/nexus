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

class Context
{
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
public: TaskState task_state;
public: uint64_t tick_count = 0;

public: Context(rclcpp_lifecycle::LifecycleNode& node)
  : node(node) {}
};

} // namespace nexus

#endif // NEXUS_CAPABILITIES__CONTEXT_HPP
