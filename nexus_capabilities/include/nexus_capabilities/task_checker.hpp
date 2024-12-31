/*
 * Copyright (C) 2024 Open Source Robotics Foundation.
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

#ifndef NEXUS_CAPABILITIES__TASK_CHECKER_HPP
#define NEXUS_CAPABILITIES__TASK_CHECKER_HPP

#include "nexus_capabilities/task.hpp"

#include "rclcpp/node_interfaces/node_interfaces.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

namespace nexus {
//==============================================================================
/**
 * An implementation of this class can be used to verify a task can be performed.
 */
class TaskChecker
{
public:
  using NodeInterfaces = rclcpp::node_interfaces::NodeInterfaces<ALL_RCLCPP_NODE_INTERFACES>;

  /// @brief Initialize the task checker.
  /// @param interfaces Node interfaces that may be used for initialization.
  virtual void initialize(NodeInterfaces interfaces) = 0;

  /// @brief Check if a task request is doable.
  /// @param req The requested task.
  virtual bool is_task_doable(const Task& task) = 0;

  virtual ~TaskChecker() = default;
};

} // namespace nexus

#endif // NEXUS_CAPABILITIES__TASK_CHECKER_HPP
