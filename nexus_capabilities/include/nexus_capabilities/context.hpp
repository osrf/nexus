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

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

//==============================================================================
namespace nexus {

class Context
{
  /**
   * There are several choices we can use here, each having their own pros and cons.
   *
   * shared_ptr:
   *   + Api friendly
   *   - Will cause circular references
   *   - Cannot create an instance in the node constructor (cannot call `shared_from_this`
   *     in constructor)
   *
   * weak_ptr:
   *   - Will almost always cause circular reference. Most ROS apis don't support weak_ptr,
   *     attempting to `lock()` and passing the `shared_ptr` will almost always cause a
   *     circular reference.
   *   - Cannot create create an instance in the node constructor (cannot call `weak_from_this`
   *     in constructor)
   *
   * raw_ptr:
   *   + Api friendly
   *   - Unsafe
   *
   * reference:
   *   - Unsafe. Reference does not protect from UAF, also most ROS apis don't support references,
   *     those that use templates may work with either shared or raw pointers.
   *     We can't get a shared_ptr from a reference so we may end up using raw pointers.
   *
   * `shared_ptr` is chosen due to api friendliness, the consequences of circular reference
   * is negligible since the node should never go out of scope.
   */
public: std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
public: Task task;
public: std::vector<std::string> errors;
public: std::unordered_set<std::string> signals;

public: Context(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
  : node(std::move(node)) {}
};

} // namespace nexus

#endif // NEXUS_CAPABILITIES__CONTEXT_HPP
