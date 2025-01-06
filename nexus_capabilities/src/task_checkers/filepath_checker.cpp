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

#include "nexus_capabilities/task_checker.hpp"

#include "nexus_common/bt_store.hpp"

#include <stdexcept>
#include <string>

namespace nexus::task_checkers {

/**
 * A task a doable if a BT with the same name as the task type exists in the local filesystem.
 */
///=============================================================================
class FilepathChecker : public TaskChecker
{
public:
  /**
   * @copydoc TaskChecker::initialize
   */
  void initialize(NodeInterfaces interfaces)
  {
    auto param_interface = interfaces.get<rclcpp::node_interfaces::NodeParametersInterface>();
    // We assume that the parameter is already declared.
    const std::string bt_path = param_interface->get_parameter("bt_path").get_value<std::string>();
    this->_bt_store.register_from_path(bt_path);
  }

  /**
   * @copydoc TaskChecker::is_task_doable
   */
  bool is_task_doable(const Task & task)
  {
    try
    {
      this->_bt_store.get_bt(task.type);
      return true;
    }
    catch (const std::out_of_range&)
    {
      return false;
    }
  }

private:
  common::BTStore _bt_store;
};

} // namespace nexus::task_checkers

///=============================================================================
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nexus::task_checkers::FilepathChecker,
  nexus::TaskChecker
)