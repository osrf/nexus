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

#include <yaml-cpp/yaml.h>

#include "add_metadata.hpp"

namespace nexus::system_orchestrator {

BT::NodeStatus AddMetadata::tick()
{
  auto tasks = this->_ctx->get_tasks();

  YAML::Node workcell_assignments = YAML::Load("[]");
  for (std::size_t i = 0; i < tasks.size(); ++i)
  {
    const std::string& workcell_task_id = tasks.at(i).task_id;
    auto assigned_workcell_id = this->_ctx->get_workcell_task_assignment(
      workcell_task_id);
    if (!assigned_workcell_id.has_value())
    {
      RCLCPP_ERROR(
        this->_ctx->get_node().get_logger(),
        "Workcell task [%s] does not have a workcell assigned",
        workcell_task_id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    workcell_assignments.push_back(assigned_workcell_id.value());
  }

  YAML::Node assignment_result;
  assignment_result["workcell_assignments"] = workcell_assignments;
  assignment_result["current_index"] = 0;

  for (std::size_t i = 0; i < tasks.size(); ++i)
  {
    auto& task = tasks.at(i);
    // TODO debug
    RCLCPP_INFO(
      this->_ctx->get_node().get_logger(),
      "payload of [%s] before:\n%s",
      task.task_id.c_str(), task.payload.c_str());

    YAML::Node payload;
    try
    {
      payload = YAML::Load(task.payload);
    }
    catch (YAML::ParserException& e)
    {
      RCLCPP_ERROR(
        this->_ctx->get_node().get_logger(),
        "Failed to parse payload for workcell task [%s]: %s",
        task.task_id.c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }
    YAML::Node metadata = payload["metadata"];
    if (!metadata.IsMap())
    {
      RCLCPP_ERROR(
        this->_ctx->get_node().get_logger(),
        "Metadata of payload of workcell task [%s] is not a map",
        task.task_id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    assignment_result["current_index"] = static_cast<int>(i);
    metadata["assignment_result"] = assignment_result;

    YAML::Emitter out;
    out << payload;
    task.payload = out.c_str();

    // TODO debug
    RCLCPP_INFO(
      this->_ctx->get_node().get_logger(),
      "payload of [%s] after:\n%s",
      task.task_id.c_str(), task.payload.c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nexus::system_orchestrator
