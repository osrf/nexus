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

#ifndef NEXUS_CAPABILITITY_PLUGIN__TASK_PARSER_HPP
#define NEXUS_CAPABILITITY_PLUGIN__TASK_PARSER_HPP

#include <nexus_capabilities/task.hpp>
#include <nexus_orchestrator_msgs/msg/workcell_task.hpp>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

//==============================================================================
namespace nexus::workcell_orchestrator {

class TaskParser
{
  // Default constructor
public: TaskParser() {}

  /**
   * Parses a workcell task.
   * @throws YAML::Exception if parsing the payload fails.
   */
public: Task parse_task(const nexus_orchestrator_msgs::msg::WorkcellTask& task);

  /**
   * Add task type to remap to.
   */
public: void add_remap_task_type(const std::string& remap_from_type,
    const std::string& remap_to_type);

  /**
   * Remaps task types if a remap entry is found for the given type.
   */
public: std::string remap_task_type(const std::string& task_type);

private: std::unordered_map<std::string, std::string> _remap_task_types;
};

} // namespace nexus::workcell_orchestrator

#endif
