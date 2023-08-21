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

#include "task_parser.hpp"

#include <functional>
#include <unordered_map>
#include <unordered_set>

namespace nexus::workcell_orchestrator {

//==============================================================================
Task TaskParser::parse_task(
  const nexus_orchestrator_msgs::msg::WorkcellTask& workcell_task)
{
  return Task{
    workcell_task.id,
    this->remap_task_type(workcell_task.type),
    YAML::Load(workcell_task.payload),
    YAML::Load(workcell_task.previous_results),
  };
}

//==============================================================================
void TaskParser::add_remap_task_type(const std::string& remap_from_type,
  const std::string& remap_to_type)
{
  this->_remap_task_types[remap_from_type] = remap_to_type;
}

//==============================================================================
std::string TaskParser::remap_task_type(const std::string& task_type)
{
  auto remap_it = this->_remap_task_types.find(task_type);

  if (remap_it != this->_remap_task_types.end())
  {
    return remap_it->second;
  }
  return task_type;
}

} // namespace nexus::workcell_orchestrator
