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
    workcell_task.type,
    YAML::Load(workcell_task.payload),
    YAML::Load(workcell_task.previous_results),
  };
}

} // namespace nexus::workcell_orchestrator
