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
  std::unordered_map<std::string, std::string> input_item_to_station_map;
  for (const auto& item : workcell_task.inputs)
  {
    input_item_to_station_map[item.item_id] = item.station_id;
  }

  std::unordered_map<std::string, std::string> output_item_to_station_map;
  for (const auto& item : workcell_task.outputs)
  {
    output_item_to_station_map[item.item_id] = item.station_id;
  }

  return Task{
    workcell_task.work_order_id,
    workcell_task.task_id,
    workcell_task.type,
    std::move(input_item_to_station_map),
    std::move(output_item_to_station_map),
    YAML::Load(workcell_task.payload),
    YAML::Load(workcell_task.previous_results),
  };
}

} // namespace nexus::workcell_orchestrator
