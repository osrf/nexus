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

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__TASK_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__TASK_HPP

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

//==============================================================================
namespace nexus {

using TaskData = YAML::Node;

struct Task
{
public: std::string work_order_id;
public: std::string task_id;
public: std::string type;
public: std::unordered_map<std::string, std::string> input_item_to_station_map;
public: std::unordered_map<std::string, std::string> output_item_to_station_map;
public: TaskData data;
public: YAML::Node previous_results;
};

} // namespace nexus

#endif // NEXUS_WORKCELL_ORCHESTRATOR__TASK_HPP
