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

#ifndef NEXUS_COMMON__MODELS__WORK_ORDER_HPP
#define NEXUS_COMMON__MODELS__WORK_ORDER_HPP

#include <nexus_common/yaml_helpers.hpp>
#include <nexus_common/models/item.hpp>
#include <nexus_common/models/parameter.hpp>
#include <nexus_common/models/step.hpp>

namespace nexus::common {

struct WorkOrder
{
  YAML::Node yaml;

  WorkOrder(YAML::Node yaml)
  : yaml(std::move(yaml)) {}

  WorkOrder() {}

  std::string work_instruction_name() const
  {
    return this->yaml["workInstructionName"].as<std::string>();
  }

  std::vector<Step> steps() const
  {
    return this->yaml["steps"].as<std::vector<Step>>();
  }
};

} // namespace nexus::common

namespace YAML {

template<>
struct convert<nexus::common::WorkOrder>
{
  static Node encode(const nexus::common::WorkOrder& data)
  {
    return data.yaml;
  }

  static bool decode(const Node& node, nexus::common::WorkOrder& data)
  {
    data.yaml = node;
    return true;
  }
};

} // namespace YAML

#endif // NEXUS_COMMON__MODELS__WORK_ORDER_HPP
