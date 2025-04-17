/*
 * Copyright (C) 2025 Open Source Robotics Foundation.
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

#ifndef NEXUS_COMMON__MODELS__STEP_HPP
#define NEXUS_COMMON__MODELS__STEP_HPP

#include <nexus_common/models/item.hpp>
#include <nexus_common/models/parameter.hpp>
#include <nexus_common/yaml_helpers.hpp>

namespace nexus::common {

struct Step
{
  YAML::Node yaml;

  Step(YAML::Node yaml)
  : yaml(std::move(yaml)) {}

  Step() {}

  std::string process_id() const
  {
    return this->yaml["processId"].as<std::string>();
  }

  std::string name() const
  {
    return this->yaml["name"].as<std::string>();
  }

  std::vector<Item> input_items() const
  {
    return this->yaml["inputItems"].as<std::vector<Item>>();
  }

  std::vector<Item> output_items() const
  {
    return this->yaml["outputItems"].as<std::vector<Item>>();
  }

  std::vector<Parameter> parameters() const
  {
    return this->yaml["parameters"].as<std::vector<Parameter>>();
  }
};

} // namespace nexus::common

namespace YAML {

template<>
struct convert<nexus::common::Step>
{
  static Node encode(const nexus::common::Step& data)
  {
    return data.yaml;
  }

  static bool decode(const Node& node, nexus::common::Step& data)
  {
    data.yaml = node;
    return true;
  }
};

} // namespace YAML

#endif // NEXUS_COMMON__MODELS__STEP_HPP
