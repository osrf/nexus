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

#ifndef NEXUS_COMMON__MODELS__PARAMETER_HPP
#define NEXUS_COMMON__MODELS__PARAMETER_HPP

#include <nexus_common/yaml_helpers.hpp>

namespace nexus::common {

struct Parameter
{
  YAML::Node yaml;

  Parameter(YAML::Node yaml)
  : yaml(std::move(yaml)) {}

  Parameter() {}

  std::string name() const
  {
    return this->yaml["name"].as<std::string>();
  }

  std::string string_value() const
  {
    return this->yaml["stringValue"].as<std::string>();
  }

  int int_value() const
  {
    return this->yaml["intValue"].as<int>();
  }

  double double_value() const
  {
    return this->yaml["doubleValue"].as<double>();
  }

  bool bool_value() const
  {
    return this->yaml["boolValue"].as<bool>();
  }

  int enumeration_value() const
  {
    return this->yaml["enumerationValue"].as<int>();
  }
};

} // namespace nexus::common

namespace YAML {

template<>
struct convert<nexus::common::Parameter>
{
  static Node encode(const nexus::common::Parameter& data)
  {
    return data.yaml;
  }

  static bool decode(const Node& node, nexus::common::Parameter& data)
  {
    data.yaml = node;
    return true;
  }
};

} // namespace YAML

#endif // NEXUS_COMMON__MODELS__PARAMETER_HPP