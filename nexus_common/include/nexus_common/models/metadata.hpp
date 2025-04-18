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

#ifndef NEXUS_COMMON__MODELS__METADATA_HPP
#define NEXUS_COMMON__MODELS__METADATA_HPP

#include <optional>

#include "../yaml_helpers.hpp"

namespace nexus::common {

class MetaData
{
public:
  YAML::Node yaml;

  MetaData(YAML::Node yaml)
  : yaml(std::move(yaml)) {}

  MetaData() {}
};

}  // namespace nexus::common

namespace YAML {

template<>
struct convert<nexus::common::MetaData>
{
  static Node encode(const nexus::common::MetaData& data)
  {
    return data.yaml;
  }

  static bool decode(const Node& node, nexus::common::MetaData& data)
  {
    data.yaml = node;
    return true;
  }
};

}  // namespace YAML

#endif  // NEXUS_COMMON__MODELS__METADATA_HPP
