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

#ifndef NEXUS_COMMON__MODELS__ITEM_HPP
#define NEXUS_COMMON__MODELS__ITEM_HPP

#include <optional>
#include <stdexcept>

#include "metadata.hpp"
#include "../yaml_helpers.hpp"

namespace nexus::common {

class Item
{
public:
  YAML::Node yaml;

  Item(YAML::Node yaml)
  : yaml(std::move(yaml)) {}

  Item() {}

  std::string guid() const
  {
    return this->yaml["guid"].as<std::string>();
  }

  std::optional<MetaData> metadata() const
  {
    if (this->yaml["metadata"])
    {
      return this->yaml["metadata"];
    }
    return std::nullopt;
  }

  std::optional<std::string> station() const
  {
    if (this->yaml["station"])
    {
      return this->yaml["station"].as<std::string>();
    }
    return std::nullopt;
  }
};

}  // namespace nexus::common

namespace YAML {

template<>
struct convert<nexus::common::Item>
{
  static Node encode(const nexus::common::Item& data)
  {
    return data.yaml;
  }

  static bool decode(const Node& node, nexus::common::Item& data)
  {
    if (!node["guid"] || node["guid"].as<std::string>().empty())
    {
      throw std::invalid_argument("missing required [guid] field");
    }

    data.yaml = node;
    return true;
  }
};

}  // namespace YAML

#endif  // NEXUS_COMMON__MODELS__ITEM_HPP
