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

#ifndef NEXUS_COMMON__MODELS__ITEM_HPP
#define NEXUS_COMMON__MODELS__ITEM_HPP

#include <nexus_common/yaml_helpers.hpp>

#include <string>

namespace nexus::common {

struct Item
{
  YAML::Node yaml;

  Item(YAML::Node yaml)
  : yaml(std::move(yaml)) {}

  Item() {}

  std::string sku_id() const
  {
    return this->yaml["SkuId"].as<std::string>();
  }

  std::string description() const
  {
    return this->yaml["description"].as<std::string>();
  }

  std::string unit() const
  {
    return this->yaml["unit"].as<std::string>();
  }

  int32_t quantity() const
  {
    return this->yaml["quantity"].as<double>();
  }

  int32_t quantity_per_pallet() const
  {
    return this->yaml["quantityPerPallet"].as<double>();
  }
};

} // namespace nexus::common

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
    data.yaml = node;
    return true;
  }
};

} // namespace YAML

#endif // NEXUS_COMMON__MODELS__ITEM_HPP