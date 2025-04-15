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

#include <optional>
#include <stdexcept>

#include "item.hpp"
#include "metadata.hpp"
#include "../yaml_helpers.hpp"

namespace nexus::common {

struct WorkOrder
{
public: struct Step
  {
    YAML::Node yaml;

    Step(YAML::Node yaml)
    : yaml(std::move(yaml)) {}

    Step() {}

    std::string process_id() const
    {
      return this->yaml["processId"].as<std::string>();
    }

    YAML::Node process_params() const
    {
      return this->yaml["processParams"];
    }

    std::vector<Item> input_items() const
    {
      if (!this->yaml["inputItems"])
      {
        return {};
      }

      return this->yaml["inputItems"].as<std::vector<Item>>();
    }

    std::vector<Item> output_items() const
    {
      if (!this->yaml["outputItems"])
      {
        return {};
      }

      return this->yaml["outputItems"].as<std::vector<Item>>();
    }
  };

public: YAML::Node yaml;

public: WorkOrder(YAML::Node yaml)
  : yaml(std::move(yaml)) {}

  WorkOrder() {}

  std::string work_instruction_name() const
  {
    return this->yaml["workInstructionName"].as<std::string>();
  }

public: std::optional<MetaData> metadata() const
  {
    if (this->yaml["metadata"])
    {
      return this->yaml["metadata"];
    }
    return std::nullopt;
  }

public: std::vector<Step> steps() const
  {
    return this->yaml["steps"].as<std::vector<Step>>();
  }
};

}  // namespace nexus::common

namespace YAML {

template<>
struct convert<nexus::common::WorkOrder::Step>
{
  static Node encode(const nexus::common::WorkOrder::Step& data)
  {
    return data.yaml;
  }

  static bool decode(const Node& node,
    nexus::common::WorkOrder::Step& data)
  {
    if (!node["processId"] || node["processId"].as<std::string>().empty())
    {
      throw std::invalid_argument("missing required [processId] field");
    }

    data.yaml = node;
    return true;
  }
};

template<>
struct convert<nexus::common::WorkOrder>
{
  static Node encode(const nexus::common::WorkOrder& data)
  {
    return data.yaml;
  }

  static bool decode(const Node& node, nexus::common::WorkOrder& data)
  {
    if (!node["workInstructionName"] ||
      node["workInstructionName"].as<std::string>().empty())
    {
      throw std::invalid_argument(
              "missing required [workInstructionName] field");
    }
    if (!node["steps"] ||
      node["steps"].as<std::vector<nexus::common::WorkOrder::Step>>().empty())
    {
      throw std::invalid_argument("missing required [steps] field");
    }

    data.yaml = node;
    return true;
  }
};

}  // namespace YAML

#endif  // NEXUS_COMMON__MODELS__WORK_ORDER_HPP
