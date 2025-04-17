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

#include "../yaml_helpers.hpp"

namespace nexus::common {

class WorkOrder
{
public: struct MetaData
  {
    YAML::Node yaml;

    MetaData(YAML::Node yaml)
    : yaml(std::move(yaml)) {}

    MetaData() {}
  };

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

  };

public: YAML::Node yaml;

public: WorkOrder(YAML::Node yaml)
  : yaml(std::move(yaml)) {}

public: WorkOrder() {}

public: std::string work_instruction_name() const
  {
    return this->yaml["workInstructionName"].as<std::string>();
  }

public: std::optional<MetaData> metadata() const
  {
    if (this->yaml["metadata"]) {
      return this->yaml["metadata"];
    };
    return std::nullopt;
  }

public: std::vector<Step> steps() const
  {
    return this->yaml["steps"].as<std::vector<Step>>();
  }
};

}

namespace YAML {

template<>
struct convert<nexus::common::WorkOrder::MetaData>
{
  static Node encode(const nexus::common::WorkOrder::MetaData& data)
  {
    return data.yaml;
  }

  static bool decode(const Node& node, nexus::common::WorkOrder::MetaData& data)
  {
    data.yaml = node;
    return true;
  }
};

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
    data.yaml = node;
    return true;
  }
};

}

#endif
