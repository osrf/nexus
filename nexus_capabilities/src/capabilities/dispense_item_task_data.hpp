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

#ifndef NEXUS_CAPABILITIES__CAPABILITIES__DISPENSE_ITEM_TASK_DATA_HPP
#define NEXUS_CAPABILITIES__CAPABILITIES__DISPENSE_ITEM_TASK_DATA_HPP

#include <yaml-cpp/yaml.h>

namespace nexus::capabilities {

class DispenseItemTaskData
{
public: YAML::Node yaml;

public: explicit DispenseItemTaskData(YAML::Node data)
  : yaml(std::move(data)) {}

public: std::string item() const
  {
    return this->yaml["item"].template as<std::string>();
  }

public: DispenseItemTaskData& item(const std::string& item)
  {
    this->yaml["item"] = item;
    return *this;
  }
};

}

#endif // NEXUS_CAPABILITIES__CAPABILITIES__DISPENSE_ITEM_TASK_DATA_HPP
