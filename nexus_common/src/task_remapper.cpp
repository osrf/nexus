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

#include "task_remapper.hpp"

namespace nexus::common {

TaskRemapper::TaskRemapper(const YAML::Node& remaps)
{
  for (const auto& n : remaps)
  {
    const auto task_type = n.first.as<std::string>();
    const auto& mappings = n.second;
    for (const auto& m : mappings)
    {
      auto mapping = m.as<std::string>();
      if (mapping == "*")
      {
        this->_wildcard_match = task_type;
        this->_task_remaps.clear();
        return;
      }
      // TODO(luca) check for duplicates, logging if found
      this->_task_remaps.emplace(mapping, task_type);
    }
  }
}

std::optional<std::string> TaskRemapper::remap(const std::string& task) const
{
  if (this->_wildcard_match.has_value())
  {
    return this->_wildcard_match.value();
  }
  const auto it = this->_task_remaps.find(task);
  if (it != this->_task_remaps.end())
  {
    return it->second;
  }
  return std::nullopt;
}


}
