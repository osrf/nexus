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

#ifndef NEXUS_COMMON__TASK_REMAPPER_HPP
#define NEXUS_COMMON__TASK_REMAPPER_HPP

#include "nexus_common_export.hpp"

#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <string>

namespace nexus::common {

/**
 * Provides task remapping capability
 */
class NEXUS_COMMON_EXPORT TaskRemapper
{
public:
  /*
   * Initialize the remapper with the value of a ROS parameter containing a YAML
   */
  TaskRemapper(const std::string& param);

  /*
   * Remaps, if necessary, the input task
   * Returns a value if the task was remapped, std::nullopt otherwise
   */
  std::optional<std::string> remap(const std::string& task) const;

private:
  // If present, match every incoming task to the target task
  std::optional<std::string> _wildcard_match;
  std::unordered_map<std::string, std::string> _task_remaps;
};

}

#endif
