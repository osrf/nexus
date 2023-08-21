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

#ifndef NEXUS_COMMON__BT_STORE_HPP
#define NEXUS_COMMON__BT_STORE_HPP

#include "nexus_common_export.hpp"

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace nexus::common {

/**
 * Provides a way to discover behavior trees from the filesystem.
 */
class NEXUS_COMMON_EXPORT BTStore
{
public:
  using OnRegisterHandler = std::function<void(const std::string& key,
      const std::filesystem::path& filepath)>;

  std::vector<OnRegisterHandler> on_register_handlers;

  /**
   * Register all behavior trees found in path, their keys will be the filename without
   * extensions. e.g. "place_on_conveyor.xml" will be registered as "place_on_conveyor".
   * @param path Path to search for behavior trees.
   */
  void register_from_path(const std::filesystem::path& path);

  /**
   * Register a file.
   * @param key Key to register to.
   * @param filepath File to register.
   */
  void register_file(const std::string& key,
    const std::filesystem::path& filepath);

  /// Returns a list of all behavior trees registered.
  std::vector<std::string> list_bt() const;

  /**
   * Get the path to a behavior tree.
   *
   * @param key This is the name of the behavior tree file, without the extension.
   * @throw std::out_of_range if the key is not registered.
   */
  std::filesystem::path& get_bt(const std::string& key);

private:
  std::unordered_map<std::string, std::filesystem::path> _bt_map;
};

}

#endif
