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

#include "bt_store.hpp"

namespace nexus::common {

void BTStore::register_from_path(const std::filesystem::path& path)
{
  for (const auto& dir_entry : std::filesystem::directory_iterator {path})
  {
    if (dir_entry.is_regular_file() && dir_entry.path().extension() == ".xml")
    {
      std::string name = dir_entry.path().stem();
      this->register_file(name, dir_entry.path());
    }
  }
}

void BTStore::register_file(const std::string& key,
  const std::filesystem::path& filepath)
{
  this->_bt_map.insert({key, filepath});
  for (auto& handler : this->on_register_handlers)
  {
    handler(key, filepath);
  }
}

std::vector<std::string> BTStore::list_bt() const
{
  std::vector<std::string> keys;
  for (const auto&[k, _] : _bt_map)
  {
    keys.emplace_back(k);
  }
  return keys;
}

std::filesystem::path& BTStore::get_bt(const std::string& key)
{
  return this->_bt_map.at(key);
}

}
