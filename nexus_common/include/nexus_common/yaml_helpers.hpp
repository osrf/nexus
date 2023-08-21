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

#ifndef NEXUS_COMMON__YAML_PARSER_HPP
#define NEXUS_COMMON__YAML_PARSER_HPP

#include <yaml-cpp/yaml.h>

namespace nexus::common {

struct YamlHelpers
{
  /**
   * Writes the value of a yaml node into a variable.
   */
  template<typename T>
  static std::enable_if_t<!std::is_integral_v<T>> write(const YAML::Node& node,
    T& var)
  {
    var = node.as<T>();
  }

  /**
   * Writes the value of a yaml node into a variable, automatically converting from
   * floating value to integer if it does not result in overflow and lost of precision.
  */
  template<typename T>
  static std::enable_if_t<std::is_integral_v<T>> write(const YAML::Node& node,
    T& var)
  {
    try
    {
      var = node.as<T>();
    }
    catch (const YAML::BadConversion&)
    {
      double d = node.as<double>();
      if (d < std::numeric_limits<T>::min() ||
        d > std::numeric_limits<T>::max())
      {
        throw;
      }
      T v = d;
      if (v != d)
      {
        throw;
      }
      var = v;
    }
  }
};

}

#endif
