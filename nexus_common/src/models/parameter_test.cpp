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

#include <rmf_utils/catch.hpp>

#include <nexus_common/models/parameter.hpp>

#include <yaml-cpp/yaml.h>

namespace nexus::common::test {

TEST_CASE("String parameter serialization")
{
  std::string raw{
    R"RAW(
      {
        "name": "string_parameter_name",
        "stringValue": "dummy_string_value"
      }
    )RAW"};

  auto check_data = [](const Parameter& parameter)
  {
    CHECK(parameter.name() == "string_parameter_name");
    CHECK(parameter.string_value() == "dummy_string_value");
  };

  auto string_parameter = YAML::Load(raw).as<Parameter>();
  check_data(string_parameter);

  YAML::Emitter out;
  raw = (out << YAML::Node{string_parameter}).c_str();
  string_parameter = YAML::Load(raw).as<Parameter>();
  check_data(string_parameter);
}

TEST_CASE("Int parameter serialization")
{
  std::string raw{
    R"RAW(
      {
        "name": "int_parameter_name",
        "intValue": 123
      }
    )RAW"};

  auto check_data = [](const Parameter& parameter)
  {
    CHECK(parameter.name() == "int_parameter_name");
    CHECK(parameter.int_value() == 123);
  };

  auto int_parameter = YAML::Load(raw).as<Parameter>();
  check_data(int_parameter);

  YAML::Emitter out;
  raw = (out << YAML::Node{int_parameter}).c_str();
  int_parameter = YAML::Load(raw).as<Parameter>();
  check_data(int_parameter);
}

TEST_CASE("Double parameter serialization")
{
  std::string raw{
    R"RAW(
      {
        "name": "double_parameter_name",
        "doubleValue": 123.456
      }
    )RAW"};

  auto check_data = [](const Parameter& parameter)
  {
    CHECK(parameter.name() == "double_parameter_name");
    CHECK(std::abs(parameter.double_value() - 123.456) < 1e-3);
  };

  auto double_parameter = YAML::Load(raw).as<Parameter>();
  check_data(double_parameter);

  YAML::Emitter out;
  raw = (out << YAML::Node{double_parameter}).c_str();
  double_parameter = YAML::Load(raw).as<Parameter>();
  check_data(double_parameter);
}

TEST_CASE("Bool parameter serialization")
{
  std::string raw{
    R"RAW(
      {
        "name": "bool_parameter_name",
        "boolValue": false
      }
    )RAW"};

  auto check_data = [](const Parameter& parameter)
  {
    CHECK(parameter.name() == "bool_parameter_name");
    CHECK(parameter.bool_value() == false);
  };

  auto bool_parameter = YAML::Load(raw).as<Parameter>();
  check_data(bool_parameter);

  YAML::Emitter out;
  raw = (out << YAML::Node{bool_parameter}).c_str();
  bool_parameter = YAML::Load(raw).as<Parameter>();
  check_data(bool_parameter);
}

TEST_CASE("Enumeration parameter serialization")
{
  std::string raw{
    R"RAW(
      {
        "name": "enum_parameter_name",
        "enumerationValue": 3
      }
    )RAW"};

  auto check_data = [](const Parameter& parameter)
  {
    CHECK(parameter.name() == "enum_parameter_name");
    CHECK(parameter.enumeration_value() == 3);
  };

  auto enum_parameter = YAML::Load(raw).as<Parameter>();
  check_data(enum_parameter);

  YAML::Emitter out;
  raw = (out << YAML::Node{enum_parameter}).c_str();
  enum_parameter = YAML::Load(raw).as<Parameter>();
  check_data(enum_parameter);
}

TEST_CASE("Invalid parameter value access")
{
  std::string raw{
    R"RAW(
      {
        "name": "string_parameter_name",
        "stringValue": "dummy_string_value"
      }
    )RAW"};

  auto string_parameter = YAML::Load(raw).as<Parameter>();
  REQUIRE_THROWS_AS(string_parameter.int_value(), YAML::InvalidNode);
  REQUIRE_THROWS_AS(string_parameter.double_value(), YAML::InvalidNode);
  REQUIRE_THROWS_AS(string_parameter.bool_value(), YAML::InvalidNode);
  REQUIRE_THROWS_AS(string_parameter.enumeration_value(), YAML::InvalidNode);
}

} // namespace nexus::common::test
