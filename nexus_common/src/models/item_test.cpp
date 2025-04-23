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

#include <nexus_common/models/item.hpp>
#include <rmf_utils/catch.hpp>

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace nexus::common::test {

TEST_CASE("Item serialization with metadata")
{
  std::string raw{
    R"RAW(
      {
        "guid": "1001",
        "metadata": {
          "description": "dummy_sku",
          "unit": "dummy_unit",
          "quantity": 1,
          "quantityPerPallet": 1
        }
      }
  )RAW"};

  auto check_data = [](const Item& item)
  {
    CHECK(item.guid() == "1001");
    const auto maybe_metadata = item.metadata();
    REQUIRE(maybe_metadata.has_value());
    const auto metadata = maybe_metadata.value();
    CHECK(metadata.yaml["description"].as<std::string>() == "dummy_sku");
    CHECK(metadata.yaml["unit"].as<std::string>() == "dummy_unit");
    CHECK(metadata.yaml["quantity"].as<int>() == 1);
    CHECK(metadata.yaml["quantityPerPallet"].as<int>() == 1);
  };

  auto item = YAML::Load(raw).as<Item>();
  check_data(item);

  YAML::Emitter out;
  raw = (out << YAML::Node{item}).c_str();
  item = YAML::Load(raw).as<Item>();
  check_data(item);
}

TEST_CASE("Item serialization without guid")
{
  std::string raw{
    R"RAW(
      {
        "metadata": {
          "description": "dummy_sku",
          "unit": "dummy_unit",
          "quantity": 1,
          "quantityPerPallet": 1
        }
      }
  )RAW"};

  bool invalid_argument = false;
  try
  {
    auto item = YAML::Load(raw).as<Item>();
  }
  catch (const std::invalid_argument& e)
  {
    invalid_argument = true;
  }
  CHECK(invalid_argument);
}

}  // namespace nexus::common::test
