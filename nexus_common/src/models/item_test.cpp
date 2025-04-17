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

#include <nexus_common/models/item.hpp>
#include <rmf_utils/catch.hpp>

#include <yaml-cpp/yaml.h>

#include <unordered_set>

namespace nexus::common::test {

TEST_CASE("Item serialization", "[Model][Serialization]")
{
  std::string raw{
    R"RAW(
      {
        "SkuId": "1001",
        "description": "dummy_sku",
        "unit": "dummy_unit",
        "quantity": 1.0,
        "quantityPerPallet": 1.0
      }
  )RAW"};

  auto check_data = [](const Item& item)
  {
    CHECK(item.sku_id() == "1001");
    CHECK(item.description() == "dummy_sku");
    CHECK(item.unit() == "dummy_unit");
    CHECK(item.quantity() == 1);
    CHECK(item.quantity_per_pallet() == 1);
  };

  auto item = YAML::Load(raw).as<Item>();
  check_data(item);

  YAML::Emitter out;
  raw = (out << YAML::Node{item}).c_str();
  item = YAML::Load(raw).as<Item>();
  check_data(item);
}

} // namespace nexus::common::test
