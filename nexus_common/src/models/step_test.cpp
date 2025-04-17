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

#include <nexus_common/models/step.hpp>
#include <rmf_utils/catch.hpp>

#include <yaml-cpp/yaml.h>

#include <unordered_set>

namespace nexus::common::test {

TEST_CASE("Step serialization", "[Model][Serialization]")
{
  std::string raw{
    R"RAW(
      {
        "processId": "assemble",
        "name": "assemble items",
        "inputItems": [
          {
            "SkuId": "1001",
            "description": "dummy_sku_1",
            "unit": "dummy_unit_1",
            "quantity": 1.0,
            "quantityPerPallet": 1.0
          },
          {
            "SkuId": "1002",
            "description": "dummy_sku_2",
            "unit": "dummy_unit_2",
            "quantity": 1.0,
            "quantityPerPallet": 1.0
          }
        ],
        "outputItems": [
          {
            "SkuId": "1003",
            "description": "dummy_sku_3",
            "unit": "dummy_unit_3",
            "quantity": 1.0,
            "quantityPerPallet": 1.0
          }
        ],
        "parameters": [
          {
            "name": "debug",
            "boolValue": true
          },
        ]
      }
  )RAW"};

  auto check_data = [](const Step& step)
  {
    CHECK(step.process_id() == "assemble");
    CHECK(step.name() == "assemble items");

    const auto input_items = step.input_items();
    CHECK(input_items.size() == 2);
    CHECK(input_items[0].sku_id() == "1001");
    CHECK(input_items[0].description() == "dummy_sku_1");
    CHECK(input_items[0].unit() == "dummy_unit_1");
    CHECK(input_items[0].quantity() == 1);
    CHECK(input_items[0].quantity_per_pallet() == 1);
    CHECK(input_items[1].sku_id() == "1002");
    CHECK(input_items[1].description() == "dummy_sku_2");
    CHECK(input_items[1].unit() == "dummy_unit_2");
    CHECK(input_items[1].quantity() == 1);
    CHECK(input_items[1].quantity_per_pallet() == 1);

    const auto output_items = step.output_items();
    CHECK(output_items.size() == 1);
    CHECK(output_items[0].sku_id() == "1003");
    CHECK(output_items[0].description() == "dummy_sku_3");
    CHECK(output_items[0].unit() == "dummy_unit_3");
    CHECK(output_items[0].quantity() == 1);
    CHECK(output_items[0].quantity_per_pallet() == 1);

    const auto parameters = step.parameters();
    CHECK(parameters.size() == 1);
    CHECK(parameters[0].name() == "debug");
    CHECK(parameters[0].bool_value() == true);
  };

  auto step = YAML::Load(raw).as<Step>();
  check_data(step);

  YAML::Emitter out;
  raw = (out << YAML::Node{step}).c_str();
  step = YAML::Load(raw).as<Step>();
  check_data(step);
}

} // namespace nexus::common::test
