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

#include <rmf_utils/catch.hpp>

#include "models/work_order.hpp"

#include <yaml-cpp/yaml.h>

#include <unordered_set>

namespace nexus::common::test {

TEST_CASE("WorkOrder serialization", "[Model][Serialization]")
{
  std::string raw{
    R"RAW(
      {
        "number": "SO/2022/20/1-1",
        "workInstructionName": "CV-299 (Rev 4)",
        "item": {
          "SkuId": "1001",
          "description": "dummy_sku",
          "unit": "dummy_unit",
          "quantity": 1.0,
          "quantityPerPallet": 1.0
        },
        "steps": [
          {
            "id": 31.0,
            "processId": "pickup",
            "name": "pickup item"
          },
          {
            "id": 32.0,
            "processId": "place",
            "name": "place item"
          },
          {
            "id": 33.0,
            "processId": "inspect",
            "name": "inspect item"
          }
        ]
      }
  )RAW"};

  auto check_data = [](const WorkOrder& work_order)
    {
      CHECK(work_order.number() == "SO/2022/20/1-1");
      CHECK(work_order.work_instruction_name() == "CV-299 (Rev 4)");
      const auto item = work_order.item();
      CHECK(item.sku_id() == "1001");
      CHECK(item.description() == "dummy_sku");
      CHECK(item.unit() == "dummy_unit");
      CHECK(item.quantity() == 1);
      CHECK(item.quantity_per_pallet() == 1);
      REQUIRE(work_order.steps().size() == 3);

      const auto steps = work_order.steps();

      const auto step1 = steps[0];
      CHECK(step1.id() == 31);
      CHECK(step1.process_id() == "pickup");
      CHECK(step1.name() ==
        "pickup item");

      const auto& step2 = steps[1];
      CHECK(step2.id() == 32);
      CHECK(step2.process_id() == "place");
      CHECK(step2.name() == "place item");

      const auto& step3 = steps[2];
      CHECK(step3.id() == 33);
      CHECK(step3.process_id() == "inspect");
      CHECK(step3.name() == "inspect item");
    };

  auto work_order = YAML::Load(raw).as<WorkOrder>();
  check_data(work_order);

  YAML::Emitter out;
  raw = (out << YAML::Node{work_order}).c_str();
  work_order = YAML::Load(raw).as<WorkOrder>();
  check_data(work_order);
}

}
