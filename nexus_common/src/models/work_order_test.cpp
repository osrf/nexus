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

#include <nexus_common/models/work_order.hpp>
#include <rmf_utils/catch.hpp>

#include <yaml-cpp/yaml.h>

#include <unordered_set>

namespace nexus::common::test {

TEST_CASE("WorkOrder serialization with metadata and process params",
  "[Model][Serialization]")
{
  std::string raw{
    R"RAW(
      {
        "workInstructionName": "CV-299 (Rev 4)",
        "metadata": {
          "SkuId": "1001",
          "description": "dummy_sku",
          "unit": "dummy_unit",
          "quantity": 1,
          "quantityPerPallet": 1
        },
        "steps": [
          {
            "processId": "pickup",
            "processParams": {
              "param1": 10,
              "param2": "base_link"
            }
          },
          {
            "processId": "place"
          },
          {
            "processId": "inspect"
          }
        ]
      }
  )RAW"};

  auto check_data = [](const WorkOrder& work_order)
    {
      CHECK(work_order.work_instruction_name() == "CV-299 (Rev 4)");
      const auto maybe_metadata = work_order.metadata();
      REQUIRE(maybe_metadata.has_value());
      const auto metadata = maybe_metadata.value();
      CHECK(metadata.yaml["SkuId"].as<std::string>() == "1001");
      CHECK(metadata.yaml["description"].as<std::string>() == "dummy_sku");
      CHECK(metadata.yaml["unit"].as<std::string>() == "dummy_unit");
      CHECK(metadata.yaml["quantity"].as<int>() == 1);
      CHECK(metadata.yaml["quantityPerPallet"].as<int>() == 1);
      REQUIRE(work_order.steps().size() == 3);

      const auto steps = work_order.steps();

      const auto step1 = steps[0];
      CHECK(step1.process_id() == "pickup");
      const auto & step1_params = step1.process_params();
      REQUIRE(step1_params);
      CHECK(step1_params["param1"].as<int>() == 10);
      CHECK(step1_params["param2"].as<std::string>() == "base_link");

      const auto& step2 = steps[1];
      CHECK(step2.process_id() == "place");
      CHECK_FALSE(step2.process_params());

      const auto& step3 = steps[2];
      CHECK(step3.process_id() == "inspect");
      CHECK_FALSE(step3.process_params());
    };

  auto work_order = YAML::Load(raw).as<WorkOrder>();
  check_data(work_order);

  YAML::Emitter out;
  raw = (out << YAML::Node{work_order}).c_str();
  work_order = YAML::Load(raw).as<WorkOrder>();
  check_data(work_order);
}

TEST_CASE("WorkOrder serialization without metadata", "[Model][Serialization]")
{
  std::string raw{
    R"RAW(
      {
        "workInstructionName": "CV-299 (Rev 4)",
        "steps": [
          {
            "processId": "pickup"
          },
          {
            "processId": "place"
          },
          {
            "processId": "inspect"
          }
        ]
      }
  )RAW"};

  auto check_data = [](const WorkOrder& work_order)
    {
      CHECK(work_order.work_instruction_name() == "CV-299 (Rev 4)");
      const auto maybe_metadata = work_order.metadata();
      REQUIRE_FALSE(maybe_metadata.has_value());
      REQUIRE(work_order.steps().size() == 3);

    const auto steps = work_order.steps();

      const auto step1 = steps[0];
      CHECK(step1.process_id() == "pickup");

      const auto& step2 = steps[1];
      CHECK(step2.process_id() == "place");

      const auto& step3 = steps[2];
      CHECK(step3.process_id() == "inspect");
    };

  auto work_order = YAML::Load(raw).as<WorkOrder>();
  check_data(work_order);

  YAML::Emitter out;
  raw = (out << YAML::Node{work_order}).c_str();
  work_order = YAML::Load(raw).as<WorkOrder>();
  check_data(work_order);
}

} // namespace nexus::common::test
