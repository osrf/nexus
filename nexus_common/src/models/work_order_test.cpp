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
            },
            "inputItems": [
              {
                "guid": "1001",
                "metadata": {
                  "description": "dummy_sku",
                  "unit": "dummy_unit",
                  "quantity": 1,
                  "quantityPerPallet": 1
                }
              }
            ]
          },
          {
            "processId": "place",
            "outputItems": [
              {
                "guid": "1001",
                "metadata": {
                  "description": "dummy_sku",
                  "unit": "dummy_unit",
                  "quantity": 1,
                  "quantityPerPallet": 1
                }
              }
            ]
          },
          {
            "processId": "inspect",
            "inputItems": [
              {
                "guid": "1001",
                "metadata": {
                  "description": "dummy_sku",
                  "unit": "dummy_unit",
                  "quantity": 1,
                  "quantityPerPallet": 1
                }
              }
            ],
            "outputItems": [
              {
                "guid": "1001",
                "metadata": {
                  "description": "dummy_sku",
                  "unit": "dummy_unit",
                  "quantity": 1,
                  "quantityPerPallet": 1
                }
              }
            ]
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
    const auto& step1_params = step1.process_params();
    REQUIRE(step1_params);
    CHECK(step1_params["param1"].as<int>() == 10);
    CHECK(step1_params["param2"].as<std::string>() == "base_link");
    const auto& step1_inputs = step1.input_items();
    REQUIRE(step1_inputs.size() == 1);
    auto item = step1_inputs[0];
    CHECK(item.guid() == "1001");
    REQUIRE(item.metadata().has_value());
    const auto input_1_metadata = item.metadata().value();
    CHECK(input_1_metadata.yaml["description"].as<std::string>() ==
      "dummy_sku");
    CHECK(input_1_metadata.yaml["unit"].as<std::string>() == "dummy_unit");
    CHECK(input_1_metadata.yaml["quantity"].as<int>() == 1);
    CHECK(input_1_metadata.yaml["quantityPerPallet"].as<int>() == 1);
    CHECK(step1.output_items().size() == 0);

    const auto& step2 = steps[1];
    CHECK(step2.process_id() == "place");
    CHECK_FALSE(step2.process_params());
    CHECK(step2.input_items().size() == 0);
    const auto& step2_outputs = step2.output_items();
    REQUIRE(step2_outputs.size() == 1);
    item = step2_outputs[0];
    CHECK(item.guid() == "1001");
    REQUIRE(item.metadata().has_value());
    const auto output_2_metadata = item.metadata().value();
    CHECK(output_2_metadata.yaml["description"].as<std::string>() ==
      "dummy_sku");
    CHECK(output_2_metadata.yaml["unit"].as<std::string>() == "dummy_unit");
    CHECK(output_2_metadata.yaml["quantity"].as<int>() == 1);
    CHECK(output_2_metadata.yaml["quantityPerPallet"].as<int>() == 1);

    const auto& step3 = steps[2];
    CHECK(step3.process_id() == "inspect");
    CHECK_FALSE(step3.process_params());
    const auto& step3_inputs = step3.input_items();
    REQUIRE(step3_inputs.size() == 1);
    item = step3_inputs[0];
    CHECK(item.guid() == "1001");
    REQUIRE(item.metadata().has_value());
    const auto input_3_metadata = item.metadata().value();
    CHECK(input_3_metadata.yaml["description"].as<std::string>() ==
      "dummy_sku");
    CHECK(input_3_metadata.yaml["unit"].as<std::string>() == "dummy_unit");
    CHECK(input_3_metadata.yaml["quantity"].as<int>() == 1);
    CHECK(input_3_metadata.yaml["quantityPerPallet"].as<int>() == 1);
    const auto& step3_outputs = step3.output_items();
    REQUIRE(step3_outputs.size() == 1);
    item = step3_outputs[0];
    CHECK(item.guid() == "1001");
    REQUIRE(item.metadata().has_value());
    const auto output_3_metadata = item.metadata().value();
    CHECK(output_3_metadata.yaml["description"].as<std::string>() ==
      "dummy_sku");
    CHECK(output_3_metadata.yaml["unit"].as<std::string>() == "dummy_unit");
    CHECK(output_3_metadata.yaml["quantity"].as<int>() == 1);
    CHECK(output_3_metadata.yaml["quantityPerPallet"].as<int>() == 1);
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

}  // namespace nexus::common::test
