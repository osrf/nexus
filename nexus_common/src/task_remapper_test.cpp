/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#define CATCH_CONFIG_MAIN
#include <rmf_utils/catch.hpp>

#include "task_remapper.hpp"

namespace nexus::common::test {

TEST_CASE("task_remapping") {
  std::string param =
    R"(
      pick_and_place: [pick, place]
    )";
  auto remapper = TaskRemapper(param);
  CHECK(remapper.remap("pick") == "pick_and_place");
  CHECK(remapper.remap("place") == "pick_and_place");
  CHECK(remapper.remap("other") == "other");
}

TEST_CASE("task_remapping_with_wildcard") {
  std::string param =
    R"(
      pick_and_place: [pick, place]
      main : ["*"]
    )";
  auto remapper = TaskRemapper(param);
  CHECK(remapper.remap("pick") == "main");
  CHECK(remapper.remap("place") == "main");
  CHECK(remapper.remap("other") == "main");
}

}
