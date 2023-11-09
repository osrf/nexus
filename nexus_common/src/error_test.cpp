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

#include <catch2/catch.hpp>

#include "nexus_common/error.hpp"

namespace nexus::common::test {

TEST_CASE("get_result_value_and_error") {
  Result<int> good(1);
  CHECK(good.value() == 1);
  CHECK(!good.error());

  const Result<int> const_good(1);
  CHECK(const_good.value() == 1);
  CHECK(!const_good.error());

  Result<int> bad(std::make_shared<std::runtime_error>("bad"));
  CHECK_THROWS(bad.value());
  CHECK(bad.error());

  Result<int> bad_val_constructor(std::runtime_error("bad_val_constructor"));
  CHECK_THROWS(bad_val_constructor.value());
  CHECK(bad_val_constructor.error());

  Result<void> void_good;
  CHECK(!void_good.error());

  Result<void> void_bad(std::make_shared<std::runtime_error>("void_bad"));
  CHECK(void_bad.error());

  Result<void> void_bad_val_constructor(std::runtime_error(
      "void_bad_val_constructor"));
  CHECK(void_bad_val_constructor.error());
}

}
