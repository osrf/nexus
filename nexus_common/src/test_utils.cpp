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

#include "nexus_common_test/test_utils.hpp"

#include <chrono>
#include <random>
#include <sstream>
#include <stdexcept>

namespace nexus::common::test {

std::string unique_node_name(const std::string& prefix)
{
  // create almost unique name from uint64_t random and current time in nanoseconds.
  static std::mt19937 gen;
  std::uniform_int_distribution<uint64_t> distrib;
  std::chrono::nanoseconds now =
    std::chrono::high_resolution_clock::now().time_since_epoch();
  return (std::ostringstream{} << prefix << "_" << now.count() << "_" <<
    distrib(gen)).str();
}

}
