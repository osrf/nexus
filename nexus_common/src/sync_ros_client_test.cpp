/*
 * Copyright (C) 2023 Johnson & Johnson
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
#include <catch2/catch.hpp>

#include "sync_service_client.hpp"
#include "test_utils.hpp"

#include <example_interfaces/srv/add_two_ints.hpp>

namespace nexus::common::test {

TEST_CASE("successfully send and receive response") {
  using example_interfaces::srv::AddTwoInts;

  RosFixture fixture;
  auto server = fixture.node->create_service<AddTwoInts>("test",
      [](AddTwoInts::Request::ConstSharedPtr,
      AddTwoInts::Response::SharedPtr resp)
      {
        resp->sum = 101;
      });
  fixture.spin_in_background();

  SyncServiceClient<AddTwoInts> client(fixture.node,
    "test");
  auto resp = client.send_request(std::make_shared<AddTwoInts::Request>());
  CHECK(resp->sum == 101);
}

}
