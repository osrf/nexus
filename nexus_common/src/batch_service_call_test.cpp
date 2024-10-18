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

#include "batch_service_call.hpp"
#include "nexus_common_test/test_utils.hpp"

#include <rclcpp/rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

namespace nexus::common::test {

TEST_CASE("batch service call", "[RosUtils]") {
  using example_interfaces::srv::AddTwoInts;

  RosFixture<rclcpp::Node> client_fixture;
  RosFixture<rclcpp::Node> server_fixture;

  SECTION("successfully call services") {
    rclcpp::Service<AddTwoInts>::SharedPtr server_1;
    size_t server_recv_count1 = 0;
    rclcpp::Service<AddTwoInts>::SharedPtr server_2;
    size_t server_recv_count2 = 0;
    {
      auto& node = server_fixture.node;
      server_1 = node->create_service<AddTwoInts>("service_1",
          [&](AddTwoInts::Request::ConstSharedPtr,
          AddTwoInts::Response::SharedPtr resp)
          {
            ++server_recv_count1;
            resp->sum = 1;
          });
      server_2 = node->create_service<AddTwoInts>("service_2",
          [&](AddTwoInts::Request::ConstSharedPtr,
          AddTwoInts::Response::SharedPtr resp)
          {
            ++server_recv_count2;
            resp->sum = 2;
          });
    }
    server_fixture.spin_in_background();

    auto& node = client_fixture.node;
    std::unordered_map<std::string, BatchServiceReq<AddTwoInts>> reqs;
    reqs.emplace("service_1",
      BatchServiceReq<AddTwoInts>{node->create_client<AddTwoInts>(
          "service_1"), std::make_shared<AddTwoInts::Request>()});
    reqs.emplace("service_2",
      BatchServiceReq<AddTwoInts>{node->create_client<AddTwoInts>(
          "service_2"), std::make_shared<AddTwoInts::Request>()});
    std::promise<void> done;
    batch_service_call(node, reqs,
      std::chrono::milliseconds{1000},
      [&done](const std::unordered_map<std::string,
      BatchServiceResult<AddTwoInts>>& results)
      {
        REQUIRE(results.size() == 2);
        CHECK(results.at("service_1").success);
        CHECK(results.at("service_1").resp->sum == 1);
        CHECK(results.at("service_2").success);
        CHECK(results.at("service_2").resp->sum == 2);
        done.set_value();
      });
    client_fixture.spin_in_background();

    if (done.get_future().wait_for(std::chrono::seconds{1}) !=
      std::future_status::ready)
    {
      FAIL("Timeout");
    }
    CHECK(server_recv_count1 == 1);
    CHECK(server_recv_count2 == 1);
  }

  SECTION("timeout one 1 service") {
    rclcpp::Service<AddTwoInts>::SharedPtr server_1;
    size_t server_recv_count1 = 0;
    rclcpp::Service<AddTwoInts>::SharedPtr server_2;
    size_t server_recv_count2 = 0;
    {
      auto& node = server_fixture.node;
      server_1 = node->create_service<AddTwoInts>("service_1",
          [&](AddTwoInts::Request::ConstSharedPtr,
          AddTwoInts::Response::SharedPtr resp)
          {
            ++server_recv_count1;
            resp->sum = 1;
          });
      server_2 = node->create_service<AddTwoInts>("service_2",
          [&](rclcpp::Service<AddTwoInts>::SharedPtr,
          std::shared_ptr<rmw_request_id_t>, AddTwoInts::Request::SharedPtr)
          {
            ++server_recv_count2;
            // never send a response
          });
    }
    server_fixture.spin_in_background();

    auto& node = client_fixture.node;
    std::unordered_map<std::string, BatchServiceReq<AddTwoInts>> reqs;
    reqs.emplace("service_1",
      BatchServiceReq<AddTwoInts>{node->create_client<AddTwoInts>(
          "service_1"), std::make_shared<AddTwoInts::Request>()});
    reqs.emplace("service_2",
      BatchServiceReq<AddTwoInts>{node->create_client<AddTwoInts>(
          "service_2"), std::make_shared<AddTwoInts::Request>()});
    std::promise<void> done;
    batch_service_call(node, reqs,
      std::chrono::milliseconds{100},
      [&done](const std::unordered_map<std::string,
      BatchServiceResult<AddTwoInts>>& results)
      {
        REQUIRE(results.size() == 2);
        CHECK(results.at("service_1").success);
        CHECK(results.at("service_1").resp->sum == 1);
        CHECK_FALSE(results.at("service_2").success);
        done.set_value();
      });
    client_fixture.spin_in_background();

    if (done.get_future().wait_for(std::chrono::seconds{1}) !=
      std::future_status::ready)
    {
      FAIL("Timeout");
    }
    CHECK(server_recv_count1 == 1);
    CHECK(server_recv_count2 == 1);
  }
}

}
