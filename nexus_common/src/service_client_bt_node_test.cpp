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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include "nexus_common/service_client_bt_node.hpp"
#include "nexus_common_test/test_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <example_interfaces/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <chrono>
#include <memory>

namespace nexus::common::test {

class TestServiceBtNode : public ServiceClientBtNode<example_interfaces::srv::SetBool>
{
public: static constexpr std::chrono::milliseconds BASE_TIMEOUT{500};

public: TestServiceBtNode(const std::string& name,
    const BT::NodeConfiguration& config, rclcpp::Logger logger,
    rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client)
  : ServiceClientBtNode<example_interfaces::srv::SetBool>{name, config,
      logger, BASE_TIMEOUT}, _client{client} {}

public: static BT::PortsList providedPorts()
  {
    return {BT::InputPort<example_interfaces::srv::SetBool::Request::SharedPtr>(
        "request")};
  }

protected: rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client()
  override
  {
    return this->_client;
  }

protected: example_interfaces::srv::SetBool::Request::SharedPtr make_request()
  override
  {
    return this->getInput<example_interfaces::srv::SetBool::Request::SharedPtr>(
      "request").value();
  }

protected: bool on_response(
    rclcpp::Client<example_interfaces::srv::SetBool>::SharedResponse resp)
  override
  {
    return resp->success;
  }

private: rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr _client;
};

TEST_CASE("ServiceClientBtNode", "[BehaviorTree]")
{
  common::test::RosFixture<rclcpp::Node> uut_fixture;
  auto& uut_node = uut_fixture.node;
  auto client = uut_node->create_client<example_interfaces::srv::SetBool>(
    "set_bool");

  BT::NodeConfiguration bt_config;
  bt_config.blackboard = BT::Blackboard::create();
  bt_config.input_ports.insert({"request", "{request}"});
  auto bt_node = std::make_shared<TestServiceBtNode>("test_service_bt_node",
      bt_config, uut_node->get_logger(), client);
  BT::Tree bt;
  bt.nodes.push_back(bt_node);

  SECTION("bt fails if server is not found")
  {
    uut_fixture.spin_in_background();
    auto req = std::make_shared<example_interfaces::srv::SetBool::Request>();
    req->data = true;
    bt_config.blackboard->set("request", req);
    REQUIRE(tick_until_status(bt, BT::NodeStatus::FAILURE,
      TestServiceBtNode::BASE_TIMEOUT* 2));
  }

  common::test::RosFixture<rclcpp::Node> test_fixture;
  auto& test_node = test_fixture.node;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr test_server;

  std::promise<void> server_wait;
  SECTION("bt fails when server does not response")
  {
    test_server = test_node->create_service<example_interfaces::srv::SetBool>(
      "set_bool",
      [&server_wait](example_interfaces::srv::SetBool::Request::ConstSharedPtr
      req,
      example_interfaces::srv::SetBool::Response::SharedPtr resp)
      {
        // block server from returning result
        server_wait.get_future().wait();
        resp->success = req->data;
      });

    uut_fixture.spin_in_background();
    test_fixture.spin_in_background();
    auto req = std::make_shared<example_interfaces::srv::SetBool::Request>();
    req->data = true;
    bt_config.blackboard->set("request", req);
    REQUIRE(tick_until_status(bt, BT::NodeStatus::FAILURE,
      TestServiceBtNode::BASE_TIMEOUT* 2));

    server_wait.set_value();
    bt.haltTree();
    test_server.reset();
  }

  SECTION("bt fails when server returns failure")
  {
    test_server =
      test_node->create_service<example_interfaces::srv::SetBool>("set_bool",
        [](example_interfaces::srv::SetBool::Request::ConstSharedPtr
        req,
        example_interfaces::srv::SetBool::Response::SharedPtr resp)
        {
          resp->success = req->data;
        });

    uut_fixture.spin_in_background();
    test_fixture.spin_in_background();
    auto req = std::make_shared<example_interfaces::srv::SetBool::Request>();
    req->data = false;
    bt_config.blackboard->set("request", req);
    REQUIRE(tick_until_status(bt, BT::NodeStatus::FAILURE,
      std::chrono::seconds(1)));

    SECTION("bt success when server returns success")
    {
      auto req = std::make_shared<example_interfaces::srv::SetBool::Request>();
      req->data = true;
      bt_config.blackboard->set("request", req);
      REQUIRE(tick_until_status(bt, BT::NodeStatus::SUCCESS,
        std::chrono::seconds(1)));
    }
  }
}

}
