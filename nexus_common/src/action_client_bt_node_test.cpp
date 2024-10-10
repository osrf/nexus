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
#include <rmf_utils/catch.hpp>

#include "nexus_common/action_client_bt_node.hpp"
#include "nexus_common_test/test_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <example_interfaces/action/fibonacci.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>

#include <chrono>
#include <memory>
#include <thread>

namespace nexus::common::test {

class TestActionBtNode : public ActionClientBtNode<rclcpp::Node::SharedPtr,
    example_interfaces::action::Fibonacci>
{
public: static BT::PortsList providedPorts()
  {
    return {BT::InputPort<example_interfaces::action::Fibonacci::Goal>("goal")};
  }


public: TestActionBtNode(const std::string& name,
    const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node,
    rclcpp_action::Client<example_interfaces::action::Fibonacci>::SharedPtr client)
  : ActionClientBtNode<rclcpp::Node::SharedPtr,
      example_interfaces::action::Fibonacci>(name, config, node),
    _client(client) {}

protected: std::string get_action_name() const override
  {
    return "test_action";
  }

protected: std::optional<example_interfaces::action::Fibonacci::Goal> make_goal()
  override
  {
    return example_interfaces::action::Fibonacci::Goal();
  }

private: rclcpp_action::Client<example_interfaces::action::Fibonacci>::SharedPtr
    _client;
};

TEST_CASE("ActionClientBtNode", "[BehaviorTree]")
{
  common::test::RosFixture<rclcpp::Node> uut_fixture;
  auto& uut_node = uut_fixture.node;
  auto client =
    rclcpp_action::create_client<example_interfaces::action::Fibonacci>(
    uut_node, "test_action");

  BT::BehaviorTreeFactory bt_factory;
  bt_factory.registerBuilder<TestActionBtNode>("Test",
    [&](const std::string& name,
    const BT::NodeConfiguration& config)
    {
      return std::make_unique<TestActionBtNode>(name, config, uut_node, client);
    });
  auto bt = bt_factory.createTreeFromText(
    R"RAW(
    <?xml version='1.0' ?>
    <root main_tree_to_execute="Test">
      <BehaviorTree ID="Test">
        <Test />
      </BehaviorTree>
    </root>
  )RAW");

  uut_fixture.spin_in_background();

  SECTION("bt fails if server is not found")
  {
    CHECK(bt.tickRootWhileRunning(std::chrono::milliseconds(
        10)) == BT::NodeStatus::FAILURE);
  }

  common::test::RosFixture<rclcpp::Node> test_fixture;
  auto& test_node = test_fixture.node;
  rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr
    test_server;

  SECTION("bt fails if server rejects goal")
  {
    std::promise<void> rejected_goal;
    auto rejected_goal_fut = rejected_goal.get_future();

    test_server =
      rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
      test_node, "test_action",
      [](const rclcpp_action::GoalUUID&,
      example_interfaces::action::Fibonacci::Goal::ConstSharedPtr)
      {
        return rclcpp_action::GoalResponse::REJECT;
      },
      [&](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>>)
      {
        rejected_goal.set_value();
        return rclcpp_action::CancelResponse::REJECT;
      },
      [](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>>
        handle)
      {
        handle->abort(
          std::make_shared<example_interfaces::action::Fibonacci::Result>());
      });

    test_fixture.spin_in_background();
    CHECK(bt.tickRootWhileRunning(std::chrono::milliseconds(
        10)) == BT::NodeStatus::FAILURE);
  }

  SECTION("bt fails if server returns failure")
  {
    test_server =
      rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
      test_node, "test_action",
      [](const rclcpp_action::GoalUUID&,
      example_interfaces::action::Fibonacci::Goal::ConstSharedPtr)
      {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>>)
      {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>>
        handle)
      {
        handle->abort(
          std::make_shared<example_interfaces::action::Fibonacci::Result>());
      });

    test_fixture.spin_in_background();
    CHECK(bt.tickRootWhileRunning(std::chrono::milliseconds(
        10)) == BT::NodeStatus::FAILURE);
  }

  SECTION("cancel action when bt is halted")
  {
    std::promise<void> started_job;
    auto started_job_fut = started_job.get_future();
    std::promise<void> got_cancel;
    auto got_cancel_fut = got_cancel.get_future();
    std::thread job_thread;
    test_server =
      rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
      test_node, "test_action",
      [](const rclcpp_action::GoalUUID&,
      example_interfaces::action::Fibonacci::Goal::ConstSharedPtr)
      {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [&](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>>)
      {
        got_cancel.set_value();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [&](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>>
        handle)
      {
        started_job.set_value();
        job_thread = std::thread([&, handle]()
        {
          while (!handle->is_canceling())
          {
          }
          handle->canceled(
            std::make_shared<example_interfaces::action::Fibonacci::Result>());
        });
      });

    test_fixture.spin_in_background();
    tick_until(bt, [&](auto)
      {
        return started_job_fut.wait_for(std::chrono::seconds(
          0)) == std::future_status::ready;
      }, std::chrono::seconds(1));
    bt.haltTree();
    CHECK(got_cancel_fut.wait_for(std::chrono::seconds(0)) ==
      std::future_status::ready);
    job_thread.join();
  }

  // flaky test
  SECTION("waits for result if cancellation is rejected")
  {
    std::promise<void> started_job;
    auto started_job_fut = started_job.get_future();
    std::thread job_thread;
    test_server =
      rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
      test_node, "test_action",
      [](const rclcpp_action::GoalUUID&,
      example_interfaces::action::Fibonacci::Goal::ConstSharedPtr)
      {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [&](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>>)
      {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [&](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>>
        handle)
      {
        started_job.set_value();
        job_thread = std::thread([&, handle]()
        {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          auto result =
          std::make_shared<example_interfaces::action::Fibonacci::Result>();
          handle->succeed(result);
        });
      });

    test_fixture.spin_in_background();
    tick_until(bt, [&](auto)
      {
        return started_job_fut.wait_for(std::chrono::seconds(
          0)) == std::future_status::ready;
      }, std::chrono::seconds(1));

    // test action takes 1 second, halting the tree should cause it to block until
    // the action completes.
    auto prehalt = std::chrono::steady_clock::now();
    bt.haltTree();
    auto posthalt = std::chrono::steady_clock::now();
    CHECK(posthalt - prehalt > std::chrono::milliseconds(500));
    job_thread.join();
  }

  SECTION("bt succeed if server returns success")
  {
    test_server =
      rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
      test_node, "test_action",
      [](const rclcpp_action::GoalUUID&,
      example_interfaces::action::Fibonacci::Goal::ConstSharedPtr)
      {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>>)
      {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [](
        std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>>
        handle)
      {
        handle->succeed(
          std::make_shared<example_interfaces::action::Fibonacci::Result>());
      });

    test_fixture.spin_in_background();
    CHECK(tick_until_status(bt, BT::NodeStatus::SUCCESS, std::chrono::seconds(
        1)));
  }
}

}
