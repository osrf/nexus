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

#include "transform_pose.hpp"

#include <nexus_common_test/test_utils.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace nexus::workcell_orchestrator::test {

TEST_CASE("TransformPoseLocal") {
  common::test::RosFixture<rclcpp_lifecycle::LifecycleNode> fixture;
  auto tf2_buffer =
    std::make_shared<tf2_ros::Buffer>(fixture.node->get_clock());
  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<TransformPoseLocal>("TransformPoseLocal",
    [&](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<TransformPoseLocal>(name, config, *fixture.node,
      tf2_buffer);
    });

  SECTION(
    "transform without frame should implicitly uses the frame of base pose") {
    auto bt = factory.createTreeFromText(
      R"(
      <root>
        <BehaviorTree>
          <TransformPoseLocal base_pose="{base_pose}" transform_from_pose="{transform_from_pose}" result="{result}" />
        </BehaviorTree>
      </root>
    )");

    geometry_msgs::msg::PoseStamped base_pose;
    base_pose.header.frame_id = "test";
    base_pose.pose.position.x = 1;
    base_pose.pose.position.y = 2;
    base_pose.pose.position.z = 3;
    base_pose.pose.orientation.x = 1;
    base_pose.pose.orientation.y = 0;
    base_pose.pose.orientation.z = 0;
    base_pose.pose.orientation.w = 0;
    bt.blackboard_stack.front()->set("base_pose", base_pose);
    geometry_msgs::msg::Pose transform_from_pose;
    transform_from_pose.position.x = 3;
    transform_from_pose.position.y = 2;
    transform_from_pose.position.z = 1;
    transform_from_pose.orientation.x = -1;
    transform_from_pose.orientation.y = 0;
    transform_from_pose.orientation.z = 0;
    transform_from_pose.orientation.w = 0;
    bt.blackboard_stack.front()->set("transform_from_pose",
      transform_from_pose);
    REQUIRE(bt.tickRoot() == BT::NodeStatus::SUCCESS);
    auto result =
      bt.blackboard_stack.front()->get<geometry_msgs::msg::PoseStamped>(
      "result");
    CHECK(result.header.frame_id == "test");
    CHECK(result.pose.position.x == Approx(4));
    CHECK(result.pose.position.y == Approx(0));
    CHECK(result.pose.position.z == Approx(2));
    CHECK(result.pose.orientation.x == Approx(0));
    CHECK(result.pose.orientation.y == Approx(0));
    CHECK(result.pose.orientation.z == Approx(0));
    CHECK(result.pose.orientation.w == Approx(1));
  }

  SECTION("transform from pose stamped") {
    auto bt = factory.createTreeFromText(
      R"(
      <root>
        <BehaviorTree>
          <TransformPoseLocal base_pose="{base_pose}" transform_from_pose_stamped="{transform_from_pose_stamped}" result="{result}" />
        </BehaviorTree>
      </root>
    )");

    geometry_msgs::msg::PoseStamped base_pose;
    base_pose.header.frame_id = "test_a";
    base_pose.pose.position.x = 1;
    base_pose.pose.position.y = 2;
    base_pose.pose.position.z = 3;
    base_pose.pose.orientation.x = 1;
    base_pose.pose.orientation.y = 0;
    base_pose.pose.orientation.z = 0;
    base_pose.pose.orientation.w = 0;
    bt.blackboard_stack.front()->set("base_pose", base_pose);
    geometry_msgs::msg::PoseStamped transform_from_pose_stamped;
    transform_from_pose_stamped.header.frame_id = "test_b";
    transform_from_pose_stamped.pose.position.x = 3;
    transform_from_pose_stamped.pose.position.y = 2;
    transform_from_pose_stamped.pose.position.z = 1;
    transform_from_pose_stamped.pose.orientation.x = -1;
    transform_from_pose_stamped.pose.orientation.y = 0;
    transform_from_pose_stamped.pose.orientation.z = 0;
    transform_from_pose_stamped.pose.orientation.w = 0;
    bt.blackboard_stack.front()->set("transform_from_pose_stamped",
      transform_from_pose_stamped);

    SECTION("fail when target frame cannot be found") {
      geometry_msgs::msg::TransformStamped test_a;
      test_a.header.frame_id = "base_link";
      test_a.child_frame_id = "test_a";
      tf2_buffer->setTransform(test_a, "test");
      CHECK(bt.tickRoot() == BT::NodeStatus::FAILURE);
    }

    SECTION("fail when base frame cannot be found") {
      geometry_msgs::msg::TransformStamped test_b;
      test_b.header.frame_id = "base_link";
      test_b.child_frame_id = "test_b";
      tf2_buffer->setTransform(test_b, "test");
      CHECK(bt.tickRoot() == BT::NodeStatus::FAILURE);
    }

    SECTION("success when both base and target frame is available") {
      geometry_msgs::msg::TransformStamped test_a;
      test_a.header.frame_id = "base_link";
      test_a.child_frame_id = "test_a";
      tf2_buffer->setTransform(test_a, "test");

      geometry_msgs::msg::TransformStamped test_b;
      test_b.header.frame_id = "base_link";
      test_b.child_frame_id = "test_b";
      tf2_buffer->setTransform(test_b, "test");

      REQUIRE(bt.tickRoot() == BT::NodeStatus::SUCCESS);
      auto result =
        bt.blackboard_stack.front()->get<geometry_msgs::msg::PoseStamped>(
        "result");
      CHECK(result.header.frame_id == "test_a");
      CHECK(result.pose.position.x == Approx(4));
      CHECK(result.pose.position.y == Approx(0));
      CHECK(result.pose.position.z == Approx(-2));
      CHECK(result.pose.orientation.x == Approx(0));
      CHECK(result.pose.orientation.y == Approx(0));
      CHECK(result.pose.orientation.z == Approx(0));
      CHECK(result.pose.orientation.w == Approx(1));
    }
  }
}

}
