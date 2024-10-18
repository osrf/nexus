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
#include <rmf_utils/catch.hpp>

#include "transform_pose.hpp"

#include <nexus_common_test/test_utils.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace nexus::workcell_orchestrator::test {

TEST_CASE("ApplyTransform") {
  common::test::RosFixture<rclcpp_lifecycle::LifecycleNode> fixture;
  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<ApplyTransform>("ApplyTransform",
    [&](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<ApplyTransform>(name, config, *fixture.node);
    });

  SECTION("local transform") {
    auto bt = factory.createTreeFromText(
      R"(
      <root>
        <BehaviorTree>
          <ApplyTransform base_pose="{base_pose}" transform="{transform}" local="true" result="{result}" />
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
    geometry_msgs::msg::Transform transform;
    transform.translation.x = 3;
    transform.translation.y = 2;
    transform.translation.z = 1;
    transform.rotation.x = -1;
    transform.rotation.y = 0;
    transform.rotation.z = 0;
    transform.rotation.w = 0;
    bt.blackboard_stack.front()->set("transform", transform);
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

  SECTION("parent frame transform") {
    auto bt = factory.createTreeFromText(
      R"(
      <root>
        <BehaviorTree>
          <ApplyTransform base_pose="{base_pose}" transform="{transform}" local="false" result="{result}" />
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
    geometry_msgs::msg::Transform transform;
    transform.translation.x = 3;
    transform.translation.y = 2;
    transform.translation.z = 1;
    transform.rotation.x = -1;
    transform.rotation.y = 0;
    transform.rotation.z = 0;
    transform.rotation.w = 0;
    bt.blackboard_stack.front()->set("transform", transform);
    REQUIRE(bt.tickRoot() == BT::NodeStatus::SUCCESS);
    auto result =
      bt.blackboard_stack.front()->get<geometry_msgs::msg::PoseStamped>(
      "result");
    CHECK(result.header.frame_id == "test");
    CHECK(result.pose.position.x == Approx(4));
    CHECK(result.pose.position.y == Approx(0));
    CHECK(result.pose.position.z == Approx(-2));
    CHECK(result.pose.orientation.x == Approx(0));
    CHECK(result.pose.orientation.y == Approx(0));
    CHECK(result.pose.orientation.z == Approx(0));
    CHECK(result.pose.orientation.w == Approx(1));
  }
}

TEST_CASE("GetTransform") {
  common::test::RosFixture<rclcpp_lifecycle::LifecycleNode> fixture;
  auto tf2_buffer =
    std::make_shared<tf2_ros::Buffer>(fixture.node->get_clock());
  rclcpp::Time lookup_time(0);
  geometry_msgs::msg::TransformStamped target_tf;
  target_tf.header.stamp = lookup_time;
  target_tf.header.frame_id = "test_base";
  target_tf.child_frame_id = "test_target";
  target_tf.transform.translation.x = 1;
  target_tf.transform.translation.y = 2;
  target_tf.transform.translation.z = 3;
  target_tf.transform.rotation.x = 0;
  target_tf.transform.rotation.y = 0;
  target_tf.transform.rotation.z = 0;
  target_tf.transform.rotation.w = 1;
  tf2_buffer->setTransform(target_tf, fixture.node->get_name());
  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<GetTransform>("GetTransform",
    [&](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<GetTransform>(name, config, *fixture.node,
      tf2_buffer);
    });

  auto bt = factory.createTreeFromText(
    R"(
      <root>
        <BehaviorTree>
          <GetTransform base_pose="{base_pose}" target_pose="{target_pose}" time="{lookup_time}" result="{result}" />
        </BehaviorTree>
      </root>
    )");
  bt.blackboard_stack.front()->set("lookup_time", lookup_time);

  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header.frame_id = "test_base";
  base_pose.pose.position.x = 1;
  base_pose.pose.position.y = 0;
  base_pose.pose.position.z = 0;
  base_pose.pose.orientation.x = 0;
  base_pose.pose.orientation.y = 0;
  base_pose.pose.orientation.z = 0;
  base_pose.pose.orientation.w = 1;
  bt.blackboard_stack.front()->set("base_pose", base_pose);
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "test_target";
  target_pose.pose.position.x = 1;
  target_pose.pose.position.y = 0;
  target_pose.pose.position.z = 0;
  target_pose.pose.orientation.x = 0;
  target_pose.pose.orientation.y = 0;
  target_pose.pose.orientation.z = 0;
  target_pose.pose.orientation.w = 1;
  bt.blackboard_stack.front()->set("target_pose", target_pose);
  REQUIRE(bt.tickRoot() == BT::NodeStatus::SUCCESS);
  auto result =
    bt.blackboard_stack.front()->get<geometry_msgs::msg::Transform>(
    "result");
  CHECK(result.translation.x == Approx(1));
  CHECK(result.translation.y == Approx(2));
  CHECK(result.translation.z == Approx(3));
  CHECK(result.rotation.x == Approx(0));
  CHECK(result.rotation.y == Approx(0));
  CHECK(result.rotation.z == Approx(0));
  CHECK(result.rotation.w == Approx(1));
}

}
