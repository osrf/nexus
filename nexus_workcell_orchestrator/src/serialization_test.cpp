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

#include "serialization.hpp"

#include <nexus_common_test/test_utils.hpp>

#include <vision_msgs/msg/detection3_d_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <rmf_utils/catch.hpp>

namespace nexus::workcell_orchestrator::test {

TEST_CASE("pose serialization") {
  common::test::RosFixture<rclcpp_lifecycle::LifecycleNode> fixture;

  BT::BehaviorTreeFactory bt_factory;
  bt_factory.registerBuilder<SerializeDetections>("SerializeDetections",
    [&](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<SerializeDetections>(name, config, *fixture.node);
    });
  bt_factory.registerBuilder<DeserializeDetections>("DeserializeDetections",
    [&](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<DeserializeDetections>(name, config,
      *fixture.node);
    });

  auto bt = bt_factory.createTreeFromText(
    R"(
    <?xml version='1.0' ?>
    <root main_tree_to_execute="Test">
      <BehaviorTree ID="Test">
        <Sequence>
          <SerializeDetections detections="{detections}" result="{yaml}" />
          <DeserializeDetections yaml="{yaml}" result="{new_detections}" />
        </Sequence>
      </BehaviorTree>
    </root>
  )");

  vision_msgs::msg::Detection3DArray detections;
  auto& detection = detections.detections.emplace_back();
  auto& result = detection.results.emplace_back();
  auto& pose = result.pose;
  pose.pose.position.x = 1;
  pose.pose.position.y = 2;
  pose.pose.position.z = 3;
  bt.rootBlackboard()->set("detections", detections);

  bt.tickRoot();
  auto new_detections =
    bt.rootBlackboard()->get<vision_msgs::msg::Detection3DArray>(
    "new_detections");
  REQUIRE(new_detections.detections.size() == 1);
  const auto& new_detection = new_detections.detections[0];
  REQUIRE(new_detection.results.size() == 1);
  const auto& new_result = new_detection.results[0];
  const auto& new_pose = new_result.pose;
  CHECK(new_pose.pose.position.x == 1);
  CHECK(new_pose.pose.position.y == 2);
  CHECK(new_pose.pose.position.z == 3);
}

}
