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

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__TRANSFORM_POSE_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__TRANSFORM_POSE_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

namespace nexus::workcell_orchestrator {

/**
 * Transform a stamped pose.
 *
 * When the inputs ((x,y,z),(qx,qy,qz,qw)) or transform_from_pose are used
 * this node assumes the input poses are expressed in the frame defined by
 * base_pose. The node transforms the input into the parent frame of
 * base_pose. For example, if base_pose is ((1, 0, 0), (0, 0, 0, 1)) in frame
 * "map" and the input is ((2, 0, 0), (0, 0, 0, 1)) then the output of the
 * node is ((3, 0, 0), (0, 0, 0, 1)) in frame "map".
 *
 * When transform_from_pose_stamped is used, the meaning of the output
 * transformation is different from above, but it's frame ID is set to the
 * frame of the base_pose.
 * The behavior being different from the above may change in the future, and
 * it probably shouldn't be relied on.
 *
 * Input Ports:
 *   base_pose |geometry_msgs::msg::PoseStamped| Used as a pose to transform inputs into.
 *   x |double|
 *   y |double|
 *   z |double|
 *   qx |double|
 *   qy |double|
 *   qz |double|
 *   qw |double|
 *   transform_from_pose |geometry_msgs::msg::Pose| Use the pose as the transform parameters.
 *      The direct values are ignored if this is provided.
 *   transform_from_pose_stamped |geometry_msgs::msg::PoseStamped| Takes precedence of `transform_from_pose`.
 *   reverse |bool| Applies reverse of the transform instead, defaults to `false`. Only valid when `transform_from_pose` or `transform_from_pose_stamped` is given.
 * Output Ports:
 *   result |geometry_msgs::msg::Transform|
 */
class TransformPoseLocal : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("base_pose"),
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("z"),
      BT::InputPort<double>("qx"),
      BT::InputPort<double>("qy"),
      BT::InputPort<double>("qz"),
      BT::InputPort<double>("qw"),
      BT::InputPort<geometry_msgs::msg::Pose>("transform_from_pose",
        "Use the pose as the transform parameters. The direct values are ignored if this is provided."),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "transform_from_pose_stamped",
        "Takes precedence of `transform_from_pose`."),
      BT::InputPort<bool>("reverse",
        "Applies reverse of the transform instead, defaults to `false`. Only valid when `transform_from_pose` or `transform_from_pose_stamped` is given."),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("result"),
    };
  }

public: TransformPoseLocal(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode& node,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer)
  : BT::SyncActionNode(name, config), _node(node),
    _tf2_buffer(std::move(tf2_buffer)) {}

public: BT::NodeStatus tick() override;

private: rclcpp_lifecycle::LifecycleNode& _node;
private: std::shared_ptr<tf2_ros::Buffer> _tf2_buffer;

private: tf2::Transform _pose_to_tf(const geometry_msgs::msg::Pose& pose,
    bool reverse = false);
};

}

#endif
