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
#include <nexus_capabilities/conversions/pose_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

namespace nexus::workcell_orchestrator {

/**
 * Transform a stamped pose.
 *
 * When `transform_from_pose` is used, the pose is treated as a transform with the
 * position as translation and the orientation as rotation (in quaternion).
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
 *   transform |geometry_msgs::msg::Transform| The direct values are ignored if this is provided.
 *   local |bool| Perform a local transform instead of wrt to the parent frame.
 * Output Ports:
 *   result |geometry_msgs::msg::PoseStamped|
 */
class ApplyPoseOffset : public BT::SyncActionNode
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
      BT::InputPort<geometry_msgs::msg::Transform>("transform",
        "The direct values are ignored if this is provided."),
      BT::InputPort<bool>("local"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("result"),
    };
  }

public: ApplyPoseOffset(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode& node)
  : BT::SyncActionNode(name, config), _node(node) {}

public: BT::NodeStatus tick() override;

private: rclcpp_lifecycle::LifecycleNode& _node;
};

/**
 * Given a `base_pose` and `target_pose`, get the transform `t` such that
 * `target_pose = t * base_pose`.
 */
class GetPoseOffset : public BT::SyncActionNode
{
public: static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("base_pose"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
      BT::InputPort<rclcpp::Time>("time",
        "OPTIONAL, timepoint to lookup on, if not provided, the current time is used."),
      BT::InputPort<bool>("local",
        "OPTIONAL, if true, return t such that `target_pose = base_pose * t`."),
      BT::OutputPort<geometry_msgs::msg::Transform>("result"),
    };
  }

public: GetPoseOffset(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode& node,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer)
  : BT::SyncActionNode(name, config), _node(node),
    _tf2_buffer(std::move(tf2_buffer)) {}

public: BT::NodeStatus tick() override;

private: rclcpp_lifecycle::LifecycleNode& _node;
private: std::shared_ptr<tf2_ros::Buffer> _tf2_buffer;
};

}

#endif
