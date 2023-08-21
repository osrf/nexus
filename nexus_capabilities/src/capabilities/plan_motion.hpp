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

#ifndef NEXUS_CAPABILITIES__CAPABILITIES__PLAN_MOTION_HPP
#define NEXUS_CAPABILITIES__CAPABILITIES__PLAN_MOTION_HPP

#include <nexus_capabilities/conversions/pose_stamped.hpp>
#include <nexus_common/service_client_bt_node.hpp>
#include <nexus_endpoints.hpp>

#include <behaviortree_cpp_v3/tree_node.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <memory>

namespace nexus::capabilities {

/**
 * Plan motion for a robot arm.
 *
 * Input Ports:
 *   robot_name |std::string| The robot name to plan for.
 *   goal |geometry_msgs::msg::PoseStamped| The pose of the goal, can either be a PoseStamped or frame id.
 *   cartesian |bool| OPTIONAL. Set true to generate a cartesian plan to the goal.
 *   scale_speed | double | OPTIONAL. An override for the velocity and acceleration scale for the motion plan. This should be within (0, 1].
 *   start_constraints |std::vector<moveit_msgs::msg::JointConstraints>| OPTIONAL. If provided the the joint_constraints are used as the start condition.
 *   Else, the current state of the robot will be used as the start.
 * Output Ports:
 *   result |moveit_msgs::msg::RobotTrajectory| The resulting trajectory.
 */
class PlanMotion : public common::
  ServiceClientBtNode<endpoints::GetMotionPlanService::ServiceType>
{
  /**
   * Defines provided ports for the BT node.
   */
public: static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("robot_name", "The robot name to plan for."),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal",
        "The pose of the goal, can either be a PoseStamped or frame id."),
      BT::InputPort<bool>("cartesian",
        "OPTIONAL. Set true to generate a cartesian plan to the goal."),
      BT::InputPort<double>("scale_speed",
        "OPTIONAL. An override for the velocity and acceleration scale for the motion plan. This should be within (0, 1]."),
      BT::InputPort<std::vector<moveit_msgs::msg::JointConstraint>>(
        "start_constraints",
        "OPTIONAL. If provided the the joint_constraints are used as the start condition. Else, the current state of the robot will be used as the start."),
      BT::OutputPort<moveit_msgs::msg::RobotTrajectory>(
        "result", "The resulting trajectory.")};
  }

  /**
   * @param name name of the BT node.
   * @param config config of the BT node.
   * @param node rclcpp lifecycle node.
   * @param client the rclcpp client to use.
   */
public: PlanMotion(const std::string& name,
    const BT::NodeConfiguration& config,
    rclcpp_lifecycle::LifecycleNode& node,
    rclcpp::Client<endpoints::GetMotionPlanService::ServiceType>::SharedPtr client,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster)
  : ServiceClientBtNode<endpoints::GetMotionPlanService::ServiceType>(name,
      config, node.get_logger(), std::chrono::minutes{1}), _node{node},
    _client(client),
    _tf_broadcaster(tf_broadcaster) {}

protected: rclcpp::Client<endpoints::GetMotionPlanService::ServiceType>::
  SharedPtr
  client() final
  {
    return this->_client;
  }

protected: endpoints::GetMotionPlanService::ServiceType::Request::SharedPtr
  make_request() final;

protected: bool on_response(
    rclcpp::Client<endpoints::GetMotionPlanService::ServiceType>::SharedResponse resp)
  final;

private: rclcpp_lifecycle::LifecycleNode& _node;
private: rclcpp::Client<endpoints::GetMotionPlanService::ServiceType>::SharedPtr
    _client;
private: std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
};

}

#endif
