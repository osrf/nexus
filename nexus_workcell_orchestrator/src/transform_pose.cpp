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

#include "transform_pose.hpp"

#include <nexus_capabilities/conversions/pose_stamped.hpp>
#include <nexus_common/pretty_print.hpp>

#include <rclcpp/logging.hpp>

#include <tf2/LinearMath/Transform.h>

namespace nexus::workcell_orchestrator {

BT::NodeStatus TransformPoseLocal::tick()
{
  auto base_pose = this->getInput<geometry_msgs::msg::PoseStamped>("base_pose");
  if (!base_pose)
  {
    RCLCPP_ERROR(
      this->_node.get_logger(), "[%s]: [base_pose] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  tf2::Transform base_trans(tf2::Quaternion(base_pose->pose.orientation.x,
    base_pose->pose.orientation.y, base_pose->pose.orientation.z,
    base_pose->pose.orientation.w),
    tf2::Vector3(base_pose->pose.position.x, base_pose->pose.position.y,
    base_pose->pose.position.z));

  tf2::Transform t;
  tf2::Transform target_t = tf2::Transform::getIdentity();
  auto maybe_pose = this->getInput<geometry_msgs::msg::Pose>(
    "transform_from_pose");
  auto maybe_pose_stamped = this->getInput<geometry_msgs::msg::PoseStamped>(
    "transform_from_pose_stamped");

  auto reverse = this->getInput<bool>("reverse").value_or(false);
  if (maybe_pose_stamped)
  {
    const auto& pose = *maybe_pose_stamped;
    t = this->_pose_to_tf(pose.pose, reverse);
    try
    {
      auto transform_msg = this->_tf2_buffer->lookupTransform(
        pose.header.frame_id, pose.header.stamp,
        base_pose->header.frame_id, base_pose->header.stamp,
        pose.header.frame_id);
      target_t =
        tf2::Transform(tf2::Quaternion(transform_msg.transform.rotation.x,
          transform_msg.transform.rotation.y,
          transform_msg.transform.rotation.z,
          transform_msg.transform.rotation.w),
          tf2::Vector3(transform_msg.transform.translation.x,
          transform_msg.transform.translation.y,
          transform_msg.transform.translation.z)) * base_trans.inverse();
    }
    catch (const tf2::TransformException& e)
    {
      RCLCPP_ERROR(
        this->_node.get_logger(), "[%s]: Failed to get transform [%s]",
        this->name().c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  else if (maybe_pose)
  {
    t = this->_pose_to_tf(*maybe_pose);
  }
  else
  {
    auto x = this->getInput<double>("x");
    auto y = this->getInput<double>("y");
    auto z = this->getInput<double>("z");
    auto qx = this->getInput<double>("qx");
    auto qy = this->getInput<double>("qy");
    auto qz = this->getInput<double>("qz");
    auto qw = this->getInput<double>("qw");

    if (!x || !y || !z || !qx || !qy || !qz || !qw)
    {
      RCLCPP_ERROR(
        this->_node.get_logger(), "[%s]: Either [transform_from_pose_stamped], [transform_from_pose], or [x, y, z, qx, qy, qz, qw] ports are required.",
        this->name().c_str());
      return BT::NodeStatus::FAILURE;
    }

    t =
      tf2::Transform(tf2::Quaternion(*qx, *qy, *qz, *qw),
        tf2::Vector3(*x, *y, *z));
  }

  auto result_trans = base_trans * target_t.inverse() * t * target_t;
  geometry_msgs::msg::PoseStamped result_pose = *base_pose;
  result_pose.pose.position.x = result_trans.getOrigin().x();
  result_pose.pose.position.y = result_trans.getOrigin().y();
  result_pose.pose.position.z = result_trans.getOrigin().z();
  result_pose.pose.orientation.x = result_trans.getRotation().x();
  result_pose.pose.orientation.y = result_trans.getRotation().y();
  result_pose.pose.orientation.z = result_trans.getRotation().z();
  result_pose.pose.orientation.w = result_trans.getRotation().w();
  RCLCPP_INFO_STREAM(
    this->_node.get_logger(),
    "[" << this->name() << "]" << std::endl <<
      "Base pose: " << base_pose->pose << " frame: " << base_pose->header.frame_id << std::endl <<
      "Transform: " << t << std::endl <<
      "Result: " << result_pose.pose <<  " frame: " << result_pose.header.frame_id
    );
  this->setOutput("result", result_pose);
  return BT::NodeStatus::SUCCESS;
}

tf2::Transform TransformPoseLocal::_pose_to_tf(
  const geometry_msgs::msg::Pose& pose, bool reverse)
{
  if (!reverse)
  {
    return tf2::Transform(tf2::Quaternion(pose.orientation.x,
        pose.orientation.y, pose.orientation.z,
        pose.orientation.w),
        tf2::Vector3(pose.position.x, pose.position.y,
        pose.position.z));
  }
  else
  {
    return tf2::Transform(tf2::Quaternion(pose.orientation.x,
        pose.orientation.y, pose.orientation.z,
        pose.orientation.w).inverse(),
        tf2::Vector3(-pose.position.x, -pose.position.y,
        -pose.position.z));
  }
}

}
