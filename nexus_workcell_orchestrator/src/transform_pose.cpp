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

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>

namespace nexus::workcell_orchestrator {

BT::NodeStatus ApplyTransform::tick()
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
  auto maybe_tf = this->getInput<geometry_msgs::msg::Transform>("transform");

  if (maybe_tf)
  {
    tf2::fromMsg(*maybe_tf, t);
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

  tf2::Transform  result_trans;
  const auto maybe_local = this->getInput<bool>("local");
  if (maybe_local && *maybe_local)
  {
    result_trans = base_trans * t;
  }
  else
  {
    result_trans = t * base_trans;
  }
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

static tf2::Transform _pose_to_tf(const geometry_msgs::msg::Pose& pose)
{
  return tf2::Transform(tf2::Quaternion(pose.orientation.x,
      pose.orientation.y, pose.orientation.z,
      pose.orientation.w),
      tf2::Vector3(pose.position.x, pose.position.y,
      pose.position.z));
}

BT::NodeStatus GetTransform::tick()
{
  const auto base_pose = this->getInput<geometry_msgs::msg::PoseStamped>(
    "base_pose");
  if (!base_pose)
  {
    RCLCPP_ERROR(
      this->_node.get_logger(), "[%s]: [base_pose] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const auto base_tf = _pose_to_tf(base_pose->pose);
  const auto target_pose = this->getInput<geometry_msgs::msg::PoseStamped>(
    "target_pose");
  if (!target_pose)
  {
    RCLCPP_ERROR(
      this->_node.get_logger(), "[%s]: [target_pose] port is required",
      this->name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const auto target_tf = _pose_to_tf(target_pose->pose);

  rclcpp::Time lookup_time;
  const auto maybe_time = this->getInput<rclcpp::Time>("time");
  if (maybe_time)
  {
    lookup_time = *maybe_time;
  }
  else
  {
    lookup_time = rclcpp::Time(0);  // Get latest available transforms
  }

  tf2::Transform frame_tf;
  const auto handle_exception = [this](const std::exception& e)
    {
      RCLCPP_ERROR(this->_node.get_logger(), "%s", e.what());
      return BT::NodeStatus::FAILURE;
    };
  try
  {
    const auto tf_msg = this->_tf2_buffer->lookupTransform(
      base_pose->header.frame_id, target_pose->header.frame_id,
      lookup_time);
    tf2::fromMsg(tf_msg.transform, frame_tf);
  }
  catch (const tf2::LookupException& e)
  {
    return handle_exception(e);
  }
  catch (const tf2::ConnectivityException& e)
  {
    return handle_exception(e);
  }
  catch (const tf2::ExtrapolationException& e)
  {
    return handle_exception(e);
  }
  catch (const tf2::InvalidArgumentException& e)
  {
    return handle_exception(e);
  }

  const auto maybe_local = this->getInput<bool>("local");
  tf2::Transform result_tf;
  if (maybe_local && *maybe_local)
  {
    result_tf = base_tf.inverse() * target_tf * frame_tf;
  }
  else
  {
    result_tf = target_tf * frame_tf * base_tf.inverse();
  }
  RCLCPP_INFO_STREAM(
    this->_node.get_logger(),
    "[" << this->name() << "]" << std::endl <<
      "Base pose: " << base_pose->pose << " frame: " << base_pose->header.frame_id << std::endl <<
      "Target pose: " << target_pose->pose << "frame: " << target_pose->header.frame_id << std::endl <<
      "Result: " << result_tf;
  );
  this->setOutput("result", tf2::toMsg(result_tf));

  return BT::NodeStatus::SUCCESS;
}

}
