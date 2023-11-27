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

#include "plan_motion.hpp"

#include <nexus_common/pretty_print.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>

namespace nexus::capabilities {

endpoints::GetMotionPlanService::ServiceType::Request::SharedPtr PlanMotion::
make_request()
{
  auto req =
    std::make_shared<endpoints::GetMotionPlanService::ServiceType::Request>();

  // If a previous result is provided, set start joint to final joint states from this result.
  const auto maybe_start_constraints =
    this->getInput<std::vector<moveit_msgs::msg::JointConstraint>>(
    "start_constraints");
  if (maybe_start_constraints.has_value())
  {
    auto start_constraints =
      maybe_start_constraints.value();
    if (start_constraints.empty())
    {
      RCLCPP_ERROR(
        this->_logger,
        "Empty start_constraints provided. "
        "Defaulting to START_TYPE_CURRENT."
      );
      req->start_type =
        endpoints::GetMotionPlanService::ServiceType::Request::
        START_TYPE_CURRENT;
    }
    else
    {
      req->start_joints = std::move(start_constraints);
      req->start_type =
        endpoints::GetMotionPlanService::ServiceType::Request::START_TYPE_JOINTS;
    }
  }
  else
  {
    req->start_type =
      endpoints::GetMotionPlanService::ServiceType::Request::START_TYPE_CURRENT;
  }
  req->goal_type =
    endpoints::GetMotionPlanService::ServiceType::Request::GOAL_TYPE_POSE;

  auto goal_pose = this->getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal_pose)
  {
    RCLCPP_ERROR(this->_logger, "%s: port [goal] is required",
      this->name().c_str());
    return nullptr;
  }
  req->goal_pose = *goal_pose;

  // publish the planning target to tf
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = req->goal_pose.header.frame_id;
  tf.header.stamp = this->_node.now();
  tf.child_frame_id = "planning_target";
  tf.transform.translation.x = req->goal_pose.pose.position.x;
  tf.transform.translation.y = req->goal_pose.pose.position.y;
  tf.transform.translation.z = req->goal_pose.pose.position.z;
  tf.transform.rotation.x = req->goal_pose.pose.orientation.x;
  tf.transform.rotation.y = req->goal_pose.pose.orientation.y;
  tf.transform.rotation.z = req->goal_pose.pose.orientation.z;
  tf.transform.rotation.w = req->goal_pose.pose.orientation.w;
  this->_tf_broadcaster->sendTransform(tf);

  auto robot_name = this->getInput<std::string>("robot_name");
  if (!robot_name)
  {
    RCLCPP_ERROR(this->_logger, "%s: port [robot_name] is required",
      this->name().c_str());
    return nullptr;
  }
  req->robot_name = *robot_name;

  auto maybe_cartesian = this->getInput<bool>("cartesian");
  if (maybe_cartesian.has_value())
  {
    req->cartesian = maybe_cartesian.value();
  }
  else
  {
    req->cartesian = false;
  }

  double scale_speed = 1.0;
  auto maybe_scale_speed = this->getInput<double>("scale_speed");
  if (maybe_scale_speed.has_value())
  {
    scale_speed = maybe_scale_speed.value();
  }
  req->max_velocity_scaling_factor = scale_speed;
  req->max_acceleration_scaling_factor = scale_speed;

  auto maybe_read_only =
    this->getInput<bool>("force_cache_mode_execute_read_only");
  if (maybe_read_only.has_value())
  {
    req->force_cache_mode_execute_read_only = maybe_read_only.value();
  }
  else
  {
    req->force_cache_mode_execute_read_only = false;
  }

  RCLCPP_DEBUG_STREAM(this->_logger,
    "planning to " << req->goal_pose.pose << " at frame " <<
      req->goal_pose.header.frame_id << " with cartesian " << req->cartesian << "and scaled speed of " <<
      scale_speed);

  return req;
}

bool PlanMotion::on_response(
  rclcpp::Client<endpoints::GetMotionPlanService::ServiceType>::SharedResponse resp)
{
  if (resp->result.error_code.val !=
    moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    std::string reason =
      resp->message.empty() ? "unknown error" : resp->message;
    RCLCPP_ERROR(this->_logger, "Failed to plan motion: %s", reason.c_str());
    return false;
  }
  this->setOutput("result", resp->result.trajectory);
  return true;
}

}
