// Copyright 2022 Johnson & Johnson
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <vector>

#include "moveit/robot_state/conversions.h"
#include "moveit/robot_state/robot_state.h"
#include "warehouse_ros/database_loader.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "motion_plan_cache.hpp"

namespace nexus {
namespace motion_planner {

#define NEXUS_MATCH_RANGE(arg, tolerance) \
  arg - tolerance / 2, arg + tolerance / 2

using warehouse_ros::MessageWithMetadata;
using warehouse_ros::Metadata;
using warehouse_ros::Query;

MotionPlanCache::MotionPlanCache(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
  if (!node_->has_parameter("warehouse_plugin"))
  {
    node_->declare_parameter<std::string>(
      "warehouse_plugin", "warehouse_ros_sqlite::DatabaseConnection");
  }

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  warehouse_ros::DatabaseLoader loader(node_);
  db_ = loader.loadDatabase();
}

void MotionPlanCache::init(
  const std::string& db_path, uint32_t db_port, double exact_match_precision)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "Opening motion plan cache database at: %s (Port: %d, Precision: %f)",
    db_path.c_str(), db_port, exact_match_precision);

  exact_match_precision_ = exact_match_precision;
  db_->setParams(db_path, db_port);
  db_->connect();
}

// =============================================================================
// MOTION PLAN CACHING
// =============================================================================
// MOTION PLAN CACHING: TOP LEVEL OPS
std::vector<MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr>
MotionPlanCache::fetch_all_matching_plans(
  const moveit::planning_interface::MoveGroupInterface& move_group,
  const std::string& move_group_namespace,
  const moveit_msgs::msg::MotionPlanRequest& plan_request,
  double start_tolerance, double goal_tolerance, bool metadata_only)
{
  auto coll = db_->openCollection<moveit_msgs::msg::RobotTrajectory>(
    "move_group_plan_cache", move_group_namespace);

  Query::Ptr query = coll.createQuery();

  bool start_ok = this->extract_and_append_plan_start_to_query(
    *query, move_group, plan_request, start_tolerance);
  bool goal_ok = this->extract_and_append_plan_goal_to_query(
    *query, move_group, plan_request, goal_tolerance);

  if (!start_ok || !goal_ok)
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not construct plan query.");
    return {};
  }

  return coll.queryList(
    query, metadata_only, /* sort_by */ "execution_time_s", true);
}

MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr
MotionPlanCache::fetch_best_matching_plan(
  const moveit::planning_interface::MoveGroupInterface& move_group,
  const std::string& move_group_namespace,
  const moveit_msgs::msg::MotionPlanRequest& plan_request,
  double start_tolerance, double goal_tolerance, bool metadata_only)
{
  // First find all matching, but metadata only.
  // Then use the ID metadata of the best plan to pull the actual message.
  auto matching_plans = this->fetch_all_matching_plans(
    move_group, move_group_namespace,
    plan_request, start_tolerance, goal_tolerance,
    true);

  if (matching_plans.empty())
  {
    RCLCPP_INFO(node_->get_logger(), "No matching plans found.");
    return nullptr;
  }

  auto coll = db_->openCollection<moveit_msgs::msg::RobotTrajectory>(
    "move_group_plan_cache", move_group_namespace);

  // Best plan is at first index, since the lookup query was sorted by
  // execution_time.
  int best_plan_id = matching_plans.at(0)->lookupInt("id");
  Query::Ptr best_query = coll.createQuery();
  best_query->append("id", best_plan_id);

  return coll.findOne(best_query, metadata_only);
}

bool
MotionPlanCache::put_plan(
  const moveit::planning_interface::MoveGroupInterface& move_group,
  const std::string& move_group_namespace,
  const moveit_msgs::msg::MotionPlanRequest& plan_request,
  const moveit_msgs::msg::RobotTrajectory& plan,
  double execution_time_s,
  double planning_time_s,
  bool overwrite)
{
  // Check pre-conditions
  if (!plan.multi_dof_joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Skipping insert: Multi-DOF trajectory plans are not supported.");
    return false;
  }
  if (plan_request.workspace_parameters.header.frame_id.empty() ||
    plan.joint_trajectory.header.frame_id.empty())
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Skipping insert: Frame IDs cannot be empty.");
    return false;
  }
  if (plan_request.workspace_parameters.header.frame_id !=
    plan.joint_trajectory.header.frame_id)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Skipping insert: "
      "Plan request frame (%s) does not match plan frame (%s).",
      plan_request.workspace_parameters.header.frame_id.c_str(),
      plan.joint_trajectory.header.frame_id.c_str());
    return false;
  }

  auto coll = db_->openCollection<moveit_msgs::msg::RobotTrajectory>(
    "move_group_plan_cache", move_group_namespace);

  // If start and goal are "exact" match, AND the candidate plan is better,
  // overwrite.
  Query::Ptr exact_query = coll.createQuery();

  bool start_query_ok = this->extract_and_append_plan_start_to_query(
    *exact_query, move_group, plan_request, 0);
  bool goal_query_ok = this->extract_and_append_plan_goal_to_query(
    *exact_query, move_group, plan_request, 0);

  if (!start_query_ok || !goal_query_ok)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Skipping insert: Could not construct overwrite query.");
    return false;
  }

  auto exact_matches = coll.queryList(exact_query, /* metadata_only */ true);
  double best_execution_time_seen = std::numeric_limits<double>::infinity();
  if (!exact_matches.empty())
  {
    for (auto& match : exact_matches)
    {
      double match_execution_time_s =
        match->lookupDouble("execution_time_s");
      if (match_execution_time_s < best_execution_time_seen)
      {
        best_execution_time_seen = match_execution_time_s;
      }

      if (match_execution_time_s > execution_time_s)
      {
        // If we found any "exact" matches that are worse, delete them.
        if (overwrite)
        {
          int delete_id = match->lookupInt("id");
          RCLCPP_INFO(
            node_->get_logger(),
            "Overwriting plan (id: %d): "
            "execution_time (%es) > new plan's execution_time (%es)",
            delete_id, match_execution_time_s, execution_time_s);

          Query::Ptr delete_query = coll.createQuery();
          delete_query->append("id", delete_id);
          coll.removeMessages(delete_query);
        }
      }
    }
  }

  // Insert if candidate is best seen.
  if (execution_time_s < best_execution_time_seen)
  {
    Metadata::Ptr insert_metadata = coll.createMetadata();

    bool start_meta_ok = this->extract_and_append_plan_start_to_metadata(
      *insert_metadata, move_group, plan_request);
    bool goal_meta_ok = this->extract_and_append_plan_goal_to_metadata(
      *insert_metadata, move_group, plan_request);
    insert_metadata->append("execution_time_s", execution_time_s);
    insert_metadata->append("planning_time_s", planning_time_s);

    if (!start_meta_ok || !goal_meta_ok)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Skipping insert: Could not construct insert metadata.");
      return false;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Inserting plan: New plan execution_time (%es) "
      "is better than best plan's execution_time (%es)",
      execution_time_s, best_execution_time_seen);

    coll.insert(plan, insert_metadata);
    return true;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Skipping insert: New plan execution_time (%es) "
    "is worse than best plan's execution_time (%es)",
    execution_time_s, best_execution_time_seen);
  return false;
}

// MOTION PLAN CACHING: QUERY CONSTRUCTION
bool
MotionPlanCache::extract_and_append_plan_start_to_query(
  Query& query,
  const moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit_msgs::msg::MotionPlanRequest& plan_request,
  double match_tolerance)
{
  match_tolerance += exact_match_precision_;

  // Make ignored members explicit
  if (!plan_request.start_state.multi_dof_joint_state.joint_names.empty())
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring start_state.multi_dof_joint_states: Not supported.");
  }
  if (!plan_request.start_state.attached_collision_objects.empty())
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring start_state.attached_collision_objects: Not supported.");
  }

  // auto original = *query;  // Copy not supported.

  query.append("group_name", plan_request.group_name);

  // Workspace params
  // Match anything within our specified workspace limits.
  query.append(
    "workspace_parameters.header.frame_id",
    plan_request.workspace_parameters.header.frame_id);
  query.appendGTE(
    "workspace_parameters.min_corner.x",
    plan_request.workspace_parameters.min_corner.x);
  query.appendGTE(
    "workspace_parameters.min_corner.y",
    plan_request.workspace_parameters.min_corner.y);
  query.appendGTE(
    "workspace_parameters.min_corner.z",
    plan_request.workspace_parameters.min_corner.z);
  query.appendLTE(
    "workspace_parameters.max_corner.x",
    plan_request.workspace_parameters.max_corner.x);
  query.appendLTE(
    "workspace_parameters.max_corner.y",
    plan_request.workspace_parameters.max_corner.y);
  query.appendLTE(
    "workspace_parameters.max_corner.z",
    plan_request.workspace_parameters.max_corner.z);

  // Joint state
  //   Only accounts for joint_state position. Ignores velocity and effort.
  if (plan_request.start_state.is_diff)
  {
    // If plan request states that the start_state is_diff, then we need to get
    // the current state from the move_group.

    // NOTE: methyldragon -
    //   I think if is_diff is on, the joint states will not be populated in all
    //   of our motion plan requests? If this isn't the case we might need to
    //   apply the joint states as offsets as well.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    if (!current_state)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping start query append: Could not get robot state.");
      // *query = original;  // Undo our changes. (Can't. Copy not supported.)
      return false;
    }

    moveit_msgs::msg::RobotState current_state_msg;
    robotStateToRobotStateMsg(*current_state, current_state_msg);

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); i++)
    {
      query.append(
        "start_state.joint_state.name_" + std::to_string(i),
        current_state_msg.joint_state.name.at(i));
      query.appendRangeInclusive(
        "start_state.joint_state.position_" + std::to_string(i),
        NEXUS_MATCH_RANGE(
          current_state_msg.joint_state.position.at(i), match_tolerance)
      );
    }
  }
  else
  {
    for (
      size_t i = 0; i < plan_request.start_state.joint_state.name.size(); i++
    )
    {
      query.append(
        "start_state.joint_state.name_" + std::to_string(i),
        plan_request.start_state.joint_state.name.at(i));
      query.appendRangeInclusive(
        "start_state.joint_state.position_" + std::to_string(i),
        NEXUS_MATCH_RANGE(
          plan_request.start_state.joint_state.position.at(i), match_tolerance)
      );
    }
  }
  return true;
}

bool
MotionPlanCache::extract_and_append_plan_goal_to_query(
  Query& query,
  const moveit::planning_interface::MoveGroupInterface& /* move_group */,
  const moveit_msgs::msg::MotionPlanRequest& plan_request,
  double match_tolerance)
{
  match_tolerance += exact_match_precision_;

  // Make ignored members explicit
  bool emit_position_constraint_warning = false;
  for (auto& constraint : plan_request.goal_constraints)
  {
    for (auto& position_constraint : constraint.position_constraints)
    {
      if (!position_constraint.constraint_region.primitives.empty())
      {
        emit_position_constraint_warning = true;
        break;
      }
    }
    if (emit_position_constraint_warning)
    {
      break;
    }
  }
  if (emit_position_constraint_warning)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring goal_constraints.position_constraints.constraint_region: "
      "Not supported.");
  }

  // auto original = *query;  // Copy not supported.

  query.appendRangeInclusive(
    "max_velocity_scaling_factor",
    NEXUS_MATCH_RANGE(
      plan_request.max_velocity_scaling_factor, match_tolerance)
  );
  query.appendRangeInclusive(
    "max_acceleration_scaling_factor",
    NEXUS_MATCH_RANGE(
      plan_request.max_acceleration_scaling_factor, match_tolerance)
  );
  query.appendRangeInclusive(
    "max_cartesian_speed",
    NEXUS_MATCH_RANGE(plan_request.max_cartesian_speed, match_tolerance));

  // Extract constraints (so we don't have cardinality on goal_constraint idx.)
  std::vector<moveit_msgs::msg::JointConstraint> joint_constraints;
  std::vector<moveit_msgs::msg::PositionConstraint> position_constraints;
  std::vector<moveit_msgs::msg::OrientationConstraint> orientation_constraints;
  for (auto& constraint : plan_request.goal_constraints)
  {
    for (auto& joint_constraint : constraint.joint_constraints)
    {
      joint_constraints.push_back(joint_constraint);
    }
    for (auto& position_constraint : constraint.position_constraints)
    {
      position_constraints.push_back(position_constraint);
    }
    for (auto& orientation_constraint : constraint.orientation_constraints)
    {
      orientation_constraints.push_back(orientation_constraint);
    }
  }

  // Joint constraints
  size_t joint_idx = 0;
  for (auto& constraint : joint_constraints)
  {
    std::string meta_name =
      "goal_constraints.joint_constraints_" + std::to_string(joint_idx++);

    query.append(meta_name + ".joint_name", constraint.joint_name);
    query.appendRangeInclusive(
      meta_name + ".position",
      NEXUS_MATCH_RANGE(constraint.position, match_tolerance));
    query.appendGTE(
      meta_name + ".tolerance_above", constraint.tolerance_above);
    query.appendLTE(
      meta_name + ".tolerance_below", constraint.tolerance_below);
  }

  // Position constraints
  // All offsets will be "frozen" and computed wrt. the workspace frame
  // instead.
  if (!position_constraints.empty())
  {
    query.append(
      "goal_constraints.position_constraints.header.frame_id",
      plan_request.workspace_parameters.header.frame_id);

    size_t pos_idx = 0;
    for (auto& constraint : position_constraints)
    {
      std::string meta_name =
        "goal_constraints.position_constraints_" + std::to_string(pos_idx++);

      // Compute offsets wrt. to workspace frame.
      double x_offset = 0;
      double y_offset = 0;
      double z_offset = 0;

      if (plan_request.workspace_parameters.header.frame_id !=
        constraint.header.frame_id)
      {
        try
        {
          auto transform = tf_buffer_->lookupTransform(
            constraint.header.frame_id,
            plan_request.workspace_parameters.header.frame_id,
            tf2::TimePointZero);

          x_offset = transform.transform.translation.x;
          y_offset = transform.transform.translation.y;
          z_offset = transform.transform.translation.z;
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_WARN(
            node_->get_logger(),
            "Skipping goal query append: "
            "Could not get goal transform for translation %s to %s: %s",
            plan_request.workspace_parameters.header.frame_id.c_str(),
            constraint.header.frame_id.c_str(),
            ex.what());

          // (Can't. Copy not supported.)
          // *query = original;  // Undo our changes.
          return false;
        }
      }

      query.append(meta_name + ".link_name", constraint.link_name);

      query.appendRangeInclusive(
        meta_name + ".target_point_offset.x",
        NEXUS_MATCH_RANGE(
          x_offset + constraint.target_point_offset.x, match_tolerance));
      query.appendRangeInclusive(
        meta_name + ".target_point_offset.y",
        NEXUS_MATCH_RANGE(
          y_offset + constraint.target_point_offset.y, match_tolerance));
      query.appendRangeInclusive(
        meta_name + ".target_point_offset.z",
        NEXUS_MATCH_RANGE(
          z_offset + constraint.target_point_offset.z, match_tolerance));
    }
  }

  // Orientation constraints
  // All offsets will be "frozen" and computed wrt. the workspace frame
  // instead.
  if (!orientation_constraints.empty())
  {
    query.append(
      "goal_constraints.orientation_constraints.header.frame_id",
      plan_request.workspace_parameters.header.frame_id);

    size_t ori_idx = 0;
    for (auto& constraint : orientation_constraints)
    {
      std::string meta_name =
        "goal_constraints.orientation_constraints_" + std::to_string(ori_idx++);

      // Compute offsets wrt. to workspace frame.
      geometry_msgs::msg::Quaternion quat_offset;
      quat_offset.x = 0;
      quat_offset.y = 0;
      quat_offset.z = 0;
      quat_offset.w = 1;

      if (plan_request.workspace_parameters.header.frame_id !=
        constraint.header.frame_id)
      {
        try
        {
          auto transform = tf_buffer_->lookupTransform(
            constraint.header.frame_id,
            plan_request.workspace_parameters.header.frame_id,
            tf2::TimePointZero);

          quat_offset = transform.transform.rotation;
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_WARN(
            node_->get_logger(),
            "Skipping goal query append: "
            "Could not get goal transform for orientation %s to %s: %s",
            plan_request.workspace_parameters.header.frame_id.c_str(),
            constraint.header.frame_id.c_str(),
            ex.what());

          // (Can't. Copy not supported.)
          // *query = original;  // Undo our changes.
          return false;
        }
      }

      query.append(meta_name + ".link_name", constraint.link_name);

      // Orientation of constraint frame wrt. workspace frame
      tf2::Quaternion tf2_quat_frame_offset(
        quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);

      // Added offset on top of the constraint frame's orientation stated in
      // the constraint.
      tf2::Quaternion tf2_quat_goal_offset(
        constraint.orientation.x,
        constraint.orientation.y,
        constraint.orientation.z,
        constraint.orientation.w);

      tf2_quat_frame_offset.normalize();
      tf2_quat_goal_offset.normalize();

      auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
      final_quat.normalize();

      query.appendRangeInclusive(
        meta_name + ".target_point_offset.x",
        NEXUS_MATCH_RANGE(final_quat.getX(), match_tolerance));
      query.appendRangeInclusive(
        meta_name + ".target_point_offset.y",
        NEXUS_MATCH_RANGE(final_quat.getY(), match_tolerance));
      query.appendRangeInclusive(
        meta_name + ".target_point_offset.z",
        NEXUS_MATCH_RANGE(final_quat.getZ(), match_tolerance));
      query.appendRangeInclusive(
        meta_name + ".target_point_offset.w",
        NEXUS_MATCH_RANGE(final_quat.getW(), match_tolerance));
    }
  }

  return true;
}

// MOTION PLAN CACHING: METADATA CONSTRUCTION
bool
MotionPlanCache::extract_and_append_plan_start_to_metadata(
  Metadata& metadata,
  const moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit_msgs::msg::MotionPlanRequest& plan_request)
{
  // Make ignored members explicit
  if (!plan_request.start_state.multi_dof_joint_state.joint_names.empty())
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring start_state.multi_dof_joint_states: Not supported.");
  }
  if (!plan_request.start_state.attached_collision_objects.empty())
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring start_state.attached_collision_objects: Not supported.");
  }

  // auto original = *metadata;  // Copy not supported.

  metadata.append("group_name", plan_request.group_name);

  // Workspace params
  metadata.append(
    "workspace_parameters.header.frame_id",
    plan_request.workspace_parameters.header.frame_id);
  metadata.append(
    "workspace_parameters.min_corner.x",
    plan_request.workspace_parameters.min_corner.x);
  metadata.append(
    "workspace_parameters.min_corner.y",
    plan_request.workspace_parameters.min_corner.y);
  metadata.append(
    "workspace_parameters.min_corner.z",
    plan_request.workspace_parameters.min_corner.z);
  metadata.append(
    "workspace_parameters.max_corner.x",
    plan_request.workspace_parameters.max_corner.x);
  metadata.append(
    "workspace_parameters.max_corner.y",
    plan_request.workspace_parameters.max_corner.y);
  metadata.append(
    "workspace_parameters.max_corner.z",
    plan_request.workspace_parameters.max_corner.z);

  // Joint state
  //   Only accounts for joint_state position. Ignores velocity and effort.
  if (plan_request.start_state.is_diff)
  {
    // If plan request states that the start_state is_diff, then we need to get
    // the current state from the move_group.

    // NOTE: methyldragon -
    //   I think if is_diff is on, the joint states will not be populated in all
    //   of our motion plan requests? If this isn't the case we might need to
    //   apply the joint states as offsets as well.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    if (!current_state)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping start metadata append: Could not get robot state.");
      // *metadata = original;  // Undo our changes.  // Copy not supported
      return false;
    }

    moveit_msgs::msg::RobotState current_state_msg;
    robotStateToRobotStateMsg(*current_state, current_state_msg);

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); i++)
    {
      metadata.append(
        "start_state.joint_state.name_" + std::to_string(i),
        current_state_msg.joint_state.name.at(i));
      metadata.append(
        "start_state.joint_state.position_" + std::to_string(i),
        current_state_msg.joint_state.position.at(i));
    }
  }
  else
  {
    for (
      size_t i = 0; i < plan_request.start_state.joint_state.name.size(); i++
    )
    {
      metadata.append(
        "start_state.joint_state.name_" + std::to_string(i),
        plan_request.start_state.joint_state.name.at(i));
      metadata.append(
        "start_state.joint_state.position_" + std::to_string(i),
        plan_request.start_state.joint_state.position.at(i));
    }
  }

  return true;
}

bool
MotionPlanCache::extract_and_append_plan_goal_to_metadata(
  Metadata& metadata,
  const moveit::planning_interface::MoveGroupInterface& /* move_group */,
  const moveit_msgs::msg::MotionPlanRequest& plan_request)
{
  // Make ignored members explicit
  bool emit_position_constraint_warning = false;
  for (auto& constraint : plan_request.goal_constraints)
  {
    for (auto& position_constraint : constraint.position_constraints)
    {
      if (!position_constraint.constraint_region.primitives.empty())
      {
        emit_position_constraint_warning = true;
        break;
      }
    }
    if (emit_position_constraint_warning)
    {
      break;
    }
  }
  if (emit_position_constraint_warning)
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring goal_constraints.position_constraints.constraint_region: "
      "Not supported.");
  }

  // auto original = *metadata;  // Copy not supported.

  metadata.append("max_velocity_scaling_factor",
    plan_request.max_velocity_scaling_factor);
  metadata.append("max_acceleration_scaling_factor",
    plan_request.max_acceleration_scaling_factor);
  metadata.append("max_cartesian_speed", plan_request.max_cartesian_speed);

  // Extract constraints (so we don't have cardinality on goal_constraint idx.)
  std::vector<moveit_msgs::msg::JointConstraint> joint_constraints;
  std::vector<moveit_msgs::msg::PositionConstraint> position_constraints;
  std::vector<moveit_msgs::msg::OrientationConstraint> orientation_constraints;
  for (auto& constraint : plan_request.goal_constraints)
  {
    for (auto& joint_constraint : constraint.joint_constraints)
    {
      joint_constraints.push_back(joint_constraint);
    }
    for (auto& position_constraint : constraint.position_constraints)
    {
      position_constraints.push_back(position_constraint);
    }
    for (auto& orientation_constraint : constraint.orientation_constraints)
    {
      orientation_constraints.push_back(orientation_constraint);
    }
  }

  // Joint constraints
  size_t joint_idx = 0;
  for (auto& constraint : joint_constraints)
  {
    std::string meta_name =
      "goal_constraints.joint_constraints_" + std::to_string(joint_idx++);

    metadata.append(meta_name + ".joint_name", constraint.joint_name);
    metadata.append(meta_name + ".position", constraint.position);
    metadata.append(
      meta_name + ".tolerance_above", constraint.tolerance_above);
    metadata.append(
      meta_name + ".tolerance_below", constraint.tolerance_below);
  }

  // Position constraints
  if (!position_constraints.empty())
  {
    // All offsets will be "frozen" and computed wrt. the workspace frame
    // instead.
    metadata.append(
      "goal_constraints.position_constraints.header.frame_id",
      plan_request.workspace_parameters.header.frame_id);

    size_t position_idx = 0;
    for (auto& constraint : position_constraints)
    {
      std::string meta_name =
        "goal_constraints.position_constraints_"
        + std::to_string(position_idx++);

      // Compute offsets wrt. to workspace frame.
      double x_offset = 0;
      double y_offset = 0;
      double z_offset = 0;

      if (plan_request.workspace_parameters.header.frame_id !=
        constraint.header.frame_id)
      {
        try
        {
          auto transform = tf_buffer_->lookupTransform(
            constraint.header.frame_id,
            plan_request.workspace_parameters.header.frame_id,
            tf2::TimePointZero);

          x_offset = transform.transform.translation.x;
          y_offset = transform.transform.translation.y;
          z_offset = transform.transform.translation.z;
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_WARN(
            node_->get_logger(),
            "Skipping goal metadata append: "
            "Could not get goal transform for translation %s to %s: %s",
            plan_request.workspace_parameters.header.frame_id.c_str(),
            constraint.header.frame_id.c_str(),
            ex.what());

          // (Can't. Copy not supported.)
          // *metadata = original;  // Undo our changes.
          return false;
        }
      }

      metadata.append(meta_name + ".link_name", constraint.link_name);

      metadata.append(
        meta_name + ".target_point_offset.x",
        x_offset + constraint.target_point_offset.x);
      metadata.append(
        meta_name + ".target_point_offset.y",
        y_offset + constraint.target_point_offset.y);
      metadata.append(
        meta_name + ".target_point_offset.z",
        z_offset + constraint.target_point_offset.z);
    }
  }

  // Orientation constraints
  if (!orientation_constraints.empty())
  {
    // All offsets will be "frozen" and computed wrt. the workspace frame
    // instead.
    metadata.append(
      "goal_constraints.orientation_constraints.header.frame_id",
      plan_request.workspace_parameters.header.frame_id);

    size_t ori_idx = 0;
    for (auto& constraint : orientation_constraints)
    {
      std::string meta_name =
        "goal_constraints.orientation_constraints_" + std::to_string(ori_idx++);

      // Compute offsets wrt. to workspace frame.
      geometry_msgs::msg::Quaternion quat_offset;
      quat_offset.x = 0;
      quat_offset.y = 0;
      quat_offset.z = 0;
      quat_offset.w = 1;

      if (plan_request.workspace_parameters.header.frame_id !=
        constraint.header.frame_id)
      {
        try
        {
          auto transform = tf_buffer_->lookupTransform(
            constraint.header.frame_id,
            plan_request.workspace_parameters.header.frame_id,
            tf2::TimePointZero);

          quat_offset = transform.transform.rotation;
        }
        catch (tf2::TransformException& ex)
        {
          RCLCPP_WARN(
            node_->get_logger(),
            "Skipping goal metadata append: "
            "Could not get goal transform for orientation %s to %s: %s",
            plan_request.workspace_parameters.header.frame_id.c_str(),
            constraint.header.frame_id.c_str(),
            ex.what());

          // (Can't. Copy not supported.)
          // *metadata = original;  // Undo our changes.
          return false;
        }
      }

      metadata.append(meta_name + ".link_name", constraint.link_name);

      // Orientation of constraint frame wrt. workspace frame
      tf2::Quaternion tf2_quat_frame_offset(
        quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);

      // Added offset on top of the constraint frame's orientation stated in
      // the constraint.
      tf2::Quaternion tf2_quat_goal_offset(
        constraint.orientation.x,
        constraint.orientation.y,
        constraint.orientation.z,
        constraint.orientation.w);

      tf2_quat_frame_offset.normalize();
      tf2_quat_goal_offset.normalize();

      auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
      final_quat.normalize();

      metadata.append(meta_name + ".target_point_offset.x",
        final_quat.getX());
      metadata.append(meta_name + ".target_point_offset.y",
        final_quat.getY());
      metadata.append(meta_name + ".target_point_offset.z",
        final_quat.getZ());
      metadata.append(meta_name + ".target_point_offset.w",
        final_quat.getW());
    }
  }

  return true;
}

// =============================================================================
// CARTESIAN PLAN CACHING
// =============================================================================
// CARTESIAN PLAN CACHING: TOP LEVEL OPS
moveit_msgs::srv::GetCartesianPath::Request
MotionPlanCache::construct_get_cartesian_plan_request(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const std::vector<geometry_msgs::msg::Pose>& waypoints, double step,
  double jump_threshold, bool avoid_collisions)
{
  moveit_msgs::srv::GetCartesianPath::Request out;

  // Some of these parameters need us to pull PRIVATE values out of the
  // move_group elsewhere... Yes, it is very cursed and I hate it.
  // Fixing it requires fixing it in MoveIt.
  moveit_msgs::msg::MotionPlanRequest tmp;
  move_group.constructMotionPlanRequest(tmp);

  out.start_state = tmp.start_state;
  out.group_name = tmp.group_name;
  out.max_velocity_scaling_factor = tmp.max_velocity_scaling_factor;
  out.max_acceleration_scaling_factor = tmp.max_acceleration_scaling_factor;

  out.header.frame_id = move_group.getPoseReferenceFrame();
  out.waypoints = waypoints;
  out.max_step = step;
  out.jump_threshold = jump_threshold;
  out.path_constraints = moveit_msgs::msg::Constraints();
  out.avoid_collisions = avoid_collisions;
  out.link_name = move_group.getEndEffectorLink();

  // We were already cursed before, now we are double cursed...
  // move_group.getNodeHandle() is UNIMPLEMENTED upstream.
  out.header.stamp = node_->now();

  return out;
}

// std::vector<MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr>
// MotionPlanCache::fetch_all_matching_cartesian_plans(
//   const moveit::planning_interface::MoveGroupInterface& move_group,
//   const std::string& move_group_namespace,
//   const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
//   double start_tolerance, double goal_tolerance, bool metadata_only)
// {
//   auto coll = db_->openCollection<moveit_msgs::msg::RobotTrajectory>(
//     "move_group_plan_cache", move_group_namespace);

//   Query::Ptr query = coll.createQuery();

//   bool start_ok = this->extract_and_append_cartesian_plan_start_to_query(
//     *query, move_group, plan_request, start_tolerance);
//   bool goal_ok = this->extract_and_append_cartesian_plan_goal_to_query(
//     *query, move_group, plan_request, goal_tolerance);

//   if (!start_ok || !goal_ok)
//   {
//     RCLCPP_ERROR(node_->get_logger(), "Could not construct plan query.");
//     return {};
//   }

//   return coll.queryList(
//     query, metadata_only, /* sort_by */ "execution_time_s", true);
// }

// MessageWithMetadata<moveit_msgs::msg::RobotTrajectory>::ConstPtr
// MotionPlanCache::fetch_best_matching_cartesian_plan(
//   const moveit::planning_interface::MoveGroupInterface& move_group,
//   const std::string& move_group_namespace,
//   const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
//   double start_tolerance, double goal_tolerance, bool metadata_only)
// {
//   // First find all matching, but metadata only.
//   // Then use the ID metadata of the best plan to pull the actual message.
//   auto matching_plans = this->fetch_all_matching_plans(
//     move_group, move_group_namespace,
//     plan_request, start_tolerance, goal_tolerance,
//     true);

//   if (matching_plans.empty())
//   {
//     RCLCPP_INFO(node_->get_logger(), "No matching plans found.");
//     return nullptr;
//   }

//   auto coll = db_->openCollection<moveit_msgs::msg::RobotTrajectory>(
//     "move_group_plan_cache", move_group_namespace);

//   // Best plan is at first index, since the lookup query was sorted by
//   // execution_time.
//   int best_plan_id = matching_plans.at(0)->lookupInt("id");
//   Query::Ptr best_query = coll.createQuery();
//   best_query->append("id", best_plan_id);

//   return coll.findOne(best_query, metadata_only);
// }

// bool
// MotionPlanCache::put_cartesian_plan(
//   const moveit::planning_interface::MoveGroupInterface& move_group,
//   const std::string& move_group_namespace,
//   const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
//   const moveit_msgs::msg::RobotTrajectory& plan,
//   double execution_time_s,
//   double planning_time_s,
//   bool overwrite)
// {
//   // Check pre-conditions
//   if (!plan.multi_dof_joint_trajectory.points.empty())
//   {
//     RCLCPP_ERROR(
//       node_->get_logger(),
//       "Skipping insert: Multi-DOF trajectory plans are not supported.");
//     return false;
//   }
//   if (plan_request.workspace_parameters.header.frame_id.empty() ||
//     plan.joint_trajectory.header.frame_id.empty())
//   {
//     RCLCPP_ERROR(
//       node_->get_logger(), "Skipping insert: Frame IDs cannot be empty.");
//     return false;
//   }
//   if (plan_request.workspace_parameters.header.frame_id !=
//     plan.joint_trajectory.header.frame_id)
//   {
//     RCLCPP_ERROR(
//       node_->get_logger(),
//       "Skipping insert: "
//       "Plan request frame (%s) does not match plan frame (%s).",
//       plan_request.workspace_parameters.header.frame_id.c_str(),
//       plan.joint_trajectory.header.frame_id.c_str());
//     return false;
//   }

//   auto coll = db_->openCollection<moveit_msgs::msg::RobotTrajectory>(
//     "move_group_plan_cache", move_group_namespace);

//   // If start and goal are "exact" match, AND the candidate plan is better,
//   // overwrite.
//   Query::Ptr exact_query = coll.createQuery();

//   bool start_query_ok = this->extract_and_append_cartesian_plan_start_to_query(
//     *exact_query, move_group, plan_request, 0);
//   bool goal_query_ok = this->extract_and_append_cartesian_plan_goal_to_query(
//     *exact_query, move_group, plan_request, 0);

//   if (!start_query_ok || !goal_query_ok)
//   {
//     RCLCPP_ERROR(
//       node_->get_logger(),
//       "Skipping insert: Could not construct overwrite query.");
//     return false;
//   }

//   auto exact_matches = coll.queryList(exact_query, /* metadata_only */ true);
//   double best_execution_time_seen = std::numeric_limits<double>::infinity();
//   if (!exact_matches.empty())
//   {
//     for (auto& match : exact_matches)
//     {
//       double match_execution_time_s =
//         match->lookupDouble("execution_time_s");
//       if (match_execution_time_s < best_execution_time_seen)
//       {
//         best_execution_time_seen = match_execution_time_s;
//       }

//       if (match_execution_time_s > execution_time_s)
//       {
//         // If we found any "exact" matches that are worse, delete them.
//         if (overwrite)
//         {
//           int delete_id = match->lookupInt("id");
//           RCLCPP_INFO(
//             node_->get_logger(),
//             "Overwriting plan (id: %d): "
//             "execution_time (%es) > new plan's execution_time (%es)",
//             delete_id, match_execution_time_s, execution_time_s);

//           Query::Ptr delete_query = coll.createQuery();
//           delete_query->append("id", delete_id);
//           coll.removeMessages(delete_query);
//         }
//       }
//     }
//   }

//   // Insert if candidate is best seen.
//   if (execution_time_s < best_execution_time_seen)
//   {
//     Metadata::Ptr insert_metadata = coll.createMetadata();

//     bool start_meta_ok = this->extract_and_append_cartesian_plan_start_to_metadata(
//       *insert_metadata, move_group, plan_request);
//     bool goal_meta_ok = this->extract_and_append_cartesian_plan_goal_to_metadata(
//       *insert_metadata, move_group, plan_request);
//     insert_metadata->append("execution_time_s", execution_time_s);
//     insert_metadata->append("planning_time_s", planning_time_s);

//     if (!start_meta_ok || !goal_meta_ok)
//     {
//       RCLCPP_ERROR(
//         node_->get_logger(),
//         "Skipping insert: Could not construct insert metadata.");
//       return false;
//     }

//     RCLCPP_INFO(
//       node_->get_logger(),
//       "Inserting plan: New plan execution_time (%es) "
//       "is better than best plan's execution_time (%es)",
//       execution_time_s, best_execution_time_seen);

//     coll.insert(plan, insert_metadata);
//     return true;
//   }

//   RCLCPP_INFO(
//     node_->get_logger(),
//     "Skipping insert: New plan execution_time (%es) "
//     "is worse than best plan's execution_time (%es)",
//     execution_time_s, best_execution_time_seen);
//   return false;
// }

// // CARTESIAN PLAN CACHING: QUERY CONSTRUCTION
bool
MotionPlanCache::extract_and_append_cartesian_plan_start_to_query(
  Query& query,
  const moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
  double match_tolerance)
{
  match_tolerance += exact_match_precision_;

  // Make ignored members explicit
  if (!plan_request.start_state.multi_dof_joint_state.joint_names.empty())
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring start_state.multi_dof_joint_states: Not supported.");
  }
  if (!plan_request.start_state.attached_collision_objects.empty())
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring start_state.attached_collision_objects: Not supported.");
  }

  // auto original = *metadata;  // Copy not supported.

  query.append("group_name", plan_request.group_name);

  // Joint state
  //   Only accounts for joint_state position. Ignores velocity and effort.
  if (plan_request.start_state.is_diff)
  {
    // If plan request states that the start_state is_diff, then we need to get
    // the current state from the move_group.

    // NOTE: methyldragon -
    //   I think if is_diff is on, the joint states will not be populated in all
    //   of our motion plan requests? If this isn't the case we might need to
    //   apply the joint states as offsets as well.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    if (!current_state)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping start metadata append: Could not get robot state.");
      // *metadata = original;  // Undo our changes.  // Copy not supported
      return false;
    }

    moveit_msgs::msg::RobotState current_state_msg;
    robotStateToRobotStateMsg(*current_state, current_state_msg);

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); i++)
    {
      query.append(
        "start_state.joint_state.name_" + std::to_string(i),
        current_state_msg.joint_state.name.at(i));
      query.appendRangeInclusive(
        "start_state.joint_state.position_" + std::to_string(i),
        NEXUS_MATCH_RANGE(
          current_state_msg.joint_state.position.at(i), match_tolerance));
    }
  }
  else
  {
    for (
      size_t i = 0; i < plan_request.start_state.joint_state.name.size(); i++
    )
    {
      query.append(
        "start_state.joint_state.name_" + std::to_string(i),
        plan_request.start_state.joint_state.name.at(i));
      query.appendRangeInclusive(
        "start_state.joint_state.position_" + std::to_string(i),
        NEXUS_MATCH_RANGE(
          plan_request.start_state.joint_state.position.at(i),
          match_tolerance));
    }
  }

  return true;
}

bool
MotionPlanCache::extract_and_append_cartesian_plan_goal_to_query(
  Query& query,
  const moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
  double match_tolerance)
{
  match_tolerance += exact_match_precision_;

  // Make ignored members explicit
  if (plan_request.path_constraints.joint_constraints.empty() ||
    plan_request.path_constraints.position_constraints.empty() ||
    plan_request.path_constraints.orientation_constraints.empty() ||
    plan_request.path_constraints.visibility_constraints.empty())
  {
    RCLCPP_WARN(
      node_->get_logger(), "Ignoring path_constraints: Not supported.");
  }
  if (plan_request.avoid_collisions)
  {
    RCLCPP_WARN(node_->get_logger(),
      "Ignoring avoid_collisions: Not supported.");
  }

  // auto original = *metadata;  // Copy not supported.

  query.appendRangeInclusive(
    "max_velocity_scaling_factor",
    NEXUS_MATCH_RANGE(
      plan_request.max_velocity_scaling_factor, match_tolerance));
  query.appendRangeInclusive(
    "max_acceleration_scaling_factor",
    NEXUS_MATCH_RANGE(
      plan_request.max_acceleration_scaling_factor, match_tolerance));
  query.appendRangeInclusive(
    "max_step",
    NEXUS_MATCH_RANGE(plan_request.max_step, match_tolerance));
  query.appendRangeInclusive(
    "jump_threshold",
    NEXUS_MATCH_RANGE(plan_request.jump_threshold, match_tolerance));

  // Waypoints
  // Restating them in terms of the robot model frame (usually base_link)
  std::string base_frame = move_group.getRobotModel()->getModelFrame();

  // Compute offsets.
  double x_offset = 0;
  double y_offset = 0;
  double z_offset = 0;

  geometry_msgs::msg::Quaternion quat_offset;
  quat_offset.x = 0;
  quat_offset.y = 0;
  quat_offset.z = 0;
  quat_offset.w = 1;

  if (base_frame != plan_request.header.frame_id)
  {
    try
    {
      auto transform = tf_buffer_->lookupTransform(
        plan_request.header.frame_id, base_frame, tf2::TimePointZero);

      x_offset = transform.transform.translation.x;
      y_offset = transform.transform.translation.y;
      z_offset = transform.transform.translation.z;
      quat_offset = transform.transform.rotation;
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping goal metadata append: "
        "Could not get goal transform for %s to %s: %s",
        base_frame.c_str(), plan_request.header.frame_id.c_str(),
        ex.what());

      // (Can't. Copy not supported.)
      // *metadata = original;  // Undo our changes.
      return false;
    }
  }

  tf2::Quaternion tf2_quat_frame_offset(
    quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);
  tf2_quat_frame_offset.normalize();

  size_t waypoint_idx = 0;
  for (auto& waypoint : plan_request.waypoints)
  {
    std::string meta_name = "waypoints_" + std::to_string(waypoint_idx++);

    // Apply offsets
    // Position
    query.appendRangeInclusive(
      meta_name + ".position.x",
      NEXUS_MATCH_RANGE(x_offset + waypoint.position.x, match_tolerance));
    query.appendRangeInclusive(
      meta_name + ".position.y",
      NEXUS_MATCH_RANGE(y_offset+ waypoint.position.y, match_tolerance));
    query.appendRangeInclusive(
      meta_name + ".position.z",
      NEXUS_MATCH_RANGE(z_offset+waypoint.position.z, match_tolerance));

    // Orientation
    tf2::Quaternion tf2_quat_goal_offset(
      waypoint.orientation.x,
      waypoint.orientation.y,
      waypoint.orientation.z,
      waypoint.orientation.w);
    tf2_quat_goal_offset.normalize();

    auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
    final_quat.normalize();

    query.appendRangeInclusive(
      meta_name + ".orientation.x",
      NEXUS_MATCH_RANGE(final_quat.getX(), match_tolerance));
    query.appendRangeInclusive(
      meta_name + ".orientation.y",
      NEXUS_MATCH_RANGE(final_quat.getY(), match_tolerance));
    query.appendRangeInclusive(
      meta_name + ".orientation.z",
      NEXUS_MATCH_RANGE(final_quat.getZ(), match_tolerance));
    query.appendRangeInclusive(
      meta_name + ".orientation.w",
      NEXUS_MATCH_RANGE(final_quat.getW(), match_tolerance));
  }

  query.append("link_name", plan_request.link_name);

  return true;
}

// CARTESIAN PLAN CACHING: METADATA CONSTRUCTION
bool
MotionPlanCache::extract_and_append_cartesian_plan_start_to_metadata(
  Metadata& metadata,
  const moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit_msgs::srv::GetCartesianPath::Request& plan_request)
{
  // Make ignored members explicit
  if (!plan_request.start_state.multi_dof_joint_state.joint_names.empty())
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring start_state.multi_dof_joint_states: Not supported.");
  }
  if (!plan_request.start_state.attached_collision_objects.empty())
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "Ignoring start_state.attached_collision_objects: Not supported.");
  }

  // auto original = *metadata;  // Copy not supported.

  metadata.append("group_name", plan_request.group_name);

  // Joint state
  //   Only accounts for joint_state position. Ignores velocity and effort.
  if (plan_request.start_state.is_diff)
  {
    // If plan request states that the start_state is_diff, then we need to get
    // the current state from the move_group.

    // NOTE: methyldragon -
    //   I think if is_diff is on, the joint states will not be populated in all
    //   of our motion plan requests? If this isn't the case we might need to
    //   apply the joint states as offsets as well.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    if (!current_state)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping start metadata append: Could not get robot state.");
      // *metadata = original;  // Undo our changes.  // Copy not supported
      return false;
    }

    moveit_msgs::msg::RobotState current_state_msg;
    robotStateToRobotStateMsg(*current_state, current_state_msg);

    for (size_t i = 0; i < current_state_msg.joint_state.name.size(); i++)
    {
      metadata.append(
        "start_state.joint_state.name_" + std::to_string(i),
        current_state_msg.joint_state.name.at(i));
      metadata.append(
        "start_state.joint_state.position_" + std::to_string(i),
        current_state_msg.joint_state.position.at(i));
    }
  }
  else
  {
    for (
      size_t i = 0; i < plan_request.start_state.joint_state.name.size(); i++
    )
    {
      metadata.append(
        "start_state.joint_state.name_" + std::to_string(i),
        plan_request.start_state.joint_state.name.at(i));
      metadata.append(
        "start_state.joint_state.position_" + std::to_string(i),
        plan_request.start_state.joint_state.position.at(i));
    }
  }

  return true;
}

bool
MotionPlanCache::extract_and_append_cartesian_plan_goal_to_metadata(
  Metadata& metadata,
  const moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit_msgs::srv::GetCartesianPath::Request& plan_request)
{
  // Make ignored members explicit
  if (plan_request.path_constraints.joint_constraints.empty() ||
    plan_request.path_constraints.position_constraints.empty() ||
    plan_request.path_constraints.orientation_constraints.empty() ||
    plan_request.path_constraints.visibility_constraints.empty())
  {
    RCLCPP_WARN(
      node_->get_logger(), "Ignoring path_constraints: Not supported.");
  }
  if (plan_request.avoid_collisions)
  {
    RCLCPP_WARN(node_->get_logger(),
      "Ignoring avoid_collisions: Not supported.");
  }

  // auto original = *metadata;  // Copy not supported.

  metadata.append("max_velocity_scaling_factor",
    plan_request.max_velocity_scaling_factor);
  metadata.append("max_acceleration_scaling_factor",
    plan_request.max_acceleration_scaling_factor);
  metadata.append("max_step", plan_request.max_step);
  metadata.append("jump_threshold", plan_request.jump_threshold);

  // Waypoints
  // Restating them in terms of the robot model frame (usually base_link)
  std::string base_frame = move_group.getRobotModel()->getModelFrame();

  // Compute offsets.
  double x_offset = 0;
  double y_offset = 0;
  double z_offset = 0;

  geometry_msgs::msg::Quaternion quat_offset;
  quat_offset.x = 0;
  quat_offset.y = 0;
  quat_offset.z = 0;
  quat_offset.w = 1;

  if (base_frame != plan_request.header.frame_id)
  {
    try
    {
      auto transform = tf_buffer_->lookupTransform(
        plan_request.header.frame_id, base_frame, tf2::TimePointZero);

      x_offset = transform.transform.translation.x;
      y_offset = transform.transform.translation.y;
      z_offset = transform.transform.translation.z;
      quat_offset = transform.transform.rotation;
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Skipping goal metadata append: "
        "Could not get goal transform for %s to %s: %s",
        base_frame.c_str(), plan_request.header.frame_id.c_str(),
        ex.what());

      // (Can't. Copy not supported.)
      // *metadata = original;  // Undo our changes.
      return false;
    }
  }

  tf2::Quaternion tf2_quat_frame_offset(
    quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w);
  tf2_quat_frame_offset.normalize();

  size_t waypoint_idx = 0;
  for (auto& waypoint : plan_request.waypoints)
  {
    std::string meta_name = "waypoints_" + std::to_string(waypoint_idx++);

    // Apply offsets
    // Position
    metadata.append(meta_name + ".position.x", x_offset + waypoint.position.x);
    metadata.append(meta_name + ".position.y", y_offset+ waypoint.position.y);
    metadata.append(meta_name + ".position.z", z_offset+waypoint.position.z);

    // Orientation
    tf2::Quaternion tf2_quat_goal_offset(
      waypoint.orientation.x,
      waypoint.orientation.y,
      waypoint.orientation.z,
      waypoint.orientation.w);
    tf2_quat_goal_offset.normalize();

    auto final_quat = tf2_quat_goal_offset * tf2_quat_frame_offset;
    final_quat.normalize();

    metadata.append(meta_name + ".orientation.x", final_quat.getX());
    metadata.append(meta_name + ".orientation.y", final_quat.getY());
    metadata.append(meta_name + ".orientation.z", final_quat.getZ());
    metadata.append(meta_name + ".orientation.w", final_quat.getW());
  }

  metadata.append("link_name", plan_request.link_name);

  return true;
}

#undef NEXUS_MATCH_RANGE

}  // namespace motion_planner
}  // namespace nexus
