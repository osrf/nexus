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

#ifndef SRC__MOTION_PLAN_CACHE_HPP
#define SRC__MOTION_PLAN_CACHE_HPP

#include <chrono>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <warehouse_ros/database_loader.h>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ROS2 Messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>

// moveit modules
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/srv/get_cartesian_path.hpp>

// NEXUS messages
#include <nexus_endpoints.hpp>

namespace nexus {
namespace motion_planner {

/**
 * Cache manager for the Nexus motion planner.
 *
 * This manager facilitates cache management for MoveIt 2's `MoveGroupInterface`
 * by using `warehouse_ros` to manage a database of executed motion plans, and
 * how long they took to execute. This allows for the lookup and reuse of the
 * best performing plans found so far.
 *
 * WARNING:
 *   This cache does NOT support collision detection!
 *   Plans will be put into and fetched from the cache IGNORING collision.
 *   If your planning scene is expected to change between cache lookups, do NOT
 *   use this cache, fetched plans are likely to result in collision then.
 *
 *   To handle collisions this class will need to hash the planning scene world
 *   msg (after zeroing out std_msgs/Header timestamps and sequences) and do an
 *   appropriate lookup.
 *
 * Relevant ROS Parameters:
 *   - `warehouse_plugin`: What database to use
 *   - `warehouse_host`: Where the database should be created
 *   - `warehouse_port`: The port used for the database
 *
 * Motion plans are stored in the `move_group_plan_cache` database within the
 * database file, with plans for each move group stored in a collection named
 * after the relevant move group's name.
 *
 * For example, the "my_move_group" move group will have its cache stored in
 * `move_group_plan_cache@my_move_group`
 *
 * Motion plans are keyed on:
 *   - Plan Start: robot joint state
 *   - Plan Goal (either of):
 *     - Final pose (wrt. `planning_frame` (usually `base_link`))
 *     - Final robot joint states
 *
 * Motion plans may be looked up with some tolerance at call time.
 */
class MotionPlanCache
{
public:
  // We explicitly need a Node::SharedPtr because warehouse_ros ONLY supports
  // it...
  explicit MotionPlanCache(const rclcpp::Node::SharedPtr& node);

  void init(
    const std::string& db_path = ":memory:",
    uint32_t db_port = 0,
    double exact_match_precision = 1e-6);

  // ===========================================================================
  // MOTION PLAN CACHING
  // ===========================================================================
  // TOP LEVEL OPS
  std::vector<
    warehouse_ros::MessageWithMetadata<
      moveit_msgs::msg::RobotTrajectory
    >::ConstPtr
  >
  fetch_all_matching_plans(
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& move_group_namespace,
    const moveit_msgs::msg::MotionPlanRequest& plan_request,
    double start_tolerance, double goal_tolerance, bool metadata_only = false);

  warehouse_ros::MessageWithMetadata<
    moveit_msgs::msg::RobotTrajectory
  >::ConstPtr
  fetch_best_matching_plan(
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& move_group_namespace,
    const moveit_msgs::msg::MotionPlanRequest& plan_request,
    double start_tolerance, double goal_tolerance, bool metadata_only = false);

  bool put_plan(
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& move_group_namespace,
    const moveit_msgs::msg::MotionPlanRequest& plan_request,
    const moveit_msgs::msg::RobotTrajectory& plan,
    double execution_time_s,
    double planning_time_s,
    bool overwrite = true);

  // QUERY CONSTRUCTION
  bool extract_and_append_plan_start_to_query(
    warehouse_ros::Query& query,
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::msg::MotionPlanRequest& plan_request,
    double match_tolerance);

  bool extract_and_append_plan_goal_to_query(
    warehouse_ros::Query& query,
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::msg::MotionPlanRequest& plan_request,
    double match_tolerance);

  // METADATA CONSTRUCTION
  bool extract_and_append_plan_start_to_metadata(
    warehouse_ros::Metadata& metadata,
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::msg::MotionPlanRequest& plan_request);

  bool extract_and_append_plan_goal_to_metadata(
    warehouse_ros::Metadata& metadata,
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::msg::MotionPlanRequest& plan_request);

  // ===========================================================================
  // CARTESIAN PLAN CACHING
  // ===========================================================================
  // TOP LEVEL OPS
  // This mimics the move group computeCartesianPath signature (without path
  // constraints).
  moveit_msgs::srv::GetCartesianPath::Request
  construct_get_cartesian_plan_request(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<geometry_msgs::msg::Pose>& waypoints, double step,
    double jump_threshold, bool avoid_collisions = true);

  std::vector<
    warehouse_ros::MessageWithMetadata<
      moveit_msgs::msg::RobotTrajectory
    >::ConstPtr
  >
  fetch_all_matching_cartesian_plans(
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& move_group_namespace,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
    double min_fraction,
    double start_tolerance, double goal_tolerance, bool metadata_only = false);

  warehouse_ros::MessageWithMetadata<
    moveit_msgs::msg::RobotTrajectory
  >::ConstPtr
  fetch_best_matching_cartesian_plan(
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& move_group_namespace,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
    double min_fraction,
    double start_tolerance, double goal_tolerance, bool metadata_only = false);

  bool put_cartesian_plan(
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& move_group_namespace,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
    const moveit_msgs::msg::RobotTrajectory& plan,
    double execution_time_s,
    double planning_time_s,
    double fraction,
    bool overwrite = true);

  // QUERY CONSTRUCTION
  bool extract_and_append_cartesian_plan_start_to_query(
    warehouse_ros::Query& query,
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
    double match_tolerance);

  bool extract_and_append_cartesian_plan_goal_to_query(
    warehouse_ros::Query& query,
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request,
    double match_tolerance);

  // METADATA CONSTRUCTION
  bool extract_and_append_cartesian_plan_start_to_metadata(
    warehouse_ros::Metadata& metadata,
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request);

  bool extract_and_append_cartesian_plan_goal_to_metadata(
    warehouse_ros::Metadata& metadata,
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit_msgs::srv::GetCartesianPath::Request& plan_request);

private:
  rclcpp::Node::SharedPtr node_;
  warehouse_ros::DatabaseConnection::Ptr db_;

  double exact_match_precision_ = 1e-6;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace planning_interface
}  // namespace moveit

#endif // SRC__MOTION_PLAN_CACHE_HPP