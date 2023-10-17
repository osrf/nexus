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

#ifndef SRC__MOTION_PLANNER_SERVER_HPP
#define SRC__MOTION_PLANNER_SERVER_HPP

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// TF2
#include <tf2_ros/buffer.h>

// ROS2 Messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>

// moveit modules
#include <moveit/move_group_interface/move_group_interface.h>

// NEXUS messages
#include <nexus_endpoints.hpp>

// NEXUS motion plan cache
#include "motion_plan_cache.hpp"

namespace nexus {
namespace motion_planner {

//==============================================================================
class MotionPlannerServer : public rclcpp_lifecycle::LifecycleNode
{
public:

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  using GetMotionPlanService = nexus::endpoints::GetMotionPlanService;
  using Request = GetMotionPlanService::ServiceType::Request::ConstSharedPtr;
  using Response = GetMotionPlanService::ServiceType::Response::SharedPtr;
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using LifecycleState = rclcpp_lifecycle::State;
  /**
   * @brief Constructor for a new Motion Planner Server object
   *
   * @param options Node options
   */
  MotionPlannerServer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const LifecycleState& state) override;
  CallbackReturn on_activate(const LifecycleState& state) override;
  CallbackReturn on_deactivate(const LifecycleState& state) override;
  CallbackReturn on_cleanup(const LifecycleState& state) override;
  CallbackReturn on_shutdown(const LifecycleState& state) override;

  ~MotionPlannerServer();

private:
  std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
  rclcpp::Node::SharedPtr _internal_node;
  std::thread _spin_thread;

  rclcpp::Node::SharedPtr _internal_cache_node;
  std::thread _cache_spin_thread;

  // MoveIt planning
  std::vector<std::string> _manipulators;
  bool _use_move_group_interfaces;
  bool _use_namespace;
  std::chrono::nanoseconds _timeout_duration;
  double _goal_tolerance;
  double _planning_time;
  uint32_t _replan_attempts;
  bool _execute_trajectory;
  bool _collision_aware_cartesian_path;
  double _cartesian_max_step;
  double _cartesian_jump_threshold;
  double _get_state_wait_seconds;
  double _workspace_min_x;
  double _workspace_min_y;
  double _workspace_min_z;
  double _workspace_max_x;
  double _workspace_max_y;
  double _workspace_max_z;

  // Motion plan caching
  std::unique_ptr<nexus::motion_planner::MotionPlanCache> _motion_plan_cache;

  bool _use_motion_plan_cache;
  bool _use_cached_plans_instead_of_planning;
  bool _overwrite_worse_plans;
  std::string _cache_db_plugin;
  std::string _cache_db_host;
  int _cache_db_port;
  double _cache_exact_match_tolerance;
  double _cache_start_match_tolerance;
  double _cache_goal_match_tolerance;

  rclcpp::Service<GetMotionPlanService::ServiceType>::SharedPtr _plan_srv;

  std::unordered_map<std::string, std::shared_ptr<MoveGroupInterface>>
  _move_group_interfaces;
  // Map manipulator name to its default group name
  std::unordered_map<std::string, std::string> _group_names;

  bool initialize_move_group_interfaces();

  void plan_with_move_group(
    const GetMotionPlanService::ServiceType::Request& req,
    Response res);
}; // class MotionPlannerServer

}  // namespace planning_interface
}  // namespace moveit

#endif // SRC__MOTION_PLANNER_SERVER_HPP
