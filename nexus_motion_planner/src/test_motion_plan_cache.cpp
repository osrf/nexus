// Copyright 2023 Johnson & Johnson
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

#include <rclcpp/rclcpp.hpp>

#include "moveit/robot_state/conversions.h"
#include "moveit/robot_state/robot_state.h"
#include "motion_plan_cache.hpp"

#include <thread>

using moveit::planning_interface::MoveGroupInterface;
using nexus::motion_planner::MotionPlanCache;

const std::string g_robot_name = "panda";
const std::string g_robot_frame = "world";

// UTILS =======================================================================
// Utility function to emit a pass or fail statement.
void check_and_emit(
  bool predicate, const std::string& prefix, const std::string& label)
{
  if (predicate)
  {
    std::cout << "[PASS] " << prefix << ": " << label << std::endl;
  }
  else
  {
    std::cout << "[FAIL] " << prefix << ": " << label << std::endl;
    exit(1);
  }
}

moveit_msgs::msg::RobotTrajectory get_dummy_panda_plan()
{
  static moveit_msgs::msg::RobotTrajectory out = []()
    {
      moveit_msgs::msg::RobotTrajectory plan;

      auto traj = &plan.joint_trajectory;
      traj->header.frame_id = g_robot_frame;

      traj->joint_names.push_back(g_robot_name + "_joint1");
      traj->joint_names.push_back(g_robot_name + "_joint2");
      traj->joint_names.push_back(g_robot_name + "_joint3");
      traj->joint_names.push_back(g_robot_name + "_joint4");
      traj->joint_names.push_back(g_robot_name + "_joint5");
      traj->joint_names.push_back(g_robot_name + "_joint6");
      traj->joint_names.push_back(g_robot_name + "_joint7");

      traj->points.emplace_back();
      traj->points.at(0).positions = {0, 0, 0, 0, 0, 0};
      traj->points.at(0).velocities = {0, 0, 0, 0, 0, 0};
      traj->points.at(0).accelerations = {0, 0, 0, 0, 0, 0};
      traj->points.at(0).time_from_start.sec = 999999;

      return plan;
    }();
  return out;
}

std::vector<geometry_msgs::msg::Pose> get_dummy_waypoints()
{
  static std::vector<geometry_msgs::msg::Pose> out = []()
    {
      std::vector<geometry_msgs::msg::Pose> waypoints;
      for (size_t i = 0; i < 3; i++)
      {
        waypoints.emplace_back();
        waypoints.at(i).position.x = i;
        waypoints.at(i).position.y = i;
        waypoints.at(i).position.z = i;
        waypoints.at(i).orientation.w = i;
        waypoints.at(i).orientation.x = i + 0.1;
        waypoints.at(i).orientation.y = i + 0.1;
        waypoints.at(i).orientation.z = i + 0.1;
      }
      return waypoints;
    }();
  return out;
}


void test_motion_plans(
  std::shared_ptr<MoveGroupInterface> move_group,
  std::shared_ptr<MotionPlanCache> cache)
{
  // Setup =====================================================================
  // All variants are copies.

  /// MotionPlanRequest

  // Plain start
  moveit_msgs::msg::MotionPlanRequest plan_req;
  move_group->constructMotionPlanRequest(plan_req);
  plan_req.workspace_parameters.header.frame_id = g_robot_frame;
  plan_req.workspace_parameters.max_corner.x = 10;
  plan_req.workspace_parameters.max_corner.y = 10;
  plan_req.workspace_parameters.max_corner.z = 10;
  plan_req.workspace_parameters.min_corner.x = -10;
  plan_req.workspace_parameters.min_corner.y = -10;
  plan_req.workspace_parameters.min_corner.z = -10;
  plan_req.start_state.multi_dof_joint_state.joint_names.clear();
  plan_req.start_state.multi_dof_joint_state.transforms.clear();
  plan_req.start_state.multi_dof_joint_state.twist.clear();
  plan_req.start_state.multi_dof_joint_state.wrench.clear();

  // Empty frame start
  moveit_msgs::msg::MotionPlanRequest empty_frame_plan_req = plan_req;
  empty_frame_plan_req.workspace_parameters.header.frame_id = "";

  // is_diff = true
  auto is_diff_plan_req = plan_req;
  is_diff_plan_req.start_state.is_diff = true;
  is_diff_plan_req.start_state.joint_state.header.frame_id = "";
  is_diff_plan_req.start_state.joint_state.name.clear();
  is_diff_plan_req.start_state.joint_state.position.clear();
  is_diff_plan_req.start_state.joint_state.velocity.clear();
  is_diff_plan_req.start_state.joint_state.effort.clear();

  // Something close enough (mod 0.1 away)
  auto close_matching_plan_req = plan_req;
  close_matching_plan_req.workspace_parameters.max_corner.x += 0.05;
  close_matching_plan_req.workspace_parameters.min_corner.x -= 0.05;
  close_matching_plan_req.start_state.joint_state.position.at(0) -= 0.05;
  close_matching_plan_req.start_state.joint_state.position.at(1) += 0.05;
  close_matching_plan_req.start_state.joint_state.position.at(2) -= 0.05;
  close_matching_plan_req.goal_constraints
  .at(0).joint_constraints.at(0).position -= 0.05;
  close_matching_plan_req.goal_constraints
  .at(0).joint_constraints.at(1).position += 0.05;
  close_matching_plan_req.goal_constraints
  .at(0).joint_constraints.at(2).position -= 0.05;

  // Close with swapped constraints (mod 0.1 away)
  auto swapped_close_matching_plan_req = close_matching_plan_req;
  std::swap(
    swapped_close_matching_plan_req.goal_constraints.at(
      0).joint_constraints.at(0),
    swapped_close_matching_plan_req.goal_constraints.at(
      0).joint_constraints.at(1));

  // Smaller workspace start
  auto smaller_workspace_plan_req = plan_req;
  smaller_workspace_plan_req.workspace_parameters.max_corner.x = 1;
  smaller_workspace_plan_req.workspace_parameters.max_corner.y = 1;
  smaller_workspace_plan_req.workspace_parameters.max_corner.z = 1;
  smaller_workspace_plan_req.workspace_parameters.min_corner.x = -1;
  smaller_workspace_plan_req.workspace_parameters.min_corner.y = -1;
  smaller_workspace_plan_req.workspace_parameters.min_corner.z = -1;

  // Larger workspace start
  auto larger_workspace_plan_req = plan_req;
  larger_workspace_plan_req.workspace_parameters.max_corner.x = 50;
  larger_workspace_plan_req.workspace_parameters.max_corner.y = 50;
  larger_workspace_plan_req.workspace_parameters.max_corner.z = 50;
  larger_workspace_plan_req.workspace_parameters.min_corner.x = -50;
  larger_workspace_plan_req.workspace_parameters.min_corner.y = -50;
  larger_workspace_plan_req.workspace_parameters.min_corner.z = -50;

  // Different
  auto different_plan_req = plan_req;
  different_plan_req.workspace_parameters.max_corner.x += 1.05;
  different_plan_req.workspace_parameters.min_corner.x -= 2.05;
  different_plan_req.start_state.joint_state.position.at(0) -= 3.05;
  different_plan_req.start_state.joint_state.position.at(1) += 4.05;
  different_plan_req.start_state.joint_state.position.at(2) -= 5.05;
  different_plan_req.goal_constraints
  .at(0).joint_constraints.at(0).position -= 6.05;
  different_plan_req.goal_constraints
  .at(0).joint_constraints.at(1).position += 7.05;
  different_plan_req.goal_constraints
  .at(0).joint_constraints.at(2).position -= 8.05;

  /// RobotTrajectory

  // Plan
  auto plan = get_dummy_panda_plan();

  // Plan with no frame_id in its trajectory header
  auto empty_frame_plan = plan;
  empty_frame_plan.joint_trajectory.header.frame_id = "";

  auto different_plan = plan;
  different_plan.joint_trajectory.points.at(0).positions.at(0) = 999;
  different_plan.joint_trajectory.points.at(0).positions.at(1) = 999;
  different_plan.joint_trajectory.points.at(0).positions.at(2) = 999;

  // Test Utils

  std::string prefix;

  // Checks ====================================================================

  // Initially empty
  prefix = "test_motion_plans.empty";

  check_and_emit(
    cache->count_plans(g_robot_name) == 0,
    prefix, "Plan cache initially empty");

  check_and_emit(
    cache->fetch_all_matching_plans(
      *move_group, g_robot_name, plan_req, 999, 999).empty(),
    prefix, "Fetch all plans on empty cache returns empty");

  check_and_emit(
    cache->fetch_best_matching_plan(
      *move_group, g_robot_name, plan_req, 999, 999) == nullptr,
    prefix, "Fetch best plan on empty cache returns nullptr");

  // Put plan empty frame
  //
  // Plan must have frame in joint trajectory, expect put fail
  prefix = "test_motion_plans.put_plan_empty_frame";
  double execution_time = 999;
  double planning_time = 999;

  check_and_emit(
    !cache->put_plan(
      *move_group, g_robot_name, plan_req, empty_frame_plan,
      execution_time, planning_time, false),
    prefix, "Put empty frame plan, no delete_worse_plans, not ok");

  check_and_emit(
    cache->count_plans(g_robot_name) == 0, prefix, "No plans in cache");

  // Put plan req empty frame
  //
  // Plan request must have frame in workspace, expect put fail
  prefix = "test_motion_plans.put_plan_req_empty_frame";
  execution_time = 999;
  planning_time = 999;

  check_and_emit(
    !cache->put_plan(
      *move_group, g_robot_name, empty_frame_plan_req, plan,
      execution_time, planning_time, false),
    prefix, "Put empty frame req plan, no delete_worse_plans, not ok");

  check_and_emit(
    cache->count_plans(g_robot_name) == 0, prefix, "No plans in cache");

  // Put first, no delete_worse_plans
  prefix = "test_motion_plans.put_first";
  execution_time = 999;
  planning_time = 999;

  check_and_emit(
    cache->put_plan(
      *move_group, g_robot_name, plan_req, plan,
      execution_time, planning_time, false),
    prefix, "Put first valid plan, no delete_worse_plans, ok");

  check_and_emit(
    cache->count_plans(g_robot_name) == 1, prefix, "One plan in cache");

  // Fetch matching, no tolerance
  //
  // Exact key match should have cache hit
  prefix = "test_motion_plans.put_first.fetch_matching_no_tolerance";

  auto fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, plan_req, 0, 0);

  auto fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, plan_req, 0, 0);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble("execution_time_s") == execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  // Fetch with is_diff
  //
  // is_diff key should be equivalent to exact match if robot state did not
  // change, hence should have cache hit
  prefix = "test_motion_plans.put_first.fetch_is_diff_no_tolerance";

  fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, is_diff_plan_req, 0, 0);

  fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, is_diff_plan_req, 0, 0);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble("execution_time_s") == execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  // Fetch non-matching, out of tolerance
  //
  // Non-matching key should not have cache hit
  prefix = "test_motion_plans.put_first.fetch_non_matching_out_of_tolerance";

  fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, close_matching_plan_req, 0, 0);

  fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, close_matching_plan_req, 0, 0);

  check_and_emit(fetched_plans.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(
    fetched_plan == nullptr, prefix, "Fetch best plan is nullptr");

  // Fetch non-matching, only start in tolerance (but not goal)
  //
  // Non-matching key should not have cache hit
  prefix =
    "test_motion_plans.put_first.fetch_non_matching_only_start_in_tolerance";

  fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, close_matching_plan_req, 999, 0);

  fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, close_matching_plan_req, 999, 0);

  check_and_emit(fetched_plans.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(
    fetched_plan == nullptr, prefix, "Fetch best plan is nullptr");

  // Fetch non-matching, only goal in tolerance (but not start)
  //
  // Non-matching key should not have cache hit
  prefix =
    "test_motion_plans.put_first.fetch_non_matching_only_goal_in_tolerance";

  fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, close_matching_plan_req, 0, 999);

  fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, close_matching_plan_req, 0, 999);

  check_and_emit(fetched_plans.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(
    fetched_plan == nullptr, prefix, "Fetch best plan is nullptr");

  // Fetch non-matching, in tolerance
  //
  // Close key within tolerance limit should have cache hit
  prefix = "test_motion_plans.put_first.fetch_non_matching_in_tolerance";

  fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, close_matching_plan_req, 0.1, 0.1);

  fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, close_matching_plan_req, 0.1, 0.1);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble("execution_time_s") == execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  // Fetch swapped
  //
  // Matches should be mostly invariant to constraint ordering
  prefix = "test_motion_plans.put_first.fetch_swapped";

  fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, swapped_close_matching_plan_req, 0.1, 0.1);

  fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, swapped_close_matching_plan_req, 0.1, 0.1);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble("execution_time_s") == execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  // Fetch with smaller workspace
  //
  // Matching plans with more restrictive workspace requirements should not
  // pull up plans cached for less restrictive workspace requirements
  prefix = "test_motion_plans.put_first.fetch_smaller_workspace";

  fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, smaller_workspace_plan_req, 999, 999);

  fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, smaller_workspace_plan_req, 999, 999);

  check_and_emit(fetched_plans.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(
    fetched_plan == nullptr, prefix, "Fetch best plan is nullptr");

  // Fetch with larger workspace
  //
  // Matching plans with less restrictive workspace requirements should pull up
  // plans cached for more restrictive workspace requirements
  prefix = "test_motion_plans.put_first.fetch_larger_workspace";

  fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, larger_workspace_plan_req, 999, 999);

  fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, larger_workspace_plan_req, 999, 999);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble("execution_time_s") == execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  check_and_emit(
    fetched_plan->lookupDouble(
      "workspace_parameters.max_corner.x")
    <= larger_workspace_plan_req.workspace_parameters.max_corner.x,
    prefix, "Fetched plan has more restrictive workspace max_corner");

  check_and_emit(
    fetched_plan->lookupDouble(
      "workspace_parameters.max_corner.x")
    >= larger_workspace_plan_req.workspace_parameters.min_corner.x,
    prefix, "Fetched plan has more restrictive workspace min_corner");

  // Put worse, no delete_worse_plans
  //
  // Worse plans should not be inserted
  prefix = "test_motion_plans.put_worse";
  double worse_execution_time = execution_time + 100;

  check_and_emit(
    !cache->put_plan(
      *move_group, g_robot_name, plan_req, plan,
      worse_execution_time, planning_time, false),
    prefix, "Put worse plan, no delete_worse_plans, not ok");

  check_and_emit(
    cache->count_plans(g_robot_name) == 1, prefix, "One plan in cache");

  // Put better, no delete_worse_plans
  //
  // Better plans should be inserted
  prefix = "test_motion_plans.put_better";
  double better_execution_time = execution_time - 100;

  check_and_emit(
    cache->put_plan(
      *move_group, g_robot_name, plan_req, plan,
      better_execution_time, planning_time, false),
    prefix, "Put better plan, no delete_worse_plans, ok");

  check_and_emit(
    cache->count_plans(g_robot_name) == 2, prefix, "Two plans in cache");

  // Fetch sorted
  //
  // With multiple plans in cache, fetches should be sorted accordingly
  prefix = "test_motion_plans.put_better.fetch_sorted";

  fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, plan_req, 0, 0);

  fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, plan_req, 0, 0);

  check_and_emit(fetched_plans.size() == 2, prefix, "Fetch all returns two");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble("execution_time_s") == better_execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  check_and_emit(
    fetched_plans.at(0)->lookupDouble(
      "execution_time_s") == better_execution_time
    && fetched_plans.at(1)->lookupDouble("execution_time_s") == execution_time,
    prefix, "Fetched plans are sorted correctly");

  // Put better, delete_worse_plans
  //
  // Better, different, plans should be inserted
  prefix = "test_motion_plans.put_better_delete_worse_plans";
  double even_better_execution_time = better_execution_time - 100;

  check_and_emit(
    cache->put_plan(
      *move_group, g_robot_name, plan_req, different_plan,
      even_better_execution_time, planning_time, true),
    prefix, "Put better plan, delete_worse_plans, ok");

  check_and_emit(
    cache->count_plans(g_robot_name) == 1, prefix, "One plan in cache");

  // Fetch better plan
  prefix = "test_motion_plans.put_better_delete_worse_plans.fetch";

  fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, plan_req, 0, 0);

  fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, plan_req, 0, 0);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == different_plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble(
      "execution_time_s") == even_better_execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  // Put different req, delete_worse_plans
  //
  // Unrelated plan requests should live alongside pre-existing plans
  prefix = "test_motion_plans.put_different_req";

  check_and_emit(
    cache->put_plan(
      *move_group, g_robot_name, different_plan_req, different_plan,
      better_execution_time, planning_time, true),
    prefix, "Put different plan req, delete_worse_plans, ok");

  check_and_emit(
    cache->count_plans(g_robot_name) == 2, prefix, "Two plans in cache");

  // Fetch with different plan req
  //
  // With multiple plans in cache, fetches should be sorted accordingly
  prefix = "test_motion_plans.put_different_req.fetch";

  fetched_plans = cache->fetch_all_matching_plans(
    *move_group, g_robot_name, different_plan_req, 0, 0);

  fetched_plan = cache->fetch_best_matching_plan(
    *move_group, g_robot_name, different_plan_req, 0, 0);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == different_plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble(
      "execution_time_s") == better_execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");
}

void test_cartesian_plans(
  std::shared_ptr<MoveGroupInterface> move_group,
  std::shared_ptr<MotionPlanCache> cache)
{
  std::string prefix;

  /// First, test if construction even works...

  // Construct get cartesian plan request
  prefix = "test_cartesian_plans.construct_get_cartesian_path_request";

  int test_step = 1;
  int test_jump = 2;
  auto test_waypoints = get_dummy_waypoints();
  auto cartesian_plan_req_under_test =
    cache->construct_get_cartesian_plan_request(
    *move_group, test_waypoints, test_step, test_jump, false);

  check_and_emit(
    cartesian_plan_req_under_test.waypoints == test_waypoints
    && static_cast<int>(
      cartesian_plan_req_under_test.max_step) == test_step
    && static_cast<int>(
      cartesian_plan_req_under_test.jump_threshold) == test_jump
    && !cartesian_plan_req_under_test.avoid_collisions,
    prefix, "Ok");

  // Setup =====================================================================
  // All variants are copies.

  /// GetCartesianPath::Request

  // Plain start
  auto waypoints = get_dummy_waypoints();
  auto cartesian_plan_req = cache
    ->construct_get_cartesian_plan_request(*move_group, waypoints, 1, 1, false);
  cartesian_plan_req.start_state.multi_dof_joint_state.joint_names.clear();
  cartesian_plan_req.start_state.multi_dof_joint_state.transforms.clear();
  cartesian_plan_req.start_state.multi_dof_joint_state.twist.clear();
  cartesian_plan_req.start_state.multi_dof_joint_state.wrench.clear();
  cartesian_plan_req.path_constraints.joint_constraints.clear();
  cartesian_plan_req.path_constraints.position_constraints.clear();
  cartesian_plan_req.path_constraints.orientation_constraints.clear();
  cartesian_plan_req.path_constraints.visibility_constraints.clear();

  // Empty frame start
  auto empty_frame_cartesian_plan_req = cartesian_plan_req;
  empty_frame_cartesian_plan_req.header.frame_id = "";

  // is_diff = true
  auto is_diff_cartesian_plan_req = cartesian_plan_req;
  is_diff_cartesian_plan_req.start_state.is_diff = true;
  is_diff_cartesian_plan_req.start_state.joint_state.header.frame_id = "";
  is_diff_cartesian_plan_req.start_state.joint_state.name.clear();
  is_diff_cartesian_plan_req.start_state.joint_state.position.clear();
  is_diff_cartesian_plan_req.start_state.joint_state.velocity.clear();
  is_diff_cartesian_plan_req.start_state.joint_state.effort.clear();

  // Something close enough (mod 0.1 away)
  auto close_matching_cartesian_plan_req = cartesian_plan_req;
  close_matching_cartesian_plan_req.start_state.joint_state.position.at(0) -=
    0.05;
  close_matching_cartesian_plan_req.start_state.joint_state.position.at(1) +=
    0.05;
  close_matching_cartesian_plan_req.start_state.joint_state.position.at(2) -=
    0.05;
  close_matching_cartesian_plan_req.waypoints.at(0).position.x -= 0.05;
  close_matching_cartesian_plan_req.waypoints.at(1).position.x += 0.05;
  close_matching_cartesian_plan_req.waypoints.at(2).position.x -= 0.05;

  // Different
  auto different_cartesian_plan_req = cartesian_plan_req;
  different_cartesian_plan_req.start_state.joint_state.position.at(0) -= 1.05;
  different_cartesian_plan_req.start_state.joint_state.position.at(1) += 2.05;
  different_cartesian_plan_req.start_state.joint_state.position.at(2) -= 3.05;
  different_cartesian_plan_req.waypoints.at(0).position.x -= 1.05;
  different_cartesian_plan_req.waypoints.at(1).position.x += 2.05;
  different_cartesian_plan_req.waypoints.at(2).position.x -= 3.05;

  /// RobotTrajectory

  // Plan
  auto plan = get_dummy_panda_plan();

  // Plan with no frame_id in its trajectory header
  auto empty_frame_plan = plan;
  empty_frame_plan.joint_trajectory.header.frame_id = "";

  auto different_plan = plan;
  different_plan.joint_trajectory.points.at(0).positions.at(0) = 999;
  different_plan.joint_trajectory.points.at(0).positions.at(1) = 999;
  different_plan.joint_trajectory.points.at(0).positions.at(2) = 999;

  // Checks ====================================================================

  // Initially empty
  prefix = "test_cartesian_plans.empty";

  check_and_emit(
    cache->count_cartesian_plans(g_robot_name) == 0,
    prefix, "Plan cache initially empty");

  check_and_emit(
    cache->fetch_all_matching_cartesian_plans(
      *move_group, g_robot_name, cartesian_plan_req, 0, 999, 999).empty(),
    prefix, "Fetch all plans on empty cache returns empty");

  check_and_emit(
    cache->fetch_best_matching_cartesian_plan(
      *move_group, g_robot_name, cartesian_plan_req, 0, 999, 999) == nullptr,
    prefix, "Fetch best plan on empty cache returns nullptr");

  // Put plan empty frame
  //
  // Plan must have frame in joint trajectory, expect put fail
  prefix = "test_cartesian_plans.put_plan_empty_frame";
  double execution_time = 999;
  double planning_time = 999;
  double fraction = 0.5;

  check_and_emit(
    !cache->put_cartesian_plan(
      *move_group, g_robot_name, cartesian_plan_req, empty_frame_plan,
      execution_time, planning_time, fraction, false),
    prefix, "Put empty frame plan, no delete_worse_plans, not ok");

  check_and_emit(
    cache->count_cartesian_plans(g_robot_name) == 0,
    prefix, "No plans in cache");

  // Put plan req empty frame
  //
  // Plan request must have frame in workspace, expect put fail
  prefix = "test_cartesian_plans.put_plan_req_empty_frame";
  execution_time = 999;
  planning_time = 999;

  check_and_emit(
    !cache->put_cartesian_plan(
      *move_group, g_robot_name, empty_frame_cartesian_plan_req, plan,
      execution_time, planning_time, fraction, false),
    prefix, "Put empty frame req plan, no delete_worse_plans, not ok");

  check_and_emit(
    cache->count_cartesian_plans(g_robot_name) == 0,
    prefix, "No plans in cache");

  // Put first, no delete_worse_plans
  prefix = "test_cartesian_plans.put_first";
  execution_time = 999;
  planning_time = 999;

  check_and_emit(
    cache->put_cartesian_plan(
      *move_group, g_robot_name, cartesian_plan_req, plan,
      execution_time, planning_time, fraction, false),
    prefix, "Put first valid plan, no delete_worse_plans, ok");

  check_and_emit(
    cache->count_cartesian_plans(g_robot_name) == 1,
    prefix, "One plan in cache");

  // Fetch matching, no tolerance
  //
  // Exact key match should have cache hit
  prefix = "test_cartesian_plans.put_first.fetch_matching_no_tolerance";

  auto fetched_plans = cache->fetch_all_matching_cartesian_plans(
    *move_group, g_robot_name, cartesian_plan_req, fraction, 0, 0);

  auto fetched_plan = cache->fetch_best_matching_cartesian_plan(
    *move_group, g_robot_name, cartesian_plan_req, fraction, 0, 0);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble("execution_time_s") == execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  check_and_emit(
    fetched_plan->lookupDouble("fraction") == fraction,
    prefix, "Fetched plan has correct fraction");

  // Fetch with is_diff
  //
  // is_diff key should be equivalent to exact match if robot state did not
  // change, hence should have cache hit
  prefix = "test_cartesian_plans.put_first.fetch_is_diff_no_tolerance";

  fetched_plans = cache->fetch_all_matching_cartesian_plans(
    *move_group, g_robot_name, is_diff_cartesian_plan_req, fraction, 0, 0);

  fetched_plan = cache->fetch_best_matching_cartesian_plan(
    *move_group, g_robot_name, is_diff_cartesian_plan_req, fraction, 0, 0);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble("execution_time_s") == execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  check_and_emit(
    fetched_plan->lookupDouble("fraction") == fraction,
    prefix, "Fetched plan has correct fraction");

  // Fetch non-matching, out of tolerance
  //
  // Non-matching key should not have cache hit
  prefix = "test_cartesian_plans.put_first.fetch_non_matching_out_of_tolerance";

  fetched_plans = cache->fetch_all_matching_cartesian_plans(
    *move_group, g_robot_name, close_matching_cartesian_plan_req,
    fraction, 0, 0);

  fetched_plan = cache->fetch_best_matching_cartesian_plan(
    *move_group, g_robot_name, close_matching_cartesian_plan_req,
    fraction, 0, 0);

  check_and_emit(fetched_plans.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(
    fetched_plan == nullptr, prefix, "Fetch best plan is nullptr");

  // Fetch non-matching, only start in tolerance (but not goal)
  //
  // Non-matching key should not have cache hit
  prefix =
    "test_motion_plans.put_first.fetch_non_matching_only_start_in_tolerance";

  fetched_plans = cache->fetch_all_matching_cartesian_plans(
    *move_group, g_robot_name, close_matching_cartesian_plan_req,
    fraction, 999, 0);

  fetched_plan = cache->fetch_best_matching_cartesian_plan(
    *move_group, g_robot_name, close_matching_cartesian_plan_req,
    fraction, 999, 0);

  check_and_emit(fetched_plans.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(
    fetched_plan == nullptr, prefix, "Fetch best plan is nullptr");

  // Fetch non-matching, only goal in tolerance (but not start)
  //
  // Non-matching key should not have cache hit
  prefix =
    "test_motion_plans.put_first.fetch_non_matching_only_goal_in_tolerance";

  fetched_plans = cache->fetch_all_matching_cartesian_plans(
    *move_group, g_robot_name, close_matching_cartesian_plan_req,
    fraction, 0, 999);

  fetched_plan = cache->fetch_best_matching_cartesian_plan(
    *move_group, g_robot_name, close_matching_cartesian_plan_req,
    fraction, 0, 999);

  check_and_emit(fetched_plans.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(
    fetched_plan == nullptr, prefix, "Fetch best plan is nullptr");

  // Fetch non-matching, in tolerance
  //
  // Close key within tolerance limit should have cache hit
  prefix = "test_cartesian_plans.put_first.fetch_non_matching_in_tolerance";

  fetched_plans = cache->fetch_all_matching_cartesian_plans(
    *move_group, g_robot_name, close_matching_cartesian_plan_req,
    fraction, 0.1, 0.1);

  fetched_plan = cache->fetch_best_matching_cartesian_plan(
    *move_group, g_robot_name, close_matching_cartesian_plan_req,
    fraction, 0.1, 0.1);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble("execution_time_s") == execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  check_and_emit(
    fetched_plan->lookupDouble("fraction") == fraction,
    prefix, "Fetched plan has correct fraction");

  // Fetch with higher fraction
  //
  // Matching plans with more restrictive fraction requirements should not
  // pull up plans cached for less restrictive fraction requirements
  prefix = "test_cartesian_plans.put_first.fetch_higher_fraction";

  fetched_plans = cache->fetch_all_matching_cartesian_plans(
    *move_group, g_robot_name, cartesian_plan_req, 1, 999, 999);

  fetched_plan = cache->fetch_best_matching_cartesian_plan(
    *move_group, g_robot_name, cartesian_plan_req, 1, 999, 999);

  check_and_emit(fetched_plans.size() == 0, prefix, "Fetch all returns empty");
  check_and_emit(
    fetched_plan == nullptr, prefix, "Fetch best plan is nullptr");

  // Fetch with lower fraction
  //
  // Matching plans with less restrictive fraction requirements should pull up
  // plans cached for more restrictive fraction requirements
  prefix = "test_cartesian_plans.put_first.fetch_lower_fraction";

  fetched_plans = cache->fetch_all_matching_cartesian_plans(
    *move_group, g_robot_name, cartesian_plan_req, 0, 999, 999);

  fetched_plan = cache->fetch_best_matching_cartesian_plan(
    *move_group, g_robot_name, cartesian_plan_req, 0, 999, 999);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble("execution_time_s") == execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  check_and_emit(
    fetched_plan->lookupDouble("fraction") == fraction,
    prefix, "Fetched plan has correct fraction");

  // Put worse, no delete_worse_plans
  //
  // Worse plans should not be inserted
  prefix = "test_cartesian_plans.put_worse";
  double worse_execution_time = execution_time + 100;

  check_and_emit(
    !cache->put_cartesian_plan(
      *move_group, g_robot_name, cartesian_plan_req, plan,
      worse_execution_time, planning_time, fraction, false),
    prefix, "Put worse plan, no delete_worse_plans, not ok");

  check_and_emit(
    cache->count_cartesian_plans(g_robot_name) == 1,
    prefix, "One plan in cache");

  // Put better, no delete_worse_plans
  //
  // Better plans should be inserted
  prefix = "test_cartesian_plans.put_better";
  double better_execution_time = execution_time - 100;

  check_and_emit(
    cache->put_cartesian_plan(
      *move_group, g_robot_name, cartesian_plan_req, plan,
      better_execution_time, planning_time, fraction, false),
    prefix, "Put better plan, no delete_worse_plans, ok");

  check_and_emit(
    cache->count_cartesian_plans(g_robot_name) == 2,
    prefix, "Two plans in cache");

  // Fetch sorted
  //
  // With multiple plans in cache, fetches should be sorted accordingly
  prefix = "test_cartesian_plans.put_better.fetch_sorted";

  fetched_plans = cache->fetch_all_matching_cartesian_plans(
    *move_group, g_robot_name, cartesian_plan_req, fraction, 0, 0);

  fetched_plan = cache->fetch_best_matching_cartesian_plan(
    *move_group, g_robot_name, cartesian_plan_req, fraction, 0, 0);

  check_and_emit(fetched_plans.size() == 2, prefix, "Fetch all returns two");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble("execution_time_s") == better_execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  check_and_emit(
    fetched_plan->lookupDouble("fraction") == fraction,
    prefix, "Fetched plan has correct fraction");

  check_and_emit(
    fetched_plans.at(0)->lookupDouble(
      "execution_time_s") == better_execution_time
    && fetched_plans.at(1)->lookupDouble("execution_time_s") == execution_time,
    prefix, "Fetched plans are sorted correctly");

  // Put better, delete_worse_plans
  //
  // Better, different, plans should be inserted
  prefix = "test_cartesian_plans.put_better_delete_worse_plans";
  double even_better_execution_time = better_execution_time - 100;

  check_and_emit(
    cache->put_cartesian_plan(
      *move_group, g_robot_name, cartesian_plan_req, different_plan,
      even_better_execution_time, planning_time, fraction, true),
    prefix, "Put better plan, delete_worse_plans, ok");

  check_and_emit(
    cache->count_cartesian_plans(g_robot_name) == 1,
    prefix, "One plan in cache");

  // Fetch better plan
  prefix = "test_cartesian_plans.put_better_delete_worse_plans.fetch";

  fetched_plans = cache->fetch_all_matching_cartesian_plans(
    *move_group, g_robot_name, cartesian_plan_req, fraction, 0, 0);

  fetched_plan = cache->fetch_best_matching_cartesian_plan(
    *move_group, g_robot_name, cartesian_plan_req, fraction, 0, 0);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == different_plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble(
      "execution_time_s") == even_better_execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  check_and_emit(
    fetched_plan->lookupDouble("fraction") == fraction,
    prefix, "Fetched plan has correct fraction");

  // Put different req, delete_worse_plans
  //
  // Unrelated plan requests should live alongside pre-existing plans
  prefix = "test_cartesian_plans.put_different_req";

  check_and_emit(
    cache->put_cartesian_plan(
      *move_group, g_robot_name, different_cartesian_plan_req,
      different_plan,
      better_execution_time, planning_time, fraction, true),
    prefix, "Put different plan req, delete_worse_plans, ok");

  check_and_emit(
    cache->count_cartesian_plans(g_robot_name) == 2,
    prefix, "Two plans in cache");

  // Fetch with different plan req
  //
  // With multiple plans in cache, fetches should be sorted accordingly
  prefix = "test_cartesian_plans.put_different_req.fetch";

  fetched_plans = cache->fetch_all_matching_cartesian_plans(
    *move_group, g_robot_name, different_cartesian_plan_req, fraction, 0, 0);

  fetched_plan = cache->fetch_best_matching_cartesian_plan(
    *move_group, g_robot_name, different_cartesian_plan_req, fraction, 0, 0);

  check_and_emit(fetched_plans.size() == 1, prefix, "Fetch all returns one");
  check_and_emit(
    fetched_plan != nullptr, prefix, "Fetch best plan is not nullptr");

  check_and_emit(
    *fetched_plans.at(0) == *fetched_plan,
    prefix, "Fetched plan on both fetches match");

  check_and_emit(
    *fetched_plan == different_plan, prefix, "Fetched plan matches original");

  check_and_emit(
    fetched_plan->lookupDouble(
      "execution_time_s") == better_execution_time,
    prefix, "Fetched plan has correct execution time");

  check_and_emit(
    fetched_plan->lookupDouble("planning_time_s") == planning_time,
    prefix, "Fetched plan has correct planning time");

  check_and_emit(
    fetched_plan->lookupDouble("fraction") == fraction,
    prefix, "Fetched plan has correct fraction");
}

int main(int argc, char** argv)
{
  // Setup
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto test_node = rclcpp::Node::make_shared("test_node", node_options);
  std::thread spin_thread(
    [&]()
    {
      while (rclcpp::ok())
      {
        rclcpp::spin_some(test_node);
      }
    }
  );

  // This is necessary
  test_node->declare_parameter<std::string>(
    "warehouse_plugin", "warehouse_ros_sqlite::DatabaseConnection");

  // Test proper
  {
    auto cache = std::make_shared<MotionPlanCache>(test_node);
    check_and_emit(cache->init(":memory:", 0, 1e-4), "init", "Cache init");

    auto move_group =
      std::make_shared<MoveGroupInterface>(test_node, "panda_arm");

    auto curr_state = move_group->getCurrentState();
    move_group->setStartState(*curr_state);

    test_motion_plans(move_group, cache);
    test_cartesian_plans(move_group, cache);
  }

  rclcpp::shutdown();
  spin_thread.join();

  return 0;
}

// CARTESIAN PLANS ===

// Check construct plan request
// Remember to check fraction too!