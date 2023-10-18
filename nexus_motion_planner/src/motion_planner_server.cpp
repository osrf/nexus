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

#include "motion_planner_server.hpp"

std::string str_tolower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) { return std::tolower(c); }
  );
  return s;
}

namespace nexus {
namespace motion_planner {

constexpr bool cache_mode_is_execute(PlannerDatabaseMode mode)
{
  return mode == PlannerDatabaseMode::ExecuteBestEffort ||
    mode == PlannerDatabaseMode::ExecuteReadOnly;
}

constexpr bool cache_mode_is_training(PlannerDatabaseMode mode)
{
  return mode == PlannerDatabaseMode::TrainingOverwrite ||
    mode == PlannerDatabaseMode::TrainingAppendOnly;
}

// Convert planner database string param to PlannerDatabaseMode enum values.
PlannerDatabaseMode str_to_planner_database_mode(std::string s)
{
  std::string s_lower = str_tolower(s);

  // Using a switch-case here is... inconvenient (needs constexpr hashing or a
  // map), so we don't.
  if (s_lower == "training_overwrite" || s_lower == "trainingoverwrite")
  {
    return PlannerDatabaseMode::TrainingOverwrite;
  }
  else if (s_lower == "training_append_only" || s_lower == "trainingappendonly")
  {
    return PlannerDatabaseMode::TrainingAppendOnly;
  }
  else if (s_lower == "execute_best_effort" || s_lower == "executebesteffort")
  {
    return PlannerDatabaseMode::ExecuteBestEffort;
  }
  else if (s_lower == "execute_read_only" || s_lower == "executereadonly")
  {
    return PlannerDatabaseMode::ExecuteReadOnly;
  }
  else
  {
    return PlannerDatabaseMode::Unset;
  }
}

//==============================================================================
MotionPlannerServer::MotionPlannerServer(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("motion_planner_server", options)
{
  RCLCPP_INFO(this->get_logger(), "Motion Planner Server is running...");

  _internal_node = std::make_shared<rclcpp::Node>(
    "motion_planner_server_internal_node", options);
  _spin_thread = std::thread(
    [node = _internal_node]()
    {
      while (rclcpp::ok())
      {
        rclcpp::spin_some(node);
      }
    });

  _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());

  _use_move_group_interfaces =
    this->declare_parameter("use_move_group_interfaces", true);

  _use_namespace =
    this->declare_parameter("use_namespace", false);

  _goal_tolerance =
    this->declare_parameter("goal_tolerance", 0.005);

  _planning_time =
    this->declare_parameter("planning_time", 0.1);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter planning_time to [%.3f] seconds", _planning_time
  );
  _replan_attempts =
    this->declare_parameter("replan_attempts", 1);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter replan_attempts to [%d]", _replan_attempts
  );

  std::stringstream ss;
  std::vector<std::string> manipulators = {};
  _manipulators = this->declare_parameter("manipulators", manipulators);
  for (const auto& name : _manipulators)
  {
    ss << name << ",";

    std::string group_name = _use_namespace ?
      this->declare_parameter(name + ".group_name", name + ".manipulator") :
      this->declare_parameter("default_group_name", "manipulator");
    _group_names.insert({name, std::move(group_name)});

  }
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter [manipulators] to [%s]", ss.str().c_str()
  );

  int timeout_duration = this->declare_parameter("timeout_duration", 5);
  _timeout_duration = std::chrono::seconds(timeout_duration);

  _execute_trajectory = this->declare_parameter("execute_trajectory", false);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter execute_trajectory to [%s].",
    _execute_trajectory ? "True" : "False"
  );

  _collision_aware_cartesian_path =
    this->declare_parameter("collision_aware_cartesian_path", true);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter collision_aware_cartesian_path to [%s].",
    _collision_aware_cartesian_path ? "True" : "False"
  );

  _cartesian_max_step =
    this->declare_parameter("cartesian_max_step", 0.001);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter cartesian_max_step to [%.4f].",
    _cartesian_max_step
  );

  _cartesian_jump_threshold =
    this->declare_parameter("cartesian_jump_threshold", 0.0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter cartesian_jump_threshold to [%.4f].",
    _cartesian_jump_threshold
  );

  _workspace_min_x =
    this->declare_parameter("workspace_min_x", -1.0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter workspace_min_x to [%.4f].",
    _workspace_min_x
  );

  _workspace_min_y =
    this->declare_parameter("workspace_min_y", -1.0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter workspace_min_y to [%.4f].",
    _workspace_min_y
  );

  _workspace_min_z =
    this->declare_parameter("workspace_min_z", -1.0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter workspace_min_x to [%.4f].",
    _workspace_min_z
  );

  _workspace_max_x =
    this->declare_parameter("workspace_max_x", 1.0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter workspace_max_x to [%.4f].",
    _workspace_max_x
  );

  _workspace_max_y =
    this->declare_parameter("workspace_max_y", 1.0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter workspace_max_y to [%.4f].",
    _workspace_max_y
  );

  _workspace_max_z =
    this->declare_parameter("workspace_max_z", 1.0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter workspace_max_z to [%.4f].",
    _workspace_max_z
  );

  _get_state_wait_seconds =
    this->declare_parameter("get_state_wait_seconds", 0.01);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter get_state_wait_seconds to [%.3f]",
    _get_state_wait_seconds
  );

  // Motion plan cache params
  // unset, training, training_append_only, execute_best_effort, execute_read_only
  _planner_database_mode = this->declare_parameter(
    "planner_database_mode", "unset");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter planner_database_mode to [%s]",
    _planner_database_mode.c_str()
  );
  _cache_mode = str_to_planner_database_mode(_planner_database_mode);

  _cache_db_plugin = this->declare_parameter(
    "cache_db_plugin", "warehouse_ros_sqlite::DatabaseConnection");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter cache_db_plugin to [%s]", _cache_db_plugin.c_str()
  );

  _cache_db_host = this->declare_parameter("cache_db_host", ":memory:");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter cache_db_host to [%s]", _cache_db_host.c_str()
  );

  _cache_db_port = this->declare_parameter<int>("cache_db_port", 0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter cache_db_port to [%d]", _cache_db_port
  );

  _cache_exact_match_tolerance = this->declare_parameter(
    "cache_exact_match_tolerance", 0.0005);  // ~0.028 degrees per joint
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter cache_exact_match_tolerance to [%.2e]",
    _cache_exact_match_tolerance
  );

  _cache_start_match_tolerance = this->declare_parameter(
    "cache_start_match_tolerance", 0.025);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter cache_start_match_tolerance to [%.5f]",
    _cache_start_match_tolerance
  );

  _cache_goal_match_tolerance = this->declare_parameter(
    "cache_goal_match_tolerance", 0.001);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter cache_goal_match_tolerance to [%.5f]",
    _cache_goal_match_tolerance
  );

  if (_cache_mode != PlannerDatabaseMode::Unset)
  {
    // Push warehouse_ros parameters to internal node
    // This must happen BEFORE the MotionPlanCache is created!
    _internal_node->declare_parameter<std::string>(
      "warehouse_plugin", _cache_db_plugin);

    _internal_node->declare_parameter<std::string>(
      "warehouse_host", _cache_db_host);

    _internal_node->declare_parameter<int>(
      "warehouse_port", _cache_db_port);

    _motion_plan_cache =
      std::make_unique<nexus::motion_planner::MotionPlanCache>(_internal_node);
  }
}

//==============================================================================
MotionPlannerServer::~MotionPlannerServer()
{
  if (_spin_thread.joinable())
  {
    _spin_thread.join();
  }
}

//==============================================================================
auto MotionPlannerServer::on_configure(const LifecycleState& /*state*/)
-> CallbackReturn
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  if (_cache_mode != PlannerDatabaseMode::Unset)
  {
    _motion_plan_cache->init(
      _cache_db_host, _cache_db_port, _cache_exact_match_tolerance);
  }

  if (_use_move_group_interfaces)
  {
    auto ok = initialize_move_group_interfaces();
    if (!ok)
    {
      return CallbackReturn::ERROR;
    }
  }
  else
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Motion planner server only works when use_move_group_interfaces is true."
    );
    return CallbackReturn::ERROR;
  }

  _plan_srv = this->create_service<GetMotionPlanService::ServiceType>(
    GetMotionPlanService::service_name(),
    [this](
      const Request request,
      Response response)
    {
      if (this->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        RCLCPP_ERROR(this->get_logger(),
        "Must be in the active state to compute a plan!");
        return;
      }
      plan_with_move_group(*request, response);
    }
  );

  RCLCPP_INFO(this->get_logger(), "Successfully configured.");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
bool MotionPlannerServer::initialize_move_group_interfaces()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Initializing move_group interfaces!"
  );
  for (const auto& name : _manipulators)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Setting up interface with manipulator [%s]", name.c_str()
    );
    auto insertion = _move_group_interfaces.insert({name, nullptr});
    if (insertion.second)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Pass robot description params to internal_node"
      );

      // New manipulator
      std::string group_name = _group_names.at(name);
      MoveGroupInterface::Options options{std::move(group_name)};
      if (_use_namespace)
      {
        options.move_group_namespace = name;
        options.robot_description = name + ".robot_description";
      }
      try
      {
        auto interface = std::make_shared<MoveGroupInterface>(
          _internal_node,
          std::move(options),
          _tf_buffer,
          rclcpp::Duration(_timeout_duration)
        );
        insertion.first->second = interface;
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Unable to initialize move_group interface for manipulator [%s]. "
          "Error: %s", name.c_str(), e.what()
        );
        continue;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Successfully initialized move_group interface for manipulator [%s].",
        name.c_str()
      );
    }
    else
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Got duplicate manipulator [%s]. Ignoring...", name.c_str()
      );
    }
  }
  return true;
}

//==============================================================================
auto MotionPlannerServer::on_activate(const LifecycleState& /*state*/)
-> CallbackReturn
{
  // Begin monitoring states for all interfaces.
  // Also query for a plan so that the planning pipeline gets initialized.
  for (auto it = _move_group_interfaces.begin();
    it != _move_group_interfaces.end(); ++it)
  {
    it->second->startStateMonitor();
    const auto& pose_stamped = it->second->getCurrentPose();
    it->second->setPoseTarget(pose_stamped);
    MoveGroupInterface::Plan plan;
    it->second->plan(plan);
  }
  return CallbackReturn::SUCCESS;
}
//==============================================================================
auto MotionPlannerServer::on_deactivate(const LifecycleState& /*state*/)
-> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}
//==============================================================================
auto MotionPlannerServer::on_cleanup(const LifecycleState& /*state*/)
-> CallbackReturn
{
  _move_group_interfaces.clear();
  _plan_srv.reset();

  return CallbackReturn::SUCCESS;
}
//==============================================================================
auto MotionPlannerServer::on_shutdown(const LifecycleState& /*state*/)
-> CallbackReturn
{
  return CallbackReturn::SUCCESS;
}

//==============================================================================
void MotionPlannerServer::plan_with_move_group(
  const GetMotionPlanService::ServiceType::Request& req, Response res)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Planning with move_group"
  );
  const auto& robot_name = req.robot_name;
  auto interface_it = _move_group_interfaces.find(robot_name);
  if (interface_it == _move_group_interfaces.end())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Manipulator [%s] is not configured.",
      robot_name.c_str()
    );
    return;
  }
  auto& interface = interface_it->second;
  RCLCPP_INFO(
    this->get_logger(),
    "Planning for group [%s] with planning_frame [%s]",
    interface->getName().c_str(), interface->getPlanningFrame().c_str()
  );
  // Set start
  if (req.start_type == req.START_TYPE_CURRENT)
  {
    interface->setStartStateToCurrentState();
  }
  else if (req.start_type == req.START_TYPE_JOINTS)
  {
    const auto& joint_names = interface->getJointNames();
    if (req.start_joints.size() != joint_names.size())
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unable to plan for START_TYPE_JOINTS as provided number of JointConstraints [%ld]"
        "does not match number of joints for the robot [%ld]",
        req.start_joints.size(),
        joint_names.size()
      );
      return;
    }
    // Check if joint names match
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      if (joint_names.at(i) != req.start_joints.at(i).joint_name)
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Joint name [%s] in start_joints at index [%ld] does not match"
          "joint_name [%s] of robot configured in move_group at the same index",
          req.start_joints.at(i).joint_name.c_str(),
          i,
          joint_names.at(i).c_str()
        );
        return;
      }
    }

    // We get the current state of the robot to overwrite joint values before setting
    // it as the new start.
    auto state = interface->getCurrentState(_get_state_wait_seconds);
    if (state == nullptr)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unable to query current state of the robot [%s]. Ignoring request.",
        interface->getName().c_str()
      );
      return;
    }
    // Overwrite current_state with joint values provided.
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      const auto& joint_name = joint_names.at(i);
      state->setJointPositions(joint_name, {req.start_joints.at(i).position});
    }
    interface->setStartState(*state);
  }
  else
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Generating plan with start_pose is not yet supported. Please set "
      "start_type to START_TYPE_CURRENT or START_TYPE_JOINTS."
    );
    return;
  }
  // Set workspace bounds
  interface->setWorkspace(
    _workspace_min_x,
    _workspace_min_y,
    _workspace_min_z,
    _workspace_max_x,
    _workspace_max_y,
    _workspace_max_z
  );
  // Set goal
  interface->setGoalTolerance(_goal_tolerance);

  if (req.goal_type == req.GOAL_TYPE_POSE)
  {
    RCLCPP_INFO(this->get_logger(), "Setting goal to target pose with "
      "end-effector link [%s].", interface->getEndEffectorLink().c_str());
    interface->setPoseTarget(req.goal_pose);
    // TODO(YV): In order to use CHOMP optimizer, the planning request must contain
    // joint state goals. Hence we use setApproximateJointValueTarget() instead.
    // interface->setApproximateJointValueTarget(req.goal_pose, interface->getEndEffectorLink());
    if (req.cartesian)
    {
      // Set the reference frame for poses specified without a reference frame.
      interface->setPoseReferenceFrame(req.goal_pose.header.frame_id);
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Setting goal to target joint values.");
    for (const auto& j : req.goal_joints)
    {
      interface->setJointValueTarget(j.joint_name, j.position);
    }

  }
  // Set other planning parameters
  interface->setPlanningTime(_planning_time);
  interface->setReplanAttempts(_replan_attempts);
  // Get the clamped scaling factors
  const double vel_scale =
    (req.max_velocity_scaling_factor <= 0 ||
    req.max_velocity_scaling_factor >
    1.0) ? 1.0 : req.max_velocity_scaling_factor;
  const double acc_scale =
    (req.max_acceleration_scaling_factor <= 0 ||
    req.max_acceleration_scaling_factor >
    1.0) ? 1.0 : req.max_acceleration_scaling_factor;
  interface->setMaxVelocityScalingFactor(vel_scale);
  interface->setMaxAccelerationScalingFactor(acc_scale);
  moveit::core::MoveItErrorCode error;
  moveit_msgs::msg::MotionPlanRequest plan_req_msg;  // For caching
  if (req.cartesian)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(req.goal_pose.pose);
    // TODO(YV): For now we use simple cartesian interpolation. OMPL supports
    // constrained planning so we can consider adding orientation or line
    // constraints to the end-effector link instead and call plan().

    moveit_msgs::msg::RobotTrajectory cartesian_plan;
    double fraction;
    bool cartesian_plan_is_from_cache = false;
    auto cartesian_plan_req_msg =
      _motion_plan_cache->construct_get_cartesian_plan_request(
      *interface, waypoints, _cartesian_max_step, _cartesian_jump_threshold,
      _collision_aware_cartesian_path);

    // Fetch if in execute mode.
    if (cache_mode_is_execute(_cache_mode))
    {
      auto fetch_start = this->now();
      auto fetched_cartesian_plan =
        _motion_plan_cache->fetch_best_matching_cartesian_plan(
        *interface, robot_name, cartesian_plan_req_msg,
        /* min_fraction */ 1.0,
        _cache_start_match_tolerance, _cache_goal_match_tolerance);
      auto fetch_end = this->now();
      // Set plan if a cached cartesian plan was fetched.
      if (fetched_cartesian_plan)
      {
        fraction = fetched_cartesian_plan->lookupDouble("fraction");
        cartesian_plan_is_from_cache = true;
        cartesian_plan = *fetched_cartesian_plan;
        res->result.error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        res->result.planning_time = (fetch_end - fetch_start).seconds();
        RCLCPP_INFO(
          this->get_logger(),
          "Cache fetch took %es, planning time of fetched plan was: %es",
          (fetch_end - fetch_start).seconds(),
          fetched_cartesian_plan->lookupDouble("planning_time_s"));
      }
      // Fail if ReadOnly mode and no cached cartesian plan was fetched.
      else if (_cache_mode == PlannerDatabaseMode::ExecuteReadOnly)
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Cache mode was ExecuteReadOnly, and could not find "
          "cached cartesian plan for cartesian plan request: \n\n%s",
          moveit_msgs::srv::to_yaml(cartesian_plan_req_msg).c_str());
        res->result.error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return;
      }
    }

    // Plan if needed.
    // This is if we didn't fetch a cartesian plan from the cache.
    // (In training or unset mode we never attempt to fetch, so it will always
    // plan.)
    if (!cartesian_plan_is_from_cache)
    {
      auto cartesian_plan_start = this->now();
      fraction = interface->computeCartesianPath(
        waypoints,
        _cartesian_max_step,
        _cartesian_jump_threshold,
        cartesian_plan,
        _collision_aware_cartesian_path
      );
      auto cartesian_plan_end = this->now();

      RCLCPP_INFO(
        this->get_logger(),
        "Cartesian interpolation returned a fraction of [%.2f]", fraction
      );
      if (fraction <= 0.0)
      {
        res->result.error_code.val = -1;
      }
      else
      {
        res->result.error_code.val = 1;
      }

      res->result.planning_time =
        (cartesian_plan_end - cartesian_plan_start).seconds();

      RCLCPP_INFO(
        this->get_logger(),
        "Plan status: %d, planning time: %es",
        res->result.error_code.val, res->result.planning_time);
    }
    else
    {
      if (fraction <= 0)
      {
        res->result.error_code.val = -1;
      }
      else
      {
        res->result.error_code.val = 1;
      }
    }

    // Do NOT move this. We use the cartesian_plan_req_msg later.
    res->result.trajectory_start = cartesian_plan_req_msg.start_state;
    res->result.trajectory = std::move(cartesian_plan);

    if (res->result.error_code.val !=
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Cartesian planning did not succeed: %d", res->result.error_code.val);
      return;
    }

    // Put plan if in training mode.
    // Make sure we check if the plan we have was fetched (so we don't have
    // duplicate caches.)
    if (cache_mode_is_training(_cache_mode) && !cartesian_plan_is_from_cache)
    {
      if (!_motion_plan_cache->put_cartesian_plan(
          *interface, robot_name,
          std::move(cartesian_plan_req_msg), std::move(res->result.trajectory),
          rclcpp::Duration(
            res->result.trajectory.joint_trajectory.points.back()
            .time_from_start
          ).seconds(),
          res->result.planning_time, fraction,
          _cache_mode == PlannerDatabaseMode::TrainingOverwrite))
      {
        RCLCPP_WARN(
          this->get_logger(), "Did not put cartesian plan into cache.");
      }
    }
  }
  else
  {
    MoveGroupInterface::Plan plan;
    bool plan_is_from_cache = false;
    interface->constructMotionPlanRequest(plan_req_msg);

    // Fetch if in execute mode.
    if (cache_mode_is_execute(_cache_mode))
    {
      auto fetch_start = this->now();
      auto fetched_plan = _motion_plan_cache->fetch_best_matching_plan(
        *interface, robot_name, plan_req_msg,
        _cache_start_match_tolerance, _cache_goal_match_tolerance);
      auto fetch_end = this->now();
      // Set plan if a cached plan was fetched.
      if (fetched_plan)
      {
        plan_is_from_cache = true;
        plan.start_state = plan_req_msg.start_state;
        plan.trajectory = *fetched_plan;
        plan.planning_time = (fetch_end - fetch_start).seconds();
        res->result.error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        RCLCPP_INFO(
          this->get_logger(),
          "Cache fetch took %es, planning time of fetched plan was: %es",
          (fetch_end - fetch_start).seconds(),
          fetched_plan->lookupDouble("planning_time_s"));
      }
      // Fail if ReadOnly mode and no cached plan was fetched.
      else if (_cache_mode == PlannerDatabaseMode::ExecuteReadOnly)
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "Cache mode was ExecuteReadOnly, and could not find "
          "cached plan for plan request: \n\n%s",
          moveit_msgs::msg::to_yaml(plan_req_msg).c_str());
        res->result.error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return;
      }
    }

    // Plan if needed.
    // This is if we didn't fetch a plan from the cache.
    // (In training or unset mode we never attempt to fetch, so it will always
    // plan.)
    if (!plan_is_from_cache)
    {
      res->result.error_code = interface->plan(plan);
      RCLCPP_INFO(
        this->get_logger(),
        "Plan status: %d, planning time: %es",
        res->result.error_code.val, plan.planning_time);
    }

    res->result.trajectory_start = std::move(plan.start_state);
    res->result.trajectory = std::move(plan.trajectory);
    res->result.planning_time = std::move(plan.planning_time);

    if (res->result.error_code.val !=
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Planning did not succeed: %d", res->result.error_code.val);
      return;
    }

    // Put plan if in training mode.
    // Make sure we check if the plan we have was fetched (so we don't have
    // duplicate caches.)
    if (cache_mode_is_training(_cache_mode) && !plan_is_from_cache)
    {
      if (!_motion_plan_cache->put_plan(
          *interface, robot_name,
          std::move(plan_req_msg), std::move(res->result.trajectory),
          rclcpp::Duration(
            res->result.trajectory.joint_trajectory.points.back()
            .time_from_start
          ).seconds(),
          res->result.planning_time,
          _cache_mode == PlannerDatabaseMode::TrainingOverwrite))
      {
        RCLCPP_WARN(this->get_logger(), "Did not put plan into cache.");
      }
    }
  }
  if (_execute_trajectory)
  {
    RCLCPP_INFO(this->get_logger(), "Executing trajectory!");
    // This is a blocking call.
    interface->execute(res->result.trajectory);
  }
  return;
}

}  // namespace planning_interface
}  // namespace moveit