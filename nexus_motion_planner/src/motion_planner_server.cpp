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
  const GetMotionPlanService::ServiceType::Request& req,
  Response res)
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
  if (req.cartesian)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(req.goal_pose.pose);
    // TODO(YV): For now we use simple cartesian interpolation. OMPL supports
    // constrained planning so we can consider adding orientation or line
    // constraints to the end-effector link instead and call plan().
    const double error = interface->computeCartesianPath(
      waypoints,
      _cartesian_max_step,
      _cartesian_jump_threshold,
      res->result.trajectory,
      _collision_aware_cartesian_path
    );
    RCLCPP_INFO(
      this->get_logger(),
      "Cartesian interpolation returned a fraction of [%.2f]", error
    );
    if (error <= 0.0)
    {
      res->result.error_code.val = -1;
    }
    else
    {
      res->result.error_code.val = 1;

    }
  }
  else
  {
    MoveGroupInterface::Plan plan;
    res->result.error_code = interface->plan(plan);
    res->result.trajectory_start = std::move(plan.start_state);
    res->result.trajectory = std::move(plan.trajectory);
    res->result.planning_time = std::move(plan.planning_time);
  }
  if (_execute_trajectory)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Executing trajectory!");
    // This is a blocking call.
    interface->execute(res->result.trajectory);
  }
  return;
}
