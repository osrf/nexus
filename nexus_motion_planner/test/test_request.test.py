# Copyright 2022 Johnson & Johnson
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import pytest
import unittest
import xacro
import yaml

import launch
from launch.actions import (
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction
)
from launch.event_handlers import (
    OnExecutionComplete,
    OnProcessExit,
    OnProcessStart,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import launch_testing

from ament_index_python.packages import get_package_share_directory

def set_lifecycle(server_name, transition):
    """Executes service to transition lifecycle states

    Parameters
    ----------
    server_name : str
        Name of lifecycle node
    transition : str
        lifecycle state to transition to

    Returns
    -------
    launch.actions.ExecuteProcess
        An ExecuteProcess action that executes the lifecycle transition
    """
    return ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' lifecycle set ',
            server_name + ' ',
            transition
        ]],
        name=server_name + '_set_lifecycle_to_' + transition,
        shell=True,
        output='screen',
    )

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

@pytest.mark.launch_test
def generate_test_description():
    """This test sequence tests if the Motion Planner Server is able to generate a
    valid plan from the GetMotionPlan interface.

    Returns
    -------
    launch.LaunchDescription
        Nodes to be launched for testing
    """
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '0')

    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    robot_xacro_file = "irb1300_10_115.xacro"
    moveit_config_file = "abb_irb1300_10_115.srdf.xacro"
    planner_config_file = "planner_params.yaml"

    moveit_config_package = "abb_irb1300_10_115_moveit_config"
    support_package = "abb_irb1300_support"
    runtime_config_package = "nexus_motion_planner"

    planner_server_params = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            planner_config_file,
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(support_package), "urdf", robot_xacro_file]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", moveit_config_file]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    kinematics_yaml = load_yaml(
        moveit_config_package, "config/kinematics.yaml")

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        moveit_config_package, "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        moveit_config_package, "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        # MoveIt does not handle controller switching automatically
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # Start the nexus_motion_planner server
    motion_planner = Node(
        package = "nexus_motion_planner",
        executable = "motion_planner_server",
        name = "motion_planner_server",
        output="both",
        parameters=[
          robot_description,
          robot_description_semantic,
          kinematics_yaml,
          planner_server_params,
        ],
    )

    # Static TF
    static_tf_node_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_t_base_link",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Static TF representing item pickup pose
    static_tf_node_2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_base_link_t_item",
        output="log",
        arguments=["0.3", "0.2", "0.1", "0.0", "0.0", "0.0", "base_link", "item"],
    )

    # Send get motion plan requests with specified start and target
    # pose values
    get_plan_irb1300_poses = Node(
        package='nexus_motion_planner',
        executable='test_request',
        arguments=[
            "-name", "abb_irb1300",
            "-frame_id", "item",
            "-goal_type", "0",
            "-t", "0.794981,0.000000,0.063564,0.000000,0.707107,0.000000,0.707107",
        ],
        output='both',
    )

    # The following set of lifecycle state transitions tests the lifecycle
    # capabilities of the motion planner server

    lifecycle_set_configure = set_lifecycle('/motion_planner_server', 'configure')
    lifecycle_set_activate = set_lifecycle('/motion_planner_server', 'activate')
    lifecycle_set_deactivate = set_lifecycle('/motion_planner_server', 'deactivate')
    lifecycle_set_cleanup = set_lifecycle('/motion_planner_server', 'cleanup')
    lifecycle_set_configure2 = set_lifecycle('/motion_planner_server', 'configure')
    lifecycle_set_activate2 = set_lifecycle('/motion_planner_server', 'activate')

    trigger_lifecycle_1 = RegisterEventHandler(
      OnProcessStart(
        target_action=motion_planner,
        on_start=[
          TimerAction(
            period=5.0,
            actions=[lifecycle_set_configure]
          )
        ]
      )
    )

    trigger_lifecycle_2 = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_set_configure,
            on_exit=[lifecycle_set_activate]))

    trigger_lifecycle_3 = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_set_activate,
            on_exit=[lifecycle_set_deactivate]))

    trigger_lifecycle_4 = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_set_deactivate,
            on_exit=[lifecycle_set_cleanup]))

    trigger_lifecycle_5 = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_set_cleanup,
            on_exit=[lifecycle_set_configure2]))

    trigger_lifecycle_6 = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_set_configure2,
            on_exit=[lifecycle_set_activate2]))

    trigger_get_plan_irb1300_poses = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_set_activate2,
            on_exit=[
                LogInfo(msg='Sending motion planning task with end-effector poses'),
                TimerAction(
                    period=5.0,
                    actions=[get_plan_irb1300_poses]
                )
            ]
        )
    )

    node_mapping = {
        'motion_planner': motion_planner,
        'get_plan_irb1300_poses': get_plan_irb1300_poses,
    }

    return launch.LaunchDescription([
        motion_planner,
        move_group_node,
        static_tf_node_1,
        static_tf_node_2,
        trigger_lifecycle_1,
        trigger_lifecycle_2,
        trigger_lifecycle_3,
        trigger_lifecycle_4,
        trigger_lifecycle_5,
        trigger_lifecycle_6,
        trigger_get_plan_irb1300_poses,
        launch_testing.actions.ReadyToTest()
    ]), node_mapping

class TestGetMotionPlanService(unittest.TestCase):
    def test_read_task_client_stderr(
        self,
        proc_output,
        get_plan_irb1300_poses):
        """Check if task client has sent service request and received successful response"""

        get_plan_poses_str = [
            "Target pose specified",
            "Moveit Error Code = 1",
        ]

        for expected_output in get_plan_poses_str:
          proc_output.assertWaitFor(
              expected_output,
              process=get_plan_irb1300_poses, timeout=40, stream='stderr')

    def test_read_server_stderr(
        self,
        proc_output,
        motion_planner):
        """Check if server has successfully computed motion plan"""

        motion_planner_server_str = [
            "Planning request accepted",
            "Planning request complete!",
        ]

        for expected_output in motion_planner_server_str:
            proc_output.assertWaitFor(
                expected_output,
                process=motion_planner, timeout=60, stream='stderr')

class TestWait(unittest.TestCase):
    def test_wait_for_get_plan_clients(self, proc_info, get_plan_irb1300_poses):
        """Wait until process ends"""
        proc_info.assertWaitForShutdown(process=get_plan_irb1300_poses, timeout=40)

@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_read_task_client_stderr(
        self,
        proc_info,
        get_plan_irb1300_poses):
      """Check if the processes exited normally."""

      launch_testing.asserts.assertExitCodes(proc_info, [0],
                                             process=get_plan_irb1300_poses)
