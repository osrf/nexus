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
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return {}


def launch_setup(context, *args, **kwargs):
    planner_config_package = LaunchConfiguration("planner_config_package")
    planner_config_file = LaunchConfiguration("planner_config_file")
    publish_item = LaunchConfiguration("publish_item")
    support_package = LaunchConfiguration("support_package")
    robot_xacro_file = LaunchConfiguration("robot_xacro_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    ur_type = LaunchConfiguration("ur_type")
    ur_name = LaunchConfiguration("ur_name")

    planner_server_params = PathJoinSubstitution(
        [
            FindPackageShare(planner_config_package),
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
            " use_mock_hardware:=true name:=", ur_name.perform(context), " ur_type:=", ur_type.perform(context),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " name:=", ur_name.perform(context),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    kinematics_yaml = {"robot_description_kinematics": load_yaml(
        moveit_config_package.perform(context), "config/kinematics.yaml")}

    joint_limits_yaml = {
        "robot_description_planning": load_yaml(moveit_config_package.perform(context), "config/joint_limits.yaml")
    }

    chomp_config = load_yaml(
        moveit_config_package.perform(context), "config/chomp_planning.yaml")

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        moveit_config_package.perform(context), "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        moveit_config_package.perform(context), "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml["moveit_simple_controller_manager"],
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
            joint_limits_yaml,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            chomp_config,
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

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory(moveit_config_package.perform(context)), "config"
    )
    rviz_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    # Static TF
    static_tf_node_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="motion_planner_static_transform_publisher_world_t_base_link",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Static TF representing item pickup pose
    static_tf_node_2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="motion_planner_static_transform_publisher_base_link_t_item",
        output="log",
        arguments=["0.6", "0.2", "0.5", "3.14", "0.0", "3.14", "base_link", "item"],
        condition=IfCondition(publish_item),
    )

    nodes = [
        move_group_node,
        motion_planner,
        rviz_node,
        static_tf_node_1,
        static_tf_node_2
    ]

    return nodes


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'planner_config_file',
            default_value='planner_params.yaml',
            description="Name of planner server configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "planner_config_package",
            default_value="nexus_motion_planner",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "support_package",
            default_value="ur_robot_driver",
            description="Name of the support package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_xacro_file",
            default_value="ur.urdf.xacro",
            description="Xacro describing the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="Name of the support package",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="Name of the SRDF file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Launch in headless mode (no gui)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_item",
            default_value="true",
            description="Publish the static transform for item",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_name",
            default_value="ur5e",
            description="Name of the UR robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="UR robot type (i.e. ur5e, ur10, etc.) to be used for xacro generation",
        )
    )
    return LaunchDescription(
      declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

def main():
    generate_launch_description()


if __name__ == '__main__':
    main()
