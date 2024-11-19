# Copyright (C) 2022 Johnson & Johnson
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
import sys
import time
import unittest
import xacro

import ament_index_python
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution

import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_testing.asserts import assertSequentialStdout

import pytest


def get_test_process_action(cmd, args=[]):
    return ExecuteProcess(
        cmd=cmd,
        name=' '.join(cmd),
        # This is necessary to get unbuffered output from the process under test
        additional_env={'PYTHONUNBUFFERED': '1'},
        output='screen'
    )


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_description_path = get_package_share_directory("abb_irb910sc_support")
    robot_description_config = xacro.process_file(
        os.path.join(
            robot_description_path,
            "urdf",
            "irb910sc_3_45.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    start_rviz = LaunchConfiguration("start_rviz")
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(robot_description_path, "rviz", "urdf_description.rviz"),
        ],
        output="log",
        condition=IfCondition(start_rviz),
    )

    robot_controller_path = get_package_share_directory("nexus_robot_controller")
    robot_controller_test_params = PathJoinSubstitution(
        [robot_controller_path, "test", "abb_irb910sc_test_params.yaml"]
    )
    controller_params = PathJoinSubstitution(
        [robot_controller_path, "config", "test_abb_irb910sc_controllers.yaml"]
    )
    remappings = []

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        rviz,

        Node(
            package='nexus_robot_controller',
            executable='robot_controller_server',
            output='screen',
            parameters=[robot_controller_test_params, controller_params, robot_description],
            remappings=remappings
        ),

        launch_testing.actions.ReadyToTest(),
    ])


class TestLifeCycle(unittest.TestCase):
    def test_robot_controller_server(self, launch_service, proc_info, proc_output):
        echo_joint_states = get_test_process_action(
            ['ros2', 'topic', 'echo', '/joint_states']
        )

        os.system('ros2 lifecycle set /robot_controller_server configure')
        proc_output.assertWaitFor('Configuring', timeout=10, stream='stderr')
        proc_output.assertWaitFor("Successful 'configure'", timeout=10, stream='stderr')

        os.system('ros2 lifecycle set /robot_controller_server activate')
        proc_output.assertWaitFor('Activating', timeout=10, stream='stderr')
        proc_output.assertWaitFor('Switched on Controllers', timeout=10, stream='stderr')

        # Test that commands were actually sent
        with launch_testing.tools.launch_process(
            launch_service, echo_joint_states, proc_info, proc_output
        ):
            proc_info.assertWaitForStartup(process=echo_joint_states, timeout=5)

            os.system('ros2 topic pub -1 '
                      'forward_command_controller_position/commands std_msgs/msg/Float64MultiArray '
                      '"{data: [-5.0, -5.0, -5.0, -5.0]}"')

            proc_output.assertWaitFor('- -5.', timeout=10, stream='stdout')

            os.system('ros2 lifecycle set /robot_controller_server deactivate')
            proc_output.assertWaitFor('Deactivating', timeout=10, stream='stderr')
            proc_output.assertWaitFor('Switched off Controllers', timeout=10, stream='stderr')

            os.system('ros2 lifecycle set /robot_controller_server activate')
            proc_output.assertWaitFor('Activating', timeout=10, stream='stderr')
            proc_output.assertWaitFor('Switched on Controllers', timeout=10, stream='stderr')

            os.system('ros2 topic pub -1 '
                      'forward_command_controller_position/commands std_msgs/msg/Float64MultiArray '
                      '"{data: [5.0, 5.0, 5.0, 5.0]}"')

            proc_output.assertWaitFor('- 5.', timeout=10, stream='stdout')

            os.system('ros2 topic pub -1 '
                      'forward_command_controller_position/commands std_msgs/msg/Float64MultiArray '
                      '"{data: [0.0, 0.0, 0.0, 0.0]}"')

        os.system('ros2 lifecycle set /robot_controller_server deactivate')
        proc_output.assertWaitFor('Deactivating', timeout=10, stream='stderr')
        proc_output.assertWaitFor('Switched off Controllers', timeout=10, stream='stderr')

        os.system('ros2 lifecycle set /robot_controller_server cleanup')
        proc_output.assertWaitFor('Cleaning Up', timeout=10, stream='stderr')
        proc_output.assertWaitFor('Unloading Controllers', timeout=10, stream='stderr')
        proc_output.assertWaitFor('CM Thread Ending', timeout=10, stream='stderr')
        proc_output.assertWaitFor('CM Thread Ending', timeout=10, stream='stderr')
        proc_output.assertWaitFor('Thread Terminated', timeout=10, stream='stderr')
