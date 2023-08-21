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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('nexus_robot_controller')

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("rrbot_description"), "urdf", "rrbot.urdf.xacro",]
        ),
    ])
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [pkg_dir, "config", "rrbot_demo", "rrbot_controllers.yaml",]
    )

    configured_params = PathJoinSubstitution(
        [pkg_dir, "config", "rrbot_demo", "demo_params.yaml",]
    )

    remappings = []

    rrbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'include', 'rrbot_base.launch.py')
        ),
        launch_arguments={'runtime_config_package': 'nexus_robot_controller'}.items()
    )

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'params', 'rrbot_demo', 'demo_params.yaml'),
            description='Full path to the ROS2 parameters file to use'
        ),

        Node(
            package='nexus_robot_controller',
            executable='robot_controller_server',
            output='screen',
            parameters=[configured_params, robot_controllers, robot_description],
            remappings=remappings
        ),

        rrbot_launch
    ])
