# Copyright 2025 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import (
    ExecutableInPackage,
    FindPackageShare,
)


def launch_setup(context, *args, **kwargs):
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    log_level = LaunchConfiguration("log_level")
    zenoh_config_package = LaunchConfiguration("zenoh_config_package")
    zenoh_router_config_filename = LaunchConfiguration("zenoh_router_config_filename")

    cmd = [
        ExecutableInPackage(
            executable="rmw_zenohd",
            package="rmw_zenoh_cpp",
        )
    ]

    zenoh_router_exec = ExecuteProcess(
        cmd=cmd,
        shell=True,
        additional_env={
            "ROS_DOMAIN_ID": ros_domain_id.perform(context),
            "ZENOH_ROUTER_CONFIG_URI": PathJoinSubstitution([
                FindPackageShare(zenoh_config_package),
                zenoh_router_config_filename
            ]),
            "RUST_LOG": log_level.perform(context)
        },
    )

    return [zenoh_router_exec]

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            name="ros_domain_id",
            default_value="0",
            description="ROS_DOMAIN_ID environment variable",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="log_level",
            default_value="zenoh=debug",
            description="Log level of zenoh router",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="zenoh_config_package",
            default_value="nexus_demos",
            description="Package containing RWM Zenoh configurations",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="zenoh_router_config_filename",
            default_value="config/zenoh/system_orchestrator_router_config.json5",
            description="RMW Zenoh configuration filepath",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
