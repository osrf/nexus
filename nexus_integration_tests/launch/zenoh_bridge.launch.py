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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import (
    ExecutableInPackage,
    FindPackageShare,
)


def launch_setup(context, *args, **kwargs):

    zenoh_config_package = LaunchConfiguration("zenoh_config_package")
    zenoh_config_filename = LaunchConfiguration("zenoh_config_filename")
    dds_localhost_only = LaunchConfiguration("dds_localhost_only")
    log_level = LaunchConfiguration("log_level")
    ros_domain_id = LaunchConfiguration("ros_domain_id")

    cmd = [
        ExecutableInPackage(
            executable="zenoh-bridge-ros2dds",
            package="nexus_zenoh_bridge_dds_vendor",
        ),
        "--config",
        PathJoinSubstitution([
            FindPackageShare(zenoh_config_package),
            zenoh_config_filename
        ]),
        "--domain",
        ros_domain_id,
    ]

    if IfCondition(dds_localhost_only).evaluate(context):
        cmd.append("--dds-localhost-only")

    zenoh_bridge_exec = ExecuteProcess(
        cmd=cmd,
        shell=True,
        additional_env={"RUST_LOG": log_level.perform(context)},
    )

    return [zenoh_bridge_exec]

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
            default_value="error",
            description="Log level of zenoh DDS Bridge",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="dds_localhost_only",
            default_value="false",
            description="If true, the DDS discovery and traffic "
            "will occur only on the localhost interface (127.0.0.1)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="zenoh_config_package",
            default_value="nexus_integration_tests",
            description="Package containing Zenoh DDS bridge configurations",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            name="zenoh_config_filename",
            default_value="config/zenoh/system_orchestrator.json5",
            description="Zenoh DDS bridge configuration filepath",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
