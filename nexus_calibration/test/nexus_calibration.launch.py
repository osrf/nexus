# Copyright (C) 2023 Johnson & Johnson
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
import pathlib
from ament_index_python.packages import get_package_share_directory

import launch_ros
from launch_ros.actions import LifecycleNode
import launch
from launch.actions import (
    OpaqueFunction,
    ExecuteProcess,
)

def launch_setup(context, *args, **kwargs):
    curdir = pathlib.Path(__file__).parent.resolve()
    config_path = os.path.join(curdir, "test_config.yaml")
    world_path = os.path.join(curdir, "test_world.world")

    simulation = ExecuteProcess(
            # -v verbose
            # -s server only
            # -r run on start
            name="Gazebo",
            cmd=['ign gazebo -v 4 -r ', world_path],
            output='screen',
            shell=True,
    )

    calibration_node = LifecycleNode(
            namespace="",
            package="nexus_calibration",
            executable="nexus_calibration_node",
            name="nexus_calibration_node",
            parameters=[config_path],
            autostart=True
        )

    return [
        simulation,
        calibration_node
    ]

def generate_launch_description():
    return launch.LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
