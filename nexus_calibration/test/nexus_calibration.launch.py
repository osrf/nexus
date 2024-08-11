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
from launch_ros.event_handlers import OnStateTransition
import launch
from launch.actions import (
    GroupAction,
    OpaqueFunction,
    ExecuteProcess,
)
import lifecycle_msgs

def activate_node(target_node: LifecycleNode, depend_node: LifecycleNode = None):

    configure_event = None

    if depend_node is not None:
        configure_event = launch.actions.RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=depend_node,
                goal_state="active",
                entities=[
                    launch.actions.EmitEvent(
                        event=launch_ros.events.lifecycle.ChangeState(
                            lifecycle_node_matcher=launch.events.matches_action(
                                target_node
                            ),
                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                        )
                    )
                ],
            )
        )
    else:
        # Make the talker node take the 'configure' transition.
        configure_event = launch.actions.EmitEvent(
            event=launch_ros.events.lifecycle.ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(target_node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        )

    inactive_state_handler = launch.actions.RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=target_node,
            goal_state="inactive",
            entities=[
                launch.actions.LogInfo(
                    msg=f"node {target_node.node_executable} reached the 'inactive' state."
                ),
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            target_node
                        ),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    active_state_handler = launch.actions.RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=target_node,
            goal_state="active",
            entities=[
                launch.actions.LogInfo(
                    msg=f"node {target_node.node_executable} reached the 'active' state"
                ),
            ],
        )
    )

    return GroupAction(
        [
            configure_event,
            inactive_state_handler,
            active_state_handler,
        ]
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
            parameters=[config_path]
        )

    return [
        simulation,
        calibration_node,
        activate_node(calibration_node)
    ]

def generate_launch_description():
    return launch.LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
