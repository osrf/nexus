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
from ament_index_python.packages import get_package_share_directory

import launch_ros
from launch_ros.actions import Node, LifecycleNode
from launch_ros.descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs

import launch
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


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
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_rmf_transporter = LaunchConfiguration("use_rmf_transporter")
    use_zenoh_bridge = LaunchConfiguration("use_zenoh_bridge")
    zenoh_config_package = LaunchConfiguration("zenoh_config_package")
    zenoh_config_filename = LaunchConfiguration("zenoh_config_filename")
    transporter_plugin = LaunchConfiguration("transporter_plugin")
    activate_system_orchestrator = LaunchConfiguration("activate_system_orchestrator")
    headless = LaunchConfiguration("headless")
    main_bt_filename = LaunchConfiguration("main_bt_filename")
    remap_task_types = LaunchConfiguration("remap_task_types")
    nexus_rviz_config = LaunchConfiguration("nexus_rviz_config")
    system_orchestrator_bt_dir = LaunchConfiguration("system_orchestrator_bt_dir")
    max_jobs = LaunchConfiguration("max_jobs")

    system_orchestrator_node = LifecycleNode(
        name="system_orchestrator",
        namespace="",
        package="nexus_system_orchestrator",
        executable="nexus_system_orchestrator",
        parameters=[
            {
                "bt_path": system_orchestrator_bt_dir,
                "remap_task_types": ParameterValue(remap_task_types, value_type=str),
                "main_bt_filename": main_bt_filename,
                "max_jobs": max_jobs,
            }
        ],
    )

    rmf_transporter = GroupAction(
        actions=[
            IncludeLaunchDescription(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nexus_demos"),
                            "launch",
                            "rmf_transporter.launch.xml",
                        ]
                    )
                ],
                launch_arguments={
                    "use_simulator": use_fake_hardware,
                    "headless": headless,
                }.items(),
                condition=IfCondition(use_rmf_transporter),
            )
        ],
    )

    transporter_node = LifecycleNode(
        namespace="",
        package="nexus_transporter",
        executable="nexus_transporter_node",
        name="transporter_node",
        parameters=[
            {"transporter_plugin": transporter_plugin},
            {"destinations": ["loading", "workcell_1", "workcell_2", "unloading",]},
            {"x_increment": 1.0},
            {"speed": 1.0},
            {"unloading_station": "unloading"},
        ],
        condition=UnlessCondition(use_rmf_transporter),
    )

    activate_transporter_node = GroupAction(
        [
            activate_node(transporter_node),
        ],
        condition=UnlessCondition(use_rmf_transporter),
    )

    mock_emergency_alarm_node = LifecycleNode(
        name="mock_emergency_alarm",
        namespace="",
        package="nexus_demos",
        executable="mock_emergency_alarm",
    )

    nexus_panel = Node(
        package="rviz2",
        executable="rviz2",
        name="nexus_panel",
        arguments=["-d", nexus_rviz_config.perform(context)],
        condition=UnlessCondition(headless),
    )

    zenoh_bridge = GroupAction(
        [
            IncludeLaunchDescription(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nexus_demos"),
                            "launch",
                            "zenoh_bridge.launch.py",
                        ]
                    )
                ],
                launch_arguments={
                    "zenoh_config_package": zenoh_config_package,
                    "zenoh_config_filename": zenoh_config_filename,
                    "ros_domain_id": ros_domain_id.perform(context),
                }.items(),
            )
        ],
        condition=IfCondition(use_zenoh_bridge),
    )

    activate_system_orchestrator = GroupAction(
        [
            activate_node(system_orchestrator_node),
        ],
        condition=IfCondition(activate_system_orchestrator),
    )

    return [
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
        system_orchestrator_node,
        rmf_transporter,
        transporter_node,
        mock_emergency_alarm_node,
        nexus_panel,
        zenoh_bridge,
        activate_system_orchestrator,
        activate_transporter_node,
        activate_node(mock_emergency_alarm_node),
    ]


def generate_launch_description():

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value="0",
                description="ROS_DOMAIN_ID environment variable",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Set True if running with real hardware.",
            ),
            DeclareLaunchArgument(
                "use_rmf_transporter",
                default_value="false",
                description="Set true to rely on an Open-RMF managed fleet to transport material\
                between workcells.",
            ),
            DeclareLaunchArgument(
                "use_zenoh_bridge",
                default_value="true",
                description="Set true to launch the Zenoh DDS Bridge",
            ),
            DeclareLaunchArgument(
                name="zenoh_config_package",
                default_value="nexus_demos",
                description="Package containing Zenoh DDS bridge configurations",
            ),
            DeclareLaunchArgument(
                name="zenoh_config_filename",
                default_value="config/zenoh/system_orchestrator.json5",
                description="Zenoh DDS bridge configuration filepath",
            ),
            DeclareLaunchArgument(
                "transporter_plugin",
                default_value="nexus_transporter::MockTransporter",
                description="The transporter plugin to load by the TransporterNode",
            ),
            DeclareLaunchArgument(
                "activate_system_orchestrator",
                default_value="false",
                description="Whether to activate system orchestrator upon launch",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Launch in headless mode (no gui)",
            ),
            DeclareLaunchArgument(
                "main_bt_filename",
                default_value="main.xml",
                description="File name of the main system orchestrator behavior tree",
            ),
            DeclareLaunchArgument(
                "remap_task_types",
                default_value="\"pick_and_place: [place_on_conveyor, pick_from_conveyor]\"",
                description="File name of the main system orchestrator behavior tree",
            ),
            DeclareLaunchArgument(
                "nexus_rviz_config",
                default_value=os.path.join(get_package_share_directory("nexus_demos"), "rviz", "nexus_panel.rviz"),
                description="Absolute path to an RViZ config file.",
            ),
            DeclareLaunchArgument(
                "system_orchestrator_bt_dir",
                default_value=os.path.join(get_package_share_directory("nexus_demos"), "config", "system_bts"),
                description="Absolute path directory containing BTs for the system orchestrator.",
            ),
            DeclareLaunchArgument(
                "max_jobs",
                default_value="2",
                description="Maximum number of jobs the system orchestrator can process in parallel.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
