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

from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import launch
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable

from typing import List


def activate_node_service(node_name, ros_domain_id):
    activate_node_proc = ExecuteProcess(
        cmd=[
            'python3',
            [FindPackageShare('nexus_demos'), "/scripts/activate_node.py"],
            node_name,
        ],
        additional_env={'ROS_DOMAIN_ID': ros_domain_id},
    )

    def check_activate_return_code(event, _):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"Activating node '{node_name}' failed!"),
                EmitEvent(event=Shutdown(reason=f"Activating node '{node_name}' failed!"))
            ]
        return []

    return GroupAction(
        [
            LogInfo(msg=f"Activating {node_name}..."),
            activate_node_proc,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=activate_node_proc,
                    on_exit=check_activate_return_code,
                )
            ),
        ],
    )


def launch_setup(context, *args, **kwargs):
    config_path = get_package_share_directory("nexus_demos")

    # Initialize launch configuration
    workcell_id = LaunchConfiguration("workcell_id")
    bt_path = LaunchConfiguration("bt_path")
    task_checker_plugin = LaunchConfiguration("task_checker_plugin")
    max_jobs = LaunchConfiguration("max_jobs")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    headless = LaunchConfiguration("headless")
    controller_config_package = LaunchConfiguration("controller_config_package")
    planner_config_package = LaunchConfiguration("planner_config_package")
    planner_config_file = LaunchConfiguration("planner_config_file")
    support_package = LaunchConfiguration("support_package")
    robot_xacro_file = LaunchConfiguration("robot_xacro_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    controllers_file = LaunchConfiguration("controllers_file")
    tf_publisher_launch_file = LaunchConfiguration("tf_publisher_launch_file")
    sku_detection_params_file = LaunchConfiguration("sku_detection_params_file")
    dispenser_properties = LaunchConfiguration("dispenser_properties")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    use_zenoh_bridge = LaunchConfiguration("use_zenoh_bridge")
    zenoh_config_package = LaunchConfiguration("zenoh_config_package")
    zenoh_config_filename = LaunchConfiguration("zenoh_config_filename")
    remap_task_types = LaunchConfiguration("remap_task_types")
    # todo(Yadunund): There is no good way to get a list of strings via CLI and parse it using
    # LaunchConfiguration. The best way to configure this would be via a YAML params which we
    # pass to this node.
    bt_logging_blocklist: List[str] = ["IsPauseTriggered"]
    remap_task_input_output_stations = LaunchConfiguration("remap_task_input_output_stations")

    workcell_id_str = workcell_id.perform(context)

    mock_dispenser_node = LifecycleNode(
        name=workcell_id.perform(context) + "_mock_dispenser",
        namespace="",
        package="nexus_demos",
        executable="mock_printer_node",
        parameters=[
            {
                "autostart": True,
            }
        ],
    )

    mock_product_detector_node = LifecycleNode(
        name=workcell_id.perform(context) + "_mock_product_detector",
        namespace="",
        package="nexus_demos",
        executable="mock_detector",
        parameters=[
            {"autostart": True},
            {"config_file":
                PathJoinSubstitution([
                    FindPackageShare("nexus_demos"),
                    "config",
                    sku_detection_params_file
                ])
            },
        ],
    )

    mock_gripper_node = LifecycleNode(
        name=workcell_id.perform(context) + "_mock_gripper",
        namespace="",
        package="nexus_demos",
        executable="mock_gripper",
        parameters=[{"autostart": True}],
    )

    workcell_orchestrator_node = LifecycleNode(
        name=workcell_id_str,
        namespace="",
        package="nexus_workcell_orchestrator",
        executable="nexus_workcell_orchestrator",
        parameters=[{
            "capabilities": [
                "nexus::capabilities::DetectionCapability",
                "nexus::capabilities::DispenseItemCapability",
                "nexus::capabilities::ExecuteTrajectoryCapability",
                "nexus::capabilities::GripperCapability",
                "nexus::capabilities::PlanMotionCapability",
            ],
            "bt_path": bt_path,
            "task_checker_plugin": task_checker_plugin,
            "max_jobs": max_jobs,
            "hardware_nodes": [
                workcell_id.perform(context) + "_mock_dispenser",
                workcell_id.perform(context) + "_mock_gripper",
                workcell_id.perform(context) + "_mock_product_detector",
            ],
            "dispensers": [
                workcell_id.perform(context) + "_mock_dispenser"
            ],
            "dispenser_properties": [
                str(dispenser_properties.perform(context))
            ],
            "grippers": [
                workcell_id.perform(context) + "_mock_gripper"
            ],
            "detectors": [
                workcell_id.perform(context) + "_mock_product_detector"
            ],
            "robot_name": "abb_irb1300",
            "gripper_max_effort": 0.0,
            "remap_task_types": remap_task_types,
            "bt_logging_blocklist": bt_logging_blocklist,
            "remap_task_input_output_stations": ParameterValue(remap_task_input_output_stations, value_type=str),
        }],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return [
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        mock_dispenser_node,
        mock_product_detector_node,
        mock_gripper_node,
        workcell_orchestrator_node,
        GroupAction(
            [
                IncludeLaunchDescription(
                    [
                        PathJoinSubstitution([
                            FindPackageShare("abb_bringup"),
                            'launch',
                            'abb_control.launch.py',
                        ])
                    ],
                    launch_arguments=[
                        ("runtime_config_package", controller_config_package),
                        ("controllers_file", controllers_file),
                        ("description_package", support_package),
                        ("description_file", robot_xacro_file),
                        ("launch_rviz", "false"),
                        ("moveit_config_package", moveit_config_package),
                        ("use_fake_hardware", use_fake_hardware),
                        ("rws_ip", robot_ip),
                    ],
                )
            ]
        ),
        GroupAction(
            [
                IncludeLaunchDescription(
                    [
                        PathJoinSubstitution([
                            FindPackageShare("nexus_motion_planner"),
                            'launch',
                            'demo_planner_server.launch.py',
                        ])
                    ],
                    launch_arguments=[
                        ("planner_config_package", planner_config_package),
                        ("planner_config_file", planner_config_file),
                        ("publish_item", "false"),
                        ("support_package", support_package),
                        ("robot_xacro_file", robot_xacro_file),
                        ("moveit_config_package", moveit_config_package),
                        ("moveit_config_file", moveit_config_file),
                        ("headless", headless),
                    ]
                ),
            ]
        ),
        GroupAction(
            [
                IncludeLaunchDescription(
                    [
                        PathJoinSubstitution([
                            FindPackageShare("nexus_demos"),
                            'launch',
                            tf_publisher_launch_file,
                        ])
                    ]
                )
            ]
        ),
        GroupAction(
            [
                IncludeLaunchDescription(
                    [
                        PathJoinSubstitution([
                            FindPackageShare('nexus_demos'),
                            'launch',
                            'zenoh_bridge.launch.py'
                        ])
                    ],
                    launch_arguments={
                        'zenoh_config_package': zenoh_config_package,
                        'zenoh_config_filename': zenoh_config_filename,
                        'ros_domain_id': ros_domain_id.perform(context),
                    }.items()
                )
            ],
            condition=IfCondition(use_zenoh_bridge),
        ),
        activate_node_service("motion_planner_server", ros_domain_id.perform(context)),
    ]


def generate_launch_description():

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "workcell_id",
            default_value="workcell_1",
            description="Workcell ID used for registration with the system orchestrator",
        ),
        DeclareLaunchArgument(
            "bt_path",
            description="Path to load the BTs from",
        ),
        DeclareLaunchArgument(
            "task_checker_plugin",
            description="Fully qualified name of the plugin to load to check if a task is doable.",
        ),
        DeclareLaunchArgument(
            "max_jobs",
            default_value="1",
            description="Maximum number of parallel jobs that this workcell is allowed to handle.",
        ),
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="0",
            description="ROS_DOMAIN_ID environment variable",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Launch in headless mode (no gui)",
        ),
        DeclareLaunchArgument(
            "controller_config_package",
            default_value="nexus_demos",
            description="Package with the controller\'s configuration in 'config' folder",
        ),
        DeclareLaunchArgument(
            "planner_config_package",
            default_value="nexus_motion_planner",
            description="Package with the planner params config",
        ),
        DeclareLaunchArgument(
            "planner_config_file",
            default_value="planner_params.yaml",
            description="Name of planner server configuration file",
        ),
        DeclareLaunchArgument(
            "support_package",
            default_value="abb_irb910sc_support",
            description="Name of the support package",
        ),
        DeclareLaunchArgument(
            "robot_xacro_file",
            default_value="irb910sc_3_45.xacro",
            description="Xacro describing the robot.",
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="abb_irb910sc_3_45_moveit_config",
            description="MoveIt configuration package for the robot",
        ),
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="abb_irb910sc_3_45.srdf.xacro",
            description="Name of the SRDF file",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="abb_irb910sc_controllers.yaml",
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "tf_publisher_launch_file",
            default_value="workcell_1_tf.launch.py",
            description="Launch file to start TF publisher nodes.",
        ),
        DeclareLaunchArgument(
            "sku_detection_params_file",
            default_value="irb910sc_detection.yaml",
            description="Config file with the detected SKU poses",
        ),
        DeclareLaunchArgument(
            "dispenser_properties",
            default_value="",
            description="Dispenser properties of the workcell",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="True",
            description="Set True if running with real hardware.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="",
            description="The IP address of the real robot when use_fake_hardware is False.",
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
            default_value="config/zenoh/workcell_1.json5",
            description="Zenoh DDS bridge configuration filepath",
        ),
        DeclareLaunchArgument(
            "remap_task_types",
            default_value="",
            description="A yaml containing a dictionary of task types and an array of remaps",
        ),
        DeclareLaunchArgument(
            "remap_task_input_output_stations",
            default_value="",
            description="A yaml containing a dictionary of remapping task types and input/output station names. By default the workcell name is used if the work order expects an input or output station",
        ),
        OpaqueFunction(function = launch_setup)
    ])
