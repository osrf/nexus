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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    if (
        "RMW_IMPLEMENTATION" not in os.environ
        or os.environ["RMW_IMPLEMENTATION"] != "rmw_zenoh_cpp"
    ):
        print(
            "Only RMW Zenoh is supported, the environment variable RMW_IMPLEMENTATION must be set to rmw_zenoh_cpp",
            file=sys.stderr,
        )
        exit(1)

    headless = LaunchConfiguration("headless")
    use_rmf_transporter = LaunchConfiguration("use_rmf_transporter")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot1_ip = LaunchConfiguration("robot1_ip")
    robot2_ip = LaunchConfiguration("robot2_ip")
    run_workcell_1 = LaunchConfiguration("run_workcell_1")
    run_workcell_2 = LaunchConfiguration("run_workcell_2")

    log_msg = ""

    main_bt_filename = "main.xml"
    remap_task_types = """{
                        pick_and_place: [place_on_conveyor, pick_from_conveyor],
                    }"""
    rviz_config_filename = "nexus_panel.rviz"
    if (use_rmf_transporter.perform(context).lower() == "true"):
        remap_task_types = """{
                            pick_and_place_rmf: [place_on_conveyor, pick_from_conveyor],
                        }"""
        main_bt_filename = "main_rmf.xml"
        rviz_config_filename = "nexus_panel_rmf.rviz"

    log_msg += f"System Orchestrator will load : {main_bt_filename}\n"
    nexus_rviz_config = os.path.join(
        get_package_share_directory("nexus_demos"), "rviz", rviz_config_filename)

    launch_inter_workcell = GroupAction(
        actions=[
            IncludeLaunchDescription(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nexus_demos"),
                            "launch",
                            "inter_workcell.launch.py",
                        ]
                    )
                ],
                launch_arguments={
                    "zenoh_config_package": "nexus_demos",
                    "zenoh_router_config_filename": "config/zenoh/system_orchestrator_router_config.json5",
                    "zenoh_session_config_filename": "config/zenoh/system_orchestrator_session_config.json5",
                    "use_rmf_transporter": use_rmf_transporter,
                    "transporter_plugin": "nexus_transporter::MockTransporter",
                    "activate_system_orchestrator": headless,
                    "headless": headless,
                    "main_bt_filename": main_bt_filename,
                    "remap_task_types": remap_task_types,
                    "nexus_rviz_config": nexus_rviz_config,
                }.items(),
            ),
        ],
    )

    launch_workcell_1 = GroupAction(
        actions=[
            IncludeLaunchDescription(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nexus_demos"),
                            "launch",
                            "workcell.launch.py",
                        ]
                    )
                ],
                launch_arguments={
                    "workcell_id": "workcell_1",
                    "bt_path": (
                        FindPackageShare("nexus_demos"),
                        "/config/workcell_1_bts",
                    ),
                    "task_checker_plugin": "nexus::task_checkers::FilepathChecker",
                    "headless": headless,
                    "controller_config_package": "nexus_demos",
                    "planner_config_package": "nexus_demos",
                    "planner_config_file": "abb_irb910sc_planner_params.yaml",
                    "support_package": "abb_irb910sc_support",
                    "robot_xacro_file": "irb910sc_3_45.xacro",
                    "moveit_config_package": "abb_irb910sc_3_45_moveit_config",
                    "moveit_config_file": "abb_irb910sc_3_45.srdf.xacro",
                    "controllers_file": "abb_irb910sc_controllers.yaml",
                    "tf_publisher_launch_file": "workcell_1_tf.launch.py",
                    "sku_detection_params_file": "product_detector_config.yaml",
                    "dispenser_properties": "productA",
                    "use_fake_hardware": use_fake_hardware,
                    "robot_ip": robot1_ip,
                    "zenoh_config_package": "nexus_demos",
                    "zenoh_router_config_filename": "config/zenoh/workcell_1_router_config.json5",
                    "zenoh_session_config_filename": "config/zenoh/workcell_1_session_config.json5",
                }.items(),
                condition=IfCondition(run_workcell_1),
            )
        ],
    )

    launch_workcell_2 = GroupAction(
        actions=[
            IncludeLaunchDescription(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nexus_demos"),
                            "launch",
                            "workcell.launch.py",
                        ]
                    )
                ],
                launch_arguments={
                    "workcell_id": "workcell_2",
                    "bt_path": (
                        FindPackageShare("nexus_demos"),
                        "/config/workcell_2_bts",
                    ),
                    "task_checker_plugin": "nexus::task_checkers::FilepathChecker",
                    "headless": headless,
                    "controller_config_package": "nexus_demos",
                    "planner_config_package": "nexus_demos",
                    "planner_config_file": "abb_irb1300_planner_params.yaml",
                    "support_package": "abb_irb1300_support",
                    "robot_xacro_file": "irb1300_10_115.xacro",
                    "moveit_config_package": "abb_irb1300_10_115_moveit_config",
                    "moveit_config_file": "abb_irb1300_10_115.srdf.xacro",
                    "controllers_file": "abb_irb1300_controllers.yaml",
                    "tf_publisher_launch_file": "workcell_2_tf.launch.py",
                    "sku_detection_params_file": "product_detector_config.yaml",
                    "dispenser_properties": "productB",
                    "use_fake_hardware": use_fake_hardware,
                    "robot_ip": robot2_ip,
                    "zenoh_config_package": "nexus_demos",
                    "zenoh_router_config_filename": "config/zenoh/workcell_2_router_config.json5",
                    "zenoh_session_config_filename": "config/zenoh/workcell_2_session_config.json5",
                }.items(),
                condition=IfCondition(run_workcell_2),
            )
        ],
    )

    return [
        LogInfo(msg=log_msg),
        launch_inter_workcell,
        launch_workcell_1,
        launch_workcell_2,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="Launch in headless mode (no gui)",
            ),
            DeclareLaunchArgument(
                "use_rmf_transporter",
                default_value="false",
                description="Set true to rely on an Open-RMF managed fleet to transport material\
                between workcells.",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Set True if running with real hardware.",
            ),
            DeclareLaunchArgument(
                "robot1_ip",
                default_value="",
                description="The IP address of workcell 1 robot when use_fake_hardware is False.",
            ),
            DeclareLaunchArgument(
                "robot2_ip",
                default_value="",
                description="The IP address of workcell 1 robot when use_fake_hardware is False.",
            ),
            DeclareLaunchArgument(
                "run_workcell_1",
                default_value="true",
                description="Whether to run workcell_1",
            ),
            DeclareLaunchArgument(
                "run_workcell_2",
                default_value="true",
                description="Whether to run workcell_2",
            ),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            OpaqueFunction(function=launch_setup),
        ]
    )
