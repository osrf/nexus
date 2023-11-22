# Copyright 2023 Johnson & Johnson
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_pytest.tools import process as process_tools
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

import launch_pytest

import pytest
import os

# This would have been a gtest, but we need a move_group instance, which
# requires some parameters loaded and a separate node started.


@pytest.fixture
def moveit_config():
    return (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .planning_pipelines("ompl", ["ompl"])  # Sadly necessary
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .to_moveit_configs()
    )


# We need this so the move_group has something to interact with
@pytest.fixture
def robot_fixture(moveit_config):
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[moveit_config.to_dict()],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0",
                   "0.0", "0.0", "world", "panda_link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="log",
    )

    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(
                    controller)],
                shell=True,
                output="log",
            )
        ]

    return [
        run_move_group_node,
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        *load_controllers
    ]


@pytest.fixture
def motion_cache_test_runner_node(moveit_config):
    return Node(
        package="nexus_motion_planner",
        executable="test_motion_plan_cache",
        name="test_motion_plan_cache_node",
        output="screen",
        cached_output=True,
        parameters=[moveit_config.to_dict()]
    )


@launch_pytest.fixture
def launch_description(motion_cache_test_runner_node, robot_fixture):
    return LaunchDescription(
        robot_fixture +
        [
            motion_cache_test_runner_node,
            launch_pytest.actions.ReadyToTest()
        ]
    )


def validate_stream(expected):
    def wrapped(output):
        assert expected in output.splitlines(
        ), f"Did not get expected: {expected} in output:\n\n{output}"
    return wrapped


@pytest.mark.launch(fixture=launch_description)
def test_read_stdout(motion_cache_test_runner_node, launch_context):
    test_cases = [
        # Cache init
        ('init', [
            'Cache init',
        ]),

        # Motion plan cache
        ('test_motion_plans.empty', [
            'Plan cache initially empty',
            'Fetch all plans on empty cache returns empty',
            'Fetch best plan on empty cache returns nullptr',
        ]),

        ('test_motion_plans.put_plan_empty_frame', [
            'Put empty frame plan, no overwrite, not ok',
            'No plans in cache',
        ]),

        ('test_motion_plans.put_plan_req_empty_frame', [
            'Put empty frame req plan, no overwrite, not ok',
            'No plans in cache',
        ]),

        ('test_motion_plans.put_first', [
            'Put first valid plan, no overwrite, ok',
            'One plan in cache',
        ]),
        ('test_motion_plans.put_first.fetch_matching_no_tolerance', [
            'Fetch all returns one',
            'Fetch best plan is not nullptr',
            'Fetched plan on both fetches match',
            'Fetched plan matches original',
            'Fetched plan has correct execution time',
            'Fetched plan has correct planning time',
        ]),
        ('test_motion_plans.put_first.fetch_is_diff_no_tolerance', [
            'Fetch all returns one',
            'Fetch best plan is not nullptr',
            'Fetched plan on both fetches match',
            'Fetched plan matches original',
            'Fetched plan has correct execution time',
            'Fetched plan has correct planning time',
        ]),
        ('test_motion_plans.put_first.fetch_non_matching_out_of_tolerance', [
            'Fetch all returns empty',
            'Fetch best plan is nullptr',
        ]),
        ('test_motion_plans.put_first.fetch_non_matching_in_tolerance', [
            'Fetch all returns one',
            'Fetch best plan is not nullptr',
            'Fetched plan on both fetches match',
            'Fetched plan matches original',
            'Fetched plan has correct execution time',
            'Fetched plan has correct planning time',
        ]),
        ('test_motion_plans.put_first.fetch_swapped', [
            'Fetch all returns one',
            'Fetch best plan is not nullptr',
            'Fetched plan on both fetches match',
            'Fetched plan matches original',
            'Fetched plan has correct execution time',
            'Fetched plan has correct planning time',
        ]),
        ('test_motion_plans.put_first.fetch_smaller_workspace', [
            'Fetch all returns empty',
            'Fetch best plan is nullptr',
        ]),
        ('test_motion_plans.put_first.fetch_larger_workspace', [
            'Fetch all returns one',
            'Fetch best plan is not nullptr',
            'Fetched plan on both fetches match',
            'Fetched plan matches original',
            'Fetched plan has correct execution time',
            'Fetched plan has correct planning time',
            'Fetched plan has more restrictive workspace max_corner',
            'Fetched plan has more restrictive workspace min_corner',
        ]),

        ('test_motion_plans.put_worse', [
            'Put worse plan, no overwrite, not ok',
            'One plan in cache',
        ]),

        ('test_motion_plans.put_better', [
            'Put better plan, no overwrite, ok',
            'Two plans in cache',
        ]),
        ('test_motion_plans.put_better.fetch_sorted', [
            'Fetch all returns two',
            'Fetch best plan is not nullptr',
            'Fetched plan on both fetches match',
            'Fetched plan matches original',
            'Fetched plan has correct execution time',
            'Fetched plan has correct planning time',
            'Fetched plans are sorted correctly',
        ]),

        ('test_motion_plans.put_better_overwrite', [
            'Put better plan, overwrite, ok',
            'One plan in cache',
        ]),
        ('test_motion_plans.put_better_overwrite.fetch', [
            'Fetch all returns one',
            'Fetch best plan is not nullptr',
            'Fetched plan on both fetches match',
            'Fetched plan matches original',
            'Fetched plan has correct execution time',
            'Fetched plan has correct planning time',
        ]),

        ('test_motion_plans.put_different_req', [
            'Put different plan req, overwrite, ok',
            'Two plans in cache',
        ]),
        ('test_motion_plans.put_different_req.fetch', [
            'Fetch all returns one',
            'Fetch best plan is not nullptr',
            'Fetched plan on both fetches match',
            'Fetched plan matches original',
            'Fetched plan has correct execution time',
            'Fetched plan has correct planning time',
        ]),

    ]

    for prefix, labels in test_cases:
        for label in labels:
            process_tools.assert_output_sync(
                launch_context,
                motion_cache_test_runner_node,
                validate_stream(f"[PASS] {prefix}: {label}"),
                timeout=10
            )

    # Check no occurrences of [FAIL] in output
    assert not process_tools.wait_for_output_sync(
        launch_context,
        motion_cache_test_runner_node,
        lambda x: "[FAIL]" in x,
        timeout=10
    )

    # Wait for process to end and check for graceful exit
    yield
    assert motion_cache_test_runner_node.return_code == 0
