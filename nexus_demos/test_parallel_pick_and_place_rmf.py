# Copyright (C) 2025 Open Source Robotics Foundation
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
from typing import cast

from action_msgs.msg import GoalStatus
from managed_process import managed_process
from nexus_orchestrator_msgs.action import ExecuteWorkOrder
from nexus_orchestrator_msgs.msg import TaskState
from nexus_test_case import NexusTestCase
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from ros_testcase import RosTestCase
import subprocess

class ParallelPickAndPlaceRmfTest(NexusTestCase):
    @RosTestCase.timeout(60)
    async def asyncSetUp(self):
        # todo(YV): Find a better fix to the problem below.
        # zenoh-bridge was bumped to 0.72 as part of the upgrade to
        # ROS 2 Iron. However with this upgrade, the bridge does not clearly
        # terminate when a SIGINT is received leaving behind zombie bridge
        # processes from previous test cases. As a result, workcell registration
        # fails for this testcase due to multiple bridges remaining active.
        # Hence we explicitly kill any zenoh processes before launching the test.
        subprocess.Popen('pkill -9 -f zenoh', shell=True)

        self.proc = managed_process(
            (
                "ros2",
                "launch",
                "nexus_demos",
                "launch.py",
                "sim_update_rate:=10000",
                "use_rmf_transporter:=true"
            ),
        )
        self.proc.__enter__()

        # give some time for discovery to happen
        await self.ros_sleep(5)

    def tearDown(self):
        self.proc.__exit__(None, None, None)

    @RosTestCase.timeout(3000)  # 5min
    async def test_parallel_pick_and_place_wo(self):
        print("waiting for nodes to be ready...", file=sys.stderr)
        self.wait_for_nodes("system_orchestrator")
        await self.wait_for_lifecycle_active("system_orchestrator")

        await self.wait_for_workcells("workcell_1", "workcell_2", "rmf_nexus_transporter")
        print("all workcells are ready")
        await self.wait_for_robot_state()
        print("AMRs are ready")

        # create action client to send work order
        self.action_client = ActionClient(
            self.node, ExecuteWorkOrder, "/system_orchestrator/execute_order"
        )
        self.action_client.wait_for_server()

        goal_msg = ExecuteWorkOrder.Goal()
        with open(f"{os.path.dirname(__file__)}/config/pick_and_place.json") as f:
            goal_msg.order.work_order = f.read()

        # First goal
        goal_msg.order.work_order_id = "1"
        first_feedbacks: list[ExecuteWorkOrder.Feedback] = []
        first_fb_fut = Future()

        def on_first_fb(msg):
            first_feedbacks.append(msg.feedback)
            if len(first_feedbacks) >= 5:
                first_fb_fut.set_result(None)

        first_goal_handle = cast(
            ClientGoalHandle, await self.action_client.send_goal_async(goal_msg, on_first_fb)
        )
        self.assertTrue(first_goal_handle.accepted)

        # Second goal
        goal_msg.order.work_order_id = "2"
        second_feedbacks: list[ExecuteWorkOrder.Feedback] = []
        second_fb_fut = Future()

        def on_second_fb(msg):
            second_feedbacks.append(msg.feedback)
            if len(second_feedbacks) >= 5:
                second_fb_fut.set_result(None)

        second_goal_handle = cast(
            ClientGoalHandle, await self.action_client.send_goal_async(goal_msg, on_second_fb)
        )
        self.assertTrue(second_goal_handle.accepted)

        # # Third goal
        # goal_msg.order.work_order_id = "3"
        # third_feedbacks: list[ExecuteWorkOrder.Feedback] = []
        # third_fb_fut = Future()

        # def on_third_fb(msg):
        #     third_feedbacks.append(msg.feedback)
        #     if len(third_feedbacks) >= 5:
        #         third_fb_fut.set_result(None)

        # third_goal_handle = cast(
        #     ClientGoalHandle, await self.action_client.send_goal_async(goal_msg, on_third_fb)
        # )
        # self.assertTrue(third_goal_handle.accepted)

        # Results
        results = await first_goal_handle.get_result_async()
        self.assertEqual(results.status, GoalStatus.STATUS_SUCCEEDED)
        results = await second_goal_handle.get_result_async()
        self.assertEqual(results.status, GoalStatus.STATUS_SUCCEEDED)
        # results = await third_goal_handle.get_result_async()
        # self.assertEqual(results.status, GoalStatus.STATUS_SUCCEEDED)
