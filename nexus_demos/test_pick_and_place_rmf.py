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

class PickAndPlaceTest(NexusTestCase):
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
        print("waiting for nodes to be ready...", file=sys.stderr)
        self.wait_for_nodes("system_orchestrator")
        await self.wait_for_lifecycle_active("system_orchestrator")

        await self.wait_for_workcells("workcell_1", "workcell_2", "rmf_nexus_transporter")
        print("all workcells are ready")
        await self.wait_for_robot_state()
        print("AMRs are ready")

        # give some time for discovery to happen
        await self.ros_sleep(5)

        # create action client to send work order
        self.action_client = ActionClient(
            self.node, ExecuteWorkOrder, "/system_orchestrator/execute_order"
        )

    def tearDown(self):
        self.proc.__exit__(None, None, None)

    @RosTestCase.timeout(600)  # 10min
    async def test_pick_and_place_wo(self):
        self.action_client.wait_for_server()
        goal_msg = ExecuteWorkOrder.Goal()
        goal_msg.order.work_order_id = "1"
        with open(f"{os.path.dirname(__file__)}/config/pick_and_place.json") as f:
            goal_msg.order.work_order = f.read()
        feedbacks: list[ExecuteWorkOrder.Feedback] = []
        fb_fut = Future()

        def on_fb(msg):
            feedbacks.append(msg.feedback)
            if len(feedbacks) >= 5:
                fb_fut.set_result(None)

        goal_handle = cast(
            ClientGoalHandle, await self.action_client.send_goal_async(goal_msg, on_fb)
        )
        self.assertTrue(goal_handle.accepted)

        results = await goal_handle.get_result_async()
        self.assertEqual(results.status, GoalStatus.STATUS_SUCCEEDED)

        # check that we receive the correct feedbacks
        # FIXME(koonpeng): First few feedbacks are sometimes missed when the system in under
        #   high load so we only check the last feedback as a workaround.
        self.assertGreater(len(feedbacks), 0)
        for msg in feedbacks:
            # The first task is transportation
            self.assertEqual(len(msg.task_states), 3)
            state: TaskState = msg.task_states[1]  # type: ignore
            self.assertEqual(state.workcell_id, "workcell_1")
            self.assertEqual(state.task_id, "1/place_on_conveyor/0")
            state: TaskState = msg.task_states[2]  # type: ignore
            self.assertEqual(state.workcell_id, "workcell_2")
            self.assertEqual(state.task_id, "1/pick_from_conveyor/1")

        # state: TaskState = feedbacks[-1].task_states[0]  # type: ignore
        # self.assertEqual(state.status, TaskState.STATUS_FINISHED)
        # state: TaskState = feedbacks[-1].task_states[1]  # type: ignore
        # self.assertEqual(state.status, TaskState.STATUS_FINISHED)
        # state: TaskState = feedbacks[-1].task_states[2]  # type: ignore
        # self.assertEqual(state.status, TaskState.STATUS_FINISHED)
