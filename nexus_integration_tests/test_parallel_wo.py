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
from typing import cast

from nexus_orchestrator_msgs.action import ExecuteWorkOrder
from nexus_test_case import NexusTestCase
from managed_process import managed_process
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from ros_testcase import RosTestCase
import subprocess


class ParallelWoTest(NexusTestCase):
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
            ("ros2", "launch", "nexus_integration_tests", "office.launch.xml"),
        )
        self.proc.__enter__()
        print("waiting for nodes to be ready...", file=sys.stderr)
        self.wait_for_nodes("system_orchestrator")
        await self.wait_for_lifecycle_active("system_orchestrator")

        await self.wait_for_workcells("workcell_1", "workcell_2")
        print("all workcells are ready")
        await self.wait_for_transporters("transporter_node")
        print("all transporters are ready")

        # create action client to send work order
        self.action_client = ActionClient(
            self.node, ExecuteWorkOrder, "/system_orchestrator/execute_order"
        )
        self.action_client.wait_for_server()

    def tearDown(self):
        self.proc.__exit__(None, None, None)

    @RosTestCase.timeout(180)  # 3min
    async def test_reject_jobs_over_max(self):
        """
        New jobs should be rejected when the max number of jobs is already executing.
        """
        goal_msg = ExecuteWorkOrder.Goal()
        goal_msg.order.id = "1"
        with open(f"{os.path.dirname(__file__)}/config/place_on_conveyor.json") as f:
            goal_msg.order.work_order = f.read()
        goal_handle = cast(
            ClientGoalHandle, await self.action_client.send_goal_async(goal_msg)
        )
        self.assertTrue(goal_handle.accepted)

        goal_msg_2 = ExecuteWorkOrder.Goal()
        goal_msg_2.order.id = "2"
        with open(f"{os.path.dirname(__file__)}/config/pick_from_conveyor.json") as f:
            goal_msg_2.order.work_order = f.read()
        goal_handle_2 = cast(
            ClientGoalHandle, await self.action_client.send_goal_async(goal_msg_2)
        )
        self.assertTrue(goal_handle_2.accepted)

        goal_msg_3 = goal_msg
        goal_msg_3.order.id = "3"
        goal_handle_3 = cast(
            ClientGoalHandle, await self.action_client.send_goal_async(goal_msg_3)
        )
        self.assertFalse(goal_handle_3.accepted)
