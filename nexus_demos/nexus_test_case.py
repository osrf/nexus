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

from typing import Sequence

from nexus_orchestrator_msgs.srv import ListTransporters, ListWorkcells

from ros_testcase import RosTestCase


class NexusTestCase(RosTestCase):
    async def wait_for_workcells(self, *workcells: Sequence[str]):
        s = set(workcells)
        while True:
            client = self.node.create_client(ListWorkcells, "/list_workcells")
            client.wait_for_service()
            resp: ListWorkcells.Response = await client.call_async(
                ListWorkcells.Request()
            )
            missing = s.difference(wc.workcell_id for wc in resp.workcells)
            if not missing:
                return
            else:
                print("waiting for workcells", missing)
                await self.ros_sleep(0.1)

    async def wait_for_transporters(self, *transporters: Sequence[str]):
        s = set(transporters)
        while True:
            client = self.node.create_client(ListTransporters, "/list_transporters")
            client.wait_for_service()
            resp: ListTransporters.Response = await client.call_async(
                ListTransporters.Request()
            )
            missing = s.difference(wc.workcell_id for wc in resp.workcells)
            if not missing:
                return
            else:
                print("waiting for transporters", missing)
                await self.ros_sleep(0.1)
