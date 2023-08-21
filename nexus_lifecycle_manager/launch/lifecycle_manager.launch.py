# Copyright 2022 Johnson & Johnson
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

from launch import LaunchDescription
from launch.actions import (ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler)
from launch.substitutions import (FindExecutable)
from launch_ros.actions import LifecycleNode
from launch.event_handlers import (OnProcessExit)

def set_lifecycle(server_name, transition):
  return ExecuteProcess(
      cmd=[[
          FindExecutable(name='ros2'),
          ' lifecycle set ',
          server_name + ' ',
          transition
      ]],
      name='nexus_lifecycle_manager_task_client',
      shell=True,
      output='screen',
  )


def generate_launch_description():
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    lifecycle_set_configure = set_lifecycle('/lifecycle_manager', 'configure')
    lifecycle_set_activate = set_lifecycle('/lifecycle_manager', 'activate')

    trigger_lifecycle_activate = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_set_configure,
            on_exit=[
                lifecycle_set_activate
            ]
        )
    )

    return LaunchDescription([
        lifecycle_set_configure,
        trigger_lifecycle_activate,
        LifecycleNode(package='nexus_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager', namespace='', output='screen')
    ])
