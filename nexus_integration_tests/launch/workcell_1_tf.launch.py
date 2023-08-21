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


import rclpy
import rclpy.node
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import GroupAction

def generate_launch_description():
    """Publish static TFs.
    """

    # Static TF of robot base_link to world
    static_tf_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="irb910sc_static_transform_publisher_world_t_base_link",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
    )

    # Static TF of robot idle/home pose
    static_tf_home = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="irb910sc_static_transform_publisher_pickup_pose_t_home",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.1",
            "--z",
            "-0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "pickup_pose",
            "--child-frame-id",
            "home",
        ],
    )

    # Static TF representing pickup pose (from dispenser)
    static_tf_pickup_pose = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="irb910sc_static_transform_publisher_base_link_t_pickup_pose",
        arguments=[
            "--x",
            "0.45",
            "--y",
            "0.0",
            "--z",
            "0.2",
            "--roll",
            "3.14",
            "--yaw",
            "3.14",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "pickup_pose",
        ],
    )

    # Static TF of pallet
    static_tf_pallet = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="irb910sc_static_transform_publisher_base_link_t_pallet",
        arguments=[
            "--x",
            "0.24412",
            "--y",
            "-0.23849",
            "--z",
            "-0.0433",
            "--qw",
            "1.0",
            "--qx",
            "0.0",
            "--qy",
            "0.0",
            "--qz",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "pallet",
        ],
    )

    # Static TF of dropoff_pose
    static_tf_dropoff = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="irb910sc_static_transform_publisher_base_link_t_pallet",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "-0.0",
            "--z",
            "0.25",
            "--qw",
            "0.0",
            "--qx",
            "1.0",
            "--qy",
            "-0.0",
            "--qz",
            "0.0",
            "--frame-id",
            "pallet",
            "--child-frame-id",
            "dropoff_pose",
        ],
    )

    return LaunchDescription(
        [
            GroupAction(
                actions=[
                    static_tf_base_link,
                    static_tf_home,
                    static_tf_pickup_pose,
                    static_tf_pallet,
                    static_tf_dropoff,
                ]
            )
        ]
    )
