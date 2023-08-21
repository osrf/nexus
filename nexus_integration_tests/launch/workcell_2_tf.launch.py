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
        name="irb1300_static_transform_publisher_world_t_base_link",
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
        name="irb1300_static_transform_publisher_t_home",
        arguments=[
            "--x",
            "-0.2",
            "--y",
            "0.0",
            "--z",
            "-0.05",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "dropoff_pose",
            "--child-frame-id",
            "home",
        ],
    )

    # Static TF representing dropoff pose
    static_tf_dropoff_goal_pose = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="irb1300_static_transform_publisher_base_link_t_dropoff_goal_pose",
        arguments=[
            "--x",
            "0.18717",
            "--y",
            "0.59841",
            "--z",
            "0.43492",
            "--qw",
            "0.0163",
            "--qx",
            "0.01503",
            "--qy",
            "1.0",
            "--qz",
            "0.00245",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "dropoff_pose",
        ],
    )

    # Static TF of camera that detects the product
    static_tf_sku_detection_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="irb1300_static_transform_publisher_base_link_t_sku_detection_camera",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "-1.0",
            "--roll",
            "0.0",
            "--pitch",
            "0",
            "--yaw",
            "0.0",
            "--frame-id",
            "pallet",
            "--child-frame-id",
            "sku_detection_camera",
        ],
    )

    # Static TF of pallet
    static_tf_pallet = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="irb1300_static_transform_publisher_base_link_t_pallet",
        arguments=[
            "--x",
            "0.59456",
            "--y",
            "0.37689",
            "--z",
            "0.37805",
            "--qw",
            "0.0163",
            "--qx",
            "0.01503",
            "--qy",
            "1.0",
            "--qz",
            "0.00245",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "pallet",
        ],
    )

    return LaunchDescription(
        [
            GroupAction(
                actions=[
                    static_tf_base_link,
                    static_tf_home,
                    static_tf_dropoff_goal_pose,
                    static_tf_sku_detection_camera,
                    static_tf_pallet,
                ]
            )
        ]
    )
