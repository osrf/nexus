# nexus_integration_tests

## Launch system and workcell orchestrators and all mock nodes
The [launch.py script](launch/launch.py) will launch the system orchestrator and 2 workcells orchestrators (each with a IRB910SC and IRB1300 robot), along with a Zenoh bridge to link selected ROS endpoints between them. These 3 orchestrators and their accompany components will be in different ROS_DOMAIN_IDs. To launch these components individually, use the following examples given.

>NOTE: The ROS_DOMAIN_ID occupied by the Zenoh bridges during launch time may be different from the `domain` values in the Zenoh bridge configurations. This is because the launch file overrides the domain ID of the zenoh bridges to ensure that it is same as that of the orchestrator.

### Launch system orchestrator, IRB1300 workcell and IRB910SC Workcell together with Zenoh bridge
> NOTE: Before running any of these commands, you must set the rmw implmentation to cyclonedds with
`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
(If testing with real hardware, specify the arguments `use_fake_hardware=False`, `robot1_ip=<IP>` and `robot2_ip=<IP>`)
```bash
ros2 launch nexus_integration_tests launch.py headless:=False
```

### Launch System Orchestrator and 1 Workcell without Zenoh bridge (Same ROS_DOMAIN_ID)
Launch with Workcell 1
```bash
ros2 launch nexus_integration_tests launch.py headless:=False use_zenoh_bridge:=False run_workcell_1:=true run_workcell_2:=false
```

Launch with Workcell 2
```bash
ros2 launch nexus_integration_tests launch.py headless:=False use_zenoh_bridge:=False run_workcell_1:=false run_workcell_2:=true
```

Testing with real hardware
```bash
ros2 launch nexus_integration_tests launch.py headless:=False use_zenoh_bridge:=False run_workcell_1:=True run_workcell_2:=False use_fake_hardware:=False robot1_ip:=<IP_ADDR>
```

## Launch Orchestrators individually

### System Orchestrator
```bash
ros2 launch nexus_integration_tests control_center.launch.py ros_domain_id:=0 headless:=False
```

### IRB910SC Workcell
```bash
ros2 launch nexus_integration_tests workcell.launch.py workcell_id:=workcell_1 ros_domain_id:=1 support_package:=abb_irb910sc_support robot_xacro_file:=irb910sc_3_45.xacro moveit_config_package:=abb_irb910sc_3_45_moveit_config controllers_file:=abb_irb910sc_controllers.yaml moveit_config_file:=abb_irb910sc_3_45.srdf.xacro tf_publisher_launch_file:=irb910sc_tf.launch.py planner_config_package:=nexus_integration_tests planner_config_file:=irb910sc_planner_params.yaml sku_detection_params_file:=irb910sc_detection.yaml zenoh_config_file:=workcell_1.json5 headless:=False
```

### IRB1300 Workcell
```bash
ros2 launch nexus_integration_tests workcell.launch.py workcell_id:=workcell_2 ros_domain_id:=2 support_package:=abb_irb1300_support robot_xacro_file:=irb1300_10_115.xacro moveit_config_package:=abb_irb1300_10_115_moveit_config controllers_file:=abb_irb1300_controllers.yaml moveit_config_file:=abb_irb1300_10_115.srdf.xacro tf_publisher_launch_file:=irb1300_tf.launch.py sku_detection_params_file:=irb1300_detection.yaml zenoh_config_file:=workcell_2.json5 headless:=False
```

### Send a goal to the workcell orchestrator
```bash
ros2 action send_goal /test_workcell/request nexus_orchestrator_msgs/action/WorkcellTask "$(cat config/workcell_task.yaml)" -f
```

### Send a work order to the system orchestrator
> Note: Set your ROS_DOMAIN_ID environment variable to that of the system orchestrator before executing the work order

`place_on_conveyor` work order:
```bash
ros2 action send_goal /system_orchestrator/execute_order nexus_orchestrator_msgs/action/ExecuteWorkOrder "{order: {id: '23', work_order: '$(cat config/place_on_conveyor.json)'}}"
```

`pick_from_conveyor` work order:
```bash
ros2 action send_goal /system_orchestrator/execute_order nexus_orchestrator_msgs/action/ExecuteWorkOrder "{order: {id: '24', work_order: '$(cat config/pick_from_conveyor.json)'}}"
```

## mock gripper and mock detection

Gripper position from 0 to `gripper_max_value`:

```bash
ros2 run nexus_integration_tests mock_gripper --ros-args -p gripper_max_value:=0.5
```

To transition lifecycle states
```bash
ros2 lifecycle set /mock_gripper configure
ros2 lifecycle set /mock_gripper activate
```

Call the action

```bash
ros2 action send_goal /mock_gripper/command  control_msgs/action/GripperCommand "{command: {position: 0.42}}" -f

## mock vision system

Define the mock detections:

```yaml
keys:
  - sku_id1
  - sku_id2
sku_id1:
  score: [0.2, 0.8]
  class_id: [id1, id1]
  position: [[2, 2.1, 2.2], [4.5, 5.4, 3.7]]
  rotation: [[0.707, 0, 0.707, 1], [0, 0, 0, 1]]
sku_id2:
  score: [0.7, 0.3]
  class_id: [id2, id2]
  position: [[1, 0, 0], [0, 0, 0]]
  rotation: [[0, 0, 0, 1], [0, 0, 0, 1]]
```

Save this file and launch the vision mock node:

```bash
ros2 run nexus_integration_tests mock_detection --ros-args -p config_file:=<path config file>
```

To transition lifecycle states
```bash
ros2 lifecycle set /mock_detector_node configure
ros2 lifecycle set /mock_detector_node activate
```

```bash
ros2 service call /mock_detector_node/detect nexus_detector_msgs/srv/Detect '{id: "sku_id1"}'
```

## mock_transporter
The [mock_transporter.cpp](src/mock_transporter.cpp) implements a pluginlib
loadable implementation of the [Transporter](../nexus_transporter/include/nexus_transporter/Transporter.hpp) class.

The plugin has a fully qualified name of `nexus_transporter::MockTransporter`.
To launch the `TransporterNode` with this plugin:
```bash
ros2 run nexus_transporter nexus_transporter_node --ros-args -p transporter_plugin:=nexus_transporter::MockTransporter
```

## mock_printer
To launch the mock printer:
```bash
ros2 run nexus_integration_tests mock_printer_node
```

To transition lifecycle states
```bash
ros2 lifecycle set /mock_printer configure
ros2 lifecycle set /mock_printer activate
```

To request for the printer to dispense a sample:
```bash
ros2 service call /mock_printer/dispense nexus_dispenser_msgs/srv/Dispense '{}'
```

## mock_robot_arm_controller
To launch the mock_robot_arm_controller
```bash
ros2 run nexus_integration_tests mock_robot_arm_controller
```

To launch the mock_robot_arm_controller
```bash
ros2 lifecycle set /mock_robot_arm_controller configure
ros2 lifecycle set /mock_robot_arm_controller activate
```

To request for a controller action:
```
ros2 action send_goal /joint_trajectory_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{}' -f
```

## abb bringup

### IRB910SC

First launch `abb_control`
```bash
ros2 launch abb_bringup abb_control.launch.py runtime_config_package:=nexus_integration_tests description_package:=abb_irb910sc_support description_file:=irb910sc_3_45.xacro launch_rviz:=false moveit_config_package:=abb_irb910sc_3_45_moveit_config use_fake_hardware:=true controllers_file:=abb_irb910sc_controllers.yaml
```

If running with real robot
```bash
ros2 launch abb_bringup abb_control.launch.py runtime_config_package:=nexus_integration_tests description_package:=abb_irb910sc_support description_file:=irb910sc_3_45.xacro launch_rviz:=false moveit_config_package:=abb_irb910sc_3_45_moveit_config use_fake_hardware:=true controllers_file:=abb_irb910sc_controllers.yaml use_fake_hardware:=false rws_ip:=<ROBOTSTUDIO_IP>
```

Then launch moveit
```bash
ros2 launch nexus_integration_tests abb_irb910sc_moveit.launch.py
```

### IRB1300

For more information see README in `abb_ros2` repo.
First launch `abb_control`
```bash
ros2 launch abb_bringup abb_control.launch.py runtime_config_package:=nexus_integration_tests description_package:=abb_irb1300_support description_file:=irb1300_10_115.xacro launch_rviz:=false moveit_config_package:=abb_irb1300_10_115_moveit_config use_fake_hardware:=true controllers_file:=abb_irb1300_controllers.yaml
```

If running with real robot
```bash
ros2 launch abb_bringup abb_control.launch.py runtime_config_package:=nexus_integration_tests description_package:=abb_irb1300_support description_file:=irb1300_10_115.xacro launch_rviz:=false moveit_config_package:=abb_irb1300_10_114_moveit_config use_fake_hardware:=true controllers_file:=abb_irb1300_controllers.yaml use_fake_hardware:=false rws_ip:=<ROBOTSTUDIO_IP>
```

Then launch moveit
```bash
ros2 launch nexus_integration_tests abb_irb1300_moveit.launch.py
```
