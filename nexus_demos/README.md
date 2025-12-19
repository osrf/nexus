# nexus_demos

![](../docs/media/nexus_demo.png)

## Launch system and workcell orchestrators and all mock nodes
The [launch.py script](launch/launch.py) will launch the inter-workcell nodes (including the system orchestrator) and 2 workcells orchestrators (each with a IRB910SC and IRB1300 robot). The inter-workcell system, as well as each workcell's system will also be launched with a Zenoh router, while each workcell will require a Zenoh router that acts as a bridge for specific topics, services and actions. To launch these components individually, use the following examples given.

### Method 1: Launch system orchestrator, IRB1300 workcell and IRB910SC Workcell together with designated routers and bridges
> NOTE: Before running any of these commands, you must set the rmw implmentation to rmw_zenoh_cpp with
`export RMW_IMPLEMENTATION=rmw_zenoh_cpp`
(If testing with real hardware, specify the arguments `use_fake_hardware=False`, `robot1_ip=<IP>` and `robot2_ip=<IP>`)
```bash
ros2 launch nexus_demos launch.py headless:=False
```

### Method 2: Launch System Orchestrator and 1 Workcell
Launch with Workcell 1
```bash
ros2 launch nexus_demos launch.py headless:=False run_workcell_1:=true run_workcell_2:=false
```

Launch with Workcell 2
```bash
ros2 launch nexus_demos launch.py headless:=False run_workcell_1:=false run_workcell_2:=true
```

Testing with real hardware
```bash
ros2 launch nexus_demos launch.py headless:=False run_workcell_1:=True run_workcell_2:=False use_fake_hardware:=False robot1_ip:=<IP_ADDR>
```

## Launch Orchestrators individually

### System Orchestrator
```bash
ros2 launch nexus_demos inter_workcell.launch.py headless:=False
```

### IRB910SC Workcell
```bash
ros2 launch nexus_demos workcell.launch.py workcell_id:=workcell_1 support_package:=abb_irb910sc_support robot_xacro_file:=irb910sc_3_45.xacro moveit_config_package:=abb_irb910sc_3_45_moveit_config controllers_file:=abb_irb910sc_controllers.yaml moveit_config_file:=abb_irb910sc_3_45.srdf.xacro tf_publisher_launch_file:=irb910sc_tf.launch.py planner_config_package:=nexus_demos planner_config_file:=irb910sc_planner_params.yaml sku_detection_params_file:=irb910sc_detection.yaml zenoh_router_config_filename:=config/zenoh/workcell_1_router_config.json5 zenoh_session_config_filename:=config/zenoh/workcell_1_session_config.json5 io_stations_config_file_path:=src/nexus/nexus_demos/config/workcell_1_io_config.yaml headless:=False
```

Start the bridge router,

```bash
ros2 launch nexus_demos zenoh_router.launch.py zenoh_router_config_filename:=config/zenoh/workcell_1_bridge_config.json5
```

### IRB1300 Workcell
```bash
ros2 launch nexus_demos workcell.launch.py workcell_id:=workcell_2 support_package:=abb_irb1300_support robot_xacro_file:=irb1300_10_115.xacro moveit_config_package:=abb_irb1300_10_115_moveit_config controllers_file:=abb_irb1300_controllers.yaml moveit_config_file:=abb_irb1300_10_115.srdf.xacro tf_publisher_launch_file:=irb1300_tf.launch.py sku_detection_params_file:=irb1300_detection.yaml zenoh_router_config_filename:=config/zenoh/workcell_2_router_config.json5 zenoh_session_config_filename:=config/zenoh/workcell_2_session_config.json5 io_stations_config_file_path:=src/nexus/nexus_demos/config/workcell_2_io_config.yaml headless:=False
```

Start the bridge router,

```bash
ros2 launch nexus_demos zenoh_router.launch.py zenoh_router_config_filename:=config/zenoh/workcell_2_bridge_config.json5
```

## Submit a job

Modify the values of `work_order_id` and the name of the work orders to these provided values in our examples,
- `place_on_conveyor`
- `pick_from_conveyor`, only works if `workcell_2` is launched
- `pick_and_place_conveyor`, only works if both `workcell_1` and `workcell_2` are launched
- `pick_and_place_amr`, only works if both `workcell_1` and `workcell_2` are launched, and the demo is started with `use_multiple_transporters:=True`

Using `place_on_conveyor` as an example work order, with ID 23:
```bash
ros2 action send_goal /system_orchestrator/execute_order nexus_orchestrator_msgs/action/ExecuteWorkOrder "{order: {work_order_id: '23', work_order: '$(cat config/place_on_conveyor.json)'}}"
```

## Debugging

## workcell

Send a request to a specific workcell, eg. `test_workcell`.

```bash
ros2 action send_goal /test_workcell/request nexus_orchestrator_msgs/action/WorkcellTask "$(cat config/workcell_task.yaml)" -f
```

## mock gripper and mock detection

Gripper position from 0 to `gripper_max_value`:

```bash
ros2 run nexus_demos mock_gripper --ros-args -p gripper_max_value:=0.5
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
ros2 run nexus_demos mock_detector --ros-args -p config_file:=<path config file>
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
ros2 run nexus_demos mock_printer_node
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
ros2 run nexus_demos mock_robot_arm_controller
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
ros2 launch abb_bringup abb_control.launch.py runtime_config_package:=nexus_demos description_package:=abb_irb910sc_support description_file:=irb910sc_3_45.xacro launch_rviz:=false moveit_config_package:=abb_irb910sc_3_45_moveit_config use_fake_hardware:=true controllers_file:=abb_irb910sc_controllers.yaml
```

If running with real robot
```bash
ros2 launch abb_bringup abb_control.launch.py runtime_config_package:=nexus_demos description_package:=abb_irb910sc_support description_file:=irb910sc_3_45.xacro launch_rviz:=false moveit_config_package:=abb_irb910sc_3_45_moveit_config use_fake_hardware:=true controllers_file:=abb_irb910sc_controllers.yaml use_fake_hardware:=false rws_ip:=<ROBOTSTUDIO_IP>
```

Then launch moveit
```bash
ros2 launch nexus_demos abb_irb910sc_moveit.launch.py
```

### IRB1300

For more information see README in `abb_ros2` repo.
First launch `abb_control`
```bash
ros2 launch abb_bringup abb_control.launch.py runtime_config_package:=nexus_demos description_package:=abb_irb1300_support description_file:=irb1300_10_115.xacro launch_rviz:=false moveit_config_package:=abb_irb1300_10_115_moveit_config use_fake_hardware:=true controllers_file:=abb_irb1300_controllers.yaml
```

If running with real robot
```bash
ros2 launch abb_bringup abb_control.launch.py runtime_config_package:=nexus_demos description_package:=abb_irb1300_support description_file:=irb1300_10_115.xacro launch_rviz:=false moveit_config_package:=abb_irb1300_10_114_moveit_config use_fake_hardware:=true controllers_file:=abb_irb1300_controllers.yaml use_fake_hardware:=false rws_ip:=<ROBOTSTUDIO_IP>
```

Then launch moveit
```bash
ros2 launch nexus_demos abb_irb1300_moveit.launch.py
```
