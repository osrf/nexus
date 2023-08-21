# NEXUS Robot Controller Server

# Description

Wraps `ros2_control` controller manager into a lifecycle node.

The controller manager manages the loading of hardware interface plugins and controllers.
And this node manages the manager in accordance with node lifecycles.

Specifically, it does as appropriate at lifecycle transitions:

- Load/unload controllers
- Switch controllers
- Spin up or destroy the `ros2_control` controller manager instance

# Usage

To interface with this server, you just need to set ROS 2 parameters!

## Set These ROS Parameters

- The robot description on `~/robot_description`
  - [With the appropriate hardware descriptions](https://control.ros.org/master/doc/getting_started/getting_started.html#hardware-description-in-urdf)
- The controller configs
  - [Like in the ros2_control example](https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_bringup/config/rrbot_controllers.yaml))
- The controllers you want this server to load and activate on `~/managed_controllers`

## Transition The Server

Then you'll just have to run the appropriate lifecycle transitions!

- **Configure** spins up the `ros2_control` controller manager instance, loads the hardware interface plugins, loads the robot description, and loads the managed controllers
- **Activate** switches the managed controllers on, allowing the robot to be controlled
- **Deactivate** undoes **Activate**
- **Cleanup** undoes **Configure**

Configure will spin the `ros2_control` controller manager in a separate thread, and in a separate node.

## Interfacing with the Server

You should not be calling the `ros2_control` controller manager's services!

To change what controllers you're using, transition the server's lifecycle to **Unconfigured** and change the `~/managed_controllers` parameters to the appropriate controllers you want loaded and activated!
The server will dynamically update what controllers are loaded!
Fancy!

# Demo

A simple demo has been provided, using the `ros2_control` RRbot.

## Setup

You'll need to install [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)! This is to get the `rrbot_description` package that the demo relies on.

```
git clone https://github.com/ros-controls/ros2_control_demos
```

Also remember to install any dependencies!

```
rosdep install --from-paths src/nexus/nexus_robot_controller --ignore-src
```

## Running

In one terminal run the demo launchfile, which starts the RRbot base, RViz, and the robot controller server:

```
ros2 launch nexus_robot_controller rrbot_demo.launch.py
```

In another terminal run a loop that sends commands to the controller we'll start:

```shell
watch -n0.1 "ros2 topic pub -1 forward_position_controller/commands std_msgs/msg/Float64MultiArray '{data: [-5.0, -5.0]}' \
          && ros2 topic pub -1 forward_position_controller/commands std_msgs/msg/Float64MultiArray '{data: [5.0, 5.0]}'"
```

Finally, transition the robot controller server!

```shell
ros2 lifecycle set /robot_controller_server configure   # Loads the controllers
ros2 lifecycle set /robot_controller_server activate    # Switch the controllers on (the robot should start moving)
ros2 lifecycle set /robot_controller_server deactivate  # Switch the controllers off (the robot should stop moving)
ros2 lifecycle set /robot_controller_server cleanup     # Unloads the controllers
```

