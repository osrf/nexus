# nexus_workcell_editor

A GUI for assembling workcells from components that is built off [rmf_site](https://github.com/open-rmf/rmf_site).

## Setup

Follow instructions [here](https://github.com/ros2-rust/ros2_rust) to setup ros2_rust.
> Note: Checkout `9a845c17873cbdf49e8017d5f0af6d8f795589cc` commit to include fix for https://github.com/ros2-rust/ros2_rust/issues/449. 


## Build
```
# source the ros distro and ros2_rust workspace.
cd ~/ws_nexus
rosdep install --from-paths src --ignore-src --rosdistro <DISTRO> # Replace <DISTRO> with supported ROS 2 distro, eg. jazzy.
colcon build
```

## Run
```bash
cd ~/ws_nexus
source ~/ws_nexus/install/setup.bash
ros2 run nexus_workcell_editor nexus_workcell_editor
```

