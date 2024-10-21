# nexus_workcell_editor

A GUI for assembling workcells from components that is built off [rmf_site](https://github.com/open-rmf/rmf_site).

## Setup

Install rustup from the Rust website: https://www.rust-lang.org/tools/install

```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Follow instructions [here](https://github.com/ros2-rust/ros2_rust) to setup ros2_rust.

## Build
```
# source the ros distro and ros2_rust workspace.
cd ~/ws_nexus
colcon build
```

## Run
```bash
cd ~/ws_nexus
source ~/ws_nexus/install/setup.bash
ros2 run nexus_workcell_editor nexus_workcell_editor
```

