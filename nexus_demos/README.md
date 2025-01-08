# nexus_demos

## Regenerating the simulation world file and nav graph

Source build of `rmf_building_map_tools` is required, at least from commit hash [0d18f59](https://github.com/open-rmf/rmf_traffic_editor/tree/0d18f593356fa2e4de0dbfa297ae1fba66b8e101) onwards.

Generate world file,

```bash
# Source the workspace where rmf_building_map_tools is built

cd ~/ws_nexus/src/nexus/nexus_demos
ros2 run rmf_building_map_tools building_map_generator gazebo \
  maps/depot/depot.building.yaml \
  maps/depot/depot.world \
  maps/depot/models \
  --TEMPLATE_WORLD_FILE maps/depot/template/depot_world.sdf \
  --SKIP_CAMERA_POSE
```

Generate navigation graphs,

```bash
ros2 run rmf_building_map_tools building_map_generator nav \
  maps/depot/depot.building.yaml \
  maps/depot/nav_graphs
```

## Build

Build `nexus_demos`,

```bash
colcon build --packages-up-to nexus_demos
```

## Launch

Launch the demo,

```bash
ros2 launch nexus_demos depot.launch.xml headless:=0
```
