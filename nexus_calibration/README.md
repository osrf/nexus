## nexus_calibration

This package provides a ROS 2 Lifecyle node, `nexus_calibration_node`, that connects to a [VRPN](https://vrpn.github.io/) server and builds a cache of rigid body poses in a local `TF2` buffer.
These rigid bodies could represent calibration links (or reference links) on components within a workcell.
The poses of these links can then be queried via a ROS 2 service call over the `nexus::endpoints::ExtrinsicCalibrationService` endpoint.

## Test
```bash
cd nexus_calibration/
ros2 launch test/nexus_calibration.launch.py
``
Then to retrieve poses of components wrt to the `robot_calibration_link`, ie a reference frame on the robot's `base_link`,
```bash
ros2 service call /workcell_1/calibrate_extrinsics nexus_calibration_msgs/src/CalibrateExtrinsics '{frame_id: robot_calibration_link}'
```
