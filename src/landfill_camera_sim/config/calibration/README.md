# Sensor calibration

Plain-text, editable calibration for the RGB camera, thermal camera, and
Livox lidar, loaded at launch time by `scripts/calibration_publisher.py`
and consumed by `scripts/sensor_fusion_node.py`:

- `extrinsics.yaml` -- rigid transform from the rig frame
  (`camera_platform/camera_link`) to each sensor's frame. Published as
  tf2 static transforms; used to project lidar points into each camera.
- `intrinsics.yaml` -- pinhole camera matrix and distortion coefficients
  for the RGB and thermal cameras, in `sensor_msgs/CameraInfo` layout.
  Republished on the `.../camera_info` topics in place of Gazebo's own
  camera_info bridge, so this file is the single source of truth for
  intrinsics used anywhere downstream.

Edit either file and relaunch to change calibration -- nothing else reads
these values at build time, only `calibration_publisher.py` at runtime.
