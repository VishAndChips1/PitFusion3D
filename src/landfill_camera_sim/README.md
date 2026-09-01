# Landfill Camera Simulation

Gazebo + ROS 2 simulation of a sensor rig -- an RGB camera, a thermal
camera, and a Livox Avia-style lidar, all co-located in one enclosure --
plus a calibration layer and a fusion pipeline that colorizes the lidar
cloud with the camera data and merges it into a denser, motion-compensated
map.

## Scenarios

Two interchangeable worlds, both using the exact same sensor rig, config
schema, and node stack:

- **`landfill_camera_sim.launch.py`** (`config/sensors_sim.yaml`,
  `worlds/landfill_camera_world.sdf.in`) -- the rig rides a platform that
  drives back and forth along a landfill access road.
- **`scan_test_sim.launch.py`** (`config/scan_test_sim.yaml`,
  `worlds/scan_test_world.sdf.in`) -- the rig is stationary and slowly
  yaws in place (no translation) inside a ring of ~24 colorful primitive
  shapes (boxes, spheres, cylinders), for a clean 360-degree scan test.

```bash
ros2 launch landfill_camera_sim landfill_camera_sim.launch.py
ros2 launch landfill_camera_sim scan_test_sim.launch.py
```

Server-only (no GUI):

```bash
ros2 launch landfill_camera_sim landfill_camera_sim.launch.py gz_gui:=false rviz:=false
```

Sensor settings and topic names live in the YAML files under `config/`.
Platform motion is under `platform.motion` in each -- linear speed +
travel distance for the landfill run, `travel_distance: 0` with a nonzero
`angular_speed` for the stationary scan (see `platform_motion_controller.py`).

RViz opens by default (`rviz:=true`) showing the RGB image, thermal image,
and lidar point cloud together (`rviz/landfill_camera.rviz.in`).

## Lidar scan pattern

`livox_pattern_filter.py` reshapes the native Gazebo `gpu_lidar` scan into
the real, non-repetitive Avia scan pattern from
[Livox-SDK/livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)
(`config/scan_patterns/avia.csv`), rather than a synthetic curve -- see
`config/scan_patterns/README.md`.

## Calibration

`config/calibration/extrinsics.yaml` and `intrinsics.yaml` are the
editable, plain-text calibration for the rig: the rigid transform from the
rig frame to each sensor, and the RGB/thermal pinhole camera matrices.
`calibration_publisher.py` loads them at launch, publishes the extrinsics
as tf2 static transforms, and republishes `camera_info` on the RGB/thermal
topics from the intrinsics file (replacing Gazebo's own camera_info, so
the file is the single source of truth). Edit either file and relaunch to
change calibration -- see `config/calibration/README.md`.

## Sensor fusion pipeline

`sensor_fusion.launch.py` runs alongside either scenario above:

```bash
ros2 launch landfill_camera_sim landfill_camera_sim.launch.py
ros2 launch landfill_camera_sim sensor_fusion.launch.py
```

- **`sensor_fusion_node.py`** time-synchronizes one lidar scan with the
  RGB and thermal images (`message_filters.ApproximateTimeSynchronizer`),
  projects each lidar point into both cameras using the calibration files,
  and republishes the cloud with two extra fields: a packed `rgb` color
  and a `thermal` temperature (Kelvin) sampled from the images
  (`/.../fusion/points_rgbt`). Points outside a camera's field of view
  keep their lidar geometry but get no color/thermal value.
- **`registration_merge_node.py`** buffers a rolling 1-second window of
  those colorized scans and merges them into one denser cloud, aligning
  each scan to the growing map with point-to-point ICP (SciPy `cKDTree` +
  SVD/Kabsch, implemented in-file -- no PCL/Open3D dependency) to
  compensate for platform motion between scans before merging
  (`/.../fusion/points_merged`).

Both are plain parameters (topics, sync slop, window duration, ICP voxel
size/iterations) -- see `sensor_fusion.launch.py` for the full list, and
override `*_topic` there when running against `scan_test_sim.launch.py`'s
`/scan_test/...` topics instead of the landfill scenario's `/landfill/...`.

## Streaming the fused cloud over the network

`web_stream.launch.py` serves the merged cloud to any browser -- no ROS
install needed on the viewing device:

```bash
ros2 launch landfill_camera_sim web_stream.launch.py
```

`fusion_web_streamer.py` subscribes to `points_merged` (override
`input_topic` for `points_rgbt` or a `/scan_test/...` topic), keeps the
latest scan as a compact binary buffer, and serves it plus a
self-contained three.js viewer (RGB/thermal color toggle, orbit camera)
over plain HTTP on port 8080 (`http_port` param) -- open
`http://<this-machine>:8080/`.

**Reaching it from another device on WiFi, from WSL2:** WSL2 puts this
machine behind a NAT (its own IP, e.g. `172.18.213.120`, isn't reachable
from the LAN), but Windows forwards `localhost` into WSL2 automatically,
so `http://localhost:8080/` already works *on the Windows host itself*.
For other devices on the WiFi network, forward the port from Windows to
WSL2 -- in an elevated PowerShell on the Windows host:

```powershell
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=8080 connectaddress=<WSL2-IP> connectport=8080
netsh advfirewall firewall add rule name="PitFusion3D web stream" dir=in action=allow protocol=TCP localport=8080
```

Get `<WSL2-IP>` from `ip addr show eth0` inside WSL2 (it can change on
reboot, so redo the `portproxy` line if it stops working after one).
Then browse to `http://<windows-wifi-ip>:8080/` from any device on the
network (find the Windows WiFi IP with `ipconfig`).
