# PitFusion3D

A ROS 2 (Jazzy) + Gazebo simulation of a multi-sensor rig -- RGB camera,
thermal camera, and Livox Avia-style lidar -- with a calibration layer and
a sensor-fusion pipeline that colorizes the lidar cloud with the camera
data and merges it into a denser, motion-compensated point cloud.

All of the simulation, calibration, and fusion work lives in
[`src/landfill_camera_sim`](src/landfill_camera_sim) -- see that
package's README for the full picture: the two interchangeable scenarios
(a moving platform on a landfill access road, and a stationary rig doing
a slow 360-degree scan of a ring of colorful shapes), the real
Livox Avia scan-pattern data, the editable extrinsic/intrinsic
calibration files, and the fusion/registration nodes.

`src/my_cpp_package` and `src/my_first_package` are empty `ros2 pkg
create` scaffolds (C++ and Python respectively) with no functional code;
they're not part of the sensor-fusion work.

## Quick start

```bash
# from the workspace root
colcon build
source install/setup.bash

# either scenario, with RViz showing RGB, thermal, and lidar
ros2 launch landfill_camera_sim landfill_camera_sim.launch.py
ros2 launch landfill_camera_sim scan_test_sim.launch.py

# fusion pipeline, run alongside whichever scenario is up
ros2 launch landfill_camera_sim sensor_fusion.launch.py
```

Requires ROS 2 Jazzy, Gazebo (`gz-sim`) via `ros_gz_sim`/`ros_gz_bridge`,
and Python's `numpy`, `scipy`, and `opencv`/`cv_bridge` (all standard with
a `ros-jazzy-desktop` install).

## Layout

```
src/
  landfill_camera_sim/   # sensors, worlds, calibration, fusion pipeline (see its README)
  my_cpp_package/        # empty ament_cmake scaffold
  my_first_package/      # empty ament_python scaffold
```
