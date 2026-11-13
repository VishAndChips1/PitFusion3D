#!/bin/bash
# Lint and unit-test the runtime code:  ./run_checks.sh   (from the src/ directory)
cd "$(dirname "$0")" || exit 1
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null
status=0
python3 -m flake8 landfill_camera_sim/scripts/{sensor_fusion_node,registration_merge_node,fusion_web_streamer,calibration_publisher,cloud_io}.py \
    landfill_camera_sim/launch/{sensor_fusion,web_stream}.launch.py \
    pitfusion3d_bringup/pitfusion3d_bringup/*.py pitfusion3d_bringup/launch/*.py pitfusion3d_bringup/scripts/*.py landfill_camera_sim/test/*.py || status=1
python3 -m pytest -q landfill_camera_sim/test || status=1
[ $status = 0 ] && echo "ALL CHECKS PASSED" || echo "CHECKS FAILED"
exit $status
