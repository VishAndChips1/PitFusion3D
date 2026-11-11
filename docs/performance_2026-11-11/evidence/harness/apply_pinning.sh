#!/bin/bash
# usage: apply_pinning.sh <profile>   profiles: none | roles
pin() { for pid in $(pgrep -f "$1"); do taskset -a -cp "$2" "$pid" >/dev/null 2>&1; done; }
case "$1" in
 none)  for pat in livox_ros2_avia_node livox_restamp_node calibration_publisher rgb_camera_node rpicam-vid thermal_camera_node sensor_fusion_node registration_merge_node fusion_web_streamer; do pin "$pat" 0-3; done ;;
 roles) pin livox_ros2_avia_node 0; pin livox_restamp_node 0; pin calibration_publisher 0
        pin rgb_camera_node 1; pin rpicam-vid 1; pin thermal_camera_node 1
        pin sensor_fusion_node 2; pin fusion_web_streamer 2
        pin registration_merge_node 3 ;;
esac
for pat in livox_ros2_avia_node rgb_camera_node sensor_fusion_node registration_merge_node fusion_web_streamer; do
  pid=$(pgrep -f "$pat" | head -1); echo -n "$pat: "; taskset -cp $pid | sed 's/.*: //'; done
