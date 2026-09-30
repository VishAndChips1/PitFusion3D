#!/bin/bash
# Revert wlan0 from the PitFusion3D hotspot back to normal Patelnet/internet
# connectivity, and restart the pipeline so it's healthy on the network
# again (same reasoning as start_hotspot_viewing.sh -- a live interface
# mode change breaks already-running ROS2 nodes' discovery of each other).
#
# Usage: ~/ros2_ws/src/pitfusion3d_bringup/scripts/stop_hotspot_viewing.sh
set -e

echo "== Stopping any running pipeline =="
PIDS=$(ps aux | grep -E "rpicam-vid|thermal_camera_node|rgb_camera_node|livox_ros2_avia_node|calibration_publisher.py|sensor_fusion_node.py|registration_merge_node.py|fusion_web_streamer.py|livox_restamp_node" | grep -v grep | awk '{print $2}')
if [ -n "$PIDS" ]; then
  kill -9 $PIDS 2>/dev/null || true
  sleep 2
fi

echo "== Reconnecting to Patelnet =="
nmcli connection up Patelnet

echo "== Waiting for the interface to settle =="
sleep 3

echo "== Confirming internet is back =="
if curl -sS -m 4 -o /dev/null -w "%{http_code}" https://api.anthropic.com | grep -qE "^[0-9]+$"; then
  echo "OK: internet reachable again."
else
  echo "WARNING: internet check failed -- verify manually before relying on it."
fi

echo "== Starting the pipeline fresh (normal local/localhost viewing) =="
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
source /home/vish/ros2_ws/install/setup.bash
setsid ros2 launch pitfusion3d_bringup pitfusion3d_full.launch.py > /home/vish/pitfusion3d_pipeline.log 2>&1 &
disown

sleep 12
if curl -sS -m 3 -o /dev/null -w "%{http_code}" http://localhost:8080/ | grep -q 200; then
  echo "OK: viewer page reachable at http://localhost:8080/ again."
else
  echo "WARNING: viewer page did not respond -- check /home/vish/pitfusion3d_pipeline.log"
fi
