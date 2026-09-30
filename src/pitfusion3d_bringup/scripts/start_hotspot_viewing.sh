#!/bin/bash
# Switch this Pi's WiFi to the standalone PitFusion3D-AP hotspot and
# (re)start the sensor pipeline fresh on top of it.
#
# This DROPS THE PI'S INTERNET ACCESS -- wlan0 becomes a standalone
# access point with no uplink, and eth0 (used for the Livox) has no
# default gateway configured. Anything on this machine that needs the
# internet (this Claude Code session included) will stop working until
# you run stop_hotspot_viewing.sh.
#
# A restart is required, not optional: ROS2's discovery (DDS/Fast-RTPS)
# broadcasts on every network interface, including WiFi, even for
# communication between processes on this same machine. Switching wlan0
# from client to AP mode is a full driver-level mode change, not just an
# IP change -- nodes that were already running and had already
# discovered each other over wlan0 go silent (data keeps flowing to
# their own topics, but the web viewer's subscriptions can stop
# receiving anything), which is exactly the "loads but blank" symptom
# this fixes. Restarting after the switch makes every node discover
# everyone else fresh, under the network state that's actually current.
#
# Usage: ~/ros2_ws/src/pitfusion3d_bringup/scripts/start_hotspot_viewing.sh
set -e

echo "== Stopping any running pipeline =="
PIDS=$(ps aux | grep -E "rpicam-vid|thermal_camera_node|rgb_camera_node|livox_ros2_avia_node|calibration_publisher.py|sensor_fusion_node.py|registration_merge_node.py|fusion_web_streamer.py|livox_restamp_node" | grep -v grep | awk '{print $2}')
if [ -n "$PIDS" ]; then
  kill -9 $PIDS 2>/dev/null || true
  sleep 2
fi

echo "== Switching wlan0 to the PitFusion3D hotspot (internet will drop now) =="
nmcli connection up PitFusion3D-AP

echo "== Waiting for the interface to settle =="
sleep 3

echo "== Starting the pipeline fresh =="
# Root-caused: ROS2 discovery defaults to ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET,
# which makes DDS depend on wlan0's current subnet -- fine while wlan0 is a
# normal client, broken the moment it becomes this standalone AP (a
# different subnet entirely). Every node here runs on this one machine, so
# force discovery to localhost instead; confirmed this must be exported
# before `ros2 launch` starts, not set via the launch file's own
# SetEnvironmentVariable action (that applied inconsistently across nested
# launch includes when tested).
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
source /home/vish/ros2_ws/install/setup.bash
setsid ros2 launch pitfusion3d_bringup pitfusion3d_full.launch.py > /home/vish/pitfusion3d_pipeline.log 2>&1 &
disown

echo "== Waiting for nodes to come up =="
sleep 12

echo "== Checking the page loads =="
if curl -sS -m 3 -o /dev/null -w "%{http_code}" http://10.42.0.1:8080/ | grep -q 200; then
  echo "OK: viewer page reachable at http://10.42.0.1:8080/"
else
  echo "FAIL: viewer page did not respond at all -- check /home/vish/pitfusion3d_pipeline.log"
  echo "(the page loading is not enough on its own -- it'll load even with zero data behind it)"
fi

echo "== Checking for REAL DATA (not just the page) -- retrying for up to 15s =="
DATA_OK=0
for i in 1 2 3 4 5; do
  N=$(curl -sS -m 3 http://10.42.0.1:8080/merged.bin 2>/dev/null | python3 -c "
import sys, struct
buf = sys.stdin.buffer.read()
print(struct.unpack_from('<I', buf, 0)[0] if len(buf) >= 4 else 0)
" 2>/dev/null || echo 0)
  if [ "$N" -gt 0 ] 2>/dev/null; then
    echo "OK: merged.bin has $N real points -- data is flowing correctly on the hotspot."
    DATA_OK=1
    break
  fi
  sleep 3
done
if [ "$DATA_OK" -eq 0 ]; then
  echo "FAIL: merged.bin still has 0 points after 15s of retries."
  echo "Do NOT bother connecting your laptop yet -- it'll show the same blank screen you saw before."
  echo "Report this back so it can be debugged further; check /home/vish/pitfusion3d_pipeline.log"
  echo "for errors in the meantime."
fi

echo
if [ "$DATA_OK" -eq 1 ]; then
  echo "Connect your laptop to WiFi 'PitFusion3D' (password: pitfusion123),"
  echo "then open http://10.42.0.1:8080/ -- it should show real, updating data."
fi
echo
echo "When done: run stop_hotspot_viewing.sh to restore Patelnet + internet."
