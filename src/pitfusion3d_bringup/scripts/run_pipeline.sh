#!/bin/bash
# Launches the full PitFusion3D sensor pipeline with the environment it
# needs. Invoked by the pitfusion3d.service systemd --user unit at boot
# (via `loginctl enable-linger`, so no login/keyboard/mouse is needed --
# see that unit file and this package's README, "Auto-start on boot").
#
# Deliberately does NOT background itself (no setsid/disown/&) -- this
# script's own process must stay in the foreground so systemd can track
# it as the service's main PID and manage its whole process group
# (cleanly killing every node it spawns on stop/restart, and noticing if
# it exits so Restart=on-failure can act).
set -e

# Every node in this pipeline runs on this one machine -- DDS discovery
# has no business depending on wlan0's subnet (the environment default,
# ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET, makes it do exactly that, which
# breaks communication if wlan0 ever changes network/mode -- see
# real_sensors.launch.py and the hotspot scripts for the full story).
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

source /opt/ros/jazzy/setup.bash
source /home/vish/ros2_ws/install/setup.bash

exec ros2 launch pitfusion3d_bringup pitfusion3d_full.launch.py
