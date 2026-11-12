#!/bin/bash
# usage: renice_others.sh <niceness>  -- set niceness of every pipeline process EXCEPT the LiDAR driver (its receive thread must win the CPU)
for p in $(pgrep -f "lib/(pitfusion3d_bringup|landfill_camera_sim)/|rpicam-vid"); do renice -n $1 -p $p >/dev/null 2>&1; for t in /proc/$p/task/*; do renice -n $1 -p $(basename $t) >/dev/null 2>&1; done; done
