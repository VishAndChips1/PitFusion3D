#!/bin/bash
# Give the camera back to the pipeline.
N=$(ps -eo pid,args | awk '/lib\/pitfusion3d_bringup\/thermal_camera_node/ && !/awk/ {print $1; exit}')
[ -n "$N" ] && kill -CONT $N && echo "resumed thermal node $N"
