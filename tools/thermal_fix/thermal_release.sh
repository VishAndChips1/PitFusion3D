#!/bin/bash
# Free the thermal camera for direct use (e.g. ffplay): GRACEFULLY stop the pipeline's thermal node (SIGINT, so it closes the
# camera cleanly -- a SIGKILL can leave the camera unable to stream until it is power-cycled), then freeze the respawned copy
# before it opens the device.
pid() { ps -eo pid,args | awk '/lib\/pitfusion3d_bringup\/thermal_camera_node/ && !/awk/ {print $1; exit}'; }
OLD=$(pid); [ -n "$OLD" ] && kill -INT $OLD
for i in $(seq 1 100); do N=$(pid); if [ -n "$N" ] && [ "$N" != "$OLD" ]; then kill -STOP $N; echo "frozen new thermal node $N (video fds held: $(ls -l /proc/$N/fd 2>/dev/null | grep -c video))"; exit 0; fi; sleep 0.1; done
echo "new node not found"; exit 1
