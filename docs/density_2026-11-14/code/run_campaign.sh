#!/bin/bash
# Start the pipeline with a given merge window (own launch, service stopped), run a campaign phase, always restore the service.
# usage: run_campaign.sh <window_s> <phase> <tag> <args...>
source ~/density_deep_dive_20261114/code/env.sh
CODE=~/density_deep_dive_20261114/code; LOGS=~/density_deep_dive_20261114/data/pipeline_logs; mkdir -p $LOGS
W=$1; shift
start_pipeline() {
  systemctl --user stop pitfusion3d.service; sleep 6
  python3 ~/ros2_ws/src/pitfusion3d_bringup/scripts/clean_stale_shm.py >/dev/null 2>&1
  DENS_WINDOW=$W setsid bash -c "exec ros2 launch $CODE/pitfusion3d_density.launch.py" > $LOGS/launch_w$W.log 2>&1 &
  PIPE=$!
}
stop_pipeline() { [ -n "$PIPE" ] && kill -TERM -- -$PIPE 2>/dev/null; sleep 10; }
restore() { stop_pipeline; systemctl --user start pitfusion3d.service; }
trap restore EXIT
start_pipeline; sleep 50
echo "pipeline up, window $W"; python3 -u $CODE/density_campaign.py "$@"
