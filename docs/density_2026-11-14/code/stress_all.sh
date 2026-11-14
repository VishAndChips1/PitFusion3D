#!/bin/bash
# Stress phases at window 0.5 s, cap 80000, on the NEW streamer + page logic. Stops early if a safety gate trips.
source ~/density_deep_dive_20261114/code/env.sh
D=~/density_deep_dive_20261114; CODE=$D/code; LOGS=$D/data/pipeline_logs; mkdir -p $LOGS
systemctl --user stop pitfusion3d.service; sleep 6
python3 ~/ros2_ws/src/pitfusion3d_bringup/scripts/clean_stale_shm.py >/dev/null 2>&1
DENS_WINDOW=0.5 setsid bash -c "exec ros2 launch $CODE/pitfusion3d_density.launch.py" > $LOGS/launch_stress.log 2>&1 &
PIPE=$!
restore() { kill -TERM -- -$PIPE 2>/dev/null; sleep 10; systemctl --user start pitfusion3d.service; }
trap restore EXIT
sleep 50
check() { if grep -q '"aborted": "' $D/data/results_$1.jsonl 2>/dev/null; then echo "ABORTED in $1" > $D/data/stress_aborted.flag; exit 1; fi; }
run() { tag=$1; shift; python3 -u $CODE/density_campaign.py "$@" > $D/data/driver_$tag.log 2>&1; check $tag; }
run stress_multi multi stress_multi 80000 3 120
run stress_link link stress_link 80000 20,10,5,3,2,1 75
ADAPTIVE=0 run stress_link_old link stress_link_old 80000 2,1 75
run stress_soak soak stress_soak 80000 20
echo done > $D/data/stress_done.flag
