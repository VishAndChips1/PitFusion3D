#!/bin/bash
# Start Run 1 (thermal camera may stay plugged in: its presence and stream status are recorded) as a detached background job. No arming, no reboot needed.
#   ./run1.sh              start a new run: 2 minute countdown, then the hotspot-served tests (~1.2 h), reports at the end
#   ./run1.sh --resume     continue the latest unfinished run (after a power loss) without repeating the countdown
#   ./run1.sh --force      start even if a thermal camera USB device is detected (it will be recorded in the report)
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$HOME/thesis_tests"; mkdir -p "$ROOT"
FORCE=0; RESUME=0
for a in "$@"; do [ "$a" = "--force" ] && FORCE=1; [ "$a" = "--resume" ] && RESUME=1; done
if systemctl --user is-active --quiet thesis-run1.service; then echo "Run 1 is already running (touch $ROOT/STOP to stop it early)."; exit 1; fi
if ls /dev/v4l/by-id 2>/dev/null | grep -qi thermal; then echo "Thermal camera USB device detected: it stays plugged in and is recorded in the results."; else echo "No thermal camera USB device detected: recorded as not connected."; fi
if [ ! -f "$ROOT/site_notes.env" ]; then
cat > "$ROOT/site_notes.env" <<'NOTES'
# Optional. Fill in BEFORE starting; the test reads this when it starts. Blank = reported as "not supplied" (never guessed).
SITE=
SCENE=
AMBIENT_C=
# distances measured with a tape/laser from the LiDAR window to each flat surface, any labels: fence:5.02,wall:20.4
TRUTH=
BANK_PCT_START=
LID_FITTED=yes
NOTES
echo "Created $ROOT/site_notes.env (optional: edit it and re-run, or ignore it)."
fi
if [ $RESUME = 1 ]; then
  RUN=$(ls -d "$ROOT"/*_run1 2>/dev/null | while read d; do grep -q '"finalized": true' "$d/state.json" 2>/dev/null || echo "$d"; done | tail -1)
  [ -z "$RUN" ] && { echo "no unfinished run to resume"; exit 1; }
else
  RUN="$ROOT/$(date +%Y%m%d_%H%M%S)_run1"; mkdir -p "$RUN"
fi
rm -f "$ROOT/STOP"
CMD="source /opt/ros/jazzy/setup.bash; source \$HOME/ros2_ws/install/setup.bash; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTRTPS_DEFAULT_PROFILES_FILE=\$(find \$HOME/ros2_ws/src -name fastdds_large_buffers.xml | head -1); exec python3 -u $HERE/TestRun1.py --profile full --countdown 120 --run-dir $RUN"
systemd-run --user --unit=thesis-run1 --collect --property=StandardOutput=append:"$RUN/console.log" --property=StandardError=append:"$RUN/console.log" bash -c "$CMD" >/dev/null 2>&1 || { echo "could not start"; exit 1; }
echo "Run 1 started. Folder: $RUN"
echo "  Countdown 2 min (hotspot PitFusion3D, 10.42.0.1, switched on now), then about 1.2 h of tests, all unattended."
echo "  Progress: cat $RUN/STATUS.json      Finished when: $RUN/DONE.json exists"
echo "  Stop early: touch $ROOT/STOP        After a power loss: $HERE/run1.sh --resume"
echo "  Results: $RUN/{RESULTS_RUN1.md,PAPER_MAP.md,DATA_DICTIONARY.md,thesis_numbers.json,results_bundle.zip}"
