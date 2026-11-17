#!/bin/bash
# Arm (or disarm) Run 1.
#   ./arm.sh            create the run folder, install and enable thesis-offline.service (it starts at the next power-on)
#   ./arm.sh --disarm   remove the flag and disable the service (nothing will start at boot)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$HOME/thesis_tests"
UNIT="$HOME/.config/systemd/user/thesis-offline.service"
mkdir -p "$ROOT" "$HOME/.config/systemd/user"
if [ "$1" = "--disarm" ]; then
  rm -f "$ROOT/ARMED_1"
  systemctl --user disable thesis-offline.service 2>/dev/null || true
  echo "Disarmed: Run 1 will not start at the next power-on."; exit 0
fi
if [ ! -f "$ROOT/site_notes.env" ]; then
cat > "$ROOT/site_notes.env" <<'NOTES'
# Fill this in at the site BEFORE powering on. Anything left blank is reported as "not supplied" (never guessed).
SITE=
SCENE=
AMBIENT_C=
# tape or laser measured distances from the LiDAR window to each flat surface: label:metres,label:metres
TRUTH=
BANK_PCT_START=
LID_FITTED=yes
NOTES
fi
RUN="$ROOT/$(date +%Y%m%d)_run1"
if [ -d "$RUN" ] && [ -n "$(ls -A "$RUN" 2>/dev/null)" ]; then mv "$RUN" "${RUN}_old_$(date +%H%M%S)"; fi
mkdir -p "$RUN"
echo "$RUN" > "$ROOT/ARMED_1"
rm -f "$ROOT/STOP"
cat > "$UNIT" <<UNITEOF
[Unit]
Description=Thesis Run 1 (offline, hands-off measurement run)
After=pitfusion3d.service
ConditionPathExists=$ROOT/ARMED_1

[Service]
Type=simple
Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash; source $HOME/ros2_ws/install/setup.bash; export FASTRTPS_DEFAULT_PROFILES_FILE=\$(find $HOME/ros2_ws/src -name fastdds_large_buffers.xml | head -1); exec python3 -u $HERE/TestRun1.py --profile full --countdown 120'
Restart=no
StandardOutput=append:$ROOT/run1_console.log
StandardError=append:$ROOT/run1_console.log

[Install]
WantedBy=default.target
UNITEOF
systemctl --user daemon-reload
systemctl --user enable thesis-offline.service
echo "ARMED. Run folder: $RUN"
echo "  Start now without rebooting : systemctl --user start thesis-offline.service"
echo "  Or: power the Pi off, set it up outside, unplug the thermal camera USB, power on (a 2 minute countdown runs first)."
echo "  Stop early                  : touch $ROOT/STOP"
echo "  Disarm                      : $HERE/arm.sh --disarm"
