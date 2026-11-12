#!/bin/bash
# Run by lidar-power.service (system unit, User=vish):
#   start (boot)     wake the LiDAR to Normal BEFORE the ROS pipeline starts (the driver then finds it awake and streams)
#   stop (shutdown)  put the LiDAR into Standby AFTER the ROS pipeline has released it
# Every action is appended to power_events.log as one JSON line, so each boot/shutdown leaves its own evidence.
HERE=/home/vish/lidar_ctl; LOG=$HERE/power_events.log; CODE=${LIDAR_CODE:-3JEDLAJ001G0651}; SLEEP_MODE=${LIDAR_SLEEP_MODE:-standby}   # standby (0.9 s wake) or low (Power-Saving, 9.7 s wake)
export XDG_RUNTIME_DIR=/run/user/$(id -u) DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus
log() { echo "{\"time\":\"$(date '+%Y-%m-%d %H:%M:%S')\",\"event\":\"$1\",\"detail\":$2}" >> $LOG; }
run() {   # run lidar_ctl <mode> <timeout>; echo its JSON result line
  local t0=$(date +%s.%N); local out; out=$(timeout $(( $2 + 20 )) $HERE/lidar_ctl $1 --code $CODE --timeout $2 2>>$HERE/lidar.log | grep '^{' | tail -n1); local rc=$?
  [ -z "$out" ] && out='{"ok":false,"error":"no result"}'; echo "$out" | sed "s/}\$/,\"elapsed_s\":$(python3 -c "print(round($(date +%s.%N)-$t0,1))")}/"
}
case "$1" in
  start)
    # first look at what state the LiDAR is in (this shows whether the previous shutdown left it in Standby)
    before=$(run status 25); log boot_state_seen "$before"
    res=$(run normal 40); log boot_wake_to_normal "$res" ;;
  stop)
    # free the LiDAR if the pipeline's driver still holds it (at a real shutdown the user manager has normally stopped it already)
    timeout 20 systemctl --user stop pitfusion3d.service >/dev/null 2>&1; pkill -TERM -f livox_ros2_avia_node 2>/dev/null; sleep 1
    res=$(run $SLEEP_MODE 25); log shutdown_sleep "$res" ;;
  *) echo "usage: $0 start|stop"; exit 1 ;;
esac
exit 0
