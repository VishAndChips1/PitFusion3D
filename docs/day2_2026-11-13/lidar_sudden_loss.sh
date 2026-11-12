#!/bin/bash
# What does the LiDAR do when its host disappears without any goodbye (= sudden Pi power loss)?
C=/home/vish/lidar_ctl; CODE=3JEDLAJ001G0651; R=data/lidar_sudden_loss
echo "[$(date +%T)] 1. wake to Normal"; $C/lidar_ctl normal --code $CODE --timeout 60 2>/dev/null | grep '^{'
echo "[$(date +%T)] 2. start streaming and keep a live host connection for 20 s"
$C/lidar_ctl sampling-on --code $CODE --timeout 40 --watch 60 >/dev/null 2>&1 & PID=$!
sleep 20; echo "[$(date +%T)] 3. SIGKILL the host (no disconnect, no more heartbeats)"; kill -9 $PID; T0=$(date +%s)
python3 $C/udp_sniff.py 240 > $R.json 2>/dev/null
echo "[$(date +%T)] 4. sniffer result: $(cat $R.json)"
echo "[$(date +%T)] 5. reconnect and read the state"; $C/lidar_ctl status --code $CODE --timeout 60 2>/dev/null | grep '^{'
