#!/bin/bash
# Hypothesis H3: the camera needs a long quiet period after its own reset before it can stream.
# 1) one failed open triggers the camera's self-reset  2) wait for the reset and then stay completely hands-off for WAIT seconds  3) open once.
WAIT=${1:-120}; DEV=/dev/v4l/by-id/usb-Thermal_Cam_Co._Ltd_Camera_202206223-video-index0
t() { date +%T; }; since=$(date '+%Y-%m-%d %H:%M:%S')
try() { timeout 25 v4l2-ctl -d $DEV --set-fmt-video=width=640,height=512,pixelformat=YUYV --stream-mmap=4 --stream-count=60 --stream-poll --stream-to=/tmp/claude-1000/pt.yuv >/dev/null 2>&1; sz=$(stat -c %s /tmp/claude-1000/pt.yuv 2>/dev/null || echo 0); rm -f /tmp/claude-1000/pt.yuv; echo $(( sz / 655360 )); }
echo "[$(t)] attempt 1 (expected to fail and trigger the self-reset): frames=$(try)"
echo "[$(t)] waiting for the camera's self-reset (USB disconnect)..."
for i in $(seq 1 90); do n=$(journalctl -k --no-pager --since "$since" | grep -c "usb 2-2: USB disconnect"); [ $n -ge 1 ] && break; sleep 1; done
echo "[$(t)] self-reset seen after ~${i}s (disconnect events: $n); waiting for it to re-enumerate"
for i in $(seq 1 40); do lsusb | grep -qi 3474:43e2 && break; sleep 1; done; T0=$(date +%s); echo "[$(t)] re-enumerated; hands-off for ${WAIT}s"
sleep $WAIT
echo "[$(t)] attempt 2 after $(( $(date +%s) - T0 ))s of quiet: frames=$(try)"
