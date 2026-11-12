#!/bin/bash
# Controlled thermal-camera recovery experiment. usage: cam_cycle_test.sh <off_seconds> <idle_seconds>
OFF=${1:-10}; IDLE=${2:-60}; DEV=/dev/v4l/by-id/usb-Thermal_Cam_Co._Ltd_Camera_202206223-video-index0
PORT=/sys/devices/platform/axi/1000120000.pcie/1f00200000.usb/xhci-hcd.0/usb2/2-0:1.0/usb2-port2
t() { date +%T; }
present() { lsusb | grep -qi "3474:43e2" && echo yes || echo no; }
stream() { timeout 20 v4l2-ctl -d $DEV --set-fmt-video=width=640,height=512,pixelformat=YUYV --stream-mmap=4 --stream-count=${1:-60} --stream-poll --stream-to=/tmp/claude-1000/cam_test.yuv >/dev/null 2>&1; rc=$?; sz=$(stat -c %s /tmp/claude-1000/cam_test.yuv 2>/dev/null || echo 0); echo "frames=$(( sz / 655360 )) rc=$rc"; rm -f /tmp/claude-1000/cam_test.yuv; }
echo "[$(t)] off=${OFF}s idle=${IDLE}s | camera present before: $(present)"
echo "[$(t)] writable disable knob: $([ -w $PORT/disable ] && echo yes || echo NO)"
echo -n "[$(t)] stream attempt BEFORE power-cycle: "; stream 30
echo "[$(t)] port power OFF"; echo 1 > $PORT/disable; sleep 3; echo "[$(t)] camera present while port off: $(present)"; sleep $((OFF-3))
echo "[$(t)] port power ON"; echo 0 > $PORT/disable; for i in $(seq 1 30); do [ "$(present)" = yes ] && break; sleep 1; done; echo "[$(t)] camera re-enumerated after ${i}s: $(present)"
echo "[$(t)] leaving the camera completely alone for ${IDLE}s"; sleep $IDLE
echo -n "[$(t)] FIRST stream attempt after idle: "; stream 90
echo -n "[$(t)] second attempt right after: "; stream 90
