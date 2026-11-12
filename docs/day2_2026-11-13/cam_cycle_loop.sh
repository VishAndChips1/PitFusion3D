#!/bin/bash
# Repeat: port power-off -> on -> idle -> ONE stream attempt, N times; log success per cycle. usage: cam_cycle_loop.sh <N> <off_s> <idle_s>
N=${1:-8}; OFF=${2:-8}; IDLE=${3:-30}; DEV=/dev/v4l/by-id/usb-Thermal_Cam_Co._Ltd_Camera_202206223-video-index0
PORT=/sys/devices/platform/axi/1000120000.pcie/1f00200000.usb/xhci-hcd.0/usb2/2-0:1.0/usb2-port2
for c in $(seq 1 $N); do
  echo 1 > $PORT/disable; sleep $OFF; echo 0 > $PORT/disable
  for i in $(seq 1 30); do lsusb | grep -qi 3474:43e2 && break; sleep 1; done; sleep $IDLE
  timeout 25 v4l2-ctl -d $DEV --set-fmt-video=width=640,height=512,pixelformat=YUYV --stream-mmap=4 --stream-count=60 --stream-poll --stream-to=/tmp/claude-1000/cam_loop.yuv >/dev/null 2>&1
  sz=$(stat -c %s /tmp/claude-1000/cam_loop.yuv 2>/dev/null || echo 0); rm -f /tmp/claude-1000/cam_loop.yuv; fr=$(( sz / 655360 ))
  echo "$(date +%T) cycle $c/$N off=${OFF}s idle=${IDLE}s -> frames=$fr $([ $fr -gt 0 ] && echo SUCCESS || echo stuck)"
  [ $fr -gt 0 ] && { sleep 5; break; }
done
