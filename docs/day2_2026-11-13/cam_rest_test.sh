#!/bin/bash
# Rest test: leave the camera untouched for REST_MIN minutes (nothing opens it), then try ONE stream and report frames received.
REST=${1:-20}; OUT=data/cam_rest_test.txt
echo "start $(date +%T) rest=${REST}min (pipeline+watchdog stopped)" > $OUT
end=$(( $(date +%s)+REST*60 )); n=0
until [ $(date +%s) -ge $end ]; do sleep 10; done
echo "disconnects during rest: $(journalctl -k --since "-${REST} min" --no-pager | grep -c 'USB disconnect')" >> $OUT
D=$(ls /dev/v4l/by-id/*Thermal*index0 2>/dev/null); echo "dev=$D" >> $OUT
rm -f /tmp/rest.raw; S=$(date +%T)
timeout 25 v4l2-ctl -d $D --set-fmt-video=width=640,height=512,pixelformat=YUYV --set-parm=30 --stream-mmap=4 --stream-count=150 --stream-to=/tmp/rest.raw >> $OUT 2>&1
echo "frames=$(( $(stat -c %s /tmp/rest.raw 2>/dev/null || echo 0)/655360 )) started=$S ended=$(date +%T)" >> $OUT
echo "disconnects in last 40s: $(journalctl -k --since '-40 sec' --no-pager | grep -c 'USB disconnect')" >> $OUT
echo done >> $OUT
