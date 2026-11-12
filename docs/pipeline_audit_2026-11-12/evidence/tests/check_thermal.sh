#!/bin/bash
# Usage: ~/check_thermal.sh [minutes]     (default 1 minute; e.g. ~/check_thermal.sh 30 for a soak)
MIN=${1:-1}
source /opt/ros/jazzy/setup.bash; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
echo "Checking thermal camera for $MIN minute(s) -- do not unplug it or kill anything meanwhile."
if ! lsusb | grep -qi "3474:43e2"; then echo; echo "RESULT: FAIL - camera is not detected on USB at all (cable/port/power)."; exit 1; fi
python3 - "$MIN" <<'PY'
import sys,time,subprocess,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as q
from sensor_msgs.msg import Image
mins=float(sys.argv[1]);rclpy.init();n=Node('check_thermal');ts=[];sums=[]
def cb(m):
    ts.append(time.monotonic());sums.append(hash(bytes(m.data[::997])))
n.create_subscription(Image,'/landfill/thermal/image_raw',cb,q)
def camdrops():
    o=subprocess.run(['journalctl','-k','--no-pager','--since',f'-{int(mins*60+60)} sec'],capture_output=True,text=True).stdout
    return sum(1 for l in o.splitlines() if 'Failed to resubmit video URB' in l or ('3474' in l and 'New USB device found' in l))
d0=camdrops();t0=time.time();last=0;row=[]
while time.time()-t0<mins*60:
    rclpy.spin_once(n,timeout_sec=0.1)
    if time.time()-last>=10:
        last=time.time();row.append(len(ts));print(f'  {time.strftime("%H:%M:%S")}  frames so far: {len(ts)}',flush=True)
a=np.array(ts);dur=time.time()-t0
hz=len(a)/dur if dur else 0;gap=float(np.diff(a).max()) if len(a)>1 else dur
uniq=len(set(sums))/max(len(sums),1);resets=camdrops()-d0
print(f'\nframes={len(a)}  average={hz:.1f} Hz  longest gap={gap:.1f}s  frames-all-different={uniq*100:.0f}%  camera resets={resets}')
if len(a)==0: v='FAIL - camera is detected but sends NO frames (the stuck state; needs a full power-off)'
elif resets>0 or gap>2.0: v='PROBLEM - frames arrive but the camera dropped out/stalled (unreliable)'
elif hz<15: v=f'PROBLEM - only {hz:.1f} Hz (expected about 25-30)'
elif uniq<0.9: v='PROBLEM - frames are repeating (frozen image)'
else: v=f'PASS - working exactly as expected ({hz:.0f} Hz, no gaps, no resets)'
print('RESULT:',v)
PY
