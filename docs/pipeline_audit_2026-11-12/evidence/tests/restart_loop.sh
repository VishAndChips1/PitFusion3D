#!/bin/bash
source /opt/ros/jazzy/setup.bash; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
for i in 1 2 3 4 5; do
  systemctl --user restart pitfusion3d.service; sleep 28
  python3 - <<PY
import rclpy,time,urllib.request,struct,gzip
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as q
from sensor_msgs.msg import Image,PointCloud2
rclpy.init();n=Node('rl$i');c={'t':0,'r':0,'l':0}
n.create_subscription(Image,'/landfill/thermal/image_raw',lambda m:c.__setitem__('t',c['t']+1),q)
n.create_subscription(Image,'/landfill/rgb/image_raw',lambda m:c.__setitem__('r',c['r']+1),q)
t=time.time()
while time.time()-t<6: rclpy.spin_once(n,timeout_sec=0.05)
try: code=urllib.request.urlopen('http://localhost:8080/',timeout=3).status
except Exception as e: code=str(e)[:30]
print('restart $i: thermal %.1f Hz | rgb %.1f Hz | viewer http %s'%(c['t']/6,c['r']/6,code))
PY
  echo "      camera kernel events since restart: $(journalctl -k --no-pager --since '-40 sec' | grep -cE 'uvcvideo|3474|Found UVC')"
done
