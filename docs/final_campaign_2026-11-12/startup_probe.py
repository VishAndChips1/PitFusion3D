#!/usr/bin/env python3
"""Subscribe BEFORE the pipeline starts; report wall-clock time of the first message on each topic. Usage: startup_probe.py <seconds> -> JSON"""
import sys, time, json, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2, Image
T = [('/livox/lidar', PointCloud2), ('/landfill/livox/points', PointCloud2), ('/landfill/rgb/image_raw', Image), ('/landfill/thermal/image_raw', Image), ('/landfill/fusion/points_rgbt', PointCloud2), ('/landfill/fusion/points_merged', PointCloud2)]
rclpy.init(); n = Node('startup_probe'); first = {}; cnt = {t: 0 for t, _ in T}
for t, ty in T: n.create_subscription(ty, t, lambda m, t=t: (first.setdefault(t, time.time()), cnt.__setitem__(t, cnt[t] + 1)), Q)
t0 = time.time(); print(json.dumps(dict(started=t0)), flush=True)
while time.time() - t0 < float(sys.argv[1]): rclpy.spin_once(n, timeout_sec=0.05)
print(json.dumps(dict(started=t0, first=first, counts=cnt)))
