"""Save the xyz of one full (uncapped) cloud from a ROS topic. Usage: grab_full.py <topic> <out.npy>"""
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2
sys.path.insert(0, "/home/vish/ros2_ws/src/landfill_camera_sim/scripts")
import cloud_io
rclpy.init(); n = Node('grab_full'); box = {}
n.create_subscription(PointCloud2, sys.argv[1], lambda m: box.setdefault('m', m), Q)
t0 = time.time()
while time.time() - t0 < 15 and 'm' not in box: rclpy.spin_once(n, timeout_sec=0.2)
if 'm' in box:
    a = cloud_io.read_fields(box['m'], ('x', 'y', 'z'), dtype=np.float32); a = a[np.isfinite(a).all(axis=1)]
    np.save(sys.argv[2], np.ascontiguousarray(a))
rclpy.shutdown()
