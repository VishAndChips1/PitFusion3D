#!/usr/bin/env python3
"""LiDAR packet/scan loss probe: UDP counters + scans per second (from device stamps) over N seconds. JSON on last line."""
import sys, time, json, numpy as np, rclpy, subprocess
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2
secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20
def udp():
    u = [l.split() for l in open('/proc/net/snmp') if l.startswith('Udp:')][1]; return int(u[1]), int(u[5])
rclpy.init(); n = Node('lidar_loss'); st = []; arr = []
n.create_subscription(PointCloud2, '/livox/lidar', lambda m: (st.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), arr.append(time.time())), Q)
u0 = udp(); t0 = time.time()
while time.time() - t0 < secs: rclpy.spin_once(n, timeout_sec=0.05)
u1 = udp(); el = time.time() - t0; st = np.array(st); d = np.diff(st) * 1000
nominal = 100.0; span = st[-1] - st[0] if len(st) > 1 else 0; expected = span / (nominal / 1000) + 1 if span else 0
print(json.dumps(dict(secs=el, scans_received=len(st), rx_rate_hz=len(st) / el, expected_scans_from_stamp_span=expected, scans_missing_pct=100 * (1 - len(st) / expected) if expected else None,
    stamp_gap_over_150ms=int((d > 150).sum()), stamp_gap_max_ms=float(d.max()) if len(d) else None, udp_datagrams=u1[0] - u0[0], udp_rcvbuf_errors=u1[1] - u0[1],
    udp_drop_pct=100 * (u1[1] - u0[1]) / max(u1[0] - u0[0] + u1[1] - u0[1], 1))))
