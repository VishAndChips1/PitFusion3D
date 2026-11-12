#!/usr/bin/env python3
"""Subscribe to topics for N seconds; print one JSON line: rate, inter-arrival stats, header-stamp age at arrival,
message size, points/pixels. Optionally save per-message (arrival, stamp) arrays to an .npz."""
import sys, time, json, argparse, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2, Image
ap = argparse.ArgumentParser(); ap.add_argument('--topics', nargs='+', required=True); ap.add_argument('--secs', type=float, default=10); ap.add_argument('--save', default=None)
a = ap.parse_args()
rclpy.init(); n = Node('probe_topics')
TYPES = {}
for name, types in n.get_topic_names_and_types():
    if name in a.topics: TYPES[name] = types[0]
time.sleep(0.5)
for name, types in n.get_topic_names_and_types():
    if name in a.topics: TYPES[name] = types[0]
rec = {t: dict(arr=[], stamp=[], size=[], npts=[], w=0, h=0, enc='') for t in a.topics}
def mk(t):
    def cb(m):
        r = rec[t]; r['arr'].append(time.time()); r['stamp'].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if hasattr(m, 'encoding'): r['size'].append(len(m.data)); r['w'] = m.width; r['h'] = m.height; r['enc'] = m.encoding; r['npts'].append(m.width * m.height)
        else: r['size'].append(len(m.data)); r['npts'].append(m.width * m.height)
    return cb
for t in a.topics:
    ty = PointCloud2 if TYPES.get(t, '').endswith('PointCloud2') or 'points' in t or 'lidar' in t else Image
    n.create_subscription(ty, t, mk(t), Q)
t0 = time.time()
while time.time() - t0 < a.secs: rclpy.spin_once(n, timeout_sec=0.05)
out = {}; sv = {}
for t, r in rec.items():
    arr = np.array(r['arr']); st = np.array(r['stamp']); k = len(arr)
    if k < 2: out[t] = dict(n=k, hz=k / a.secs, note='too few messages'); continue
    d = np.diff(arr); ds = np.diff(st); age = (arr - st) * 1000
    pc = lambda x, p: float(np.percentile(x, p))
    out[t] = dict(n=k, hz=k / a.secs, interval_ms=dict(mean=float(d.mean() * 1e3), std=float(d.std() * 1e3), p50=pc(d, 50) * 1e3, p95=pc(d, 95) * 1e3, p99=pc(d, 99) * 1e3, max=float(d.max() * 1e3)),
                  stamp_interval_ms=dict(mean=float(ds.mean() * 1e3), std=float(ds.std() * 1e3), p50=pc(ds, 50) * 1e3, p95=pc(ds, 95) * 1e3, max=float(ds.max() * 1e3), min=float(ds.min() * 1e3)),
                  stamp_age_ms=dict(mean=float(age.mean()), p50=pc(age, 50), p95=pc(age, 95), max=float(age.max()), min=float(age.min())),
                  gaps_over_2x_nominal=int((d > 2 * np.median(d)).sum()), msg_bytes_mean=float(np.mean(r['size'])), points_or_pixels_mean=float(np.mean(r['npts'])),
                  width=r['w'], height=r['h'], encoding=r['enc'])
    sv[t.strip('/').replace('/', '_') + '_arr'] = arr; sv[t.strip('/').replace('/', '_') + '_stamp'] = st
if a.save: np.savez(a.save, **sv)
print(json.dumps(out))
