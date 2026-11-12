#!/usr/bin/env python3
"""LiDAR range stability over time (warm-up / drift). Every PERIOD s: collect a 3 s burst of scans, fit the dominant plane (RANSAC) and
track its distance, normal direction and RMS, plus the median range inside a fixed 6x6 degree window straight ahead. Output: data/lidar_stability.csv"""
import sys, time, csv, numpy as np, rclpy
sys.path.insert(0, '/home/vish/ros2_ws/src/landfill_camera_sim/scripts')
import cloud_io
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2
DUR, PERIOD, BURST = float(sys.argv[1]), 30.0, 3.0
rclpy.init(); node = Node('lidar_stability'); buf = []; active = [False]
node.create_subscription(PointCloud2, '/livox/lidar', lambda m: buf.append(m) if active[0] else None, Q)
rng = np.random.default_rng(3)
def soc(): return float(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000
def fit(xyz):
    sub = xyz[rng.choice(len(xyz), min(8000, len(xyz)), replace=False)]; best = (0, None)
    for _ in range(300):
        p = sub[rng.choice(len(sub), 3, replace=False)]; n = np.cross(p[1] - p[0], p[2] - p[0]); l = np.linalg.norm(n)
        if l < 1e-9: continue
        n /= l; c = int((np.abs((sub - p[0]) @ n) < 0.03).sum())
        if c > best[0]: best = (c, (n, p[0]))
    n, p0 = best[1]; inl = sub[np.abs((sub - p0) @ n) < 0.03]; c = inl.mean(0); n = np.linalg.svd(inl - c, full_matrices=False)[2][2]
    res = (xyz - c) @ n; m = np.abs(res) < 0.03
    return abs(float(c @ n)), n, float(np.sqrt((res[m] ** 2).mean()) * 1000), float(m.mean())
f = open('data/lidar_stability.csv', 'w', newline=''); w = csv.writer(f); w.writerow(['t_s', 'wall', 'soc_c', 'scans', 'points', 'plane_dist_m', 'plane_rms_mm', 'plane_inlier_frac', 'normal_x', 'normal_y', 'normal_z', 'centre_range_m', 'centre_points'])
t0 = time.time(); k = 0; ref_n = None
while time.time() - t0 < DUR:
    tn = t0 + k * PERIOD; k += 1
    while time.time() < tn: rclpy.spin_once(node, timeout_sec=0.05)
    buf.clear(); active[0] = True; te = time.time() + BURST
    while time.time() < te: rclpy.spin_once(node, timeout_sec=0.05)
    active[0] = False; msgs = list(buf)
    if len(msgs) < 5: continue
    X = np.concatenate([np.stack([a['x'], a['y'], a['z']], 1) for a in (cloud_io.structured_view(m) for m in msgs)]).astype(np.float64)
    X = X[np.isfinite(X).all(1) & (np.abs(X).sum(1) > 0)]
    d, n, rms, inl = fit(X)
    if ref_n is None: ref_n = n
    if n @ ref_n < 0: n = -n
    az = np.degrees(np.arctan2(X[:, 1], X[:, 0])); el = np.degrees(np.arcsin(X[:, 2] / np.linalg.norm(X, axis=1))); win = (np.abs(az) < 3) & (np.abs(el) < 3)
    cr = float(np.median(np.linalg.norm(X[win], axis=1))) if win.sum() > 20 else float('nan')
    w.writerow([f'{time.time()-t0:.0f}', time.strftime('%H:%M:%S'), f'{soc():.1f}', len(msgs), len(X), f'{d:.4f}', f'{rms:.2f}', f'{inl:.3f}', *[f'{v:.4f}' for v in n], f'{cr:.4f}', int(win.sum())]); f.flush()
