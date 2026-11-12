#!/usr/bin/env python3
"""Wire-format accuracy and compression test on REAL merged/fused clouds. JSON on last line."""
import sys, os, time, json, gzip, struct, numpy as np, rclpy
SC = '/home/vish/ros2_ws/src/landfill_camera_sim/scripts'; sys.path.insert(0, SC)
import cloud_io, importlib.util
spec = importlib.util.spec_from_file_location('fws', f'{SC}/fusion_web_streamer.py'); fws = importlib.util.module_from_spec(spec); spec.loader.exec_module(fws)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2
rclpy.init(); n = Node('accuracy'); got = {}
for t in ('/landfill/fusion/points_merged', '/landfill/fusion/points_rgbt'):
    n.create_subscription(PointCloud2, t, lambda m, t=t: got.setdefault(t, m), Q)
t0 = time.time()
while time.time() - t0 < 25 and len(got) < 2: rclpy.spin_once(n, timeout_sec=0.1)
R = {}; rng = np.random.default_rng(1)
def st(x): x = np.asarray(x, float); return dict(mean=float(x.mean()), rms=float(np.sqrt((x ** 2).mean())), max=float(x.max()), p99=float(np.percentile(x, 99)))
for topic, m in got.items():
    key = topic.split('/')[-1]; a = cloud_io.read_fields(m, ['x', 'y', 'z', 'rgb', 'thermal']); N = len(a)
    t1 = time.perf_counter(); cnt, payload = fws._encode_cloud(m, 10 ** 7, rng); enc_ms = (time.perf_counter() - t1) * 1000     # cap above N: no subsampling, order preserved
    mins = np.frombuffer(payload[:12], '<f4'); scale = float(np.frombuffer(payload[12:16], '<f4')[0]); dt = np.dtype([('xyz', '<i2', 3), ('rgb', 'u1', 3), ('t', '<i2')]); body = np.frombuffer(payload[16:], dt, count=cnt)
    xyz = body['xyz'].astype(np.float64) * scale + mins; err = np.linalg.norm(xyz - a[:, :3].astype(np.float64), axis=1) * 1000
    packed = a[:, 3].view(np.uint32); rgb_ref = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], 1).astype(np.uint8)
    th = a[:, 4]; fin = np.isfinite(th); terr = np.abs(body['t'][fin].astype(np.float64) / 100.0 - th[fin]) if fin.any() else np.array([0.0])
    ext = (a[:, :3].max(0) - a[:, :3].min(0)); raw = payload; sizes = {}
    for lvl in (1, 3, 6, 9):
        t2 = time.perf_counter(); z = gzip.compress(raw, compresslevel=lvl); sizes[f'gzip{lvl}'] = dict(bytes=len(z), ratio=len(raw) / len(z), ms=(time.perf_counter() - t2) * 1000)
    R[key] = dict(points=N, ros_msg_bytes=len(m.data), wire_uncompressed_bytes=len(raw), bytes_per_point=len(raw) / N, ros_to_wire_ratio=len(m.data) / len(raw), encode_ms=enc_ms, extent_m=[float(v) for v in ext], quant_scale_mm=scale * 1000,
                  position_error_mm=st(err), theoretical_max_error_mm=float(np.sqrt(3) * scale / 2 * 1000), rgb_exact_match_frac=float((body['rgb'] == rgb_ref).all(1).mean()), thermal_valid_frac=float(fin.mean()), thermal_abs_error_counts=st(terr), thermal_sentinel_ok=bool((body['t'][~fin] == fws._THERMAL_SENTINEL).all()) if (~fin).any() else True, gzip=sizes)
    for cap in (5000, 20000, 60000):
        if N > cap:
            t3 = time.perf_counter(); c2, p2 = fws._encode_cloud(m, cap, rng); R[key][f'encode_ms_cap{cap}'] = (time.perf_counter() - t3) * 1000; R[key][f'wire_bytes_cap{cap}'] = len(p2)
print(json.dumps(R))
