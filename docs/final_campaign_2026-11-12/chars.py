#!/usr/bin/env python3
"""Sensor / point-cloud characterisation. Usage: chars.py <lidar|rgb|thermal|fused|merged> <seconds>  -> JSON on last stdout line.
Each mode runs in its own process so a slow subscriber can't make another mode lose messages."""
import sys, os, time, json, numpy as np, rclpy
sys.path.insert(0, '/home/vish/ros2_ws/src/landfill_camera_sim/scripts'); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cloud_io
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2, Image
import cv2
MODE = sys.argv[1]; SECS = float(sys.argv[2]); OUT = os.path.dirname(os.path.abspath(__file__))
rng = np.random.default_rng(7)
def pct(x, ps=(5, 50, 95)): return {f'p{p}': float(np.percentile(x, p)) for p in ps} if len(x) else {}
def stats(x): x = np.asarray(x, float); return dict(mean=float(x.mean()), std=float(x.std()), min=float(x.min()), max=float(x.max()), **pct(x)) if len(x) else {}
def plane_fit(xyz, iters=300, thr=0.03):
    if len(xyz) < 300: return None
    sub = xyz[rng.choice(len(xyz), min(6000, len(xyz)), replace=False)]; best = (0, None)
    for _ in range(iters):
        p = sub[rng.choice(len(sub), 3, replace=False)]; nrm = np.cross(p[1] - p[0], p[2] - p[0]); l = np.linalg.norm(nrm)
        if l < 1e-9: continue
        nrm /= l; d = np.abs((sub - p[0]) @ nrm); c = int((d < thr).sum())
        if c > best[0]: best = (c, (nrm, p[0]))
    if best[1] is None: return None
    nrm, p0 = best[1]; inl = sub[np.abs((sub - p0) @ nrm) < thr]
    c = inl.mean(0); u, s, vt = np.linalg.svd(inl - c, full_matrices=False); nrm = vt[2]
    res = (xyz - c) @ nrm; m = np.abs(res) < thr; r = res[m]
    return dict(inlier_frac=float(m.mean()), rms_mm=float(np.sqrt((r ** 2).mean()) * 1000), std_mm=float(r.std() * 1000), n_inliers=int(m.sum()),
                normal=[float(v) for v in nrm], distance_m=float(abs(c @ nrm)))
rclpy.init(); n = Node('chars_' + MODE); R = {}
def collect(topic, ty, keep_msg=False):
    box = dict(arr=[], stamp=[], msgs=[])
    def cb(m):
        box['arr'].append(time.time()); box['stamp'].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if keep_msg: box['msgs'].append(m)
    n.create_subscription(ty, topic, cb, Q); return box
def spin(secs):
    t0 = time.time()
    while time.time() - t0 < secs: rclpy.spin_once(n, timeout_sec=0.05)

if MODE == 'lidar':
    b = collect('/livox/lidar', PointCloud2, True); spin(SECS); msgs = b['msgs']; R['scans'] = len(msgs); R['rate_hz'] = len(msgs) / SECS
    per = []; planes = []; ranges_all = []; ints = []; az_all = []; el_all = []; lines = {}
    for i, m in enumerate(msgs):
        a = cloud_io.structured_view(m); xyz = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
        ok = np.isfinite(xyz).all(1) & (np.abs(xyz).sum(1) > 0); xyz = xyz[ok]; r = np.linalg.norm(xyz, axis=1)
        per.append(dict(n_msg=int(m.width * m.height), n_valid=int(ok.sum()), mean_range=float(r.mean()) if len(r) else 0, max_range=float(r.max()) if len(r) else 0))
        if i % 3 == 0:
            ranges_all.append(r[::5]); ints.append(a['intensity'][ok][::5]); az_all.append(np.degrees(np.arctan2(xyz[::5, 1], xyz[::5, 0]))); el_all.append(np.degrees(np.arcsin(xyz[::5, 2] / np.maximum(r[::5], 1e-6))))
            for ln in np.unique(a['line']): lines[int(ln)] = lines.get(int(ln), 0) + int((a['line'][ok] == ln).sum())
        if i % 4 == 0 and len(planes) < 25:
            pf = plane_fit(xyz)
            if pf: planes.append(pf)
        if i == len(msgs) // 2: np.savez(f'{OUT}/data/sample_lidar_scan.npz', xyz=xyz.astype(np.float32), intensity=a['intensity'][ok])
    rr = np.concatenate(ranges_all) if ranges_all else np.array([]); az = np.concatenate(az_all) if az_all else np.array([]); el = np.concatenate(el_all) if el_all else np.array([])
    R['points_per_scan'] = stats([p['n_msg'] for p in per]); R['valid_points_per_scan'] = stats([p['n_valid'] for p in per]); R['points_per_second_mean'] = float(np.mean([p['n_msg'] for p in per]) * R['rate_hz']) if per else 0
    R['range_m'] = stats(rr); R['range_hist_m'] = {k: float(((rr >= lo) & (rr < hi)).mean()) for k, (lo, hi) in {'0-1': (0, 1), '1-2': (1, 2), '2-5': (2, 5), '5-10': (5, 10), '10-20': (10, 20), '20+': (20, 1e9)}.items()} if len(rr) else {}
    R['intensity'] = stats(np.concatenate(ints)) if ints else {}; R['azimuth_deg'] = dict(min=float(az.min()), max=float(az.max())) if len(az) else {}; R['elevation_deg'] = dict(min=float(el.min()), max=float(el.max())) if len(el) else {}
    R['points_by_line'] = lines; R['plane_fit'] = dict(n_planes=len(planes), rms_mm=stats([p['rms_mm'] for p in planes]), inlier_frac=stats([p['inlier_frac'] for p in planes]), note='RANSAC largest plane, 3 cm inlier band; rms = LiDAR range precision proxy (includes real surface roughness)') if planes else {}
    R['interarrival_ms'] = stats(np.diff(b['arr']) * 1000) if len(b['arr']) > 2 else {}; R['device_stamp_interval_ms'] = stats(np.diff(b['stamp']) * 1000) if len(b['stamp']) > 2 else {}
    R['fov_note'] = 'Livox Avia nominal FOV 70.4 deg x 77.2 deg, non-repetitive scan, 6 lines'

elif MODE == 'rgb':
    b = collect('/landfill/rgb/image_raw', Image, True); spin(SECS); msgs = b['msgs']; R['frames'] = len(msgs); R['rate_hz'] = len(msgs) / SECS
    lum = []; sharp = []; sat = []; diffs = []; prev = None; chm = []
    for i, m in enumerate(msgs):
        ch = 3 if m.encoding in ('bgr8', 'rgb8') else 1; im = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, ch) if ch == 3 else np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
        g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) if ch == 3 else im
        lum.append(float(g.mean())); sharp.append(float(cv2.Laplacian(g, cv2.CV_64F).var())); sat.append(float(((g < 3) | (g > 252)).mean()))
        if prev is not None and i % 2 == 0: diffs.append(float(np.median(np.abs(g.astype(np.int16) - prev.astype(np.int16)))))
        prev = g
        if ch == 3 and i % 5 == 0: chm.append(im.reshape(-1, 3)[::50].mean(0))
        if i == len(msgs) // 2: cv2.imwrite(f'{OUT}/frames/rgb_sample.jpg', im if m.encoding == 'bgr8' else cv2.cvtColor(im, cv2.COLOR_RGB2BGR) if ch == 3 else im)
    R['resolution'] = [msgs[0].width, msgs[0].height] if msgs else None; R['encoding'] = msgs[0].encoding if msgs else None; R['bytes_per_frame'] = len(msgs[0].data) if msgs else None
    R['mean_luma'] = stats(lum); R['sharpness_laplacian_var'] = stats(sharp); R['clipped_pixel_frac'] = stats(sat); R['frame_diff_median_abs'] = stats(diffs); R['channel_means_bgr_or_rgb'] = np.mean(chm, 0).tolist() if chm else None
    R['interarrival_ms'] = stats(np.diff(b['arr']) * 1000) if len(b['arr']) > 2 else {}; R['stamp_interval_ms'] = stats(np.diff(b['stamp']) * 1000) if len(b['stamp']) > 2 else {}; R['stamp_age_at_arrival_ms'] = stats((np.array(b['arr']) - np.array(b['stamp'])) * 1000) if b['arr'] else {}

elif MODE == 'thermal':
    b = collect('/landfill/thermal/image_raw', Image, True); spin(SECS); msgs = b['msgs']; R['frames'] = len(msgs); R['rate_hz'] = len(msgs) / SECS
    if msgs:
        A = np.stack([np.frombuffer(m.data, np.uint16).reshape(m.height, m.width) >> 8 for m in msgs[:300]]).astype(np.float32)
        R['resolution'] = [msgs[0].width, msgs[0].height]; R['encoding'] = msgs[0].encoding; R['dynamic_range_8bit'] = dict(min=float(A.min()), max=float(A.max()), mean=float(A.mean()), std_spatial=float(A.std((1, 2)).mean()))
        tn = A.std(0); R['temporal_std_per_pixel'] = dict(p10=float(np.percentile(tn, 10)), median=float(np.median(tn)), p90=float(np.percentile(tn, 90)), note='includes scene motion; p10 approximates sensor noise floor (8-bit AGC counts)')
        R['stuck_pixel_frac'] = float(((A.max(0) == A.min(0)) & ((A.max(0) == 0) | (A.max(0) == 255))).mean()); R['unique_frame_frac'] = len({bytes(m.data[::997]) for m in msgs}) / len(msgs)
        cv2.imwrite(f'{OUT}/frames/thermal_sample.png', cv2.applyColorMap(A[len(A) // 2].astype(np.uint8), cv2.COLORMAP_INFERNO))
        R['interarrival_ms'] = stats(np.diff(b['arr']) * 1000); R['stamp_age_at_arrival_ms'] = stats((np.array(b['arr']) - np.array(b['stamp'])) * 1000)
    else: R['note'] = 'NO FRAMES RECEIVED (camera not streaming during this window)'

elif MODE == 'fused':
    bl = collect('/landfill/livox/points', PointCloud2); br = collect('/landfill/rgb/image_raw', Image); bt = collect('/landfill/thermal/image_raw', Image); bf = collect('/landfill/fusion/points_rgbt', PointCloud2, True)
    spin(SECS); msgs = bf['msgs']; R['fused_clouds'] = len(msgs); R['fused_rate_hz'] = len(msgs) / SECS; R['lidar_in_rate_hz'] = len(bl['arr']) / SECS; R['rgb_rate_hz'] = len(br['arr']) / SECS; R['thermal_rate_hz'] = len(bt['arr']) / SECS
    R['fused_per_lidar_scan'] = len(msgs) / max(len(bl['arr']), 1)
    fs = np.array(bf['stamp']); rs = np.array(br['stamp']); ts = np.array(bt['stamp'])
    if len(fs) and len(rs): R['dt_fused_to_nearest_rgb_ms'] = stats(np.array([np.min(np.abs(rs - s)) for s in fs]) * 1000)
    if len(fs) and len(ts): R['dt_fused_to_nearest_thermal_ms'] = stats(np.array([np.min(np.abs(ts - s)) for s in fs]) * 1000)
    vr = []; vt = []; npts = []; both = []; rngs = []
    for i, m in enumerate(msgs):
        a = cloud_io.read_fields(m, ['x', 'y', 'z', 'rgb', 'thermal']); rgb = a[:, 3].view(np.uint32) != 0; th = np.isfinite(a[:, 4])
        vr.append(float(rgb.mean())); vt.append(float(th.mean())); both.append(float((rgb & th).mean())); npts.append(len(a)); rngs.append(float(np.linalg.norm(a[:, :3], axis=1).mean()))
        if i == len(msgs) // 2: np.savez(f'{OUT}/data/sample_fused_cloud.npz', xyz=a[:, :3].astype(np.float32), rgb=a[:, 3].view(np.uint32), thermal=a[:, 4])
    R['points_per_fused_cloud'] = stats(npts); R['valid_rgb_frac'] = stats(vr); R['valid_thermal_frac'] = stats(vt); R['both_valid_frac'] = stats(both); R['mean_range_m'] = stats(rngs)
    R['note'] = 'valid_* = fraction of LiDAR points that project inside that camera image (FOV overlap), not a quality score'

elif MODE == 'merged':
    bm = collect('/landfill/fusion/points_merged', PointCloud2, True); spin(SECS); msgs = bm['msgs']; R['merged_clouds'] = len(msgs); R['merged_rate_hz'] = len(msgs) / SECS
    R['points_per_merged_cloud'] = stats([m.width * m.height for m in msgs]); R['bytes_per_merged_cloud'] = stats([len(m.data) for m in msgs])
    R['interarrival_s'] = stats(np.diff(bm['arr'])) if len(bm['arr']) > 2 else {}; R['stamp_age_at_arrival_s'] = stats(np.array(bm['arr']) - np.array(bm['stamp'])) if msgs else {}
    if len(msgs) >= 2:
        i = len(msgs) // 2; a = cloud_io.read_fields(msgs[i], ['x', 'y', 'z', 'rgb', 'thermal']); np.savez(f'{OUT}/data/sample_merged_cloud.npz', xyz=a[:, :3].astype(np.float32), rgb=a[:, 3].view(np.uint32), thermal=a[:, 4])
        ext = a[:, :3].max(0) - a[:, :3].min(0); R['extent_m'] = [float(v) for v in ext]; R['quantisation_step_mm_at_this_extent'] = float(ext.max() / 32000 * 1000)
        try:
            import registration_merge_node as rm
            tr = []
            for j in range(0, len(msgs) - 1):
                sx = cloud_io.read_fields(msgs[j], ['x', 'y', 'z'])[::6].astype(np.float64); tx = cloud_io.read_fields(msgs[j + 1], ['x', 'y', 'z'])[::6].astype(np.float64)
                Rm, tv, conv = rm.icp_point_to_point(sx, tx, 15, 0.5, 200)
                ang = np.degrees(np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1))); tr.append((float(np.linalg.norm(tv) * 1000), float(ang), bool(conv)))
            if tr: R['consecutive_merged_clouds_icp'] = dict(translation_mm=stats([t[0] for t in tr]), rotation_deg=stats([t[1] for t in tr]), n=len(tr), converged_frac=float(np.mean([t[2] for t in tr])), note='residual motion between consecutive merged clouds; ~0 for a static rig, grows if the platform moves')
        except Exception as e: R['icp_error'] = repr(e)[:200]
print(json.dumps(R))
