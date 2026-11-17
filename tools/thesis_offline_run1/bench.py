#!/usr/bin/env python3
"""Offline benchmarks for the TestWithLidV1 field test (isolated ROS domain, uses captured real data).

  bench.py <capture.npz> <outdir> [--reps N] [--trials N] [--windowreps N]

Writes: stage_timing.csv, icp_trials.csv, grid_compare.csv, bench_summary.json
Sections follow the thesis plan: 3 (time per stage), 6.3 (merging grid 0.1 vs 0.2 m), 8.2 (merging accuracy vs known motion).
"""
import argparse
import csv
import gzip
import json
import os
import struct
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '77'          # isolated: nothing here can reach the live pipeline
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
sys.path.insert(0, os.path.expanduser('~/ros2_ws/src/landfill_camera_sim/scripts'))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

import cloud_io  # noqa: E402
import fusion_web_streamer as streamer  # noqa: E402
import registration_merge_node as merge  # noqa: E402
import sensor_fusion_node as fusion  # noqa: E402


def timed(fn, reps, warm=2):
    for _ in range(warm):
        fn()
    out = []
    for _ in range(reps):
        t0 = time.perf_counter()
        fn()
        out.append((time.perf_counter() - t0) * 1000.0)
    return out


def summarise(ms):
    a = np.asarray(ms)
    return float(np.median(a)), float(np.percentile(a, 95)), len(a)


def split(z, key):
    xyz, off = z[f'{key}_xyz'], z[f'{key}_offsets']
    return [xyz[off[i]:off[i + 1]] for i in range(len(off) - 1)]


def make_image(arr, encoding, step_bytes):
    msg = Image()
    msg.header = Header()
    msg.height, msg.width, msg.encoding, msg.step = arr.shape[0], arr.shape[1], encoding, step_bytes
    msg.data = arr.tobytes()
    return msg


def rotation(axis, deg):
    axis = axis / np.linalg.norm(axis)
    a = np.radians(deg)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(a) * k + (1 - np.cos(a)) * (k @ k)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('capture')
    ap.add_argument('outdir')
    ap.add_argument('--reps', type=int, default=200)
    ap.add_argument('--trials', type=int, default=25)
    ap.add_argument('--windowreps', type=int, default=8)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    z = np.load(args.capture, allow_pickle=True)
    lidar = [s[np.isfinite(s).all(axis=1)] for s in split(z, 'lidar')]
    fused_xyz, fused_rest = split(z, 'fused'), None
    rest_all, off = z['fused_rest'], z['fused_offsets']
    fused_rest = [rest_all[off[i]:off[i + 1]] for i in range(len(off) - 1)]
    rows, summary = [], {}

    def add(stage, setting, ms):
        med, p95, n = summarise(ms)
        rows.append([stage, setting, f'{med:.3f}', f'{p95:.3f}', n])
        summary[f'{stage}|{setting}'] = {'median_ms': med, 'p95_ms': p95, 'n': n}
        print(f'{stage:40s} {setting:24s} median {med:9.3f} ms  p95 {p95:9.3f} ms  n={n}', flush=True)

    rclpy.init()

    # ---- 3a. time matching (ApproximateTimeSynchronizer, 0.15 s slop), cost per scan with its RGB and thermal message
    import message_filters
    a, b, c = message_filters.SimpleFilter(), message_filters.SimpleFilter(), message_filters.SimpleFilter()
    sync = message_filters.ApproximateTimeSynchronizer([a, b, c], queue_size=5, slop=0.15)
    hits = []
    sync.registerCallback(lambda *m: hits.append(1))

    def stamped(t):
        m = Image()
        m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
        return m
    per_scan = []
    t = 1000.0
    for _ in range(args.reps * 3):
        t += 0.1
        msgs = (stamped(t), stamped(t + 0.012), stamped(t + 0.005))
        t0 = time.perf_counter()
        a.signalMessage(msgs[0])
        b.signalMessage(msgs[1])
        c.signalMessage(msgs[2])
        per_scan.append((time.perf_counter() - t0) * 1000.0)
    add('time_matching', 'per scan (3 streams, slop 0.15 s)', per_scan[10:])
    summary['time_matching_matches'] = len(hits)

    # ---- 3b. fusion node internals with the deployed calibration (a real node object, isolated domain)
    node = fusion.SensorFusionNode()
    scan = lidar[0]
    m, bvec = node._rgb_transform
    ri = node._rgb_intrinsics
    ti = node._thermal_intrinsics
    mt, bt = node._thermal_transform
    add('projection_rgb', f'{len(scan)} points', timed(lambda: fusion.project_to_pixels(
        scan @ m + bvec, ri['fx'], ri['fy'], ri['cx'], ri['cy'], ri['width'], ri['height'], ri['dist']), args.reps))
    add('projection_thermal', f'{len(scan)} points', timed(lambda: fusion.project_to_pixels(
        scan @ mt + bt, ti['fx'], ti['fy'], ti['cx'], ti['cy'], ti['width'], ti['height'], ti['dist']), args.reps))
    rgb_img = z['rgb_img'][0] if 'rgb_img' in z.files else None
    ui, vi, valid = fusion.project_to_pixels(scan @ m + bvec, ri['fx'], ri['fy'], ri['cx'], ri['cy'], ri['width'], ri['height'], ri['dist'])
    summary['rgb_points_in_view_pct'] = float(100.0 * valid.mean())
    if rgb_img is not None:
        def colour():
            s = rgb_img[vi[valid], ui[valid]].astype(np.uint32)
            packed = np.zeros(len(scan), np.uint32)
            packed[valid] = (s[:, 0] << 16) | (s[:, 1] << 8) | s[:, 2]
            return packed
        add('colouring_rgb', f'{len(scan)} points', timed(colour, args.reps))
        arr4 = np.zeros((len(scan), 4), np.float32)
        arr4[:, :3] = scan
        cloud_msg = cloud_io.make_cloud(Header(), cloud_io.XYZI_FIELDS, arr4)
        rgb_msg = make_image(rgb_img.astype(np.uint8), 'rgb8', rgb_img.shape[1] * 3)
        thermal_msg = None
        if 'thermal_img' in z.files:
            th = z['thermal_img'][0].astype(np.uint16)
            thermal_msg = make_image(th, 'mono16', th.shape[1] * 2)
        add('fusion_step_total', 'rgb only (thermal absent)' if thermal_msg is None else 'rgb + thermal',
            timed(lambda: node._process_synced(cloud_msg, rgb_msg, thermal_msg), args.reps))
    else:
        summary['note_no_rgb_frame'] = 'no RGB frame was captured; colouring and total fusion step not timed'
    node.destroy_node()

    # ---- 3c/6.3. ICP per scan at 0.1 m and 0.2 m grids (the node's own functions and caps)
    rng = np.random.default_rng(5)
    grid_stats = {}
    for grid in (0.1, 0.2):
        def one(k, grid=grid):
            A, B = fused_xyz[k % (len(fused_xyz) - 1)], fused_xyz[k % (len(fused_xyz) - 1) + 1]
            tgt = np.asarray(A[merge.voxel_downsample_indices(A, grid)], dtype=np.float64)
            src = np.asarray(B[merge.voxel_downsample_indices(B, grid)], dtype=np.float64)
            if len(src) > 1500:
                src = src[rng.choice(len(src), 1500, replace=False)]
            if len(tgt) > 10000:
                tgt = tgt[rng.choice(len(tgt), 10000, replace=False)]
            return merge.icp_point_to_point(src, tgt, 10, 0.5, 200)
        counter = {'k': 0}

        def step():
            counter['k'] += 1
            one(counter['k'])
        ms = timed(step, args.reps)
        add('icp_registration_per_scan', f'grid {grid} m', ms)
        grid_stats[grid] = {'icp_ms_median': float(np.median(ms))}

    # ---- 3d. merging a window of scans through the real node code (11 scans and the deployed ~0.5 s window)
    mnode = merge.RegistrationMergeNode()
    for grid in (0.1, 0.2):
        mnode._icp_voxel_size = grid
        for count, label in ((len(fused_xyz) if len(fused_xyz) < 11 else 11, '11 scans'), (min(6, len(fused_xyz)), '6 scans (0.5 s window)')):
            buf = [(1000.0 + 0.1 * i, fused_xyz[i].astype(np.float32), fused_rest[i].astype(np.float32)) for i in range(count)]
            ms = timed(lambda: mnode._register_and_publish(list(buf)), args.windowreps, warm=1)
            add('merge_window', f'{label}, grid {grid} m', ms)
    mnode.destroy_node()

    # ---- 3e. streamer encode (pack + gzip) of the real merged cloud at several caps, both wire formats
    if 'merged_xyz' in z.files:
        mxyz, mrest = z['merged_xyz'], z['merged_rest']
        mmsg = cloud_io.make_cloud(Header(), cloud_io.XYZIRT_FIELDS, np.hstack([mxyz, mrest]).astype(np.float32))
        r2 = np.random.default_rng(0)
        for cap in (5000, 20000, 80000):
            def enc1():
                n, p = streamer._encode_cloud(mmsg, cap, r2)
                return gzip.compress(struct.pack('<II', n, 1) + p, compresslevel=6)

            def enc2():
                n, p = streamer._encode_cloud_v2(mmsg, cap, r2)
                return gzip.compress(struct.pack('<II', n, 1) + p, compresslevel=1)
            add('streamer_encode_v1', f'{cap} points', timed(enc1, max(20, args.reps // 4)))
            add('streamer_encode_v2', f'{cap} points', timed(enc2, max(20, args.reps // 4)))
            summary[f'wire_bytes_v1_{cap}'] = len(enc1())
            summary[f'wire_bytes_v2_{cap}'] = len(enc2())
        summary['merged_cloud_points_in_capture'] = int(len(mxyz))

    # ---- 8.2 / 6.3 merging accuracy against known motion (exact ground truth: a rigid transform of a real scan)
    trials = []
    base = fused_xyz[0].astype(np.float64)
    base = base[np.isfinite(base).all(axis=1)]
    t_levels = [0.0, 0.01, 0.02, 0.05, 0.10, 0.20]
    r_levels = [0.0, 0.2, 0.5, 1.0, 2.0]
    for grid in (0.1, 0.2):
        tgt_all = base[merge.voxel_downsample_indices(base, grid)]
        for tl in t_levels:
            for rl in r_levels:
                for trial in range(args.trials):
                    d = rng.normal(size=3)
                    d = d / np.linalg.norm(d) * tl
                    R = rotation(rng.normal(size=3), rl)
                    moved = base @ R.T + d                       # the "new scan": the old one moved by a known amount
                    src = moved[merge.voxel_downsample_indices(moved, grid)]
                    if len(src) > 1500:
                        src = src[rng.choice(len(src), 1500, replace=False)]
                    tgt = tgt_all if len(tgt_all) <= 10000 else tgt_all[rng.choice(len(tgt_all), 10000, replace=False)]
                    t0 = time.perf_counter()
                    Re, te, conv = merge.icp_point_to_point(src, tgt, 10, 0.5, 200)
                    ms = (time.perf_counter() - t0) * 1000.0
                    aligned = moved @ Re.T + te
                    err = float(np.linalg.norm(aligned - base, axis=1).mean() * 1000.0)
                    trials.append([grid, round(tl * 100, 1), rl, trial, int(bool(conv)), f'{err:.3f}', f'{ms:.2f}'])
    with open(os.path.join(args.outdir, 'icp_trials.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['grid_m', 'move_cm', 'rot_deg', 'trial', 'converged', 'err_mm', 'time_ms'])
        w.writerows(trials)
    arr = np.array([[float(x) for x in t] for t in trials])
    for grid in (0.1, 0.2):
        sel = arr[(arr[:, 0] == grid) & (arr[:, 1] <= 5.0) & (arr[:, 2] <= 0.5)]
        grid_stats[grid].update({'icp_err_mm_median_within_envelope': float(np.median(sel[:, 5])), 'icp_err_mm_p95_within_envelope': float(np.percentile(sel[:, 5], 95)),
                                 'success_pct_within_envelope(err<30mm)': float(100.0 * np.mean(sel[:, 5] < 30.0))})
    with open(os.path.join(args.outdir, 'grid_compare.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['grid_m', 'metric', 'value'])
        for g, d in grid_stats.items():
            for k, v in d.items():
                w.writerow([g, k, f'{v:.3f}'])
    with open(os.path.join(args.outdir, 'stage_timing.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['stage', 'setting', 'median_ms', 'p95_ms', 'n'])
        w.writerows(rows)
    summary['grid_compare'] = {str(k): v for k, v in grid_stats.items()}
    with open(os.path.join(args.outdir, 'bench_summary.json'), 'w') as fh:
        json.dump(summary, fh, indent=1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
