#!/usr/bin/env python3
"""Offline benchmark: baseline vs optimized pipeline kernels.

Runs entirely on synthetic data with no Gazebo and no ROS graph, so the
numbers reflect CPU cost of the processing code rather than whatever
real-time factor the simulator happens to achieve. That makes it
reproducible on both a workstation and the Raspberry Pi 5 target.

    python3 benchmark/bench_offline.py --repeat 20 --json results.json

Baselines are the pre-optimization implementations preserved verbatim
under benchmark/baseline/.
"""

import argparse
import json
import os
import platform
import statistics
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'baseline'))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'scripts'))

import cloud_io  # noqa: E402  (optimized helpers)
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402
from sensor_msgs_py import point_cloud2 as pc2  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

import registration_merge_node_baseline as merge_base  # noqa: E402
import sensor_fusion_node_baseline as fusion_base  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'scripts'))
import registration_merge_node as merge_opt  # noqa: E402
import sensor_fusion_node as fusion_opt  # noqa: E402


GRID_H, GRID_V = 200, 120
IMAGE_W, IMAGE_H = 1280, 720
SCAN_POINTS = 24000
WINDOW_SCANS = 11


def timeit(fn, repeat, warmup=2):
    for _ in range(warmup):
        fn()
    samples = []
    for _ in range(repeat):
        start = time.perf_counter()
        fn()
        samples.append((time.perf_counter() - start) * 1000.0)
    return {
        'median_ms': statistics.median(samples),
        'mean_ms': statistics.fmean(samples),
        'min_ms': min(samples),
        'samples': len(samples),
    }


def make_scene(rng, n_points=SCAN_POINTS):
    """A plausible landfill-ish scan: sloped ground plus a few mounds."""
    azimuth = rng.uniform(-0.61, 0.61, n_points)
    elevation = rng.uniform(-0.67, 0.67, n_points)
    base = 12.0 + 8.0 * np.cos(azimuth * 2.0) + 3.0 * np.sin(elevation * 3.0)
    ranges = base + rng.normal(0.0, 0.25, n_points)
    ranges = np.clip(ranges, 0.5, 60.0)
    xyz = np.empty((n_points, 3), dtype=np.float32)
    xyz[:, 0] = ranges * np.cos(elevation) * np.cos(azimuth)
    xyz[:, 1] = ranges * np.cos(elevation) * np.sin(azimuth)
    xyz[:, 2] = ranges * np.sin(elevation)
    return xyz


def make_xyzi_cloud(xyz, stride=32):
    """PointCloud2 with a padded stride, matching gz-sim's 32-byte layout."""
    n = xyz.shape[0]
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = 'camera_platform/camera_link'
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = stride
    msg.row_step = stride * n
    buf = np.zeros((n, stride // 4), dtype=np.float32)
    buf[:, 0:3] = xyz
    buf[:, 3] = np.arange(n) % 255
    msg.data = buf.tobytes()
    msg.is_dense = False
    return msg


def make_xyzirt_cloud(xyz):
    payload = np.empty((xyz.shape[0], 6), dtype=np.float32)
    payload[:, 0:3] = xyz
    payload[:, 3] = 1.0
    payload[:, 4] = 0.0
    payload[:, 5] = 300.0
    return cloud_io.make_cloud(Header(), cloud_io.XYZIRT_FIELDS, payload)


# --------------------------------------------------------------------------
# Stage 1: PointCloud2 decode
# --------------------------------------------------------------------------
def bench_cloud_read(rng, repeat, results):
    msg = make_xyzi_cloud(make_scene(rng))
    fields = ['x', 'y', 'z', 'intensity']
    base = timeit(lambda: pc2.read_points_numpy(msg, field_names=fields), repeat)
    opt = timeit(lambda: cloud_io.read_fields(msg, fields), repeat)
    results['cloud_read'] = {
        'label': 'PointCloud2 decode (24k pts, 32B stride)',
        'baseline': base, 'optimized': opt,
    }


# --------------------------------------------------------------------------
# Stage 2: PointCloud2 encode
# --------------------------------------------------------------------------
def bench_cloud_write(rng, repeat, results):
    xyz = make_scene(rng)
    payload = np.empty((xyz.shape[0], 6), dtype=np.float32)
    payload[:, 0:3] = xyz
    payload[:, 3:6] = 1.0
    header = Header()
    fields = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate(cloud_io.XYZIRT_FIELDS)
    ]
    base = timeit(lambda: pc2.create_cloud(header, fields, payload), repeat)
    opt = timeit(lambda: cloud_io.make_cloud(header, cloud_io.XYZIRT_FIELDS, payload), repeat)
    results['cloud_write'] = {
        'label': 'PointCloud2 encode (24k pts, 6 fields)',
        'baseline': base, 'optimized': opt,
    }


# --------------------------------------------------------------------------
# Stage 3: voxel downsample
# --------------------------------------------------------------------------
def bench_voxel(rng, repeat, results):
    xyz = np.concatenate([make_scene(rng) for _ in range(WINDOW_SCANS)], axis=0)
    base = timeit(lambda: merge_base._voxel_downsample_indices(xyz, 0.01), repeat)
    opt = timeit(lambda: merge_opt.voxel_downsample_indices(xyz, 0.01), repeat)
    a = np.sort(merge_base._voxel_downsample_indices(xyz, 0.01))
    b = np.sort(merge_opt.voxel_downsample_indices(xyz, 0.01))
    results['voxel_downsample'] = {
        'label': 'Voxel downsample (%d pts, 1cm)' % xyz.shape[0],
        'baseline': base, 'optimized': opt,
        'kept_baseline': int(a.size), 'kept_optimized': int(b.size),
        'identical_selection': bool(a.size == b.size and np.array_equal(a, b)),
    }


# --------------------------------------------------------------------------
# Stage 4: RGB + thermal fusion hot path
# --------------------------------------------------------------------------
def bench_fusion(rng, repeat, results):
    xyz32 = make_scene(rng)
    rgb_img = rng.integers(0, 255, (IMAGE_H, IMAGE_W, 3), dtype=np.uint8)
    thermal_raw = rng.integers(0, 40000, (IMAGE_H, IMAGE_W), dtype=np.uint16)
    intr = {'fx': 1108.5, 'fy': 1108.5, 'cx': 640.0, 'cy': 360.0,
            'width': IMAGE_W, 'height': IMAGE_H}
    extr = {}
    min_temp, resolution = 253.15, 0.01

    def baseline():
        # Mirrors the original _on_synced: float64 points, full-image
        # thermal conversion, per-camera transform lookups, np.errstate
        # divide with nan_to_num.
        xyz = xyz32.astype(np.float64)
        thermal_k = thermal_raw.astype(np.float32) * resolution + min_temp
        p_rgb = fusion_base._lidar_point_to_camera_frame(
            xyz, 'ref', extr, 'lidar', 'rgb')
        ui, vi, valid_rgb = fusion_base._project_to_pixels(
            p_rgb, intr['fx'], intr['fy'], intr['cx'], intr['cy'],
            intr['width'], intr['height'])
        p_th = fusion_base._lidar_point_to_camera_frame(
            xyz, 'ref', extr, 'lidar', 'thermal')
        ti, tj, valid_th = fusion_base._project_to_pixels(
            p_th, intr['fx'], intr['fy'], intr['cx'], intr['cy'],
            intr['width'], intr['height'])
        n = xyz.shape[0]
        colors = np.zeros((n, 3), dtype=np.uint8)
        colors[valid_rgb] = rgb_img[vi[valid_rgb], ui[valid_rgb]]
        packed = ((colors[:, 0].astype(np.uint32) << 16)
                  | (colors[:, 1].astype(np.uint32) << 8)
                  | colors[:, 2].astype(np.uint32)).view(np.float32)
        tv = np.full(n, np.nan, dtype=np.float32)
        tv[valid_th] = thermal_k[tj[valid_th], ti[valid_th]]
        out = np.empty((n, 6), dtype=np.float32)
        out[:, 0:3] = xyz.astype(np.float32)
        out[:, 3] = 1.0
        out[:, 4] = packed
        out[:, 5] = tv
        return out

    m, b = np.eye(3, dtype=np.float32), np.zeros(3, dtype=np.float32)

    def optimized():
        n = xyz32.shape[0]
        ui, vi, valid_rgb = fusion_opt.project_to_pixels(
            xyz32 @ m + b, intr['fx'], intr['fy'], intr['cx'], intr['cy'],
            intr['width'], intr['height'])
        # co-located rig: thermal reuses the RGB projection
        ti, tj, valid_th = ui, vi, valid_rgb
        out = np.empty((n, 6), dtype=np.float32)
        out[:, 0:3] = xyz32
        out[:, 3] = 1.0
        packed = np.zeros(n, dtype=np.uint32)
        s = rgb_img[vi[valid_rgb], ui[valid_rgb]].astype(np.uint32)
        packed[valid_rgb] = (s[:, 0] << 16) | (s[:, 1] << 8) | s[:, 2]
        out[:, 4] = packed.view(np.float32)
        tv = np.full(n, np.nan, dtype=np.float32)
        tv[valid_th] = (thermal_raw[tj[valid_th], ti[valid_th]].astype(np.float32)
                        * resolution + min_temp)
        out[:, 5] = tv
        return out

    results['fusion_projection'] = {
        'label': 'RGB+thermal fusion (24k pts, 1280x720)',
        'baseline': timeit(baseline, repeat),
        'optimized': timeit(optimized, repeat),
    }


# --------------------------------------------------------------------------
# Stage 5: windowed ICP merge
# --------------------------------------------------------------------------
def bench_merge(rng, repeat, results):
    scans = []
    for i in range(WINDOW_SCANS):
        xyz = make_scene(rng)
        xyz[:, 0] += 0.03 * i   # small inter-scan platform motion
        xyz[:, 1] += 0.01 * i
        scans.append(xyz)

    icp_voxel, out_voxel = 0.15, 0.01
    max_iter, max_corr, min_corr = 20, 0.5, 200

    def baseline():
        map_xyz = scans[0].astype(np.float64)
        merged = [map_xyz]
        for xyz32 in scans[1:]:
            xyz = xyz32.astype(np.float64)
            tgt_idx = merge_base._voxel_downsample_indices(map_xyz, icp_voxel)
            src_idx = merge_base._voxel_downsample_indices(xyz, icp_voxel)
            r, t, ok = merge_base.icp_point_to_point(
                xyz[src_idx], map_xyz[tgt_idx], max_iter, max_corr, min_corr)
            if not ok:
                r, t = np.eye(3), np.zeros(3)
            aligned = xyz @ r.T + t
            merged.append(aligned)
            map_xyz = np.vstack([map_xyz, aligned])
        all_xyz = np.vstack(merged)
        return all_xyz[merge_base._voxel_downsample_indices(all_xyz, out_voxel)]

    rng_sub = np.random.default_rng(0)

    def subsample(xyz, limit):
        if xyz.shape[0] <= limit:
            return xyz
        return xyz[rng_sub.choice(xyz.shape[0], size=limit, replace=False)]

    def optimized():
        first = scans[0]
        merged = [first]
        map_ds = np.asarray(
            first[merge_opt.voxel_downsample_indices(first, icp_voxel)], dtype=np.float64)
        for xyz32 in scans[1:]:
            src = np.asarray(
                xyz32[merge_opt.voxel_downsample_indices(xyz32, icp_voxel)], dtype=np.float64)
            r, t, ok = merge_opt.icp_point_to_point(
                subsample(src, 3000), subsample(map_ds, 20000),
                max_iter, max_corr, min_corr)
            if not ok:
                r, t = np.eye(3), np.zeros(3)
            merged.append((xyz32 @ r.T.astype(np.float32)) + t.astype(np.float32))
            aligned_ds = src @ r.T + t
            map_ds = np.vstack((map_ds, aligned_ds))
            map_ds = map_ds[merge_opt.voxel_downsample_indices(map_ds, icp_voxel)]
        all_xyz = np.concatenate(merged, axis=0)
        return all_xyz[merge_opt.voxel_downsample_indices(all_xyz, out_voxel)]

    # ICP is the slowest stage; fewer repeats keeps the run bearable.
    merge_repeat = max(3, repeat // 4)
    results['window_merge'] = {
        'label': 'Windowed ICP merge (%d scans x 24k pts)' % WINDOW_SCANS,
        'baseline': timeit(baseline, merge_repeat, warmup=1),
        'optimized': timeit(optimized, merge_repeat, warmup=1),
        'out_points_baseline': int(baseline().shape[0]),
        'out_points_optimized': int(optimized().shape[0]),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repeat', type=int, default=15)
    parser.add_argument('--json', default=os.path.join(HERE, 'results.json'))
    parser.add_argument('--seed', type=int, default=7)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    results = {}

    print('Running offline benchmark (repeat=%d)...\n' % args.repeat)
    for name, fn in [
        ('PointCloud2 decode', bench_cloud_read),
        ('PointCloud2 encode', bench_cloud_write),
        ('Voxel downsample', bench_voxel),
        ('Fusion projection', bench_fusion),
        ('Windowed ICP merge', bench_merge),
    ]:
        print('  %-22s ...' % name, end='', flush=True)
        started = time.perf_counter()
        fn(rng, args.repeat, results)
        print(' done (%.1fs)' % (time.perf_counter() - started))

    meta = {
        'platform': platform.platform(),
        'machine': platform.machine(),
        'processor': platform.processor() or platform.machine(),
        'python': platform.python_version(),
        'numpy': np.__version__,
        'cpu_count': os.cpu_count(),
    }
    payload = {'meta': meta, 'results': results}
    with open(args.json, 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2)

    print('\n%-42s %11s %11s %9s' % ('stage', 'baseline', 'optimized', 'speedup'))
    print('-' * 76)
    total_base = total_opt = 0.0
    for entry in results.values():
        base_ms = entry['baseline']['median_ms']
        opt_ms = entry['optimized']['median_ms']
        total_base += base_ms
        total_opt += opt_ms
        print('%-42s %9.2fms %9.2fms %8.2fx'
              % (entry['label'][:42], base_ms, opt_ms, base_ms / opt_ms))
    print('-' * 76)
    print('%-42s %9.2fms %9.2fms %8.2fx'
          % ('TOTAL', total_base, total_opt, total_base / total_opt))
    print('\n%s | %d cores | numpy %s' % (meta['machine'], meta['cpu_count'], meta['numpy']))
    print('Wrote %s' % args.json)


if __name__ == '__main__':
    main()
