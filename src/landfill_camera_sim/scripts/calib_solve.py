#!/usr/bin/env python3
"""Solve thermal-camera extrinsics/intrinsics from captured calibration poses.

For each pose .npz (vs. baseline.npz with nobody in frame):
  - lidar: points with no close neighbour in the baseline cloud are "new"
    (the person); their centroid is the person's 3D position in the lidar
    frame (x forward, y left, z up -- same convention sensor_fusion_node.py
    uses).
  - thermal: pixels warmer than baseline by a threshold are "new"; their
    intensity-weighted centroid is the person's pixel position.

Then fits (tx, ty, tz, focal_scale) by nonlinear least squares so that
projecting each pose's lidar centroid through the pinhole model (using
sensor_fusion_node.py's exact convention) lands on that pose's measured
thermal pixel centroid, across all poses at once -- multiple poses at
different depths/offsets are what let translation and focal length be
told apart (both affect the single-pose case similarly).

Usage: calib_solve.py <calib_dir> [--apply]
  --apply writes the solved values into extrinsics.yaml / intrinsics.yaml.
  Without it, only prints the recommended values and the fit residuals.
"""
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.ndimage import label

CX, CY = 320.0, 256.0  # thermal principal point -- held fixed, see note below
FX0 = FY0 = 1443.0      # current (guessed) thermal focal length
WIDTH, HEIGHT = 640, 512

INTRINSICS_PATH = '/home/vish/ros2_ws/src/landfill_camera_sim/config/calibration/intrinsics.yaml'
EXTRINSICS_PATH = '/home/vish/ros2_ws/src/landfill_camera_sim/config/calibration/extrinsics.yaml'


def _downsample(points, cap):
    if points.shape[0] <= cap:
        return points
    idx = np.random.default_rng(0).choice(points.shape[0], size=cap, replace=False)
    return points[idx]


def lidar_person_centroid(baseline_xyz, pose_xyz, dist_thresh=0.08):
    if baseline_xyz.shape[0] == 0 or pose_xyz.shape[0] == 0:
        return None
    # The live pipeline (sensor_fusion_node, both camera nodes, etc.) is
    # still running and already saturating this Pi's 4 cores -- querying
    # the full ~1M-point clouds with parallel workers (workers=-1) fights
    # the pipeline for CPU and was observed taking 10+ minutes. A person
    # only needs a few hundred points to get a solid centroid, so
    # downsampling both clouds first cuts the query cost by ~20x with no
    # real loss of accuracy, and workers=1 avoids adding more contention.
    baseline_ds = _downsample(baseline_xyz, 60_000)
    pose_ds = _downsample(pose_xyz, 30_000)
    tree = cKDTree(baseline_ds)
    dist, _ = tree.query(pose_ds, k=1, workers=1)
    pose_xyz = pose_ds
    new_mask = dist > dist_thresh
    new_pts = pose_xyz[new_mask]
    if new_pts.shape[0] < 30:
        return None
    # Trim outliers: keep points within 1m of the median (rejects stray
    # noise/other-moving-object points far from the real cluster).
    med = np.median(new_pts, axis=0)
    close = np.linalg.norm(new_pts - med, axis=1) < 1.0
    new_pts = new_pts[close]
    if new_pts.shape[0] < 20:
        return None
    return np.mean(new_pts, axis=0), new_pts.shape[0]


def thermal_person_centroid(baseline_img, pose_img, percentile=92.0, min_blob=150,
                             static_hot_percentile=95.0):
    # NOT a baseline diff: this sensor is AGC'd (auto-gain-controlled, see
    # thermal_camera_node.py), so adding a person to the scene shifts the
    # whole frame's exposure/levels, not just the person's pixels -- a
    # diff against an empty-room baseline picks up that global shift and
    # flags 10-20% of the frame as "new" (confirmed: first attempt gave
    # 38k-63k px out of 327k, and a nonsense 1.5m/12x-focal-length fit).
    # A person is reliably the hottest, largest contiguous *moving*
    # object in an indoor scene regardless of the frame's overall gain
    # level -- but this room also has a static hot patch (direct
    # sunlight on the bed, confirmed in baseline.npz at up to 254/255,
    # same column range several poses land in) that's bigger than the
    # person at a loose threshold and steals the "largest blob" pick.
    # So: exclude whatever was already hot in the empty-room baseline
    # (that's static scenery, not the person), then take this frame's
    # own brightest percentile of what's left and the largest connected
    # blob within it.
    static_hot = baseline_img > np.percentile(baseline_img, static_hot_percentile)
    candidate = pose_img.copy()
    candidate[static_hot] = -np.inf
    thresh = np.percentile(pose_img, percentile)
    mask = candidate > thresh
    labels, n = label(mask)
    if n == 0:
        return None
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0  # background label
    best = int(np.argmax(sizes))
    if sizes[best] < min_blob:
        return None
    blob_mask = labels == best
    weights = pose_img * blob_mask
    ys, xs = np.nonzero(blob_mask)
    w = weights[ys, xs]
    u = np.sum(xs * w) / np.sum(w)
    v = np.sum(ys * w) / np.sum(w)
    return np.array([u, v]), int(sizes[best])


def project(xyz, t, fx, fy, cx, cy):
    p = xyz - t
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    inv_x = 1.0 / x
    u = fx * (-y * inv_x) + cx
    v = fy * (-z * inv_x) + cy
    return np.stack([u, v], axis=1)


def residuals(params, lidar_pts, thermal_uv):
    tx, ty, tz, k = params
    pred = project(lidar_pts, np.array([tx, ty, tz]), FX0 * k, FY0 * k, CX, CY)
    return (pred - thermal_uv).ravel()


def main():
    if len(sys.argv) < 2:
        print('usage: calib_solve.py <calib_dir> [--apply]')
        sys.exit(1)
    calib_dir = sys.argv[1].rstrip('/')
    apply_fix = '--apply' in sys.argv[2:]

    baseline = np.load(f'{calib_dir}/baseline.npz')
    base_xyz, base_thermal = baseline['lidar_xyz'], baseline['thermal_mean']

    import glob, os
    pose_files = sorted(glob.glob(f'{calib_dir}/pose*.npz'))

    lidar_pts, thermal_uv, labels = [], [], []
    for pf in pose_files:
        name = os.path.basename(pf)
        d = np.load(pf)
        lr = lidar_person_centroid(base_xyz, d['lidar_xyz'])
        th = thermal_person_centroid(base_thermal, d['thermal_mean'])
        if lr is None or th is None:
            print(f'{name}: SKIPPED (lidar_ok={lr is not None}, thermal_ok={th is not None})')
            continue
        lidar_centroid, n_lidar = lr
        thermal_centroid, n_thermal = th
        print(
            f'{name}: lidar centroid {lidar_centroid} ({n_lidar} pts), '
            f'thermal centroid {thermal_centroid} ({n_thermal} px)'
        )
        lidar_pts.append(lidar_centroid)
        thermal_uv.append(thermal_centroid)
        labels.append(name)

    if len(lidar_pts) < 3:
        print(f'\nOnly {len(lidar_pts)} usable poses -- need at least 3 to solve. Aborting.')
        sys.exit(1)

    lidar_pts = np.array(lidar_pts)
    thermal_uv = np.array(thermal_uv)

    x0 = np.array([0.0, 0.0, 0.0, 1.0])
    result = least_squares(residuals, x0, args=(lidar_pts, thermal_uv))
    tx, ty, tz, k = result.x
    resid_px = result.fun.reshape(-1, 2)
    per_pose_err = np.linalg.norm(resid_px, axis=1)

    print('\n--- Solved thermal camera correction ---')
    print(f'translation (camera_platform/thermal_optical_frame), meters: x={tx:.3f} y={ty:.3f} z={tz:.3f}')
    print(f'focal length scale: {k:.4f}  ->  fx=fy={FX0 * k:.1f} (was {FX0:.1f})')
    print('\nPer-pose reprojection error (pixels):')
    for label, err in zip(labels, per_pose_err):
        print(f'  {label}: {err:.1f}px')
    print(f'  mean: {per_pose_err.mean():.1f}px, max: {per_pose_err.max():.1f}px')

    if apply_fix:
        _apply(tx, ty, tz, FX0 * k, FY0 * k)
        print('\nWrote corrected values into extrinsics.yaml and intrinsics.yaml.')
    else:
        print('\n(dry run -- pass --apply to write these into the calibration YAML files)')


def _apply(tx, ty, tz, fx, fy):
    import re
    with open(EXTRINSICS_PATH, 'r', encoding='utf-8') as f:
        text = f.read()
    text = re.sub(
        r'(camera_platform/thermal_optical_frame:\s*\n\s*translation:\s*)\{[^}]*\}',
        lambda m: m.group(1) + '{x: %.4f, y: %.4f, z: %.4f}' % (tx, ty, tz),
        text,
    )
    with open(EXTRINSICS_PATH, 'w', encoding='utf-8') as f:
        f.write(text)

    with open(INTRINSICS_PATH, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    in_thermal = False
    for i, line in enumerate(lines):
        if line.strip().startswith('thermal_camera:'):
            in_thermal = True
        elif in_thermal and line.strip().startswith('fx:'):
            lines[i] = f'    fx: {fx:.1f}\n'
        elif in_thermal and line.strip().startswith('fy:'):
            lines[i] = f'    fy: {fy:.1f}\n'
        elif in_thermal and line.strip().startswith('rgb_camera:'):
            in_thermal = False
    with open(INTRINSICS_PATH, 'w', encoding='utf-8') as f:
        f.writelines(lines)


if __name__ == '__main__':
    main()
