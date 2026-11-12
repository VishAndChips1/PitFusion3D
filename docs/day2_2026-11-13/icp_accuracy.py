#!/usr/bin/env python3
"""ICP registration accuracy with KNOWN ground truth, using the production code and parameters.
A real merged cloud is the map; a 24 000-point scan is drawn from it, displaced by a known rigid motion and corrupted with LiDAR-like noise;
the pipeline's own preprocessing + icp_point_to_point must recover the motion. Output: data/icp_accuracy.json"""
import sys, json, time, numpy as np
sys.path.insert(0, '/home/vish/ros2_ws/src/landfill_camera_sim/scripts')
import registration_merge_node as rm
M = np.load('/home/vish/thesis_tests/final_campaign_20261112/data/sample_merged_cloud.npz')['xyz'].astype(np.float64)
rng = np.random.default_rng(11)
P = dict(voxel=0.2, max_src=1500, max_tgt=10000, iters=10, corr=0.5, min_corr=200)   # the launch/node defaults
def rodrigues(axis, deg):
    a = np.radians(deg); k = axis / np.linalg.norm(axis); K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
def sub(x, n):
    return x if len(x) <= n else x[rng.choice(len(x), n, replace=False)]
def prep(x):
    ds = np.asarray(x[rm.voxel_downsample_indices(x, P['voxel'])]); return ds
def trial(map_xyz, trans_m, rot_deg, noise_m):
    scan = sub(map_xyz, 24000) + rng.normal(0, noise_m, (24000, 3))
    axis = rng.normal(size=3); R = rodrigues(axis, rot_deg); d = rng.normal(size=3); t = trans_m * d / np.linalg.norm(d)
    moved = scan @ R.T + t                                    # scan as seen after the motion
    tgt = sub(prep(map_xyz), P['max_tgt']); src = sub(prep(moved), P['max_src'])
    t0 = time.perf_counter(); Rr, tr, conv = rm.icp_point_to_point(src, tgt, P['iters'], P['corr'], P['min_corr']); ms = (time.perf_counter() - t0) * 1000
    # recovered transform should undo the motion: Rr @ (R x + t) + tr ~ x  =>  Rr ~ R^T, tr ~ -R^T t
    R_err = Rr @ R; ang = np.degrees(np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))); t_err = np.linalg.norm(Rr @ t + tr)
    return t_err * 1000, ang, bool(conv), ms
def run(name, map_xyz, grid, noise=0.016, n=25):
    rows = []
    for tm, rd in grid:
        r = np.array([trial(map_xyz, tm, rd, noise) for _ in range(n)])
        ok = (r[:, 0] < 50) & (r[:, 1] < 2)               # "success" = recovered to < 5 cm and < 2 deg
        rows.append(dict(translation_cm=tm * 100, rotation_deg=rd, success_pct=float(ok.mean() * 100), t_err_mm_median=float(np.median(r[ok, 0])) if ok.any() else None, t_err_mm_p95=float(np.percentile(r[ok, 0], 95)) if ok.any() else None,
                         rot_err_deg_median=float(np.median(r[ok, 1])) if ok.any() else None, converged_pct=float(r[:, 2].mean() * 100), icp_ms_median=float(np.median(r[:, 3]))))
        print(name, rows[-1])
    return rows
grid = [(0.0, 0.0), (0.01, 0.1), (0.02, 0.25), (0.05, 0.5), (0.10, 1.0), (0.20, 2.0), (0.30, 4.0), (0.50, 8.0)]
out = dict(params=P, map_points=int(len(M)), noise_m=0.016, scene=run('full scene', M, grid))
# degenerate geometry: a single plane (floor/wall) leaves in-plane sliding unobservable
z = M[:, 2]; floor = M[np.abs(z - np.median(z[z < np.percentile(z, 30)])) < 0.03]
if len(floor) > 3000: out['single_plane'] = run('single plane', floor, [(0.0, 0.0), (0.02, 0.25), (0.05, 0.5), (0.10, 1.0)], n=15); out['single_plane_points'] = int(len(floor))
out['noise_sensitivity'] = {f'{n*1000:.0f}mm': run(f'noise {n*1000:.0f}mm', M, [(0.05, 0.5)], noise=n, n=20)[0] for n in (0.005, 0.016, 0.03, 0.06)}
json.dump(out, open('data/icp_accuracy.json', 'w'), indent=1); print('saved')
