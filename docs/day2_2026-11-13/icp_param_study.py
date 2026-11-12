import sys, json, time, itertools, numpy as np
sys.path.insert(0, '/home/vish/ros2_ws/src/landfill_camera_sim/scripts')
import registration_merge_node as rm
M = np.load('/home/vish/thesis_tests/final_campaign_20261112/data/sample_merged_cloud.npz')['xyz'].astype(np.float64); rng = np.random.default_rng(5)
def rod(axis, deg):
    a = np.radians(deg); k = axis / np.linalg.norm(axis); K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]); return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
def sub(x, n): return x if len(x) <= n else x[rng.choice(len(x), n, replace=False)]
rows = []
for vox, nsrc, ntgt, it in itertools.product((0.3, 0.2, 0.1, 0.05), (1500, 4000), (10000, 25000), (10, 20)):
    if vox >= 0.2 and nsrc > 1500: continue
    errs, angs, ms, tot = [], [], [], []
    for _ in range(20):
        scan = sub(M, 24000) + rng.normal(0, 0.016, (24000, 3)); R = rod(rng.normal(size=3), 0.5); d = rng.normal(size=3); t = 0.05 * d / np.linalg.norm(d); moved = scan @ R.T + t
        t0 = time.perf_counter(); tgt = sub(M[rm.voxel_downsample_indices(M, vox)], ntgt); src = sub(moved[rm.voxel_downsample_indices(moved, vox)], nsrc)
        Rr, tr, conv = rm.icp_point_to_point(src, tgt, it, 0.5, 200); tot.append((time.perf_counter() - t0) * 1000)
        errs.append(np.linalg.norm(Rr @ t + tr) * 1000); angs.append(np.degrees(np.arccos(np.clip((np.trace(Rr @ R) - 1) / 2, -1, 1))))
    rows.append(dict(voxel_m=vox, max_src=nsrc, max_tgt=ntgt, iterations=it, t_err_mm_median=float(np.median(errs)), rot_err_deg_median=float(np.median(angs)), total_ms_median=float(np.median(tot))))
    print(rows[-1])
json.dump(rows, open('data/icp_param_study.json', 'w'), indent=1)
