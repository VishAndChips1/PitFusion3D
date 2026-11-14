"""Find the board's points and plane in each bag's accumulated LiDAR cloud, seeded by the camera's board pose."""
import numpy as np, cv2, json
from intrinsics import obj, C, NAMES, W, H, calibrate, CONFIGS, D

OPT2FWD = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)   # optical (x right,y down,z fwd) -> rig (x fwd,y left,z up)


def cloud(name, centre=None, radius=0.45):
    """Accumulate the bag's scans (compact float32 array), keeping only points within `radius` of `centre`."""
    scans = np.load(f'{D}/{name}_scans.npy', mmap_mode='r')
    out = []
    for i in range(len(scans)):
        a = np.asarray(scans[i])
        a = a[np.isfinite(a).all(1) & (np.linalg.norm(a, axis=1) > 0.3)]
        if centre is not None:
            a = a[np.linalg.norm(a - centre, axis=1) < radius]
        out.append(a)
    return np.concatenate(out)


def board_pose(name, K, d, frame=None):
    """Board pose in the camera (rig-axes) frame: centre, normal, rotation to board axes; board origin = first inner corner."""
    c = np.nanmedian(C[name], axis=0).astype(np.float32).reshape(-1, 1, 2)
    ok, rv, tv = cv2.solvePnP(obj, c, K, d, flags=cv2.SOLVEPNP_ITERATIVE)
    R, _ = cv2.Rodrigues(rv)
    return R, tv.ravel()          # p_optical = R @ p_board + t   (board axes: x right, y down, z out of the board toward ... )


def ransac_plane(P, thr=0.02, iters=400, rng=np.random.default_rng(0)):
    best = None
    for _ in range(iters):
        s = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn; dd = -n @ s[0]
        inl = np.abs(P @ n + dd) < thr
        if best is None or inl.sum() > best[0].sum():
            best = (inl, n, dd)
    inl, n, dd = best
    for _ in range(3):   # least-squares refit on inliers
        Q = P[inl]; cen = Q.mean(0); u, s_, vt = np.linalg.svd(Q - cen, full_matrices=False); n = vt[2]; dd = -n @ cen
        inl = np.abs(P @ n + dd) < thr
    return inl, n, dd


def find_board(name, K, dist, slab=0.12, radius=0.38):
    """LiDAR points of the board: near the camera-predicted board plane and centre, then a plane fit."""
    R, t = board_pose(name, K, dist)
    centre = OPT2FWD @ (R @ np.array([1.5 * 0.058, 2.5 * 0.058, 0]) + t)
    n_cam = OPT2FWD @ (R @ np.array([0, 0, 1.0]))
    near = cloud(name, centre, radius)
    near = near[np.abs((near - centre) @ n_cam) < slab]
    inl, n, dd = ransac_plane(near, thr=0.02)
    return near[inl], n, dd, centre, n_cam


if __name__ == '__main__':
    rms, K, dist, *_ = calibrate(NAMES, list(CONFIGS.values())[0])
    out = {}
    for name in NAMES:
        Q, n, dd, centre, n_cam = find_board(name, K, dist)
        ang = np.degrees(np.arccos(abs(n @ n_cam)))
        resid = Q @ n + dd
        print(f'{name}: {len(Q)} pts on board plane, resid std {resid.std()*100:.2f} cm, lidar plane dist {abs(dd):.3f} m '
              f'(camera-predicted centre range {centre[0]:.3f} m), normal vs camera normal {ang:.1f} deg', flush=True)
        out[name] = dict(n=n.tolist(), d=float(dd), pts=Q.tolist(), cam_centre=centre.tolist(), cam_normal=n_cam.tolist())
    json.dump(out, open(f'{D}/lidar_board_planes.json', 'w'))
