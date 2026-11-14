"""Held-out test: do LiDAR board points land on the board in the image?

For each pose, the LiDAR board points are chosen using the extrinsic solved WITHOUT that pose, then projected into the
image by three models. The 'true' board outline in the image comes from the detected corners (0.1 px fit).
"""
import numpy as np, cv2, json
from scipy.spatial.transform import Rotation
from intrinsics import NAMES, calibrate, CONFIGS, D, SQ, W, H
from extrinsic import load_poses, solve, to_board, outside, describe
from lidar_board import OPT2FWD

NOM_K = np.array([[1065.0, 0, 640], [0, 1065.0, 360], [0, 0, 1]])


def project(q_rig, K, dist):
    """Rig-axes camera-frame points (x fwd, y left, z up) -> pixels with radial+tangential distortion."""
    q = q_rig[q_rig[:, 0] > 0.05]
    opt = np.stack([-q[:, 1], -q[:, 2], q[:, 0]], 1)
    px, _ = cv2.projectPoints(opt, np.zeros(3), np.zeros(3), K, dist)
    return px.reshape(-1, 2)


def quad_pixels(p, K, dist):
    rect = np.array([[-SQ, -SQ, 0], [4 * SQ, -SQ, 0], [4 * SQ, 6 * SQ, 0], [-SQ, 6 * SQ, 0]], np.float32)
    rv, _ = cv2.Rodrigues(p['R']); px, _ = cv2.projectPoints(rect, rv, p['t'], K, dist)
    return px.reshape(-1, 2).astype(np.float32)


def score(px, quad):
    """Pixel distance of each projected point to the board quad (0 inside), plus centroid offset vs quad centre."""
    d = np.array([cv2.pointPolygonTest(quad.reshape(-1, 1, 2), (float(a), float(b)), True) for a, b in px])
    outside_px = np.maximum(-d, 0)
    hull = cv2.convexHull(px.astype(np.float32))
    inter = cv2.intersectConvexConvex(hull, quad.reshape(-1, 1, 2))[0]
    iou = inter / (cv2.contourArea(hull) + cv2.contourArea(quad) - inter) * 100
    return (d >= 0).mean() * 100, float(np.median(outside_px)), float(np.percentile(outside_px, 90)), iou


if __name__ == '__main__':
    rms, K, dist, *_ = calibrate(NAMES, list(CONFIGS.values())[0])
    poses = load_poses(K, dist)
    x_all, _ = solve(poses)
    rows = {k: [] for k in ('production', 'calibrated camera, old extrinsic', 'calibrated camera + new extrinsic (held out)')}
    print(f'{"pose":8s} {"model":46s} {"%inside":>8s} {"median px out":>14s} {"p90 px out":>11s} {"overlap IoU %":>15s}')
    for i, p in enumerate(poses):
        x_loo, _ = solve([q for j, q in enumerate(poses) if j != i], x0=x_all)
        Rl = Rotation.from_rotvec(x_loo[:3]).as_matrix()
        q_new = p['pts'] @ Rl.T + x_loo[3:]
        pb = to_board(q_new, p['R'], p['t'])
        sel = (outside(pb[:, :2]) < 0.03) & (np.abs(pb[:, 2]) < 0.035)       # board points, chosen without this pose
        pts = p['pts'][sel]
        quad = quad_pixels(p, K, dist)
        models = {
            'production': project(pts, NOM_K, np.zeros(5)),
            'calibrated camera, old extrinsic': project(pts, K, dist),
            'calibrated camera + new extrinsic (held out)': project(pts @ Rl.T + x_loo[3:], K, dist),
        }
        for k, px in models.items():
            s = score(px, quad); rows[k].append(s)
            print(f'{p["name"]:8s} {k:46s} {s[0]:8.1f} {s[1]:14.1f} {s[2]:11.1f} {s[3]:15.1f}')
    print('\nMEAN over the 5 held-out poses')
    summ = {}
    for k, v in rows.items():
        v = np.array(v); summ[k] = dict(pct_inside=v[:, 0].mean(), median_px_out=v[:, 1].mean(), p90_px_out=v[:, 2].mean(), iou_pct=v[:, 3].mean())
        print(f'  {k:46s} inside {v[:,0].mean():5.1f}%  median-out {v[:,1].mean():5.1f}px  p90-out {v[:,2].mean():5.1f}px  overlap IoU {v[:,3].mean():5.1f}%')
    json.dump(dict(summary=summ, K=K.tolist(), dist=dist.tolist(), x=x_all.tolist()), open(f'{D}/evaluation.json', 'w'), indent=1)
