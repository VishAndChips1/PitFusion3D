"""LiDAR -> RGB-camera extrinsic from the 5 board poses: plane distance + board-outline containment.

Each pose gives (i) the board plane in camera coordinates (from the detected corners) and (ii) LiDAR points that
lie on the board. A point is penalised for being off the plane and for falling outside the known 29 x 40.6 cm
outline. The pose set is almost all 'board facing the sensors', so which parameters are well-determined is
reported by bootstrap, not assumed.
"""
import numpy as np, json
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from intrinsics import NAMES, calibrate, CONFIGS, D, SQ
from lidar_board import cloud, board_pose, OPT2FWD

LO = np.array([-SQ, -SQ]); HI = np.array([4 * SQ, 6 * SQ])     # board outline in board axes (m), origin = first inner corner
PAD = 0.01


def to_board(q_rig, R_b, t_b):
    """Rig-axes camera-frame points -> board axes."""
    return (OPT2FWD.T @ q_rig.T).T.__sub__(t_b) @ R_b          # = R_b.T @ (p_opt - t_b) for row vectors


def outside(xy):
    d = np.maximum(np.maximum(LO - xy, xy - HI), 0)
    return np.hypot(d[:, 0], d[:, 1])


def residuals(x, poses, gate):
    Rl = Rotation.from_rotvec(x[:3]).as_matrix(); tl = x[3:]
    res = []
    for p in poses:
        pb = to_board(p['pts'] @ Rl.T + tl, p['R'], p['t'])
        out = outside(pb[:, :2]); z = pb[:, 2]
        use = (out < gate) & (np.abs(z) < 0.5 * gate + 0.02)   # near the outline AND the plane: board, not wall / pole
        r_plane = np.where(use, z, 0.0); r_edge = np.where(use, np.maximum(out - PAD, 0), 0.0)   # fixed length for the optimiser
        res += [r_plane, 3.0 * r_edge]
    return np.concatenate(res)


def load_poses(K, dist, slab=0.12, radius=0.38):
    poses = []
    for n in NAMES:
        R, t = board_pose(n, K, dist)
        centre = OPT2FWD @ (R @ np.array([1.5 * SQ, 2.5 * SQ, 0]) + t)
        n_cam = OPT2FWD @ (R @ np.array([0, 0, 1.0]))
        pts = cloud(n, centre, radius)
        pts = pts[np.abs((pts - centre) @ n_cam) < slab].astype(np.float64)
        poses.append(dict(name=n, pts=pts, R=R, t=t))
    return poses


def solve(poses, gates=(0.12, 0.08, 0.06, 0.04, 0.03), x0=None):
    x = np.zeros(6) if x0 is None else x0
    for g in gates:                                          # tighten the gate as the alignment improves
        sol = least_squares(residuals, x, args=(poses, g), loss='soft_l1', f_scale=0.02)
        x = sol.x
    return x, sol


def describe(x):
    e = Rotation.from_rotvec(x[:3]).as_euler('xyz', degrees=True)
    return f'rotation roll/pitch/yaw (deg) {np.round(e, 2)}, translation x,y,z (m) {np.round(x[3:], 3)}'


if __name__ == '__main__':
    rms, K, dist, *_ = calibrate(NAMES, list(CONFIGS.values())[0])
    poses = load_poses(K, dist)
    for p in poses:
        print(p['name'], len(p['pts']), 'points in slab', flush=True)
    x, sol = solve(poses)
    print('ALL 5 poses:', describe(x))
    for p in poses:
        r = residuals(x, [p], 0.03)
        pb = to_board((p['pts'] @ Rotation.from_rotvec(x[:3]).as_matrix().T + x[3:]), p['R'], p['t'])
        use = (outside(pb[:, :2]) < 0.03) & (np.abs(pb[:, 2]) < 0.035)
        print(f"  {p['name']}: {use.sum()} board points, plane rms {np.sqrt(np.mean(pb[use,2]**2))*100:.2f} cm, mean plane offset {pb[use,2].mean()*100:+.2f} cm")
    # leave-one-pose-out: how much does the answer move?
    loo = []
    for i, p in enumerate(poses):
        xi, _ = solve([q for j, q in enumerate(poses) if j != i], x0=x)
        loo.append(xi); print(f'without {p["name"]}:', describe(xi), flush=True)
    np.save(f'{D}/extrinsic_all.npy', x); np.save(f'{D}/extrinsic_loo.npy', np.array(loo))
