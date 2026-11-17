"""Geometry helpers for the Run 1 test: calibration loading and LiDAR-to-image projection (same maths as
sensor_fusion_node.py), plane finding by RANSAC, the indirect RGB/LiDAR alignment score, and the image renderers.
Pure numpy/OpenCV/matplotlib: no ROS needed, so everything can be re-run offline on the saved captures."""
import json
from pathlib import Path

import numpy as np
import yaml

CAL = Path.home() / 'ros2_ws/src/landfill_camera_sim/config/calibration'
LIDAR_FRAME = 'camera_platform/lidar_frame'
RGB_FRAME = 'camera_platform/rgb_optical_frame'
COLOURS = ['#4A5D7A', '#B5532A', '#7D8FAD', '#B9C3D3']


# ------------------------------------------------------------------ calibration and projection
def rpy_matrix(roll, pitch, yaw):
    """Same as scipy Rotation.from_euler('xyz', rpy) (extrinsic x, y, z)."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def load_calibration(cal_dir=CAL):
    """-> dict(intr=..., M, b) with p_cam = xyz @ M + b (row vectors), exactly as the fusion node builds it."""
    ext = yaml.safe_load(open(Path(cal_dir) / 'extrinsics.yaml')) or {}
    frames = {}
    for child, tf in (ext.get('transforms') or {}).items():
        t = np.array([float((tf.get('translation') or {}).get(k, 0.0)) for k in 'xyz'])
        rot = tf.get('rotation_rpy') or {}
        frames[str(child)] = (rpy_matrix(float(rot.get('roll', 0)), float(rot.get('pitch', 0)), float(rot.get('yaw', 0))), t)
    r_l, t_l = frames.get(LIDAR_FRAME, (np.eye(3), np.zeros(3)))
    r_c, t_c = frames.get(RGB_FRAME, (np.eye(3), np.zeros(3)))
    m = (r_l.T @ r_c)
    b = (t_l - t_c) @ r_c
    bore = (ext.get('boresight_correction_deg') or {}).get(RGB_FRAME)
    if bore:
        cy, sy = np.cos(np.radians(bore.get('yaw', 0.0))), np.sin(np.radians(bore.get('yaw', 0.0)))
        cp, sp = np.cos(np.radians(bore.get('pitch', 0.0))), np.sin(np.radians(bore.get('pitch', 0.0)))
        rc = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        m, b = m @ rc.T, b @ rc.T
    intr_all = yaml.safe_load(open(Path(cal_dir) / 'intrinsics.yaml'))
    cam = intr_all['rgb_camera']
    mat = cam['camera_matrix']
    dist = [float(v) for v in cam.get('distortion_coefficients', [])][:5]
    dist += [0.0] * (5 - len(dist))
    intr = {'width': int(cam['image_width']), 'height': int(cam['image_height']), 'fx': float(mat['fx']), 'fy': float(mat['fy']),
            'cx': float(mat['cx']), 'cy': float(mat['cy']), 'dist': tuple(dist)}
    return {'intr': intr, 'M': m.astype(np.float64), 'b': b.astype(np.float64)}


def project(pc, intr, round_px=True):
    """Camera-frame points (x fwd, y left, z up) -> (u, v, valid); lens model as in the fusion node."""
    x, y, z = pc[:, 0], pc[:, 1], pc[:, 2]
    ok = x > 0.05
    inv = np.zeros_like(x)
    np.divide(1.0, x, out=inv, where=ok)
    xn, yn = -y * inv, -z * inv
    ok &= (np.abs(xn) < 4.0) & (np.abs(yn) < 4.0)
    xn, yn = np.where(ok, xn, 0.0), np.where(ok, yn, 0.0)
    k1, k2, p1, p2, k3 = intr['dist']
    if any(intr['dist']):
        r2 = xn * xn + yn * yn
        rad = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        xn, yn = xn * rad + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn), yn * rad + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
    u, v = intr['fx'] * xn + intr['cx'], intr['fy'] * yn + intr['cy']
    if round_px:
        u, v = np.rint(u), np.rint(v)
    ok &= (u >= 0) & (u < intr['width']) & (v >= 0) & (v < intr['height'])
    return u, v, ok


def rot_axis(axis, deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return {'x': np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), 'y': np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            'z': np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def to_camera(xyz, cal, d_trans=(0.0, 0.0, 0.0), rot=None):
    """Deployed calibration, plus an optional artificial offset applied in the camera frame:
    d_trans = metres along camera x (forward), y (left), z (up); rot = (axis, degrees) about that camera axis."""
    pc = xyz.astype(np.float64) @ cal['M'] + cal['b']
    if rot is not None:
        pc = pc @ rot_axis(*rot).T
    return pc + np.asarray(d_trans)


# ------------------------------------------------------------------ planes
def ransac_plane(pts, rng, iters=250, thr=0.05, max_pts=20000):
    """Dominant plane of pts (N,3): returns (n, d, inlier_mask_on_pts) with n.x + d = 0, d<=0 oriented, or None."""
    if len(pts) < 400:
        return None
    sub = pts if len(pts) <= max_pts else pts[rng.choice(len(pts), max_pts, replace=False)]
    idx = rng.integers(0, len(sub), size=(iters, 3))
    a, b, c = sub[idx[:, 0]], sub[idx[:, 1]], sub[idx[:, 2]]
    n = np.cross(b - a, c - a)
    nn = np.linalg.norm(n, axis=1)
    good = nn > 1e-6
    if not good.any():
        return None
    n = n[good] / nn[good][:, None]
    d = -np.einsum('ij,ij->i', n, a[good])
    counts = (np.abs(sub @ n.T + d) < thr).sum(axis=0)
    k = int(np.argmax(counts))
    inl = np.abs(sub @ n[k] + d[k]) < thr
    if inl.sum() < 300:
        return None
    # refit on the inliers (SVD), then re-select inliers from the full set
    q = sub[inl]
    cen = q.mean(axis=0)
    nvec = np.linalg.svd(q - cen, full_matrices=False)[2][-1]
    dd = -float(nvec @ cen)
    if dd > 0:
        nvec, dd = -nvec, -dd
    mask = np.abs(pts @ nvec + dd) < thr
    return nvec, dd, mask


def find_planes(xyz, rng, max_planes=3, thr=0.05, min_extent=0.5):
    """Up to max_planes planes, strongest first; each (n, d, rms_m, inliers, centroid, extent_m)."""
    rest = xyz
    out = []
    for _ in range(max_planes):
        r = ransac_plane(rest, rng, thr=thr)
        if r is None:
            break
        n, d, mask = r
        q = rest[mask]
        cen = q.mean(axis=0)
        sv = np.linalg.svd(q - cen, compute_uv=False)
        extent = float(np.sqrt(sv[1] ** 2 / len(q)) * 3.46)       # spread along the plane (uniform-width equivalent)
        res = q @ n + d
        if extent >= min_extent:
            out.append((n, d, float(np.sqrt(np.mean(res ** 2))), int(mask.sum()), cen, extent))
        rest = rest[~mask]
    return out


# ------------------------------------------------------------------ alignment score
def edge_points(xyz, cal, rel=0.12, min_abs=0.20):
    """Indices of LiDAR points that sit on a depth discontinuity in the (deployed) image-plane depth map.
    The depth map is formed once from the geometry; the choice of which points are edges does not look at image content."""
    intr = cal['intr']
    pc = to_camera(xyz, cal)
    u, v, ok = project(pc, intr)
    idx = np.where(ok)[0]
    depth = pc[idx, 0]
    ui, vi = u[idx].astype(int), v[idx].astype(int)
    h, w = intr['height'], intr['width']
    cell = 4
    gh, gw = h // cell + 1, w // cell + 1
    dmin = np.full((gh, gw), np.inf)
    dmax = np.full((gh, gw), -np.inf)
    gi, gj = vi // cell, ui // cell
    np.minimum.at(dmin, (gi, gj), depth)
    np.maximum.at(dmax, (gi, gj), depth)
    import cv2
    nb_min = cv2.erode(np.where(np.isfinite(dmin), dmin, 1e6).astype(np.float32), np.ones((3, 3), np.uint8))
    nb_max = cv2.dilate(np.where(np.isfinite(dmax), dmax, 0).astype(np.float32), np.ones((3, 3), np.uint8))
    jump = (nb_max - nb_min) > np.maximum(min_abs, rel * np.where(np.isfinite(dmin), dmin, 1e6))
    jump &= np.isfinite(dmin) & (nb_min < 1e5)
    sel = jump[gi, gj]
    # keep the near side of the jump (the occluding edge): points not much farther than the cell neighbourhood minimum
    sel &= depth <= nb_min[gi, gj] + 0.5 * (nb_max[gi, gj] - nb_min[gi, gj])
    return idx[sel]


def gradient_map(img_rgb, sigma=2.0):
    import cv2
    g = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY).astype(np.float32)
    g = cv2.GaussianBlur(g, (0, 0), 1.2)
    gx, gy = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.sqrt(gx * gx + gy * gy)
    mag = cv2.GaussianBlur(mag, (0, 0), sigma)
    return mag / (np.percentile(mag, 99) + 1e-6)


def score_offset(xyz_edge, grad_maps, cal, d_trans=(0, 0, 0), rot=None):
    """Mean normalised image-gradient magnitude at the projected depth-edge points, averaged over the frames. Higher = better alignment."""
    pc = to_camera(xyz_edge, cal, d_trans, rot)
    u, v, ok = project(pc, cal['intr'])
    if ok.sum() < 20:
        return float('nan'), 0, (np.nan, np.nan)
    ui, vi = u[ok].astype(int), v[ok].astype(int)
    s = float(np.mean([g[vi, ui].mean() for g in grad_maps]))
    return s, int(ok.sum()), (float(u[ok].mean()), float(v[ok].mean()))


# ------------------------------------------------------------------ renderers
def _mpl():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'serif', 'font.serif': ['Liberation Serif', 'DejaVu Serif'], 'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.grid': True, 'axes.grid.axis': 'y', 'grid.color': '#dddddd', 'grid.linewidth': 0.6, 'savefig.dpi': 300})
    return plt


def unpack_rgb(packed):
    packed = packed.astype(np.uint32)
    return np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], axis=1) / 255.0, packed != 0


def overlay_lidar_on_rgb(img, xyz, cal, path, max_range=30.0, title=''):
    """LiDAR points drawn over the RGB frame, coloured by range, with a colour bar."""
    plt = _mpl()
    pc = to_camera(xyz, cal)
    u, v, ok = project(pc, cal['intr'], round_px=False)
    rng_m = np.linalg.norm(xyz, axis=1)
    fig, ax = plt.subplots(figsize=(6.4, 3.9))
    ax.imshow(img)
    ax.grid(False)
    sc = ax.scatter(u[ok], v[ok], c=np.clip(rng_m[ok], 0, max_range), s=1.2, cmap='turbo', vmin=0, vmax=max_range, linewidths=0)
    cb = fig.colorbar(sc, ax=ax, fraction=0.035, pad=0.01)
    cb.set_label('range (m)')
    ax.axis('off')
    ax.set_title(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=300)
    plt.close(fig)
    return int(ok.sum())


def sensor_view(xyz, colour, path, title='', cmap_range=None, max_range=30.0):
    """Cloud rendered from the sensor viewpoint (u = -y/x, v = -z/x, v increases downward); colour = Nx3 rgb or None (then range)."""
    plt = _mpl()
    m = xyz[:, 0] > 0.5
    sel = np.where(m)[0]
    if len(sel) > 150000:
        sel = np.random.default_rng(0).choice(sel, 150000, replace=False)
    u, v = -xyz[sel, 1] / xyz[sel, 0], -xyz[sel, 2] / xyz[sel, 0]
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    ax.grid(False)
    if colour is None:
        sc = ax.scatter(u, v, c=np.clip(np.linalg.norm(xyz[sel], axis=1), 0, max_range), s=0.6, cmap='turbo', vmin=0, vmax=max_range, linewidths=0)
        cb = fig.colorbar(sc, ax=ax, fraction=0.035, pad=0.01)
        cb.set_label('range (m)')
    else:
        rgb, has = unpack_rgb(colour[sel])
        ax.scatter(u[~has], v[~has], c='#999999', s=0.6, linewidths=0)
        ax.scatter(u[has], v[has], c=rgb[has], s=0.6, linewidths=0)
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(0.9, -0.9)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=300)
    plt.close(fig)


def plan_views(xyz, colour, top_path, side_path, title='', extent=30.0):
    """Top-down (x forward, y left) and side (x forward, z up) views, RGB-coloured where available, with a 5 m grid."""
    plt = _mpl()
    sel = np.where(np.isfinite(xyz).all(axis=1) & (xyz[:, 0] > 0) & (xyz[:, 0] < extent))[0]
    if len(sel) > 200000:
        sel = np.random.default_rng(1).choice(sel, 200000, replace=False)
    if colour is not None:
        rgb, has = unpack_rgb(colour[sel])
        c = np.where(has[:, None], rgb, 0.6)
    else:
        c = np.clip(xyz[sel, 0], 0, extent)
    for path, (ia, ib, la, lb, lim) in ((top_path, (1, 0, 'y, left (m)', 'x, forward (m)', (-12, 12))), (side_path, (2, 0, 'z, up (m)', 'x, forward (m)', (-6, 6)))):
        fig, ax = plt.subplots(figsize=(6.4, 5.0))
        ax.scatter(xyz[sel, ia], xyz[sel, ib], c=c, s=0.5, linewidths=0, cmap='turbo' if colour is None else None)
        ax.set_xlim(*lim)
        ax.set_ylim(0, extent)
        ax.set_xticks(np.arange(lim[0], lim[1] + 1, 5))
        ax.set_yticks(np.arange(0, extent + 1, 5))
        ax.grid(True, axis='both', color='#cccccc', linewidth=0.5)
        ax.set_xlabel(la)
        ax.set_ylabel(lb)
        ax.set_aspect('equal')
        ax.set_title(title, fontsize=9)
        fig.tight_layout()
        fig.savefig(path, dpi=300)
        plt.close(fig)


def contact_sheet(files, path, cols=6, thumb=(320, 180)):
    import cv2
    if not files:
        return False
    tiles = []
    for f in files:
        im = cv2.imread(str(f))
        if im is None:
            continue
        im = cv2.resize(im, thumb)
        cv2.putText(im, Path(f).stem.replace('rgb_', ''), (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        tiles.append(im)
    if not tiles:
        return False
    rows = []
    for i in range(0, len(tiles), cols):
        row = tiles[i:i + cols]
        row += [np.zeros_like(tiles[0])] * (cols - len(row))
        rows.append(np.hstack(row))
    cv2.imwrite(str(path), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 85])
    return True
