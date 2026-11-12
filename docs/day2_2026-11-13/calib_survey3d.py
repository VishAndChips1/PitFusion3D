#!/usr/bin/env python3
"""LiDAR -> RGB overlay with the production calibration/projection, plus a calibration-error estimate from depth-edge / image-edge alignment.
Outputs: figures/overlay_*.png (via make_day2_figures) data inputs in data/overlay_calib.npz and data/overlay_calib.json"""
import sys, time, json, numpy as np, cv2, rclpy
SC = '/home/vish/ros2_ws/src/landfill_camera_sim/scripts'; sys.path.insert(0, SC)
import sensor_fusion_node as sf, cloud_io
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2, Image
import os
GM, GS, GC = float(os.environ.get('GRID_MAX', 4.0)), float(os.environ.get('GRID_STEP', 0.5)), float(os.environ.get('GRID_CENTER', 0.0)); GCP = float(os.environ.get('GRID_CENTER_PITCH', os.environ.get('GRID_CENTER', 0.0))); NF = int(os.environ.get('NFRAMES', 6)); TAG = os.environ.get('TAG', 'coarse')
ref, extr = sf._load_extrinsics(sf._package_path('config/calibration/extrinsics.yaml')); I = sf._load_camera_intrinsics(sf._package_path('config/calibration/intrinsics.yaml'), 'rgb_camera')
Mx, bx = sf.lidar_to_camera_transform(extr, 'camera_platform/lidar_frame', 'camera_platform/rgb_optical_frame')
import os as _os
if _os.environ.get('APPLY_CORR') == '1':
    _c = sf._load_boresight_corrections(sf._package_path('config/calibration/extrinsics.yaml')).get('camera_platform/rgb_optical_frame')
    if _c: Mx, bx = sf.apply_boresight_correction((Mx, bx), *_c); print('using production boresight correction', _c)
rclpy.init(); n = Node('overlay_calib'); L = []; R = []
def st(m): return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
n.create_subscription(PointCloud2, '/landfill/livox/points', lambda m: L.append((st(m), np.stack([cloud_io.structured_view(m)[k] for k in 'xyz'], 1)[::2].astype(np.float32))), Q)
def on_img(m):
    if len(R) % 1 == 0 and (not R or st(m) - R[-1][0] >= 6.0): R.append((st(m), np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()))
n.create_subscription(Image, '/landfill/rgb/image_raw', on_img, Q)
t0 = time.time()
while time.time() - t0 < float(os.environ.get('COLLECT_S', 14)): rclpy.spin_once(n, timeout_sec=0.05)
print('collected', len(L), 'lidar scans and', len(R), 'rgb frames')
Ls = np.array([x[0] for x in L]); picks = R[1:-1]
def accum(ts, span=0.5):
    sel = [x[1] for x in L if abs(x[0] - ts) <= span]; X = np.concatenate(sel) if sel else np.zeros((0, 3), np.float32); X = X[np.isfinite(X).all(1) & (np.abs(X).sum(1) > 0)]; return X, len(sel)
def project(X, yaw=0.0, pitch=0.0, scale=1.0):
    Pc = X.astype(np.float64) @ Mx + bx
    cy, sy, cp, sp = np.cos(np.radians(yaw)), np.sin(np.radians(yaw)), np.cos(np.radians(pitch)), np.sin(np.radians(pitch))
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]); Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]); Pc = Pc @ (Rz @ Ry).T
    ui, vi, ok = sf.project_to_pixels(Pc, I['fx'] * scale, I['fy'] * scale, I['cx'], I['cy'], I['width'], I['height']); return ui, vi, ok, Pc[:, 0]
S = 4; H, W = I['height'] // S, I['width'] // S
def maps(X, gray, yaw, pitch, scale=1.0):
    ui, vi, ok, depth = project(X, yaw, pitch, scale); ui, vi, d = ui[ok] // S, vi[ok] // S, depth[ok]
    inv = np.zeros(H * W, np.float32); np.maximum.at(inv, (vi.clip(0, H - 1) * W + ui.clip(0, W - 1)), (1.0 / d).astype(np.float32)); inv = inv.reshape(H, W)
    inv = cv2.dilate(inv, np.ones((5, 5), np.uint8)); valid = cv2.erode((inv > 0).astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
    gd = cv2.magnitude(cv2.Sobel(inv, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(inv, cv2.CV_32F, 0, 1, ksize=3))
    return gd, valid

import itertools
gyaw = np.arange(-1.0, 6.01, 0.5); gpit = np.arange(-3.0, 3.01, 0.5); gsc = np.array([0.94, 0.96, 0.98, 1.0, 1.02, 1.04, 1.06])
cubes = []; res = []
for ts, img in picks:
    X, ns = accum(ts)
    if len(X) < 10000: continue
    g = cv2.GaussianBlur(cv2.resize(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), (W, H), interpolation=cv2.INTER_AREA).astype(np.float32), (0, 0), 1.2)
    gi = cv2.magnitude(cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3))
    cube = np.full((len(gyaw), len(gpit), len(gsc)), np.nan)
    for (a1, yaw), (b1, pit), (c1, sc) in itertools.product(enumerate(gyaw), enumerate(gpit), enumerate(gsc)):
        gd, valid = maps(X, g, yaw, pit, sc)
        if valid.sum() > 500: cube[a1, b1, c1] = np.corrcoef(gd[valid], gi[valid])[0, 1]
    cubes.append(cube); i = np.unravel_index(np.nanargmax(cube), cube.shape); z = cube[list(gyaw).index(0.0) if 0.0 in gyaw else 2, 6, 3]
    res.append(dict(stamp=ts, best_yaw=float(gyaw[i[0]]), best_pitch=float(gpit[i[1]]), best_scale=float(gsc[i[2]]), score_best=float(cube[i]), score_base=float(cube[list(gyaw).index(0.0), list(gpit).index(0.0), list(gsc).index(1.0)])))
    print('frame', len(res), res[-1], flush=True)
mean = np.nanmean(np.stack(cubes), 0); i = np.unravel_index(np.nanargmax(mean), mean.shape)
out = dict(per_frame=res, mean_best_yaw=float(gyaw[i[0]]), mean_best_pitch=float(gpit[i[1]]), mean_best_scale=float(gsc[i[2]]), mean_score_best=float(mean[i]), mean_score_base=float(mean[list(gyaw).index(0.0), list(gpit).index(0.0), list(gsc).index(1.0)]),
           std_yaw=float(np.std([r['best_yaw'] for r in res])), std_pitch=float(np.std([r['best_pitch'] for r in res])), std_scale=float(np.std([r['best_scale'] for r in res])), n_frames=len(res), intrinsics_fx=I['fx'])
json.dump(out, open('data/calib_survey3d.json', 'w'), indent=1); np.savez_compressed('data/calib_survey3d_cube.npz', mean=mean, gyaw=gyaw, gpit=gpit, gsc=gsc); print('GLOBAL best: yaw %.2f pitch %.2f scale %.2f score %.3f (base %.3f)' % (out['mean_best_yaw'], out['mean_best_pitch'], out['mean_best_scale'], out['mean_score_best'], out['mean_score_base']))
