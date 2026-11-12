import sys, numpy as np, cv2, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
sys.path.insert(0, '/home/vish/ros2_ws/src/landfill_camera_sim/scripts'); import sensor_fusion_node as sf
ref, extr = sf._load_extrinsics(sf._package_path('config/calibration/extrinsics.yaml')); I = sf._load_camera_intrinsics(sf._package_path('config/calibration/intrinsics.yaml'), 'rgb_camera')
Mx, bx = sf.lidar_to_camera_transform(extr, 'camera_platform/lidar_frame', 'camera_platform/rgb_optical_frame')
d = np.load('data/overlay_calib_wide.npz'); img, X = d['image'], d['xyz']
def proj(yaw, pitch):
    P = X.astype(np.float64) @ Mx + bx; cy, sy, cp, sp = np.cos(np.radians(yaw)), np.sin(np.radians(yaw)), np.cos(np.radians(pitch)), np.sin(np.radians(pitch))
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]); Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]); P = P @ (Rz @ Ry).T
    ui, vi, ok = sf.project_to_pixels(P, I['fx'], I['fy'], I['cx'], I['cy'], I['width'], I['height']); return ui[ok], vi[ok], P[ok, 0]
fig, ax = plt.subplots(1, 2, figsize=(15, 4.6))
for a, (yaw, pit, title) in zip(ax, [(0, 0, 'current calibration (no rotation)'), (3.45, -0.8, 'measured correction: yaw +3.45 deg, pitch -0.8 deg')]):
    u, v, dep = proj(yaw, pit); a.imshow(img); a.scatter(u, v, c=dep, s=0.5, cmap='turbo', alpha=0.55, vmin=1.5, vmax=6); a.set_title(title); a.axis('off')
fig.suptitle('LiDAR points (colour = range 1.5-6 m) projected onto the RGB frame with the production projection', x=0.01, ha='left', fontsize=10)
plt.tight_layout(); plt.savefig('figures/overlay_before_after.png', dpi=130); print('ok')
