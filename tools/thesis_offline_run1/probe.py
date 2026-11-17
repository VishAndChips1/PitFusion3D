#!/usr/bin/env python3
"""ROS-side helper for the TestWithLidV1 field test.

  probe.py log <outdir> <status.json>       record every topic's arrival time / stamp / size until killed (SIGTERM)
  probe.py capture <out.npz> <spec> [secs]  grab real messages, e.g.  lidar:20,fused:12,rgb:1,thermal:1,merged:1,caminfo:1

Subscriptions are raw (serialised bytes), so a 2.7 MB camera frame costs a copy, not a Python object. The header
stamp, point count and size are read straight from the CDR bytes. Everything is written in plain CSV.
"""
import csv
import json
import os
import signal
import struct
import sys
import time

import numpy as np

sys.path.insert(0, os.path.expanduser('~/ros2_ws/src/landfill_camera_sim/scripts'))
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2  # noqa: E402

import cloud_io  # noqa: E402

TOPICS = {
    'lidar_raw': ('/livox/lidar', PointCloud2),
    'lidar_restamped': ('/landfill/livox/points', PointCloud2),
    'rgb': ('/landfill/rgb/image_raw', Image),
    'thermal': ('/landfill/thermal/image_raw', Image),
    'fused': ('/landfill/fusion/points_rgbt', PointCloud2),
    'merged': ('/landfill/fusion/points_merged', PointCloud2),
    'caminfo_rgb': ('/landfill/rgb/camera_info', CameraInfo),
    'caminfo_thermal': ('/landfill/thermal/camera_info', CameraInfo),
}


def parse_header(data, with_size=True):
    """Return (stamp_seconds, n) from serialised bytes (CDR little-endian); n = points for clouds, pixels for images."""
    if len(data) < 24 or data[1] != 1:
        return 0.0, 0
    sec, nsec = struct.unpack_from('<iI', data, 4)
    stamp = sec + nsec * 1e-9
    if not with_size:
        return stamp, 0
    flen = struct.unpack_from('<I', data, 12)[0]
    off = (16 + flen + 3) & ~3
    try:
        height, width = struct.unpack_from('<II', data, off)
    except struct.error:
        return stamp, 0
    return stamp, int(height) * int(width)


def plane_fit(xyz, rng):
    """Dominant plane (RANSAC on a subsample): returns (distance_m, nx, ny, nz, inliers, rms_m) or None."""
    pts = xyz[np.isfinite(xyz).all(axis=1)]
    pts = pts[np.linalg.norm(pts, axis=1) > 1.0]
    if len(pts) < 500:
        return None
    sub = pts[rng.choice(len(pts), min(4000, len(pts)), replace=False)]
    best = None
    for _ in range(150):
        s = sub[rng.choice(len(sub), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        d = -n @ s[0]
        inl = np.abs(sub @ n + d) < 0.02
        if best is None or inl.sum() > best[0]:
            best = (inl.sum(), n, d)
    _, n, d = best
    inl = np.abs(pts @ n + d) < 0.02
    q = pts[inl]
    c = q.mean(axis=0)
    n = np.linalg.svd(q - c, full_matrices=False)[2][2]
    d = -n @ c
    res = q @ n + d
    if d < 0:
        n, d = -n, -d
    return float(d), float(n[0]), float(n[1]), float(n[2]), int(inl.sum()), float(np.sqrt(np.mean(res ** 2)))


RGB_DIR = os.environ.get('PROBE_RGB_DIR', '')            # off unless set: the Run 1 test sets it to keep one RGB frame per minute
RGB_EVERY = float(os.environ.get('PROBE_RGB_EVERY', '60'))


class Logger(Node):
    def __init__(self, outdir, status_path):
        super().__init__('testwithlid_probe')
        os.makedirs(outdir, exist_ok=True)
        self.status_path = status_path
        self.files, self.writers, self.recent = {}, {}, {}
        self.counts = {}
        self.rng = np.random.default_rng(0)
        for key, (topic, typ) in TOPICS.items():
            f = open(os.path.join(outdir, f'topic_{key}.csv'), 'a', newline='')
            w = csv.writer(f)
            if f.tell() == 0:
                w.writerow(['t_arrival', 'stamp', 'n', 'nbytes'])
            self.files[key], self.writers[key], self.recent[key], self.counts[key] = f, w, [], 0
            self.create_subscription(typ, topic, self._make_cb(key), qos_profile_sensor_data, raw=True)
        self.create_subscription(Imu, '/livox/imu', self._imu, qos_profile_sensor_data, raw=True)
        self.imu_n = 0
        self.imu_f = open(os.path.join(outdir, 'topic_imu_per_second.csv'), 'a', newline='')
        self.imu_w = csv.writer(self.imu_f)
        if self.imu_f.tell() == 0:
            self.imu_w.writerow(['t', 'messages_in_last_second'])
        self.plane_f = open(os.path.join(outdir, 'lidar_plane.csv'), 'a', newline='')
        self.plane_w = csv.writer(self.plane_f)
        if self.plane_f.tell() == 0:
            self.plane_w.writerow(['t_arrival', 'distance_m', 'nx', 'ny', 'nz', 'inliers', 'rms_m'])
        self.cov_f = open(os.path.join(outdir, 'coverage_raw.csv'), 'a', newline='')
        self.cov_w = csv.writer(self.cov_f)
        if self.cov_f.tell() == 0:
            self.cov_w.writerow(['t_arrival', 'n_points', 'pct_rgb', 'pct_thermal', 'pct_both', 'pct_neither'])
        self.create_timer(2.0, self._flush)
        self.create_timer(1.0, self._imu_tick)
        self.create_timer(5.0, self._status)

    def _make_cb(self, key):
        def cb(data):
            now = time.time()
            stamp, n = parse_header(data)
            self.writers[key].writerow([f'{now:.4f}', f'{stamp:.4f}', n, len(data)])
            self.counts[key] += 1
            r = self.recent[key]
            r.append(now)
            if key == 'rgb' and RGB_DIR and now - getattr(self, '_last_jpg', 0.0) >= RGB_EVERY:
                self._last_jpg = now
                self._save_rgb(now, data)
            if key == 'lidar_restamped' and self.counts[key] % 10 == 0:
                self._plane(now, data)
            if key == 'fused' and self.counts[key] % 5 == 0:
                self._coverage(now, data)
        return cb

    def _save_rgb(self, now, data):
        try:
            import cv2
            msg = deserialize_message(data, Image)
            a = np.frombuffer(msg.data, np.uint8)[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            os.makedirs(RGB_DIR, exist_ok=True)
            cv2.imwrite(os.path.join(RGB_DIR, f'rgb_{int(now)}.jpg'), cv2.cvtColor(a, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
        except Exception:
            pass

    def _plane(self, now, data):
        try:
            msg = deserialize_message(data, PointCloud2)
            xyz = cloud_io.read_fields(msg, ('x', 'y', 'z'), dtype=np.float32)
            res = plane_fit(xyz.astype(np.float64), self.rng)
            if res:
                self.plane_w.writerow([f'{now:.3f}'] + [f'{v:.5f}' if isinstance(v, float) else v for v in res])
        except Exception:
            pass

    def _coverage(self, now, data):
        try:
            msg = deserialize_message(data, PointCloud2)
            arr = cloud_io.read_fields(msg, cloud_io.XYZIRT_FIELDS, dtype=np.float32)
            n = arr.shape[0]
            if n == 0:
                return
            rgb = arr[:, 4].view(np.uint32) != 0
            th = np.isfinite(arr[:, 5])
            both = rgb & th
            neither = ~(rgb | th)
            self.cov_w.writerow([f'{now:.3f}', n] + [f'{100.0 * float(x.mean()):.2f}' for x in (rgb, th, both, neither)])
        except Exception:
            pass

    def _imu(self, data):
        self.imu_n += 1

    def _imu_tick(self):
        self.imu_w.writerow([f'{time.time():.2f}', self.imu_n])
        self.imu_n = 0

    def _flush(self):
        for f in list(self.files.values()) + [self.imu_f, self.plane_f, self.cov_f]:
            f.flush()

    def _status(self):
        now = time.time()
        out = {'t': now}
        for key, r in self.recent.items():
            self.recent[key] = [t for t in r if now - t < 10.0]
            out[key] = {'hz_10s': round(len(self.recent[key]) / 10.0, 2), 'last_s_ago': round(now - r[-1], 1) if r else None,
                        'total': self.counts[key]}
        tmp = self.status_path + '.tmp'
        with open(tmp, 'w') as fh:
            json.dump(out, fh)
        os.replace(tmp, self.status_path)


def run_log(outdir, status_path):
    rclpy.init()
    node = Logger(outdir, status_path)
    stop = {'now': False}
    signal.signal(signal.SIGTERM, lambda *a: stop.update(now=True))
    signal.signal(signal.SIGINT, lambda *a: stop.update(now=True))
    while not stop['now'] and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    node._flush()
    node.destroy_node()
    rclpy.shutdown()


def run_capture(out_npz, spec, max_secs):
    """Collect real messages. spec items: lidar:N fused:N merged:N rgb:N thermal:N caminfo:1 (N messages each)."""
    want = {k: int(v) for k, v in (item.split(':') for item in spec.split(','))}
    rclpy.init()
    node = Node('testwithlid_capture')
    got = {k: [] for k in want}
    topic_of = {'lidar': ('/landfill/livox/points', PointCloud2), 'fused': ('/landfill/fusion/points_rgbt', PointCloud2),
                'merged': ('/landfill/fusion/points_merged', PointCloud2), 'rgb': ('/landfill/rgb/image_raw', Image),
                'thermal': ('/landfill/thermal/image_raw', Image), 'caminfo': ('/landfill/rgb/camera_info', CameraInfo)}

    def make(key):
        def cb(data):
            if len(got[key]) < want[key]:
                got[key].append((time.time(), data))
        return cb
    for key in want:
        topic, typ = topic_of[key]
        node.create_subscription(typ, topic, make(key), qos_profile_sensor_data, raw=True)
    t0 = time.time()
    while time.time() - t0 < max_secs and any(len(got[k]) < want[k] for k in want):
        rclpy.spin_once(node, timeout_sec=0.2)
    out = {'meta_counts': json.dumps({k: len(v) for k, v in got.items()})}
    for key in ('lidar', 'fused', 'merged'):
        if key in got and got[key]:
            xyz_all, rest_all, offs, stamps, arr_t = [], [], [0], [], []
            for t_arr, data in got[key]:
                msg = deserialize_message(data, PointCloud2)
                if key == 'lidar':
                    a = cloud_io.read_fields(msg, ('x', 'y', 'z'), dtype=np.float32)
                    xyz_all.append(a[:, :3])
                else:
                    a = cloud_io.read_fields(msg, cloud_io.XYZIRT_FIELDS, dtype=np.float32)
                    xyz_all.append(a[:, :3])
                    rest_all.append(a[:, 3:6])
                offs.append(offs[-1] + a.shape[0])
                stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                arr_t.append(t_arr)
            out[f'{key}_xyz'] = np.concatenate(xyz_all)
            out[f'{key}_offsets'] = np.array(offs)
            out[f'{key}_stamps'] = np.array(stamps)
            out[f'{key}_arrival'] = np.array(arr_t)
            if rest_all:
                out[f'{key}_rest'] = np.concatenate(rest_all)       # columns: intensity, rgb (float32 bits of uint32), thermal
    for key in ('rgb', 'thermal'):
        if key in got and got[key]:
            imgs, stamps = [], []
            for _t, data in got[key]:
                msg = deserialize_message(data, Image)
                a = np.frombuffer(msg.data, np.uint8 if key == 'rgb' else np.uint16)
                step = msg.step // (1 if key == 'rgb' else 2)
                a = a[:msg.height * step].reshape(msg.height, step)[:, :msg.width * (3 if key == 'rgb' else 1)]
                imgs.append(a.reshape(msg.height, msg.width, 3) if key == 'rgb' else a.reshape(msg.height, msg.width))
                stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                out[f'{key}_encoding'] = msg.encoding
            out[f'{key}_img'] = np.stack(imgs)
            out[f'{key}_stamps'] = np.array(stamps)
    if got.get('caminfo'):
        ci = deserialize_message(got['caminfo'][0][1], CameraInfo)
        out['caminfo_json'] = json.dumps({'k': list(ci.k), 'd': list(ci.d), 'p': list(ci.p), 'width': ci.width, 'height': ci.height,
                                          'model': ci.distortion_model, 'frame_id': ci.header.frame_id})
    np.savez_compressed(out_npz, **out)
    node.destroy_node()
    rclpy.shutdown()
    print(out['meta_counts'])


if __name__ == '__main__':
    if len(sys.argv) >= 4 and sys.argv[1] == 'log':
        run_log(sys.argv[2], sys.argv[3])
    elif len(sys.argv) >= 4 and sys.argv[1] == 'capture':
        run_capture(sys.argv[2], sys.argv[3], float(sys.argv[4]) if len(sys.argv) > 4 else 30.0)
    else:
        sys.exit(__doc__)
