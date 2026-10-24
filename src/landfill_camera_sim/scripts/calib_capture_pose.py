#!/usr/bin/env python3
"""One-shot capture for thermal/lidar calibration: run once per pose.

Subscribes to /landfill/livox/points and /landfill/thermal/image_raw,
accumulates ~2.5s of frames (reduces per-frame noise), then saves a .npz
with the raw lidar points (xyz) and the mean thermal frame (float32 Y,
0-255 range) to the given output path and exits.

Usage: calib_capture_pose.py <output.npz> [duration_seconds]
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

sys.path.insert(0, '/home/vish/ros2_ws/src/landfill_camera_sim/scripts')
import cloud_io


class CalibCaptureNode(Node):
    def __init__(self, duration):
        super().__init__('calib_capture_pose')
        self._duration = duration
        self._lidar_batches = []
        self._thermal_frames = []
        self.create_subscription(
            PointCloud2, '/landfill/livox/points', self._on_lidar, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, '/landfill/thermal/image_raw', self._on_thermal, qos_profile_sensor_data
        )
        self._start = time.monotonic()
        self.create_timer(0.1, self._check_done)

    def _on_lidar(self, msg):
        pts = cloud_io.read_fields(msg, cloud_io.XYZI_FIELDS, dtype=np.float32)
        if pts.shape[0]:
            self._lidar_batches.append(np.array(pts[:, 0:3]))

    def _on_thermal(self, msg):
        flat = np.frombuffer(msg.data, dtype=np.uint16)
        if flat.shape[0] >= msg.height * msg.width:
            img = flat[:msg.height * msg.width].reshape(msg.height, msg.width)
            self._thermal_frames.append((img.astype(np.float32) / 256.0))

    def _check_done(self):
        if time.monotonic() - self._start >= self._duration:
            rclpy.shutdown()


def main():
    if len(sys.argv) < 2:
        print('usage: calib_capture_pose.py <output.npz> [duration_seconds]')
        sys.exit(1)
    out_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5

    rclpy.init()
    node = CalibCaptureNode(duration)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass

    lidar_xyz = np.concatenate(node._lidar_batches, axis=0) if node._lidar_batches else np.zeros((0, 3), dtype=np.float32)
    thermal_mean = (
        np.mean(np.stack(node._thermal_frames, axis=0), axis=0)
        if node._thermal_frames else np.zeros((512, 640), dtype=np.float32)
    )
    np.savez(out_path, lidar_xyz=lidar_xyz, thermal_mean=thermal_mean)
    print(
        f'Saved {out_path}: {lidar_xyz.shape[0]} lidar points '
        f'({len(node._lidar_batches)} scans), '
        f'thermal mean over {len(node._thermal_frames)} frames'
    )


if __name__ == '__main__':
    main()
