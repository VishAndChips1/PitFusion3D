#!/usr/bin/env python3
"""Colorize lidar points with time-synced RGB + thermal camera samples.

Subscribes to one lidar point cloud (already reshaped into the real Livox
Avia scan pattern by livox_pattern_filter.py) plus the RGB and thermal
images, time-synchronizes them, and republishes the same points with two
extra fields: a packed 'rgb' color sampled from the RGB image and a
'thermal' temperature (Kelvin) sampled from the thermal image, using the
pinhole projection defined by config/calibration/intrinsics.yaml and the
rig-to-sensor offsets in config/calibration/extrinsics.yaml.
"""

import numpy as np
import yaml

import message_filters
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


def _package_path(relative):
    return get_package_share_directory('landfill_camera_sim') + '/' + relative


def _load_extrinsics(path):
    with open(path, 'r', encoding='utf-8') as yaml_file:
        data = yaml.safe_load(yaml_file) or {}
    reference_frame = str(data.get('reference_frame', 'camera_platform/camera_link'))
    frames = {}
    for child_frame, transform in (data.get('transforms') or {}).items():
        translation = transform.get('translation', {})
        rotation = transform.get('rotation_rpy', {})
        t = np.array([
            float(translation.get('x', 0.0)),
            float(translation.get('y', 0.0)),
            float(translation.get('z', 0.0)),
        ])
        rpy = [
            float(rotation.get('roll', 0.0)),
            float(rotation.get('pitch', 0.0)),
            float(rotation.get('yaw', 0.0)),
        ]
        r = Rotation.from_euler('xyz', rpy).as_matrix()
        frames[str(child_frame)] = (r, t)
    return reference_frame, frames


def _load_camera_intrinsics(path, camera_key):
    with open(path, 'r', encoding='utf-8') as yaml_file:
        data = yaml.safe_load(yaml_file) or {}
    camera = data[camera_key]
    matrix = camera['camera_matrix']
    return {
        'width': int(camera['image_width']),
        'height': int(camera['image_height']),
        'fx': float(matrix['fx']),
        'fy': float(matrix['fy']),
        'cx': float(matrix['cx']),
        'cy': float(matrix['cy']),
    }


def _lidar_point_to_camera_frame(points_xyz, reference_frame, extrinsic_frames, lidar_frame, camera_frame):
    """Points published in the shared rig frame -> a specific camera's local frame.

    extrinsic_frames maps child_frame -> (R, t) such that
    p_reference = R @ p_child + t. Both lidar_frame and camera_frame default
    to identity (co-located sensors) unless overridden in extrinsics.yaml.
    """
    r_lidar, t_lidar = extrinsic_frames.get(lidar_frame, (np.eye(3), np.zeros(3)))
    r_cam, t_cam = extrinsic_frames.get(camera_frame, (np.eye(3), np.zeros(3)))
    p_reference = points_xyz @ r_lidar.T + t_lidar
    p_camera = (p_reference - t_cam) @ r_cam
    return p_camera


def _project_to_pixels(points_camera_frame, fx, fy, cx, cy, width, height):
    """Rig/robot axis convention (x forward, y left, z up) -> pixel coords."""
    x = points_camera_frame[:, 0]
    y = points_camera_frame[:, 1]
    z = points_camera_frame[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = fx * (-y / x) + cx
        v = fy * (-z / x) + cy
    ui = np.rint(np.nan_to_num(u, nan=-1.0)).astype(np.int32)
    vi = np.rint(np.nan_to_num(v, nan=-1.0)).astype(np.int32)
    valid = (x > 0.05) & (ui >= 0) & (ui < width) & (vi >= 0) & (vi < height)
    return ui, vi, valid


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        self.declare_parameter('lidar_points_topic', '/landfill/livox/points')
        self.declare_parameter('rgb_image_topic', '/landfill/rgb/image_raw')
        self.declare_parameter('thermal_image_topic', '/landfill/thermal/image_raw')
        self.declare_parameter('output_topic', '/landfill/fusion/points_rgbt')
        self.declare_parameter('extrinsics_path', _package_path('config/calibration/extrinsics.yaml'))
        self.declare_parameter('intrinsics_path', _package_path('config/calibration/intrinsics.yaml'))
        self.declare_parameter('lidar_frame', 'camera_platform/lidar_frame')
        self.declare_parameter('rgb_frame', 'camera_platform/rgb_optical_frame')
        self.declare_parameter('thermal_frame', 'camera_platform/thermal_optical_frame')
        self.declare_parameter('thermal_min_temp', 253.15)
        self.declare_parameter('thermal_resolution', 0.01)
        self.declare_parameter('sync_slop', 0.15)
        self.declare_parameter('queue_size', 5)

        lidar_topic = str(self.get_parameter('lidar_points_topic').value)
        rgb_topic = str(self.get_parameter('rgb_image_topic').value)
        thermal_topic = str(self.get_parameter('thermal_image_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        extrinsics_path = str(self.get_parameter('extrinsics_path').value)
        intrinsics_path = str(self.get_parameter('intrinsics_path').value)
        self._lidar_frame = str(self.get_parameter('lidar_frame').value)
        self._rgb_frame = str(self.get_parameter('rgb_frame').value)
        self._thermal_frame = str(self.get_parameter('thermal_frame').value)
        self._thermal_min_temp = float(self.get_parameter('thermal_min_temp').value)
        self._thermal_resolution = float(self.get_parameter('thermal_resolution').value)
        sync_slop = float(self.get_parameter('sync_slop').value)
        queue_size = int(self.get_parameter('queue_size').value)

        self._reference_frame, self._extrinsics = _load_extrinsics(extrinsics_path)
        self._rgb_intrinsics = _load_camera_intrinsics(intrinsics_path, 'rgb_camera')
        self._thermal_intrinsics = _load_camera_intrinsics(intrinsics_path, 'thermal_camera')
        self._bridge = CvBridge()

        self._publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self._lidar_sub = message_filters.Subscriber(self, PointCloud2, lidar_topic)
        self._rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self._thermal_sub = message_filters.Subscriber(self, Image, thermal_topic)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._lidar_sub, self._rgb_sub, self._thermal_sub],
            queue_size=queue_size,
            slop=sync_slop,
        )
        self._sync.registerCallback(self._on_synced)

        self.get_logger().info(
            'Fusing %s + %s + %s -> %s (slop %.3fs)'
            % (lidar_topic, rgb_topic, thermal_topic, output_topic, sync_slop)
        )

    def _thermal_to_kelvin(self, thermal_msg):
        raw = self._bridge.imgmsg_to_cv2(thermal_msg, desired_encoding='passthrough')
        if np.issubdtype(raw.dtype, np.integer):
            return raw.astype(np.float32) * self._thermal_resolution + self._thermal_min_temp
        return raw.astype(np.float32)

    def _on_synced(self, cloud_msg, rgb_msg, thermal_msg):
        points = pc2.read_points_numpy(cloud_msg, field_names=['x', 'y', 'z', 'intensity'])
        if points.size == 0:
            return
        xyz = points[:, 0:3].astype(np.float64)
        intensity = points[:, 3].astype(np.float32)

        rgb_img = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
        thermal_k = self._thermal_to_kelvin(thermal_msg)

        rgb_intr = self._rgb_intrinsics
        points_rgb_frame = _lidar_point_to_camera_frame(
            xyz, self._reference_frame, self._extrinsics, self._lidar_frame, self._rgb_frame
        )
        ui, vi, valid_rgb = _project_to_pixels(
            points_rgb_frame, rgb_intr['fx'], rgb_intr['fy'], rgb_intr['cx'], rgb_intr['cy'],
            rgb_intr['width'], rgb_intr['height'],
        )

        thermal_intr = self._thermal_intrinsics
        points_thermal_frame = _lidar_point_to_camera_frame(
            xyz, self._reference_frame, self._extrinsics, self._lidar_frame, self._thermal_frame
        )
        ti, tj, valid_thermal = _project_to_pixels(
            points_thermal_frame, thermal_intr['fx'], thermal_intr['fy'],
            thermal_intr['cx'], thermal_intr['cy'], thermal_intr['width'], thermal_intr['height'],
        )

        n = xyz.shape[0]
        colors = np.zeros((n, 3), dtype=np.uint8)
        colors[valid_rgb] = rgb_img[vi[valid_rgb], ui[valid_rgb]]
        packed_rgb = (
            (colors[:, 0].astype(np.uint32) << 16)
            | (colors[:, 1].astype(np.uint32) << 8)
            | colors[:, 2].astype(np.uint32)
        ).view(np.float32)

        thermal_values = np.full(n, np.nan, dtype=np.float32)
        thermal_values[valid_thermal] = thermal_k[tj[valid_thermal], ti[valid_thermal]]

        cloud_out = np.empty((n, 6), dtype=np.float32)
        cloud_out[:, 0:3] = xyz.astype(np.float32)
        cloud_out[:, 3] = intensity
        cloud_out[:, 4] = packed_rgb
        cloud_out[:, 5] = thermal_values

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='thermal', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        out_msg = pc2.create_cloud(cloud_msg.header, fields, cloud_out)
        self._publisher.publish(out_msg)


def main():
    rclpy.init()
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
