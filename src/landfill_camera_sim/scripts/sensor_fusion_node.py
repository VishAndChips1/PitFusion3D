#!/usr/bin/env python3
"""Colorize lidar points with time-synced RGB + thermal camera samples.

Subscribes to one lidar point cloud (already resampled into the real Livox
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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, PointCloud2

import cloud_io


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


def lidar_to_camera_transform(extrinsic_frames, lidar_frame, camera_frame):
    """Collapse rig-frame hops into one (M, b) so points map with xyz @ M + b.

    extrinsic_frames maps child_frame -> (R, t) with p_reference = R @ p_child + t,
    so p_camera = R_cam.T @ (R_lidar @ p_lidar + t_lidar - t_cam), which is
    the same as p_lidar @ (R_lidar.T @ R_cam) + (t_lidar - t_cam) @ R_cam.
    """
    r_lidar, t_lidar = extrinsic_frames.get(lidar_frame, (np.eye(3), np.zeros(3)))
    r_cam, t_cam = extrinsic_frames.get(camera_frame, (np.eye(3), np.zeros(3)))
    m = (r_lidar.T @ r_cam).astype(np.float32)
    b = ((t_lidar - t_cam) @ r_cam).astype(np.float32)
    return m, b


def project_to_pixels(points_camera_frame, fx, fy, cx, cy, width, height):
    """Rig/robot axis convention (x forward, y left, z up) -> pixel coords."""
    x = points_camera_frame[:, 0]
    y = points_camera_frame[:, 1]
    z = points_camera_frame[:, 2]
    in_front = x > 0.05
    inv_x = np.zeros_like(x)
    np.divide(1.0, x, out=inv_x, where=in_front)
    ui = np.rint(fx * (-y * inv_x) + cx).astype(np.int32)
    vi = np.rint(fy * (-z * inv_x) + cy).astype(np.int32)
    valid = in_front & (ui >= 0) & (ui < width) & (vi >= 0) & (vi < height)
    # Keep gathers in-bounds; `valid` decides which samples are actually used.
    np.clip(ui, 0, width - 1, out=ui)
    np.clip(vi, 0, height - 1, out=vi)
    return ui, vi, valid


def image_to_array(msg, channels, dtype):
    """View an Image message's buffer without copying, honouring msg.step."""
    itemsize = np.dtype(dtype).itemsize
    if msg.is_bigendian and itemsize > 1:
        dtype = np.dtype(dtype).newbyteorder('>')
    flat = np.frombuffer(msg.data, dtype=dtype)
    row_items = msg.step // itemsize
    if row_items <= 0 or flat.shape[0] < msg.height * row_items:
        raise ValueError('Image buffer smaller than height * step')
    rows = flat[:msg.height * row_items].reshape(msg.height, row_items)
    used = msg.width * channels
    return rows[:, :used].reshape(msg.height, msg.width, channels)


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
        lidar_frame = str(self.get_parameter('lidar_frame').value)
        rgb_frame = str(self.get_parameter('rgb_frame').value)
        thermal_frame = str(self.get_parameter('thermal_frame').value)
        self._thermal_min_temp = np.float32(self.get_parameter('thermal_min_temp').value)
        self._thermal_resolution = np.float32(self.get_parameter('thermal_resolution').value)
        sync_slop = float(self.get_parameter('sync_slop').value)
        queue_size = int(self.get_parameter('queue_size').value)

        self._reference_frame, extrinsics = _load_extrinsics(extrinsics_path)
        self._rgb_intrinsics = _load_camera_intrinsics(intrinsics_path, 'rgb_camera')
        self._thermal_intrinsics = _load_camera_intrinsics(intrinsics_path, 'thermal_camera')

        self._rgb_transform = lidar_to_camera_transform(extrinsics, lidar_frame, rgb_frame)
        self._thermal_transform = lidar_to_camera_transform(extrinsics, lidar_frame, thermal_frame)
        # Co-located rigs share one transform; skip the second projection.
        self._shared_transform = (
            np.array_equal(self._rgb_transform[0], self._thermal_transform[0])
            and np.array_equal(self._rgb_transform[1], self._thermal_transform[1])
        )

        # BEST_EFFORT throughout: this node's inputs (lidar/rgb/thermal,
        # each 10-30Hz) and output (fused cloud) are all live sensor
        # streams where a dropped message is superseded almost
        # immediately, so DDS's default RELIABLE ACK/retransmit tracking
        # is pure overhead here. Every publisher feeding these topics
        # (rgb_camera_node, thermal_camera_node, livox_restamp_node) and
        # every subscriber of this node's output (registration_merge_node,
        # fusion_web_streamer) must use a compatible QoS -- see their own
        # create_publisher/create_subscription calls.
        self._publisher = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self._lidar_sub = message_filters.Subscriber(
            self, PointCloud2, lidar_topic, qos_profile=qos_profile_sensor_data
        )
        self._rgb_sub = message_filters.Subscriber(
            self, Image, rgb_topic, qos_profile=qos_profile_sensor_data
        )
        self._thermal_sub = message_filters.Subscriber(
            self, Image, thermal_topic, qos_profile=qos_profile_sensor_data
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._lidar_sub, self._rgb_sub, self._thermal_sub],
            queue_size=queue_size,
            slop=sync_slop,
        )
        self._sync.registerCallback(self._on_synced)
        self._last_stamp = None
        self._lidar_sub.registerCallback(self._watch_clock)

        self.get_logger().info(
            'Fusing %s + %s + %s -> %s (slop %.3fs, shared rgb/thermal transform: %s)'
            % (lidar_topic, rgb_topic, thermal_topic, output_topic, sync_slop,
               self._shared_transform)
        )

    def _watch_clock(self, cloud_msg):
        """Drop cached messages when the clock rewinds.

        Restarting the simulator resets sim time; the synchronizer would
        otherwise keep matching against queued messages stamped in the
        future and never emit again.
        """
        stamp = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9
        if self._last_stamp is not None and stamp < self._last_stamp - 1.0:
            self.get_logger().warn(
                'Timestamps jumped backwards (%.3f < %.3f); clearing sync queues'
                % (stamp, self._last_stamp)
            )
            with self._sync.lock:
                for queue in self._sync.queues:
                    queue.clear()
        self._last_stamp = stamp

    def _on_synced(self, cloud_msg, rgb_msg, thermal_msg):
        points = cloud_io.read_fields(cloud_msg, cloud_io.XYZI_FIELDS, dtype=np.float32)
        if points.shape[0] == 0:
            return
        xyz = points[:, 0:3]
        n = xyz.shape[0]

        rgb_img = image_to_array(rgb_msg, 3, np.uint8)
        thermal_raw = image_to_array(thermal_msg, 1, np.uint16)[:, :, 0]

        rgb_intr = self._rgb_intrinsics
        m, b = self._rgb_transform
        ui, vi, valid_rgb = project_to_pixels(
            xyz @ m + b, rgb_intr['fx'], rgb_intr['fy'], rgb_intr['cx'], rgb_intr['cy'],
            rgb_intr['width'], rgb_intr['height'],
        )

        thermal_intr = self._thermal_intrinsics
        if self._shared_transform and thermal_intr == rgb_intr:
            ti, tj, valid_thermal = ui, vi, valid_rgb
        else:
            m, b = self._thermal_transform
            ti, tj, valid_thermal = project_to_pixels(
                xyz @ m + b, thermal_intr['fx'], thermal_intr['fy'],
                thermal_intr['cx'], thermal_intr['cy'],
                thermal_intr['width'], thermal_intr['height'],
            )

        cloud_out = np.empty((n, 6), dtype=np.float32)
        cloud_out[:, 0:3] = xyz
        cloud_out[:, 3] = points[:, 3]

        # Gather only at projected pixels: sampling ~2e4 points is far
        # cheaper than converting the whole ~1e6-pixel image per frame.
        packed = np.zeros(n, dtype=np.uint32)
        sampled_rgb = rgb_img[vi[valid_rgb], ui[valid_rgb]].astype(np.uint32)
        packed[valid_rgb] = (
            (sampled_rgb[:, 0] << 16) | (sampled_rgb[:, 1] << 8) | sampled_rgb[:, 2]
        )
        cloud_out[:, 4] = packed.view(np.float32)

        thermal_values = np.full(n, np.nan, dtype=np.float32)
        sampled_thermal = thermal_raw[tj[valid_thermal], ti[valid_thermal]]
        thermal_values[valid_thermal] = (
            sampled_thermal.astype(np.float32) * self._thermal_resolution + self._thermal_min_temp
        )
        cloud_out[:, 5] = thermal_values

        self._publisher.publish(
            cloud_io.make_cloud(cloud_msg.header, cloud_io.XYZIRT_FIELDS, cloud_out)
        )


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
