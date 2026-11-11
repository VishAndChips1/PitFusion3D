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

import csv
import os
import time

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
        # If no thermal frame has arrived for this long, keep publishing
        # fused clouds from lidar + RGB alone (thermal field = NaN, which
        # the viewer already draws as grey) instead of going silent. The
        # 3-way synchroniser below never fires without thermal, so a
        # flaky/absent thermal USB camera used to blank the whole viewer.
        self.declare_parameter('thermal_timeout_s', 1.0)

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
        self._thermal_timeout = float(self.get_parameter('thermal_timeout_s').value)
        self._last_full_mono = None  # last time a 3-way (lidar+rgb+thermal) fusion succeeded
        self._start_mono = time.monotonic()
        self._degraded = False
        self._last_fused_stamp = None

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
        self._sync.registerCallback(self._on_synced_full)
        self._sync_no_thermal = message_filters.ApproximateTimeSynchronizer(
            [self._lidar_sub, self._rgb_sub],
            queue_size=queue_size,
            slop=sync_slop,
        )
        self._sync_no_thermal.registerCallback(self._on_synced_no_thermal)
        self._last_stamp = None
        self._lidar_sub.registerCallback(self._watch_clock)

        # Off by default (PF3D_TIMING env var, per the campaign's own
        # ground rule that instrumentation must not change default
        # behaviour): per-stage wall-clock timing of _process_synced,
        # appended as one CSV row per processed triple. B2-d in the
        # 2026-11-11 campaign measures this writer's own overhead by
        # comparing rates/CPU with and without it.
        self._timing = os.environ.get('PF3D_TIMING') == '1'
        self._timing_writer = None
        if self._timing:
            timing_path = os.environ.get(
                'PF3D_TIMING_PATH',
                os.path.join(os.path.expanduser('~'), 'thesis_tests', 'fusion_timing.csv'),
            )
            os.makedirs(os.path.dirname(timing_path), exist_ok=True)
            write_header = not os.path.exists(timing_path)
            self._timing_file = open(timing_path, 'a', newline='')
            self._timing_writer = csv.writer(self._timing_file)
            if write_header:
                self._timing_writer.writerow([
                    'wall_time', 'cloud_stamp', 'rgb_stamp', 'thermal_stamp',
                    'pairing_gap_rgb_ms', 'pairing_gap_thermal_ms',
                    'sync_wait_ms', 'decode_ms', 'project_ms', 'colour_ms',
                    'publish_ms', 'total_ms', 'n_points',
                ])
                self._timing_file.flush()
            self.get_logger().info(f'PF3D_TIMING=1 -- writing per-triple stage timing to {timing_path}')

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
            self._last_fused_stamp = None
            self.get_logger().warn(
                'Timestamps jumped backwards (%.3f < %.3f); clearing sync queues'
                % (stamp, self._last_stamp)
            )
            with self._sync.lock:
                for queue in self._sync.queues:
                    queue.clear()
            with self._sync_no_thermal.lock:
                for queue in self._sync_no_thermal.queues:
                    queue.clear()
        self._last_stamp = stamp

    def _on_synced_full(self, cloud_msg, rgb_msg, thermal_msg):
        self._last_full_mono = time.monotonic()
        if self._degraded:
            self._degraded = False
            self.get_logger().info('Thermal fusion working again -- resuming full RGB+thermal fusion')
        self._on_synced(cloud_msg, rgb_msg, thermal_msg)

    def _thermal_stale(self):
        # Keyed on "3-way fusion hasn't succeeded", not on "no thermal frame
        # arrived": thermal frames that arrive but can't be time-matched to
        # the lidar (clock skew/step) would otherwise leave the viewer blank.
        ref = max(self._last_full_mono or 0.0, self._start_mono)
        return time.monotonic() - ref > self._thermal_timeout

    def _on_synced_no_thermal(self, cloud_msg, rgb_msg):
        if not self._thermal_stale():
            return  # normal 3-way path handles it
        if not self._degraded:
            self._degraded = True
            self.get_logger().warn(
                f'No lidar+RGB+thermal fusion for >{self._thermal_timeout:.0f}s (thermal missing or '
                f'not time-matched) -- fusing lidar + RGB only (thermal = NaN) until it returns')
        self._on_synced(cloud_msg, rgb_msg, None)

    def _on_synced(self, cloud_msg, rgb_msg, thermal_msg):
        # Defensive: this callback runs continuously on live sensor data,
        # and nothing here was previously caught -- an unexpected error
        # (a malformed image buffer from a USB/CSI hiccup, a degenerate
        # projection, etc.) would propagate out of this subscription
        # callback and kill the entire node process, matching a gap found
        # the same way in registration_merge_node.py's window-publish
        # step (see its comment). systemd's Restart=on-failure is a
        # backstop, but skipping one bad synced triple and continuing is
        # far less disruptive than a full node restart.
        stamp_s = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9
        if self._last_fused_stamp is not None and stamp_s <= self._last_fused_stamp:
            return  # this scan was already fused via the other path
        self._last_fused_stamp = stamp_s
        try:
            self._process_synced(cloud_msg, rgb_msg, thermal_msg)
        except Exception as exc:
            self.get_logger().error(f'Fusion of synced triple failed, skipping: {exc!r}')

    def _process_synced(self, cloud_msg, rgb_msg, thermal_msg):
        timing = self._timing
        if timing:
            wall_entry = time.time()
            t_entry = time.perf_counter()

        points = cloud_io.read_fields(cloud_msg, cloud_io.XYZI_FIELDS, dtype=np.float32)
        if points.shape[0] == 0:
            return
        xyz = points[:, 0:3]
        # Root-cause fix, not just a symptom-level catch: a lidar "no
        # return" is sometimes represented as NaN/Inf rather than being
        # omitted from the cloud entirely, depending on the driver. Left
        # in, a NaN/Inf point survives the 1/x division in
        # project_to_pixels (silently becoming an out-of-bounds/invalid
        # sample, harmless here) but would propagate downstream into
        # registration_merge_node's ICP/SVD (xyz.min/max, Kabsch) and
        # fusion_web_streamer's quantization (xyz.min/max, scale factor),
        # where a single non-finite value can corrupt an entire cloud's
        # worth of output (e.g. scale becomes NaN, every point's
        # quantized position becomes garbage) rather than just one point.
        # Filtering once here, at the earliest point raw lidar data enters
        # this pipeline, protects every downstream consumer at once.
        finite = np.isfinite(xyz).all(axis=1)
        if not finite.all():
            points = points[finite]
            xyz = points[:, 0:3]
        n = xyz.shape[0]
        if n == 0:
            return

        rgb_img = image_to_array(rgb_msg, 3, np.uint8)
        thermal_raw = (None if thermal_msg is None
                       else image_to_array(thermal_msg, 1, np.uint16)[:, :, 0])
        if timing:
            t_decode = time.perf_counter()

        rgb_intr = self._rgb_intrinsics
        m, b = self._rgb_transform
        ui, vi, valid_rgb = project_to_pixels(
            xyz @ m + b, rgb_intr['fx'], rgb_intr['fy'], rgb_intr['cx'], rgb_intr['cy'],
            rgb_intr['width'], rgb_intr['height'],
        )

        thermal_intr = self._thermal_intrinsics
        if thermal_msg is None:
            ti = tj = valid_thermal = None
        elif self._shared_transform and thermal_intr == rgb_intr:
            ti, tj, valid_thermal = ui, vi, valid_rgb
        else:
            m, b = self._thermal_transform
            ti, tj, valid_thermal = project_to_pixels(
                xyz @ m + b, thermal_intr['fx'], thermal_intr['fy'],
                thermal_intr['cx'], thermal_intr['cy'],
                thermal_intr['width'], thermal_intr['height'],
            )
        if timing:
            t_project = time.perf_counter()

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
        if thermal_raw is not None:
            sampled_thermal = thermal_raw[tj[valid_thermal], ti[valid_thermal]]
            thermal_values[valid_thermal] = (
                sampled_thermal.astype(np.float32) * self._thermal_resolution + self._thermal_min_temp
            )
        cloud_out[:, 5] = thermal_values
        if timing:
            t_colour = time.perf_counter()

        self._publisher.publish(
            cloud_io.make_cloud(cloud_msg.header, cloud_io.XYZIRT_FIELDS, cloud_out)
        )

        if timing:
            t_publish = time.perf_counter()
            cloud_stamp = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9
            rgb_stamp = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
            thermal_stamp = (float('nan') if thermal_msg is None
                             else thermal_msg.header.stamp.sec + thermal_msg.header.stamp.nanosec * 1e-9)
            self._timing_writer.writerow([
                f'{wall_entry:.6f}', f'{cloud_stamp:.6f}', f'{rgb_stamp:.6f}', f'{thermal_stamp:.6f}',
                f'{abs(rgb_stamp - cloud_stamp) * 1000:.3f}',
                f'{abs(thermal_stamp - cloud_stamp) * 1000:.3f}',
                '',  # sync_wait_ms: needs per-topic arrival time outside this callback; left blank, see B2 README
                f'{(t_decode - t_entry) * 1000:.3f}',
                f'{(t_project - t_decode) * 1000:.3f}',
                f'{(t_colour - t_project) * 1000:.3f}',
                f'{(t_publish - t_colour) * 1000:.3f}',
                f'{(t_publish - t_entry) * 1000:.3f}',
                n,
            ])
            self._timing_file.flush()


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
