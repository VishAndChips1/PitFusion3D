#!/usr/bin/env python3
"""Resample a gz-sim gpu_lidar grid into the real Livox Avia scan pattern.

gz-sim's gpu_lidar publishes a uniform horizontal_samples x vertical_samples
grid of raycast returns. The Avia is not a uniform-grid sensor: two Risley
prisms sweep a non-repetitive rosette across a 70.4 x 77.2 deg FOV, so the
grid is treated here purely as a *range source*.

For every row of the Avia pattern table this node takes that row's exact
azimuth/zenith, bilinearly interpolates a range out of the grid's range
image at those angles, and rebuilds the point along the pattern's own unit
direction. Snapping each point to the nearest grid cell instead (the
previous behaviour) quantised every return onto one of only
vertical_samples distinct elevations, which renders as horizontal banding
that looks like a multi-beam 2D scanner rather than a Livox rosette.

Angle convention follows LivoxPointsPlugin::convertDataToRotateInfo:
azimuth is used as-is and zenith is shifted by -90 deg, giving a ray
direction of Rz(azimuth) @ Ry(zenith) @ x_hat, i.e.
    x = cos(zenith) cos(azimuth)
    y = cos(zenith) sin(azimuth)
    z = -sin(zenith)
so a direction's elevation is exactly -zenith and its azimuth is azimuth,
independent of one another -- which is what makes the (azimuth, elevation)
grid interpolation below valid.
"""

import hashlib
import math
import os
from pathlib import Path

import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

import cloud_io


def _pattern_cache_path(csv_path):
    digest = hashlib.md5(csv_path.encode('utf-8')).hexdigest()[:12]
    cache_dir = Path(os.environ.get('ROS_HOME', Path.home() / '.ros')) / 'landfill_camera_sim'
    cache_dir.mkdir(parents=True, exist_ok=True)
    return cache_dir / ('avia_pattern_%s.npy' % digest)


def _load_scan_pattern(csv_path):
    """Load (azimuth, zenith) radians from a Livox-SDK scan_mode CSV.

    The Avia table is ~960k rows / 34 MB; parsing it costs seconds, so the
    converted array is cached as .npy and reused whenever it is newer than
    the CSV.
    """
    cache_path = _pattern_cache_path(csv_path)
    try:
        if cache_path.exists() and cache_path.stat().st_mtime >= os.path.getmtime(csv_path):
            cached = np.load(cache_path)
            if cached.ndim == 2 and cached.shape[1] == 2 and cached.shape[0] > 0:
                return cached[:, 0], cached[:, 1]
    except Exception:
        pass

    raw = np.loadtxt(csv_path, delimiter=',', skiprows=1, usecols=(1, 2), dtype=np.float64)
    if raw.ndim != 2 or raw.shape[0] == 0:
        raise ValueError('Scan pattern CSV %s contained no data rows' % csv_path)

    table = np.empty((raw.shape[0], 2), dtype=np.float32)
    table[:, 0] = np.radians(raw[:, 1 - 1])          # azimuth
    table[:, 1] = np.radians(raw[:, 2 - 1]) - math.pi / 2.0   # zenith, shifted
    try:
        np.save(cache_path, table)
    except Exception:
        pass
    return table[:, 0], table[:, 1]


class LivoxPatternFilter(Node):
    def __init__(self):
        super().__init__('livox_pattern_filter')

        default_csv = ''
        try:
            default_csv = (
                get_package_share_directory('landfill_camera_sim')
                + '/config/scan_patterns/avia.csv'
            )
        except Exception:
            pass

        self.declare_parameter('input_points_topic', '/landfill/livox/raw_scan/points')
        self.declare_parameter('output_points_topic', '/landfill/livox/points')
        self.declare_parameter('frame_id', 'camera_platform/camera_link')
        self.declare_parameter('horizontal_samples', 200)
        self.declare_parameter('vertical_samples', 120)
        self.declare_parameter('points_per_scan', 24000)
        self.declare_parameter('downsample', 1)
        self.declare_parameter('horizontal_min_angle', -0.614356)
        self.declare_parameter('horizontal_max_angle', 0.614356)
        self.declare_parameter('vertical_min_angle', -0.673697)
        self.declare_parameter('vertical_max_angle', 0.673697)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 200.0)
        self.declare_parameter('scan_pattern_csv', default_csv)
        # Blending two returns straddling a depth edge invents a point in
        # empty space, so fall back to nearest-neighbour when the 4 grid
        # neighbours disagree by more than this many metres.
        self.declare_parameter('interpolation_depth_tolerance', 0.5)

        self.input_points_topic = str(self.get_parameter('input_points_topic').value)
        self.output_points_topic = str(self.get_parameter('output_points_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.horizontal_samples = int(self.get_parameter('horizontal_samples').value)
        self.vertical_samples = int(self.get_parameter('vertical_samples').value)
        self.points_per_scan = int(self.get_parameter('points_per_scan').value)
        self.downsample = max(1, int(self.get_parameter('downsample').value))
        self.horizontal_min_angle = float(self.get_parameter('horizontal_min_angle').value)
        self.horizontal_max_angle = float(self.get_parameter('horizontal_max_angle').value)
        self.vertical_min_angle = float(self.get_parameter('vertical_min_angle').value)
        self.vertical_max_angle = float(self.get_parameter('vertical_max_angle').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.depth_tolerance = float(self.get_parameter('interpolation_depth_tolerance').value)
        scan_pattern_csv = str(self.get_parameter('scan_pattern_csv').value)

        if not scan_pattern_csv:
            raise RuntimeError(
                'scan_pattern_csv is empty and the packaged avia.csv could not be located'
            )
        azimuth, zenith = _load_scan_pattern(scan_pattern_csv)
        self.pattern_size = azimuth.shape[0]

        elevation = -zenith
        cos_zenith = np.cos(zenith)
        self.pattern_dirs = np.empty((self.pattern_size, 3), dtype=np.float32)
        self.pattern_dirs[:, 0] = cos_zenith * np.cos(azimuth)
        self.pattern_dirs[:, 1] = cos_zenith * np.sin(azimuth)
        self.pattern_dirs[:, 2] = -np.sin(zenith)

        # Fractional grid coordinates of every pattern ray. Kept as float32
        # (~7.7 MB for the Avia table) and indexed per scan; the 4-neighbour
        # indices and weights are derived for the sampled rows only.
        self.pattern_col = self._angle_to_grid_coord(
            azimuth, self.horizontal_min_angle, self.horizontal_max_angle, self.horizontal_samples
        )
        self.pattern_row = self._angle_to_grid_coord(
            elevation, self.vertical_min_angle, self.vertical_max_angle, self.vertical_samples
        )
        # Pattern rays outside the grid's FOV have no range to sample.
        self.pattern_in_fov = (
            (azimuth >= self.horizontal_min_angle) & (azimuth <= self.horizontal_max_angle)
            & (elevation >= self.vertical_min_angle) & (elevation <= self.vertical_max_angle)
        )

        self.scan_offsets = np.arange(0, self.points_per_scan, self.downsample, dtype=np.int64)
        self.pattern_index = 0
        self.expected_points = self.horizontal_samples * self.vertical_samples
        self.warned_grid_shape = False
        self._row_ascending = None

        self.publisher = self.create_publisher(PointCloud2, self.output_points_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_points_topic,
            self._on_points,
            10,
        )

        self.get_logger().info(
            'Publishing Livox Avia pattern %s -> %s (%d points/scan, %d-row scan table from %s, '
            'true-angle range interpolation)'
            % (
                self.input_points_topic,
                self.output_points_topic,
                self.points_per_scan // self.downsample,
                self.pattern_size,
                scan_pattern_csv,
            )
        )

    @staticmethod
    def _angle_to_grid_coord(angle, min_angle, max_angle, sample_count):
        """Map an angle to a fractional grid coordinate in [0, sample_count-1]."""
        if sample_count <= 1 or max_angle <= min_angle:
            return np.zeros_like(angle, dtype=np.float32)
        normalized = (angle - min_angle) / (max_angle - min_angle)
        return (np.clip(normalized, 0.0, 1.0) * (sample_count - 1)).astype(np.float32)

    def _detect_row_direction(self, ranges, grid_z):
        """Determine whether grid row 0 is the bottom or the top of the FOV."""
        valid = np.isfinite(ranges) & (ranges > 0.0)
        rows = np.arange(self.vertical_samples)
        mean_z = np.full(self.vertical_samples, np.nan, dtype=np.float64)
        for r in rows:
            row_valid = valid[r]
            if row_valid.any():
                mean_z[r] = grid_z[r][row_valid].mean()
        known = np.isfinite(mean_z)
        if known.sum() < 2:
            return True
        first, last = rows[known][0], rows[known][-1]
        return bool(mean_z[last] >= mean_z[first])

    def _on_points(self, cloud_msg):
        grid_point_count = cloud_msg.width * cloud_msg.height
        if grid_point_count != self.expected_points and not self.warned_grid_shape:
            self.warned_grid_shape = True
            self.get_logger().warn(
                'Expected a %dx%d (%d point) lidar grid, got %dx%d (%d points); '
                'indices may not line up with the configured FOV.'
                % (
                    self.horizontal_samples, self.vertical_samples, self.expected_points,
                    cloud_msg.width, cloud_msg.height, grid_point_count,
                )
            )
        if grid_point_count < self.expected_points:
            return

        view = cloud_io.structured_view(cloud_msg)
        gx = view['x'][:self.expected_points].astype(np.float32)
        gy = view['y'][:self.expected_points].astype(np.float32)
        gz = view['z'][:self.expected_points].astype(np.float32)

        ranges = np.sqrt(gx * gx + gy * gy + gz * gz)
        ranges[~np.isfinite(ranges)] = np.nan
        ranges = ranges.reshape(self.vertical_samples, self.horizontal_samples)

        if self._row_ascending is None:
            self._row_ascending = self._detect_row_direction(
                ranges, gz.reshape(self.vertical_samples, self.horizontal_samples)
            )
            self.get_logger().info(
                'Lidar grid row 0 is the %s of the vertical FOV'
                % ('bottom' if self._row_ascending else 'top')
            )

        start_index = self.pattern_index
        self.pattern_index = (self.pattern_index + self.points_per_scan) % self.pattern_size
        rows = (start_index + self.scan_offsets) % self.pattern_size

        col_f = self.pattern_col[rows]
        row_f = self.pattern_row[rows]
        if not self._row_ascending:
            row_f = (self.vertical_samples - 1) - row_f

        sampled_range = self._sample_range(ranges, col_f, row_f)

        valid = (
            self.pattern_in_fov[rows]
            & np.isfinite(sampled_range)
            & (sampled_range >= self.range_min)
            & (sampled_range <= self.range_max)
        )
        if not valid.any():
            return

        kept_rows = rows[valid]
        kept_range = sampled_range[valid].astype(np.float32)
        xyz = self.pattern_dirs[kept_rows] * kept_range[:, None]

        n = xyz.shape[0]
        payload = np.empty((n, 4), dtype=np.float32)
        payload[:, 0:3] = xyz
        # The Avia reports reflectivity; gz-sim's grid carries no usable
        # per-ray intensity, so the pattern row index stands in for it.
        payload[:, 3] = (kept_rows % 255).astype(np.float32)

        header = cloud_msg.header
        header.frame_id = self.frame_id
        self.publisher.publish(cloud_io.make_cloud(header, cloud_io.XYZI_FIELDS, payload))

    def _sample_range(self, ranges, col_f, row_f):
        """Bilinearly interpolate the range image at fractional grid coords."""
        h_max = self.horizontal_samples - 1
        v_max = self.vertical_samples - 1

        c0 = np.floor(col_f).astype(np.int32)
        r0 = np.floor(row_f).astype(np.int32)
        np.clip(c0, 0, h_max, out=c0)
        np.clip(r0, 0, v_max, out=r0)
        c1 = np.minimum(c0 + 1, h_max)
        r1 = np.minimum(r0 + 1, v_max)

        wc = (col_f - c0).astype(np.float32)
        wr = (row_f - r0).astype(np.float32)

        q00 = ranges[r0, c0]
        q01 = ranges[r0, c1]
        q10 = ranges[r1, c0]
        q11 = ranges[r1, c1]

        neighbours = np.stack((q00, q01, q10, q11), axis=1)
        finite = np.isfinite(neighbours)
        finite_count = finite.sum(axis=1)
        # Zero-filled copy keeps the reductions below warning-free; every
        # result derived from it is gated on finite_count.
        filled = np.where(finite, neighbours, 0.0)

        blended = (
            q00 * (1.0 - wc) * (1.0 - wr)
            + q01 * wc * (1.0 - wr)
            + q10 * (1.0 - wc) * wr
            + q11 * wc * wr
        )

        # Only blend where all four neighbours exist and agree in depth;
        # otherwise take the nearest cell so depth edges stay sharp.
        spread = filled.max(axis=1) - filled.min(axis=1)
        smooth = (finite_count == 4) & (spread <= self.depth_tolerance)

        nearest_r = np.where(wr >= 0.5, r1, r0)
        nearest_c = np.where(wc >= 0.5, c1, c0)
        nearest = ranges[nearest_r, nearest_c]
        # A missing nearest cell still has up to three usable neighbours.
        partial_mean = np.divide(
            filled.sum(axis=1), finite_count,
            out=np.full(finite_count.shape, np.nan, dtype=np.float32),
            where=finite_count > 0,
        )
        fallback = np.where(np.isfinite(nearest), nearest, partial_mean)

        return np.where(smooth, blended, fallback)


def main():
    rclpy.init()
    node = LivoxPatternFilter()
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
