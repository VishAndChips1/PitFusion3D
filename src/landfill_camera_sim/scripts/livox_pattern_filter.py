#!/usr/bin/env python3

import csv
import math

import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


def _load_scan_pattern(csv_path):
    """Load a Livox-SDK livox_laser_simulation scan_mode CSV.

    Columns are Time/s, Azimuth/deg, Zenith/deg. Azimuth/zenith are
    converted the same way LivoxPointsPlugin::convertDataToRotateInfo does:
    azimuth stays as-is and zenith is shifted by -90 deg so 0 rad means
    "straight ahead" instead of "straight up".
    """
    azimuths = []
    zeniths = []
    deg_to_rad = math.pi / 180.0
    with open(csv_path, 'r', encoding='utf-8-sig', newline='') as csv_file:
        reader = csv.reader(csv_file)
        next(reader, None)  # header: Time/s, Azimuth/deg, Zenith/deg
        for row in reader:
            if len(row) != 3:
                continue
            azimuths.append(float(row[1]) * deg_to_rad)
            zeniths.append(float(row[2]) * deg_to_rad - math.pi / 2.0)
    if not azimuths:
        raise ValueError('Scan pattern CSV %s contained no data rows' % csv_path)
    return azimuths, zeniths


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

        # Gazebo Sim's gpu_lidar sensor auto-publishes a native structured
        # point cloud on <lidar topic>/points (gz.msgs.PointCloudPacked,
        # bridged as sensor_msgs/msg/PointCloud2) with real per-ray x,y,z
        # from its own raycasting, laid out as a regular
        # horizontal_samples x vertical_samples grid. That -- not the
        # bridged sensor_msgs/msg/LaserScan, which only carries a single
        # flat horizontal row with no elevation information at all -- is
        # this node's input, so every one of the 24000 Avia pattern points
        # gets its own real 3D sample instead of reusing one row's range
        # across every elevation angle (which produced a flat, planar
        # cloud).
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

        self.input_points_topic = self.get_parameter('input_points_topic').value
        self.output_points_topic = self.get_parameter('output_points_topic').value
        self.frame_id = self.get_parameter('frame_id').value
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
        scan_pattern_csv = str(self.get_parameter('scan_pattern_csv').value)

        if not scan_pattern_csv:
            raise RuntimeError(
                'scan_pattern_csv is empty and the packaged avia.csv could not be located'
            )
        pattern_azimuth, pattern_zenith = _load_scan_pattern(scan_pattern_csv)
        self.pattern_size = len(pattern_azimuth)

        # Precompute each Avia pattern row's target (horizontal_index,
        # vertical_index) into the incoming grid once, since the pattern
        # table itself never changes.
        pattern_azimuth = np.asarray(pattern_azimuth)
        pattern_zenith = np.asarray(pattern_zenith)
        horizontal_angle = pattern_azimuth
        # Upstream's zenith-after-90deg-shift is a pitch where positive
        # means "down" (Gazebo's pitch convention, also used for this
        # package's camera rpy); the incoming grid's z is up, so flip sign.
        vertical_angle = -pattern_zenith
        self.pattern_horizontal_index = self._angle_to_index_array(
            horizontal_angle, self.horizontal_min_angle, self.horizontal_max_angle, self.horizontal_samples
        )
        self.pattern_vertical_index = self._angle_to_index_array(
            vertical_angle, self.vertical_min_angle, self.vertical_max_angle, self.vertical_samples
        )
        self.pattern_grid_index = (
            self.pattern_vertical_index * self.horizontal_samples + self.pattern_horizontal_index
        )

        # Running index into the scan pattern table, advanced by
        # points_per_scan every update (regardless of downsample) and
        # wrapped modulo the table size, matching
        # LivoxPointsPlugin::InitializeRays' currStartIndex behavior so
        # consecutive scans walk consecutive, non-repeating slices of the
        # real Avia pattern.
        self.pattern_index = 0

        self.expected_points = self.horizontal_samples * self.vertical_samples
        self.warned_grid_shape = False

        self.publisher = self.create_publisher(PointCloud2, self.output_points_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_points_topic,
            self._on_points,
            10,
        )

        self.get_logger().info(
            'Publishing Livox Avia pattern %s -> %s (%d points/scan, %d-row scan table from %s)'
            % (
                self.input_points_topic,
                self.output_points_topic,
                self.points_per_scan,
                self.pattern_size,
                scan_pattern_csv,
            )
        )

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
        if grid_point_count == 0:
            return

        grid = pc2.read_points_numpy(cloud_msg, field_names=['x', 'y', 'z', 'intensity'])
        if grid.shape[0] < self.expected_points:
            return  # grid smaller than configured FOV; indices would be out of range

        start_index = self.pattern_index
        self.pattern_index = (self.pattern_index + self.points_per_scan) % self.pattern_size

        sample_rows = (start_index + np.arange(0, self.points_per_scan, self.downsample)) % self.pattern_size
        grid_indices = self.pattern_grid_index[sample_rows]

        sampled = grid[grid_indices]
        xyz = sampled[:, 0:3]
        intensity = sampled[:, 3]

        distance = np.linalg.norm(xyz, axis=1)
        valid = np.isfinite(distance) & (distance >= self.range_min) & (distance <= self.range_max)

        out_xyz = xyz[valid].astype(np.float32)
        out_intensity = intensity[valid].astype(np.float32)
        # NaN/garbage intensity from unhit rays; fall back to the pattern
        # row index (matches the previous synthetic-intensity behavior).
        out_intensity = np.where(np.isfinite(out_intensity), out_intensity, sample_rows[valid] % 255)

        n = out_xyz.shape[0]
        payload = np.empty((n, 4), dtype=np.float32)
        payload[:, 0:3] = out_xyz
        payload[:, 3] = out_intensity

        cloud = PointCloud2()
        cloud.header = cloud_msg.header
        cloud.header.frame_id = self.frame_id
        cloud.height = 1
        cloud.width = n
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * n
        cloud.data = payload.tobytes()
        cloud.is_dense = False
        self.publisher.publish(cloud)

    @staticmethod
    def _angle_to_index_array(angle, min_angle, max_angle, sample_count):
        if sample_count <= 1 or max_angle <= min_angle:
            return np.zeros_like(angle, dtype=np.int64)
        normalized = (angle - min_angle) / (max_angle - min_angle)
        normalized = np.clip(normalized, 0.0, 1.0)
        return np.round(normalized * (sample_count - 1)).astype(np.int64)


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
