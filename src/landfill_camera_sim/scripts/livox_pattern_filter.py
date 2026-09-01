#!/usr/bin/env python3

import csv
import math
import struct

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


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

        self.declare_parameter('input_scan_topic', '/landfill/livox/raw_scan')
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

        self.input_scan_topic = self.get_parameter('input_scan_topic').value
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
        self.pattern_azimuth, self.pattern_zenith = _load_scan_pattern(scan_pattern_csv)
        self.pattern_size = len(self.pattern_azimuth)

        # Running index into the scan pattern table, advanced by
        # points_per_scan every update (regardless of downsample) and
        # wrapped modulo the table size, matching
        # LivoxPointsPlugin::InitializeRays' currStartIndex behavior so
        # consecutive scans walk consecutive, non-repeating slices of the
        # real Avia pattern.
        self.pattern_index = 0

        self.expected_ranges = self.horizontal_samples * self.vertical_samples
        self.warned_range_shape = False

        self.publisher = self.create_publisher(PointCloud2, self.output_points_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            self.input_scan_topic,
            self._on_scan,
            10,
        )

        self.get_logger().info(
            'Publishing Livox Avia pattern %s -> %s (%d points/scan, %d-row scan table from %s)'
            % (
                self.input_scan_topic,
                self.output_points_topic,
                self.points_per_scan,
                self.pattern_size,
                scan_pattern_csv,
            )
        )

    def _on_scan(self, scan):
        range_count = len(scan.ranges)
        if range_count < self.horizontal_samples:
            self.get_logger().warn(
                'Expected at least %d lidar ranges, received %d'
                % (self.horizontal_samples, range_count)
            )
            return
        has_vertical_grid = range_count >= self.expected_ranges
        if not has_vertical_grid and not self.warned_range_shape:
            self.warned_range_shape = True
            self.get_logger().warn(
                'ROS LaserScan bridge exposed %d horizontal ranges; sampling the '
                'real Avia scan pattern while using measured horizontal ranges.'
                % range_count
            )

        start_index = self.pattern_index
        self.pattern_index = (self.pattern_index + self.points_per_scan) % self.pattern_size

        points = bytearray()
        for step in range(0, self.points_per_scan, self.downsample):
            pattern_row = (start_index + step) % self.pattern_size
            horizontal_angle, vertical_angle = self._pattern_angles(pattern_row)
            horizontal_index = self._angle_to_index(
                horizontal_angle,
                self.horizontal_min_angle,
                self.horizontal_max_angle,
                self.horizontal_samples,
            )
            vertical_index = self._angle_to_index(
                vertical_angle,
                self.vertical_min_angle,
                self.vertical_max_angle,
                self.vertical_samples,
            )

            if has_vertical_grid:
                range_index = vertical_index * self.horizontal_samples + horizontal_index
            else:
                range_index = min(range_count - 1, horizontal_index)
            distance = float(scan.ranges[range_index])
            if (
                not math.isfinite(distance)
                or distance < self.range_min
                or distance > self.range_max
            ):
                continue

            cos_vertical = math.cos(vertical_angle)
            x = distance * cos_vertical * math.cos(horizontal_angle)
            y = distance * cos_vertical * math.sin(horizontal_angle)
            z = distance * math.sin(vertical_angle)
            intensity = float(pattern_row % 255)
            points.extend(struct.pack('<ffff', x, y, z, intensity))

        cloud = PointCloud2()
        cloud.header = scan.header
        cloud.header.frame_id = self.frame_id
        cloud.height = 1
        cloud.width = len(points) // 16
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = bytes(points)
        cloud.is_dense = False
        self.publisher.publish(cloud)

    def _pattern_angles(self, pattern_row):
        horizontal_angle = self.pattern_azimuth[pattern_row]
        # Upstream's zenith-after-90deg-shift is a pitch where positive
        # means "down" (Gazebo's pitch convention, also used for this
        # package's camera rpy). This node treats positive vertical_angle
        # as "up" (z = distance * sin(vertical_angle)), so the sign is
        # flipped here.
        vertical_angle = -self.pattern_zenith[pattern_row]
        return horizontal_angle, vertical_angle

    @staticmethod
    def _angle_to_index(angle, min_angle, max_angle, sample_count):
        if sample_count <= 1 or max_angle <= min_angle:
            return 0
        normalized = (angle - min_angle) / (max_angle - min_angle)
        normalized = min(1.0, max(0.0, normalized))
        return int(round(normalized * (sample_count - 1)))


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
