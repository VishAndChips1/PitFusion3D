#!/usr/bin/env python3
"""Merge a rolling window of fused (RGB+thermal-colored) lidar scans.

Buffers sensor_fusion_node.py's output for a fixed time window (1s by
default), registers each scan against the accumulated map with point-to-
point ICP to compensate for platform motion between scans, and publishes
one denser merged point cloud per window.
"""

import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


def _voxel_downsample_indices(xyz, voxel_size):
    if voxel_size <= 0.0 or xyz.shape[0] == 0:
        return np.arange(xyz.shape[0])
    keys = np.floor(xyz / voxel_size).astype(np.int64)
    _, unique_indices = np.unique(keys, axis=0, return_index=True)
    return unique_indices


def _kabsch(source, target):
    """Rigid R, t minimizing ||target - (source @ R.T + t)||^2."""
    centroid_source = source.mean(axis=0)
    centroid_target = target.mean(axis=0)
    src_centered = source - centroid_source
    tgt_centered = target - centroid_target
    h = src_centered.T @ tgt_centered
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    correction = np.diag([1.0, 1.0, d])
    r = vt.T @ correction @ u.T
    t = centroid_target - r @ centroid_source
    return r, t


def icp_point_to_point(source_xyz, target_xyz, max_iterations, max_correspondence_dist, min_correspondences):
    """Point-to-point ICP. Returns (R, t, converged) aligning source onto target."""
    if source_xyz.shape[0] < min_correspondences or target_xyz.shape[0] < min_correspondences:
        return np.eye(3), np.zeros(3), False

    target_tree = cKDTree(target_xyz)
    working = source_xyz.copy()
    r_total = np.eye(3)
    t_total = np.zeros(3)
    prev_error = None
    converged = False

    for _ in range(max_iterations):
        distances, indices = target_tree.query(working)
        mask = distances < max_correspondence_dist
        if mask.sum() < min_correspondences:
            break
        r, t = _kabsch(working[mask], target_xyz[indices[mask]])
        working = working @ r.T + t
        r_total = r @ r_total
        t_total = r @ t_total + t
        mean_error = float(distances[mask].mean())
        if prev_error is not None and abs(prev_error - mean_error) < 1e-5:
            converged = True
            break
        prev_error = mean_error
    else:
        converged = True

    return r_total, t_total, converged


class RegistrationMergeNode(Node):
    def __init__(self):
        super().__init__('registration_merge_node')

        self.declare_parameter('input_topic', '/landfill/fusion/points_rgbt')
        self.declare_parameter('output_topic', '/landfill/fusion/points_merged')
        self.declare_parameter('window_duration', 1.0)
        self.declare_parameter('icp_voxel_size', 0.15)
        self.declare_parameter('max_icp_iterations', 20)
        self.declare_parameter('max_correspondence_distance', 0.5)
        self.declare_parameter('min_correspondences', 200)
        # 0 = keep every point from every scan in the window (e.g. 10 scans
        # of 24000 points/scan merges into 240000 points, not decimated).
        self.declare_parameter('output_voxel_size', 0.0)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._window_duration = float(self.get_parameter('window_duration').value)
        self._icp_voxel_size = float(self.get_parameter('icp_voxel_size').value)
        self._max_icp_iterations = int(self.get_parameter('max_icp_iterations').value)
        self._max_correspondence_distance = float(self.get_parameter('max_correspondence_distance').value)
        self._min_correspondences = int(self.get_parameter('min_correspondences').value)
        self._output_voxel_size = float(self.get_parameter('output_voxel_size').value)

        self._buffer = []  # list of (stamp_sec, xyz(N,3) float32, rest(N,3) float32 [intensity, rgb, thermal])
        self._window_start = None
        self._frame_id = None

        self._publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self._subscription = self.create_subscription(PointCloud2, input_topic, self._on_cloud, 10)

        self.get_logger().info(
            'Merging %s -> %s over %.2fs windows with ICP motion compensation'
            % (input_topic, output_topic, self._window_duration)
        )

    def _on_cloud(self, msg):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._frame_id = msg.header.frame_id

        points = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z', 'intensity', 'rgb', 'thermal'])
        if points.size == 0:
            return
        xyz = points[:, 0:3].astype(np.float64)
        rest = points[:, 3:6].astype(np.float32)  # intensity, rgb, thermal

        if self._window_start is None:
            self._window_start = stamp_sec

        self._buffer.append((stamp_sec, xyz, rest))

        if stamp_sec - self._window_start >= self._window_duration:
            self._register_and_publish()
            self._buffer = []
            self._window_start = None

    def _register_and_publish(self):
        if not self._buffer:
            return
        self._buffer.sort(key=lambda entry: entry[0])

        first_stamp, map_xyz, map_rest = self._buffer[0]
        merged_xyz = [map_xyz]
        merged_rest = [map_rest]

        for stamp_sec, xyz, rest in self._buffer[1:]:
            icp_target_idx = _voxel_downsample_indices(map_xyz, self._icp_voxel_size)
            icp_source_idx = _voxel_downsample_indices(xyz, self._icp_voxel_size)
            r, t, converged = icp_point_to_point(
                xyz[icp_source_idx],
                map_xyz[icp_target_idx],
                self._max_icp_iterations,
                self._max_correspondence_distance,
                self._min_correspondences,
            )
            if not converged:
                self.get_logger().warn(
                    'ICP did not converge for scan at t=%.3f; merging without motion compensation' % stamp_sec
                )
                r, t = np.eye(3), np.zeros(3)

            aligned_xyz = xyz @ r.T + t
            merged_xyz.append(aligned_xyz)
            merged_rest.append(rest)
            # Grow the running map so later scans in the window register
            # against everything accumulated so far, not just frame 0.
            map_xyz = np.vstack([map_xyz, aligned_xyz])

        all_xyz = np.vstack(merged_xyz)
        all_rest = np.vstack(merged_rest)

        keep_idx = _voxel_downsample_indices(all_xyz, self._output_voxel_size)
        out_xyz = all_xyz[keep_idx].astype(np.float32)
        out_rest = all_rest[keep_idx]

        cloud_out = np.empty((out_xyz.shape[0], 6), dtype=np.float32)
        cloud_out[:, 0:3] = out_xyz
        cloud_out[:, 3:6] = out_rest

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='thermal', offset=20, datatype=PointField.FLOAT32, count=1),
        ]
        stamp_sec_int = int(self._buffer[-1][0])
        stamp_nanosec = int(round((self._buffer[-1][0] - stamp_sec_int) * 1e9))
        ros_header = Header()
        ros_header.stamp.sec = stamp_sec_int
        ros_header.stamp.nanosec = stamp_nanosec
        ros_header.frame_id = self._frame_id or ''

        out_msg = pc2.create_cloud(ros_header, fields, cloud_out)
        self._publisher.publish(out_msg)

        self.get_logger().info(
            'Merged %d scans (%d raw pts) into %d pts over %.2fs window'
            % (len(self._buffer), all_xyz.shape[0], out_xyz.shape[0],
               self._buffer[-1][0] - first_stamp)
        )


def main():
    rclpy.init()
    node = RegistrationMergeNode()
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
