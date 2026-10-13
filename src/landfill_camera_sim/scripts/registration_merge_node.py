#!/usr/bin/env python3
"""Merge a rolling window of fused (RGB+thermal-colored) lidar scans.

Buffers sensor_fusion_node.py's output for a fixed time window (1s by
default), registers each scan against the accumulated map with point-to-
point ICP to compensate for platform motion between scans, and publishes
one denser merged point cloud per window.

The running map is kept voxel-downsampled: ICP registers against that
small map rather than against every raw point accumulated so far, which
keeps per-window cost roughly linear in the number of scans instead of
growing with the square of the points already merged.
"""

import time

import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

import cloud_io


def voxel_downsample_indices(xyz, voxel_size):
    """Indices of one representative point per voxel.

    Hashes the 3 integer voxel coordinates into a single int64 key so the
    dedupe is a 1-D sort; np.unique(..., axis=0) instead lexsorts an (N, 3)
    array, which is markedly slower for the ~1e5-point clouds seen here.
    """
    n = xyz.shape[0]
    if voxel_size <= 0.0 or n == 0:
        return np.arange(n)

    keys = np.floor(np.asarray(xyz, dtype=np.float64) / voxel_size).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = keys.max(axis=0) + 1
    # Fall back when the voxel grid is too large to index with an int64 key.
    if float(dims[0]) * float(dims[1]) * float(dims[2]) >= 9.0e18:
        _, unique_indices = np.unique(keys, axis=0, return_index=True)
        return unique_indices

    flat = (keys[:, 0] * dims[1] + keys[:, 1]) * dims[2] + keys[:, 2]
    _, unique_indices = np.unique(flat, return_index=True)
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


# Below this many query points, scipy's per-call thread spawn costs more
# than the parallel correspondence search saves.
PARALLEL_QUERY_THRESHOLD = 20000


def icp_point_to_point(source_xyz, target_xyz, max_iterations, max_correspondence_dist,
                       min_correspondences, workers=None):
    """Point-to-point ICP. Returns (R, t, converged) aligning source onto target."""
    if source_xyz.shape[0] < min_correspondences or target_xyz.shape[0] < min_correspondences:
        return np.eye(3), np.zeros(3), False

    if workers is None:
        workers = -1 if source_xyz.shape[0] >= PARALLEL_QUERY_THRESHOLD else 1

    target_tree = cKDTree(target_xyz)
    working = source_xyz.copy()
    r_total = np.eye(3)
    t_total = np.zeros(3)
    prev_error = None
    converged = False

    for _ in range(max_iterations):
        # Correspondence search dominates ICP; spread it over all cores.
        distances, indices = target_tree.query(
            working, distance_upper_bound=max_correspondence_dist, workers=workers
        )
        mask = np.isfinite(distances)
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
        # 1cm voxel: merges near-duplicate points where scans overlap
        # without visibly thinning the cloud (0 keeps every raw point,
        # e.g. 10 scans of 24000 points/scan -> 240000 points).
        self.declare_parameter('output_voxel_size', 0.01)
        # Cap the ICP target so a long window cannot grow the correspondence
        # search without bound.
        self.declare_parameter('max_icp_target_points', 20000)
        # Consecutive scans are ~0.1s apart, so the pose delta ICP has to
        # recover is centimetres; a few thousand well-spread points resolve
        # that just as well as the full scan and dominate the window cost.
        self.declare_parameter('max_icp_source_points', 3000)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._window_duration = float(self.get_parameter('window_duration').value)
        self._icp_voxel_size = float(self.get_parameter('icp_voxel_size').value)
        self._max_icp_iterations = int(self.get_parameter('max_icp_iterations').value)
        self._max_correspondence_distance = float(self.get_parameter('max_correspondence_distance').value)
        self._min_correspondences = int(self.get_parameter('min_correspondences').value)
        self._output_voxel_size = float(self.get_parameter('output_voxel_size').value)
        self._max_icp_target_points = int(self.get_parameter('max_icp_target_points').value)
        self._max_icp_source_points = int(self.get_parameter('max_icp_source_points').value)

        self._buffer = []  # list of (stamp_sec, xyz(N,3) float32, rest(N,3) float32)
        self._window_start = None
        self._frame_id = None
        self._last_publish_wall_time = None
        self._rng = np.random.default_rng(0)

        # BEST_EFFORT: matches sensor_fusion_node.py's output QoS (this
        # node's input) and lets fusion_web_streamer.py's subscription to
        # this node's output skip DDS's reliable-delivery bookkeeping too.
        self._publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, qos_profile_sensor_data
        )

        self.get_logger().info(
            'Merging %s -> %s over %.2fs windows with ICP motion compensation'
            % (input_topic, output_topic, self._window_duration)
        )

    def _on_cloud(self, msg):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._frame_id = msg.header.frame_id

        points = cloud_io.read_fields(msg, cloud_io.XYZIRT_FIELDS, dtype=np.float32)
        if points.shape[0] == 0:
            return

        # A sim restart rewinds the clock; without this the window never
        # closes again because stamp_sec stays below the stale start.
        # 1-second tolerance (matching sensor_fusion_node.py's identical
        # check), not a hair-trigger on *any* backward movement: on real
        # hardware with no battery-backed RTC, the clock gets stepped by
        # NTP as it settles after every boot -- small backward jitter
        # from that is normal and was previously enough to reset this
        # window every few seconds, discarding whatever scans had
        # already accumulated and tanking the merged cloud's refresh
        # rate for the first several minutes after every power-on.
        if self._window_start is not None and stamp_sec < self._window_start - 1.0:
            self.get_logger().warn(
                'Timestamps jumped backwards (%.3f < %.3f); resetting merge window'
                % (stamp_sec, self._window_start)
            )
            self._buffer = []
            self._window_start = None

        if self._window_start is None:
            self._window_start = stamp_sec

        self._buffer.append((stamp_sec, points[:, 0:3], points[:, 3:6]))

        if stamp_sec - self._window_start >= self._window_duration:
            self._register_and_publish()
            self._buffer = []
            self._window_start = None

    def _subsample(self, xyz, limit):
        if limit <= 0 or xyz.shape[0] <= limit:
            return xyz
        return xyz[self._rng.choice(xyz.shape[0], size=limit, replace=False)]

    def _register_and_publish(self):
        if not self._buffer:
            return
        self._buffer.sort(key=lambda entry: entry[0])

        first_stamp, first_xyz, first_rest = self._buffer[0]
        merged_xyz = [first_xyz]
        merged_rest = [first_rest]

        # Running map, held downsampled so ICP never searches the full
        # accumulated cloud.
        map_ds_xyz = np.asarray(
            first_xyz[voxel_downsample_indices(first_xyz, self._icp_voxel_size)],
            dtype=np.float64,
        )
        non_converged = 0

        for stamp_sec, xyz, rest in self._buffer[1:]:
            source = np.asarray(
                xyz[voxel_downsample_indices(xyz, self._icp_voxel_size)], dtype=np.float64
            )
            r, t, converged = icp_point_to_point(
                self._subsample(source, self._max_icp_source_points),
                self._subsample(map_ds_xyz, self._max_icp_target_points),
                self._max_icp_iterations,
                self._max_correspondence_distance,
                self._min_correspondences,
            )
            if not converged:
                non_converged += 1
                r, t = np.eye(3), np.zeros(3)

            aligned_xyz = (xyz @ r.T.astype(np.float32)) + t.astype(np.float32)
            merged_xyz.append(aligned_xyz)
            merged_rest.append(rest)

            aligned_ds = source @ r.T + t
            map_ds_xyz = np.vstack((map_ds_xyz, aligned_ds))
            map_ds_xyz = map_ds_xyz[voxel_downsample_indices(map_ds_xyz, self._icp_voxel_size)]

        if non_converged:
            self.get_logger().warn(
                '%d of %d scans did not converge; merged without motion compensation'
                % (non_converged, len(self._buffer) - 1)
            )

        all_xyz = np.concatenate(merged_xyz, axis=0)
        all_rest = np.concatenate(merged_rest, axis=0)

        keep_idx = voxel_downsample_indices(all_xyz, self._output_voxel_size)
        cloud_out = np.empty((keep_idx.shape[0], 6), dtype=np.float32)
        cloud_out[:, 0:3] = all_xyz[keep_idx]
        cloud_out[:, 3:6] = all_rest[keep_idx]

        last_stamp = self._buffer[-1][0]
        ros_header = Header()
        ros_header.stamp.sec = int(last_stamp)
        ros_header.stamp.nanosec = int(round((last_stamp - int(last_stamp)) * 1e9))
        ros_header.frame_id = self._frame_id or ''

        self._publisher.publish(
            cloud_io.make_cloud(ros_header, cloud_io.XYZIRT_FIELDS, cloud_out)
        )

        now_wall = time.monotonic()
        if self._last_publish_wall_time is not None:
            publish_hz = 1.0 / max(1e-6, now_wall - self._last_publish_wall_time)
        else:
            publish_hz = 0.0
        self._last_publish_wall_time = now_wall

        self.get_logger().info(
            'Published merged cloud: %d pts (%d scans, %d raw pts, %.2fs window) @ %.2f Hz'
            % (cloud_out.shape[0], len(self._buffer), all_xyz.shape[0],
               last_stamp - first_stamp, publish_hz)
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
