#!/usr/bin/env python3
"""Merge a rolling window of fused scans into one denser point cloud.

Scans are buffered for window_duration seconds, each is registered against the
accumulated map with point-to-point ICP (to compensate platform motion), and
one merged cloud is published per window. The map is kept voxel-downsampled so
the cost grows linearly with the number of scans.
"""

import queue
import threading
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
    """Return the indices of one representative point per voxel (voxel coordinates hashed to one int64 key)."""
    n = xyz.shape[0]
    if voxel_size <= 0.0 or n == 0:
        return np.arange(n)

    keys = np.floor(np.asarray(xyz, dtype=np.float64) / voxel_size).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = keys.max(axis=0) + 1
    # Grid too large for an int64 key: fall back to the slower row-wise unique.
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


# Below this many query points a parallel KD-tree search costs more than it saves.
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
        # Nearest-neighbour search dominates ICP cost.
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
    """Accumulate fused scans into windows and register/publish them on a worker thread."""

    def __init__(self):
        super().__init__('registration_merge_node')

        self.declare_parameter('input_topic', '/landfill/fusion/points_rgbt')
        self.declare_parameter('output_topic', '/landfill/fusion/points_merged')
        self.declare_parameter('window_duration', 1.0)
        # ICP settings: finer voxels are more accurate; fewer iterations and points are cheaper.
        self.declare_parameter('icp_voxel_size', 0.1)
        self.declare_parameter('max_icp_iterations', 10)
        self.declare_parameter('max_correspondence_distance', 0.5)
        self.declare_parameter('min_correspondences', 200)
        # Output voxel (m): merges near-duplicate points where scans overlap; 0 keeps every point.
        self.declare_parameter('output_voxel_size', 0.01)
        # Caps on the points ICP uses (scans are ~0.1 s apart, so a sparse subset is enough).
        self.declare_parameter('max_icp_target_points', 10000)
        self.declare_parameter('max_icp_source_points', 1500)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._window_duration = float(self.get_parameter('window_duration').value)
        # A window also closes after this many scans, in case the stamps stall.
        self._max_window_scans = 60
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

        # ICP runs on a worker thread so the callback keeps receiving scans. One window may
        # wait while another is processed; older ones are dropped to keep latency bounded.
        self._work_q = queue.Queue(maxsize=1)
        self._max_window_age = 1.0
        self._dropped_windows = 0
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        # Best-effort QoS, matching the fusion node's output.
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
        """Accumulate one fused scan; hand the window to the worker when it is complete."""
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._frame_id = msg.header.frame_id
        points = self._finite_points(msg)
        if points is None:
            return
        self._reset_window_if_clock_rewound(stamp_sec)
        if self._window_start is None:
            self._window_start = stamp_sec
        self._buffer.append((stamp_sec, points[:, 0:3], points[:, 3:6]))
        if self._window_is_complete(stamp_sec):
            self._submit_window()

    @staticmethod
    def _finite_points(msg):
        """Return the cloud's (N, 6) array without non-finite points, or None if nothing is left."""
        points = cloud_io.read_fields(msg, cloud_io.XYZIRT_FIELDS, dtype=np.float32)
        if points.shape[0] == 0:
            return None
        finite = np.isfinite(points[:, 0:3]).all(axis=1)
        if not finite.all():
            points = points[finite]
        return points if points.shape[0] else None

    def _reset_window_if_clock_rewound(self, stamp_sec):
        """Start a fresh window if stamps jump backwards by more than 1 s (clock step or restart)."""
        if self._window_start is not None and stamp_sec < self._window_start - 1.0:
            self.get_logger().warn(
                'Timestamps jumped backwards (%.3f < %.3f); resetting merge window'
                % (stamp_sec, self._window_start)
            )
            self._buffer = []
            self._window_start = None

    def _window_is_complete(self, stamp_sec):
        """Return True once the window spans window_duration seconds, or at the scan cap."""
        return (stamp_sec - self._window_start >= self._window_duration
                or len(self._buffer) >= self._max_window_scans)

    def _submit_window(self):
        """Hand the closed window to the worker thread, replacing a stale waiting one."""
        window, self._buffer = self._buffer, []
        self._window_start = None
        item = (time.monotonic(), window)
        try:
            self._work_q.put_nowait(item)
            return
        except queue.Full:
            pass
        # Worker busy and a window already waiting: keep only the newest.
        try:
            self._work_q.get_nowait()
        except queue.Empty:
            pass
        self._dropped_windows += 1
        if self._dropped_windows % 10 == 1:
            self.get_logger().warn(
                f'Merge worker behind -- replaced a stale window with the newest '
                f'({self._dropped_windows} dropped so far)')
        try:
            self._work_q.put_nowait(item)
        except queue.Full:
            pass

    def _worker_loop(self):
        """Register and publish queued windows; one bad window must not stop the node."""
        while rclpy.ok():
            try:
                closed_at, window = self._work_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if time.monotonic() - closed_at > self._max_window_age:
                self._dropped_windows += 1
                continue  # too old to be worth publishing in a live view
            try:
                self._register_and_publish(window)
            except Exception as exc:
                self.get_logger().error(f'Window registration/publish failed, skipping: {exc!r}')

    def _subsample(self, xyz, limit):
        """Return a random subset of at most `limit` points (0 = no limit)."""
        if limit <= 0 or xyz.shape[0] <= limit:
            return xyz
        return xyz[self._rng.choice(xyz.shape[0], size=limit, replace=False)]

    def _register_and_publish(self, buffer):
        """ICP-register every scan onto the running map, merge them and publish the result."""
        if not buffer:
            return
        buffer.sort(key=lambda entry: entry[0])

        first_stamp, first_xyz, first_rest = buffer[0]
        merged_xyz = [first_xyz]
        merged_rest = [first_rest]

        # Running map, kept downsampled for ICP.
        map_ds_xyz = np.asarray(
            first_xyz[voxel_downsample_indices(first_xyz, self._icp_voxel_size)],
            dtype=np.float64,
        )
        non_converged = 0

        for stamp_sec, xyz, rest in buffer[1:]:
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
                % (non_converged, len(buffer) - 1)
            )

        all_xyz = np.concatenate(merged_xyz, axis=0)
        all_rest = np.concatenate(merged_rest, axis=0)

        keep_idx = voxel_downsample_indices(all_xyz, self._output_voxel_size)
        cloud_out = np.empty((keep_idx.shape[0], 6), dtype=np.float32)
        cloud_out[:, 0:3] = all_xyz[keep_idx]
        cloud_out[:, 3:6] = all_rest[keep_idx]

        last_stamp = buffer[-1][0]
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
            % (cloud_out.shape[0], len(buffer), all_xyz.shape[0],
               last_stamp - first_stamp, publish_hz)
        )


def main():
    """Run the merge node until interrupted."""
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
