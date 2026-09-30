#!/usr/bin/env python3
"""Re-stamp the Livox driver's PointCloud2 into the host's clock domain.

livox_ros2_avia (xfer_format=0, PointCloud2 path) stamps each cloud with
the lidar's own internal hardware counter -- nanoseconds since the
sensor booted (lddc.cpp: `cloud.header.stamp = rclcpp::Time(timestamp)`
from `GetStoragePacketTimestamp`), not the host clock. That's correct
only when `enable_timesync` bridges it to PTP/GPS/PPS, which our
livox_lidar_config.json has disabled -- measured live, the device clock
reads ~3034s (uptime) against a host time of ~1.79e9s (Unix epoch), an
offset of billions of seconds. Left alone, that gap never falls inside
sensor_fusion_node's ApproximateTimeSynchronizer slop against the
RGB/thermal nodes' host-clock stamps: lidar/rgb/thermal all publish
fine standalone, but points_rgbt stays silent forever.

The lidar's *relative* timing is hardware-precise (packet-to-packet
deltas measured ~100.00ms +/- <1ms at the configured 10Hz), so the right
fix is an epoch shift, not a wholesale re-stamp with arrival time --
re-stamping with raw arrival time would fold ~8ms of measured
network/ROS scheduling jitter into every point's timestamp instead.

Offset estimation: for each message, sample_offset = arrival_host_ns -
device_ns. True offset + (network/scheduling latency, which is >= 0) is
what we observe, so the minimum sample_offset over a trailing window is
the best estimate of the true fixed offset (the same "min-delay" idea
NTP/PTP filtering uses). Re-derived from a rolling window (not a single
startup sample) so it can track slow crystal drift between the lidar's
free-running oscillator and the host clock over a long-running session.
A large drop in device_ns (driver/sensor restart) resets the window
immediately rather than slowly re-converging.
"""
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class LivoxRestampNode(Node):
    def __init__(self):
        super().__init__('livox_restamp_node')
        self.declare_parameter('input_topic', 'livox/lidar')
        self.declare_parameter('output_topic', '/landfill/livox/points')
        self.declare_parameter('offset_window', 200)  # ~20s at 10Hz
        self.declare_parameter('log_period', 50)  # messages between diagnostic logs

        self._window_size = int(self.get_parameter('offset_window').value)
        self._log_period = int(self.get_parameter('log_period').value)
        self._offsets = deque(maxlen=self._window_size)
        self._offset_ns = None
        self._last_device_ns = None
        self._msg_count = 0

        # BEST_EFFORT: a lidar scan arriving at 10Hz makes retransmitting a
        # dropped one pointless, and it removes DDS's ACK/retransmit
        # bookkeeping from this node and sensor_fusion_node.py (its
        # subscriber). The raw livox/lidar topic (input side) is published
        # RELIABLE by the compiled driver and can't be changed, but a
        # BEST_EFFORT *request* is compatible with a RELIABLE *offer* (DDS
        # only refuses to match when the request is stricter than what's
        # offered), so this is safe here too.
        self._publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter('output_topic').value), qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            PointCloud2, str(self.get_parameter('input_topic').value), self._on_cloud,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Re-stamping {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value} "
            f"(min-offset over a {self._window_size}-sample window)"
        )

    def _on_cloud(self, msg):
        arrival_ns = self.get_clock().now().nanoseconds
        device_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if self._last_device_ns is not None and device_ns < self._last_device_ns - 1_000_000_000:
            self.get_logger().warn(
                'Livox device clock jumped backwards (sensor/driver restart?); '
                'resetting offset estimate'
            )
            self._offsets.clear()
            self._offset_ns = None
        self._last_device_ns = device_ns

        sample_offset = arrival_ns - device_ns
        self._offsets.append(sample_offset)
        self._offset_ns = min(self._offsets)

        corrected_ns = device_ns + self._offset_ns
        msg.header.stamp.sec = corrected_ns // 1_000_000_000
        msg.header.stamp.nanosec = corrected_ns % 1_000_000_000
        self._publisher.publish(msg)

        self._msg_count += 1
        if self._msg_count % self._log_period == 0:
            jitter_ms = (max(self._offsets) - min(self._offsets)) / 1e6
            self.get_logger().info(
                f'offset={self._offset_ns / 1e9:.6f}s window_jitter={jitter_ms:.2f}ms '
                f'(n={len(self._offsets)})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LivoxRestampNode()
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
