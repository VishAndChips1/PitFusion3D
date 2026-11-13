#!/usr/bin/env python3
"""Re-stamp the Livox driver's PointCloud2 into the host clock.

The driver stamps each cloud with the LiDAR's own counter (seconds since the
sensor booted), which never lines up with the camera stamps, so nothing would
fuse. The LiDAR's relative timing is precise, so only the epoch is shifted:

* offset = min(arrival_time - device_time) over a rolling window (the minimum
  discards network/scheduling delay, like NTP/PTP filtering), which also
  tracks slow clock drift;
* the window is reset when the device clock jumps backwards (sensor restart) or
  when the host clock steps relative to the LiDAR clock (the Pi has no RTC, so
  NTP steps its clock shortly after boot).
"""
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class LivoxRestampNode(Node):
    """Subscribes to the raw driver topic and republishes with corrected stamps."""

    def __init__(self):
        super().__init__('livox_restamp_node')
        self.declare_parameter('input_topic', 'livox/lidar')
        self.declare_parameter('output_topic', '/landfill/livox/points')
        self.declare_parameter('offset_window', 200)  # ~20s at 10Hz
        self.declare_parameter('log_period', 50)  # messages between diagnostic logs
        self.declare_parameter('clock_step_threshold_s', 0.1)  # must stay below sensor_fusion_node's sync_slop (0.15s)

        self._window_size = int(self.get_parameter('offset_window').value)
        self._log_period = int(self.get_parameter('log_period').value)
        self._offsets = deque(maxlen=self._window_size)
        self._offset_ns = None
        self._last_device_ns = None
        self._last_arrival_ns = None
        self._step_threshold_ns = int(float(self.get_parameter('clock_step_threshold_s').value) * 1e9)
        self._msg_count = 0
        self.clock_resets = 0

        # Best-effort QoS: a dropped 10 Hz scan is not worth retransmitting. The driver's
        # reliable output still matches a best-effort subscription.
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

    def _estimate(self, arrival_ns, device_ns):
        """Update the offset estimate for one scan and return the corrected stamp (ns)."""
        if self._last_device_ns is not None:
            if device_ns < self._last_device_ns - 1_000_000_000:
                self.get_logger().warn(
                    'Livox device clock jumped backwards (sensor/driver restart?); '
                    'resetting offset estimate', throttle_duration_sec=5.0
                )
                self._offsets.clear()
                self.clock_resets += 1
            else:
                skew = (arrival_ns - self._last_arrival_ns) - (device_ns - self._last_device_ns)
                if abs(skew) > self._step_threshold_ns:
                    self.get_logger().warn(
                        f'Host clock stepped by {skew / 1e9:+.3f}s relative to the lidar clock '
                        '(NTP step?); resetting offset estimate', throttle_duration_sec=5.0
                    )
                    self._offsets.clear()
                    self.clock_resets += 1
        self._last_device_ns = device_ns
        self._last_arrival_ns = arrival_ns

        self._offsets.append(arrival_ns - device_ns)
        self._offset_ns = min(self._offsets)
        return device_ns + self._offset_ns

    def _on_cloud(self, msg):
        arrival_ns = self.get_clock().now().nanoseconds
        device_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        corrected_ns = self._estimate(arrival_ns, device_ns)
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
    """Run the re-stamp node until interrupted."""
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
