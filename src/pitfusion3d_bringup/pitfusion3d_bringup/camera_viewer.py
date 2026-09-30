#!/usr/bin/env python3
"""Debug tool: side-by-side RGB + thermal preview with sync-offset readout.

Not part of the fusion pipeline -- just a quick visual sanity check that
both cameras are alive and time-synchronized before trusting the fused
cloud. Run standalone:

    ros2 run pitfusion3d_bringup camera_viewer
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.declare_parameter('rgb_topic', '/landfill/rgb/image_raw')
        self.declare_parameter('thermal_topic', '/landfill/thermal/image_raw')

        self.bridge = CvBridge()
        rgb_topic = str(self.get_parameter('rgb_topic').value)
        thermal_topic = str(self.get_parameter('thermal_topic').value)

        self.rgb_sub = Subscriber(self, Image, rgb_topic)
        self.thermal_sub = Subscriber(self, Image, thermal_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.thermal_sub], queue_size=10, slop=0.15,
        )
        self.sync.registerCallback(self.synced_callback)
        self.get_logger().info(f'Camera Viewer started: {rgb_topic} + {thermal_topic}')

    def synced_callback(self, rgb_msg, thermal_msg):
        rgb_ts = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        thermal_ts = thermal_msg.header.stamp.sec + thermal_msg.header.stamp.nanosec * 1e-9
        diff_ms = abs(rgb_ts - thermal_ts) * 1000

        rgb_frame = cv2.cvtColor(
            self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8'), cv2.COLOR_RGB2BGR
        )
        thermal_raw = self.bridge.imgmsg_to_cv2(thermal_msg, desired_encoding='mono16')
        thermal_8bit = (thermal_raw >> 8).astype(np.uint8)
        thermal_frame = cv2.applyColorMap(thermal_8bit, cv2.COLORMAP_INFERNO)

        target_height = 512
        rgb_aspect = rgb_frame.shape[1] / rgb_frame.shape[0]
        rgb_resized = cv2.resize(rgb_frame, (int(target_height * rgb_aspect), target_height))
        thermal_resized = cv2.resize(thermal_frame, (640, target_height))

        cv2.putText(rgb_resized, f'RGB  ts:{rgb_ts:.3f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(thermal_resized, f'THERMAL  ts:{thermal_ts:.3f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(thermal_resized, f'Sync diff: {diff_ms:.2f}ms', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        stacked = np.hstack([rgb_resized, thermal_resized])
        cv2.imshow('RGB | THERMAL', stacked)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
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
