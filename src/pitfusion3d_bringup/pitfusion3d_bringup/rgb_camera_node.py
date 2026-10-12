#!/usr/bin/env python3
"""Publish the Raspberry Pi (imx477) camera as sensor_msgs/Image.

Captures raw YUV420 frames from `rpicam-vid` over a pipe -- the same
capture path already validated standalone:

    rpicam-vid -t 0 --width 1280 --height 720 --framerate 30 \\
        --codec yuv420 -o -

and republishes them as 'rgb8' so PitFusion3D's sensor_fusion_node.py
(which packs channel 0/1/2 directly as R/G/B with no encoding check) gets
the channel order it expects.
"""
import subprocess
import threading
import time

import cv2

# OpenCV defaults to spawning an internal thread pool sized to the core
# count (4 on this Pi) in *every* process that imports it. With this
# node, thermal_camera_node, and anything else in the pipeline all doing
# that independently, that's severe oversubscription on a 4-core board --
# measured as this node staying pinned near 50% CPU with declining
# throughput under load even after cutting its capture rate. A single
# cvtColor call per frame doesn't benefit from multi-threading anyway;
# forcing single-threaded mode avoids the thread-pool contention instead.
cv2.setNumThreads(1)

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class RgbCameraNode(Node):
    def __init__(self):
        super().__init__('rgb_camera_node')

        self.declare_parameter('topic', '/landfill/rgb/image_raw')
        self.declare_parameter('frame_id', 'camera_platform/rgb_optical_frame')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        # The Livox publishes at 10Hz, so anything captured/converted past
        # ~15Hz is CPU spent on frames the ApproximateTimeSynchronizer will
        # never use (fusion measured this node alone at ~49% of a core at
        # 30fps; 15fps still keeps frame age well inside the 0.15s sync
        # slop with room to spare).
        self.declare_parameter('framerate', 15)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        framerate = int(self.get_parameter('framerate').value)
        self._frame_size = self._width * self._height * 3 // 2  # I420

        self._bridge = CvBridge()
        # BEST_EFFORT (not the default RELIABLE): a live camera feed has a
        # newer frame arriving immediately anyway, so retransmitting a
        # dropped one is pure overhead. Every downstream subscriber to
        # this topic must use a compatible (BEST_EFFORT-or-looser) QoS or
        # DDS will silently refuse to connect them -- see
        # sensor_fusion_node.py and camera_viewer.py.
        self._publisher = self.create_publisher(
            Image, str(self.get_parameter('topic').value), qos_profile_sensor_data
        )

        self._framerate = framerate
        self._process = None
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _spawn(self):
        self._process = subprocess.Popen(
            [
                'rpicam-vid', '-t', '0',
                '--width', str(self._width),
                '--height', str(self._height),
                '--framerate', str(self._framerate),
                '--codec', 'yuv420',
                '--nopreview',
                '-o', '-',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=self._frame_size * 2,
        )
        self.get_logger().info(
            f'rpicam-vid capture started ({self._width}x{self._height}@{self._framerate}), '
            f'publishing rgb8 on {self._publisher.topic_name}'
        )

    def _read_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            if self._process.poll() is not None:
                return None
            chunk = self._process.stdout.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _capture_loop(self):
        # Self-healing: on boot (systemd/lingering, no one watching) the
        # imx477/unicam device can take a few seconds to be ready, and
        # rpicam-vid can be killed or glitch mid-run. Respawning here with
        # backoff means a transient hiccup fixes itself in seconds instead
        # of leaving this node's thread dead and the node silently
        # publishing nothing forever -- the node itself keeps running (no
        # exception reaches rclpy.spin), so nothing would otherwise notice
        # or restart it.
        backoff = 1.0
        while self._running and rclpy.ok():
            try:
                self._spawn()
            except OSError as exc:
                self.get_logger().error(f'Failed to start rpicam-vid: {exc}; retrying')
                time.sleep(backoff)
                backoff = min(backoff * 2, 10.0)
                continue

            stream_started = False
            while self._running and rclpy.ok():
                raw = self._read_exact(self._frame_size)
                if raw is None:
                    break
                stream_started = True
                backoff = 1.0  # a good frame proves the stream recovered
                yuv = np.frombuffer(raw, dtype=np.uint8).reshape(
                    (self._height * 3 // 2, self._width)
                )
                rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_I420)
                msg = self._bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self._frame_id
                self._publisher.publish(msg)

            if not self._running or not rclpy.ok():
                break
            self.get_logger().error(
                f'rpicam-vid stream ended {"unexpectedly" if stream_started else "before producing any frames"}'
                f'; respawning in {backoff:.0f}s'
            )
            self._terminate_process()
            time.sleep(backoff)
            backoff = min(backoff * 2, 10.0)

    def _terminate_process(self):
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()

    def destroy_node(self):
        self._running = False
        self._terminate_process()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RgbCameraNode()
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
