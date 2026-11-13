#!/usr/bin/env python3
"""Publish the Raspberry Pi HQ camera (IMX477) as sensor_msgs/Image (rgb8).

Frames come from `rpicam-vid` as raw YUV420 over a pipe, are converted to RGB
and published with the capture time as the header stamp.
"""
import ctypes
import signal
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# One OpenCV thread per process: the nodes share 4 cores and one cvtColor per frame gains nothing from a pool.
cv2.setNumThreads(1)


def _die_with_parent():
    """Ask the kernel to SIGTERM rpicam-vid if this node dies, so it never keeps the camera busy."""
    try:
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG
    except Exception:
        pass


class RgbCameraNode(Node):
    """Capture loop around rpicam-vid that respawns the process if the stream ends."""

    def __init__(self):
        super().__init__('rgb_camera_node')

        self.declare_parameter('topic', '/landfill/rgb/image_raw')
        self.declare_parameter('frame_id', 'camera_platform/rgb_optical_frame')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        # The LiDAR runs at 10 Hz, so 15 fps is enough for the synchroniser.
        self.declare_parameter('framerate', 15)
        # ISP colour saturation (1.0 = camera default, which looks washed out).
        self.declare_parameter('saturation', 1.8)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        framerate = int(self.get_parameter('framerate').value)
        self._saturation = float(self.get_parameter('saturation').value)
        self._frame_size = self._width * self._height * 3 // 2  # bytes per I420 frame

        self._bridge = CvBridge()
        # Best-effort QoS: a newer frame is always on its way, so never retransmit.
        # Subscribers must use a compatible (best-effort) profile.
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
                '--saturation', str(self._saturation),
                '--nopreview',
                '--flush',   # write every frame immediately instead of in bursts
                '-o', '-',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=self._frame_size * 2,
            preexec_fn=_die_with_parent,
        )
        self.get_logger().info(
            f'rpicam-vid capture started ({self._width}x{self._height}@{self._framerate}), '
            f'publishing rgb8 on {self._publisher.topic_name}'
        )

    def _read_exact(self, n):
        """Read exactly n bytes from the pipe; None if the process ended."""
        buf = bytearray()
        while len(buf) < n:
            if self._process.poll() is not None:
                return None
            chunk = self._process.stdout.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _publish_frame(self, raw, stamp):
        """Convert one YUV420 frame to rgb8 and publish it.

        Runs in a daemon thread, so a bad frame is logged and skipped: an
        uncaught exception here would silently stop publishing.
        """
        try:
            yuv = np.frombuffer(raw, dtype=np.uint8).reshape((self._height * 3 // 2, self._width))
            rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_I420)
            msg = self._bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            self._publisher.publish(msg)
        except Exception as exc:
            self.get_logger().error(f'Failed to process/publish frame, skipping: {exc!r}')

    def _capture_loop(self):
        """Run rpicam-vid and publish its frames; respawn with backoff if it stops."""
        subprocess.run(['pkill', '-x', 'rpicam-vid'], check=False)  # clear an orphan from a previous crash
        time.sleep(0.5)
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
                # Stamp as soon as the frame is read, before the colour conversion.
                stamp = self.get_clock().now().to_msg()
                stream_started = True
                backoff = 1.0  # a good frame proves the stream recovered
                self._publish_frame(raw, stamp)

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
        """Stop capturing and end the rpicam-vid process."""
        self._running = False
        self._terminate_process()
        super().destroy_node()


def main(args=None):
    """Run the RGB camera node until interrupted."""
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
