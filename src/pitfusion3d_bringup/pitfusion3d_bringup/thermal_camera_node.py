#!/usr/bin/env python3
"""Publish the UVC thermal camera (640x512 YUYV) as sensor_msgs/Image (mono16).

* The device is opened by its /dev/v4l/by-id path, which stays stable across
  reboots (plain /dev/videoN numbers move around).
* The camera only offers an auto-gain 8-bit stream, so the luma channel is
  published shifted into 16 bits (Y * 256). With thermal_resolution=1/256 and
  thermal_min_temp=0 the fused cloud recovers the 0-255 value: a relative
  hot/cold signal, not an absolute temperature.
"""
import os
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

cv2.setNumThreads(1)  # one OpenCV thread per process (4 shared cores)


class _CaptureHealth:
    """Retry/back-off bookkeeping for ThermalCameraNode's capture loop."""

    def __init__(self):
        self.open_backoff = 1.0     # seconds to wait after a failed open (doubles to 10 s)
        self.read_failures = 0      # consecutive failed reads
        self.cooldown = 0.0         # current reopen cool-down; forgotten after 30 good frames
        self.good_frames = 0        # consecutive good frames since the last failure

    def on_opened(self):
        self.open_backoff = 1.0
        self.read_failures = 0
        self.good_frames = 0

    def on_good_frame(self):
        self.read_failures = 0
        self.good_frames += 1
        if self.good_frames == 30:
            self.cooldown = 0.0     # genuinely streaming again -- forget the penalty


class ThermalCameraNode(Node):
    """Capture loop that waits for the camera, publishes frames and recovers from failures."""

    def __init__(self):
        super().__init__('thermal_camera_node')

        self.declare_parameter('topic', '/landfill/thermal/image_raw')
        self.declare_parameter('frame_id', 'camera_platform/thermal_optical_frame')
        self.declare_parameter(
            'device',
            '/dev/v4l/by-id/usb-Thermal_Cam_Co._Ltd_Camera_202206223-video-index0',
        )
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 512)
        # The camera only offers 30 or 60 fps; lower values are clamped to 30.
        self.declare_parameter('framerate', 30)
        # Colourised preview image; off by default (debugging only).
        self.declare_parameter('publish_preview', False)
        self.declare_parameter('preview_topic', '/landfill/thermal/image_preview')
        # The camera is mounted rotated 180 degrees: flipping both axes restores the orientation.
        self.declare_parameter('vflip', True)
        self.declare_parameter('hflip', True)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._framerate = int(self.get_parameter('framerate').value)
        self._device = str(self.get_parameter('device').value)
        self._publish_preview = bool(self.get_parameter('publish_preview').value)
        self._vflip = bool(self.get_parameter('vflip').value)
        self._hflip = bool(self.get_parameter('hflip').value)

        self._bridge = CvBridge()
        # Best-effort QoS (subscribers must match).
        self._publisher = self.create_publisher(
            Image, str(self.get_parameter('topic').value), qos_profile_sensor_data
        )
        self._preview_publisher = None
        if self._publish_preview:
            self._preview_publisher = self.create_publisher(
                Image, str(self.get_parameter('preview_topic').value), qos_profile_sensor_data
            )

        # A misbehaving camera can reset and re-enumerate repeatedly. Wait for the device
        # to be stable before opening it, and back off after each failed streak.
        self.declare_parameter('device_settle_s', 3.0)
        self.declare_parameter('recovery_cooldown_max_s', 20.0)
        self._settle_s = float(self.get_parameter('device_settle_s').value)
        self._cooldown_max = float(self.get_parameter('recovery_cooldown_max_s').value)

        self._cap = None
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _sleep_while_running(self, seconds):
        end = time.monotonic() + seconds
        while self._running and rclpy.ok() and time.monotonic() < end:
            time.sleep(0.25)

    def _wait_for_device(self):
        """Block until the device node has existed for device_settle_s (enumeration finished)."""
        since = None
        waited_since = time.monotonic()
        last_note = waited_since
        while self._running and rclpy.ok():
            if not os.path.exists(self._device) and time.monotonic() - last_note > 30.0:
                last_note = time.monotonic()
                self.get_logger().warn(
                    f'Thermal camera device {self._device} not present for '
                    f'{time.monotonic() - waited_since:.0f}s -- waiting (fusion continues without thermal)')
            if os.path.exists(self._device):
                now = time.monotonic()
                if since is None:
                    since = now
                elif now - since >= self._settle_s:
                    return True
            else:
                since = None
            time.sleep(0.25)
        return False

    def _open_capture(self):
        cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS, self._framerate)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        if not cap.isOpened():
            cap.release()
            return None
        return cap

    def _capture_loop(self):
        """Wait for the device, open it, read and publish frames, and recover from failures.

        Errors are retried rather than raised: an exception here would stop only this thread
        and leave a node that looks healthy but never publishes.
        """
        health = _CaptureHealth()
        while self._running and rclpy.ok():
            if self._cap is None and not self._connect(health):
                continue
            raw = self._read_frame(health)
            if raw is not None:
                self._publish_frame(raw)

    def _connect(self, health):
        """Wait for the device and open it. Returns True once streaming; False means retry/shutdown."""
        if not self._wait_for_device():
            return False
        self._cap = self._open_capture()
        if self._cap is None:
            self.get_logger().error(
                f'Failed to open thermal camera at {self._device}; retrying in {health.open_backoff:.0f}s')
            time.sleep(health.open_backoff)
            health.open_backoff = min(health.open_backoff * 2, 10.0)
            return False
        self.get_logger().info(
            f'{self._device} capture started '
            f'({self._width}x{self._height}@{self._framerate}), '
            f'publishing mono16 on {self._publisher.topic_name} '
            '(relative AGC signal, not radiometric Kelvin)')
        health.on_opened()
        return True

    def _read_frame(self, health):
        """Read one raw frame, or None after handling a failed read."""
        ok, raw = self._cap.read()
        if not ok or raw is None:
            self._on_read_failure(health)
            return None
        health.on_good_frame()
        return raw

    def _on_read_failure(self, health):
        """Count a failed read; after 10 in a row release the device and cool down.

        The cool-down grows from 3 s up to the configured maximum, so a faulty camera
        is not reopened in a tight loop.
        """
        health.read_failures += 1
        health.good_frames = 0
        if health.read_failures in (1, 10):
            self.get_logger().warn(
                f'Failed to read frame from thermal camera ({health.read_failures} in a row)')
        if health.read_failures < 10:
            time.sleep(0.1)
            return
        health.cooldown = min(max(health.cooldown * 2.0, 3.0), self._cooldown_max)
        self.get_logger().error(
            f'Too many consecutive read failures; reopening device after a {health.cooldown:.0f}s cool-down')
        self._cap.release()
        self._cap = None
        self._sleep_while_running(health.cooldown)

    def _publish_frame(self, raw):
        """Convert a raw YUYV frame to rotated mono16 and publish it (bad frames are skipped)."""
        try:
            luma = raw.reshape(self._height, self._width, 2)[:, :, 0]
            if self._vflip:
                luma = np.flipud(luma)
            if self._hflip:
                luma = np.fliplr(luma)
            mono16 = luma.astype(np.uint16) << 8
            stamp = self.get_clock().now().to_msg()
            self._publish_image(self._publisher, mono16, 'mono16', stamp)
            if self._preview_publisher is not None:
                colourised = cv2.applyColorMap(luma, cv2.COLORMAP_INFERNO)
                self._publish_image(self._preview_publisher, colourised, 'bgr8', stamp)
        except Exception as exc:
            self.get_logger().error(f'Failed to process/publish frame, skipping: {exc!r}')

    def _publish_image(self, publisher, array, encoding, stamp):
        msg = self._bridge.cv2_to_imgmsg(array, encoding=encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        publisher.publish(msg)

    def destroy_node(self):
        """Stop the capture thread, then release the camera."""
        # Join the capture thread before releasing the device: closing it while a read is in
        # progress can leave this camera unable to stream.
        self._running = False
        if self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=3.0)
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        super().destroy_node()


def main(args=None):
    """Run the thermal camera node until interrupted."""
    rclpy.init(args=args)
    node = ThermalCameraNode()
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
