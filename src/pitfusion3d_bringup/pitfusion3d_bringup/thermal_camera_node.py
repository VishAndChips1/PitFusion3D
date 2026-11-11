#!/usr/bin/env python3
"""Publish the UVC thermal camera (640x512 YUYV) as sensor_msgs/Image.

Opened by its /dev/v4l/by-id/ path (keyed to the device's USB serial),
not a raw /dev/videoN number -- the Pi enumerates video devices in
whatever order the kernel probes them at boot, and the RPi camera's
internal CSI/ISP pipeline (rp1-cfe, several nodes) competes for the same
numbering space as this USB UVC camera. A device that was /dev/video8
one boot came back as /dev/video0 after a reboot in testing, which
starves this node without touching the actual hardware -- the by-id path
is stable across that.

This module (USB VID 0x3474 "Thermal Cam Co.,Ltd") only exposes an 8-bit
AGC'd YUYV video stream over UVC -- there is no vendor SDK available here
to switch it into a raw14/TLinear radiometric mode. So this node samples
the Y (luma) channel, which tracks scene temperature monotonically but is
auto-gain-controlled, NOT an absolute radiometric measurement, and
publishes it left-shifted into 'mono16' (value = Y * 256) so it fits
PitFusion3D's sensor_fusion_node.py, which expects a single-channel
uint16 image and recovers a value via `raw * thermal_resolution +
thermal_min_temp`.

Launch this with thermal_resolution=0.00390625 (1/256) and
thermal_min_temp=0.0 so the recovered 'thermal' field in the fused cloud
comes back as the original 0-255 Y value -- a relative hot/cold signal,
not Kelvin. Swap in a radiometric driver later (e.g. once vendor raw16
mode access is available) without touching anything downstream.
"""
import os
import threading
import time

import cv2

# See rgb_camera_node.py's comment: avoid OpenCV's own internal thread
# pool (sized to core count by default) stacking with every other
# OpenCV-using process in the pipeline on this 4-core board.
cv2.setNumThreads(1)

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ThermalCameraNode(Node):
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
        # Unlike the RGB camera, this module's UVC mode only offers two
        # discrete frame intervals (30fps/60fps -- see its
        # --list-formats-ext output); requesting 15fps here is silently
        # clamped back to 30fps by the driver, so there's no point
        # pretending otherwise. The real CPU savings for this node come
        # from publish_preview=False and cv2.setNumThreads(1) below, not
        # from a framerate this hardware can't actually do.
        self.declare_parameter('framerate', 30)
        # Off by default: applyColorMap on every frame is pure overhead
        # once you're viewing the fused cloud in the web viewer instead of
        # this raw feed; flip on only for direct thermal-camera debugging.
        self.declare_parameter('publish_preview', False)
        self.declare_parameter('preview_topic', '/landfill/thermal/image_preview')
        # This module is physically mounted rotated a full 180 degrees
        # relative to the RGB camera/rig, not just upside down -- a
        # vertical-only flip fixed up/down (confirmed visually: a person
        # standing upright went from head-down/feet-up to upright) but
        # left/right then came back inverted (walking left in the real
        # world showed as walking right), which is exactly what a 180
        # degree in-plane rotation produces when only one axis is
        # corrected. Flipping both axes is a full rotation, which fixes
        # both at once. A parameter rather than a hardcoded flip so it's
        # one launch arg to flip back if the physical mounting ever
        # changes, not a code edit.
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
        # BEST_EFFORT: see rgb_camera_node.py's comment on the same
        # change -- sensor_fusion_node.py and camera_viewer.py's
        # subscriptions to this topic must match.
        self._publisher = self.create_publisher(
            Image, str(self.get_parameter('topic').value), qos_profile_sensor_data
        )
        self._preview_publisher = None
        if self._publish_preview:
            self._preview_publisher = self.create_publisher(
                Image, str(self.get_parameter('preview_topic').value), qos_profile_sensor_data
            )

        # Observed 2026-11-11: this camera can come up after (warm) boot
        # in a state where it enumerates but then fails every read, drops
        # off the USB bus ("USB disconnect", error -71) and re-enumerates
        # dozens of times. Reopening it the instant its device node
        # reappears (mid-enumeration) and again immediately after each
        # failed streak kept it in that loop for hours; the fusion node
        # needs thermal frames to sync, so the viewer stayed blank. Wait
        # for the node to be stable before opening, and cool down for a
        # growing interval after each failed streak.
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
        """Block until the device node has existed continuously for
        device_settle_s (i.e. USB enumeration has finished, not just begun)."""
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
        # Self-healing, matching rgb_camera_node.py: on boot (systemd/
        # lingering, no one watching) this USB device can take a few
        # seconds to enumerate, and a cable hiccup in the field shouldn't
        # need a human to notice and restart anything. Previously this
        # raised in __init__ on a failed open, which would kill the node's
        # *process* -- observable in a log, but ros2 launch doesn't by
        # default propagate a child's exit into its own, so nothing
        # upstream (including a systemd Restart=on-failure on the whole
        # pipeline) was guaranteed to actually act on that. Retrying here
        # instead means the node's own process survives and recovers on
        # its own once the camera is actually available.
        backoff = 1.0
        consecutive_read_failures = 0
        cooldown = 0.0
        good_frames = 0
        while self._running and rclpy.ok():
            if self._cap is None:
                if not self._wait_for_device():
                    break
                self._cap = self._open_capture()
                if self._cap is None:
                    self.get_logger().error(
                        f'Failed to open thermal camera at {self._device}; '
                        f'retrying in {backoff:.0f}s'
                    )
                    time.sleep(backoff)
                    backoff = min(backoff * 2, 10.0)
                    continue
                self.get_logger().info(
                    f'{self._device} capture started '
                    f'({self._width}x{self._height}@{self._framerate}), '
                    f'publishing mono16 on {self._publisher.topic_name} '
                    '(relative AGC signal, not radiometric Kelvin)'
                )
                backoff = 1.0
                consecutive_read_failures = 0
                good_frames = 0

            ok, raw = self._cap.read()
            if not ok or raw is None:
                consecutive_read_failures += 1
                good_frames = 0
                if consecutive_read_failures in (1, 10):
                    self.get_logger().warn(
                        f'Failed to read frame from thermal camera '
                        f'({consecutive_read_failures} in a row)')
                # A handful of isolated drops is normal; many in a row
                # means the device is actually gone (unplugged, etc.) --
                # reopen rather than busy-loop calling read() on a dead
                # capture forever.
                if consecutive_read_failures >= 10:
                    cooldown = min(max(cooldown * 2.0, 3.0), self._cooldown_max)
                    self.get_logger().error(
                        f'Too many consecutive read failures; reopening device '
                        f'after a {cooldown:.0f}s cool-down')
                    self._cap.release()
                    self._cap = None
                    self._sleep_while_running(cooldown)
                else:
                    time.sleep(0.1)
                continue
            consecutive_read_failures = 0
            good_frames += 1
            if good_frames == 30:
                cooldown = 0.0  # genuinely streaming again -- forget the penalty

            # Same gap/fix as rgb_camera_node.py's capture loop: this runs
            # in a daemon thread, so an uncaught exception here (e.g. raw
            # not actually being height*width*2 bytes despite cap.read()
            # reporting success) would silently kill only this thread --
            # no exception reaches rclpy.spin(), no process exit for
            # systemd to react to. The node would look like a healthy
            # running process while never publishing another frame.
            try:
                yuyv = raw.reshape(self._height, self._width, 2)
                y = yuyv[:, :, 0]
                if self._vflip:
                    y = np.flipud(y)
                if self._hflip:
                    y = np.fliplr(y)
                mono16 = (y.astype(np.uint16) << 8)

                stamp = self.get_clock().now().to_msg()

                msg = self._bridge.cv2_to_imgmsg(mono16, encoding='mono16')
                msg.header.stamp = stamp
                msg.header.frame_id = self._frame_id
                self._publisher.publish(msg)

                if self._preview_publisher is not None:
                    colorized = cv2.applyColorMap(y, cv2.COLORMAP_INFERNO)
                    preview_msg = self._bridge.cv2_to_imgmsg(colorized, encoding='bgr8')
                    preview_msg.header.stamp = stamp
                    preview_msg.header.frame_id = self._frame_id
                    self._preview_publisher.publish(preview_msg)
            except Exception as exc:
                self.get_logger().error(f'Failed to process/publish frame, skipping: {exc!r}')

    def destroy_node(self):
        self._running = False
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
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
