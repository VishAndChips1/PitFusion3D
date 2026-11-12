#!/usr/bin/env python3
"""Thermal-camera-only viewer.

Subscribes to /landfill/thermal/image_raw (published by thermal_camera_node, which owns the USB
device -- this script never touches the camera) and serves a colourised live view:

    http://<pi>:8081/             page
    http://<pi>:8081/stream.mjpg  MJPEG stream
    http://<pi>:8081/frame.jpg    single frame

A "NO SIGNAL" picture is served when no frame arrived in the last STALE_AFTER_S seconds. Frames
are only encoded while a client is asking for them, so an idle viewer costs almost nothing.
"""
import http.server
import socketserver
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PORT = 8081
TOPIC = '/landfill/thermal/image_raw'
STALE_AFTER_S = 3.0        # show NO SIGNAL if the newest frame is older than this
IDLE_UNSUBSCRIBE_S = 10.0  # drop the ROS subscription this long after the last request (no viewer = no cost)
STREAM_PERIOD_S = 0.1      # MJPEG frame interval (10 fps is plenty for a status view)
JPEG_QUALITY = 80

PAGE = b"""<!doctype html>
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Thermal camera</title>
<body style="margin:0;background:#111;color:#ddd;font:14px sans-serif;text-align:center">
<p>Thermal camera (relative signal, not absolute temperature)</p>
<img src="/stream.mjpg" style="max-width:100%;image-rendering:pixelated"></body>"""


class LatestFrame:
    """Thread-safe holder for the newest thermal frame (ROS thread writes, HTTP threads read)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._image = None          # uint16 array, value = Y * 256
        self._stamp = 0.0           # time.monotonic() of arrival
        self._last_request = 0.0    # time.monotonic() of the last HTTP request for a picture

    def set(self, image):
        with self._lock:
            self._image, self._stamp = image, time.monotonic()

    def touch(self):
        """Record that a client asked for a picture (keeps the ROS subscription alive)."""
        self._last_request = time.monotonic()

    def wanted(self):
        """True while a client has asked for a picture recently."""
        return time.monotonic() - self._last_request < IDLE_UNSUBSCRIBE_S

    def get(self):
        """Return the newest frame, or None if there is none or it is stale."""
        with self._lock:
            if self._image is None or time.monotonic() - self._stamp > STALE_AFTER_S:
                return None
            return self._image


FRAME = LatestFrame()


def no_signal_picture():
    """A 640x512 black picture explaining that the camera is not delivering frames."""
    picture = np.zeros((512, 640, 3), np.uint8)
    cv2.putText(picture, 'NO SIGNAL', (200, 250), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 255), 3)
    cv2.putText(picture, 'thermal camera is not delivering frames', (95, 300),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
    return picture


def encode_jpeg():
    """Colourise the newest frame (INFERNO palette) and return it as JPEG bytes."""
    FRAME.touch()
    image = FRAME.get()
    if image is None:
        picture = no_signal_picture()
    else:
        picture = cv2.applyColorMap((image >> 8).astype(np.uint8), cv2.COLORMAP_INFERNO)
    return cv2.imencode('.jpg', picture, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])[1].tobytes()


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        """Silence the default per-request logging."""

    def _reply(self, status, content_type=None, body=b''):
        self.send_response(status)
        if content_type:
            self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        while True:
            jpeg = encode_jpeg()
            self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(jpeg)
                             + jpeg + b'\r\n')
            time.sleep(STREAM_PERIOD_S)

    def do_GET(self):
        try:
            if self.path.startswith('/stream.mjpg'):
                self._stream()
            elif self.path.startswith('/frame.jpg'):
                self._reply(200, 'image/jpeg', encode_jpeg())
            elif self.path == '/':
                self._reply(200, 'text/html', PAGE)
            else:
                self._reply(404)
        except (BrokenPipeError, ConnectionError, TimeoutError):
            pass    # the browser went away


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def on_image(msg):
    if msg.encoding == 'mono16':
        FRAME.set(np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width))


def main():
    rclpy.init()
    node = Node('thermal_view')
    state = {'subscription': None}

    def manage_subscription():
        """Subscribe while someone is watching, unsubscribe when nobody has asked for a while."""
        if FRAME.wanted() and state['subscription'] is None:
            state['subscription'] = node.create_subscription(Image, TOPIC, on_image, qos_profile_sensor_data)
        elif not FRAME.wanted() and state['subscription'] is not None:
            node.destroy_subscription(state['subscription'])
            state['subscription'] = None

    node.create_timer(1.0, manage_subscription)
    threading.Thread(target=Server(('0.0.0.0', PORT), Handler).serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
