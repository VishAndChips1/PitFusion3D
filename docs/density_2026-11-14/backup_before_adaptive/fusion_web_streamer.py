#!/usr/bin/env python3
"""Serve the fused point clouds to any browser on the network.

Subscribes to the per-scan fused cloud (live, ~10 Hz) and the ICP-merged cloud
(denser, ~1.5 Hz), keeps each as a compact quantised binary buffer and serves
them, plus a three.js viewer page (web_assets/viewer.html), over plain HTTP.
No ROS is needed on the viewing device: open http://<this-machine>:<port>/.
All scripts are served locally, so the page also works on the Pi's own offline
hotspot where no CDN is reachable.
"""

import gzip
import http.server
import os
import socketserver
import struct
import sys
import threading
import time

import numpy as np

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class _ReusableThreadingTCPServer(socketserver.ThreadingTCPServer):
    """Threaded HTTP server that can rebind its port straight after a restart."""

    allow_reuse_address = True

    def handle_error(self, request, client_address):
        # A viewer dropping off WiFi mid-request is routine: stay quiet.
        exc = sys.exc_info()[1]
        if isinstance(exc, (ConnectionError, TimeoutError)):
            return
        super().handle_error(request, client_address)


class _StreamState:
    """One served point cloud source: the gzipped payload, its sequence number and capture stamp.

    The sequence number lets a client skip a payload it already has.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.buffer = gzip.compress(struct.pack('<II', 0, 0), compresslevel=6)
        self.seq = 0
        # Lazy encoding: the ROS callback only keeps the newest cloud; it is quantised and
        # gzipped when a client asks for newer data, so an idle server costs nothing.
        self.encode_lock = threading.Lock()
        self.latest_msg = None
        self.latest_id = 0
        self.encoded_id = 0
        self.encoder = None
        self.stamp = 0.0    # header stamp (s) of the cloud currently served

    def offer(self, msg):
        """Remember the newest cloud (called from the ROS callback)."""
        self.latest_msg = msg
        self.latest_id += 1

    def refresh(self):
        """Encode the newest cloud if it has not been encoded yet."""
        if self.encoder is None or self.latest_id == self.encoded_id:
            return
        with self.encode_lock:
            lid, msg = self.latest_id, self.latest_msg
            if msg is None or lid == self.encoded_id:
                return
            n, payload = self.encoder(msg)
            self.set_payload(n, payload, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            self.encoded_id = lid

    def set_payload(self, n_points, payload_bytes, stamp=0.0):
        # Compress once per data update, not once per poll.
        with self.lock:
            next_seq = self.seq + 1
            body = gzip.compress(
                struct.pack('<II', n_points, next_seq) + payload_bytes, compresslevel=6
            )
            self.seq = next_seq
            self.buffer = body
            self.stamp = stamp

    def get_buffer_seq_stamp(self):
        """Return (gzipped payload, sequence number, capture stamp) consistently."""
        with self.lock:
            return self.buffer, self.seq, self.stamp


class _SharedState:
    """The two cloud sources the server can serve."""

    def __init__(self):
        self.merged = _StreamState()
        self.live = _StreamState()


class _Handler(http.server.BaseHTTPRequestHandler):
    shared_state: _SharedState = None
    static_assets: dict = None   # path -> (body bytes, content type), loaded at startup
    page_html: bytes = b''
    # HTTP/1.1 keep-alive: avoids a new TCP connection per poll, which cost real CPU and
    # peak current over WiFi. The 'http_keepalive' parameter switches back to HTTP/1.0.
    protocol_version = 'HTTP/1.1'
    timeout = 30   # seconds before an idle keep-alive connection is closed

    def log_message(self, format_, *args):
        """Silence per-request logging."""

    def _serve_bytes(self, body, content_type):
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass  # client (flaky WiFi, closed tab) disconnected mid-write

    def _serve_cloud(self, stream_state):
        """Serve a cloud as gzip, or a bodyless 304 if the client already has this sequence number."""
        try:
            stream_state.refresh()
        except Exception:
            pass  # serve the previous payload rather than fail the request
        body, seq, stamp = stream_state.get_buffer_seq_stamp()
        etag = f'"{seq}"'
        if self.headers.get('If-None-Match') == etag:
            self.send_response(304)
            self.send_header('ETag', etag)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-Type', 'application/octet-stream')
        self.send_header('Content-Encoding', 'gzip')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('ETag', etag)
        # Data age = X-Server-Time - X-Cloud-Stamp (both from the Pi's clock).
        self.send_header('X-Cloud-Stamp', f'{stamp:.3f}')
        self.send_header('X-Server-Time', f'{time.time():.3f}')
        self.send_header('Access-Control-Expose-Headers', 'X-Cloud-Stamp, X-Server-Time')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass  # client (flaky WiFi, closed tab) disconnected mid-write

    def do_GET(self):
        """Route: page, merged/live clouds, static scripts, else 404."""
        if self.path in ('/', '/index.html'):
            self._serve_bytes(self.page_html, 'text/html; charset=utf-8')
        elif self.path in ('/merged.bin', '/latest.bin'):
            self._serve_cloud(self.shared_state.merged)
        elif self.path == '/live.bin':
            self._serve_cloud(self.shared_state.live)
        elif self.static_assets and self.path in self.static_assets:
            body, content_type = self.static_assets[self.path]
            self._serve_bytes(body, content_type)
        else:
            # Content-Length is required under HTTP/1.1 keep-alive.
            self.send_response(404)
            self.send_header('Content-Length', '0')
            self.end_headers()


_THERMAL_SENTINEL = -32768  # int16 "no thermal value" marker (source was NaN)


def _encode_cloud(msg, max_points, rng):
    """Pack a cloud into the compact wire format the viewer reads.

    Header: origin xyz and scale (4 x float32). Then 11 bytes per point: xyz as int16 (origin + value * scale,
    sub-millimetre error), rgb as 3 x uint8, thermal as int16 in 0.01 units (-32768 = no value).
    Returns (number of points, payload bytes).
    """
    points = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z', 'rgb', 'thermal'])
    if points.size == 0:
        return 0, struct.pack('<ffff', 0.0, 0.0, 0.0, 1.0)
    n = points.shape[0]
    if n > max_points:
        idx = rng.choice(n, size=max_points, replace=False)
        points = points[idx]
        n = points.shape[0]

    xyz = points[:, 0:3].astype(np.float64)  # headroom for the subtraction below
    mins = xyz.min(axis=0)
    extent = float((xyz.max(axis=0) - mins).max())
    scale = max(extent, 1e-6) / 32000.0
    xyz_q = np.round((xyz - mins) / scale).astype(np.int16)

    packed_rgb = points[:, 3].astype(np.float32).view(np.uint32)
    r = ((packed_rgb >> 16) & 0xFF).astype(np.uint8)
    g = ((packed_rgb >> 8) & 0xFF).astype(np.uint8)
    b = (packed_rgb & 0xFF).astype(np.uint8)

    thermal = points[:, 4].astype(np.float32)
    finite = np.isfinite(thermal)
    thermal_q = np.full(n, _THERMAL_SENTINEL, dtype=np.int16)
    thermal_q[finite] = np.clip(np.round(thermal[finite] * 100.0), -32767, 32767).astype(np.int16)

    header = struct.pack('<ffff', mins[0], mins[1], mins[2], scale)
    point_dtype = np.dtype([
        ('xyz', np.int16, 3), ('rgb', np.uint8, 3), ('t', np.int16),
    ])
    point_data = np.empty(n, dtype=point_dtype)
    point_data['xyz'] = xyz_q
    point_data['rgb'] = np.stack([r, g, b], axis=1)
    point_data['t'] = thermal_q
    return n, header + point_data.tobytes()


_WEB_ASSETS_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'web_assets')


def _load_web_assets():
    """Read the viewer page and its scripts from web_assets/ once at startup."""
    def read(name):
        with open(os.path.join(_WEB_ASSETS_DIR, name), 'rb') as f:
            return f.read()

    scripts = {f'/{name}': (read(name), 'application/javascript') for name in ('three.min.js', 'OrbitControls.js')}
    return read('viewer.html'), scripts


class FusionWebStreamer(Node):
    """ROS node that feeds the two cloud sources and runs the HTTP server."""

    def __init__(self):
        super().__init__('fusion_web_streamer')

        self.declare_parameter('merged_topic', '/landfill/fusion/points_merged')
        self.declare_parameter('live_topic', '/landfill/fusion/points_rgbt')
        self.declare_parameter('http_port', 8080)
        # Point caps per served cloud. They set the transfer size per update, and 20000 / 5000
        # were measured to keep the peak input current under 5 A during real WiFi viewing.
        self.declare_parameter('max_points_merged', 20000)
        self.declare_parameter('max_points_live', 5000)
        # Older single-stream parameter name: overrides merged_topic when set.
        self.declare_parameter('input_topic', '')
        self.declare_parameter('http_keepalive', True)

        merged_topic = str(self.get_parameter('input_topic').value) or str(self.get_parameter('merged_topic').value)
        live_topic = str(self.get_parameter('live_topic').value)
        self._http_port = int(self.get_parameter('http_port').value)
        self._max_points_merged = int(self.get_parameter('max_points_merged').value)
        self._max_points_live = int(self.get_parameter('max_points_live').value)
        # The caps can be changed at runtime: ros2 param set /fusion_web_streamer max_points_merged N
        self.add_on_set_parameters_callback(self._on_param_change)

        http_keepalive = bool(self.get_parameter('http_keepalive').value)

        self._state = _SharedState()
        _rng_merged, _rng_live = np.random.default_rng(0), np.random.default_rng(1)
        self._state.merged.encoder = lambda m: _encode_cloud(m, self._max_points_merged, _rng_merged)
        self._state.live.encoder = lambda m: _encode_cloud(m, self._max_points_live, _rng_live)
        page_html, static_assets = _load_web_assets()
        handler = type(
            'BoundHandler', (_Handler,),
            {
                'shared_state': self._state,
                'static_assets': static_assets,
                'page_html': page_html,
                'protocol_version': 'HTTP/1.1' if http_keepalive else 'HTTP/1.0',
            },
        )
        self._server = _ReusableThreadingTCPServer(('0.0.0.0', self._http_port), handler)
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

        # Best-effort QoS, matching the publishers.
        self._merged_sub = self.create_subscription(
            PointCloud2, merged_topic, self._on_merged, qos_profile_sensor_data
        )
        self._live_sub = self.create_subscription(
            PointCloud2, live_topic, self._on_live, qos_profile_sensor_data
        )

        self.get_logger().info(
            'Serving merged=%s (max %d pts) + live=%s (max %d pts) at http://0.0.0.0:%d/ (keep-alive: %s)'
            % (merged_topic, self._max_points_merged, live_topic, self._max_points_live,
               self._http_port, http_keepalive)
        )

    def _on_param_change(self, params):
        """Validate and apply runtime changes to the point caps."""
        for p in params:
            if p.name in ('max_points_merged', 'max_points_live'):
                if not isinstance(p.value, int) or not (100 <= p.value <= 1_000_000):
                    return SetParametersResult(successful=False, reason='must be an int in [100, 1000000]')
        for p in params:
            if p.name == 'max_points_merged':
                self._max_points_merged = int(p.value)
            elif p.name == 'max_points_live':
                self._max_points_live = int(p.value)
        return SetParametersResult(successful=True)

    def _on_merged(self, msg):
        """Keep the newest merged cloud."""
        self._state.merged.offer(msg)

    def _on_live(self, msg):
        """Keep the newest live cloud."""
        self._state.live.offer(msg)

    def destroy_node(self):
        """Stop the HTTP server, then the node."""
        self._server.shutdown()
        super().destroy_node()


def main():
    """Run the streamer node until interrupted."""
    rclpy.init()
    node = FusionWebStreamer()
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
