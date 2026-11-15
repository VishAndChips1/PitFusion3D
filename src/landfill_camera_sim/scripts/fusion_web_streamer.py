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
import json
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


# Point counts a client may ask for with /merged.bin?n=...; the server's cap is always the top level.
TIERS = (8000, 20000, 40000, 60000, 80000, 120000)
LIVE_IDLE_S = 20.0   # stop receiving the live cloud this long after the last /live.bin request


class _StreamState:
    """One served point cloud source, encoded lazily at the point count each client asks for.

    A viewer on a weak link asks for fewer points (?n=); each (cloud, level) pair is encoded once and shared by every
    client. Every encoded payload gets its own increasing sequence number, so a level change is never mistaken for old data.
    """

    def __init__(self):
        self.encode_lock = threading.Lock()
        self.latest = (0, None)     # (cloud id, message), replaced as one object so readers never see a mixed pair
        self.encoder = None         # encoder(msg, max_points) -> (n_points, payload bytes), format 1
        self.encoder_v2 = None      # the same for the compact format 2
        self.max_points = 20000     # the server-side ceiling (the max_points_* parameter)
        self.last_request = 0.0     # monotonic time of the last client request
        self._cache = {}            # (level, format) -> (gzipped payload, seq, capture stamp) for cloud _cache_id
        self._cache_id = 0
        self._seq = 0               # increases with every payload encoded, so a payload is never mistaken for another
        self._empty = (gzip.compress(struct.pack('<II', 0, 0), compresslevel=6), 0, 0.0)

    def offer(self, msg):
        """Remember the newest cloud (called from the ROS callback)."""
        self.latest = (self.latest[0] + 1, msg)

    def levels(self):
        """List the selectable point counts, ending at the server ceiling."""
        return [t for t in TIERS if t < self.max_points] + [self.max_points]

    def get(self, requested=None, fmt=1):
        """Return (gzipped payload, seq, stamp) of the newest cloud at the largest level not above `requested`."""
        self.last_request = time.monotonic()
        levels = self.levels()
        idx = len(levels) - 1
        if requested is not None:
            idx = max([i for i, t in enumerate(levels) if t <= requested], default=0)
        encoder = self.encoder_v2 if fmt == 2 else self.encoder
        if encoder is None:
            return self._empty
        with self.encode_lock:
            lid, msg = self.latest
            if msg is None:
                return self._empty
            if lid != self._cache_id:
                self._cache, self._cache_id = {}, lid
            entry = self._cache.get((levels[idx], fmt))
            if entry is None:
                n, payload = encoder(msg, levels[idx])
                self._seq += 1
                # Format 2 is already delta-coded: level 1 is 5x faster than 6 for about 2 % more bytes.
                body = gzip.compress(struct.pack('<II', n, self._seq) + payload, compresslevel=1 if fmt == 2 else 6)
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                entry = self._cache[(levels[idx], fmt)] = (body, self._seq, stamp)
            return entry


class _Counters:
    """What the server saw, so a failed frame on the viewer's side can be explained from the Pi's side."""

    def __init__(self):
        self.lock = threading.Lock()
        self.started = time.time()
        self.c = {'requests': 0, 'sent_200': 0, 'not_modified_304': 0, 'aborted_by_client': 0, 'bytes_sent': 0,
                  'write_seconds': 0.0, 'write_seconds_max': 0.0, 'format2': 0}

    def add(self, **kw):
        with self.lock:
            for k, v in kw.items():
                self.c[k] = max(self.c[k], v) if k.endswith('_max') else self.c[k] + v

    def snapshot(self):
        with self.lock:
            out = dict(self.c)
        out['uptime_s'] = round(time.time() - self.started)
        sent = max(out['sent_200'], 1)
        out['mean_write_s'] = round(out['write_seconds'] / sent, 3)
        out['abort_pct'] = round(100.0 * out['aborted_by_client'] / max(out['sent_200'] + out['aborted_by_client'], 1), 1)
        return out


class _SharedState:
    """The two cloud sources the server can serve, plus the request counters."""

    def __init__(self):
        self.merged = _StreamState()
        self.live = _StreamState()
        self.counters = _Counters()


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

    def _serve_cloud(self, stream_state, requested=None, fmt=1):
        """Serve a cloud as gzip, or a bodyless 304 if the client already has this sequence number."""
        counters = self.shared_state.counters
        counters.add(requests=1)
        try:
            body, seq, stamp = stream_state.get(requested, fmt)
        except Exception:
            body, seq, stamp = stream_state._empty   # serve an empty cloud rather than fail the request
        etag = f'"{seq}"'
        if self.headers.get('If-None-Match') == etag:
            self.send_response(304)
            self.send_header('ETag', etag)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            counters.add(not_modified_304=1)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'application/octet-stream')
        self.send_header('Content-Encoding', 'gzip')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('ETag', etag)
        # The point counts a client may ask for with ?n= (an adaptive client reads this; older clients ignore it).
        self.send_header('X-Quality-Levels', ','.join(str(n) for n in stream_state.levels()))
        self.send_header('X-Formats', '1,2' if stream_state.encoder_v2 is not None else '1')   # ?fmt=2 asks for the compact format
        # Data age = X-Server-Time - X-Cloud-Stamp (both from the Pi's clock).
        self.send_header('X-Cloud-Stamp', f'{stamp:.3f}')
        self.send_header('X-Server-Time', f'{time.time():.3f}')
        self.send_header('Access-Control-Expose-Headers', 'X-Cloud-Stamp, X-Server-Time')
        self.end_headers()
        t0 = time.monotonic()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            counters.add(aborted_by_client=1)   # client (flaky WiFi, closed tab) went away mid-write
            self.close_connection = True
            return
        took = time.monotonic() - t0
        counters.add(sent_200=1, bytes_sent=len(body), write_seconds=took, write_seconds_max=took, format2=1 if fmt == 2 else 0)

    def do_GET(self):
        """Route: page, merged/live clouds (merged takes ?n=<points>), static scripts, else 404."""
        path, _, query = self.path.partition('?')
        requested, fmt = None, 1
        for part in query.split('&'):
            if part.startswith('n=') and part[2:].isdigit():
                requested = int(part[2:])
            elif part == 'fmt=2':
                fmt = 2
        if path in ('/', '/index.html'):
            self._serve_bytes(self.page_html, 'text/html; charset=utf-8')
        elif path in ('/merged.bin', '/latest.bin'):
            self._serve_cloud(self.shared_state.merged, requested, fmt)
        elif path == '/live.bin':
            self._serve_cloud(self.shared_state.live, None, fmt)
        elif path == '/stats':
            self._serve_bytes(json.dumps(self.shared_state.counters.snapshot()).encode(), 'application/json')
        elif self.static_assets and path in self.static_assets:
            body, content_type = self.static_assets[path]
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


V2_MIN_QUANTUM = 0.0025    # m: position grid of the compact format (LiDAR noise is ~20 mm, so 2.5 mm loses nothing visible)


def _morton_keys(q):
    """Z-order key of integer grid coordinates (each < 2**21): points close in space get close keys."""
    def spread(v):
        v = v.astype(np.uint64) & np.uint64(0x1FFFFF)
        v = (v | (v << np.uint64(32))) & np.uint64(0x1F00000000FFFF)
        v = (v | (v << np.uint64(16))) & np.uint64(0x1F0000FF0000FF)
        v = (v | (v << np.uint64(8))) & np.uint64(0x100F00F00F00F00F)
        v = (v | (v << np.uint64(4))) & np.uint64(0x10C30C30C30C30C3)
        v = (v | (v << np.uint64(2))) & np.uint64(0x1249249249249249)
        return v
    return spread(q[:, 0]) | (spread(q[:, 1]) << np.uint64(1)) | (spread(q[:, 2]) << np.uint64(2))


def _encode_cloud_v2(msg, max_points, rng):
    """Pack a cloud into the compact wire format (v2): about half the bytes of v1 and several times faster to build.

    Points are sorted along a space-filling curve and each coordinate is sent as the difference from the previous
    point (neighbours are close, so the differences are small and compress well). Layout after the 8-byte n/seq
    header added by the caller: origin xyz and grid size (4 x float32), flags (uint32: bit 0 = thermal present),
    then n x int16 for each of dx, dy, dz, [n x int16 thermal in 0.01 units], then n x uint8 for each of dr, dg, db
    (differences taken modulo 256, so a running sum reproduces the colours exactly).
    Returns (number of points, payload bytes).
    """
    points = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z', 'rgb', 'thermal'])
    if points.size == 0:
        return 0, struct.pack('<ffffI', 0.0, 0.0, 0.0, 1.0, 0)
    n = points.shape[0]
    if n > max_points:
        points = points[rng.choice(n, size=max_points, replace=False)]
        n = points.shape[0]
    xyz = points[:, 0:3].astype(np.float64)
    mins = xyz.min(axis=0)
    quantum = max(float((xyz.max(axis=0) - mins).max()) / 32000.0, V2_MIN_QUANTUM)
    grid = np.round((xyz - mins) / quantum).astype(np.int64)
    order = np.argsort(_morton_keys(grid))
    points, grid = points[order], grid[order]

    deltas = np.diff(grid, axis=0, prepend=np.zeros((1, 3), np.int64)).astype('<i2')
    packed = points[:, 3].astype(np.float32).view(np.uint32)
    colour = np.stack([(packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF], axis=1).astype(np.int16)
    colour_delta = (np.diff(colour, axis=0, prepend=np.zeros((1, 3), np.int16)) & 0xFF).astype(np.uint8)
    thermal = points[:, 4].astype(np.float32)
    finite = np.isfinite(thermal)
    has_thermal = bool(finite.any())

    parts = [struct.pack('<ffffI', mins[0], mins[1], mins[2], quantum, 1 if has_thermal else 0)]
    parts += [deltas[:, i].tobytes() for i in range(3)]
    if has_thermal:
        t_q = np.full(n, _THERMAL_SENTINEL, dtype=np.int16)
        t_q[finite] = np.clip(np.round(thermal[finite] * 100.0), -32767, 32767).astype(np.int16)
        parts.append(t_q.tobytes())
    parts += [colour_delta[:, i].tobytes() for i in range(3)]
    return n, b''.join(parts)


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
        # Ceiling on points per served cloud. Viewers on weaker links ask for fewer (?n=). 80000 was measured over a
        # 20-minute soak with real WiFi-radio load: 1.5 updates/s, about 7.5 Mbit/s, worst-case input current 3.1 A.
        self.declare_parameter('max_points_merged', 80000)
        self.declare_parameter('max_points_live', 5000)
        # Older single-stream parameter name: overrides merged_topic when set.
        self.declare_parameter('input_topic', '')
        self.declare_parameter('http_keepalive', True)

        merged_topic = str(self.get_parameter('input_topic').value) or str(self.get_parameter('merged_topic').value)
        live_topic = str(self.get_parameter('live_topic').value)
        self._http_port = int(self.get_parameter('http_port').value)
        # The caps can be changed at runtime: ros2 param set /fusion_web_streamer max_points_merged N
        self.add_on_set_parameters_callback(self._on_param_change)

        http_keepalive = bool(self.get_parameter('http_keepalive').value)

        self._state = _SharedState()
        _rng_merged, _rng_live = np.random.default_rng(0), np.random.default_rng(1)
        self._state.merged.max_points = int(self.get_parameter('max_points_merged').value)
        self._state.live.max_points = int(self.get_parameter('max_points_live').value)
        self._state.merged.encoder = lambda m, n: _encode_cloud(m, n, _rng_merged)
        self._state.live.encoder = lambda m, n: _encode_cloud(m, n, _rng_live)
        self._state.merged.encoder_v2 = lambda m, n: _encode_cloud_v2(m, n, _rng_merged)
        self._state.live.encoder_v2 = lambda m, n: _encode_cloud_v2(m, n, _rng_live)
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
        # The live cloud (10 Hz) is only received while a client is asking for it: merged-only viewing then costs no live work.
        self._live_topic = live_topic
        self._live_sub = None
        self.create_timer(1.0, self._manage_live_subscription)

        self.get_logger().info(
            'Serving merged=%s (max %d pts) + live=%s (max %d pts) at http://0.0.0.0:%d/ (keep-alive: %s)'
            % (merged_topic, self._state.merged.max_points, live_topic, self._state.live.max_points,
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
                self._state.merged.max_points = int(p.value)
            elif p.name == 'max_points_live':
                self._state.live.max_points = int(p.value)
        return SetParametersResult(successful=True)

    def _manage_live_subscription(self):
        """Receive the live cloud only while /live.bin was requested within LIVE_IDLE_S."""
        wanted = time.monotonic() - self._state.live.last_request < LIVE_IDLE_S and self._state.live.last_request > 0
        if wanted and self._live_sub is None:
            self._live_sub = self.create_subscription(PointCloud2, self._live_topic, self._on_live, qos_profile_sensor_data)
        elif not wanted and self._live_sub is not None:
            self.destroy_subscription(self._live_sub)
            self._live_sub = None
            self._state.live.latest = (self._state.live.latest[0], None)

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
