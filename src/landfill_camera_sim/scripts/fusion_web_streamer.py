#!/usr/bin/env python3
"""Serve the fused RGB+thermal point cloud to any browser on the network.

Subscribes to both the per-scan fused cloud (x,y,z,rgb,thermal, ~10Hz,
sparser -- sensor_fusion_node.py's direct output) and the windowed ICP
merge (~1Hz, denser -- registration_merge_node.py's output), keeps each
as a compact binary buffer, and serves them plus a self-contained
three.js viewer page over plain HTTP -- no ROS install needed on the
viewing device, just a browser pointed at http://<this-machine>:<port>/.
The page defaults to the denser merged view and lets you switch to the
faster live one with one click.

This node only binds a local port; reaching it from another device on
the same WiFi network still depends on the network path from that
device to this machine (see the package README for the WSL2 note).

Three.js and OrbitControls are served from web_assets/ alongside this
script, not loaded from a CDN. When the viewer is a device connected to
the Pi's own standalone hotspot (PitFusion3D-AP, no uplink -- see
pitfusion3d_bringup's start_hotspot_viewing.sh), that device has no
route to any CDN at all: the page's <script src="https://.../three.min.js">
tag would just fail to load, THREE stays undefined, and the entire page
script throws on its very first line (`new THREE.Scene()`) -- before
the fetch/poll loop that fills the view ever runs. That's a silent,
totally blank failure with nothing in the served HTML to suggest why.
Serving both files locally means the page has zero internet dependency,
which is the actual requirement for a standalone field deployment.
"""

import http.server
import os
import socketserver
import struct
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


PAGE_HTML = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>PitFusion3D live view</title>
<style>
  html, body { margin:0; height:100%; background:#0b0f14; color:#e6edf3;
    font-family: -apple-system, Segoe UI, Roboto, sans-serif; overflow:hidden; }
  #hud { position:absolute; top:12px; left:12px; z-index:2; background:rgba(15,20,27,0.75);
    border:1px solid #26313f; border-radius:10px; padding:10px 14px; backdrop-filter: blur(6px);
    min-width:190px; }
  #hud h1 { font-size:14px; margin:0 0 6px; font-weight:600; letter-spacing:.02em; color:#9fd3ff; }
  #hud div.row { font-size:12px; color:#9aa7b4; margin-top:2px; }
  #hud .group { margin-top:8px; display:flex; gap:6px; }
  #hud button { background:#1b2734; color:#e6edf3; border:1px solid #33445a;
    border-radius:6px; padding:6px 10px; font-size:12px; cursor:pointer; flex:1; }
  #hud button:hover { background:#243447; }
  #hud button.active { background:#2563eb; border-color:#2563eb; }
  #hud button.fit { margin-top:8px; width:100%; }
  #stat-hz { color:#7ee787; font-weight:600; }
  #stat-hz.stale { color:#f0883e; }
  canvas { display:block; }
</style>
</head>
<body>
<div id="hud">
  <h1>PitFusion3D &middot; live fused cloud</h1>
  <div class="row">points: <span id="stat-points">--</span></div>
  <div class="row">refresh: <span id="stat-hz">--</span></div>
  <div class="row">last update: <span id="stat-age">--</span></div>
  <div class="group">
    <button id="mode-rgb" class="active">RGB</button>
    <button id="mode-thermal">Thermal</button>
  </div>
  <div class="group">
    <button id="src-merged" class="active">Merged (dense)</button>
    <button id="src-live">Live (fast)</button>
  </div>
  <button id="fit-view" class="fit">Fit view</button>
</div>
<script src="/three.min.js"></script>
<script src="/OrbitControls.js"></script>
<script>
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
// Depth cueing: without this, distant and near points are equally sharp
// and equally bright, which reads as a flat, hard-to-parse smear rather
// than a 3D scene. Fog fading toward the background color gives the eye
// a real depth cue for free.
scene.fog = new THREE.Fog(0x0b0f14, 3, 30);
const camera = new THREE.PerspectiveCamera(60, window.innerWidth/window.innerHeight, 0.05, 500);
camera.position.set(6, -10, 6);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setSize(window.innerWidth, window.innerHeight);
// This was the main cause of the view looking "pixely": with no pixel
// ratio set, three.js renders at 1 canvas pixel per CSS pixel regardless
// of the display's actual density, so every point (and the whole scene)
// comes out visibly blocky on any modern high-DPI laptop screen. Capped
// at 2 rather than the raw devicePixelRatio (which can be 3+ on some
// 4K/Retina-class panels) to avoid needlessly quadrupling the Pi's own
// render workload for a difference the eye won't notice past 2x.
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
document.body.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.target.set(0,0,0);

const grid = new THREE.GridHelper(40, 40, 0x2a3a4d, 0x1a2531);
grid.rotation.x = Math.PI/2;
scene.add(grid);
scene.add(new THREE.AxesHelper(1.5));

// A soft circular sprite instead of THREE's default hard-edged square
// point: square points packed at any real density look like a blocky
// mosaic (part of the "pixely" complaint) -- round, soft-edged ones read
// as a smooth, photographic point cloud at the same density.
function makeDiscTexture() {
  const size = 64;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');
  const gradient = ctx.createRadialGradient(size/2, size/2, 0, size/2, size/2, size/2);
  gradient.addColorStop(0.0, 'rgba(255,255,255,1)');
  gradient.addColorStop(0.7, 'rgba(255,255,255,1)');
  gradient.addColorStop(1.0, 'rgba(255,255,255,0)');
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, size, size);
  const tex = new THREE.CanvasTexture(canvas);
  tex.needsUpdate = true;
  return tex;
}
const discTexture = makeDiscTexture();

let geometry = new THREE.BufferGeometry();
// Calibrated from measured point spacing, not guessed: a live merged
// cloud's nearest-neighbor spacing measured ~1.4cm median (live mode,
// sparser/no ICP densification, ~2.1cm). Rendering at a size much larger
// than that spacing makes adjacent points overlap into solid blobs --
// confirmed visually (identical data re-rendered at ~1/4 this size went
// from "mostly black blobs, no visible structure" to individual points
// and real geometric edges being distinguishable). ~1.3-1.5x the
// measured spacing is the standard point-cloud-rendering target: just
// enough overlap to read as a continuous surface without blobbing.
const POINT_SIZE = {merged: 0.018, live: 0.028};
let material = new THREE.PointsMaterial({
  size: POINT_SIZE.merged, vertexColors: true, sizeAttenuation: true,
  map: discTexture, transparent: true, alphaTest: 0.1, depthWrite: true,
});
let pointsObj = new THREE.Points(geometry, material);
scene.add(pointsObj);

let mode = 'rgb';
let source = 'merged';
let lastXYZ = null, lastRGB = null, lastThermal = null;
let haveFitOnce = false;
let lastFetchWasNewData = false;
let recentUpdateTimestamps = [];

document.getElementById('mode-rgb').onclick = () => setMode('rgb');
document.getElementById('mode-thermal').onclick = () => setMode('thermal');
document.getElementById('src-merged').onclick = () => setSource('merged');
document.getElementById('src-live').onclick = () => setSource('live');
document.getElementById('fit-view').onclick = () => fitView();

function setMode(m) {
  mode = m;
  document.getElementById('mode-rgb').classList.toggle('active', m === 'rgb');
  document.getElementById('mode-thermal').classList.toggle('active', m === 'thermal');
  applyColors();
}

function setSource(s) {
  source = s;
  material.size = POINT_SIZE[s];
  document.getElementById('src-merged').classList.toggle('active', s === 'merged');
  document.getElementById('src-live').classList.toggle('active', s === 'live');
  recentUpdateTimestamps = [];
  haveFitOnce = false; // re-fit once when the new source's first frame lands
}

function thermalColormap(t, tmin, tmax) {
  const x = Math.max(0, Math.min(1, (t - tmin) / Math.max(1e-6, (tmax - tmin))));
  // dark blue -> cyan -> yellow -> red
  const stops = [
    [0.02,0.02,0.20], [0.0,0.55,0.75], [0.95,0.85,0.15], [0.85,0.10,0.05]
  ];
  const seg = Math.min(2, Math.floor(x * 3));
  const f = x * 3 - seg;
  const a = stops[seg], b = stops[seg+1];
  return [a[0]+(b[0]-a[0])*f, a[1]+(b[1]-a[1])*f, a[2]+(b[2]-a[2])*f];
}

function applyColors() {
  if (!lastXYZ) return;
  const n = lastXYZ.length / 3;
  const colors = new Float32Array(n * 3);
  if (mode === 'rgb') {
    colors.set(lastRGB);
  } else {
    let tmin = Infinity, tmax = -Infinity;
    for (let i = 0; i < n; i++) {
      const t = lastThermal[i];
      if (isFinite(t)) { if (t < tmin) tmin = t; if (t > tmax) tmax = t; }
    }
    if (!isFinite(tmin)) { tmin = 0; tmax = 1; }
    for (let i = 0; i < n; i++) {
      const t = lastThermal[i];
      const c = isFinite(t) ? thermalColormap(t, tmin, tmax) : [0.15,0.15,0.15];
      colors[i*3] = c[0]; colors[i*3+1] = c[1]; colors[i*3+2] = c[2];
    }
  }
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
}

function fitView() {
  geometry.computeBoundingSphere();
  const sphere = geometry.boundingSphere;
  if (!sphere || !isFinite(sphere.radius) || sphere.radius <= 0) return;
  const dist = Math.max(1.5, sphere.radius * 2.4);
  const c = sphere.center;
  controls.target.set(c.x, c.y, c.z);
  camera.position.set(c.x + dist*0.55, c.y - dist*0.85, c.z + dist*0.55);
  camera.updateProjectionMatrix();
  controls.update();
}

function recordUpdate() {
  const now = performance.now();
  recentUpdateTimestamps.push(now);
  while (recentUpdateTimestamps.length && now - recentUpdateTimestamps[0] > 5000) {
    recentUpdateTimestamps.shift();
  }
  const hzEl = document.getElementById('stat-hz');
  if (recentUpdateTimestamps.length >= 2) {
    const span = (recentUpdateTimestamps[recentUpdateTimestamps.length-1] - recentUpdateTimestamps[0]) / 1000;
    const hz = (recentUpdateTimestamps.length - 1) / Math.max(1e-3, span);
    hzEl.textContent = hz.toFixed(2) + ' Hz';
    hzEl.classList.remove('stale');
  }
}

async function poll() {
  // A fetch() with no timeout can hang forever on a real network (a NAT'd
  // hotspot connection going stale, a dropped ACK, anything) -- when that
  // happened here, the whole polling loop died silently and permanently,
  // since setTimeout(poll, ...) below never got scheduled again. Every
  // single unexplained "loads once, then nothing" report traced back to
  // this. AbortController + a hard timeout guarantees this function
  // always returns and the loop always continues, no matter what the
  // network does to any one request.
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 4000);
  try {
    const path = source === 'merged' ? '/merged.bin' : '/live.bin';
    // 'no-cache' (not 'no-store'): always revalidate with the server, but
    // let the browser send If-None-Match and reuse its cached body on a
    // 304 -- critical over real WiFi, where re-downloading an unchanged
    // multi-hundred-KB point cloud on every poll is the difference
    // between smooth and unusable.
    const res = await fetch(path, {cache:'no-cache', signal: controller.signal});
    if (res.ok) {
      const buf = await res.arrayBuffer();
      if (buf.byteLength >= 4) {
        const dv = new DataView(buf);
        const n = dv.getUint32(0, true);
        const seq = dv.getUint32(4, true);
        if (n > 0 && seq !== poll._lastSeq) {
          poll._lastSeq = seq;
          const floats = new Float32Array(buf, 8, n * 7);
          const xyz = new Float32Array(n * 3);
          const rgb = new Float32Array(n * 3);
          const thermal = new Float32Array(n);
          for (let i = 0; i < n; i++) {
            xyz[i*3] = floats[i*7]; xyz[i*3+1] = floats[i*7+1]; xyz[i*3+2] = floats[i*7+2];
            rgb[i*3] = floats[i*7+3]; rgb[i*3+1] = floats[i*7+4]; rgb[i*3+2] = floats[i*7+5];
            thermal[i] = floats[i*7+6];
          }
          lastXYZ = xyz; lastRGB = rgb; lastThermal = thermal;
          geometry.setAttribute('position', new THREE.BufferAttribute(xyz, 3));
          applyColors();
          document.getElementById('stat-points').textContent = n.toLocaleString();
          document.getElementById('stat-age').textContent = new Date().toLocaleTimeString();
          recordUpdate();
          if (!haveFitOnce) { fitView(); haveFitOnce = true; }
        }
      }
    }
  } catch (e) { /* keep polling -- covers network errors AND our own abort-on-timeout */ }
  finally { clearTimeout(timeoutId); }
  const staleMs = performance.now() - (recentUpdateTimestamps[recentUpdateTimestamps.length-1] || 0);
  if (staleMs > 3000) document.getElementById('stat-hz').classList.add('stale');
  setTimeout(poll, source === 'merged' ? 250 : 60);
}
poll();

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth/window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
</script>
</body>
</html>
"""


class _ReusableThreadingTCPServer(socketserver.ThreadingTCPServer):
    # Without this, a quick restart (or a killed/crashed prior instance)
    # fails to rebind the port for ~a minute while it's in TIME_WAIT.
    allow_reuse_address = True


class _StreamState:
    """One (buffer, sequence number) slot per served point cloud source.

    The sequence number lets the client skip re-parsing a payload it
    already has (it polls faster than either source necessarily updates,
    especially the merged one at ~1Hz), without the server needing to
    track per-client state.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.buffer = struct.pack('<II', 0, 0)
        self.seq = 0

    def set_payload(self, payload_bytes):
        n_points = len(payload_bytes) // 28  # 7 float32 fields per point
        with self.lock:
            self.seq += 1
            self.buffer = struct.pack('<II', n_points, self.seq) + payload_bytes

    def get_buffer_and_seq(self):
        with self.lock:
            return self.buffer, self.seq


class _SharedState:
    def __init__(self):
        self.merged = _StreamState()
        self.live = _StreamState()


class _Handler(http.server.BaseHTTPRequestHandler):
    shared_state: _SharedState = None
    # path -> (body_bytes, content_type), loaded once at startup by
    # FusionWebStreamer.__init__ -- see this module's docstring for why
    # these are served locally instead of from a CDN.
    static_assets: dict = None
    # Deliberately HTTP/1.0 (closes the TCP connection after every
    # response), not 1.1. HTTP/1.1 keep-alive was tried for the
    # WiFi-friendliness of not re-handshaking every poll, but a persistent
    # connection through a NAT'd hotspot (NetworkManager's ipv4.method=
    # shared) is exactly the kind of link where a middlebox can silently
    # drop an idle connection's state without either side seeing a clean
    # FIN/RST -- the browser then reuses a connection that looks fine but
    # is actually dead, and the request hangs until the OS's own,
    # much-longer TCP timeout gives up. A fresh connection per poll can
    # never hit that: it either connects now or fails now. Combined with
    # the client's own fetch timeout (see poll() in PAGE_HTML), a bad
    # request costs one missed refresh instead of silently ending every
    # future one.
    protocol_version = 'HTTP/1.0'

    def log_message(self, format_, *args):
        pass

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
        """Conditional GET keyed on the stream's sequence number.

        Point clouds only actually change at ~1-10Hz while browsers poll
        faster than that (to catch an update quickly once it lands), so
        most polls would otherwise re-download an unchanged multi-hundred-
        KB payload for nothing -- fatal to reliability on a real WiFi link
        (this is the difference between the viewer being unusable and
        smooth from another room). Standard HTTP 304 handling: an
        unchanged sequence number gets a ~150-byte empty response instead
        of the full binary body; the client's browser cache (via
        fetch(..., {cache:'no-cache'}), which always revalidates) serves
        the last real body transparently either way.
        """
        body, seq = stream_state.get_buffer_and_seq()
        etag = f'"{seq}"'
        if self.headers.get('If-None-Match') == etag:
            self.send_response(304)
            self.send_header('ETag', etag)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-Type', 'application/octet-stream')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('ETag', etag)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass  # client (flaky WiFi, closed tab) disconnected mid-write

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._serve_bytes(PAGE_HTML.encode('utf-8'), 'text/html; charset=utf-8')
        elif self.path in ('/merged.bin', '/latest.bin'):
            self._serve_cloud(self.shared_state.merged)
        elif self.path == '/live.bin':
            self._serve_cloud(self.shared_state.live)
        elif self.static_assets and self.path in self.static_assets:
            body, content_type = self.static_assets[self.path]
            self._serve_bytes(body, content_type)
        else:
            self.send_response(404)
            self.end_headers()


def _encode_cloud(msg, max_points, rng):
    points = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z', 'rgb', 'thermal'])
    if points.size == 0:
        return b''
    n = points.shape[0]
    if n > max_points:
        idx = rng.choice(n, size=max_points, replace=False)
        points = points[idx]
        n = points.shape[0]

    xyz = points[:, 0:3].astype(np.float32)
    packed_rgb = points[:, 3].astype(np.float32).view(np.uint32)
    r = ((packed_rgb >> 16) & 0xFF).astype(np.float32) / 255.0
    g = ((packed_rgb >> 8) & 0xFF).astype(np.float32) / 255.0
    b = (packed_rgb & 0xFF).astype(np.float32) / 255.0
    thermal = points[:, 4].astype(np.float32)

    payload = np.empty((n, 7), dtype=np.float32)
    payload[:, 0:3] = xyz
    payload[:, 3] = r
    payload[:, 4] = g
    payload[:, 5] = b
    payload[:, 6] = thermal
    return payload.tobytes()


_WEB_ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web_assets')


def _load_static_assets():
    """Read three.min.js/OrbitControls.js off disk once at startup.

    See this module's docstring for why these are files here instead of
    CDN URLs in the page.
    """
    assets = {}
    files = {
        '/three.min.js': 'application/javascript',
        '/OrbitControls.js': 'application/javascript',
    }
    for path, content_type in files.items():
        file_path = os.path.join(_WEB_ASSETS_DIR, os.path.basename(path))
        with open(file_path, 'rb') as f:
            assets[path] = (f.read(), content_type)
    return assets


class FusionWebStreamer(Node):
    def __init__(self):
        super().__init__('fusion_web_streamer')

        self.declare_parameter('merged_topic', '/landfill/fusion/points_merged')
        self.declare_parameter('live_topic', '/landfill/fusion/points_rgbt')
        self.declare_parameter('http_port', 8080)
        # Sized for the standalone hotspot link, not just local loopback:
        # with the conditional-GET fix, bandwidth is only spent when a
        # stream actually updates, so these caps directly set the
        # per-update transfer size. This cap costs bandwidth + a cheap
        # numpy gather/pack here, NOT extra ICP compute in
        # registration_merge_node (that cost is independent, already
        # spent before this node ever sees the cloud) -- so it's safe to
        # raise for a denser, better-looking view without touching the
        # Pi's tighter CPU budget, which freed up considerably once every
        # topic moved to BEST_EFFORT QoS (measured ~103% of 400% available
        # at this cap, comfortable headroom). 35000 pts (~958KB) at
        # merged's ~1.3-1.8Hz -> ~1.2-1.7MB/s, still comfortable for a
        # direct hotspot link with one client. 10000 pts (~273KB) at
        # live's ~10Hz -> ~2.7MB/s -- live mode still wants a strong/local
        # link; merged is the recommended default.
        self.declare_parameter('max_points_merged', 35000)
        self.declare_parameter('max_points_live', 10000)
        # Backward-compat: an explicit input_topic overrides merged_topic,
        # matching the old single-stream parameter name.
        self.declare_parameter('input_topic', '')

        merged_topic = str(self.get_parameter('input_topic').value) or str(self.get_parameter('merged_topic').value)
        live_topic = str(self.get_parameter('live_topic').value)
        self._http_port = int(self.get_parameter('http_port').value)
        self._max_points_merged = int(self.get_parameter('max_points_merged').value)
        self._max_points_live = int(self.get_parameter('max_points_live').value)

        self._rng = np.random.default_rng(0)
        self._state = _SharedState()
        static_assets = _load_static_assets()
        handler = type(
            'BoundHandler', (_Handler,),
            {'shared_state': self._state, 'static_assets': static_assets},
        )
        self._server = _ReusableThreadingTCPServer(('0.0.0.0', self._http_port), handler)
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

        # BEST_EFFORT: matches registration_merge_node.py/sensor_fusion_node.py's
        # publisher QoS on these topics.
        self._merged_sub = self.create_subscription(
            PointCloud2, merged_topic, self._on_merged, qos_profile_sensor_data
        )
        self._live_sub = self.create_subscription(
            PointCloud2, live_topic, self._on_live, qos_profile_sensor_data
        )

        self.get_logger().info(
            'Serving merged=%s (max %d pts) + live=%s (max %d pts) at http://0.0.0.0:%d/'
            % (merged_topic, self._max_points_merged, live_topic, self._max_points_live, self._http_port)
        )

    def _on_merged(self, msg):
        self._state.merged.set_payload(_encode_cloud(msg, self._max_points_merged, self._rng))

    def _on_live(self, msg):
        self._state.live.set_payload(_encode_cloud(msg, self._max_points_live, self._rng))

    def destroy_node(self):
        self._server.shutdown()
        super().destroy_node()


def main():
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
