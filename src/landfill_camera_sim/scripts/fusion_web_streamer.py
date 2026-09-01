#!/usr/bin/env python3
"""Serve the fused RGB+thermal point cloud to any browser on the network.

Subscribes to a PointCloud2 with x,y,z,rgb,thermal fields (the output of
sensor_fusion_node.py / registration_merge_node.py), keeps the latest
scan as a compact binary buffer, and serves it plus a self-contained
three.js viewer page over plain HTTP -- no ROS install needed on the
viewing device, just a browser pointed at http://<this-machine>:<port>/.

This node only binds a local port; reaching it from another device on
the same WiFi network still depends on the network path from that
device to this machine (see the package README for the WSL2 note).
"""

import http.server
import socketserver
import struct
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
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
    border:1px solid #26313f; border-radius:10px; padding:10px 14px; backdrop-filter: blur(6px); }
  #hud h1 { font-size:14px; margin:0 0 6px; font-weight:600; letter-spacing:.02em; color:#9fd3ff; }
  #hud div { font-size:12px; color:#9aa7b4; margin-top:2px; }
  #hud button { margin-top:8px; background:#1b2734; color:#e6edf3; border:1px solid #33445a;
    border-radius:6px; padding:6px 10px; font-size:12px; cursor:pointer; }
  #hud button:hover { background:#243447; }
  #hud button.active { background:#2563eb; border-color:#2563eb; }
  canvas { display:block; }
</style>
</head>
<body>
<div id="hud">
  <h1>PitFusion3D &middot; live fused cloud</h1>
  <div id="stat-points">points: --</div>
  <div id="stat-age">last update: --</div>
  <button id="mode-rgb" class="active">Color: RGB</button>
  <button id="mode-thermal">Color: Thermal</button>
</div>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script>
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
const camera = new THREE.PerspectiveCamera(60, window.innerWidth/window.innerHeight, 0.05, 500);
camera.position.set(6, -10, 6);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.target.set(0,0,0);

const grid = new THREE.GridHelper(40, 40, 0x2a3a4d, 0x1a2531);
grid.rotation.x = Math.PI/2;
scene.add(grid);
scene.add(new THREE.AxesHelper(1.5));

let geometry = new THREE.BufferGeometry();
let material = new THREE.PointsMaterial({size:0.045, vertexColors:true, sizeAttenuation:true});
let pointsObj = new THREE.Points(geometry, material);
scene.add(pointsObj);

let mode = 'rgb';
let lastXYZ = null, lastRGB = null, lastThermal = null;

document.getElementById('mode-rgb').onclick = () => setMode('rgb');
document.getElementById('mode-thermal').onclick = () => setMode('thermal');
function setMode(m) {
  mode = m;
  document.getElementById('mode-rgb').classList.toggle('active', m === 'rgb');
  document.getElementById('mode-thermal').classList.toggle('active', m === 'thermal');
  applyColors();
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

async function poll() {
  try {
    const res = await fetch('/latest.bin', {cache:'no-store'});
    if (res.ok) {
      const buf = await res.arrayBuffer();
      if (buf.byteLength >= 4) {
        const dv = new DataView(buf);
        const n = dv.getUint32(0, true);
        const floats = new Float32Array(buf, 4, n * 7);
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
        geometry.computeBoundingSphere();
        document.getElementById('stat-points').textContent = 'points: ' + n.toLocaleString();
        document.getElementById('stat-age').textContent = 'last update: ' + new Date().toLocaleTimeString();
      }
    }
  } catch (e) { /* keep polling */ }
  setTimeout(poll, 400);
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


class _SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.buffer = struct.pack('<I', 0)


class _Handler(http.server.BaseHTTPRequestHandler):
    shared_state: _SharedState = None

    def log_message(self, format_, *args):
        pass

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            body = PAGE_HTML.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/latest.bin':
            with self.shared_state.lock:
                body = self.shared_state.buffer
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()


class FusionWebStreamer(Node):
    def __init__(self):
        super().__init__('fusion_web_streamer')

        self.declare_parameter('input_topic', '/landfill/fusion/points_merged')
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('max_points', 20000)

        input_topic = str(self.get_parameter('input_topic').value)
        self._http_port = int(self.get_parameter('http_port').value)
        self._max_points = int(self.get_parameter('max_points').value)

        self._state = _SharedState()
        handler = type('BoundHandler', (_Handler,), {'shared_state': self._state})
        self._server = socketserver.ThreadingTCPServer(('0.0.0.0', self._http_port), handler)
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

        self._subscription = self.create_subscription(PointCloud2, input_topic, self._on_cloud, 5)

        self.get_logger().info(
            'Serving %s at http://0.0.0.0:%d/ (max %d points/frame)'
            % (input_topic, self._http_port, self._max_points)
        )

    def _on_cloud(self, msg):
        points = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z', 'rgb', 'thermal'])
        if points.size == 0:
            return
        n = points.shape[0]
        if n > self._max_points:
            stride = int(np.ceil(n / self._max_points))
            points = points[::stride]
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

        buffer = struct.pack('<I', n) + payload.tobytes()
        with self._state.lock:
            self._state.buffer = buffer

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
