#!/usr/bin/env python3
"""Thermal-camera-only viewer. Subscribes to /landfill/thermal/image_raw (published by
thermal_camera_node, which owns the USB device) and serves a colourised live view:
  http://<pi>:8081/            page
  http://<pi>:8081/stream.mjpg MJPEG stream
  http://<pi>:8081/frame.jpg   single frame
Shows NO SIGNAL when no frame arrived in the last 3 s. Encodes only while a viewer is connected."""
import http.server, socketserver, threading, time
import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PORT = 8081
lock = threading.Lock()
state = {'img': None, 't': 0.0, 'n': 0}

def no_signal():
    im = np.zeros((512, 640, 3), np.uint8)
    cv2.putText(im, 'NO SIGNAL', (200, 250), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 255), 3)
    cv2.putText(im, 'thermal camera is not delivering frames', (95, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
    return im

def jpeg():
    with lock:
        img, t = state['img'], state['t']
    if img is None or time.monotonic() - t > 3.0:
        im = no_signal()
    else:
        y = (img >> 8).astype(np.uint8)
        im = cv2.applyColorMap(y, cv2.COLORMAP_INFERNO)
    return cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()

PAGE = b"""<!doctype html><meta name=viewport content="width=device-width,initial-scale=1"><title>Thermal camera</title>
<body style="margin:0;background:#111;color:#ddd;font:14px sans-serif;text-align:center">
<p>Thermal camera (relative signal, not absolute temperature)</p>
<img src="/stream.mjpg" style="max-width:100%;image-rendering:pixelated"></body>"""

class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        try:
            if self.path.startswith('/stream.mjpg'):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=f')
                self.send_header('Cache-Control', 'no-store'); self.end_headers()
                while True:
                    b = jpeg()
                    self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(b) + b + b'\r\n')
                    time.sleep(0.1)
            elif self.path.startswith('/frame.jpg'):
                b = jpeg(); self.send_response(200); self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
            elif self.path == '/':
                self.send_response(200); self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', str(len(PAGE))); self.end_headers(); self.wfile.write(PAGE)
            else:
                self.send_response(404); self.send_header('Content-Length', '0'); self.end_headers()
        except (BrokenPipeError, ConnectionError, TimeoutError):
            pass

class S(socketserver.ThreadingTCPServer):
    allow_reuse_address = True; daemon_threads = True

def on_img(m):
    if m.encoding != 'mono16': return
    a = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
    with lock:
        state['img'], state['t'] = a, time.monotonic()

def main():
    rclpy.init()
    n = Node('thermal_view')
    n.create_subscription(Image, '/landfill/thermal/image_raw', on_img, qos_profile_sensor_data)
    threading.Thread(target=S(('0.0.0.0', PORT), H).serve_forever, daemon=True).start()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass

if __name__ == '__main__':
    main()
