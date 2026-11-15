"""Tests for the streamer's per-client point-count levels, shared encode cache and HTTP behaviour."""
import gzip
import http.client
import json
import os
import struct
import sys
import threading

import numpy as np

sys.path.insert(0, os.environ.get('STREAMER_DIR', os.path.join(os.path.dirname(__file__), '..', 'scripts')))

import fusion_web_streamer as streamer  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402


def make_msg(n, sec=100, thermal=False, seed=0):
    """Build a structured cloud (a floor and a wall with noise and varied colours), like a real scene, not random dust."""
    rng = np.random.default_rng(seed)
    half = n // 2
    floor = np.column_stack([rng.uniform(0, 8, half), rng.uniform(-3, 3, half), rng.normal(0, 0.01, half)])
    wall = np.column_stack([rng.normal(8, 0.01, n - half), rng.uniform(-3, 3, n - half), rng.uniform(0, 3, n - half)])
    xyz = np.vstack([floor, wall])
    rgb = (rng.integers(0, 256, size=(n, 3)) // 8 * 8 + (xyz[:, :1] * 10 % 8)).astype(np.uint32) & 0xFF
    packed = ((rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]).astype(np.uint32).view(np.float32)
    temp = rng.uniform(280, 310, n) if thermal else np.full(n, np.nan)
    pts = np.column_stack([xyz, packed, temp]).astype(np.float32)
    msg = PointCloud2()
    msg.height, msg.width = 1, n
    msg.fields = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, k in enumerate(['x', 'y', 'z', 'rgb', 'thermal'])]
    msg.point_step, msg.row_step, msg.data = 20, 20 * n, pts.tobytes()
    msg.header.stamp.sec = sec
    return msg


def make_state(ceiling=80000):
    state = streamer._StreamState()
    rng = np.random.default_rng(1)
    state.max_points = ceiling
    state.encoder = lambda m, n: streamer._encode_cloud(m, n, rng)
    state.encoder_v2 = lambda m, n: streamer._encode_cloud_v2(m, n, rng)
    return state


def decode_v2(body):
    """Decode format 2 the way the viewer and the base station do (reference implementation)."""
    raw = gzip.decompress(body)
    n, seq = struct.unpack_from('<II', raw, 0)
    ox, oy, oz, quantum, flags = struct.unpack_from('<ffffI', raw, 8)
    off = 28
    sections = []
    for _ in range(3):
        sections.append(np.frombuffer(raw, '<i2', n, off).astype(np.int64))
        off += 2 * n
    thermal = None
    if flags & 1:
        thermal = np.frombuffer(raw, '<i2', n, off)
        off += 2 * n
    colour = []
    for _ in range(3):
        colour.append(np.frombuffer(raw, np.uint8, n, off).astype(np.int64))
        off += n
    assert off == len(raw)                                    # no stray bytes
    xyz = np.stack([np.cumsum(c) for c in sections], axis=1) * quantum + np.array([ox, oy, oz])
    rgb = np.stack([np.cumsum(c) & 0xFF for c in colour], axis=1)
    return n, xyz, rgb, thermal, quantum


def header(body):
    raw = gzip.decompress(body)
    return struct.unpack_from('<II', raw, 0)


def test_levels_end_at_the_server_ceiling():
    state = make_state(80000)
    assert state.levels() == [8000, 20000, 40000, 60000, 80000]
    state.max_points = 50000
    assert state.levels() == [8000, 20000, 40000, 50000]
    state.max_points = 5000
    assert state.levels() == [5000]


def test_client_request_picks_the_largest_level_not_above_it():
    state = make_state(80000)
    state.offer(make_msg(100000))
    assert header(state.get(None)[0])[0] == 80000          # no request: the ceiling
    assert header(state.get(45000)[0])[0] == 40000
    assert header(state.get(1000)[0])[0] == 8000            # below the lowest level: the lowest
    assert header(state.get(500000)[0])[0] == 80000         # above the ceiling: the ceiling


def test_small_cloud_is_served_whole():
    state = make_state(80000)
    state.offer(make_msg(3000))
    assert header(state.get(None)[0])[0] == 3000


def test_each_level_is_encoded_once_per_cloud_and_seq_differs_by_level():
    state = make_state(80000)
    state.offer(make_msg(100000))
    a, b = state.get(20000), state.get(40000)
    assert state.get(20000)[0] is a[0]                      # cached object, not re-encoded
    assert a[1] != b[1]                                     # a level change is never mistaken for old data
    state.offer(make_msg(100000, sec=101))
    c = state.get(20000)
    assert c[1] != a[1] and c[2] == 101.0                   # a new cloud gets a new seq and its own stamp


def test_changing_the_ceiling_never_reuses_a_sequence_number():
    state = make_state(80000)
    state.offer(make_msg(100000))
    before = state.get(None)
    state.max_points = 50000                                # runtime parameter change
    after = state.get(None)
    assert after[1] != before[1] and header(after[0])[0] == 50000


def test_format2_round_trips_positions_colours_and_thermal():
    from scipy.spatial import cKDTree
    for thermal in (False, True):
        msg = make_msg(30000, thermal=thermal)
        state = make_state(80000)
        state.offer(msg)
        n, xyz, rgb, t, quantum = decode_v2(state.get(None, fmt=2)[0])
        assert n == 30000 and (t is not None) == thermal
        orig = np.frombuffer(msg.data, np.float32).reshape(-1, 5)
        dist, idx = cKDTree(orig[:, :3].astype(np.float64)).query(xyz)
        assert dist.max() <= quantum * 0.87 + 1e-4              # every decoded point is within half a grid cell of a real one
        packed = orig[:, 3].view(np.uint32)
        expect = np.stack([(packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF], axis=1)
        # Nearest-neighbour matching is ambiguous for points closer together than the grid, so compare the full set of
        # colours (and thermal values): every original colour must come back exactly, once.
        assert sorted(map(tuple, rgb.tolist())) == sorted(map(tuple, expect.tolist()))
        if thermal:
            assert np.allclose(np.sort(t / 100.0), np.sort(orig[:, 4].astype(np.float64)), atol=0.0051)   # stored in 0.01 steps


def test_format2_is_much_smaller_than_format1_on_a_scene_like_cloud():
    state = make_state(80000)
    state.offer(make_msg(60000))
    v1, v2 = len(state.get(None, fmt=1)[0]), len(state.get(None, fmt=2)[0])
    assert v2 < 0.8 * v1


def test_format2_sequence_numbers_never_collide_with_format1():
    state = make_state(80000)
    state.offer(make_msg(10000))
    assert state.get(None, fmt=1)[1] != state.get(None, fmt=2)[1]
    assert state.get(None, fmt=2)[0] is state.get(None, fmt=2)[0]


def test_format2_handles_empty_and_tiny_clouds():
    state = make_state(80000)
    state.offer(make_msg(1))
    n, xyz, rgb, t, q = decode_v2(state.get(None, fmt=2)[0])
    assert n == 1 and xyz.shape == (1, 3)


def test_no_cloud_yet_serves_an_empty_payload_and_records_the_request():
    state = make_state()
    assert header(state.get(None)[0])[0] == 0
    assert state.last_request > 0


def test_http_levels_etag_and_304():
    state = streamer._SharedState()
    state.merged = make_state(60000)
    state.merged.offer(make_msg(100000))
    handler = type('H', (streamer._Handler,), {'shared_state': state, 'static_assets': {}, 'page_html': b'<html></html>'})
    server = streamer._ReusableThreadingTCPServer(('127.0.0.1', 0), handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        conn = http.client.HTTPConnection('127.0.0.1', server.server_address[1], timeout=10)
        conn.request('GET', '/merged.bin?n=20000')
        r = conn.getresponse()
        body, etag = r.read(), r.getheader('ETag')
        assert r.status == 200 and header(body)[0] == 20000 and float(r.getheader('X-Cloud-Stamp')) == 100.0
        assert r.getheader('X-Quality-Levels') == '8000,20000,40000,60000'
        conn.request('GET', '/merged.bin?n=20000', headers={'If-None-Match': etag})
        r = conn.getresponse()
        r.read()
        assert r.status == 304
        conn.request('GET', '/merged.bin?n=40000', headers={'If-None-Match': etag})   # other level: must not 304
        r = conn.getresponse()
        other = r.read()
        assert r.status == 200 and header(other)[0] == 40000
        conn.request('GET', '/merged.bin')
        r = conn.getresponse()
        assert header(r.read())[0] == 60000                 # no ?n=: the ceiling
        conn.request('GET', '/merged.bin?n=20000&fmt=2')
        r = conn.getresponse()
        body2 = r.read()
        assert r.status == 200 and r.getheader('X-Formats') == '1,2'
        assert decode_v2(body2)[0] == 20000
        conn.request('GET', '/stats')
        r = conn.getresponse()
        stats = json.loads(r.read())
        assert r.status == 200 and stats['sent_200'] >= 4 and stats['not_modified_304'] == 1 and stats['format2'] == 1
        assert stats['aborted_by_client'] == 0 and stats['bytes_sent'] > 0
        conn.request('GET', '/nothing')
        r = conn.getresponse()
        r.read()
        assert r.status == 404
    finally:
        server.shutdown()


def make_rgb_msg(blur=0):
    """Build a 1280x720 rgb8 image of fine detail (a checker pattern with noise), optionally blurred."""
    import cv2
    from sensor_msgs.msg import Image
    rng = np.random.default_rng(3)
    img = (((np.indices((720, 1280)).sum(axis=0) // 6) % 2) * 200 + rng.integers(0, 40, (720, 1280))).astype(np.uint8)
    img = np.stack([img] * 3, axis=2)
    if blur:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    msg = Image()
    msg.height, msg.width, msg.encoding, msg.step = 720, 1280, 'rgb8', 1280 * 3
    msg.data = img.tobytes()
    return msg


def test_focus_sharpness_ranks_a_sharp_image_above_a_blurred_one_and_the_jpeg_decodes():
    import cv2
    focus = streamer._FocusState()
    sharp_img = np.frombuffer(make_rgb_msg().data, np.uint8).reshape(720, 1280, 3)
    blur_img = np.frombuffer(make_rgb_msg(blur=3).data, np.uint8).reshape(720, 1280, 3)
    jpeg, sharp = focus.render(sharp_img)
    _, blurred = focus.render(blur_img)
    assert sharp > 5 * blurred
    decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (540, 960, 3)


def test_focus_frames_are_rate_limited_and_waiters_are_woken():
    focus = streamer._FocusState()
    focus.offer(make_rgb_msg())
    first = focus.wait_for_new(0, timeout=1.0)
    assert first is not None and first[1] == 1
    focus.offer(make_rgb_msg())                                   # immediately again: over the frame-rate cap, ignored
    assert focus.wait_for_new(1, timeout=0.05) is None


def test_focus_page_and_stream_over_http():
    state = streamer._SharedState()
    state.focus.offer(make_rgb_msg())
    handler = type('H', (streamer._Handler,), {'shared_state': state, 'static_assets': {}, 'page_html': b''})
    server = streamer._ReusableThreadingTCPServer(('127.0.0.1', 0), handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        conn = http.client.HTTPConnection('127.0.0.1', server.server_address[1], timeout=10)
        conn.request('GET', '/focus')
        r = conn.getresponse()
        assert r.status == 200 and b'/focus.mjpg' in r.read()
        conn2 = http.client.HTTPConnection('127.0.0.1', server.server_address[1], timeout=10)
        conn2.request('GET', '/focus.mjpg')
        r2 = conn2.getresponse()
        assert r2.status == 200 and 'multipart/x-mixed-replace' in r2.getheader('Content-Type')
        part = r2.fp.read(400)
        assert part.startswith(b'--frame') and b'image/jpeg' in part
        conn2.close()
    finally:
        server.shutdown()
