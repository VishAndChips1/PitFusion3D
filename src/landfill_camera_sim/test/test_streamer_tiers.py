"""Tests for the streamer's per-client point-count levels, shared encode cache and HTTP behaviour."""
import gzip
import http.client
import os
import struct
import sys
import threading

import numpy as np

sys.path.insert(0, os.environ.get('STREAMER_DIR', os.path.join(os.path.dirname(__file__), '..', 'scripts')))

import fusion_web_streamer as streamer  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402


def make_msg(n, sec=100):
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(0, 5, n), rng.uniform(-2, 2, n), rng.uniform(0, 2, n), np.zeros(n), np.full(n, np.nan)]).astype(np.float32)
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
    return state


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
        conn.request('GET', '/nothing')
        r = conn.getresponse()
        r.read()
        assert r.status == 404
    finally:
        server.shutdown()
