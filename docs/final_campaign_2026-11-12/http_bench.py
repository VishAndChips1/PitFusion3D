#!/usr/bin/env python3
"""Viewer load test: N concurrent pollers emulating the browser (merged every 250 ms, live every 120 ms, If-None-Match ETag). JSON on last line.
Usage: http_bench.py <host:port> <n_clients> <seconds>"""
import sys, time, json, threading, http.client, gzip, struct, numpy as np
host, nc, secs = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]); h, p = host.split(':')
res = {'/merged.bin': dict(lat=[], bytes=[], n200=0, n304=0, err=0, n_pts=[], raw=[]), '/live.bin': dict(lat=[], bytes=[], n200=0, n304=0, err=0, n_pts=[], raw=[])}
lock = threading.Lock(); stop = time.time() + secs
def client(i):
    path = '/merged.bin' if i % 2 == 0 else '/live.bin'; period = 0.25 if i % 2 == 0 else 0.12; etag = None; c = None; r = res[path]
    while time.time() < stop:
        t0 = time.time()
        try:
            if c is None: c = http.client.HTTPConnection(h, int(p), timeout=5)
            hd = {'Accept-Encoding': 'identity'}
            if etag: hd['If-None-Match'] = etag
            c.request('GET', path, headers=hd); resp = c.getresponse(); body = resp.read(); dt = time.time() - t0
            with lock:
                r['lat'].append(dt * 1000)
                if resp.status == 200:
                    r['n200'] += 1; r['bytes'].append(len(body)); etag = resp.getheader('ETag')
                    try: raw = gzip.decompress(body) if body[:2] == b'\x1f\x8b' else body; r['n_pts'].append(struct.unpack_from('<I', raw, 0)[0]); r['raw'].append(len(raw))
                    except Exception: pass
                elif resp.status == 304: r['n304'] += 1
                else: r['err'] += 1
        except Exception: 
            with lock: r['err'] += 1
            try: c.close()
            except Exception: pass
            c = None
        time.sleep(max(0, period - (time.time() - t0)))
ts = [threading.Thread(target=client, args=(i,), daemon=True) for i in range(nc)]; t0 = time.time()
[t.start() for t in ts]; [t.join() for t in ts]; el = time.time() - t0; out = {}
for path, r in res.items():
    if not (r['lat'] or r['err']): continue
    L = np.array(r['lat']) if r['lat'] else np.array([0.0]); B = np.array(r['bytes']) if r['bytes'] else np.array([0.0])
    out[path] = dict(requests=len(r['lat']) + r['err'], req_per_s=len(r['lat']) / el, n200=r['n200'], n304=r['n304'], errors=r['err'], latency_ms=dict(p50=float(np.percentile(L, 50)), p95=float(np.percentile(L, 95)), p99=float(np.percentile(L, 99)), max=float(L.max())),
                     payload_kb_mean=float(B.mean() / 1e3), wire_mbit_s_total=float(B.sum() * 8 / el / 1e6), updates_per_s_per_client=r['n200'] / el / max(1, (nc + 1) // 2 if path == '/merged.bin' else nc // 2), points_mean=float(np.mean(r['n_pts'])) if r['n_pts'] else None, uncompressed_kb_mean=float(np.mean(r['raw']) / 1e3) if r['raw'] else None)
print(json.dumps(dict(clients=nc, secs=el, paths=out)))
