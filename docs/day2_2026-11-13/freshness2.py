#!/usr/bin/env python3
"""Data age as received by a viewer, from the streamer's own headers (X-Cloud-Stamp, X-Server-Time). usage: freshness2.py <seconds>"""
import sys, time, json, threading, http.client, numpy as np
secs = float(sys.argv[1]); H = {'live': [], 'merged': []}; stop = time.time() + secs
def poll(path, key, period):
    c = None; etag = None
    while time.time() < stop:
        t0 = time.time()
        try:
            if c is None: c = http.client.HTTPConnection('localhost', 8080, timeout=5)
            c.request('GET', path, headers={'If-None-Match': etag} if etag else {}); r = c.getresponse(); body = r.read(); t1 = time.time()
            if r.status == 200 and r.getheader('X-Cloud-Stamp'): etag = r.getheader('ETag'); H[key].append((t0, t1, float(r.getheader('X-Cloud-Stamp')), float(r.getheader('X-Server-Time')), len(body)))
        except Exception: c = None
        time.sleep(max(0, period - (time.time() - t0)))
ts = [threading.Thread(target=poll, args=a) for a in (('/live.bin', 'live', 0.12), ('/merged.bin', 'merged', 0.25))]; [t.start() for t in ts]; [t.join() for t in ts]
out = {}
for k, v in H.items():
    a = np.array(v); pc = lambda x, p: float(np.percentile(x, p))
    sv = a[:, 3] - a[:, 2]; cl = a[:, 1] - a[:, 2]; rt = a[:, 1] - a[:, 0]
    out[k] = dict(new_payloads=len(a), age_when_sent_s=dict(p50=pc(sv, 50), p95=pc(sv, 95), max=float(sv.max())), age_at_client_s=dict(p50=pc(cl, 50), p95=pc(cl, 95), max=float(cl.max())), request_time_s=dict(p50=pc(rt, 50), p95=pc(rt, 95)), payload_kb=float(a[:, 4].mean() / 1e3))
json.dump(out, open('data/freshness_headers.json', 'w'), indent=1); print(json.dumps(out))
