#!/usr/bin/env python3
"""End-to-end data freshness as a viewer experiences it. A ROS subscriber logs (arrival time, header stamp) of every fused and merged cloud while an HTTP
client polls /live.bin (every 120 ms) and /merged.bin (every 250 ms) like the browser page. For every NEW payload the client receives at time t, the age of the
data = t - header_stamp of the newest ROS message that existed at that moment. For the merged view the *content* is older still by half its window span.
Output: data/freshness.json   usage: freshness.py <seconds>"""
import sys, time, json, threading, http.client, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2
secs = float(sys.argv[1]); LOG = {'fused': [], 'merged': []}
rclpy.init(); n = Node('freshness')
for key, tp in (('fused', '/landfill/fusion/points_rgbt'), ('merged', '/landfill/fusion/points_merged')):
    n.create_subscription(PointCloud2, tp, lambda m, k=key: LOG[k].append((time.time(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)), Q)
HT = {'live': [], 'merged': []}; stop = time.time() + secs + 5
def poll(path, key, period):
    c = None; etag = None
    while time.time() < stop:
        t0 = time.time()
        try:
            if c is None: c = http.client.HTTPConnection('localhost', 8080, timeout=5)
            c.request('GET', path, headers={'If-None-Match': etag} if etag else {}); r = c.getresponse(); r.read(); t1 = time.time()
            if r.status == 200: etag = r.getheader('ETag'); HT[key].append((t0, t1))
        except Exception: c = None
        time.sleep(max(0, period - (time.time() - t0)))
for a in (('/live.bin', 'live', 0.12), ('/merged.bin', 'merged', 0.25)): threading.Thread(target=poll, args=a, daemon=True).start()
t0 = time.time()
while time.time() - t0 < secs + 3: rclpy.spin_once(n, timeout_sec=0.02)
out = {}
for hk, rk in (('live', 'fused'), ('merged', 'merged')):
    arr = np.array([x[0] for x in LOG[rk]]); st = np.array([x[1] for x in LOG[rk]]); ages = []; waits = []
    for (ts, te) in HT[hk]:
        idx = np.searchsorted(arr, ts, side='right') - 1                       # newest ROS message that existed when the request was served
        if idx >= 0: ages.append(te - st[idx]); waits.append(te - arr[idx])
    ages = np.array(ages); waits = np.array(waits); pc = lambda a, p: float(np.percentile(a, p)) if len(a) else None
    out[hk] = dict(new_payloads=len(HT[hk]), age_of_newest_data_s=dict(p50=pc(ages, 50), p95=pc(ages, 95), max=float(ages.max()) if len(ages) else None), delay_since_ros_arrival_s=dict(p50=pc(waits, 50), p95=pc(waits, 95)),
                   response_time_s=dict(p50=pc(np.array([b - a for a, b in HT[hk]]), 50), p95=pc(np.array([b - a for a, b in HT[hk]]), 95)))
json.dump(out, open('data/freshness.json', 'w'), indent=1); print(json.dumps(out))
