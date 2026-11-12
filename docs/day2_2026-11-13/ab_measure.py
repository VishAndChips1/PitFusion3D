#!/usr/bin/env python3
"""A/B measurement of the live pipeline for N seconds: topic rates/ages + CPU of each node. usage: ab_measure.py <label> <seconds>"""
import sys, json, subprocess, time, psutil, numpy as np
label, secs = sys.argv[1], float(sys.argv[2])
PAT = {'merge': 'registration_merge_node', 'fusion': 'sensor_fusion_node', 'restamp': 'livox_restamp_node', 'streamer': 'fusion_web_streamer', 'rgb': 'rgb_camera_node', 'driver': 'livox_ros2_avia_node'}
procs = {}
for p in psutil.process_iter(['pid', 'cmdline']):
    cl = ' '.join(p.info['cmdline'] or [])
    for k, pat in PAT.items():
        if pat in cl and 'ab_measure' not in cl and k not in procs and not cl.startswith('bash'): procs[k] = p
for p in procs.values(): p.cpu_percent(None)
pr = subprocess.Popen(['python3', '/home/vish/thesis_tests/final_campaign_20261112/probe_topics.py', '--topics', '/landfill/fusion/points_rgbt', '/landfill/fusion/points_merged', '--secs', str(secs)], stdout=subprocess.PIPE, text=True)
samples = {k: [] for k in procs}; t0 = time.time()
while time.time() - t0 < secs - 2:
    time.sleep(2)
    for k, p in procs.items(): samples[k].append(p.cpu_percent(None))
r = json.loads([l for l in pr.communicate()[0].splitlines() if l.startswith('{')][-1])
out = dict(label=label, cpu={k: round(float(np.mean(v)), 1) for k, v in samples.items()})
for tp, nm in (('/landfill/fusion/points_rgbt', 'fused'), ('/landfill/fusion/points_merged', 'merged')):
    v = r[tp]; out[nm] = dict(hz=round(v['hz'], 2), interval_p95_ms=round(v['interval_ms']['p95']), age_p50_ms=round(v['stamp_age_ms']['p50']), age_p95_ms=round(v['stamp_age_ms']['p95']), pts=round(v['points_or_pixels_mean']))
print(json.dumps(out))
