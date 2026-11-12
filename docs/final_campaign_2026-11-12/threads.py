#!/usr/bin/env python3
"""Per-thread CPU accounting for pipeline processes over N seconds (from /proc/<pid>/task/*/stat). JSON on last line."""
import os, sys, time, json, glob, psutil
secs = float(sys.argv[1]); TCK = os.sysconf('SC_CLK_TCK')
PAT = {'livox_driver': 'livox_ros2_avia_node', 'restamp': 'livox_restamp_node', 'rgb_camera': 'rgb_camera_node', 'thermal_camera': 'thermal_camera_node', 'calibration': 'calibration_publisher.py',
       'fusion': 'sensor_fusion_node.py', 'merge': 'registration_merge_node.py', 'streamer': 'fusion_web_streamer.py', 'rpicam_vid': 'rpicam-vid'}
procs = {}
for p in psutil.process_iter(['pid', 'cmdline']):
    cl = ' '.join(p.info['cmdline'] or [])
    for k, pat in PAT.items():
        if pat in cl and k not in procs and 'threads.py' not in cl and not cl.startswith(('bash', '/bin/bash')): procs[k] = p.info['pid']
def snap():
    d = {}
    for k, pid in procs.items():
        for t in glob.glob(f'/proc/{pid}/task/*'):
            try:
                st = open(t + '/stat').read(); comm = open(t + '/comm').read().strip(); f = st[st.rindex(')') + 2:].split(); tid = os.path.basename(t)
                sched = open(t + '/status').read(); vol = int([l for l in sched.splitlines() if l.startswith('voluntary_ctxt_switches')][0].split()[1]); inv = int([l for l in sched.splitlines() if l.startswith('nonvoluntary_ctxt_switches')][0].split()[1])
                d[(k, tid)] = dict(comm=comm, utime=int(f[11]), stime=int(f[12]), core=int(f[36]), vol=vol, inv=inv)
            except (FileNotFoundError, ProcessLookupError, ValueError, IndexError): pass
    return d
a = snap(); t0 = time.time(); time.sleep(secs); b = snap(); el = time.time() - t0; out = {}
for key, v in b.items():
    if key not in a: continue
    k, tid = key; u = (v['utime'] - a[key]['utime']) / TCK / el * 100; s = (v['stime'] - a[key]['stime']) / TCK / el * 100
    out.setdefault(k, []).append(dict(tid=tid, name=v['comm'], user_pct=round(u, 2), sys_pct=round(s, 2), total_pct=round(u + s, 2), last_core=v['core'], vol_ctx_per_s=round((v['vol'] - a[key]['vol']) / el, 1), invol_ctx_per_s=round((v['inv'] - a[key]['inv']) / el, 1)))
summary = {}
for k, th in out.items():
    th.sort(key=lambda x: -x['total_pct']); summary[k] = dict(pid=procs[k], n_threads=len(th), total_cpu_pct=round(sum(x['total_pct'] for x in th), 1), busiest_thread=th[0], threads=th)
print(json.dumps(dict(secs=el, processes=summary)))
