#!/usr/bin/env python3
"""Pipeline performance snapshot: per-process CPU, per-core load, which core
each process runs on, context switches, thermal/clock, plus topic rates and
latency (via the fresh-process rate tracker).  Usage: pipeline_perf.py <label> <seconds>"""
import json, os, subprocess, sys, time, collections
import psutil

LABEL, DUR = sys.argv[1], int(sys.argv[2])
OUT = os.path.dirname(os.path.abspath(__file__))
TRACKER = '/home/vish/thesis_tests/20261111_v3_campaign/rate_tracker_subprocess.py'
PATTERNS = {
 'livox_driver': 'livox_ros2_avia_node', 'livox_restamp': 'livox_restamp_node',
 'calib_pub': 'calibration_publisher', 'rgb_node': 'rgb_camera_node',
 'rpicam_vid': 'rpicam-vid', 'thermal_node': 'thermal_camera_node',
 'fusion': 'sensor_fusion_node', 'merge': 'registration_merge_node',
 'streamer': 'fusion_web_streamer'}

def find():
    found = {}
    for p in psutil.process_iter(['pid', 'cmdline']):
        cl = ' '.join(p.info['cmdline'] or [])
        if 'launch_params' not in cl and 'rpicam-vid' not in cl:
            continue
        for k, pat in PATTERNS.items():
            if pat in cl and k not in found:
                found[k] = psutil.Process(p.info['pid'])
    return found

procs = find()
env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
csv = f'{OUT}/{LABEL}_rates.csv'
if os.path.exists(csv): os.remove(csv)
tr = subprocess.Popen([sys.executable, TRACKER, str(DUR), csv, LABEL, '{}'], env=env,
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
for p in procs.values(): p.cpu_percent(None)
psutil.cpu_percent(percpu=True)
cores_seen = collections.defaultdict(collections.Counter)
cpu_samples = collections.defaultdict(list)
core_util = []
temps, clocks = [], []
ctx0 = {k: p.num_ctx_switches() for k, p in procs.items()}
thr0 = {k: {t.id: t.user_time + t.system_time for t in p.threads()} for k, p in procs.items()}
t_end = time.time() + DUR
while time.time() < t_end:
    time.sleep(1.0)
    core_util.append(psutil.cpu_percent(percpu=True))
    for k, p in procs.items():
        try:
            cpu_samples[k].append(p.cpu_percent(None))
            cores_seen[k][p.cpu_num()] += 1
        except psutil.Error:
            pass
    try:
        temps.append(float(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000)
        clocks.append(int(open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq').read()) / 1e6)
    except OSError: pass
tr.wait(timeout=30)
res = {'label': LABEL, 'duration_s': DUR, 'processes': {}, 'cores_pct_mean': [],
       'soc_temp_c': (min(temps), sum(temps)/len(temps), max(temps)) if temps else None,
       'cpu_mhz_mean': sum(clocks)/len(clocks) if clocks else None,
       'load_desktop_other_cpu_pct': None}
n = len(core_util[0])
res['cores_pct_mean'] = [round(sum(s[i] for s in core_util)/len(core_util), 1) for i in range(n)]
pipe_total = 0
for k, p in procs.items():
    s = cpu_samples[k]
    if not s: continue
    mean = sum(s)/len(s); pipe_total += mean
    try:
        c1 = p.num_ctx_switches(); thr1 = {t.id: t.user_time + t.system_time for t in p.threads()}
    except psutil.Error: continue
    thr_delta = sorted(((thr1[t]-thr0[k].get(t, 0))/DUR*100, t) for t in thr1)[::-1][:3]
    res['processes'][k] = {
        'pid': p.pid, 'cpu_pct_mean': round(mean, 1), 'cpu_pct_max': round(max(s), 1),
        'threads': len(thr1), 'top_thread_cpu_pct': [round(x, 1) for x, _ in thr_delta],
        'cores_used_share': {c: round(100*v/sum(cores_seen[k].values())) for c, v in sorted(cores_seen[k].items())},
        'vol_ctx_per_s': round((c1.voluntary-ctx0[k].voluntary)/DUR),
        'invol_ctx_per_s': round((c1.involuntary-ctx0[k].involuntary)/DUR),
        'affinity': sorted(p.cpu_affinity())}
res['pipeline_total_cpu_pct'] = round(pipe_total, 1)
res['whole_system_cpu_pct'] = round(sum(res['cores_pct_mean']), 1)
res['other_cpu_pct_desktop_etc'] = round(sum(res['cores_pct_mean']) - pipe_total, 1)
rows = [r.split(',') for r in open(csv).read().strip().splitlines()[1:]] if os.path.exists(csv) else []
agg = collections.defaultdict(lambda: {'hz': [], 'lat': []})
for r in rows:
    agg[r[2]]['hz'].append(float(r[3]))
    if r[6]: agg[r[2]]['lat'].append(float(r[6]))
res['topics'] = {t: {'hz_mean': round(sum(v['hz'])/len(v['hz']), 2),
                     'lat_mean_ms': round(sum(v['lat'])/len(v['lat']), 1) if v['lat'] else None}
                 for t, v in agg.items()}
json.dump(res, open(f'{OUT}/{LABEL}.json', 'w'), indent=1)
print(json.dumps(res, indent=1))
