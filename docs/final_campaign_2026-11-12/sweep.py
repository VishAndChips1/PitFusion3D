#!/usr/bin/env python3
"""Point-cap sweep: raise the streamer's point cap in small steps and measure, per step, quality, refresh rate,
payload/bandwidth, CPU, latency and (fast-sampled) current/voltage, with hard safety gates.
Usage: sweep.py <merged|live> <cap,cap,...> <seconds_per_step> [tag]     (run from a shell with ROS sourced)"""
import os, sys, re, json, time, threading, subprocess, http.client, gzip, struct, csv, statistics as st
import numpy as np, psutil
from scipy.spatial import cKDTree

VIEW, CAPS, STEP_S = sys.argv[1], [int(x) for x in sys.argv[2].split(',')], int(sys.argv[3])
TAG = sys.argv[4] if len(sys.argv) > 4 else VIEW
HERE = os.path.dirname(os.path.abspath(__file__)); SETTLE = 12
PATH = '/merged.bin' if VIEW == 'merged' else '/live.bin'; POLL = 0.25 if VIEW == 'merged' else 0.12
DEFAULT_CAP = {'merged': 20000, 'live': 5000}[VIEW]
FULL_TOPIC = '/landfill/fusion/points_merged' if VIEW == 'merged' else '/landfill/fusion/points_rgbt'
SAFE = dict(ext5v_min=4.80, est_in_max=4.5, temp_max=78.0, core_abs=12.0)   # est_in = rails W / (V*0.85) + 0.5 A USB allowance
RES = f'{HERE}/results_{TAG}.jsonl'; RAW = f'{HERE}/power_raw_{TAG}.csv'; LOG = f'{HERE}/sweep_{TAG}.log'
def log(m):
    l = f'[{time.strftime("%H:%M:%S")}] {m}'; print(l, flush=True); open(LOG, 'a').write(l + '\n')

def pmic():
    out = subprocess.run(['vcgencmd', 'pmic_read_adc'], capture_output=True, text=True).stdout; d = {}
    for m in re.finditer(r'(\S+?)_([AV]) (?:current|volt)\(\d+\)=([\d.]+)', out): d[(m.group(1), m.group(2))] = float(m.group(3))
    return d
def throttled():
    m = re.search(r'0x([0-9a-fA-F]+)', subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True).stdout); return int(m.group(1), 16) if m else -1
def temp(): return float(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000

class Power(threading.Thread):
    def __init__(self): super().__init__(daemon=True); self.step = 'idle'; self.rows = []; self.abort = None; self.stop = False; self.core_gate = SAFE['core_abs']; self.thr0 = throttled(); self.n = 0
    def run(self):
        f = open(RAW, 'a', newline=''); w = csv.writer(f)
        if f.tell() == 0: w.writerow(['t', 'step', 'ext5v_v', 'core_a', 'core_v', 'wl_a', 'total_w', 'est_in_a', 'est_in_cons_a', 'temp_c'])
        low = high = 0; last_thr = 0
        while not self.stop:
            d = pmic(); t = time.time(); e = d.get(('EXT5V', 'V'), float('nan'))
            tw = sum(d[(b, 'V')] * d[(b, 'A')] for (b, k) in list(d) if k == 'V' and (b, 'A') in d)
            core = d.get(('VDD_CORE', 'A'), float('nan')); est = tw / e; cons = tw / (e * 0.85) + 0.5; tc = temp()
            row = (t, self.step, e, core, d.get(('VDD_CORE', 'V'), float('nan')), d.get(('3V7_WL_SW', 'A'), float('nan')), tw, est, cons, tc)
            self.rows.append(row); w.writerow([f'{x:.4f}' if isinstance(x, float) else x for x in row]); self.n += 1
            if self.n % 40 == 0: f.flush()
            low = low + 1 if e < SAFE['ext5v_min'] else 0; high = high + 1 if core > self.core_gate else 0
            why = None
            if low >= 2: why = f'EXT5V sag {e:.3f} V'
            elif cons > SAFE['est_in_max']: why = f'estimated input current {cons:.2f} A > {SAFE["est_in_max"]} A'
            elif high >= 2: why = f'VDD_CORE {core:.2f} A > gate {self.core_gate:.2f} A'
            elif tc >= SAFE['temp_max']: why = f'SoC {tc:.1f} C'
            if not why and t - last_thr > 1.0:
                last_thr = t; th = throttled()
                if th != self.thr0 and th != -1 and (th & 0xF) != 0: why = f'throttle flags now 0x{th:x}'
            if why and not self.abort: self.abort = why; log(f'!!! SAFETY ABORT: {why}'); restore()
        f.flush(); f.close()

def restore():   # fastest guaranteed way back to the default caps: respawn the streamer
    for p in psutil.process_iter(['cmdline']):
        cl = p.info['cmdline'] or []
        if '--ros-args' in cl and any('fusion_web_streamer' in t for t in cl[:3]):
            try: p.kill()
            except psutil.Error: pass

def set_cap(n):
    # `ros2 param set` occasionally hangs (CLI/discovery flake, seen once at 18:26); retry instead of dying.
    for attempt in range(3):
        try:
            r = subprocess.run(['ros2', 'param', 'set', '/fusion_web_streamer', f'max_points_{VIEW}', str(n)], capture_output=True, text=True, timeout=25)
            if 'successful' in r.stdout: return True
        except subprocess.TimeoutExpired:
            log(f'ros2 param set timed out (attempt {attempt + 1}/3)')
        time.sleep(3)
    return False

class Ref(threading.Thread):   # one simulated viewer over loopback, same cadence as the real page, with ETag
    def __init__(self): super().__init__(daemon=True); self.stop = False; self.reset()
    def reset(self): self.t_done = []; self.resp = []; self.wire = []; self.raw = []; self.n = []; self.last_payload = None; self.errors = 0; self.reqs = 0
    def run(self):
        c = http.client.HTTPConnection('localhost', 8080, timeout=10); et = None
        while not self.stop:
            t = time.time()
            try:
                c.request('GET', PATH, headers={'If-None-Match': et} if et else {}); r = c.getresponse(); b = r.read(); dt = time.time() - t; self.reqs += 1
                if r.status == 200:
                    et = r.getheader('ETag'); raw = gzip.decompress(b); self.t_done.append(time.time()); self.resp.append(dt * 1000); self.wire.append(len(b)); self.raw.append(len(raw))
                    self.n.append(struct.unpack_from('<I', raw, 0)[0]); self.last_payload = raw
            except Exception: self.errors += 1; c = http.client.HTTPConnection('localhost', 8080, timeout=10)
            time.sleep(max(0.02, POLL - (time.time() - t)))

def decode(raw):
    n, seq = struct.unpack_from('<II', raw, 0); ox, oy, oz, sc = struct.unpack_from('<ffff', raw, 8)
    a = np.frombuffer(raw, dtype=np.dtype([('xyz', '<i2', 3), ('rgb', 'u1', 3), ('t', '<i2')]), count=n, offset=24)
    return np.array([ox, oy, oz]) + a['xyz'].astype(np.float64) * sc
def vox(x, s): return set(map(tuple, np.unique(np.floor(x / s).astype(np.int64), axis=0)))
def quality(raw):
    x = decode(raw); out = dict(points=int(len(x)))
    d, _ = cKDTree(x).query(x, k=2); out['nn_median_cm'] = round(float(np.median(d[:, 1])) * 100, 3)
    out['occ_2cm'] = len(vox(x, 0.02)); out['occ_5cm'] = len(vox(x, 0.05))
    fp = f'{HERE}/_full_{TAG}.npy'
    if os.path.exists(fp): os.remove(fp)
    subprocess.run([sys.executable, f'{HERE}/grab_full.py', FULL_TOPIC, fp], timeout=40)
    if os.path.exists(fp):
        f = np.load(fp); out['full_points'] = int(len(f)); out['full_occ_2cm'] = len(vox(f, 0.02)); out['full_occ_5cm'] = len(vox(f, 0.05))
        out['coverage_2cm_pct'] = round(100 * len(vox(x, 0.02) & vox(f, 0.02)) / out['full_occ_2cm'], 1)
        out['coverage_5cm_pct'] = round(100 * len(vox(x, 0.05) & vox(f, 0.05)) / out['full_occ_5cm'], 1)
    return out

def clients():
    r = subprocess.run("ss -tn state established '( sport = :8080 )'", shell=True, capture_output=True, text=True).stdout.splitlines()[1:]
    return sum(1 for l in r if l.split() and not l.split()[-1].startswith(('127.', '[::1]', '::ffff:127.')))
def wl(): return int(open('/sys/class/net/wlan0/statistics/tx_bytes').read())
PAT = ['livox_ros2_avia_node', 'livox_restamp_node', 'calibration_publisher', 'rgb_camera_node', 'thermal_camera_node', 'sensor_fusion_node', 'registration_merge_node', 'fusion_web_streamer']
def procs():
    out = {}
    for p in psutil.process_iter(['cmdline']):
        cl = p.info['cmdline'] or []
        if '--ros-args' in cl:
            for k in PAT:
                if any(k in t for t in cl[:3]): out[k] = p
        elif cl and cl[0] == 'rpicam-vid': out['rpicam'] = p
    return out
pct = lambda a, q: float(np.percentile(a, q)) if len(a) else None

pw = Power(); pw.start(); ref = Ref(); ref.start(); time.sleep(2)
log(f'sweep {TAG}: view={VIEW} caps={CAPS} {STEP_S}s/step; gates: {SAFE}; throttle baseline 0x{pw.thr0:x}')
base_core_max = None
try:
    for cap in CAPS:
        if pw.abort: break
        if not set_cap(cap): log(f'could not set cap {cap}'); break
        pr = procs(); [p.cpu_percent(None) for p in pr.values()]; psutil.cpu_percent(percpu=True)
        time.sleep(SETTLE)
        if pw.abort: break
        step = f'{VIEW}_{cap}'; pw.step = step; ref.reset(); pr = procs(); [p.cpu_percent(None) for p in pr.values()]; psutil.cpu_percent(percpu=True)
        trk = f'{HERE}/_rates_{TAG}_{cap}.csv'
        if os.path.exists(trk): os.remove(trk)
        tp = subprocess.Popen([sys.executable, '/home/vish/thesis_tests/20261111_v3_campaign/rate_tracker_subprocess.py', str(STEP_S), trk, step, '{}'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        t0 = time.time(); tx0 = wl(); cl = [clients()]; cpu = []; core = []; q = None; sys_cpu = []
        while time.time() - t0 < STEP_S and not pw.abort:
            time.sleep(1.0)
            cpu.append({k: p.cpu_percent(None) for k, p in pr.items()}); core.append(psutil.cpu_percent(percpu=True)); cl.append(clients())
            if q is None and time.time() - t0 > STEP_S / 2 and ref.last_payload:
                try: q = quality(ref.last_payload)
                except Exception as e: q = {'error': repr(e)}
        el = time.time() - t0; tx1 = wl()
        try: tp.wait(timeout=30)
        except subprocess.TimeoutExpired: tp.kill()
        rows = [r for r in pw.rows if r[1] == step]; col = lambda i: np.array([r[i] for r in rows if r[i] == r[i]])
        gaps = np.diff(ref.t_done) * 1000 if len(ref.t_done) > 2 else np.array([])
        topics = {}
        if os.path.exists(trk):
            import collections; a = collections.defaultdict(lambda: [[], []])
            for r in list(csv.reader(open(trk)))[1:]:
                a[r[2].split('/')[-1] if 'image' not in r[2] else r[2].split('/')[2]][0].append(float(r[3]))
                if r[6]: a[r[2].split('/')[-1] if 'image' not in r[2] else r[2].split('/')[2]][1].append(float(r[6]))
            topics = {k: dict(hz=round(float(np.mean(v[0])), 2), lat_ms=round(float(np.mean(v[1])), 1) if v[1] else None) for k, v in a.items()}
        pipe = [sum(v for k, v in s.items() if k != 'fusion_web_streamer') + s.get('fusion_web_streamer', 0) for s in cpu]
        res = dict(tag=TAG, view=VIEW, cap=cap, seconds=round(el, 1), aborted=pw.abort, t_start=time.strftime('%F %T', time.localtime(t0)),
            points_per_cloud=round(float(np.mean(ref.n)), 0) if ref.n else None, clouds=len(ref.n), updates_per_s=round(len(ref.n) / el, 2),
            wire_kb_mean=round(float(np.mean(ref.wire)) / 1024, 1) if ref.wire else None, wire_kb_max=round(max(ref.wire) / 1024, 1) if ref.wire else None,
            raw_kb_mean=round(float(np.mean(ref.raw)) / 1024, 1) if ref.raw else None,
            viewer_mbit_s=round(sum(ref.wire) * 8 / el / 1e6, 3), points_per_s=round(sum(ref.n) / el),
            gap_ms=dict(p50=pct(gaps, 50), p95=pct(gaps, 95), max=float(gaps.max()) if len(gaps) else None),
            resp_ms=dict(p50=pct(ref.resp, 50), p95=pct(ref.resp, 95), max=max(ref.resp) if ref.resp else None), http_errors=ref.errors,
            wlan0_tx_mbit_s=round((tx1 - tx0) * 8 / el / 1e6, 3), real_clients=dict(min=min(cl), max=max(cl)), topics=topics,
            cpu=dict(pipeline_pct=round(float(np.mean(pipe)), 1) if pipe else None, streamer_pct=round(float(np.mean([s.get('fusion_web_streamer', 0) for s in cpu])), 1) if cpu else None,
                     merge_pct=round(float(np.mean([s.get('registration_merge_node', 0) for s in cpu])), 1) if cpu else None, system_pct_of_400=round(float(np.mean([sum(c) for c in core])), 1) if core else None,
                     per_core=[round(float(np.mean([c[i] for c in core])), 1) for i in range(4)] if core else None),
            quality=q, power=dict(samples=len(rows), hz=round(len(rows) / el, 1), ext5v_min=float(col(2).min()), ext5v_mean=float(col(2).mean()),
                core_a=dict(mean=float(col(3).mean()), p95=pct(col(3), 95), p99=pct(col(3), 99), max=float(col(3).max()), pct_samples_over_5A=round(100 * float((col(3) > 5).mean()), 2)),
                wifi_rail_a=dict(mean=float(col(5).mean()), max=float(col(5).max())), total_w=dict(mean=float(col(6).mean()), max=float(col(6).max())),
                est_input_a=dict(mean=float(col(7).mean()), max=float(col(7).max())), est_input_conservative_a_max=float(col(8).max()), temp_c_max=float(col(9).max()), throttled_now=hex(throttled())))
        open(RES, 'a').write(json.dumps(res) + '\n')
        p = res['power']; log(f"cap {cap}: {res['points_per_cloud']:.0f} pts, {res['updates_per_s']} upd/s, {res['wire_kb_mean']} KB/cloud, {res['viewer_mbit_s']} Mbit/s | CPU pipe {res['cpu']['pipeline_pct']}% streamer {res['cpu']['streamer_pct']}% | core A max {p['core_a']['max']:.2f} p99 {p['core_a']['p99']:.2f} | est in max {p['est_input_a']['max']:.2f} A (cons {p['est_input_conservative_a_max']:.2f}) | EXT5V min {p['ext5v_min']:.3f} | fused {topics.get('points_rgbt',{}).get('hz')} Hz | clients {min(cl)}-{max(cl)} | {q}")
        if base_core_max is None:
            base_core_max = p['core_a']['max']; pw.core_gate = max(5.0, 1.3 * base_core_max); log(f'baseline VDD_CORE max {base_core_max:.2f} A -> core gate for later steps {pw.core_gate:.2f} A')
finally:
    ref.stop = True
    if not pw.abort:
        try: set_cap(DEFAULT_CAP)
        except Exception: restore()
    pw.stop = True; time.sleep(1)
    log(f'done; abort={pw.abort}; caps restored to {DEFAULT_CAP}')
