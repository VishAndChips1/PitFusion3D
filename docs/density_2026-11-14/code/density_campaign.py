#!/usr/bin/env python3
"""Merged-view density campaign: how many points can the viewer show before something gives?

Each step runs the real pipeline with a given merged-point cap while measuring, with hard safety gates:
  * a viewer simulator that behaves like web_assets/viewer.html (conditional GET every 250 ms, 4 s abort, gunzip + decode),
    optionally through a throttled link and optionally several at once;
  * real WiFi-radio transmit load that mirrors the viewer's byte rate (UDP to the router out of wlan0), because the viewer
    runs on this machine and loopback traffic never touches the radio;
  * 31 Hz PMIC power sampling, SoC temperature, throttle flags, CPU per process, memory, upstream topic rates, point coverage.

Usage (ROS sourced, pipeline already running):
  density_campaign.py ladder <tag> <cap,cap,..> [secs]        step through caps at the current window
  density_campaign.py multi  <tag> <cap> <viewers> [secs]      several simultaneous viewers
  density_campaign.py link   <tag> <cap> <mbit,mbit,..> [secs] throttled links (viewer sees a slow WiFi)
  density_campaign.py soak   <tag> <cap> <minutes>             long run, one-minute snapshots
"""
import asyncio, collections, csv, gzip, http.client, json, os, re, socket, struct, subprocess, sys, threading, time
import numpy as np
import psutil
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__)); DATA = os.path.join(HERE, '..', 'data')
PHASE, TAG = sys.argv[1], sys.argv[2]
SAFE = dict(ext5v_min=4.75, est_in_max=4.4, temp_max=82.0, core_abs=12.0)   # campaign gates (estimated input current is the conservative figure)
RES = f'{DATA}/results_{TAG}.jsonl'; RAW = f'{DATA}/power_raw_{TAG}.csv'; LOG = f'{DATA}/campaign_{TAG}.log'
GATEWAY = next((l.split()[2] for l in subprocess.run(['ip', 'route'], capture_output=True, text=True).stdout.splitlines() if l.startswith('default') and 'wlan0' in l), None)
PROXY_PORT = 18081; PIPE_LOG_GLOB = '/tmp/sweep_launch/launch_'
PROCS = ['livox_ros2_avia_node', 'livox_restamp_node', 'calibration_publisher', 'rgb_camera_node', 'thermal_camera_node', 'sensor_fusion_node',
         'registration_merge_node', 'fusion_web_streamer']


def log(m):
    line = f'[{time.strftime("%H:%M:%S")}] {m}'; print(line, flush=True); open(LOG, 'a').write(line + '\n')


# ---------------------------------------------------------------- power + safety
def pmic():
    out = subprocess.run(['vcgencmd', 'pmic_read_adc'], capture_output=True, text=True).stdout; d = {}
    for m in re.finditer(r'(\S+?)_([AV]) (?:current|volt)\(\d+\)=([\d.]+)', out):
        d[(m.group(1), m.group(2))] = float(m.group(3))
    return d


def throttled():
    m = re.search(r'0x([0-9a-fA-F]+)', subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True).stdout)
    return int(m.group(1), 16) if m else -1


def soc_temp():
    return float(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000


class Power(threading.Thread):
    """Sample the PMIC at ~31 Hz, log every sample, and raise an abort flag if a safety gate trips."""

    def __init__(self):
        super().__init__(daemon=True)
        self.step, self.rows, self.abort, self.stop_flag, self.n = 'idle', [], None, False, 0
        self.core_gate, self.thr0, self.on_abort = SAFE['core_abs'], throttled(), None

    def run(self):
        f = open(RAW, 'a', newline=''); w = csv.writer(f)
        if f.tell() == 0:
            w.writerow(['t', 'step', 'ext5v_v', 'core_a', 'core_v', 'wl_a', 'total_w', 'est_in_a', 'est_in_cons_a', 'temp_c'])
        low = high = 0; last_thr = 0
        while not self.stop_flag:
            d = pmic(); t = time.time(); e = d.get(('EXT5V', 'V'), float('nan'))
            tw = sum(d[(b, 'V')] * d[(b, 'A')] for (b, k) in list(d) if k == 'V' and (b, 'A') in d)
            core = d.get(('VDD_CORE', 'A'), float('nan')); est = tw / e; cons = tw / (e * 0.85) + 0.5; tc = soc_temp()
            row = (t, self.step, e, core, d.get(('VDD_CORE', 'V'), float('nan')), d.get(('3V7_WL_SW', 'A'), float('nan')), tw, est, cons, tc)
            self.rows.append(row); w.writerow([f'{x:.4f}' if isinstance(x, float) else x for x in row]); self.n += 1
            if self.n % 40 == 0:
                f.flush()
            low = low + 1 if e < SAFE['ext5v_min'] else 0; high = high + 1 if core > self.core_gate else 0
            why = None
            if low >= 2: why = f'EXT5V sag {e:.3f} V'
            elif cons > SAFE['est_in_max']: why = f'estimated input current {cons:.2f} A > {SAFE["est_in_max"]} A'
            elif high >= 2: why = f'VDD_CORE {core:.2f} A > gate {self.core_gate:.2f} A'
            elif tc >= SAFE['temp_max']: why = f'SoC {tc:.1f} C'
            if not why and t - last_thr > 1.0:
                last_thr = t; th = throttled()
                if th != self.thr0 and th != -1 and (th & 0xF) != 0: why = f'throttle flags now 0x{th:x}'
            if why and not self.abort:
                self.abort = why; log(f'!!! SAFETY ABORT: {why}')
                if self.on_abort: self.on_abort()
        f.flush(); f.close()


# ---------------------------------------------------------------- link throttle (local proxy)
class Link:
    rate = None   # bytes/s, None = unlimited


async def _up(r, w):
    try:
        while True:
            d = await r.read(65536)
            if not d: break
            w.write(d); await w.drain()
    except Exception: pass
    finally:
        try: w.close()
        except Exception: pass


async def _down(r, w):
    try:
        t_next = time.monotonic()
        while True:
            d = await r.read(8192)
            if not d: break
            if Link.rate:
                t_next = max(t_next, time.monotonic()) + len(d) / Link.rate; delay = t_next - time.monotonic()
                if delay > 0: await asyncio.sleep(delay)
            w.write(d); await w.drain()
    except Exception: pass
    finally:
        try: w.close()
        except Exception: pass


async def _handle(cr, cw):
    try: ur, uw = await asyncio.open_connection('127.0.0.1', 8080)
    except Exception: cw.close(); return
    await asyncio.gather(_up(cr, uw), _down(ur, cw))


def start_proxy():
    def run():
        loop = asyncio.new_event_loop(); asyncio.set_event_loop(loop)
        loop.run_until_complete(asyncio.start_server(_handle, '127.0.0.1', PROXY_PORT)); loop.run_forever()
    threading.Thread(target=run, daemon=True).start(); time.sleep(1)


# ---------------------------------------------------------------- viewer simulator
DT = np.dtype([('xyz', '<i2', 3), ('rgb', 'u1', 3), ('t', '<i2')])


class TierController:
    """Python port of the page's TierController (same levels and thresholds)."""
    levels = [8000, 20000, 40000, 60000, 80000, 120000]

    def __init__(self): self.index, self.good, self.blocked = len(self.levels) - 1, 0, 0.0

    def requested(self): return self.levels[self.index]

    def report(self, download_ms, aborted, now_ms):
        if aborted:
            self.index = max(0, self.index - 2); self.good = 0; self.blocked = now_ms + 60000
        elif download_ms > 900:
            if self.index > 0: self.index -= 1; self.blocked = now_ms + 60000
            self.good = 0
        elif download_ms < 400:
            self.good += 1
            if self.good >= 8 and self.index < len(self.levels) - 1 and now_ms >= self.blocked: self.index += 1; self.good = 0
        else:
            self.good = 0


ADAPTIVE = os.environ.get('ADAPTIVE', '1') == '1'   # 1 = the new page (adaptive levels, 8 s abort); 0 = the old page (fixed, 4 s abort)


class Viewer(threading.Thread):
    """Behaves like viewer.html: conditional GET, 250 ms target interval, 4 s abort, then gunzip + decode every new cloud."""

    def __init__(self, port=PROXY_PORT, interval=0.25, abort_s=None, path='/merged.bin'):
        super().__init__(daemon=True)
        self.adaptive = ADAPTIVE; self.tier = TierController()
        self.port, self.interval, self.abort_s, self.path, self.stop_flag = port, interval, abort_s or (8.0 if ADAPTIVE else 4.0), path, False
        self.reset()

    def reset(self):
        self.t_done, self.resp, self.wire, self.n, self.decode_ms, self.age = [], [], [], [], [], []
        self.aborts = self.errors = self.reqs = self.not_modified = 0; self.last_payload = None; self.levels_used = []

    def _fetch(self, conn, etag):
        t0 = time.time(); deadline = t0 + self.abort_s
        h = {'Accept-Encoding': 'gzip'}
        if etag: h['If-None-Match'] = etag
        url = self.path + (f'?n={self.tier.requested()}' if self.adaptive else '')
        conn.request('GET', url, headers=h); r = conn.getresponse()
        if r.status == 304:
            r.read(); return 304, None, None, t0
        body = bytearray()
        while True:
            conn.sock.settimeout(max(0.05, deadline - time.time()))
            chunk = r.read(65536)
            if not chunk: break
            body += chunk
            if time.time() > deadline: raise TimeoutError('viewer abort')
        return 200, bytes(body), r, t0

    def run(self):
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=self.abort_s); etag = None
        while not self.stop_flag:
            t = time.time()
            try:
                self.reqs += 1; status, body, r, t0 = self._fetch(conn, etag)
                if status == 304: self.not_modified += 1
                else:
                    resp = time.time() - t0; etag = r.getheader('ETag'); stamp = float(r.getheader('X-Cloud-Stamp') or 0)
                    if self.adaptive: self.levels_used.append(self.tier.requested()); self.tier.report(resp * 1000, False, time.monotonic() * 1000)
                    d0 = time.time(); raw = gzip.decompress(body)
                    n = struct.unpack_from('<I', raw, 0)[0]; a = np.frombuffer(raw, DT, count=n, offset=24)
                    ox, oy, oz, sc = struct.unpack_from('<ffff', raw, 8)   # same work the page does: scale xyz, colour to 0..1, thermal to float
                    _xyz = a['xyz'].astype(np.float32) * sc + np.array([ox, oy, oz], np.float32)
                    _rgb = a['rgb'].astype(np.float32) / 255.0
                    _th = np.where(a['t'] == -32768, np.nan, a['t'] / 100.0)
                    self.decode_ms.append((time.time() - d0) * 1000); now = time.time()
                    self.t_done.append(now); self.resp.append(resp * 1000); self.wire.append(len(body)); self.n.append(n)
                    if stamp > 1e9: self.age.append((now - stamp) * 1000)
                    self.last_payload = raw
            except TimeoutError:
                self.aborts += 1; conn.close(); conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=self.abort_s); etag = None
                if self.adaptive: self.tier.report(None, True, time.monotonic() * 1000)
            except Exception:
                self.errors += 1; conn.close(); conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=self.abort_s)
            time.sleep(max(0.02, self.interval - (time.time() - t)))


# ---------------------------------------------------------------- WiFi radio load
class Radio(threading.Thread):
    """UDP datagrams to the router out of wlan0 at the byte rate the viewers just received (closed-loop mirror)."""

    def __init__(self, viewers):
        super().__init__(daemon=True)
        self.viewers, self.stop_flag, self.sent = viewers, False, 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)

    def run(self):
        pkt = os.urandom(1200); last = 0; tick = time.time(); budget = 0.0
        while not self.stop_flag:
            time.sleep(0.01); now = time.time()
            if now - tick >= 1.0:   # re-measure the viewers' byte rate once a second
                recent = sum(sum(w for t, w in zip(v.t_done, v.wire) if t > now - 3) for v in self.viewers) / 3.0
                self.rate = min(recent, 2.5e6); tick = now   # never above 20 Mbit/s
            budget += getattr(self, 'rate', 0.0) * 0.01
            while budget >= 1200 and GATEWAY:
                try: self.sock.sendto(pkt, (GATEWAY, 9)); self.sent += 1200
                except OSError: break
                budget -= 1200


def wl_tx(): return int(open('/sys/class/net/wlan0/statistics/tx_bytes').read())


# ---------------------------------------------------------------- helpers
def set_cap(n):
    for attempt in range(4):
        try:
            r = subprocess.run(['ros2', 'param', 'set', '/fusion_web_streamer', 'max_points_merged', str(n)], capture_output=True, text=True, timeout=25)
            if 'successful' in r.stdout: return True
        except subprocess.TimeoutExpired: log(f'param set timeout {attempt + 1}/4')
        time.sleep(3)
    return False


def current_cap():
    """Read the streamer's present ceiling (falls back to 80000, the production default)."""
    for _ in range(3):
        try:
            r = subprocess.run(['ros2', 'param', 'get', '/fusion_web_streamer', 'max_points_merged'], capture_output=True, text=True, timeout=25)
            m = re.search(r'(\d+)', r.stdout)
            if m: return int(m.group(1))
        except subprocess.TimeoutExpired: pass
        time.sleep(2)
    return 80000


def procs():
    out = {}
    for p in psutil.process_iter(['cmdline']):
        cl = p.info['cmdline'] or []
        if '--ros-args' in cl:
            for k in PROCS:
                if any(k in t for t in cl[:3]): out[k] = p
    return out


def pct(a, q): return float(np.percentile(a, q)) if len(a) else None


def decode_xyz(raw):
    n = struct.unpack_from('<I', raw, 0)[0]; ox, oy, oz, sc = struct.unpack_from('<ffff', raw, 8)
    a = np.frombuffer(raw, DT, count=n, offset=24)
    return np.array([ox, oy, oz]) + a['xyz'].astype(np.float64) * sc


def vox(x, s): return set(map(tuple, np.unique(np.floor(x / s).astype(np.int64), axis=0)))


def quality(raw):
    x = decode_xyz(raw); out = dict(points=int(len(x)))
    d, _ = cKDTree(x).query(x, k=2); out['nn_median_cm'] = round(float(np.median(d[:, 1])) * 100, 3)
    fp = f'{DATA}/_full_{TAG}.npy'
    if os.path.exists(fp): os.remove(fp)
    subprocess.run([sys.executable, f'{HERE}/grab_full.py', '/landfill/fusion/points_merged', fp], timeout=40)
    if os.path.exists(fp):
        f = np.load(fp); out['full_points'] = int(len(f))
        for s, k in ((0.02, '2cm'), (0.05, '5cm')):
            vf = vox(f, s); out[f'coverage_{k}_pct'] = round(100 * len(vox(x, s) & vf) / len(vf), 1)
    return out


def pipeline_log_counts():
    """(merge-worker-behind warnings, non-converged ICP warnings, window count) so far, from the newest pipeline launch log."""
    d = os.path.join(DATA, 'pipeline_logs')
    logs = sorted(os.listdir(d), key=lambda f: os.path.getmtime(os.path.join(d, f))) if os.path.isdir(d) else []
    if not logs: return (0, 0, 0)
    t = open(os.path.join(d, logs[-1]), errors='ignore').read()
    return (t.count('Merge worker behind'), t.count('did not converge'), t.count('Published merged cloud'))


# ---------------------------------------------------------------- one measured step
def run_step(pw, viewers, radio, label, cap, secs, link_mbit=None, snap_every=0):
    Link.rate = link_mbit * 1e6 / 8 if link_mbit else None
    if cap is not None and not set_cap(cap): log(f'could not set cap {cap}'); return None
    pr = procs(); [p.cpu_percent(None) for p in pr.values()]; psutil.cpu_percent(percpu=True); time.sleep(12)   # settle
    if pw.abort: return None
    pw.step = label; [v.reset() for v in viewers]; pr = procs(); [p.cpu_percent(None) for p in pr.values()]; psutil.cpu_percent(percpu=True)
    rss0 = {k: pr[k].memory_info().rss / 1e6 for k in ('fusion_web_streamer', 'registration_merge_node') if k in pr}
    trk = f'{DATA}/_rates_{TAG}_{label}.csv'
    if os.path.exists(trk): os.remove(trk)
    tp = subprocess.Popen([sys.executable, '/home/vish/thesis_tests/20261111_v3_campaign/rate_tracker_subprocess.py', str(secs), trk, label, '{}'],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0 = time.time(); tx0 = wl_tx(); sent0 = radio.sent; c0 = pipeline_log_counts(); cpu, core, q, series = [], [], None, []; next_snap = t0 + snap_every
    while time.time() - t0 < secs and not pw.abort:
        time.sleep(1.0); cpu.append({k: p.cpu_percent(None) for k, p in pr.items()}); core.append(psutil.cpu_percent(percpu=True))
        if q is None and not snap_every and time.time() - t0 > secs / 2 and viewers[0].last_payload:
            try: q = quality(viewers[0].last_payload)
            except Exception as e: q = {'error': repr(e)}
        if snap_every and time.time() >= next_snap:
            next_snap += snap_every; v0 = viewers[0]; recent = [t for t in v0.t_done if t > time.time() - snap_every]
            rows = [r for r in pw.rows[-int(31 * snap_every):]]
            series.append(dict(min=round((time.time() - t0) / 60, 1), upd_per_s=round(len(recent) / snap_every, 2), temp_c=round(max(r[9] for r in rows), 1) if rows else None,
                               streamer_rss_mb=round(pr['fusion_web_streamer'].memory_info().rss / 1e6, 1), merge_rss_mb=round(pr['registration_merge_node'].memory_info().rss / 1e6, 1),
                               aborts=sum(v.aborts for v in viewers), throttled=hex(throttled())))
            log(f'  soak {series[-1]}')
    el = time.time() - t0; tx1 = wl_tx(); c1 = pipeline_log_counts()
    try: tp.wait(timeout=30)
    except subprocess.TimeoutExpired: tp.kill()
    rows = [r for r in pw.rows if r[1] == label]; col = lambda i: np.array([r[i] for r in rows if r[i] == r[i]])
    v0 = viewers[0]; gaps = np.diff(v0.t_done) * 1000 if len(v0.t_done) > 2 else np.array([])
    topics = {}
    if os.path.exists(trk):
        a = collections.defaultdict(lambda: [[], []])
        for r in list(csv.reader(open(trk)))[1:]:
            k = r[2].split('/')[-1] if 'image' not in r[2] else r[2].split('/')[2]
            a[k][0].append(float(r[3]))
            if r[6]: a[k][1].append(float(r[6]))
        topics = {k: dict(hz=round(float(np.mean(v[0])), 2), lat_ms=round(float(np.mean(v[1])), 1) if v[1] else None) for k, v in a.items()}
    pipe = [sum(v for k, v in s.items()) for s in cpu]
    res = dict(tag=TAG, label=label, cap=cap, link_mbit=link_mbit, viewers=len(viewers), seconds=round(el, 1), aborted=pw.abort,
               t_start=time.strftime('%F %T', time.localtime(t0)),
               points_per_cloud=round(float(np.mean(v0.n)), 0) if v0.n else None, clouds=len(v0.n), updates_per_s=round(len(v0.n) / el, 3),
               mean_level_requested=round(float(np.mean(v0.levels_used))) if v0.levels_used else None, adaptive=ADAPTIVE,
               per_viewer_updates_per_s=[round(len(v.n) / el, 3) for v in viewers], aborts=[v.aborts for v in viewers], errors=[v.errors for v in viewers],
               wire_kb_mean=round(float(np.mean(v0.wire)) / 1024, 1) if v0.wire else None, viewer_mbit_s=round(sum(sum(v.wire) for v in viewers) * 8 / el / 1e6, 3),
               gap_ms=dict(p50=pct(gaps, 50), p95=pct(gaps, 95), max=float(gaps.max()) if len(gaps) else None),
               resp_ms=dict(p50=pct(v0.resp, 50), p95=pct(v0.resp, 95), max=max(v0.resp) if v0.resp else None),
               decode_ms=dict(p50=pct(v0.decode_ms, 50), p95=pct(v0.decode_ms, 95)), data_age_ms=dict(p50=pct(v0.age, 50), p95=pct(v0.age, 95)),
               radio_sent_mbit_s=round((radio.sent - sent0) * 8 / el / 1e6, 3), wlan0_tx_mbit_s=round((tx1 - tx0) * 8 / el / 1e6, 3),
               merge_worker_behind=c1[0] - c0[0], icp_not_converged=c1[1] - c0[1], windows=c1[2] - c0[2], topics=topics,
               cpu=dict(pipeline_pct=round(float(np.mean(pipe)), 1) if pipe else None, streamer_pct=round(float(np.mean([s.get('fusion_web_streamer', 0) for s in cpu])), 1) if cpu else None,
                        merge_pct=round(float(np.mean([s.get('registration_merge_node', 0) for s in cpu])), 1) if cpu else None,
                        system_pct_of_400=round(float(np.mean([sum(c) for c in core])), 1) if core else None,
                        per_core=[round(float(np.mean([c[i] for c in core])), 1) for i in range(4)] if core else None),
               rss_mb_start=rss0, rss_mb_end={k: round(pr[k].memory_info().rss / 1e6, 1) for k in rss0 if k in pr}, quality=q, series=series,
               power=dict(samples=len(rows), ext5v_min=float(col(2).min()) if len(rows) else None, ext5v_mean=float(col(2).mean()) if len(rows) else None,
                          core_a=dict(mean=float(col(3).mean()), p99=pct(col(3), 99), max=float(col(3).max())) if len(rows) else None,
                          wifi_rail_a=dict(mean=float(col(5).mean()), max=float(col(5).max())) if len(rows) else None,
                          total_w=dict(mean=float(col(6).mean()), max=float(col(6).max())) if len(rows) else None,
                          est_input_a=dict(mean=float(col(7).mean()), max=float(col(7).max())) if len(rows) else None,
                          est_input_conservative_a_max=float(col(8).max()) if len(rows) else None, temp_c_max=float(col(9).max()) if len(rows) else None,
                          throttled_now=hex(throttled())))
    open(RES, 'a').write(json.dumps(res) + '\n')
    log(f"{label}: {res['points_per_cloud']} pts, {res['updates_per_s']} upd/s (viewers {res['per_viewer_updates_per_s']}), {res['wire_kb_mean']} KB, {res['viewer_mbit_s']} Mbit/s, "
        f"aborts {res['aborts']}, age p50 {res['data_age_ms']['p50']}, streamer {res['cpu']['streamer_pct']}%, pipeline {res['cpu']['pipeline_pct']}%, "
        f"est in {res['power']['est_input_conservative_a_max']}, wlan tx {res['wlan0_tx_mbit_s']} Mbit/s, T {res['power']['temp_c_max']}")
    return res


def main():
    os.makedirs(DATA, exist_ok=True); start_proxy()
    original_cap = current_cap()   # put back whatever the running service had, never a fixed number
    pw = Power(); pw.on_abort = lambda: set_cap(original_cap); pw.start()
    n_view = int(sys.argv[4]) if PHASE == 'multi' else 1
    viewers = [Viewer() for _ in range(n_view)]; [v.start() for v in viewers]; radio = Radio(viewers); radio.start(); time.sleep(2)
    log(f'campaign {PHASE}/{TAG}: gates {SAFE}; gateway {GATEWAY}; throttle baseline 0x{pw.thr0:x}')
    try:
        if PHASE == 'ladder':
            secs = int(sys.argv[4]) if len(sys.argv) > 4 else 90
            for cap in [int(x) for x in sys.argv[3].split(',')]:
                if pw.abort: break
                run_step(pw, viewers, radio, f'{TAG}_cap{cap}', cap, secs)
        elif PHASE == 'multi':
            cap = int(sys.argv[3]); secs = int(sys.argv[5]) if len(sys.argv) > 5 else 120
            run_step(pw, viewers, radio, f'{TAG}_{n_view}viewers_cap{cap}', cap, secs)
        elif PHASE == 'link':
            cap = int(sys.argv[3]); secs = int(sys.argv[5]) if len(sys.argv) > 5 else 60
            for mb in [float(x) for x in sys.argv[4].split(',')]:
                if pw.abort: break
                run_step(pw, viewers, radio, f'{TAG}_link{mb:g}_cap{cap}', cap, secs, link_mbit=mb)
        elif PHASE == 'soak':
            cap = int(sys.argv[3]); run_step(pw, viewers, radio, f'{TAG}_soak_cap{cap}', cap, int(float(sys.argv[4]) * 60), snap_every=60)
    finally:
        radio.stop_flag = True; [setattr(v, 'stop_flag', True) for v in viewers]
        set_cap(original_cap)
        pw.stop_flag = True; time.sleep(1); log(f'done; abort={pw.abort}; cap restored to {original_cap}')


if __name__ == '__main__':
    main()
