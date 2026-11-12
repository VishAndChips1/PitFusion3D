"""Shared helpers for the final measurement campaign (2026-11-12)."""
import os, re, json, time, subprocess, threading, csv, glob
import psutil, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
DATA = f'{HERE}/data'; FIG = f'{HERE}/figures'; LOGD = f'{HERE}/logs'
ROS = 'source /opt/ros/jazzy/setup.bash; source /home/vish/ros2_ws/install/setup.bash; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/vish/ros2_ws/install/pitfusion3d_bringup/share/pitfusion3d_bringup/config/fastdds_large_buffers.xml; '
SAFE = dict(ext5v_min=4.75, est_in_max=4.4, temp_max=82.0)   # est_in = sum(rail V*I)/EXT5V ; the user's hard limit is 5 A input
NODE_PATTERNS = {'livox_driver': 'livox_ros2_avia_node', 'restamp': 'livox_restamp_node', 'rgb_camera': 'rgb_camera_node',
                 'thermal_camera': 'thermal_camera_node', 'calibration': 'calibration_publisher.py', 'fusion': 'sensor_fusion_node.py',
                 'merge': 'registration_merge_node.py', 'streamer': 'fusion_web_streamer.py', 'rpicam_vid': 'rpicam-vid'}

def now(): return time.strftime('%Y-%m-%d %H:%M:%S')
def log(msg, name='campaign'):
    line = f'[{time.strftime("%H:%M:%S")}] {msg}'; print(line, flush=True)
    with open(f'{LOGD}/{name}.log', 'a') as f: f.write(line + '\n')
def sh(cmd, timeout=60):
    try: return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout, executable='/bin/bash')
    except subprocess.TimeoutExpired as e:
        dec = lambda b: b.decode('utf-8', 'replace') if isinstance(b, bytes) else (b or '')
        return subprocess.CompletedProcess(cmd, 124, dec(e.stdout), 'TIMEOUT')
def ros_py(script, args='', timeout=120):
    """Run a helper script in a fresh process with ROS sourced (fresh rclpy context per measurement)."""
    r = sh(f'{ROS} python3 {HERE}/{script} {args}', timeout=timeout); return r
def save_json(name, obj):
    with open(f'{DATA}/{name}.json', 'w') as f: json.dump(obj, f, indent=1, default=str)
def load_json(name):
    p = f'{DATA}/{name}.json'; return json.load(open(p)) if os.path.exists(p) else None
def mark(phase, event, **kw):
    with open(f'{DATA}/markers.jsonl', 'a') as f: f.write(json.dumps(dict(t=time.time(), time=now(), phase=phase, event=event, **kw)) + '\n')
def camera_state():
    """Frames per second on the thermal topic over 4 s + whether the device is enumerated."""
    r = ros_py('probe_topics.py', '--topics /landfill/thermal/image_raw --secs 4', 30)
    try: hz = json.loads(r.stdout.strip().splitlines()[-1])['/landfill/thermal/image_raw']['hz']
    except Exception: hz = None
    usb = 'Thermal Cam' in sh('lsusb').stdout
    return dict(thermal_hz=hz, usb_present=usb)

PMIC_RE = re.compile(r'(\S+?)_([AV]) (?:current|volt)\(\d+\)=([\d.]+)')
def pmic():
    out = subprocess.run(['vcgencmd', 'pmic_read_adc'], capture_output=True, text=True).stdout; d = {}
    for m in PMIC_RE.finditer(out): d[(m.group(1), m.group(2))] = float(m.group(3))
    return d
def throttled():
    m = re.search(r'0x([0-9a-fA-F]+)', subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True).stdout)
    return int(m.group(1), 16) if m else -1
def soc_temp(): return float(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000
def est_input(d):
    ext = d.get(('EXT5V', 'V')) or 5.0
    watts = sum(d[(r, 'V')] * d[(r, 'A')] for r in {k[0] for k in d} if (r, 'V') in d and (r, 'A') in d and r not in ('EXT5V', 'BATT'))
    return watts / ext, watts / ext / 0.85 + 0.5, watts     # nominal, conservative (85% regulator eff + 0.5 A USB allowance), watts

def fix_trailing_newline(path):
    """After a power cut the last CSV row may be truncated mid-line: terminate it so appended rows start cleanly."""
    if os.path.exists(path) and os.path.getsize(path) > 0:
        with open(path, 'rb+') as f:
            f.seek(-1, 2)
            if f.read(1) != b'\n': f.write(b'\n')

class PowerSampler(threading.Thread):
    """Continuous fast sampler of every PMIC rail (~30 Hz) with hard safety gates. Writes data/power_raw.csv."""
    def __init__(self, path=None):
        super().__init__(daemon=True); self.path = path or f'{DATA}/power_raw.csv'; self.paused = False; self.stop_flag = False
        self.abort_reason = None; self.latest = {}; self.n = 0; self.phase = 'init'
    def run(self):
        fix_trailing_newline(self.path); existed = os.path.exists(self.path) and os.path.getsize(self.path) > 0; f = open(self.path, 'a', newline=''); w = None; last_temp = 0; temp = 0.0; last_thr = 0; thr = 0
        while not self.stop_flag:
            if self.paused: time.sleep(0.2); continue
            t = time.time(); d = pmic()
            if not d: time.sleep(0.1); continue
            if t - last_temp > 1.0: temp = soc_temp(); last_temp = t
            if t - last_thr > 2.0: thr = throttled(); last_thr = t
            nom, cons, watts = est_input(d)
            if w is None:
                keys = sorted(d); w = csv.writer(f)
                if not existed: w.writerow(['t', 'phase', 'est_in_a', 'est_in_cons_a', 'watts', 'temp_c', 'throttled'] + [f'{k[0]}_{k[1]}' for k in keys])
                self.keys = keys
            w.writerow([f'{t:.3f}', self.phase, f'{nom:.4f}', f'{cons:.4f}', f'{watts:.4f}', f'{temp:.1f}', thr] + [d.get(k, '') for k in self.keys])
            self.n += 1
            if self.n % 100 == 0: f.flush()
            self.latest = dict(t=t, est=nom, cons=cons, ext5v=d.get(('EXT5V', 'V')), temp=temp, thr=thr)
            if (d.get(('EXT5V', 'V')) or 5) < SAFE['ext5v_min'] or nom > SAFE['est_in_max'] or temp > SAFE['temp_max']:
                self.abort_reason = f'SAFETY: ext5v={d.get(("EXT5V","V"))} est_in={nom:.2f} temp={temp:.1f}'
                if t - getattr(self, '_last_log', 0) > 5: self._last_log = t; log(self.abort_reason)
        f.close()

class SystemSampler(threading.Thread):
    """1 Hz sampler: per-process CPU/RSS/threads, per-thread CPU, per-core load, memory, freq, temps."""
    def __init__(self, period=1.0):
        super().__init__(daemon=True); self.period = period; self.stop_flag = False; self.phase = 'init'
    def net(self):
        try:
            u = [l.split() for l in open('/proc/net/snmp') if l.startswith('Udp:')][1]; udp_in, rcv = u[1], u[5]
            def rd(i, f): return open(f'/sys/class/net/{i}/statistics/{f}').read().strip()
            return [udp_in, rcv, rd('eth0', 'rx_packets'), rd('eth0', 'rx_dropped'), rd('wlan0', 'rx_bytes'), rd('wlan0', 'tx_bytes')]
        except Exception: return [''] * 6
    def others(self, procs):
        """CPU used by processes that are NOT part of the pipeline (IDE, desktop, system services), sampled every 5 s."""
        t = time.time()
        if t - getattr(self, '_ot', 0) < 5: return getattr(self, '_ov', ['', '', ''])
        self._ot = t; mine = {p.pid for p in procs.values()}; tot = 0.0; top = ('', 0.0)
        for p in psutil.process_iter(['pid', 'name']):
            if p.info['pid'] in mine: continue
            try: c = p.cpu_percent(None)
            except psutil.Error: continue
            tot += c
            if c > top[1]: top = (p.info['name'], c)
        self._ov = [f'{tot:.0f}', top[0], f'{top[1]:.0f}']; return self._ov
    def find(self):
        out = {}
        for p in psutil.process_iter(['pid', 'cmdline']):
            cl = ' '.join(p.info['cmdline'] or [])
            for k, pat in NODE_PATTERNS.items():
                if pat in cl and 'sampler' not in cl and 'campaign' not in cl and not cl.startswith(('bash', '/bin/bash', 'sh ')) and k not in out:
                    out[k] = p
        return out
    def run(self):
        fix_trailing_newline(f'{DATA}/system_1hz.csv'); existed = os.path.exists(f'{DATA}/system_1hz.csv') and os.path.getsize(f'{DATA}/system_1hz.csv') > 0; f = open(f'{DATA}/system_1hz.csv', 'a', newline=''); w = csv.writer(f); procs = {}
        if not existed: w.writerow(['t', 'phase', 'cpu_total_pct', 'load1', 'mem_used_mb', 'mem_avail_mb', 'swap_used_mb', 'shm_mb', 'arm_clock_mhz', 'core_clock_mhz', 'gpu_temp_c', 'cpu_c0', 'cpu_c1', 'cpu_c2', 'cpu_c3', 'udp_in', 'udp_rcvbuf_err', 'eth0_rx_pkts', 'eth0_rx_drop', 'wlan0_rx_bytes', 'wlan0_tx_bytes', 'other_cpu_pct', 'top_other', 'top_other_cpu'] +
                   [f'{k}_{m}' for k in NODE_PATTERNS for m in ('cpu', 'rss_mb', 'threads')])
        psutil.cpu_percent(percpu=True); n = 0; last_find = 0
        while not self.stop_flag:
            t0 = time.time()
            if t0 - last_find > 10 or not procs: procs = self.find(); last_find = t0
            cores = psutil.cpu_percent(percpu=True); vm = psutil.virtual_memory(); sw = psutil.swap_memory()
            shm = sum(os.path.getsize(p) for p in glob.glob('/dev/shm/*') if os.path.isfile(p)) / 1e6
            clk = lambda c: (re.search(r'=(\d+)', sh(f'vcgencmd measure_clock {c}').stdout) or [0, 0])[1]
            row = [f'{t0:.2f}', self.phase, f'{sum(cores):.1f}', f'{os.getloadavg()[0]:.2f}', f'{vm.used/1e6:.0f}', f'{vm.available/1e6:.0f}', f'{sw.used/1e6:.0f}', f'{shm:.0f}',
                   f'{int(clk("arm"))/1e6:.0f}', f'{int(clk("core"))/1e6:.0f}', '', *[f'{c:.0f}' for c in cores], *self.net(), *self.others(procs)]
            for k in NODE_PATTERNS:
                p = procs.get(k)
                try: row += [f'{p.cpu_percent(None):.1f}', f'{p.memory_info().rss/1e6:.0f}', p.num_threads()] if p else ['', '', '']
                except psutil.Error: row += ['', '', '']
            w.writerow(row); n += 1
            if n % 10 == 0: f.flush()
            time.sleep(max(0, self.period - (time.time() - t0)))
        f.close()
