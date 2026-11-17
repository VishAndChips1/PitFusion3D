#!/usr/bin/env python3
"""TestWithLidV1: unattended field test of the PitFusion3D rig on its own WiFi hotspot.

Start it with run_test.sh. It waits through a countdown (default 120 s), switches the Pi's WiFi to the PitFusion3D-AP hotspot,
restarts the pipeline on it, then runs every phase below on its own, writes all raw data into
~/TestWithLidV1/results/<run_id>/tests_final/ and finishes by writing RESULTS.md, thesis_numbers.json and a zip.

Every phase is fenced: an exception or timeout in one phase is logged and the next phase still runs.
See README.md for the full description.
"""
import argparse
import csv
import glob
import gzip
import http.client
import json
import multiprocessing
import os
import re
import shutil
import signal
import statistics
import struct
import subprocess
import sys
import threading
import time
import traceback
import zipfile
import zlib
from collections import deque
from pathlib import Path

import numpy as np
import psutil

HOME = Path.home()
HERE = Path(__file__).resolve().parent
PIPE_PROCS = ['livox_ros2_avia_node', 'livox_restamp_node', 'calibration_publisher', 'rgb_camera_node', 'thermal_camera_node',
              'sensor_fusion_node', 'registration_merge_node', 'fusion_web_streamer']
UNIT = 'pitfusion3d.service'
AP_PROFILE = 'PitFusion3D-AP'
PORT = 8080

# values from the previous campaign (thesis plan, section 17) for the "previous" column of thesis_numbers.json
PREVIOUS = {
    'rate_lidar_hz': 10.0, 'lidar_scans_lost_pct': 0, 'rate_rgb_fps': 10, 'rate_thermal_fps': 28.8, 'rate_fused_hz': '9.8-10', 'rate_merged_hz': 1.5,
    'merged_points_before_cap': 108500, 'fusion_ms_total': 17, 'projection_ms': 5, 'colouring_ms': 7, 'icp_ms_grid01': 58, 'icp_ms_grid02': 47,
    'merge_window11_s': 1.4, 'age_live_p50_ms': 81, 'age_live_p95_ms': 137, 'age_merged_p50_ms': 467, 'age_merged_p95_ms': 638,
    'merged_content_age_s': 0.77, 'pipeline_cpu_pct_of_core': 94, 'pipeline_mem_mb': 1548, 'soak_mem_slope_mb_per_min': 0,
    'stress_fused_hz_one_core_busy': 9.7, 'sweep_merged_cov5cm_20k': 62, 'sweep_merged_cov5cm_60k': 78, 'sweep_merged_mbit_20k': 2.0,
    'sweep_merged_mbit_60k': 6.1, 'sweep_live_cov5cm_5k': 37, 'sweep_live_cov5cm_25k': 66, 'drift_ppm': '7.2 and 15.9',
    'gap_rgb_p50_ms': 12, 'gap_rgb_p95_ms': 15, 'gap_thermal_p50_ms': 5, 'lidar_precision_mm': 16.3, 'lidar_warmup_drift_cm': '3-4',
    'icp_error_mm_grid01': 18, 'icp_error_mm_grid02': 32, 'icp_envelope_cm': 5, 'icp_envelope_deg': 0.5, 'current_peak_a': 2.02,
    'current_cons_peak_a': 2.88, 'current_mean_a': 0.82, 'power_mean_w': 4.1, 'ext5v_min_v': 4.84, 'storage_free_gb': 70, 'sd_write_mb_s': 53,
    'gbh_fused': 20.7, 'gbh_merged_compact': 4.9, 'gbh_raw_lidar': 15.6, 'gbh_raw_rgb': 99.5, 'gbh_raw_thermal': 70.8, 'raw_all_mb_s': 52,
    'raw_fill_minutes': 23, 'temp_normal_steady_c': '66-70 (open enclosure)', 'temp_overload_time_to_80_min': 13, 'temp_overload_steady_c': '81-83',
    'temp_overload_peak_c': 84.2, 'overload_slowdown_pct': '3-8', 'boot_os_s': 13.8, 'boot_fused_s': 21.8, 'boot_merged_s': 25.3, 'sw_restart_s': '12-14',
    'node_restart_s': 2, 'corner_cases_passed': 42, 'corner_cases_total': 43, 'lidar_standby_s': 0.9, 'viewer_mbit_per_viewer': 2.0,
    'viewers_16_total_mbit': 26, 'viewers_16_resp_ms': 50, 'rgb_calib_rms_px': 0.33, 'thermal_lens_mm': None,
}
ALL_KEYS = list(dict.fromkeys(list(PREVIOUS) + [
    'lidar_scans_lost_pct', 'boot_lidar_s', 'undervoltage_events', 'battery_runtime_h_measured', 'battery_runtime_h_standby_measured',
    'ambient_c', 'thermal_cpu_cost_pct_core', 'coverage_rgb_pct', 'coverage_thermal_pct', 'coverage_both_pct', 'hours_to_exceed_tolerance',
    'wifi_usable_range_m', 'stopwatch_delay_ms', 'mass_kg', 'dimensions_mm', 'accuracy_vs_range']))

ALL_PHASES = ['meta', 'normal_use', 'rates', 'stage_timing', 'freshness', 'cpu_mem', 'tradeoffs', 'rgb_fps', 'accuracy', 'storage', 'startup',
              'wireless', 'fusion_info', 'overload']
SECTION = {'meta': '00_meta', 'normal_use': '12_thermal_enclosure', 'rates': '02_rates', 'stage_timing': '03_timing_stages', 'freshness': '04_freshness',
           'cpu_mem': '05_cpu_memory', 'tradeoffs': '06_tradeoffs', 'rgb_fps': '06_tradeoffs', 'accuracy': '08_accuracy', 'storage': '11_storage',
           'startup': '13_startup_reliability', 'wireless': '14_wireless', 'fusion_info': '15_fusion_info', 'overload': '12_thermal_enclosure'}
EST_MIN = {'meta': 1, 'normal_use': 60, 'rates': 13, 'stage_timing': 6, 'freshness': 13, 'cpu_mem': 17, 'tradeoffs': 19, 'rgb_fps': 8, 'accuracy': 2,
           'storage': 5, 'startup': 25, 'wireless': 6, 'fusion_info': 5, 'overload': 55}


class PhaseTimeout(Exception):
    pass


class Stop(Exception):
    pass


class State:
    args = None
    out = None
    t_start = time.time()
    scale = 1.0
    phase = 'starting'
    phase_idx = 0
    phases = []
    status_log = []
    gate_event = threading.Event()
    gate_reason = None
    gate_temp = 85.0
    expect_throttle = False
    power = deque(maxlen=40000)
    stop_flag = False
    hotspot = {'requested': False, 'status': 'not_requested'}
    numbers = {}
    summaries = {}
    probe_proc = None
    probe_status_path = None
    threads = []
    markers_path = None
    ap_clients = 0


S = State()


# ------------------------------------------------------------------ small utilities
def log(msg):
    line = f'[{time.strftime("%Y-%m-%d %H:%M:%S")}] {msg}'
    print(line, flush=True)
    if S.out:
        with open(S.out / 'RUN_LOG.txt', 'a') as fh:
            fh.write(line + '\n')


def sh(cmd, timeout=60, env=None):
    try:
        r = subprocess.run(cmd, shell=isinstance(cmd, str), capture_output=True, text=True, timeout=timeout, env=env)
        return r.returncode, (r.stdout + r.stderr).strip()
    except subprocess.TimeoutExpired:
        return 124, 'timeout'
    except OSError as exc:
        return 127, str(exc)


def P(section, name=''):
    d = S.out / section
    d.mkdir(parents=True, exist_ok=True)
    return d / name if name else d


def jwrite(path, obj):
    with open(path, 'w') as fh:
        json.dump(obj, fh, indent=1, default=str)


def write_csv(path, header, rows):
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)


def mark(name, **extra):
    with open(S.markers_path, 'a') as fh:
        fh.write(json.dumps({'t': time.time(), 'marker': name, **extra}) + '\n')


def D(seconds, floor=10.0):
    """Scale a duration; never below `floor` seconds."""
    return max(float(floor), seconds * S.scale)


def num(key, value, unit='', source='', note=''):
    S.numbers[key] = {'value': value, 'unit': unit, 'previous': PREVIOUS.get(key), 'source_file': source, 'note': note}


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else None


def pstats(a):
    a = np.asarray(a, dtype=float)
    if not len(a):
        return {'n': 0}
    return {'n': int(len(a)), 'mean': float(a.mean()), 'median': float(np.median(a)), 'p95': float(np.percentile(a, 95)), 'max': float(a.max()), 'min': float(a.min()),
            'std': float(a.std())}


def soc_temp():
    try:
        return float(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000.0
    except OSError:
        return float('nan')


def throttled_flags():
    rc, out = sh(['vcgencmd', 'get_throttled'], 5)
    m = re.search(r'0x([0-9a-fA-F]+)', out)
    return int(m.group(1), 16) if m else -1


def fan_rpm():
    for f in glob.glob('/sys/class/hwmon/hwmon*/fan1_input'):
        try:
            return int(open(f).read())
        except (OSError, ValueError):
            pass
    return ''


def check_stop():
    if S.stop_flag:
        raise Stop()


def sleep_checked(seconds, gate=True):
    """Sleep in 1 s steps; return early (False) if a safety gate trips or a stop is requested."""
    end = time.time() + seconds
    while time.time() < end:
        check_stop()
        if gate and S.gate_event.is_set():
            return False
        time.sleep(min(1.0, max(0.0, end - time.time())))
    return True


def cooldown(target=68.0, max_s=900):
    """After a gate trip (or hot phase): wait until the SoC is below target."""
    t0 = time.time()
    while soc_temp() > target and time.time() - t0 < max_s:
        check_stop()
        time.sleep(5)
    S.gate_event.clear()
    S.gate_reason = None
    log(f'cooldown finished after {time.time() - t0:.0f} s at {soc_temp():.1f} C')


# ------------------------------------------------------------------ background samplers
def pmic():
    rc, out = sh(['vcgencmd', 'pmic_read_adc'], 5)
    d = {}
    for m in re.finditer(r'(\S+?)_([AV]) (?:current|volt)\(\d+\)=([\d.]+)', out):
        d[(m.group(1), m.group(2))] = float(m.group(3))
    return d


class Resilient(threading.Thread):
    """Thread whose loop is restarted after an unexpected exception."""

    def __init__(self):
        super().__init__(daemon=True)
        self.stop_flag = False

    def run(self):
        while not self.stop_flag:
            try:
                self.loop()
            except Exception as exc:
                log(f'{type(self).__name__} error: {exc!r}: restarting its loop')
                time.sleep(1)


class PowerThread(Resilient):
    """~30 Hz PMIC sampling, one CSV, and the safety gates (the thesis plan's section 0.2)."""

    def loop(self):
        f = open(P('09_power', 'power_raw.csv'), 'a', newline='')
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow(['t', 'ext5v_v', 'est_input_a', 'est_input_cons_a', 'total_w', 'vdd_core_a', 'vdd_core_v', 'wifi_rail_a', 'soc_c'])
        low, n, last_thr = 0, 0, 0.0
        while not self.stop_flag:
            d = pmic()
            t = time.time()
            e = d.get(('EXT5V', 'V'), float('nan'))
            tw = sum(d[(b, 'V')] * d[(b, 'A')] for (b, k) in list(d) if k == 'V' and (b, 'A') in d)
            est = tw / e if e == e and e > 0 else float('nan')
            cons = tw / (e * 0.85) + 0.5 if e == e and e > 0 else float('nan')
            tc = soc_temp()
            row = (t, e, est, cons, tw, d.get(('VDD_CORE', 'A'), float('nan')), d.get(('VDD_CORE', 'V'), float('nan')), d.get(('3V7_WL_SW', 'A'), float('nan')), tc)
            S.power.append(row)
            w.writerow([f'{x:.4f}' for x in row])
            n += 1
            if n % 60 == 0:
                f.flush()
            low = low + 1 if e == e and e < 4.75 else 0
            why = None
            if low >= 2:
                why = f'EXT5V {e:.3f} V < 4.75 V'
            elif est == est and est > 4.5:
                why = f'estimated input current {est:.2f} A > 4.5 A'
            elif tc == tc and tc >= S.gate_temp:
                why = f'SoC {tc:.1f} C >= {S.gate_temp:.0f} C'
            if not why and t - last_thr > 1.0:
                last_thr = t
                th = throttled_flags()
                if th != -1 and (th & 0x1 or (not S.expect_throttle and th & 0x4)):
                    why = f'unexpected power/throttle flag 0x{th:x}'
            if why and not S.gate_event.is_set():
                S.gate_reason = why
                S.gate_event.set()
                log(f'!!! SAFETY GATE: {why}')
                with open(P('00_meta', 'gate_events.jsonl'), 'a') as gf:
                    gf.write(json.dumps({'t': t, 'phase': S.phase, 'reason': why}) + '\n')
        f.close()


class SystemSampler(Resilient):
    """1 Hz: temperatures, clocks, throttle flags, per-core and per-process CPU/RSS/threads, memory, wlan counters, clients."""

    def __init__(self):
        super().__init__()
        self.procs = {}
        self.thread_last = {}

    def find(self):
        for p in psutil.process_iter(['pid', 'cmdline']):
            cl = p.info['cmdline'] or []
            if '--ros-args' in cl:
                for name in PIPE_PROCS:
                    if any(name in tok for tok in cl[:3]) and (name not in self.procs or self.procs[name].pid != p.info['pid']):
                        try:
                            self.procs[name] = psutil.Process(p.info['pid'])
                            self.procs[name].cpu_percent(None)
                        except psutil.Error:
                            pass

    def loop(self):
        cols = (['t', 'soc_c', 'arm_mhz', 'throttled_hex', 'load1', 'mem_available_mb', 'swap_used_mb', 'cpu_total_pct', 'cpu0', 'cpu1', 'cpu2', 'cpu3',
                 'pipeline_cpu_pct_of_core', 'pipeline_rss_mb', 'busiest_thread_pct', 'busiest_thread_proc', 'fan_rpm', 'external_clients_8080',
                 'ap_stations', 'wlan_tx_bytes', 'wlan_rx_bytes', 'wlan_tx_errors', 'wlan_tx_dropped']
                + [f'{n}_{k}' for n in PIPE_PROCS for k in ('cpu', 'rss_mb', 'threads')])
        f = open(P('00_meta', 'system_1hz.csv'), 'a', newline='')
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow(cols)
        psutil.cpu_percent(percpu=True)
        last = time.time()
        while not self.stop_flag:
            time.sleep(max(0.05, 1.0 - (time.time() - last)))
            now = time.time()
            dt = now - last
            last = now
            self.find()
            percore = psutil.cpu_percent(percpu=True)
            row = {'t': f'{now:.2f}', 'soc_c': f'{soc_temp():.1f}', 'throttled_hex': hex(max(throttled_flags(), 0)), 'load1': os.getloadavg()[0],
                   'mem_available_mb': f'{psutil.virtual_memory().available / 1e6:.0f}', 'swap_used_mb': f'{psutil.swap_memory().used / 1e6:.0f}',
                   'cpu_total_pct': f'{sum(percore):.1f}', 'fan_rpm': fan_rpm(), 'ap_stations': S.ap_clients}
            for i in range(4):
                row[f'cpu{i}'] = percore[i] if i < len(percore) else ''
            rc, out = sh(['vcgencmd', 'measure_clock', 'arm'], 3)
            m = re.search(r'=(\d+)', out)
            row['arm_mhz'] = int(m.group(1)) // 1000000 if m else ''
            tot_cpu = tot_rss = 0.0
            busiest, busiest_proc = 0.0, ''
            for name in PIPE_PROCS:
                p = self.procs.get(name)
                try:
                    cpu = p.cpu_percent(None)
                    rss = p.memory_info().rss / 1e6
                    th = p.threads()
                    for t_ in th:
                        key = (p.pid, t_.id)
                        cur = t_.user_time + t_.system_time
                        if key in self.thread_last and dt > 0:
                            pcent = 100.0 * (cur - self.thread_last[key]) / dt
                            if pcent > busiest:
                                busiest, busiest_proc = pcent, name
                        self.thread_last[key] = cur
                    row[f'{name}_cpu'], row[f'{name}_rss_mb'], row[f'{name}_threads'] = f'{cpu:.1f}', f'{rss:.0f}', len(th)
                    tot_cpu += cpu
                    tot_rss += rss
                except (psutil.Error, AttributeError):
                    row[f'{name}_cpu'], row[f'{name}_rss_mb'], row[f'{name}_threads'] = '', '', ''
            row['pipeline_cpu_pct_of_core'], row['pipeline_rss_mb'] = f'{tot_cpu:.1f}', f'{tot_rss:.0f}'
            row['busiest_thread_pct'], row['busiest_thread_proc'] = f'{busiest:.1f}', busiest_proc
            try:
                net = psutil.net_io_counters(pernic=True).get('wlan0')
                if net:
                    row['wlan_tx_bytes'], row['wlan_rx_bytes'] = net.bytes_sent, net.bytes_recv
                    row['wlan_tx_errors'], row['wlan_tx_dropped'] = net.errout, net.dropout
            except Exception:
                pass
            row['external_clients_8080'] = len(external_clients())
            w.writerow([row.get(c, '') for c in cols])
            f.flush()
        f.close()


def external_clients(ports=(8080, 8081)):
    """TCP connections to the rig's ports from other machines (the real base station)."""
    rc, out = sh("ss -tn state established '( sport = :8080 or sport = :8081 )'", 5)
    peers = []
    for line in out.splitlines()[1:]:
        f = line.split()
        if len(f) >= 4 and not f[3].startswith(('127.', '[::1]', '::ffff:127.')):
            peers.append((f[3].rsplit(':', 1)[0], f[2].rsplit(':', 1)[1]))
    return peers


class WifiMonitor(Resilient):
    """AP-side view of the real client: link counters, stations, per-client TCP stats (rtt, retransmits, goodput), pings, server counters."""

    def stations(self):
        rc, out = sh('ip neigh show dev wlan0', 5)
        ips = []
        for line in out.splitlines():
            f = line.split()
            if f and f[-1] in ('REACHABLE', 'STALE', 'DELAY', 'PROBE') and ':' not in f[0]:
                ips.append(f[0])
        return ips

    def loop(self):
        link = open(P('14_wireless', 'wifi_link.csv'), 'a', newline='')
        tcp = open(P('14_wireless', 'wifi_clients_tcp.csv'), 'a', newline='')
        ping = open(P('14_wireless', 'wifi_ping.csv'), 'a', newline='')
        stats = open(P('14_wireless', 'server_stats.csv'), 'a', newline='')
        wl, wt, wp, ws = csv.writer(link), csv.writer(tcp), csv.writer(ping), csv.writer(stats)
        if link.tell() == 0:
            wl.writerow(['t', 'mode', 'frequency', 'tx_power_dbm', 'ap_stations', 'wlan_tx_bytes', 'wlan_rx_bytes', 'wlan_tx_errors', 'wlan_tx_dropped'])
            wt.writerow(['t', 'peer_ip', 'local_port', 'send_q_bytes', 'rtt_ms', 'rttvar_ms', 'cwnd', 'retrans_current', 'retrans_total', 'bytes_acked', 'delivery_rate_mbps'])
            wp.writerow(['t', 'target', 'rtt_ms'])
            ws.writerow(['t', 'requests', 'sent_200', 'not_modified_304', 'aborted_by_client', 'bytes_sent', 'write_seconds', 'write_seconds_max', 'format2'])
        tick = 0
        while not self.stop_flag:
            t0 = time.time()
            tick += 1
            try:
                rc, iw = sh(['iwconfig', 'wlan0'], 4)
                mode = re.search(r'Mode:(\w+)', iw)
                freq = re.search(r'Frequency[:=]([\d.]+)', iw)
                txp = re.search(r'Tx-Power=(-?\d+)', iw)
                sts = self.stations()
                S.ap_clients = len(sts)
                net = psutil.net_io_counters(pernic=True).get('wlan0')
                wl.writerow([f'{t0:.2f}', mode.group(1) if mode else '', freq.group(1) if freq else '', txp.group(1) if txp else '', len(sts),
                             net.bytes_sent if net else '', net.bytes_recv if net else '', net.errout if net else '', net.dropout if net else ''])
                rc, out = sh("ss -tin state established '( sport = :8080 or sport = :8081 )'", 5)
                lines = out.splitlines()[1:]
                i = 0
                while i < len(lines):
                    head = lines[i].split()
                    info = lines[i + 1] if i + 1 < len(lines) else ''
                    i += 2
                    if len(head) < 4 or head[3].startswith(('127.', '[::1]')):
                        continue
                    g = lambda pat: (re.search(pat, info).group(1) if re.search(pat, info) else '')  # noqa: E731
                    rtt = re.search(r'rtt:([\d.]+)/([\d.]+)', info)
                    rt = re.search(r'retrans:(\d+)/(\d+)', info)
                    dr = re.search(r'delivery_rate ([\d.]+)([MKG]?)bps', info)
                    drm = ''
                    if dr:
                        drm = float(dr.group(1)) * {'': 1e-6, 'K': 1e-3, 'M': 1.0, 'G': 1e3}[dr.group(2)]
                    wt.writerow([f'{t0:.2f}', head[3].rsplit(':', 1)[0], head[2].rsplit(':', 1)[1], head[1], rtt.group(1) if rtt else '', rtt.group(2) if rtt else '',
                                 g(r'cwnd:(\d+)'), rt.group(1) if rt else 0, rt.group(2) if rt else 0, g(r'bytes_acked:(\d+)'), drm])
                if tick % 5 == 0:
                    for target in sts[:4]:
                        rc, out = sh(['ping', '-c', '1', '-W', '1', target], 3)
                        m = re.search(r'time=([\d.]+)', out)
                        wp.writerow([f'{time.time():.2f}', target, m.group(1) if m else ''])
                if tick % 10 == 0:
                    try:
                        c = http.client.HTTPConnection('127.0.0.1', PORT, timeout=3)
                        c.request('GET', '/stats')
                        d = json.loads(c.getresponse().read())
                        c.close()
                        ws.writerow([f'{time.time():.2f}'] + [d.get(k, '') for k in ('requests', 'sent_200', 'not_modified_304', 'aborted_by_client', 'bytes_sent', 'write_seconds', 'write_seconds_max', 'format2')])
                    except Exception:
                        pass
                for fh in (link, tcp, ping, stats):
                    fh.flush()
            except Exception as exc:
                log(f'wifi monitor: {exc!r}')
            time.sleep(max(0.05, 1.0 - (time.time() - t0)))
        for fh in (link, tcp, ping, stats):
            fh.close()


# ------------------------------------------------------------------ probe (topic recorder) and topic statistics
def start_probe():
    stop_probe()
    S.probe_status_path = str(P('02_rates', 'probe_status.json'))
    log_f = open(P('02_rates', 'probe.log'), 'a')
    S.probe_proc = subprocess.Popen([sys.executable, str(HERE / 'probe.py'), 'log', str(P('02_rates')), S.probe_status_path], stdout=log_f, stderr=subprocess.STDOUT)
    log(f'probe started (pid {S.probe_proc.pid})')


def stop_probe():
    if S.probe_proc and S.probe_proc.poll() is None:
        S.probe_proc.terminate()
        try:
            S.probe_proc.wait(10)
        except subprocess.TimeoutExpired:
            S.probe_proc.kill()
    S.probe_proc = None


def probe_alive():
    if S.probe_proc is None or S.probe_proc.poll() is not None:
        log('probe died: restarting')
        start_probe()


def probe_status():
    try:
        return json.load(open(S.probe_status_path))
    except Exception:
        return {}


def thermal_status():
    st = probe_status().get('thermal', {})
    hz = st.get('hz_10s', 0) or 0
    return ('streaming' if hz > 20 else 'intermittent' if hz > 0 else 'absent'), hz


_topic_cache = {}


def load_topic(key):
    path = P('02_rates', f'topic_{key}.csv')
    if not path.exists():
        return np.zeros((0, 4))
    sig = (path.stat().st_size, path.stat().st_mtime)
    if _topic_cache.get(key, (None,))[0] == sig:
        return _topic_cache[key][1]
    try:
        a = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    except Exception:
        a = np.zeros((0, 4))
    if a.size == 0:
        a = np.zeros((0, 4))
    _topic_cache[key] = (sig, a)
    return a


def topic_window(key, t0, t1):
    a = load_topic(key)
    return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)] if len(a) else a


def rate_stats(key, t0, t1):
    a = topic_window(key, t0, t1)
    out = {'n': int(len(a))}
    if len(a) >= 2:
        iv = np.diff(a[:, 0]) * 1000.0
        span = a[-1, 0] - a[0, 0]
        out.update({'hz_mean': (len(a) - 1) / span if span > 0 else 0.0, 'interval_p50_ms': float(np.median(iv)), 'interval_p95_ms': float(np.percentile(iv, 95)),
                    'interval_std_ms': float(iv.std()), 'interval_max_ms': float(iv.max()), 'points_per_msg_median': float(np.median(a[:, 2])),
                    'kbytes_per_msg_median': float(np.median(a[:, 3]) / 1000.0), 'mean_msg_bytes': float(a[:, 3].mean())})
    else:
        out.update({'hz_mean': 0.0})
    return out


def hz_between(key, t0, t1):
    a = topic_window(key, t0, t1)
    return len(a) / (t1 - t0) if t1 > t0 else 0.0


def sampler_slice(t0, t1):
    """Rows of system_1hz.csv between t0 and t1 as a dict of numpy columns."""
    path = P('00_meta', 'system_1hz.csv')
    rows = []
    with open(path) as fh:
        r = csv.DictReader(fh)
        for row in r:
            try:
                if t0 <= float(row['t']) <= t1:
                    rows.append(row)
            except (ValueError, KeyError):
                pass
    cols = {}
    for k in (rows[0].keys() if rows else []):
        try:
            cols[k] = np.array([float(r[k]) if r[k] != '' else np.nan for r in rows])
        except ValueError:
            cols[k] = [r[k] for r in rows]
    return cols


def power_slice(t0, t1):
    rows = [r for r in list(S.power) if t0 <= r[0] <= t1]
    return np.array(rows) if rows else np.zeros((0, 9))


def power_summary(t0, t1):
    a = power_slice(t0, t1)
    if not len(a):
        return {}
    return {'est_input_a_mean': float(np.nanmean(a[:, 2])), 'est_input_a_p99': float(np.nanpercentile(a[:, 2], 99)), 'est_input_a_max': float(np.nanmax(a[:, 2])),
            'cons_input_a_max': float(np.nanmax(a[:, 3])), 'power_w_mean': float(np.nanmean(a[:, 4])), 'ext5v_min': float(np.nanmin(a[:, 1])),
            'soc_c_max': float(np.nanmax(a[:, 8])), 'n_samples': int(len(a))}


# ------------------------------------------------------------------ talking to the pipeline
def http_get(path, timeout=6, headers=None):
    c = http.client.HTTPConnection('127.0.0.1', PORT, timeout=timeout)
    try:
        c.request('GET', path, headers=headers or {})
        r = c.getresponse()
        return r.status, r.read(), dict(r.getheaders())
    finally:
        c.close()


def cloud_points(path='/merged.bin'):
    try:
        st, body, _ = http_get(path, 6, {'Accept-Encoding': 'gzip'})
        if st != 200 or len(body) < 10:
            return 0
        return struct.unpack_from('<I', zlib.decompressobj(31).decompress(body, 16), 0)[0]
    except Exception:
        return 0


def wait_ready(timeout=240):
    t0 = time.time()
    while time.time() - t0 < timeout:
        check_stop()
        if cloud_points() > 1000:
            return time.time() - t0
        time.sleep(2)
    return None


def set_cap(n, which='merged'):
    for attempt in range(4):
        rc, out = sh(['ros2', 'param', 'set', '/fusion_web_streamer', f'max_points_{which}', str(n)], 25)
        if 'successful' in out:
            return True
        time.sleep(3)
    return False


def get_cap(which='merged'):
    for _ in range(3):
        rc, out = sh(['ros2', 'param', 'get', '/fusion_web_streamer', f'max_points_{which}'], 25)
        m = re.search(r'(\d+)\s*$', out.strip())
        if m:
            return int(m.group(1))
        time.sleep(2)
    return None


def restart_pipeline():
    sh(['systemctl', '--user', 'restart', UNIT], 90)
    return wait_ready(240)


def pipeline_pids():
    out = {}
    for p in psutil.process_iter(['pid', 'cmdline']):
        cl = p.info['cmdline'] or []
        if '--ros-args' in cl:
            for name in PIPE_PROCS:
                if any(name in tok for tok in cl[:3]):
                    out.setdefault(name, []).append(p.info['pid'])
    return out


class ViewerSim(threading.Thread):
    """An emulated viewer: conditional GETs on keep-alive, recording arrival, size, response time and data age."""

    def __init__(self, path, interval, keep_last=False, name=''):
        super().__init__(daemon=True)
        self.path, self.interval, self.keep_last, self.name = path, interval, keep_last, name or path
        self.stop_flag = False
        self.rows = []              # (t_recv, resp_ms, nbytes, stamp, age_ms, n_points)
        self.errors = 0
        self.last_payload = None

    def run(self):
        conn, etag = None, None
        while not self.stop_flag:
            t = time.time()
            try:
                if conn is None:
                    conn = http.client.HTTPConnection('127.0.0.1', PORT, timeout=10)
                h = {'Accept-Encoding': 'gzip'}
                if etag:
                    h['If-None-Match'] = etag
                conn.request('GET', self.path, headers=h)
                r = conn.getresponse()
                body = r.read()
                if r.status == 200:
                    etag = r.getheader('ETag')
                    now = time.time()
                    stamp = float(r.getheader('X-Cloud-Stamp') or 0)
                    try:
                        n = struct.unpack_from('<I', zlib.decompressobj(31).decompress(body, 16), 0)[0]
                    except Exception:
                        n = 0
                    self.rows.append((now, (now - t) * 1000.0, len(body), stamp, (now - stamp) * 1000.0 if stamp > 1e9 else float('nan'), n))
                    if self.keep_last:
                        self.last_payload = body
            except Exception:
                self.errors += 1
                try:
                    conn.close()
                except Exception:
                    pass
                conn, etag = None, None
            time.sleep(max(0.02, self.interval - (time.time() - t)))

    def window(self, t0, t1):
        return [r for r in self.rows if t0 <= r[0] <= t1]


def start_viewers(merged=True, live=True):
    vs = []
    if merged:
        vs.append(ViewerSim('/merged.bin', 0.25, name='merged'))
    if live:
        vs.append(ViewerSim('/live.bin', 0.12, name='live'))
    for v in vs:
        v.start()
    return vs


def stop_viewers(vs):
    for v in vs:
        v.stop_flag = True


def capture(spec, name, timeout=60):
    out = P('00_meta', f'_capture_{name}.npz')
    rc, text = sh([sys.executable, str(HERE / 'probe.py'), 'capture', str(out), spec, str(timeout)], timeout + 30)
    return out if out.exists() else None


# ------------------------------------------------------------------ hotspot
def hotspot_up():
    S.hotspot = {'requested': True, 'status': 'starting'}
    sh(['systemctl', '--user', 'stop', 'pitfusion3d-wifi.service'], 30)       # it must not fight the access point
    sh(['nmcli', 'radio', 'wifi', 'on'], 15)
    sh(['nmcli', 'connection', 'modify', AP_PROFILE, '802-11-wireless.powersave', '2'], 15)    # WiFi power saving OFF for the hotspot (2 = disable)
    if S.args.ap_band:
        sh(['nmcli', 'connection', 'modify', AP_PROFILE, '802-11-wireless.band', S.args.ap_band], 15)
    if S.args.ap_channel:
        sh(['nmcli', 'connection', 'modify', AP_PROFILE, '802-11-wireless.channel', str(S.args.ap_channel)], 15)
    ok = False
    for attempt in range(1, 6):
        rc, out = sh(['nmcli', 'connection', 'up', AP_PROFILE], 70)
        time.sleep(4)
        rc2, state = sh(['nmcli', '-g', 'GENERAL.STATE', 'device', 'show', 'wlan0'], 10)
        rc3, ip = sh('ip -4 addr show wlan0', 10)
        log(f'hotspot attempt {attempt}: nmcli rc={rc} state="{state.strip()}" ip10.42.0.1={"10.42.0.1" in ip}')
        if state.strip().startswith('100') and '10.42.0.1' in ip:
            ok = True
            break
        time.sleep(8)
    rc, iw = sh(['iwconfig', 'wlan0'], 5)
    log('wifi power management after the switch: ' + ' '.join(re.findall(r'Power Management:\w+', iw)))
    rc, prof = sh(['nmcli', '-t', '-f', 'connection.id,802-11-wireless.ssid,802-11-wireless.band,802-11-wireless.channel,802-11-wireless.mode,ipv4.method,ipv4.addresses', 'connection', 'show', AP_PROFILE], 10)
    S.hotspot = {'requested': True, 'status': 'up' if ok else 'FAILED', 'iwconfig': iw, 'profile': prof, 'time': time.time()}
    jwrite(P('14_wireless', 'hotspot_status.json'), S.hotspot)
    # the existing hotspot script restarts the pipeline after the switch: nodes started on the old interface state can go silent
    if ok or True:
        t = restart_pipeline()
        log(f'pipeline restarted on the new network state: data after {t if t is None else round(t)} s')
    return ok


def hotspot_down():
    sh(['nmcli', 'connection', 'up', 'Patelnet'], 90)
    time.sleep(5)
    restart_pipeline()
    sh(['systemctl', '--user', 'start', 'pitfusion3d-wifi.service'], 30)
    rc, out = sh(['curl', '-sS', '-m', '6', '-o', '/dev/null', '-w', '%{http_code}', 'https://api.anthropic.com'], 15)
    log(f'returned to home WiFi; internet check: {out}')


# ------------------------------------------------------------------ phase runner
def run_phase(name, fn, expected_min):
    S.phase = name
    S.phase_idx += 1
    extra = {'tradeoffs': 1500, 'startup': 2400, 'rgb_fps': 1000, 'stage_timing': 900, 'wireless': 900}.get(name, 600)
    limit = int(expected_min * 60 * max(S.scale, 0.05) * 1.5 + extra)
    t0 = time.time()
    mark(f'{name}_start')
    status = 'ok'
    detail = ''

    def on_alarm(signum, frame):
        raise PhaseTimeout(f'phase {name} exceeded {limit} s')
    signal.signal(signal.SIGALRM, on_alarm)
    signal.alarm(limit)
    log(f'=== PHASE {name} (limit {limit} s) ===')
    kill_stray_workers()
    try:
        fn()
    except Stop:
        status, detail = 'stopped', 'stop requested'
        raise
    except PhaseTimeout as exc:
        status, detail = 'timeout', str(exc)
        log(detail)
    except Exception as exc:
        status, detail = 'failed', f'{exc!r}'
        log(f'PHASE {name} FAILED: {exc!r}\n{traceback.format_exc()}')
    finally:
        signal.alarm(0)
        kill_stray_workers()
        S.expect_throttle = False
        S.gate_temp = 85.0
        mark(f'{name}_end', status=status)
        S.status_log.append({'phase': name, 'status': status, 'detail': detail, 'start': t0, 'end': time.time(), 'minutes': round((time.time() - t0) / 60, 1)})
        jwrite(S.out / 'phase_status.json', S.status_log)
        if S.gate_event.is_set():
            log('a safety gate was tripped in this phase: cooling down before continuing')
            try:
                cooldown()
            except Stop:
                raise
    return status


def write_readme(section, title, lines):
    with open(P(section, 'README.md'), 'a') as fh:
        fh.write(f'## {title}\n\n' + '\n'.join(lines) + '\n\n')


def conditions(t0, t1):
    st, hz = thermal_status()
    return {'t_start_iso': time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(t0)), 't_end_iso': time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(t1)), 'enclosure': S.args.enclosure,
            'power_source': S.args.power_source, 'ambient_c': S.args.ambient, 'thermal_status': st, 'thermal_hz_10s': hz, 'throttled_flags_end': hex(max(throttled_flags(), 0)),
            'hotspot': S.hotspot.get('status'), 'external_clients': len(external_clients()), 'soc_c_end': round(soc_temp(), 1)}


# ================================================================== PHASES
def phase_meta():
    t0 = time.time()
    d = P('00_meta')
    info = {}
    for key, cmd in {'uname': 'uname -a', 'os_release': 'cat /etc/os-release', 'model': 'tr -d "\\0" < /proc/device-tree/model', 'bootloader_config': 'vcgencmd bootloader_config',
                     'bootloader_version': 'vcgencmd bootloader_version', 'throttled': 'vcgencmd get_throttled', 'memory': 'free -m', 'disk': 'df -h /', 'cpuinfo_summary': 'lscpu',
                     'ip_addr': 'ip addr', 'ip_route': 'ip route', 'iwconfig': 'iwconfig wlan0', 'nmcli_devices': 'nmcli device status', 'timesync': 'timedatectl timesync-status',
                     'services': 'systemctl --user list-units --type=service --no-pager', 'pitfusion_unit': 'systemctl --user cat pitfusion3d.service',
                     'wifi_power_management': 'iwconfig wlan0 | grep -i "power management"; nmcli -t -f NAME,TYPE connection show | grep wireless | cut -d: -f1 | while read n; do echo "$n: powersave=$(nmcli -g 802-11-wireless.powersave connection show "$n")"; done', 'cpu_governor': 'cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor', 'usb_autosuspend_delay_s': 'cat /sys/module/usbcore/parameters/autosuspend', 'ros_version': 'ros2 --version || true', 'python': 'python3 --version', 'pip_numpy': 'python3 -c "import numpy,scipy,cv2;print(numpy.__version__,scipy.__version__,cv2.__version__)"',
                     'uptime': 'uptime -p; uptime -s', 'lidar_sleep_mode': 'cat ~/lidar_ctl/sleep_mode', 'wifi_profiles': 'nmcli -t -f NAME,TYPE,AUTOCONNECT connection show'}.items():
        info[key] = sh(cmd, 20)[1]
    for repo in (HOME / 'PitFusion3D_repo', HOME / 'ros2_ws/src'):
        rc, h = sh(f'git -C {repo} rev-parse HEAD 2>/dev/null; git -C {repo} branch --show-current 2>/dev/null; git -C {repo} status --short 2>/dev/null | head -5', 15)
        info[f'git_{repo.name}'] = h
    info['args'] = vars(S.args)
    info['hotspot'] = S.hotspot
    jwrite(d / 'system_inventory.json', info)
    for src in ('landfill_camera_sim/config/calibration/intrinsics.yaml', 'landfill_camera_sim/config/calibration/extrinsics.yaml', 'pitfusion3d_bringup/launch/pitfusion3d_full.launch.py',
                'pitfusion3d_bringup/launch/real_sensors.launch.py', 'landfill_camera_sim/launch/web_stream.launch.py', 'landfill_camera_sim/launch/sensor_fusion.launch.py'):
        f = HOME / 'ros2_ws/src' / src
        if f.exists():
            shutil.copy(f, d / f.name)
    params = {}
    for node in ('/fusion_web_streamer', '/registration_merge_node', '/sensor_fusion_node', '/livox_restamp_node', '/rgb_camera_node', '/thermal_camera_node', '/calibration_publisher'):
        params[node] = sh(['ros2', 'param', 'dump', node], 30)[1]
    jwrite(d / 'node_parameters.json', params)
    cap = capture('caminfo:1', 'caminfo', 20)
    if cap:
        z = np.load(cap, allow_pickle=True)
        if 'caminfo_json' in z.files:
            jwrite(d / 'published_camera_info_rgb.json', json.loads(str(z['caminfo_json'])))
    st, hz = thermal_status()
    jwrite(d / 'thermal_status.json', {'status': st, 'hz_10s': hz, 'watchdog_active': sh(['systemctl', '--user', 'is-active', 'thermal-watchdog.service'], 5)[1],
                                       'note': 'no extra recovery attempted: thermal-watchdog.service is the existing recovery and runs continuously'})
    num('ambient_c', S.args.ambient, 'C', '00_meta/system_inventory.json', 'supplied with --ambient' if S.args.ambient is not None else 'not supplied')
    S.summaries['meta'] = conditions(t0, time.time())
    write_readme('00_meta', 'System inventory, configuration, calibration files, node parameters, thermal status',
                 ['Everything needed to reproduce the setup: system_inventory.json, copied launch/calibration files, node_parameters.json (ros2 param dump), published camera info.',
                  'system_1hz.csv is the master 1 Hz log (temperature, clocks, throttle flags, per-core and per-process CPU/RSS/threads, memory, WiFi counters) for the whole run.',
                  'gate_events.jsonl lists any safety gate that tripped.'])


def phase_normal_use():
    """Cold-ish, 60 min of normal use: heat, soak memory, clock drift, sync gap, LiDAR stability and coverage all come from this one window."""
    t0 = time.time()
    start_temp = soc_temp()
    if S.args.cold_wait_min > 0:
        log(f'waiting {S.args.cold_wait_min} min for the SoC to cool first')
        sleep_checked(S.args.cold_wait_min * 60, gate=False)
        t0 = time.time()
        start_temp = soc_temp()
    vs = start_viewers()
    try:
        sleep_checked(D(3600, 120))
    finally:
        stop_viewers(vs)
    t1 = time.time()
    analyse_normal_use(t0, t1, start_temp)


def analyse_normal_use(t0, t1, start_temp):
    sec = '12_thermal_enclosure'
    s = sampler_slice(t0, t1)
    rows = []
    for i, t in enumerate(s.get('t', [])):
        rows.append([f'{t - t0:.0f}', s['soc_c'][i], s['arm_mhz'][i], s['throttled_hex'][i], s['fan_rpm'][i] if 'fan_rpm' in s else '',
                     f'{hz_between("fused", t - 10, t):.2f}', f'{hz_between("merged", t - 10, t):.2f}'])
    write_csv(P(sec, 'heat_normal.csv'), ['t_s', 'soc_c', 'arm_mhz', 'throttled_hex', 'fan_rpm', 'fused_hz_10s', 'merged_hz_10s'], rows)
    temps = np.asarray(s.get('soc_c', []), dtype=float)
    steady = temps[-max(1, len(temps) // 5):] if len(temps) else temps
    heat = {'start_temp_c': round(start_temp, 1), 'start_was_cold': bool(start_temp < 50), 'peak_c': float(np.nanmax(temps)) if len(temps) else None,
            'steady_c_last_20pct': float(np.nanmean(steady)) if len(steady) else None, 'minutes': round((t1 - t0) / 60, 1),
            'throttle_flags_seen': sorted(set(s.get('throttled_hex', []))), 'fused_hz_mean': hz_between('fused', t0, t1), 'merged_hz_mean': hz_between('merged', t0, t1)}
    num('temp_normal_steady_c', heat['steady_c_last_20pct'], 'C', f'{sec}/heat_normal.csv', f'start {start_temp:.0f} C; cold start {"yes" if start_temp < 50 else "NO (pipeline was already running)"}')
    try:
        # 5c / soak memory
        mem_rows, xs, ys = [], [], []
        t_arr = np.asarray(s.get('t', []), dtype=float)
        rss = np.asarray(s.get('pipeline_rss_mb', []), dtype=float)
        avail = np.asarray(s.get('mem_available_mb', []), dtype=float)
        for i in range(0, len(t_arr), 10):
            mem_rows.append([f'{t_arr[i] - t0:.0f}', rss[i], avail[i]])
        write_csv(P('05_cpu_memory', 'soak_memory.csv'), ['t_s', 'pipeline_rss_mb', 'mem_available_mb'], mem_rows)
        slope = None
        if len(t_arr) > 120:
            k = int(len(t_arr) * 0.1)
            slope = float(np.polyfit((t_arr[k:] - t_arr[k]) / 60.0, rss[k:], 1)[0])
        heat['soak_rss_slope_mb_per_min'] = slope
        num('soak_mem_slope_mb_per_min', slope, 'MB/min', '05_cpu_memory/soak_memory.csv', 'first 10% excluded')
    except Exception as exc:
        log(f'analysis block 0 failed: {exc!r}')
    try:
        # 7.1 clock drift from raw vs re-stamped LiDAR stamps
        raw, rs = topic_window('lidar_raw', t0, t1), topic_window('lidar_restamped', t0, t1)
        drift = {}
        if len(raw) > 200 and len(rs) > 200 and (raw[-1, 0] - raw[0, 0]) > 360:
            off = raw[:, 0] - raw[:, 1]                                  # arrival minus device timestamp, seconds
            applied = []
            j = 0
            for k in range(len(rs)):
                while j + 1 < len(raw) and raw[j + 1, 0] <= rs[k, 0]:
                    j += 1
                applied.append((rs[k, 1] - raw[j, 1]) * 1000.0 if rs[k, 0] - raw[j, 0] < 0.2 else np.nan)
            write_csv(P('07_time_sync', 'clock_offset.csv'), ['scan_idx', 't_arrival', 't_lidar_device', 'raw_offset_s', 'applied_offset_ms'],
                      [[i, f'{raw[i, 0]:.3f}', f'{raw[i, 1]:.4f}', f'{off[i]:.6f}', f'{applied[i] if i < len(applied) and applied[i] == applied[i] else ""}'] for i in range(0, len(raw), 1)])
            tt = raw[:, 0] - raw[0, 0]
            bins = np.floor(tt / 60.0).astype(int)
            mins = np.array([off[bins == b].min() for b in np.unique(bins) if (bins == b).sum() > 20])
            mid = np.array([tt[bins == b].mean() for b in np.unique(bins) if (bins == b).sum() > 20])
            if len(mins) > 5:
                ppm = float(np.polyfit(mid, mins, 1)[0] * 1e6)
                spread = off - np.interp(tt, mid, mins)
                drift = {'drift_ppm': ppm, 'spread_above_envelope_ms_p50': float(np.percentile(spread, 50) * 1000), 'spread_above_envelope_ms_p95': float(np.percentile(spread, 95) * 1000),
                         'hours_to_exceed_0.15s_tolerance': (0.15 / (abs(ppm) * 1e-6) / 3600.0) if abs(ppm) > 0.01 else None, 'minutes_used': round(tt[-1] / 60, 1),
                         'resets_detected(step>100ms)': int(np.sum(np.abs(np.diff(mins)) > 0.1))}
                num('drift_ppm', round(ppm, 2), 'ppm', '07_time_sync/clock_offset.csv', 'slope of the rolling 1-minute minimum offset')
                num('hours_to_exceed_tolerance', drift['hours_to_exceed_0.15s_tolerance'], 'h', '07_time_sync/clock_offset.csv', '0.15 s tolerance / drift rate')
        jwrite(P('07_time_sync', 'summary.json'), {'clock_drift': drift, 'conditions': conditions(t0, t1)})
    except Exception as exc:
        log(f'analysis block 1 failed: {exc!r}')
    try:
        # 7.2 scan-to-image gap
        fused, rgb, th = topic_window('fused', t0, t1), topic_window('rgb', t0, t1), topic_window('thermal', t0, t1)
        gaps = {}
        if len(fused) > 20 and len(rgb) > 20:
            def nearest(a, b):
                idx = np.clip(np.searchsorted(b, a), 1, len(b) - 1)
                return np.minimum(np.abs(a - b[idx - 1]), np.abs(a - b[idx])) * 1000.0
            rg = nearest(fused[:, 1], np.sort(rgb[:, 1]))
            tg = nearest(fused[:, 1], np.sort(th[:, 1])) if len(th) > 20 else None
            write_csv(P('07_time_sync', 'sync_gap.csv'), ['scan_stamp', 'rgb_gap_ms', 'thermal_gap_ms'],
                      [[f'{fused[i, 1]:.4f}', f'{rg[i]:.2f}', f'{tg[i]:.2f}' if tg is not None else ''] for i in range(len(fused))])
            gaps = {'rgb_gap_ms': pstats(rg), 'thermal_gap_ms': pstats(tg) if tg is not None else 'thermal absent'}
            num('gap_rgb_p50_ms', gaps['rgb_gap_ms']['median'], 'ms', '07_time_sync/sync_gap.csv', 'nearest RGB stamp to each fused scan (the synchroniser picks the best match, so this is its best case)')
            num('gap_rgb_p95_ms', gaps['rgb_gap_ms']['p95'], 'ms', '07_time_sync/sync_gap.csv')
        jwrite(P('07_time_sync', 'sync_gap_summary.json'), gaps)
    except Exception as exc:
        log(f'analysis block 2 failed: {exc!r}')
    try:
        # 8.1 LiDAR stability from the dominant-plane distance sampled once per second
        pl = P('02_rates', 'lidar_plane.csv')
        stab = {}
        if pl.exists():
            a = np.loadtxt(pl, delimiter=',', skiprows=1, ndmin=2)
            a = a[(a[:, 0] >= t0) & (a[:, 0] <= t1)] if len(a) else a
            if len(a) > 60:
                tm = (a[:, 0] - a[0, 0]) / 60.0
                win = 5.0 if tm[-1] >= 12 else max(0.2, tm[-1] / 3.0)              # minutes per bin (short test runs use shorter bins)
                five = [(round(m, 2), float(np.median(a[(tm >= m) & (tm < m + win), 1]))) for m in np.arange(0, tm[-1] - win + 1e-9, win)]
                if len(five) < 2:
                    raise ValueError('window too short for a stability comparison')
                write_csv(P('08_accuracy', 'lidar_stability.csv'), ['t_min', 'plane_distance_m_median_per_bin'], [[m, f'{v:.4f}'] for m, v in five])
                write_csv(P('08_accuracy', 'lidar_plane_raw.csv'), ['t_min', 'distance_m', 'rms_m', 'inliers'], [[f'{tm[i]:.2f}', f'{a[i, 1]:.4f}', f'{a[i, 6]:.5f}', int(a[i, 5])] for i in range(0, len(a), 5)])
                stab = {'dominant_plane_rms_mm_median': float(np.median(a[:, 6]) * 1000), 'bin_minutes': win, 'first_bin_distance_m': five[0][1], 'last_bin_distance_m': five[-1][1],
                        'range_change_cm_first_to_last': (five[-1][1] - five[0][1]) * 100, 'note': 'dominant plane in view; only meaningful if the scene was static'}
                num('lidar_warmup_drift_cm', stab['range_change_cm_first_to_last'], 'cm', '08_accuracy/lidar_stability.csv', 'distance to the largest plane, first vs last 5 min')
        jwrite(P('08_accuracy', 'lidar_stability_summary.json'), stab)
    except Exception as exc:
        log(f'analysis block 3 failed: {exc!r}')
    try:
        # 15.1 coverage
        cv = P('02_rates', 'coverage_raw.csv')
        if cv.exists():
            a = np.loadtxt(cv, delimiter=',', skiprows=1, ndmin=2)
            a = a[(a[:, 0] >= t0) & (a[:, 0] <= t1)] if len(a) else a
            if len(a):
                first = a[a[:, 0] <= a[0, 0] + 120]
                write_csv(P('15_fusion_info', 'coverage.csv'), ['t_arrival', 'n_points', 'pct_rgb', 'pct_thermal', 'pct_both', 'pct_neither'], a.tolist())
                cov = {'whole_window_mean': dict(zip(['pct_rgb', 'pct_thermal', 'pct_both', 'pct_neither'], np.mean(a[:, 2:], axis=0).tolist())),
                       'first_2_minutes_mean': dict(zip(['pct_rgb', 'pct_thermal', 'pct_both', 'pct_neither'], np.mean(first[:, 2:], axis=0).tolist())), 'n_clouds': int(len(a))}
                jwrite(P('15_fusion_info', 'coverage_summary.json'), cov)
                num('coverage_rgb_pct', cov['first_2_minutes_mean']['pct_rgb'], '%', '15_fusion_info/coverage.csv', 'depends on what the camera sees')
                num('coverage_thermal_pct', cov['first_2_minutes_mean']['pct_thermal'], '%', '15_fusion_info/coverage.csv', 'thermal absent => 0')
                num('coverage_both_pct', cov['first_2_minutes_mean']['pct_both'], '%', '15_fusion_info/coverage.csv')
    except Exception as exc:
        log(f'analysis block 4 failed: {exc!r}')
    heat['conditions'] = conditions(t0, t1)
    jwrite(P(sec, 'summary_normal_use.json'), heat)
    S.summaries['normal_use'] = heat
    write_readme(sec, '12a. Normal use (long window)',
                 [f'Full pipeline with an emulated live and merged viewer plus any real base station on the hotspot, {heat["minutes"]} min. Start temperature {start_temp:.0f} C '
                  f'({"cold start" if start_temp < 50 else "NOT a cold start: the pipeline was already running before the test"}). heat_normal.csv has SoC temp, clock, throttle flags, fan, fused/merged Hz every second.',
                  'The same window provides: 05_cpu_memory/soak_memory.csv, 07_time_sync/clock_offset.csv and sync_gap.csv, 08_accuracy/lidar_stability.csv, 15_fusion_info/coverage.csv.'])


def phase_rates():
    t0 = time.time()
    vs = start_viewers()
    try:
        sleep_checked(D(600, 30))
    finally:
        stop_viewers(vs)
    t1 = time.time()
    rows, summ = [], {}
    for key in ('lidar_raw', 'lidar_restamped', 'rgb', 'thermal', 'fused', 'merged', 'caminfo_rgb', 'caminfo_thermal'):
        st = rate_stats(key, t0, t1)
        summ[key] = st
        rows.append([key, f'{st.get("hz_mean", 0):.3f}', f'{st.get("interval_p50_ms", "")}', f'{st.get("interval_p95_ms", "")}', f'{st.get("interval_std_ms", "")}', st['n'],
                     st.get('points_per_msg_median', ''), st.get('kbytes_per_msg_median', '')])
    imu = P('02_rates', 'topic_imu_per_second.csv')
    if imu.exists():
        a = np.loadtxt(imu, delimiter=',', skiprows=1, ndmin=2)
        a = a[(a[:, 0] >= t0) & (a[:, 0] <= t1)] if len(a) else a
        if len(a):
            rows.append(['imu', f'{np.mean(a[:, 1]):.2f}', '', '', '', int(a[:, 1].sum()), '', ''])
    write_csv(P('02_rates', 'rates.csv'), ['topic', 'hz_mean', 'interval_p50_ms', 'interval_p95_ms', 'interval_std_ms', 'n', 'points_per_msg_median', 'kbytes_per_msg_median'], rows)
    lr = topic_window('lidar_restamped', t0, t1)
    loss = None
    if len(lr) > 10:
        expected = (lr[-1, 1] - lr[0, 1]) / 0.1 + 1
        loss = max(0.0, 100.0 * (1 - len(lr) / expected))
        summ['lidar_scans_lost_pct'] = loss
        summ['lidar_gaps_over_150ms'] = int(np.sum(np.diff(lr[:, 0]) > 0.15))
    for key, hz in (('rate_lidar_hz', summ['lidar_raw'].get('hz_mean')), ('rate_rgb_fps', summ['rgb'].get('hz_mean')), ('rate_thermal_fps', summ['thermal'].get('hz_mean')),
                    ('rate_fused_hz', summ['fused'].get('hz_mean')), ('rate_merged_hz', summ['merged'].get('hz_mean'))):
        num(key, round(hz, 3) if hz is not None else None, 'Hz', '02_rates/rates.csv', 'viewers: emulated live+merged and any real client')
    num('lidar_scans_lost_pct', loss, '%', '02_rates/rates.csv')
    num('merged_points_before_cap', summ['merged'].get('points_per_msg_median'), 'points', '02_rates/rates.csv', 'median of the merged topic (before the viewer cap)')
    st, hz = thermal_status()
    summ['conditions'] = conditions(t0, t1)
    summ['note_fallback'] = ('thermal is ABSENT in this run: these rates ARE the "LiDAR + RGB only" column' if st == 'absent' else
                             'thermal is present: a LiDAR+RGB-only column cannot be produced without unplugging it [HUMAN]')
    jwrite(P('02_rates', 'summary.json'), summ)
    S.summaries['rates'] = summ
    write_readme('02_rates', '2. Data rates', ['rates.csv: per-topic rate and interval statistics from independent raw subscribers (same QoS and DDS profile as the pipeline).',
                                              'topic_*.csv are the raw per-message records (arrival time, header stamp, points/pixels, bytes) for the whole run; slice them by time for any other analysis.'])


def phase_stage_timing():
    t0 = time.time()
    st, _ = thermal_status()
    spec = 'lidar:40,fused:14,merged:1,rgb:1,caminfo:1' + (',thermal:1' if st == 'streaming' else '')
    cap = capture(spec, 'bench', 90)
    if not cap:
        raise RuntimeError('capture for the benchmark failed')
    env = dict(os.environ)
    outdir = P('03_timing_stages')
    reps, trials, wreps = max(30, int(200 * S.scale)), max(3, int(25 * S.scale)), max(2, int(8 * S.scale))
    rc, out = sh([sys.executable, str(HERE / 'bench.py'), str(cap), str(outdir), '--reps', str(reps), '--trials', str(trials), '--windowreps', str(wreps)], 3000, env)
    (outdir / 'bench_stdout.txt').write_text(out[-20000:])
    if rc != 0:
        raise RuntimeError(f'bench.py failed rc={rc}: {out[-600:]}')
    b = json.load(open(outdir / 'bench_summary.json'))

    def med(key):
        return b.get(key, {}).get('median_ms')
    num('projection_ms', med('projection_rgb|24000 points'), 'ms', '03_timing_stages/stage_timing.csv')
    num('colouring_ms', med('colouring_rgb|24000 points'), 'ms', '03_timing_stages/stage_timing.csv')
    tot = med('fusion_step_total|rgb + thermal') or med('fusion_step_total|rgb only (thermal absent)')
    num('fusion_ms_total', tot, 'ms', '03_timing_stages/stage_timing.csv', 'thermal absent' if st != 'streaming' else '')
    num('icp_ms_grid01', med('icp_registration_per_scan|grid 0.1 m'), 'ms', '03_timing_stages/stage_timing.csv')
    num('icp_ms_grid02', med('icp_registration_per_scan|grid 0.2 m'), 'ms', '03_timing_stages/stage_timing.csv')
    num('merge_window11_s', (med('merge_window|11 scans, grid 0.1 m') or 0) / 1000.0 or None, 's', '03_timing_stages/stage_timing.csv')
    # section 8.2 / 6.3 outputs belong to the accuracy folder too
    for f in ('icp_trials.csv', 'grid_compare.csv'):
        shutil.copy(outdir / f, P('08_accuracy', f))
        shutil.copy(outdir / f, P('06_tradeoffs', f))
    g = b.get('grid_compare', {})
    num('icp_error_mm_grid01', g.get('0.1', {}).get('icp_err_mm_median_within_envelope'), 'mm', '08_accuracy/icp_trials.csv', 'median over trials with move<=5 cm and rotation<=0.5 deg; ground truth = exact rigid transform of a real scan')
    num('icp_error_mm_grid02', g.get('0.2', {}).get('icp_err_mm_median_within_envelope'), 'mm', '08_accuracy/icp_trials.csv')
    try:
        a = np.loadtxt(P('08_accuracy', 'icp_trials.csv'), delimiter=',', skiprows=1)
        ok = a[(a[:, 0] == 0.1) & (a[:, 5] < 30)]
        within = sorted({(m, r) for m, r in zip(a[:, 1], a[:, 2]) if np.mean(a[(a[:, 0] == 0.1) & (a[:, 1] == m) & (a[:, 2] == r)][:, 5] < 30) >= 0.95})
        if within:
            num('icp_envelope_cm', max(m for m, r in within), 'cm', '08_accuracy/icp_trials.csv', 'largest translation level with >=95% trials < 30 mm error (at 0 to 2 deg)')
            num('icp_envelope_deg', max(r for m, r in within), 'deg', '08_accuracy/icp_trials.csv')
    except Exception:
        pass
    b['conditions'] = conditions(t0, time.time())
    jwrite(outdir / 'summary.json', b)
    S.summaries['stage_timing'] = b
    write_readme('03_timing_stages', '3. Time per stage', ['bench.py, isolated ROS domain, real captured data (24,000-point scans, a real RGB frame, a real merged cloud).',
                                                          'stage_timing.csv: median and p95 per stage; streamer_encode rows compare wire format 1 and 2; wire_bytes_* in bench_summary.json.',
                                                          'Merge windows use the real RegistrationMergeNode code path with the deployed parameters.'])


def phase_freshness():
    t0 = time.time()
    rows, summ = [], {}
    dur = D(300, 20)
    for label, path, iv in (('live', '/live.bin', 0.12), ('merged', '/merged.bin', 0.25), ('merged_v2', '/merged.bin?fmt=2', 0.25)):
        v = ViewerSim(path, iv)
        v.start()
        a = time.time()
        sleep_checked(dur if label != 'merged_v2' else dur / 2)
        v.stop_flag = True
        for r in v.rows:
            rows.append([label, f'{r[0]:.3f}', f'{r[3]:.3f}', f'{r[4]:.1f}', r[2], f'{r[1]:.1f}'])
        ages = [r[4] for r in v.rows if r[4] == r[4]]
        summ[label] = {'age_ms': pstats(ages), 'response_ms': pstats([r[1] for r in v.rows]), 'updates_per_s': len(v.rows) / max(time.time() - a, 1), 'errors': v.errors}
    write_csv(P('04_freshness', 'freshness_samples.csv'), ['view', 't_recv', 'cloud_stamp', 'age_ms', 'wire_bytes', 'response_ms'], rows)
    t1 = time.time()
    stages = []
    lr, fu, me = topic_window('lidar_restamped', t0, t1), topic_window('fused', t0, t1), topic_window('merged', t0, t1)
    for name, a in (('capture_to_restamp_output', lr), ('capture_to_fusion_output', fu), ('capture_to_merge_output (stamp = last scan in window)', me)):
        if len(a):
            age = (a[:, 0] - a[:, 1]) * 1000.0
            stages.append([name, f'{np.median(age):.1f}', f'{np.percentile(age, 95):.1f}', len(age)])
    for label in ('live', 'merged'):
        if label in summ and summ[label]['age_ms'].get('n'):
            stages.append([f'capture_to_client_receipt ({label})', f'{summ[label]["age_ms"]["median"]:.1f}', f'{summ[label]["age_ms"]["p95"]:.1f}', summ[label]['age_ms']['n']])
    write_csv(P('04_freshness', 'freshness_stages.csv'), ['stage', 'median_ms', 'p95_ms', 'n'], stages)
    window = 0.5
    if summ.get('merged', {}).get('age_ms', {}).get('median') is not None:
        summ['merged_mean_content_age_s'] = summ['merged']['age_ms']['median'] / 1000.0 + window / 2
    for key, view, q in (('age_live_p50_ms', 'live', 'median'), ('age_live_p95_ms', 'live', 'p95'), ('age_merged_p50_ms', 'merged', 'median'), ('age_merged_p95_ms', 'merged', 'p95')):
        num(key, summ.get(view, {}).get('age_ms', {}).get(q), 'ms', '04_freshness/freshness_samples.csv', 'client on the Pi itself (localhost); hotspot client delay is in 14_wireless')
    num('merged_content_age_s', summ.get('merged_mean_content_age_s'), 's', '04_freshness/freshness_samples.csv', 'age at receipt + half the 0.5 s merge window')
    summ['conditions'] = conditions(t0, t1)
    jwrite(P('04_freshness', 'summary.json'), summ)
    S.summaries['freshness'] = summ
    write_readme('04_freshness', '4. Data freshness', ['Age = client receipt time minus the cloud capture stamp (same clock, localhost client). merged_v2 uses the compact frame format.',
                                                      'freshness_stages.csv gives the stage delays from the topic stamps; stage differences give the time spent in each stage.',
                                                      'The hotspot-side delay to the real client is estimated from 14_wireless/wifi_clients_tcp.csv (rtt) and server_stats.csv.'])


BUSY_CODE = 'x = 1.0001\nwhile True:\n    x = x * 1.0000001 % 7.0\n'
MEM_CODE = 'import numpy as np\na = np.ones(64 * 1024 * 1024)\nwhile True:\n    a += 1.0\n'


def spawn_worker(code):
    """A stress worker as a fresh program. (Never fork this multi-threaded process: the child would inherit the sampler
    threads' pipes and keep them open forever, hanging the power sampler and the safety gates.)"""
    return subprocess.Popen([sys.executable, '-c', code], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            close_fds=True, start_new_session=True)


def kill_stray_workers():
    """Kill any stress worker left over from an earlier step (found by their one-line program text)."""
    killed = 0
    for p in psutil.process_iter(['pid', 'cmdline']):
        cl = ' '.join(p.info['cmdline'] or [])
        if ('x = 1.0001' in cl or 'a = np.ones(64 * 1024' in cl) and p.info['pid'] != os.getpid():
            try:
                p.kill()
                killed += 1
            except psutil.Error:
                pass
    if killed:
        log(f'killed {killed} stray stress worker(s)')
    return killed


def stop_workers(procs):
    for p in procs:
        try:
            p.terminate()
        except OSError:
            pass
    for p in procs:
        try:
            p.wait(5)
        except subprocess.TimeoutExpired:
            p.kill()


def phase_cpu_mem():
    t0 = time.time()
    summ = {}
    for label, with_viewers in (('no_viewers', False), ('with_viewers', True)):
        vs = start_viewers() if with_viewers else []
        a = time.time()
        sleep_checked(D(300, 20))
        b = time.time()
        stop_viewers(vs)
        s = sampler_slice(a, b)
        res = {'external_clients_mean': float(np.nanmean(s['external_clients_8080'])) if len(s.get('t', [])) else None}
        for name in PIPE_PROCS:
            if f'{name}_cpu' in s:
                res[name] = {'cpu_pct_of_core_mean': float(np.nanmean(s[f'{name}_cpu'])), 'rss_mb_mean': float(np.nanmean(s[f'{name}_rss_mb'])), 'threads': float(np.nanmax(s[f'{name}_threads']))}
        res['pipeline_cpu_pct_of_core_mean'] = float(np.nanmean(s['pipeline_cpu_pct_of_core']))
        res['pipeline_cpu_pct_of_400_mean'] = res['pipeline_cpu_pct_of_core_mean'] / 4.0
        res['pipeline_rss_mb_mean'] = float(np.nanmean(s['pipeline_rss_mb']))
        res['busiest_thread_pct_max'] = float(np.nanmax(s['busiest_thread_pct']))
        res['per_core_mean_pct'] = [float(np.nanmean(s[f'cpu{i}'])) for i in range(4)]
        summ[label] = res
    rows = []
    for cond, res in summ.items():
        for name in PIPE_PROCS:
            if name in res:
                rows.append([name, f'{res[name]["cpu_pct_of_core_mean"]:.1f}', f'{res[name]["rss_mb_mean"]:.0f}', int(res[name]['threads']), cond])
        rows.append(['WHOLE_PIPELINE', f'{res["pipeline_cpu_pct_of_core_mean"]:.1f}', f'{res["pipeline_rss_mb_mean"]:.0f}', '', cond])
    write_csv(P('05_cpu_memory', 'cpu_mem_steady.csv'), ['process', 'cpu_pct_of_one_core', 'rss_mb', 'max_threads', 'condition'], rows)
    num('pipeline_cpu_pct_of_core', summ['with_viewers']['pipeline_cpu_pct_of_core_mean'], '% of one core', '05_cpu_memory/cpu_mem_steady.csv', 'with emulated live+merged viewers; sampler runs on the same Pi (small overhead)')
    num('pipeline_mem_mb', summ['with_viewers']['pipeline_rss_mb_mean'], 'MB', '05_cpu_memory/cpu_mem_steady.csv')
    # 5d stress
    stress_rows, procs = [], []
    ctx = multiprocessing.get_context('fork')
    S.expect_throttle = True
    base_fused = hz_between('fused', t0, time.time())
    try:
        for step, (workers, mem) in enumerate([(1, False), (2, False), (3, False), (4, False), (4, True)]):
            while len([q for q in procs if q.args[2] == BUSY_CODE]) < workers:
                procs.append(spawn_worker(BUSY_CODE))
            if mem and not [q for q in procs if q.args[2] == MEM_CODE]:
                procs.append(spawn_worker(MEM_CODE))
            a = time.time()
            ok = sleep_checked(D(60, 12))
            b = time.time()
            pw = power_summary(a, b)
            s = sampler_slice(a, b)
            stress_rows.append([workers, 'yes' if mem else 'no', f'{hz_between("fused", a, b):.2f}', f'{hz_between("merged", a, b):.2f}', f'{np.nanmax(s["soc_c"]):.1f}' if len(s.get('t', [])) else '',
                                f'{pw.get("est_input_a_mean", float("nan")):.2f}', f'{pw.get("est_input_a_max", float("nan")):.2f}', sorted(set(s.get('throttled_hex', []))), 'gate_tripped' if not ok else ''])
            if not ok:
                break
    finally:
        stop_workers(procs)
    S.expect_throttle = False
    write_csv(P('05_cpu_memory', 'stress_steps.csv'), ['busy_workers', 'plus_512MB_memory_traffic', 'fused_hz', 'merged_hz', 'soc_max_c', 'current_mean_a', 'current_max_a', 'throttle_flags', 'note'], stress_rows)
    one = [r for r in stress_rows if r[0] == 1]
    num('stress_fused_hz_one_core_busy', float(one[0][2]) if one else None, 'Hz', '05_cpu_memory/stress_steps.csv')
    summ['stress_rows'] = stress_rows
    summ['conditions'] = conditions(t0, time.time())
    jwrite(P('05_cpu_memory', 'summary.json'), summ)
    S.summaries['cpu_mem'] = summ
    write_readme('05_cpu_memory', '5. Processor and memory', ['cpu_mem_steady.csv: per-process CPU (percent of one core) and RSS with and without emulated viewers (a real base station may also be attached: see external_clients_mean in summary.json).',
                                                             'soak_memory.csv comes from the long normal-use window (12_thermal_enclosure). stress_steps.csv: busy python workers (no stress-ng on this Pi) then 512 MB memory traffic.'])


def sweep_step(view, cap, secs, which):
    set_cap(cap, which)
    sleep_checked(12)
    v = ViewerSim('/merged.bin' if which == 'merged' else '/live.bin', 0.25 if which == 'merged' else 0.12, keep_last=True)
    v.start()
    a = time.time()
    sleep_checked(secs / 2)
    q = {}
    try:
        topic = 'merged:1' if which == 'merged' else 'fused:1'
        cap_npz = capture(topic, f'sweep_{which}', 25)
        pay = v.last_payload
        if cap_npz and pay:
            raw = zlib.decompress(pay, 31)
            n, seq = struct.unpack_from('<II', raw, 0)
            ox, oy, oz, sc = struct.unpack_from('<ffff', raw, 8)
            dt = np.dtype([('xyz', '<i2', 3), ('rgb', 'u1', 3), ('t', '<i2')])
            arr = np.frombuffer(raw, dt, count=n, offset=24)
            xyz = np.array([ox, oy, oz]) + arr['xyz'].astype(np.float64) * sc
            full = np.load(cap_npz)['merged_xyz' if which == 'merged' else 'fused_xyz']
            from scipy.spatial import cKDTree
            d, _ = cKDTree(xyz).query(xyz, k=2)
            q['nn_median_cm'] = float(np.median(d[:, 1]) * 100)
            q['full_points'] = int(len(full))
            for s_, k in ((0.02, '2cm'), (0.05, '5cm')):
                vf = set(map(tuple, np.unique(np.floor(full / s_).astype(np.int64), axis=0)))
                vx = set(map(tuple, np.unique(np.floor(xyz / s_).astype(np.int64), axis=0)))
                q[f'coverage_{k}_pct'] = 100.0 * len(vf & vx) / max(len(vf), 1)
    except Exception as exc:
        q['error'] = repr(exc)
    sleep_checked(secs / 2)
    b = time.time()
    v.stop_flag = True
    rows = v.window(a, b)
    s = sampler_slice(a, b)
    pw = power_summary(a, b)
    res = {'view': which, 'cap': cap, 'points_sent': float(np.mean([r[5] for r in rows])) if rows else None, 'kb_per_cloud': float(np.mean([r[2] for r in rows]) / 1024) if rows else None,
           'updates_per_s': len(rows) / (b - a), 'mbit_s': sum(r[2] for r in rows) * 8 / (b - a) / 1e6, 'response_p95_ms': pct([r[1] for r in rows], 95), 'age_p50_ms': pct([r[4] for r in rows], 50),
           'errors': v.errors, 'fused_hz': hz_between('fused', a, b), 'merged_hz': hz_between('merged', a, b),
           'pipeline_cpu_pct': float(np.nanmean(s['pipeline_cpu_pct_of_core'])) if len(s.get('t', [])) else None, 'streamer_cpu_pct': float(np.nanmean(s['fusion_web_streamer_cpu'])) if len(s.get('t', [])) else None,
           'soc_max_c': pw.get('soc_c_max'), 'est_input_a_max': pw.get('est_input_a_max'), 'cons_input_a_max': pw.get('cons_input_a_max'), 'ext5v_min': pw.get('ext5v_min')}
    res.update(q)
    return res


def phase_tradeoffs():
    t0 = time.time()
    orig = {w: get_cap(w) for w in ('merged', 'live')}
    log(f'original caps {orig}')
    out = {'merged': [], 'live': []}
    sweep_cols = ['view', 'cap', 'points_sent', 'kb_per_cloud', 'updates_per_s', 'mbit_s', 'response_p95_ms', 'age_p50_ms', 'errors', 'fused_hz', 'merged_hz', 'pipeline_cpu_pct',
                  'streamer_cpu_pct', 'soc_max_c', 'est_input_a_max', 'cons_input_a_max', 'ext5v_min', 'nn_median_cm', 'full_points', 'coverage_2cm_pct', 'coverage_5cm_pct', 'error']
    for which in ('merged', 'live'):
        write_csv(P('06_tradeoffs', f'sweep_{which}.csv'), sweep_cols, [])      # header now; rows are appended after every step so a timeout cannot lose them
    try:
        for which, caps in (('merged', [20000, 40000, 60000, 80000, 105000]), ('live', [5000, 10000, 15000, 20000, 25000])):
            for cap in caps:
                if S.gate_event.is_set():
                    break
                r = sweep_step(which, cap, D(90, 20), which)
                out[which].append(r)
                with open(P('06_tradeoffs', f'sweep_{which}.csv'), 'a', newline='') as fh:
                    csv.writer(fh).writerow([r.get(k, '') for k in sweep_cols])
                log(f'{which} cap {cap}: {r["points_sent"]} pts, {r["updates_per_s"]:.2f} upd/s, {r["mbit_s"]:.2f} Mbit/s, cov5 {r.get("coverage_5cm_pct")}')
    finally:
        for w in ('merged', 'live'):
            if orig.get(w):
                set_cap(orig[w], w)
    def get(which, cap, key):
        for r in out[which]:
            if r['cap'] == cap:
                return r.get(key)
    num('sweep_merged_cov5cm_20k', get('merged', 20000, 'coverage_5cm_pct'), '%', '06_tradeoffs/sweep_merged.csv')
    num('sweep_merged_cov5cm_60k', get('merged', 60000, 'coverage_5cm_pct'), '%', '06_tradeoffs/sweep_merged.csv')
    num('sweep_merged_mbit_20k', get('merged', 20000, 'mbit_s'), 'Mbit/s', '06_tradeoffs/sweep_merged.csv')
    num('sweep_merged_mbit_60k', get('merged', 60000, 'mbit_s'), 'Mbit/s', '06_tradeoffs/sweep_merged.csv')
    num('sweep_live_cov5cm_5k', get('live', 5000, 'coverage_5cm_pct'), '%', '06_tradeoffs/sweep_live.csv')
    num('sweep_live_cov5cm_25k', get('live', 25000, 'coverage_5cm_pct'), '%', '06_tradeoffs/sweep_live.csv')
    for r in out['merged']:
        if r['cap'] == 20000:
            num('viewer_mbit_per_viewer', r['mbit_s'], 'Mbit/s', '06_tradeoffs/sweep_merged.csv', 'at the 20,000 cap, wire format 1')
    out['conditions'] = conditions(t0, time.time())
    out['caps_restored_to'] = orig
    jwrite(P('06_tradeoffs', 'summary.json'), out)
    S.summaries['tradeoffs'] = out
    write_readme('06_tradeoffs', '6. Configuration trade-offs', ['sweep_merged.csv / sweep_live.csv: per point cap, what the viewer receives and what it costs (wire format 1, as the base station uses).',
                                                                'icp_trials.csv / grid_compare.csv: merging grid 0.1 vs 0.2 m (from the offline benchmark). Deployed caps are restored afterwards.'])


def phase_rgb_fps():
    t0 = time.time()
    src = HOME / 'ros2_ws/src/pitfusion3d_bringup/launch'
    d = P('06_tradeoffs', 'fps_variant')
    d.mkdir(exist_ok=True)
    rs = (src / 'real_sensors.launch.py').read_text()
    assert "'framerate': 10," in rs
    (d / 'real_sensors_15fps.launch.py').write_text(rs.replace("'framerate': 10,", "'framerate': 15,", 1))
    full = (src / 'pitfusion3d_full.launch.py').read_text()
    new, n = re.subn(r"PathJoinSubstitution\(\s*\[FindPackageShare\('pitfusion3d_bringup'\), 'launch', 'real_sensors.launch.py'\]\s*\)", repr(str(d / 'real_sensors_15fps.launch.py')), full)
    if n != 1:
        raise RuntimeError('could not build the 15 fps launch variant')
    (d / 'pitfusion3d_15fps.launch.py').write_text(new)
    res = {}
    a = time.time()
    sleep_checked(D(120, 20))
    res['10fps'] = fps_window(a, time.time())
    proc = None
    try:
        sh(['systemctl', '--user', 'stop', UNIT], 90)
        time.sleep(5)
        sh([sys.executable, str(HOME / 'ros2_ws/src/pitfusion3d_bringup/scripts/clean_stale_shm.py')], 30)
        env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        proc = subprocess.Popen(['ros2', 'launch', str(d / 'pitfusion3d_15fps.launch.py')], stdout=open(d / 'launch_15fps.log', 'w'), stderr=subprocess.STDOUT, env=env, start_new_session=True)
        wait_ready(240)
        sleep_checked(20)
        a = time.time()
        sleep_checked(D(120, 20))
        res['15fps'] = fps_window(a, time.time())
    finally:
        if proc and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(30)
            except subprocess.TimeoutExpired:
                pass
        time.sleep(8)
        sh(['systemctl', '--user', 'start', UNIT], 60)
        wait_ready(240)
    write_csv(P('06_tradeoffs', 'rgb_fps.csv'), ['setting', 'rgb_hz', 'fused_hz', 'merged_hz', 'pipeline_cpu_pct_of_core', 'soc_max_c'], [[k, v['rgb_hz'], v['fused_hz'], v['merged_hz'], v['cpu'], v['soc']] for k, v in res.items()])
    res['conditions'] = conditions(t0, time.time())
    jwrite(P('06_tradeoffs', 'rgb_fps_summary.json'), res)
    S.summaries['rgb_fps'] = res


def fps_window(a, b):
    s = sampler_slice(a, b)
    return {'rgb_hz': round(hz_between('rgb', a, b), 2), 'fused_hz': round(hz_between('fused', a, b), 2), 'merged_hz': round(hz_between('merged', a, b), 2),
            'cpu': round(float(np.nanmean(s['pipeline_cpu_pct_of_core'])), 1) if len(s.get('t', [])) else None, 'soc': round(float(np.nanmax(s['soc_c'])), 1) if len(s.get('t', [])) else None}


def phase_accuracy():
    t0 = time.time()
    cap = capture('lidar:200', 'precision', 60)
    out = {}
    if cap:
        sys.path.insert(0, str(HERE))
        from probe import plane_fit
        z = np.load(cap)
        xyz, off = z['lidar_xyz'], z['lidar_offsets']
        rng = np.random.default_rng(1)
        fits = [plane_fit(xyz[off[i]:off[i + 1]].astype(np.float64), rng) for i in range(len(off) - 1)]
        fits = [f for f in fits if f]
        if fits:
            arr = np.array(fits)
            out = {'scans': len(fits), 'plane_rms_mm_median': float(np.median(arr[:, 5]) * 1000), 'plane_distance_std_mm': float(arr[:, 0].std() * 1000), 'plane_distance_mean_m': float(arr[:, 0].mean()),
                   'inliers_median': float(np.median(arr[:, 4])), 'note': 'dominant plane in the scene at the time; only a fair precision figure if that plane is a flat wall/board'}
            write_csv(P('08_accuracy', 'lidar_precision_scans.csv'), ['distance_m', 'nx', 'ny', 'nz', 'inliers', 'rms_m'], arr.tolist())
            num('lidar_precision_mm', out['plane_rms_mm_median'], 'mm', '08_accuracy/lidar_precision_scans.csv', 'RMS point-to-plane of the dominant plane over 200 scans')
    out['conditions'] = conditions(t0, time.time())
    out['human_items'] = '8.3 physical movement check needs a person'
    jwrite(P('08_accuracy', 'summary.json'), out)
    S.summaries['accuracy'] = out
    write_readme('08_accuracy', '8. Geometric accuracy', ['lidar_precision_scans.csv: plane fit per scan on 200 consecutive scans (dominant plane). lidar_stability.csv: plane distance over the long window.',
                                                         'icp_trials.csv: merging error vs known motion (exact rigid transforms of a real scan), grids 0.1 and 0.2 m, 25 trials per level; grid_compare.csv: summary per grid.'])


def phase_storage():
    t0 = time.time()
    du = shutil.disk_usage(HOME)
    res = {'free_gb': du.free / 1e9, 'total_gb': du.total / 1e9}
    tmp = P('11_storage', '_ddtest')
    count = 1024 if S.scale >= 0.5 else 256
    rc, out = sh(f'dd if=/dev/zero of={tmp} bs=1M count={count} conv=fdatasync 2>&1', 300)
    m = re.search(r'([\d.]+) ([MG]B)/s', out)
    res['sd_write_mb_s'] = float(m.group(1)) * (1000 if m and m.group(2) == 'GB' else 1) if m else None
    res['dd_output'] = out[-200:]
    try:
        tmp.unlink()
    except OSError:
        pass
    a, b = time.time() - 120, time.time()
    rows = []
    sizes = {}
    for key, label in (('lidar_raw', 'raw_lidar'), ('rgb', 'raw_rgb'), ('thermal', 'raw_thermal'), ('fused', 'fused_cloud'), ('merged', 'merged_full')):
        st = rate_stats(key, a, b)
        mbs = st.get('mean_msg_bytes', 0) * st.get('hz_mean', 0) / 1e6 if st.get('n', 0) > 1 else 0
        sizes[label] = mbs
        rows.append([label, f'{st.get("hz_mean", 0):.2f}', f'{st.get("mean_msg_bytes", 0) / 1000:.1f}', f'{mbs:.3f}', f'{mbs * 3.6:.2f}', f'{res["free_gb"] / (mbs * 3.6) if mbs else ""}'])
    for label, path in (('merged_viewer_v1', '/merged.bin'), ('merged_viewer_v2_compact', '/merged.bin?fmt=2')):
        try:
            sizes_b = []
            for _ in range(3):
                st, body, hd = http_get(path, 8, {'Accept-Encoding': 'gzip'})
                sizes_b.append(len(body))
                time.sleep(1.2)
            hz = rate_stats('merged', a, b).get('hz_mean', 1.5)
            mbs = np.mean(sizes_b) * hz / 1e6
            rows.append([label, f'{hz:.2f}', f'{np.mean(sizes_b) / 1000:.1f}', f'{mbs:.3f}', f'{mbs * 3.6:.2f}', f'{res["free_gb"] / (mbs * 3.6)}'])
            sizes[label] = mbs
        except Exception as exc:
            rows.append([label, 'error', repr(exc), '', '', ''])
    write_csv(P('11_storage', 'storage.csv'), ['stream', 'hz', 'kbytes_per_msg', 'MB_per_s', 'GB_per_hour', 'hours_until_free_space_full'], rows)
    raw_all = sizes.get('raw_lidar', 0) + sizes.get('raw_rgb', 0) + sizes.get('raw_thermal', 0)
    res['raw_all_mb_s'] = raw_all
    res['raw_fill_minutes'] = res['free_gb'] * 1000 / raw_all / 60 if raw_all else None
    num('storage_free_gb', res['free_gb'], 'GB', '11_storage/storage.csv')
    num('sd_write_mb_s', res['sd_write_mb_s'], 'MB/s', '11_storage/storage.csv', f'dd {count} MB with fdatasync')
    for key, label in (('gbh_fused', 'fused_cloud'), ('gbh_raw_lidar', 'raw_lidar'), ('gbh_raw_rgb', 'raw_rgb'), ('gbh_raw_thermal', 'raw_thermal')):
        num(key, round(sizes.get(label, 0) * 3.6, 2), 'GB/h', '11_storage/storage.csv', 'thermal absent => 0' if label == 'raw_thermal' else '')
    num('gbh_merged_compact', round(sizes.get('merged_viewer_v2_compact', 0) * 3.6, 2), 'GB/h', '11_storage/storage.csv', 'wire format 2')
    num('raw_all_mb_s', round(raw_all, 2), 'MB/s', '11_storage/storage.csv', 'lidar + rgb + thermal (IMU excluded)')
    num('raw_fill_minutes', res['raw_fill_minutes'], 'min', '11_storage/storage.csv')
    # 60 s raw bag: dropped messages
    bag = P('11_storage', '_bag')
    shutil.rmtree(bag, ignore_errors=True)
    secs = int(D(60, 10))
    topics = ['/livox/lidar', '/landfill/rgb/image_raw', '/livox/imu'] + (['/landfill/thermal/image_raw'] if thermal_status()[0] != 'absent' else [])
    a = time.time()
    p = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag)] + topics, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    sleep_checked(secs)
    os.killpg(p.pid, signal.SIGINT)
    try:
        p.wait(40)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGTERM)
    b = time.time()
    rc, info = sh(['ros2', 'bag', 'info', str(bag)], 30)
    counts = {m.group(1): int(m.group(2)) for m in re.finditer(r'Topic: (\S+) \| Type: \S+ \| Count: (\d+)', info)}
    exp = {'/livox/lidar': rate_stats('lidar_raw', a, b).get('hz_mean', 0) * (b - a), '/landfill/rgb/image_raw': rate_stats('rgb', a, b).get('hz_mean', 0) * (b - a)}
    bag_rows = [[t, c, f'{exp.get(t, ""):.0f}' if t in exp else '', f'{100 * (1 - c / exp[t]):.1f}' if t in exp and exp[t] else ''] for t, c in counts.items()]
    write_csv(P('11_storage', 'bag_record_test.csv'), ['topic', 'messages_in_bag', 'messages_expected(topic_probe)', 'estimated_drop_pct'], bag_rows)
    shutil.rmtree(bag, ignore_errors=True)
    res['bag_seconds'] = secs
    res['conditions'] = conditions(t0, time.time())
    jwrite(P('11_storage', 'summary.json'), res)
    S.summaries['storage'] = res
    write_readme('11_storage', '11. Storage', ['storage.csv: measured message sizes x rates = MB/s and GB/hour per stream, and hours until the free space fills.',
                                              'bag_record_test.csv: a short real recording of the raw streams to the SD card, comparing messages written against messages seen by the independent probe (the bag is deleted afterwards).'])


def boot_events(boot):
    """Events for one boot, in seconds since that boot started (journal monotonic timestamps: the Pi has no RTC, so wall-clock stamps of early entries are wrong)."""
    ev = []
    pats = [('system', 'Reached target multi-user.target', 'os_ready'), ('system', 'Started lidar-power.service', 'lidar_power_service_started'),
            ('user', 'Started pitfusion3d.service', 'pipeline_service_started'), ('user', 'Serving merged', 'viewer_page_serving'),
            ('user', 'rpicam-vid capture started', 'rgb_capture_started'), ('user', 'window_jitter', 'restamp_first_log (about 5 s after the first LiDAR scan)'),
            ('user', 'Published merged cloud', 'first_merged_cloud')]
    for scope, pat, name in pats:
        cmd = f'journalctl {"--user " if scope == "user" else ""}-b {boot} -o short-monotonic --no-pager 2>/dev/null | grep -m1 -F "{pat}"'
        rc, out = sh(cmd, 90)
        m = re.match(r'^\[\s*([\d.]+)\]', out)
        if m:
            ev.append((name, float(m.group(1)), scope, 'estimated' if 'restamp' in name else 'measured'))
    return ev


def phase_startup():
    t0 = time.time()
    res = {}
    rows = []
    # 13.1 boot timelines from the journals (previous boots included): a power-cycle repeat is a [HUMAN] test
    rc, lb = sh('journalctl --list-boots --no-pager 2>/dev/null | tail -4', 30)
    res['boots_listed'] = lb
    for boot in (0, -1, -2):
        for name, t, scope, kind in boot_events(boot):
            rows.append([boot, name, f'{t:.1f}', scope, kind])
    rc, an = sh('systemd-analyze 2>/dev/null | head -2', 20)
    res['systemd_analyze'] = an
    write_csv(P('13_startup_reliability', 'boot_timeline.csv'), ['boot_index(0=current)', 'event', 't_s_after_first_journal_entry', 'journal', 'measured_or_estimated'], rows)
    cur = {r[1]: float(r[2]) for r in rows if r[0] == 0}
    num('boot_os_s', cur.get('os_ready'), 's', '13_startup_reliability/boot_timeline.csv', 'current boot, from journals')
    num('boot_merged_s', cur.get('first_merged_cloud'), 's', '13_startup_reliability/boot_timeline.csv', 'current boot, from journals')
    if 'restamp_first_log (about 5 s after the first LiDAR scan)' in cur:
        num('boot_lidar_s', cur['restamp_first_log (about 5 s after the first LiDAR scan)'] - 5.0, 's', '13_startup_reliability/boot_timeline.csv', 'estimated: first restamp log minus 5 s')
    # 13.2 software restarts
    rs_rows = []
    times = []
    for i in range(3):
        a = time.time()
        mark(f'restart_{i + 1}_start')
        sh(['systemctl', '--user', 'restart', UNIT], 120)
        t_data = wait_ready(240)
        times.append(t_data)
        b = time.time()
        firsts = {}
        time.sleep(8)
        for key in ('lidar_raw', 'lidar_restamped', 'rgb', 'thermal', 'fused', 'merged'):
            w = topic_window(key, a, b + 8)
            firsts[key] = float(w[0, 0] - a) if len(w) else None
        rs_rows.append([i + 1, f'{t_data:.1f}' if t_data else '', json.dumps(firsts)])
        mark(f'restart_{i + 1}_end')
        sleep_checked(D(30, 8))
    write_csv(P('13_startup_reliability', 'software_restart.csv'), ['restart', 'seconds_until_viewer_has_data', 'first_message_seconds_per_topic'], rs_rows)
    ok = [x for x in times if x]
    num('sw_restart_s', float(np.median(ok)) if ok else None, 's', '13_startup_reliability/software_restart.csv', 'median of 3')
    res['software_restart_s'] = times
    # 13.3 kill each node
    kills = []
    topic_of = {'livox_restamp_node': 'lidar_restamped', 'sensor_fusion_node': 'fused', 'registration_merge_node': 'merged', 'rgb_camera_node': 'rgb', 'livox_ros2_avia_node': 'lidar_raw',
                'calibration_publisher': 'caminfo_rgb', 'fusion_web_streamer': None}
    viewer = ViewerSim('/live.bin', 0.1)
    viewer.start()
    try:
        for name in ['livox_restamp_node', 'calibration_publisher', 'sensor_fusion_node', 'registration_merge_node', 'fusion_web_streamer', 'rgb_camera_node', 'livox_ros2_avia_node']:
            before = pipeline_pids().get(name, [])
            if not before:
                kills.append([name, 'not running', '', '', ''])
                continue
            tk = time.time()
            for pid in before:
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
            respawn = None
            while time.time() - tk < 40:
                now_p = pipeline_pids().get(name, [])
                if now_p and not set(now_p) & set(before):
                    respawn = time.time() - tk
                    break
                time.sleep(0.25)
            time.sleep(10)
            tkey = topic_of.get(name)
            resume = None
            if tkey:
                w = topic_window(tkey, tk + 0.05, tk + 30)
                if len(w):
                    gap = np.diff(np.concatenate([[tk], w[:, 0]]))
                    resume = float(w[0, 0] - tk)
            rows_v = viewer.window(tk, tk + 15)
            max_gap = max(np.diff([tk] + [r[0] for r in rows_v]).tolist() + [0]) if rows_v else None
            kills.append([name, 'killed with SIGKILL', f'{respawn:.1f}' if respawn else 'not within 40 s', f'{resume:.1f}' if resume is not None else '', f'{max_gap:.1f}' if max_gap else ''])
            sleep_checked(D(15, 8))
    finally:
        viewer.stop_flag = True
    kills.append(['thermal_camera_node', 'SKIPPED by rule: never SIGKILL the thermal node (can wedge the camera)', '', '', ''])
    write_csv(P('13_startup_reliability', 'node_kill_recovery.csv'), ['node', 'action', 'seconds_until_process_back', 'seconds_until_first_output_message', 'longest_viewer_gap_s_in_15s'], kills)
    rs = [float(k[2]) for k in kills if re.match(r'^[\d.]+$', str(k[2]))]
    num('node_restart_s', max(rs) if rs else None, 's', '13_startup_reliability/node_kill_recovery.csv', 'slowest node')
    # 13.4 corner-case suites and unit tests
    corner = P('13_startup_reliability', 'corner_logs')
    corner.mkdir(parents=True, exist_ok=True)
    cdir = HOME / 'PitFusion3D_repo/docs/final_campaign_2026-11-12/corner'
    passed = failed = 0
    suite_rows = []
    for script in ('t_fusion.py', 't_merge.py', 't_merge_stall.py', 't_streamer.py', 't_restamp.py'):
        f = cdir / script
        if not f.exists():
            suite_rows.append([script, 'script missing', '', ''])
            continue
        if script == 't_streamer.py':
            txt = f.read_text()
            txt = txt.replace("'--ros-args','-p',f'http_port:={PORT}'", "'--ros-args','-p',f'http_port:={PORT}','-p','max_points_merged:=20000','-p','max_points_live:=5000'")
            txt = txt.replace("s,e2,b=get('/live.bin'); n2,_=npts(b);", "get('/live.bin')\nfor _ in range(30): pl.publish(cloud(24000)); time.sleep(0.1)\ns,e2,b=get('/live.bin'); n2,_=npts(b);")
            pdir = P('scripts', 'corner_patched')
            pdir.mkdir(exist_ok=True)
            (pdir / 't_streamer.py').write_text('# PATCHED COPY for the deployed defaults: explicit 20000/5000 caps for the capping checks (the deployed merged cap is now 80000),\n'
                                               '# and one /live.bin request first so the on-demand live subscription is active (S5). Everything else is unchanged.\n' + txt)
            f = pdir / 't_streamer.py'
        rc, out = sh(f'cd {corner} && timeout 420 python3 {f} 2>&1', 460)
        (corner / f'{script}.txt').write_text(out[-30000:])
        p, fl = len(re.findall(r'\[PASS\]', out)), len(re.findall(r'\[FAIL\]', out))
        passed += p
        failed += fl
        suite_rows.append([script, rc, p, fl])
    rc, ut = sh(f'cd {HOME}/ros2_ws/src && python3 -m pytest -q landfill_camera_sim/test 2>&1 | tail -3', 300)
    suite_rows.append(['pytest landfill_camera_sim/test', rc, ut.splitlines()[-1] if ut else '', ''])
    write_csv(P('13_startup_reliability', 'corner_case_suites.csv'), ['suite', 'exit_code', 'pass_count', 'fail_count'], suite_rows)
    num('corner_cases_passed', passed, 'cases', '13_startup_reliability/corner_case_suites.csv', 'counts [PASS] lines of t_fusion/t_merge/t_merge_stall/t_streamer')
    num('corner_cases_total', passed + failed, 'cases', '13_startup_reliability/corner_case_suites.csv')
    # 13.5 LiDAR standby / wake through the real shutdown and boot hooks
    hook = HOME / 'lidar_ctl/lidar_power_hook.sh'
    life = {}
    if hook.exists():
        mark('lidar_hook_stop')
        rc, out = sh([str(hook), 'stop'], 120)
        rc, tail = sh(f'tail -1 {HOME}/lidar_ctl/power_events.log', 10)
        life['stop_event'] = tail
        mark('lidar_hook_start')
        rc, out = sh([str(hook), 'start'], 150)
        rc, tail2 = sh(f'tail -2 {HOME}/lidar_ctl/power_events.log', 10)
        life['start_events'] = tail2
        sh(['systemctl', '--user', 'start', UNIT], 60)
        wait_ready(240)
        try:
            ev = json.loads(tail)
            num('lidar_standby_s', ev['detail'].get('switch_seconds'), 's', '13_startup_reliability/lidar_hook.json', f'mode {ev["detail"].get("command")} (sleep mode file: {(HOME / "lidar_ctl/sleep_mode").read_text().strip()})')
        except Exception:
            pass
    jwrite(P('13_startup_reliability', 'lidar_hook.json'), life)
    # 13.6 thermal fallback
    rc, jl = sh(f'journalctl --user -u {UNIT} --since "-2 hours" --no-pager 2>/dev/null | grep -c "lidar + RGB only"', 40)
    st, hz = thermal_status()
    res['thermal'] = {'status': st, 'hz': hz, 'fallback_log_lines_last_2h': jl,
                      'note': 'unplug/replug cannot be done unattended [HUMAN]; with thermal absent the fused rate in 02_rates shows the LiDAR+RGB fallback works'}
    res['conditions'] = conditions(t0, time.time())
    jwrite(P('13_startup_reliability', 'summary.json'), res)
    S.summaries['startup'] = res
    write_readme('13_startup_reliability', '13. Start-up and reliability', ['boot_timeline.csv: events from the system and user journals for this and the two previous boots (a fresh power-cycle timeline is a [HUMAN] repeat).',
                                                                           'software_restart.csv, node_kill_recovery.csv (SIGKILL each node except thermal), corner_case_suites.csv + corner_logs/, lidar_hook.json (real shutdown/boot hooks).'])


def phase_wireless():
    t0 = time.time()
    rows = []
    for count in (1, 2, 4, 8, 16):
        vs = []
        for i in range(count):
            vs.append(ViewerSim('/merged.bin' if i % 2 == 0 else '/live.bin', 0.25 if i % 2 == 0 else 0.12))
        for v in vs:
            v.start()
        try:
            s0 = json.loads(http_get('/stats')[1])
        except Exception:
            s0 = {}
        a = time.time()
        sleep_checked(D(60, 15))
        b = time.time()
        for v in vs:
            v.stop_flag = True
        try:
            s1 = json.loads(http_get('/stats')[1])
        except Exception:
            s1 = {}
        allrows = [r for v in vs for r in v.window(a, b)]
        mbit = (s1.get('bytes_sent', 0) - s0.get('bytes_sent', 0)) * 8 / (b - a) / 1e6 if s0 else sum(r[2] for r in allrows) * 8 / (b - a) / 1e6
        sl = sampler_slice(a, b)
        pw = power_summary(a, b)
        rows.append([count, f'{mbit:.2f}', f'{np.mean([len(v.window(a, b)) / (b - a) for v in vs]):.2f}', f'{pct([r[1] for r in allrows], 50):.0f}' if allrows else '', f'{pct([r[1] for r in allrows], 95):.0f}' if allrows else '',
                     f'{np.nanmean(sl["fusion_web_streamer_cpu"]):.1f}' if len(sl.get('t', [])) else '', f'{pw.get("est_input_a_max", float("nan")):.2f}', sum(v.errors for v in vs)])
        sleep_checked(8)
    write_csv(P('14_wireless', 'viewer_scaling.csv'), ['viewers', 'total_mbit_s_served', 'per_viewer_updates_per_s', 'response_p50_ms', 'response_p95_ms', 'streamer_cpu_pct', 'peak_input_a', 'errors'], rows)
    write_csv(P('14_wireless', 'wifi_range.csv'), ['location', 'distance_m', 'walls', 'band', 'throughput_mbit', 'merged_updates_per_s', 'age_p50_ms', 'usable_yes_no'],
              [['NOT AUTOMATED [HUMAN]: needs a person to move the laptop/phone', '', '', '', '', '', '', '']])
    last = rows[-1]
    num('viewers_16_total_mbit', float(last[1]), 'Mbit/s', '14_wireless/viewer_scaling.csv', 'emulated viewers over loopback (not the radio); see wifi_clients_tcp.csv for the real client')
    num('viewers_16_resp_ms', float(last[3]) if last[3] else None, 'ms', '14_wireless/viewer_scaling.csv', 'median response')
    S.summaries['wireless'] = {'viewer_scaling': rows, 'conditions': conditions(t0, time.time())}
    jwrite(P('14_wireless', 'summary.json'), S.summaries['wireless'])
    write_readme('14_wireless', '14. Wireless viewing', ['viewer_scaling.csv: 1..16 emulated viewers on this Pi (loopback; they load the server, not the radio).',
                                                        'wifi_link.csv / wifi_clients_tcp.csv / wifi_ping.csv / server_stats.csv: the WHOLE run, from the access-point side, for the real base station if one was connected (rtt, retransmits, goodput, send queue, pings, aborted frames).',
                                                        'hotspot_status.json: whether the hotspot came up, its profile, band and channel. wifi_session_summary.json is written at the end.'])


def snapshot(tag):
    st, _ = thermal_status()
    cap = capture('merged:1,rgb:1' + (',thermal:1' if st == 'streaming' else ''), f'snap_{tag}', 60)
    if not cap:
        return
    z = np.load(cap, allow_pickle=True)
    d = P('15_fusion_info')
    xyz = z['merged_xyz'].astype(np.float32)
    rest = z['merged_rest']
    rgbpk = rest[:, 1].astype(np.float32).view(np.uint32)
    np.savez_compressed(d / f'merged_{tag}.npz', xyz=xyz, rgb=rgbpk, thermal=rest[:, 2].astype(np.float32), intensity=rest[:, 0].astype(np.float32))
    import cv2
    if 'rgb_img' in z.files:
        cv2.imwrite(str(d / f'rgb_frame_{tag}.png'), cv2.cvtColor(z['rgb_img'][0], cv2.COLOR_RGB2BGR))
    if 'thermal_img' in z.files:
        t_img = z['thermal_img'][0].astype(np.float32)
        lo, hi = np.percentile(t_img, [1, 99])
        cv2.imwrite(str(d / f'thermal_frame_{tag}.png'), np.clip((t_img - lo) / max(hi - lo, 1) * 255, 0, 255).astype(np.uint8))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    m = xyz[:, 0] > 0.5
    sel = np.where(m)[0]
    if len(sel) > 120000:
        sel = np.random.default_rng(0).choice(sel, 120000, replace=False)
    u, v = -xyz[sel, 1] / xyz[sel, 0], -xyz[sel, 2] / xyz[sel, 0]
    col = np.stack([(rgbpk[sel] >> 16) & 255, (rgbpk[sel] >> 8) & 255, rgbpk[sel] & 255], axis=1) / 255.0
    has = rgbpk[sel] != 0
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.scatter(u[~has], v[~has], c=xyz[sel][~has, 0], s=1, cmap='viridis')
    ax.scatter(u[has], v[has], c=col[has], s=1)
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(0.9, -0.9)                      # v increases DOWNWARD like an image (floor at the bottom)
    ax.set_aspect('equal')
    ax.set_title(f'merged cloud {tag} (sensor view, u=-y/x, v=-z/x)')
    fig.savefig(d / f'quicklook_{tag}.png', dpi=100)
    plt.close(fig)


def phase_fusion_info():
    t0 = time.time()
    for i in range(3):
        snapshot(f'phase_{i + 1}')
        sleep_checked(D(30, 5))
    st, hz = thermal_status()
    s = sampler_slice(t0, time.time())
    res = {'combination_measured': f'LiDAR + RGB + thermal({st})', 'fused_hz': hz_between('fused', t0, time.time()),
           'pipeline_cpu_pct_of_core': float(np.nanmean(s['pipeline_cpu_pct_of_core'])) if len(s.get('t', [])) else None,
           'not_automatable': '15.2 other sensor combinations need nodes stopped or a sensor unplugged [HUMAN]; 15.3 scenes (warm object, outdoors handheld/static) need a person; 15.4 observations need a person',
           'conditions': conditions(t0, time.time())}
    jwrite(P('15_fusion_info', 'summary.json'), res)
    S.summaries['fusion_info'] = res
    write_readme('15_fusion_info', '15. Information gained by fusion', ['coverage.csv: share of points with colour / thermal / both / neither, every 5th fused cloud for the whole long window (summary in coverage_summary.json).',
                                                                       'merged_*.npz: xyz (float32), rgb (uint32 0xRRGGBB, 0 = none), thermal (float32 relative, NaN = none), intensity. quicklook_*.png: sensor view, v increases downward.',
                                                                       'These snapshots are of whatever scene the rig was pointed at; named scenes (warm object, outdoors) are a [HUMAN] repeat.'])


def phase_overload():
    t0 = time.time()
    sec = '12_thermal_enclosure'
    base_fused = hz_between('fused', t0 - 600, t0)
    S.expect_throttle = True
    S.gate_temp = 86.0
    procs = []
    ctx = multiprocessing.get_context('fork')
    rows = []
    t_80 = None
    vs = start_viewers()
    try:
        for _ in range(4):
            procs.append(spawn_worker(BUSY_CODE))
        a = time.time()
        limit = D(2400, 90)
        while time.time() - a < limit:
            check_stop()
            time.sleep(2)
            now = time.time()
            tc = soc_temp()
            rows.append([f'{now - a:.0f}', f'{tc:.1f}', hex(max(throttled_flags(), 0)), f'{hz_between("fused", now - 10, now):.2f}', f'{hz_between("merged", now - 10, now):.2f}'])
            if tc >= 80 and t_80 is None:
                t_80 = now - a
            if S.gate_event.is_set():
                log(f'overload stopped by gate: {S.gate_reason}')
                break
    finally:
        stop_workers(procs)
        stop_viewers(vs)
    b = time.time()
    write_csv(P(sec, 'heat_overload.csv'), ['t_s', 'soc_c', 'throttled_hex', 'fused_hz_10s', 'merged_hz_10s'], rows)
    temps = np.array([float(r[1]) for r in rows]) if rows else np.array([])
    fused = np.array([float(r[3]) for r in rows]) if rows else np.array([])
    over = {'time_to_80c_min': t_80 / 60 if t_80 else None, 'peak_c': float(temps.max()) if len(temps) else None, 'steady_c_last_5min': float(temps[-150:].mean()) if len(temps) else None,
            'slowdown_pct_fused': float(100 * (1 - fused[len(fused) // 2:].mean() / base_fused)) if len(fused) and base_fused else None, 'baseline_fused_hz': base_fused, 'minutes': round((b - a) / 60, 1),
            'ended_by_gate': S.gate_reason}
    S.gate_event.clear()
    S.gate_reason = None
    num('temp_overload_time_to_80_min', over['time_to_80c_min'], 'min', f'{sec}/heat_overload.csv')
    num('temp_overload_steady_c', over['steady_c_last_5min'], 'C', f'{sec}/heat_overload.csv')
    num('temp_overload_peak_c', over['peak_c'], 'C', f'{sec}/heat_overload.csv')
    num('overload_slowdown_pct', over['slowdown_pct_fused'], '%', f'{sec}/heat_overload.csv', 'fused Hz in the second half vs the 10 minutes before')
    # 12c cool-down
    cd = []
    c0 = time.time()
    S.gate_temp = 85.0
    dur = D(900, 60)
    while time.time() - c0 < dur:
        check_stop()
        time.sleep(2)
        now = time.time()
        cd.append([f'{now - c0:.0f}', f'{soc_temp():.1f}', hex(max(throttled_flags(), 0)), f'{hz_between("fused", now - 10, now):.2f}', f'{hz_between("merged", now - 10, now):.2f}'])
    write_csv(P(sec, 'cooldown.csv'), ['t_s', 'soc_c', 'throttled_hex', 'fused_hz_10s', 'merged_hz_10s'], cd)
    over['conditions'] = conditions(t0, time.time())
    jwrite(P(sec, 'summary_overload.json'), over)
    S.summaries['overload'] = over
    write_readme(sec, '12b/12c. Overload and cool-down', ['4 busy workers on top of the pipeline until the time limit or 86 C, then a cool-down with the pipeline still running. Safety gate temperature for this step is 86 C.',
                                                         'Ambient temperature is whatever was supplied with --ambient; it matters for every number here.'])


def phase_battery():
    """Run until the bank cuts out. Written last because the Pi loses power at the end; every row is fsync'd."""
    d = P('10_battery')
    jwrite(d / 'battery_run_start.json', {'t': time.time(), 'iso': time.strftime('%Y-%m-%dT%H:%M:%S'), 'note': 'bank percentage at start: [HUMAN] note it before leaving', 'args': vars(S.args)})
    f = open(d / 'battery_run.csv', 'a', newline='')
    w = csv.writer(f)
    if f.tell() == 0:
        w.writerow(['t', 'uptime_s', 'ext5v_v', 'est_input_a', 'soc_c', 'fused_hz_10s', 'merged_hz_10s', 'throttled_hex', 'external_clients'])
    vs = start_viewers()
    log('battery run-down started: the Pi will lose power at the end of this phase')
    while True:
        t = time.time()
        row = list(S.power)[-1] if S.power else None
        w.writerow([f'{t:.1f}', f'{float(open("/proc/uptime").read().split()[0]):.0f}', f'{row[1]:.3f}' if row else '', f'{row[2]:.3f}' if row else '', f'{soc_temp():.1f}',
                    f'{hz_between("fused", t - 10, t):.2f}', f'{hz_between("merged", t - 10, t):.2f}', hex(max(throttled_flags(), 0)), len(external_clients())])
        f.flush()
        os.fsync(f.fileno())
        probe_alive()
        time.sleep(5)


# ================================================================== finalisation
def wifi_summary():
    out = {}
    d = S.out / '14_wireless'
    try:
        a = np.loadtxt(d / 'wifi_clients_tcp.csv', delimiter=',', skiprows=1, usecols=(0, 4, 5, 8, 10), ndmin=2)
        if len(a):
            out['client_tcp'] = {'samples': int(len(a)), 'rtt_ms': pstats(a[:, 1][~np.isnan(a[:, 1])]), 'delivery_rate_mbps': pstats(a[:, 4][~np.isnan(a[:, 4])]),
                                 'retransmitted_segments_total_end': float(np.nanmax(a[:, 3])), 'seconds_with_a_client': int(len(a))}
    except Exception:
        pass
    try:
        a = np.loadtxt(d / 'wifi_ping.csv', delimiter=',', skiprows=1, usecols=(0, 2), ndmin=2)
        if len(a):
            ok = ~np.isnan(a[:, 1])
            out['ping_to_clients'] = {'sent': int(len(a)), 'lost_pct': float(100 * (1 - ok.mean())), 'rtt_ms': pstats(a[ok, 1]) if ok.any() else None,
                                      'note': 'clients (e.g. Windows) may block ping: 100% loss then means "no reply", not a dead link'}
    except Exception:
        pass
    try:
        a = np.loadtxt(d / 'server_stats.csv', delimiter=',', skiprows=1, ndmin=2)
        if len(a) > 1:
            out['server_counters'] = {'sent_200': float(a[-1, 2] - a[0, 2]), 'not_modified_304': float(a[-1, 3] - a[0, 3]), 'aborted_by_client': float(a[-1, 4] - a[0, 4]),
                                      'abort_pct': float(100 * (a[-1, 4] - a[0, 4]) / max(a[-1, 2] - a[0, 2] + a[-1, 4] - a[0, 4], 1)), 'bytes_sent_mb': float((a[-1, 5] - a[0, 5]) / 1e6),
                                      'worst_single_write_s': float(a[:, 7].max())}
    except Exception:
        pass
    try:
        a = np.loadtxt(d / 'wifi_link.csv', delimiter=',', skiprows=1, usecols=(0, 4, 5, 6, 7, 8), ndmin=2)
        if len(a):
            out['link'] = {'max_stations': float(a[:, 1].max()), 'tx_errors_total': float(a[-1, 4] - a[0, 4]), 'tx_dropped_total': float(a[-1, 5] - a[0, 5]),
                           'tx_mb': float((a[-1, 2] - a[0, 2]) / 1e6), 'rx_mb': float((a[-1, 3] - a[0, 3]) / 1e6)}
    except Exception:
        pass
    jwrite(d / 'wifi_session_summary.json', out)
    return out


def power_by_phase():
    rows = []
    try:
        pa = np.loadtxt(S.out / '09_power' / 'power_raw.csv', delimiter=',', skiprows=1, ndmin=2)
    except Exception:
        return
    for ph in S.status_log:
        a = pa[(pa[:, 0] >= ph['start']) & (pa[:, 0] <= ph['end'])]
        if len(a):
            rows.append([ph['phase'], len(a), f'{np.nanmean(a[:, 2]):.3f}', f'{np.nanpercentile(a[:, 2], 99):.3f}', f'{np.nanmax(a[:, 2]):.3f}', f'{np.nanmax(a[:, 3]):.3f}', f'{np.nanmean(a[:, 4]):.2f}',
                         f'{np.nanmin(a[:, 1]):.3f}', f'{np.nanmax(a[:, 8]):.1f}'])
    write_csv(P('09_power', 'power_by_phase.csv'), ['phase', 'samples', 'est_input_a_mean', 'est_input_a_p99', 'est_input_a_max', 'cons_input_a_max', 'power_w_mean', 'ext5v_min_v', 'soc_c_max'], rows)
    if len(pa):
        num('current_peak_a', float(np.nanmax(pa[:, 2])), 'A', '09_power/power_raw.csv', 'estimated from the PMIC rails (no input shunt): validate with a USB meter')
        num('current_cons_peak_a', float(np.nanmax(pa[:, 3])), 'A', '09_power/power_raw.csv', 'conservative estimate I/0.85 + 0.5 A')
        num('current_mean_a', float(np.nanmean(pa[:, 2])), 'A', '09_power/power_raw.csv')
        num('power_mean_w', float(np.nanmean(pa[:, 4])), 'W', '09_power/power_raw.csv')
        num('ext5v_min_v', float(np.nanmin(pa[:, 1])), 'V', '09_power/power_raw.csv')
    jwrite(P('09_power', 'summary.json'), {'phases': rows, 'note': 'est_input_a = sum(rail V*I)/EXT5V from vcgencmd pmic_read_adc (~30 Hz); no direct input-current channel exists on the Pi 5. The USB-meter reading is a [HUMAN] item.'})
    write_readme('09_power', '9. Power and current', ['power_raw.csv (gzipped at the end if large): every sample; power_by_phase.csv: per test phase; markers.jsonl in 00_meta has all phase boundaries.'])


# ================================================================== figures
def make_figures(root):
    """Standard graphs from the saved CSVs (each one is independent: a missing file only skips that figure)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root = Path(root)
    fig_dir = root / 'figures'
    fig_dir.mkdir(exist_ok=True)
    made = []
    try:
        phases = json.load(open(root / 'phase_status.json'))
    except Exception:
        phases = []

    def rows(path):
        try:
            with open(root / path) as fh:
                return list(csv.DictReader(fh))
        except Exception:
            return []

    def arr(rs, key):
        out = []
        for r in rs:
            try:
                out.append(float(r[key]))
            except (ValueError, KeyError, TypeError):
                out.append(np.nan)
        return np.array(out)

    def bands(ax, t0):
        for i, ph in enumerate(phases):
            ax.axvspan((ph['start'] - t0) / 60, (ph['end'] - t0) / 60, color=('#dde6f2' if i % 2 == 0 else '#f2e6dd'), alpha=0.6, lw=0)
            ax.text((ph['start'] - t0) / 60, 1.01, ph['phase'], transform=ax.get_xaxis_transform(), fontsize=6, rotation=45, ha='left', va='bottom')

    def save(fig, name):
        fig.tight_layout()
        fig.savefig(fig_dir / name, dpi=110)
        plt.close(fig)
        made.append(name)

    sysr = rows('00_meta/system_1hz.csv')
    t0 = arr(sysr, 't')[0] if sysr else (phases[0]['start'] if phases else 0)
    try:
        fig, ax = plt.subplots(figsize=(12, 4.5))
        for key, lab in (('lidar_raw', 'LiDAR'), ('rgb', 'RGB'), ('fused', 'fused'), ('merged', 'merged')):
            a = np.loadtxt(root / '02_rates' / f'topic_{key}.csv', delimiter=',', skiprows=1, usecols=0, ndmin=1)
            if len(a):
                edges = np.arange(t0, a.max() + 10, 10)
                ax.plot((edges[:-1] - t0) / 60, np.histogram(a, edges)[0] / 10.0, label=lab, lw=1)
        bands(ax, t0)
        ax.set_xlabel('minutes from start')
        ax.set_ylabel('messages per second (10 s bins)')
        ax.set_title('Topic rates across the whole test')
        ax.legend(loc='lower right', fontsize=8)
        save(fig, 'F01_rates_over_time.png')
    except Exception as exc:
        log(f'figure F01: {exc!r}')
    try:
        fig, ax = plt.subplots(figsize=(12, 4.5))
        t = (arr(sysr, 't') - t0) / 60
        ax.plot(t, arr(sysr, 'soc_c'), color='tab:red', lw=1)
        ax.set_ylabel('SoC temperature (C)', color='tab:red')
        ax2 = ax.twinx()
        ax2.plot(t, arr(sysr, 'arm_mhz'), color='tab:blue', lw=0.8)
        ax2.set_ylabel('ARM clock (MHz)', color='tab:blue')
        bands(ax, t0)
        ax.set_xlabel('minutes from start')
        ax.set_title('Temperature and clock (a falling clock at high temperature means throttling)')
        save(fig, 'F02_temperature_clock.png')
    except Exception as exc:
        log(f'figure F02: {exc!r}')
    try:
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
        t = (arr(sysr, 't') - t0) / 60
        stack = [np.nan_to_num(arr(sysr, f'{n}_cpu')) for n in PIPE_PROCS]
        a1.stackplot(t, stack, labels=[n.replace('_node', '') for n in PIPE_PROCS], lw=0)
        a1.set_ylabel('CPU, % of one core')
        a1.legend(loc='upper right', fontsize=6, ncol=4)
        a1.set_title('CPU by process (stacked) and per core')
        for i in range(4):
            a2.plot(t, arr(sysr, f'cpu{i}'), lw=0.7, label=f'core {i}')
        a2.set_ylabel('% busy')
        a2.set_xlabel('minutes from start')
        a2.legend(fontsize=7, ncol=4)
        bands(a1, t0)
        save(fig, 'F03_cpu.png')
    except Exception as exc:
        log(f'figure F03: {exc!r}')
    try:
        pa = np.loadtxt(root / '09_power' / 'power_raw.csv', delimiter=',', skiprows=1, ndmin=2)[::15]
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        t = (pa[:, 0] - t0) / 60
        a1.plot(t, pa[:, 1], lw=0.6)
        a1.axhline(4.75, color='r', ls='--', lw=0.8)
        a1.set_ylabel('EXT5V (V)')
        a2.plot(t, pa[:, 2], lw=0.6, label='estimated input')
        a2.plot(t, pa[:, 3], lw=0.6, label='conservative')
        a2.axhline(5.0, color='r', ls='--', lw=0.8)
        a2.set_ylabel('current (A)')
        a2.set_xlabel('minutes from start')
        a2.legend(fontsize=7)
        bands(a1, t0)
        save(fig, 'F04_power.png')
    except Exception as exc:
        log(f'figure F04: {exc!r}')
    try:
        fig, ax = plt.subplots(figsize=(12, 3.5))
        ax.plot((arr(sysr, 't') - t0) / 60, arr(sysr, 'pipeline_rss_mb'), label='pipeline RSS (MB)')
        ax.plot((arr(sysr, 't') - t0) / 60, arr(sysr, 'mem_available_mb'), label='memory available (MB)')
        bands(ax, t0)
        ax.legend(fontsize=7)
        ax.set_xlabel('minutes from start')
        save(fig, 'F05_memory.png')
    except Exception as exc:
        log(f'figure F05: {exc!r}')
    try:
        fig, axs = plt.subplots(2, 3, figsize=(13, 7))
        for r, which in enumerate(('merged', 'live')):
            rs = rows(f'06_tradeoffs/sweep_{which}.csv')
            if not rs:
                continue
            caps = arr(rs, 'cap')
            for c, (key, lab) in enumerate((('mbit_s', 'link needed (Mbit/s)'), ('coverage_5cm_pct', '5 cm coverage (%)'), ('updates_per_s', 'updates per second'))):
                axs[r, c].plot(caps, arr(rs, key), 'o-')
                axs[r, c].set_xlabel(f'{which} point cap')
                axs[r, c].set_ylabel(lab)
                axs[r, c].grid(alpha=0.3)
        fig.suptitle('Point cap trade-offs (top merged, bottom live)')
        save(fig, 'F06_point_cap_tradeoffs.png')
    except Exception as exc:
        log(f'figure F06: {exc!r}')
    try:
        rs = rows('03_timing_stages/stage_timing.csv')
        keep = [r for r in rs if r['stage'] in ('projection_rgb', 'colouring_rgb', 'fusion_step_total', 'icp_registration_per_scan', 'merge_window', 'streamer_encode_v1', 'streamer_encode_v2')]
        fig, ax = plt.subplots(figsize=(11, 6))
        labels = [f"{r['stage']} | {r['setting']}" for r in keep]
        med, p95 = arr(keep, 'median_ms'), arr(keep, 'p95_ms')
        y = np.arange(len(keep))
        ax.barh(y, med, color='tab:blue', label='median')
        ax.errorbar(med, y, xerr=[np.zeros(len(keep)), np.maximum(p95 - med, 0)], fmt='none', ecolor='k', lw=0.8, label='p95')
        ax.set_yticks(y)
        ax.set_yticklabels(labels, fontsize=7)
        ax.set_xscale('log')
        ax.set_xlabel('ms (log scale)')
        ax.legend(fontsize=7)
        save(fig, 'F07_stage_timing.png')
    except Exception as exc:
        log(f'figure F07: {exc!r}')
    try:
        rs = rows('04_freshness/freshness_samples.csv')
        fig, ax = plt.subplots(figsize=(8, 5))
        for view in ('live', 'merged', 'merged_v2'):
            v = np.sort([float(r['age_ms']) for r in rs if r['view'] == view and r['age_ms'] not in ('', 'nan')])
            if len(v):
                ax.plot(v, np.arange(1, len(v) + 1) / len(v), label=f'{view} (median {np.median(v):.0f} ms)')
        ax.set_xlabel('age of data at the client (ms)')
        ax.set_ylabel('cumulative share')
        ax.legend()
        ax.grid(alpha=0.3)
        save(fig, 'F08_freshness_cdf.png')
    except Exception as exc:
        log(f'figure F08: {exc!r}')
    try:
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4))
        co = rows('07_time_sync/clock_offset.csv')
        if co:
            tt = arr(co, 't_arrival')
            off = arr(co, 'raw_offset_s')
            a1.plot((tt - tt[0]) / 60, (off - np.nanmin(off)) * 1000, lw=0.5)
            a1.set_xlabel('minutes')
            a1.set_ylabel('raw LiDAR clock offset above its minimum (ms)')
        sg = rows('07_time_sync/sync_gap.csv')
        if sg:
            a2.hist(arr(sg, 'rgb_gap_ms'), bins=50)
            a2.set_xlabel('LiDAR scan to nearest RGB frame (ms)')
        save(fig, 'F09_time_sync.png')
    except Exception as exc:
        log(f'figure F09: {exc!r}')
    try:
        tcp = rows('14_wireless/wifi_clients_tcp.csv')
        st = rows('14_wireless/server_stats.csv')
        fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        if tcp:
            tt = (arr(tcp, 't') - t0) / 60
            axs[0].plot(tt, arr(tcp, 'rtt_ms'), '.', ms=2)
            axs[0].set_ylabel('client TCP rtt (ms)')
            axs[1].plot(tt, arr(tcp, 'delivery_rate_mbps'), '.', ms=2)
            axs[1].set_ylabel('delivery rate (Mbit/s)')
        if st:
            axs[2].plot((arr(st, 't') - t0) / 60, arr(st, 'aborted_by_client'), label='frames abandoned by clients')
            axs[2].plot((arr(st, 't') - t0) / 60, arr(st, 'sent_200') / 10.0, label='frames sent / 10')
            axs[2].legend(fontsize=7)
        axs[2].set_xlabel('minutes from start')
        axs[0].set_title('Real client on the access point (empty if no client was connected)')
        save(fig, 'F10_wifi_client.png')
    except Exception as exc:
        log(f'figure F10: {exc!r}')
    try:
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.5))
        ho, cd = rows('12_thermal_enclosure/heat_overload.csv'), rows('12_thermal_enclosure/cooldown.csv')
        hn = rows('12_thermal_enclosure/heat_normal.csv')
        if hn:
            a1.plot(arr(hn, 't_s') / 60, arr(hn, 'soc_c'), label='normal use')
        if ho:
            a1.plot(arr(ho, 't_s') / 60, arr(ho, 'soc_c'), label='overload (4 busy workers)')
        a1.set_xlabel('minutes')
        a1.set_ylabel('SoC (C)')
        a1.legend(fontsize=7)
        if cd:
            a2.plot(arr(cd, 't_s') / 60, arr(cd, 'soc_c'))
            a2.set_title('cool-down after overload')
            a2.set_xlabel('minutes')
        save(fig, 'F11_heat.png')
    except Exception as exc:
        log(f'figure F11: {exc!r}')
    try:
        rs = rows('08_accuracy/icp_trials.csv')
        fig, axs = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)
        for ax, grid in zip(axs, (0.1, 0.2)):
            sel = [r for r in rs if abs(float(r['grid_m']) - grid) < 1e-6]
            for rot in sorted({float(r['rot_deg']) for r in sel}):
                pts = {}
                for r in sel:
                    if float(r['rot_deg']) == rot:
                        pts.setdefault(float(r['move_cm']), []).append(float(r['err_mm']))
                ms = sorted(pts)
                ax.plot(ms, [np.median(pts[m]) for m in ms], 'o-', label=f'{rot} deg')
            ax.set_title(f'merge grid {grid} m')
            ax.set_xlabel('movement between scans (cm)')
            ax.set_yscale('log')
            ax.legend(fontsize=7)
        axs[0].set_ylabel('median merging error (mm)')
        save(fig, 'F12_icp_accuracy.png')
    except Exception as exc:
        log(f'figure F12: {exc!r}')
    try:
        rs = rows('14_wireless/viewer_scaling.csv')
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4))
        v = arr(rs, 'viewers')
        a1.bar(range(len(v)), arr(rs, 'total_mbit_s_served'), tick_label=[int(x) for x in v])
        a1.set_xlabel('viewers')
        a1.set_ylabel('total Mbit/s served')
        a2.plot(v, arr(rs, 'response_p95_ms'), 'o-', label='p95')
        a2.plot(v, arr(rs, 'response_p50_ms'), 'o-', label='p50')
        a2.set_xlabel('viewers')
        a2.set_ylabel('response (ms)')
        a2.legend()
        save(fig, 'F13_viewer_scaling.png')
    except Exception as exc:
        log(f'figure F13: {exc!r}')
    try:
        pr = rows('08_accuracy/lidar_plane_raw.csv')
        fig, ax = plt.subplots(figsize=(11, 3.8))
        ax.plot(arr(pr, 't_min'), arr(pr, 'distance_m'), lw=0.6)
        ax.set_xlabel('minutes')
        ax.set_ylabel('distance to the dominant plane (m)')
        save(fig, 'F14_lidar_stability.png')
    except Exception as exc:
        log(f'figure F14: {exc!r}')
    try:
        cv = rows('15_fusion_info/coverage.csv')
        fig, ax = plt.subplots(figsize=(11, 3.8))
        for key in ('pct_rgb', 'pct_thermal', 'pct_both', 'pct_neither'):
            ax.plot((arr(cv, 't_arrival') - t0) / 60, arr(cv, key), lw=0.7, label=key)
        ax.set_xlabel('minutes')
        ax.set_ylabel('% of fused points')
        ax.legend(fontsize=7)
        save(fig, 'F15_fusion_coverage.png')
    except Exception as exc:
        log(f'figure F15: {exc!r}')
    try:
        bt = [r for r in rows('13_startup_reliability/boot_timeline.csv') if r.get('boot_index(0=current)') == '0']
        fig, ax = plt.subplots(figsize=(10, 4))
        names = [r['event'].split(' (')[0] for r in bt]
        ax.barh(range(len(bt)), [float(r['t_s_after_first_journal_entry']) for r in bt])
        ax.set_yticks(range(len(bt)))
        ax.set_yticklabels(names, fontsize=8)
        ax.set_xlabel('seconds after boot')
        save(fig, 'F16_boot_timeline.png')
    except Exception as exc:
        log(f'figure F16: {exc!r}')
    (fig_dir / 'FIGURES.md').write_text('# Figures made automatically\n\n' + '\n'.join(f'- {n}' for n in made) + '\n\nEach is drawn from the CSVs in the numbered folders; regenerate with `python3 TestWithLidV1.py --figures <tests_final folder>`.\n')
    log(f'{len(made)} figures written to {fig_dir}')



def human_files():
    P('16_human_tests', 'CHECKLIST.md').write_text("""# Items that need a person (not part of the unattended hotspot field test)

1. **Calibration (thesis section 1):** RGB and thermal intrinsics with the full-size 6x4 ChArUco board, LiDAR-to-camera extrinsics (>= 10 poses), alignment accuracy at 2.5/4/6/10/15-20 m,
   then installing the values. Do this first and re-run the tests that depend on it. (The current calibration was made 14 Nov with the 5x7 plain checkerboard, not in the enclosure.)
2. 8.3 Physical movement check of the ICP merge (ruler and a measured turn).
3. 9: USB-C power meter readings at the bank output (idle, scanning, streaming). 10: write down the bank percentage display at the start of the battery run.
4. 12d: ambient room temperature, air temperature inside the enclosure, whether the lid felt warm, any direct sun. (Pass --ambient when you start the test.)
5. 13.1: a true power-cycle boot timeline (3 repeats) and 13.6 unplugging/replugging the thermal camera.
6. 14.2: range test on the hotspot at 1/10/20/30 m and behind 1/2 walls (throughput, viewer rate, data age, usable yes/no). The unattended run records the real client's link statistics but cannot move it.
7. 15.3/15.4: named scenes (warm object in view, outdoors handheld, outdoors static) and one or two sentences about what each layer showed.
8. 16.1 stopwatch end-to-end latency, 16.2 mass and dimensions, 16.3 photos, 16.4 costs.
""")
    P('16_human_tests', 'stopwatch.csv').write_text('trial,displayed_ms,real_ms,delay_ms\n')


def thermal_rerun():
    st, hz = thermal_status()
    lines = ['# Tests whose thermal-specific numbers must be repeated with a replacement thermal camera', '', f'Thermal status during this run: **{st}** ({hz} Hz in the last 10 s of the run).', '',
             '- 02_rates: thermal rate and the thermal interval statistics; the "LiDAR + RGB only" column is the whole run if thermal was absent.',
             '- 03_timing_stages: thermal projection/assignment inside the fusion step (the total here is RGB-only if thermal was absent).',
             '- 05_cpu_memory and 06_tradeoffs: CPU/rates include no thermal processing if absent.', '- 07_time_sync: thermal gap (sync_gap.csv thermal column).',
             '- 11_storage: raw thermal GB/hour (0 if absent). 12_thermal_enclosure: heat and rates with thermal load included.', '- 13_startup_reliability: thermal start-up and fall-back/resume test (13.6).',
             '- 15_fusion_info: thermal and both-layer coverage (0 if absent), all snapshots show no thermal layer if absent.', '- Section 1.2 thermal calibration and 1.4 thermal accuracy at range (not run at all).']
    (S.out / 'THERMAL_RERUN.md').write_text('\n'.join(lines) + '\n')


def results_md():
    L = ['# RESULTS (TestWithLidV1, unattended hotspot field test)', '',
         f'Run id: {S.args.run_id}. Started {time.strftime("%Y-%m-%d %H:%M", time.localtime(S.t_start))}. Profile **{S.args.profile}** (time scale {S.scale}). Hotspot: **{S.hotspot.get("status")}**.',
         f'Thermal status at the end: {thermal_status()[0]}. Ambient: {S.args.ambient}. Enclosure: {S.args.enclosure}. Power: {S.args.power_source}.', '', '## Phase status', '',
         '| phase | status | minutes | note |', '|---|---|---|---|']
    for ph in S.status_log:
        L.append(f'| {ph["phase"]} | {ph["status"]} | {ph["minutes"]} | {ph["detail"][:120]} |')
    sj = S.out / '00_meta' / 'settle.json'
    if sj.exists():
        L += ['', '## Settle gate at start', '', '```json', sj.read_text(), '```']
    L += ['', '## Numbers (value, previous, source)', '', '| key | value | unit | previous | source | note |', '|---|---|---|---|---|---|']
    for k in ALL_KEYS:
        d = S.numbers.get(k)
        if d and d['value'] is not None:
            v = d['value']
            v = f'{v:.4g}' if isinstance(v, float) else v
            L.append(f'| {k} | {v} | {d["unit"]} | {d["previous"]} | {d["source_file"]} | {d["note"]} |')
    missing = [k for k in ALL_KEYS if not (S.numbers.get(k) and S.numbers[k]['value'] is not None)]
    L += ['', '## Not measured in this run', '', ', '.join(missing), '', '(see 16_human_tests/CHECKLIST.md and THERMAL_RERUN.md for why)', '']
    gates = S.out / '00_meta' / 'gate_events.jsonl'
    L += ['## Safety gate events', '', gates.read_text() if gates.exists() else 'none', '', '## Wireless session (from the access-point side)', '', '```json', json.dumps(wifi_summary(), indent=1), '```', '']
    for sec, key in (('Rates', 'rates'), ('Normal use / heat', 'normal_use'), ('Overload', 'overload'), ('Tradeoffs', 'tradeoffs'), ('Startup', 'startup')):
        if key in S.summaries:
            L += [f'## {sec} (summary json)', '', '```json', json.dumps(S.summaries[key], indent=1, default=str)[:6000], '```', '']
    (S.out / 'RESULTS.md').write_text('\n'.join(L))


def index_md():
    L = ['# Index of every file in this run', '']
    for f in sorted(S.out.rglob('*')):
        if f.is_file():
            L.append(f'- `{f.relative_to(S.out)}` ({f.stat().st_size / 1e6:.2f} MB)')
    (S.out / 'INDEX.md').write_text('\n'.join(L) + '\n')


def finalize():
    log('=== FINALISING ===')
    S.phase = 'finalising'
    kill_stray_workers()
    try:
        stop_probe()
    except Exception:
        pass
    for th in S.threads:
        th.stop_flag = True
    time.sleep(2)
    for fn in (power_by_phase, human_files, thermal_rerun):
        try:
            fn()
        except Exception as exc:
            log(f'finalise step {fn.__name__} failed: {exc!r}')
    try:
        make_figures(S.out)
    except Exception as exc:
        log(f'figures failed: {exc!r}')
    num_dump = {k: S.numbers.get(k, {'value': None, 'unit': '', 'previous': PREVIOUS.get(k), 'source_file': '', 'note': 'not measured in this run'}) for k in ALL_KEYS}
    jwrite(S.out / 'thesis_numbers.json', num_dump)
    try:
        results_md()
    except Exception as exc:
        log(f'RESULTS.md failed: {exc!r}')
    for f in list(S.out.rglob('*.csv')):
        if f.stat().st_size > 20e6:
            with open(f, 'rb') as src, gzip.open(str(f) + '.gz', 'wb') as dst:
                shutil.copyfileobj(src, dst)
            f.unlink()
    for f in S.out.rglob('_capture_*.npz'):
        f.unlink()
    index_md()
    zpath = S.out.parent / f'tests_final_{S.args.run_id}.zip'
    with zipfile.ZipFile(zpath, 'w', zipfile.ZIP_DEFLATED) as z:
        for f in S.out.rglob('*'):
            if f.is_file():
                z.write(f, f.relative_to(S.out.parent))
    log(f'zip written: {zpath} ({zpath.stat().st_size / 1e6:.1f} MB)')
    if S.args.return_home and S.hotspot.get('requested'):
        try:
            hotspot_down()
        except Exception as exc:
            log(f'return to home WiFi failed: {exc!r}')
    jwrite(S.out / 'DONE.json', {'finished': time.time(), 'iso': time.strftime('%Y-%m-%dT%H:%M:%S'), 'zip': str(zpath), 'hotspot_left_running': not S.args.return_home})


def top_snapshot():
    """The busiest processes right now (second iteration of top, so the percentages are real)."""
    rc, out = sh('top -b -n 2 -d 1 -w 512', 20)
    last = out.split('top - ')[-1].splitlines()
    idx = next((i for i, l in enumerate(last) if l.strip().startswith('PID')), None)
    return [re.sub(r'\s+', ' ', l.strip())[:140] for l in last[idx + 1: idx + 9]] if idx is not None else []


def settle_gate(max_s=600):
    """Wait until the pipeline is healthy and the machine quiet (boot-time jobs, desktop, editors can starve it), then start."""
    t0, good = time.time(), 0
    while time.time() - t0 < max_s:
        check_stop()
        st = probe_status()

        def hz(k):
            return (st.get(k) or {}).get('hz_10s', 0) or 0
        ok = hz('lidar_raw') >= 9.5 and hz('rgb') >= 9.0 and hz('fused') >= 9.0
        good = good + 1 if ok else 0
        if good >= 3:
            break
        time.sleep(10)
    settled = good >= 3
    jwrite(P('00_meta', 'settle.json'), {'settled': settled, 'waited_s': round(time.time() - t0), 'rates_at_start': {k: (probe_status().get(k) or {}).get('hz_10s') for k in ('lidar_raw', 'rgb', 'fused', 'merged')},
                                         'busiest_processes': top_snapshot(), 'note': 'tests started when lidar/rgb/fused were all near their nominal rates for 30 s; settled=false means they never were within the wait limit'})
    log(f'settle gate: settled={settled} after {time.time() - t0:.0f} s')


def status_thread():
    ticks = 0
    while not S.stop_flag:
        ticks += 1
        try:
            if ticks % 3 == 1:
                with open(P('00_meta', 'top_processes.log'), 'a') as fh:
                    fh.write(f'--- {time.strftime("%H:%M:%S")} phase={S.phase}\n' + '\n'.join(top_snapshot()) + '\n')
            tc = soc_temp()
            row = list(S.power)[-1] if S.power else None
            done = sum(1 for p in S.status_log)
            jwrite(S.out / 'STATUS.json', {'now': time.strftime('%H:%M:%S'), 'phase': S.phase, 'phases_done': done, 'phases_total': len(S.phases), 'elapsed_min': round((time.time() - S.t_start) / 60, 1),
                                           'soc_c': round(tc, 1), 'ext5v': row[1] if row else None, 'est_input_a': round(row[2], 2) if row else None, 'hotspot': S.hotspot.get('status'),
                                           'ap_stations': S.ap_clients, 'external_clients': len(external_clients()), 'gate': S.gate_reason, 'thermal': thermal_status()[0]})
            probe_alive() if S.probe_proc else None
            for i, th in enumerate(list(S.threads)):
                quiet = (type(th) is PowerThread and S.power and time.time() - S.power[-1][0] > 20)
                if quiet and not S.stop_flag and S.phase != 'finalising':
                    log('power sampler went quiet (hung?): starting a new one')
                    th.stop_flag = True
                    new = PowerThread()
                    new.start()
                    S.threads[i] = new
        except Exception:
            pass
        time.sleep(10)


# ================================================================== main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', choices=['full', 'short', 'quick'], default='full')
    ap.add_argument('--countdown', type=int, default=120)
    ap.add_argument('--no-hotspot', action='store_true')
    ap.add_argument('--return-home', action='store_true', help='at the end switch back to home WiFi (Patelnet) and restart the keepalive')
    ap.add_argument('--battery-run', action='store_true', help='LAST phase: run until the bank cuts out')
    ap.add_argument('--only', default='')
    ap.add_argument('--skip', default='')
    ap.add_argument('--ambient', type=float, default=None)
    ap.add_argument('--enclosure', default='closed, clear lid')
    ap.add_argument('--power-source', default='Anker 737 via PD trigger + 52Pi')
    ap.add_argument('--cold-wait-min', type=float, default=0.0)
    ap.add_argument('--ap-band', default='')
    ap.add_argument('--ap-channel', default='')
    ap.add_argument('--run-id', default=time.strftime('%Y%m%d_%H%M%S'))
    ap.add_argument('--session', default='', help='group several runs (for example two halves) under results/<session>/')
    ap.add_argument('--part', default='', help='folder name for this run inside the session (for example part1_thermal_and_performance)')
    ap.add_argument('--summarize-battery', default='')
    ap.add_argument('--figures', default='', help='only (re)draw the figures for an existing tests_final folder')
    S.args = args = ap.parse_args()
    if args.figures:
        S.out = Path(args.figures)
        make_figures(S.out)
        return
    if args.summarize_battery:
        a = np.loadtxt(args.summarize_battery, delimiter=',', skiprows=1, usecols=(0, 1, 2, 3), ndmin=2)
        print(json.dumps({'runtime_h': (a[-1, 0] - a[0, 0]) / 3600.0, 'last_uptime_s': a[-1, 1], 'last_ext5v': a[-1, 2], 'mean_est_input_a': float(np.nanmean(a[:, 3])), 'rows': len(a)}))
        return
    S.scale = {'full': 1.0, 'short': 0.3, 'quick': 0.04}[args.profile]
    S.out = (HERE / 'results' / args.session / (args.part or args.run_id) / 'tests_final') if args.session else (HERE / 'results' / args.run_id / 'tests_final')
    S.out.mkdir(parents=True, exist_ok=True)
    S.markers_path = str(S.out / '00_meta' / 'markers.jsonl')
    P('00_meta')
    for d in ('01_calibration', '02_rates', '03_timing_stages', '04_freshness', '05_cpu_memory', '06_tradeoffs', '07_time_sync', '08_accuracy', '09_power', '10_battery', '11_storage',
              '12_thermal_enclosure', '13_startup_reliability', '14_wireless', '15_fusion_info', '16_human_tests', 'scripts'):
        P(d)
    (P('01_calibration') / 'README.md').write_text('Calibration needs a person with the board and is NOT part of the unattended field test. See 16_human_tests/CHECKLIST.md item 1. '
                                                     'The calibration files in use during this run are copied to 00_meta.\n')
    for f in ('TestWithLidV1.py', 'probe.py', 'bench.py', 'run_test.sh'):
        if (HERE / f).exists():
            shutil.copy(HERE / f, P('scripts', f))
    if 'FASTRTPS_DEFAULT_PROFILES_FILE' not in os.environ:
        rc, p = sh('find ~/ros2_ws/src -name fastdds_large_buffers.xml | head -1', 20)
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = p
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    phases = [p for p in ALL_PHASES if (not args.only or p in args.only.split(',')) and p not in args.skip.split(',')]
    if args.profile == 'quick':
        phases = [p for p in phases]
    S.phases = phases
    plan = {'run_id': args.run_id, 'profile': args.profile, 'scale': S.scale, 'phases': phases, 'estimated_minutes_full_scale': {p: EST_MIN[p] for p in phases},
            'estimated_total_hours': round(sum(EST_MIN[p] for p in phases) * S.scale / 60.0 + 0.1, 2), 'countdown_s': args.countdown, 'battery_run': args.battery_run, 'args': vars(args)}
    jwrite(S.out / 'PLAN.json', plan)
    signal.signal(signal.SIGTERM, lambda *a: setattr(S, 'stop_flag', True))
    log(f'TestWithLidV1 {args.run_id}: profile {args.profile}, phases {phases}, estimated {plan["estimated_total_hours"]} h; output {S.out}')
    for cls in (PowerThread, SystemSampler, WifiMonitor):
        th = cls()
        th.start()
        S.threads.append(th)
    threading.Thread(target=status_thread, daemon=True).start()
    hs = None
    if not args.no_hotspot:
        hs = threading.Thread(target=lambda: hotspot_up(), daemon=True)
        hs.start()
    else:
        S.hotspot = {'requested': False, 'status': 'skipped (--no-hotspot)'}
    end = time.time() + args.countdown
    try:
        while time.time() < end:
            check_stop()
            left = int(end - time.time())
            jwrite(S.out / 'COUNTDOWN.json', {'seconds_left': left, 'hotspot': S.hotspot.get('status')})
            if left % 10 == 0:
                log(f'countdown: {left} s left (hotspot: {S.hotspot.get("status")}); connect the base station to the PitFusion3D hotspot now')
            time.sleep(1)
        if hs:
            hs.join(timeout=600)
        if wait_ready(240) is None:
            log('WARNING: the pipeline is not serving clouds yet; continuing anyway (phases will record what happens)')
        start_probe()
        time.sleep(15)
        thermal_note = thermal_status()
        log(f'thermal status at start: {thermal_note}')
        settle_gate()
        for name in phases:
            fn = globals()[f'phase_{name}']
            run_phase(name, fn, EST_MIN[name])
            probe_alive()
    except Stop:
        log('stop requested: finalising early')
    except Exception as exc:
        log(f'FATAL in main loop: {exc!r}\n{traceback.format_exc()}')
    finally:
        try:
            finalize()
        except Exception as exc:
            log(f'finalize failed: {exc!r}\n{traceback.format_exc()}')
    if args.battery_run and not S.stop_flag:
        start_probe()
        threads_alive = all(t.is_alive() for t in S.threads)
        if not threads_alive:
            for cls in (PowerThread, SystemSampler):
                th = cls()
                th.start()
                S.threads.append(th)
        phase_battery()


if __name__ == '__main__':
    main()
