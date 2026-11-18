#!/usr/bin/env python3
"""Client side of the hotspot range test. Run it on the laptop (Windows, Linux or macOS) joined to the PitFusion3D hotspot.
Needs only Python 3.8+ (standard library). iperf3 is used when it can be found (same folder, PATH, or --iperf3 path).

    python wifi_range_test.py --positions 1,5,10,wall --repeats 2

At each position it asks you to stand there, then measures for about 3 minutes:
  * the radio link as the laptop sees it (signal, link rate, channel) once a second,
  * round-trip time and loss (HTTP pings at 10 Hz and the system ping),
  * iperf3: TCP downlink and uplink throughput, UDP loss and jitter at two rates (if iperf3 is available),
  * the real application traffic: the same /merged.bin and /live.bin requests a viewer makes (conditional GETs on one keep-alive connection),
    giving the update rate you actually get, how long each update takes, and the stalls.
The Pi logs its side at the same time (signal as the access point sees it, retries, TCP rtt/retransmits). Results are in one folder.
"""
import argparse
import csv
import http.client
import json
import os
import platform
import re
import shutil
import statistics
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

VERSION = '1.0'
CONTROL_PORT = 8082
HTTP_PORT = 8080


# ------------------------------------------------------------------ helpers
def pct(a, q):
    if not a:
        return None
    s = sorted(a)
    k = (len(s) - 1) * q / 100.0
    lo, hi = int(k), min(int(k) + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def med(a):
    return statistics.median(a) if a else None


def r(v, n=2):
    return '' if v is None else round(v, n)


def jget(host, path, timeout=8):
    with urllib.request.urlopen(f'http://{host}:{CONTROL_PORT}{path}', timeout=timeout) as resp:
        return json.loads(resp.read())


def mark(host, label, event, note=''):
    try:
        return jget(host, f'/mark?label={label}&event={event}&note={urllib.request.quote(note)}')
    except Exception as exc:
        print(f'  (could not tell the Pi about {label} {event}: {exc})')
        return {'t': None}


# ------------------------------------------------------------------ the laptop's own view of the radio link
def link_info():
    """One sample of what this machine's WiFi adapter reports. Windows: netsh. Linux: nmcli / iw. Anything else: not available."""
    sysname = platform.system()
    d = {'signal_pct': None, 'signal_dbm_est': None, 'rx_rate_mbps': None, 'tx_rate_mbps': None, 'channel': None, 'bssid': None, 'radio': None, 'ssid': None}
    try:
        if sysname == 'Windows':
            out = subprocess.run(['netsh', 'wlan', 'show', 'interfaces'], capture_output=True, text=True, timeout=6).stdout

            def g(key):
                m = re.search(r'^\s*' + key + r'\s*:\s*(.+)$', out, re.M)
                return m.group(1).strip() if m else None
            sig = g('Signal')
            if sig:
                d['signal_pct'] = float(sig.strip('%'))
                d['signal_dbm_est'] = d['signal_pct'] / 2.0 - 100.0          # Windows' own conversion; an estimate
            d['rx_rate_mbps'] = float(g(r'Receive rate \(Mbps\)') or 'nan') if g(r'Receive rate \(Mbps\)') else None
            d['tx_rate_mbps'] = float(g(r'Transmit rate \(Mbps\)') or 'nan') if g(r'Transmit rate \(Mbps\)') else None
            d['channel'], d['bssid'], d['radio'], d['ssid'] = g('Channel'), g('BSSID'), g('Radio type'), g('SSID')
        elif sysname == 'Linux':
            if shutil.which('iw'):
                dev = subprocess.run('iw dev | awk \'$1=="Interface"{print $2; exit}\'', shell=True, capture_output=True, text=True, timeout=5).stdout.strip()
                out = subprocess.run(['iw', 'dev', dev, 'link'], capture_output=True, text=True, timeout=5).stdout
                m = re.search(r'signal:\s*(-?\d+)', out)
                d['signal_dbm_est'] = float(m.group(1)) if m else None
                d['signal_pct'] = max(0.0, min(100.0, 2 * (d['signal_dbm_est'] + 100))) if m else None
                for key, k2 in (('rx bitrate', 'rx_rate_mbps'), ('tx bitrate', 'tx_rate_mbps')):
                    m = re.search(key + r':\s*([\d.]+)', out)
                    d[k2] = float(m.group(1)) if m else None
                m = re.search(r'Connected to ([0-9a-f:]+)', out)
                d['bssid'] = m.group(1) if m else None
                m = re.search(r'SSID:\s*(.+)', out)
                d['ssid'] = m.group(1).strip() if m else None
                m = re.search(r'freq:\s*(\d+)', out)
                d['channel'] = m.group(1) + ' MHz' if m else None
            elif shutil.which('nmcli'):
                out = subprocess.run(['nmcli', '-t', '-f', 'IN-USE,SSID,BSSID,CHAN,RATE,SIGNAL', 'dev', 'wifi'], capture_output=True, text=True, timeout=8).stdout
                for line in out.splitlines():
                    if line.startswith('*'):
                        parts = re.split(r'(?<!\\):', line)
                        d['ssid'], d['bssid'], d['channel'] = parts[1], parts[2].replace('\\', ''), parts[3]
                        d['rx_rate_mbps'] = float(re.search(r'[\d.]+', parts[4]).group()) if re.search(r'[\d.]+', parts[4]) else None
                        d['signal_pct'] = float(parts[5])
                        d['signal_dbm_est'] = d['signal_pct'] / 2.0 - 100.0
    except Exception:
        pass
    return d


class LinkLogger(threading.Thread):
    def __init__(self, path):
        super().__init__(daemon=True)
        self.path, self.stop_flag, self.label, self.rows = path, False, '', []
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t_unix', 'position', 'signal_pct', 'signal_dbm_est', 'rx_rate_mbps', 'tx_rate_mbps', 'channel', 'bssid', 'radio', 'ssid'])

    def run(self):
        while not self.stop_flag:
            t = time.time()
            d = link_info()
            row = [f'{t:.2f}', self.label, d['signal_pct'], d['signal_dbm_est'], d['rx_rate_mbps'], d['tx_rate_mbps'], d['channel'], d['bssid'], d['radio'], d['ssid']]
            self.rows.append((self.label, t, d))
            self.w.writerow(row)
            self.f.flush()
            time.sleep(max(0.2, 1.0 - (time.time() - t)))

    def summary(self, label):
        ds = [d for lab, t, d in self.rows if lab == label]

        def col(k):
            return [x[k] for x in ds if x.get(k) is not None]
        return {'signal_pct_median': med(col('signal_pct')), 'signal_pct_min': min(col('signal_pct')) if col('signal_pct') else None, 'signal_dbm_est_median': med(col('signal_dbm_est')),
                'rx_rate_mbps_median': med(col('rx_rate_mbps')), 'tx_rate_mbps_median': med(col('tx_rate_mbps')),
                'channel': (col('channel') or [None])[-1], 'bssid': (col('bssid') or [None])[-1], 'radio': (col('radio') or [None])[-1], 'n': len(ds)}


# ------------------------------------------------------------------ round-trip time
def http_ping(host, secs, out_csv):
    """HTTP round trips to the Pi's control server at 10 Hz on one connection (includes TCP retransmission effects). Timeouts count as lost."""
    rows, lost = [], 0
    conn = None
    end = time.time() + secs
    while time.time() < end:
        t = time.time()
        try:
            if conn is None:
                conn = http.client.HTTPConnection(host, CONTROL_PORT, timeout=2)
            conn.request('GET', '/ping')
            conn.getresponse().read()
            rows.append((t, (time.time() - t) * 1000.0))
        except Exception:
            lost += 1
            rows.append((t, None))
            try:
                conn.close()
            except Exception:
                pass
            conn = None
        time.sleep(max(0.0, 0.1 - (time.time() - t)))
    with open(out_csv, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_unix', 'rtt_ms'])
        w.writerows([[f'{a:.3f}', '' if b is None else f'{b:.2f}'] for a, b in rows])
    ok = [b for a, b in rows if b is not None]
    return {'http_rtt_ms_p50': med(ok), 'http_rtt_ms_p95': pct(ok, 95), 'http_rtt_ms_max': max(ok) if ok else None, 'http_ping_loss_pct': 100.0 * lost / max(1, len(rows)), 'http_pings': len(rows)}


def system_ping(host, n=10):
    try:
        if platform.system() == 'Windows':
            out = subprocess.run(['ping', '-n', str(n), '-w', '1000', host], capture_output=True, text=True, timeout=n * 2 + 10).stdout
            m = re.search(r'Average = (\d+)ms', out)
            loss = re.search(r'\((\d+)% loss\)', out)
            mx = re.search(r'Maximum = (\d+)ms', out)
        else:
            out = subprocess.run(['ping', '-c', str(n), '-W', '1', host], capture_output=True, text=True, timeout=n * 2 + 10).stdout
            m = re.search(r'= [\d.]+/([\d.]+)/', out)
            loss = re.search(r'([\d.]+)% packet loss', out)
            mx = re.search(r'= [\d.]+/[\d.]+/([\d.]+)', out)
        return {'icmp_rtt_ms_avg': float(m.group(1)) if m else None, 'icmp_rtt_ms_max': float(mx.group(1)) if mx else None, 'icmp_loss_pct': float(loss.group(1)) if loss else None}
    except Exception:
        return {'icmp_rtt_ms_avg': None, 'icmp_rtt_ms_max': None, 'icmp_loss_pct': None}


# ------------------------------------------------------------------ iperf3
def find_iperf3(user_path):
    for cand in ([user_path] if user_path else []) + [str(Path(__file__).resolve().parent / 'iperf3.exe'), str(Path(__file__).resolve().parent / 'iperf3'), 'iperf3']:
        p = shutil.which(cand) or (cand if Path(cand).exists() else None)
        if p:
            return p
    return None


def run_iperf(exe, host, args, secs, out_json):
    try:
        res = subprocess.run([exe, '-c', host, '-J', '-t', str(secs)] + args, capture_output=True, text=True, timeout=secs + 25)
        Path(out_json).write_text(res.stdout or res.stderr)
        d = json.loads(res.stdout)
        if 'error' in d:
            return {'error': d['error']}
        return d
    except Exception as exc:
        return {'error': repr(exc)}


def iperf_suite(exe, host, label, rep, outdir, secs, udp_rates):
    out = {}
    if not exe:
        return {'iperf3': 'not available (download iperf3, put it next to this script or on PATH)'}
    d = run_iperf(exe, host, ['-R'], secs, outdir / f'iperf3_{label}_{rep}_tcp_down.json')          # Pi -> laptop (the direction the point clouds travel)
    if 'error' in d:
        out['tcp_down_error'] = d['error']
    else:
        e = d['end']
        out['tcp_down_mbps'] = e['sum_received']['bits_per_second'] / 1e6
        out['tcp_down_retransmits_pi'] = e.get('sum_sent', {}).get('retransmits')
    d = run_iperf(exe, host, [], secs, outdir / f'iperf3_{label}_{rep}_tcp_up.json')                # laptop -> Pi
    if 'error' in d:
        out['tcp_up_error'] = d['error']
    else:
        e = d['end']
        out['tcp_up_mbps'] = e['sum_received']['bits_per_second'] / 1e6
        out['tcp_up_retransmits_laptop'] = e.get('sum_sent', {}).get('retransmits')
    for rate in udp_rates:
        d = run_iperf(exe, host, ['-u', '-R', '-b', f'{rate}M'], secs, outdir / f'iperf3_{label}_{rep}_udp_down_{rate}M.json')
        if 'error' in d:
            out[f'udp{rate}_error'] = d['error']
        else:
            s = d['end'].get('sum') or d['end'].get('sum_received') or {}
            out[f'udp{rate}_received_mbps'] = (s.get('bits_per_second') or 0) / 1e6
            out[f'udp{rate}_loss_pct'] = s.get('lost_percent')
            out[f'udp{rate}_jitter_ms'] = s.get('jitter_ms')
    return out


# ------------------------------------------------------------------ the real application traffic
def fetch_loop(host, path, secs, tag, rows, errors):
    """What a viewer does: conditional GETs on one keep-alive connection, polling quickly so that every new frame is picked up at once."""
    conn, etag = None, None
    end = time.time() + secs
    while time.time() < end:
        t = time.time()
        try:
            if conn is None:
                conn = http.client.HTTPConnection(host, HTTP_PORT, timeout=6)
            h = {'Accept-Encoding': 'gzip'}
            if etag:
                h['If-None-Match'] = etag
            conn.request('GET', path, headers=h)
            resp = conn.getresponse()
            body = resp.read()
            now = time.time()
            if resp.status == 200:
                etag = resp.getheader('ETag')
                rows.append((tag, t, now, (now - t) * 1000.0, len(body), float(resp.getheader('X-Cloud-Stamp') or 0), 200))
            elif resp.status == 304:
                rows.append((tag, t, now, (now - t) * 1000.0, len(body), 0.0, 304))
            else:
                errors.append((tag, t, f'HTTP {resp.status}'))
        except Exception as exc:
            errors.append((tag, t, repr(exc)[:80]))
            try:
                conn.close()
            except Exception:
                pass
            conn, etag = None, None
            time.sleep(0.2)
        time.sleep(max(0.0, 0.02 - (time.time() - t)))


def app_stats(rows, errors, secs):
    new = [x for x in rows if x[6] == 200]
    out = {'requests': len(rows), 'errors': len(errors), 'new_frames': len(new), 'update_rate_hz': len(new) / secs if secs else None}
    if len(new) >= 2:
        arr = [new[i + 1][2] - new[i][2] for i in range(len(new) - 1)]
        out.update({'interval_p50_s': med(arr), 'interval_p95_s': pct(arr, 95), 'interval_max_s': max(arr), 'stalls_over_1s': sum(1 for a in arr if a > 1.0), 'stalls_over_2s': sum(1 for a in arr if a > 2.0)})
        stamps = [x[5] for x in new if x[5] > 1e9]
        if len(stamps) >= 3:
            nominal = med([stamps[i + 1] - stamps[i] for i in range(len(stamps) - 1)])
            span = stamps[-1] - stamps[0]
            out['frames_delivered_pct'] = 100.0 * (len(stamps) - 1) / (span / nominal) if nominal and span > 0 else None
            out['server_frame_interval_s'] = nominal
    out['frame_response_ms_p50'] = med([x[3] for x in new])
    out['frame_response_ms_p95'] = pct([x[3] for x in new], 95)
    out['frame_response_ms_max'] = max([x[3] for x in new]) if new else None
    out['kbytes_per_frame'] = med([x[4] / 1000.0 for x in new])
    out['goodput_mbit_s'] = sum(x[4] for x in new) * 8 / 1e6 / secs if secs else None
    return out


def app_suite(host, label, rep, outdir, scale):
    plan = [('merged_fmt1', [('/merged.bin', 'm')], 20), ('merged_fmt2', [('/merged.bin?fmt=2', 'm')], 20), ('live_fmt2', [('/live.bin?fmt=2', 'l')], 15),
            ('merged_and_live_fmt2', [('/merged.bin?fmt=2', 'm'), ('/live.bin?fmt=2', 'l')], 25)]
    res = {}
    for name, paths, secs in plan:
        secs = max(6, secs * scale)
        allrows, allerrs, th = [], [], []
        for path, tag in paths:
            rows, errs = [], []
            allrows.append((tag, rows))
            allerrs.append(errs)
            t = threading.Thread(target=fetch_loop, args=(host, path, secs, tag, rows, errs), daemon=True)
            th.append(t)
            t.start()
        for t in th:
            t.join(secs + 20)
        with open(outdir / f'requests_{label}_{rep}_{name}.csv', 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['stream', 't_send_unix', 't_done_unix', 'response_ms', 'bytes', 'server_cloud_stamp', 'status'])
            for tag, rows in allrows:
                w.writerows([[x[0], f'{x[1]:.3f}', f'{x[2]:.3f}', f'{x[3]:.1f}', x[4], f'{x[5]:.3f}', x[6]] for x in rows])
        for (tag, rows), errs in zip(allrows, allerrs):
            st = app_stats(rows, errs, secs)
            for k, v in st.items():
                res[f'app_{name}_{tag}_{k}'] = v
    return res


# ------------------------------------------------------------------ one position
def measure(host, label, rep, desc, outdir, link, exe, scale, udp_rates, notes_extra):
    link.label = f'{label}#{rep}'
    t_start = mark(host, f'{label}#{rep}', 'start', desc).get('t')
    row = {'position': label, 'repeat': rep, 'description': desc, 'time': time.strftime('%Y-%m-%d %H:%M:%S')}
    print(f'  ping ...', flush=True)
    row.update(http_ping(host, max(6, 15 * scale), outdir / f'ping_{label}_{rep}.csv'))
    row.update(system_ping(host, 10 if scale >= 0.5 else 4))
    print(f'  iperf3 ...', flush=True)
    row.update(iperf_suite(exe, host, label, rep, outdir, max(3, int(10 * scale)), udp_rates))
    print(f'  application traffic (merged, live) ...', flush=True)
    row.update(app_suite(host, label, rep, outdir, scale))
    t_end = mark(host, f'{label}#{rep}', 'end').get('t')
    try:
        pi = jget(host, f'/station?t0={t_start}&t1={t_end}') if t_start and t_end else {}
    except Exception as exc:
        pi = {'error': repr(exc)}
    (outdir / f'pi_side_{label}_{rep}.json').write_text(json.dumps(pi, indent=1))
    row['pi_ap_signal_dbm_median'] = ((pi.get('ap_signal_dbm') or {}).get('median'))
    row['pi_ap_tx_retries'] = pi.get('ap_tx_retries_delta')
    row['pi_ap_tx_failed'] = pi.get('ap_tx_failed_delta')
    row['pi_tcp8080_rtt_ms_median'] = (((pi.get('tcp_8080') or {}).get('rtt_ms')) or {}).get('median')
    row['pi_tcp5201_rtt_ms_median'] = (((pi.get('tcp_5201') or {}).get('rtt_ms')) or {}).get('median')
    row['pi_soc_c_max'] = pi.get('soc_c_max')
    ls = link.summary(f'{label}#{rep}')
    row.update({f'client_{k}': v for k, v in ls.items()})
    link.label = ''
    return row


def write_tables(rows, outdir):
    if not rows:
        return
    keys = []
    for rw in rows:
        for k in rw:
            if k not in keys:
                keys.append(k)
    with open(outdir / 'results_all_columns.csv', 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    cols = [('position', 'position'), ('repeat', 'rep'), ('pi_ap_signal_dbm_median', 'signal at the Pi (dBm)'), ('pi_ap_tx_bitrate_median', 'AP tx rate (Mbit/s)'), ('pi_tcp8082_retrans', 'TCP retransmits (Pi)'), ('client_signal_dbm_est_median', 'signal (dBm, est.)'), ('client_signal_pct_median', 'signal (%)'), ('client_rx_rate_mbps_median', 'link rate rx (Mbit/s)'),
            ('client_channel', 'channel'), ('http_rtt_ms_p50', 'RTT p50 (ms)'), ('http_rtt_ms_p95', 'RTT p95 (ms)'), ('http_ping_loss_pct', 'ping loss (%)'),
            ('tcp_down_mbps', 'TCP down (Mbit/s)'), ('tcp_up_mbps', 'TCP up (Mbit/s)'), ('udp5_loss_pct', 'UDP 5M loss (%)'), ('udp5_jitter_ms', 'UDP 5M jitter (ms)'),
            ('udp20_loss_pct', 'UDP 20M loss (%)'), ('app_merged_fmt2_m_update_rate_hz', 'merged update rate fmt2 (Hz)'), ('app_merged_fmt2_m_interval_p95_s', 'merged interval p95 (s)'),
            ('app_merged_fmt2_m_stalls_over_1s', 'merged stalls >1 s'), ('app_merged_fmt2_m_frame_response_ms_p50', 'merged frame time p50 (ms)'),
            ('app_live_fmt2_l_update_rate_hz', 'live update rate (Hz)'), ('app_live_fmt2_l_interval_p95_s', 'live interval p95 (s)'), ('app_live_fmt2_l_stalls_over_1s', 'live stalls >1 s'),
            ('app_merged_and_live_fmt2_m_update_rate_hz', 'merged rate with live (Hz)'), ('app_merged_and_live_fmt2_l_update_rate_hz', 'live rate with merged (Hz)')]
    cols = [(k, n) for k, n in cols if any(k in rw for rw in rows)]

    def fmt(v):
        return '' if v is None else (f'{v:.2f}' if isinstance(v, float) else str(v))
    lines = ['| ' + ' | '.join(n for k, n in cols) + ' |', '|' + '---|' * len(cols)]
    for rw in rows:
        lines.append('| ' + ' | '.join(fmt(rw.get(k)) for k, n in cols) + ' |')
    (outdir / 'results_table.md').write_text('# Hotspot range test (summary table)\n\nSee results_all_columns.csv for every column. Signal in dBm is an estimate from Windows\' percentage (dBm = % / 2 - 100) unless the adapter reports dBm.\n\n' + '\n'.join(lines) + '\n')
    with open(outdir / 'results_table.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow([n for k, n in cols])
        for rw in rows:
            w.writerow([rw.get(k, '') for k, n in cols])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pi', default='10.42.0.1')
    ap.add_argument('--positions', default='1,5,10,wall', help='comma list; numbers are metres in line of sight, any other word (wall, corner, ...) is a described position')
    ap.add_argument('--repeats', type=int, default=2)
    ap.add_argument('--out', default='')
    ap.add_argument('--iperf3', default='')
    ap.add_argument('--udp-rates', default='5,20', help='Mbit/s, comma list')
    ap.add_argument('--quick', action='store_true', help='short test of the script itself (about 15 s per position)')
    ap.add_argument('--auto', action='store_true', help='no prompts (testing)')
    args = ap.parse_args()
    scale = 0.25 if args.quick else 1.0
    outdir = Path(args.out or f'wifi_range_{time.strftime("%Y%m%d_%H%M%S")}')
    outdir.mkdir(parents=True, exist_ok=True)
    try:
        info = jget(args.pi, '/info')
    except Exception as exc:
        sys.exit(f'Cannot reach the Pi control server at {args.pi}:{CONTROL_PORT} ({exc}). Is the laptop joined to the PitFusion3D WiFi and is wifi_range_pi.sh running?')
    exe = find_iperf3(args.iperf3)
    print(f'Connected to the Pi. Hotspot: {info.get("hotspot")}. iperf3 on the Pi: {info.get("iperf3")}. iperf3 here: {exe or "NOT FOUND (iperf3 tests will be skipped)"}.')
    (outdir / 'run_info.json').write_text(json.dumps({'version': VERSION, 'args': vars(args), 'laptop': platform.platform(), 'python': sys.version, 'pi_info': info, 'iperf3_client': exe, 'start': time.strftime('%Y-%m-%d %H:%M:%S')}, indent=1))
    link = LinkLogger(outdir / 'client_link.csv')
    link.start()
    rows = []
    positions = [p.strip() for p in args.positions.split(',') if p.strip()]
    udp_rates = [int(x) for x in args.udp_rates.split(',') if x.strip()]
    for rep in range(1, args.repeats + 1):
        for pos in positions:
            is_num = re.fullmatch(r'[\d.]+', pos) is not None
            desc = f'{pos} m, line of sight' if is_num else pos
            if not args.auto:
                print(f'\n=== Repeat {rep}/{args.repeats}: position "{pos}" ===')
                if is_num:
                    print(f'Stand {pos} m from the Pi (measure with a tape, laptop at the same height as the Pi, antenna side facing it), hold the laptop still.')
                else:
                    extra = input(f'Position "{pos}": describe it (for example "through 1 brick wall, 10 m from the Pi, wall about 20 cm thick") and press Enter: ').strip()
                    desc = f'{pos}: {extra}' if extra else pos
                input('Press Enter when you are in place and holding still...')
            print(f'Measuring {desc} (about {int(3 * scale + 0.5) if scale < 1 else 3} minutes, keep the laptop still) ...', flush=True)
            time.sleep(2)
            rows.append(measure(args.pi, pos, rep, desc, outdir, link, exe, scale, udp_rates, {}))
            write_tables(rows, outdir)
            print(f'  done: signal {r(rows[-1].get("client_signal_dbm_est_median"), 0)} dBm(est), RTT p50 {r(rows[-1].get("http_rtt_ms_p50"), 1)} ms, merged {r(rows[-1].get("app_merged_fmt2_m_update_rate_hz"), 2)} Hz, live {r(rows[-1].get("app_live_fmt2_l_update_rate_hz"), 2)} Hz'
                  + (f', TCP down {r(rows[-1].get("tcp_down_mbps"), 1)} Mbit/s' if rows[-1].get('tcp_down_mbps') is not None else ''))
    link.stop_flag = True
    try:
        with urllib.request.urlopen(f'http://{args.pi}:{CONTROL_PORT}/logs.zip', timeout=60) as resp:
            (outdir / 'pi_logs.zip').write_bytes(resp.read())
    except Exception as exc:
        print(f'(could not download the Pi logs: {exc}; they are on the Pi in ~/TestWithLidV1/wifi_range/)')
    (outdir / 'README.txt').write_text('Hotspot range test. results_table.md/csv: the thesis table. results_all_columns.csv: every measured column. requests_*.csv: every application request. '
                                      'iperf3_*.json: raw iperf3 output. ping_*.csv: HTTP round trips. client_link.csv: the laptop\'s own radio view once a second. pi_side_*.json and pi_logs.zip: the access-point side.\n')
    print(f'\nFinished. Results: {outdir.resolve()}')
    print('Copy this whole folder back to the Pi or send it as it is. Tell the Pi to stop with: ./wifi_range_pi.sh --stop')


if __name__ == '__main__':
    main()
