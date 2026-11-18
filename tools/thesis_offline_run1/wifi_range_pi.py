#!/usr/bin/env python3
"""Pi side of the hotspot range test.

Switches the Pi to its own hotspot (PitFusion3D, 10.42.0.1), restarts the pipeline on it, runs an iperf3 server (if installed),
logs the access-point side (client signal and bitrate if `iw` is installed, retries, per-connection TCP rtt/retransmits, wlan counters) once a second,
and runs a small control server on port 8082 that the client script (wifi_range_test.py) talks to:
  /ping  /mark?label=..&event=..  /station?t0=..&t1=..  /logs.zip  /wifi_range_test.py  /finish
Start with wifi_range_pi.sh. Everything is written to ~/TestWithLidV1/wifi_range/<timestamp>/.
"""
import argparse
import csv
import http.client
import http.server
import json
import os
import re
import shutil
import signal
import socketserver
import subprocess
import sys
import tempfile
import threading
import time
import zipfile
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import parse_qs, urlparse

import numpy as np
import psutil

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import TestWithLidV1 as T  # noqa: E402

S = T.S
STOP = threading.Event()
SESSION = None
STOP_FILE = HERE / 'wifi_range' / 'STOP'
CONTROL_PORT = 8082


def log(msg):
    line = f'[{time.strftime("%Y-%m-%d %H:%M:%S")}] {msg}'
    print(line, flush=True)
    if SESSION:
        with open(SESSION / 'RUN_LOG.txt', 'a') as fh:
            fh.write(line + '\n')


def sh(cmd, timeout=10):
    return T.sh(cmd, timeout)


# ------------------------------------------------------------------ AP-side loggers
def station_loop():
    if not shutil.which('iw'):
        log('iw is not installed: client signal strength is not logged on the Pi side (sudo apt install -y iw). The client script still records its own signal.')
        return
    f = open(SESSION / 'ap_station.csv', 'a', newline='')
    w = csv.writer(f)
    w.writerow(['t', 'mac', 'signal_dbm', 'signal_avg_dbm', 'tx_bitrate_mbps', 'rx_bitrate_mbps', 'tx_retries', 'tx_failed', 'tx_packets', 'rx_packets', 'inactive_ms', 'connected_s'])
    while not STOP.is_set():
        t0 = time.time()
        rc, out = sh(['iw', 'dev', 'wlan0', 'station', 'dump'], 5)
        for block in re.split(r'(?m)^Station ', out)[1:]:
            mac = block.split()[0]

            def g(pat):
                m = re.search(pat, block)
                return m.group(1) if m else ''
            w.writerow([f'{t0:.2f}', mac, g(r'signal:\s+(-?\d+)'), g(r'signal avg:\s+(-?\d+)'), g(r'tx bitrate:\s+([\d.]+)'), g(r'rx bitrate:\s+([\d.]+)'), g(r'tx retries:\s+(\d+)'),
                        g(r'tx failed:\s+(\d+)'), g(r'tx packets:\s+(\d+)'), g(r'rx packets:\s+(\d+)'), g(r'inactive time:\s+(\d+)'), g(r'connected time:\s+(\d+)')])
        f.flush()
        time.sleep(max(0.1, 1.0 - (time.time() - t0)))


def tcp_loop():
    f = open(SESSION / 'ap_tcp.csv', 'a', newline='')
    w = csv.writer(f)
    w.writerow(['t', 'peer_ip', 'peer_port', 'local_port', 'send_q_bytes', 'rtt_ms', 'rttvar_ms', 'cwnd', 'retrans_current', 'retrans_total', 'bytes_acked', 'delivery_rate_mbps'])
    while not STOP.is_set():
        t0 = time.time()
        rc, out = sh("ss -tin state established '( sport = :8080 or sport = :8082 or sport = :5201 )'", 5)
        lines = out.splitlines()[1:]
        i = 0
        while i < len(lines):
            head = lines[i].split()
            info = lines[i + 1] if i + 1 < len(lines) else ''
            i += 2
            if len(head) < 4 or head[3].startswith(('127.', '[::1]')):
                continue
            rtt = re.search(r'rtt:([\d.]+)/([\d.]+)', info)
            rt = re.search(r'retrans:(\d+)/(\d+)', info)
            dr = re.search(r'delivery_rate ([\d.]+)([MKG]?)bps', info)
            cw = re.search(r'cwnd:(\d+)', info)
            ba = re.search(r'bytes_acked:(\d+)', info)
            drm = float(dr.group(1)) * {'': 1e-6, 'K': 1e-3, 'M': 1.0, 'G': 1e3}[dr.group(2)] if dr else ''
            w.writerow([f'{t0:.2f}', head[3].rsplit(':', 1)[0], head[3].rsplit(':', 1)[1], head[2].rsplit(':', 1)[1], head[1], rtt.group(1) if rtt else '', rtt.group(2) if rtt else '', cw.group(1) if cw else '',
                        rt.group(1) if rt else 0, rt.group(2) if rt else 0, ba.group(1) if ba else '', drm])
        f.flush()
        time.sleep(max(0.1, 1.0 - (time.time() - t0)))


def wlan_loop():
    f = open(SESSION / 'ap_wlan.csv', 'a', newline='')
    w = csv.writer(f)
    w.writerow(['t', 'tx_bytes', 'rx_bytes', 'tx_errors', 'tx_dropped', 'soc_c'])
    while not STOP.is_set():
        t0 = time.time()
        n = psutil.net_io_counters(pernic=True).get('wlan0')
        if n:
            w.writerow([f'{t0:.2f}', n.bytes_sent, n.bytes_recv, n.errout, n.dropout, f'{T.soc_temp():.1f}'])
        f.flush()
        time.sleep(max(0.1, 1.0 - (time.time() - t0)))


def iperf_loop():
    exe = shutil.which('iperf3')
    if not exe:
        log('iperf3 is not installed on the Pi: iperf3 tests will be skipped (sudo apt install -y iperf3)')
        return
    while not STOP.is_set():
        lf = open(SESSION / 'iperf3_server.log', 'a')
        p = subprocess.Popen([exe, '-s', '-p', '5201'], stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)
        while not STOP.is_set() and p.poll() is None:
            time.sleep(1)
        if p.poll() is None:
            p.terminate()
        time.sleep(1)


# ------------------------------------------------------------------ summaries for /station
def rows(path):
    try:
        with open(path) as fh:
            return list(csv.DictReader(fh))
    except Exception:
        return []


def fnum(rs, key):
    out = []
    for r in rs:
        try:
            out.append(float(r[key]))
        except (ValueError, KeyError, TypeError):
            pass
    return np.array(out)


def window(rs, t0, t1):
    return [r for r in rs if t0 <= float(r['t']) <= t1]


def stat(a):
    return {'median': float(np.median(a)), 'min': float(a.min()), 'max': float(a.max()), 'p95': float(np.percentile(a, 95)), 'n': int(len(a))} if len(a) else None


def summary(t0, t1, ip=None, mac=None):
    out = {'t0': t0, 't1': t1, 'client_ip': ip, 'client_mac': mac}
    st = window(rows(SESSION / 'ap_station.csv'), t0, t1)
    if mac:
        st = [r for r in st if r['mac'].lower() == mac.lower()]
    out['ap_signal_dbm'] = stat(fnum(st, 'signal_dbm'))
    out['ap_tx_bitrate_mbps'] = stat(fnum(st, 'tx_bitrate_mbps'))
    out['ap_rx_bitrate_mbps'] = stat(fnum(st, 'rx_bitrate_mbps'))
    for key in ('tx_retries', 'tx_failed', 'tx_packets'):
        a = fnum(st, key)
        out[f'ap_{key}_delta'] = float(a[-1] - a[0]) if len(a) > 1 else None
    tcp = window(rows(SESSION / 'ap_tcp.csv'), t0, t1)
    if ip:
        tcp = [r for r in tcp if r['peer_ip'] == ip]
    for port in ('8082', '5201'):
        sub = [r for r in tcp if r['local_port'] == port]
        conns = {}
        for r in sub:
            try:
                conns[r['peer_port']] = max(conns.get(r['peer_port'], 0), float(r['retrans_total']))
            except (ValueError, KeyError):
                pass
        out[f'tcp_{port}'] = {'rtt_ms': stat(fnum(sub, 'rtt_ms')), 'delivery_rate_mbps': stat(fnum(sub, 'delivery_rate_mbps')), 'samples': len(sub), 'connections': len(conns),
                              'retransmits_total': float(sum(conns.values()))}
    wl = window(rows(SESSION / 'ap_wlan.csv'), t0, t1)
    if len(wl) > 1:
        out['wlan_tx_mbit_s_mean'] = float((fnum(wl, 'tx_bytes')[-1] - fnum(wl, 'tx_bytes')[0]) * 8 / 1e6 / max(1e-6, float(wl[-1]['t']) - float(wl[0]['t'])))
        out['wlan_tx_errors_delta'] = float(fnum(wl, 'tx_errors')[-1] - fnum(wl, 'tx_errors')[0])
        out['wlan_tx_dropped_delta'] = float(fnum(wl, 'tx_dropped')[-1] - fnum(wl, 'tx_dropped')[0])
    out['soc_c_max'] = float(np.max(fnum(wl, 'soc_c'))) if len(wl) else None
    return out


def client_mac(ip):
    rc, out = sh(['ip', 'neigh', 'show', ip], 4)
    m = re.search(r'lladdr\s+([0-9a-f:]{17})', out)
    return m.group(1) if m else None


RESULTS = []
RESULTS_LOCK = threading.Lock()
BLOCK = os.urandom(1 << 20)


def save_result(body, ip):
    label, notes, t0 = body.get('label', 'position'), body.get('notes', ''), float(body.get('t0') or time.time() - 120)
    t1 = time.time()
    mac = client_mac(ip)
    pi = summary(t0, t1, ip, mac)
    with RESULTS_LOCK:
        rep = 1 + sum(1 for r in RESULTS if r['position'] == label)
        tag = re.sub(r'[^A-Za-z0-9_.-]+', '_', label)
        (SESSION / f'pi_side_{tag}_{rep}.json').write_text(json.dumps(pi, indent=1))
        raw = body.get('raw', {})
        with open(SESSION / f'ping_{tag}_{rep}.csv', 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['rtt_ms'])
            w.writerows([[f'{x:.2f}'] for x in raw.get('ping', [])])
        for name, streams in (raw.get('app') or {}).items():
            with open(SESSION / f'requests_{tag}_{rep}_{name}.csv', 'w', newline='') as fh:
                w = csv.writer(fh)
                w.writerow(['stream', 't_send_ms_browser_clock', 't_done_ms_browser_clock', 'response_ms', 'bytes', 'server_cloud_stamp'])
                for stag, rws in streams.items():
                    w.writerows([[stag] + [f'{v:.3f}' if isinstance(v, float) else v for v in r] for r in rws])
        (SESSION / f'browser_raw_{tag}_{rep}.json').write_text(json.dumps({'down_ticks_mbit_per_s': raw.get('down_ticks'), 'up_post_ms': raw.get('up_post_ms'), 'user_agent': body.get('user_agent')}))
        row = {'position': label, 'repeat': rep, 'description': notes, 'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'client_ip': ip, 'client_mac': mac}
        row.update(body.get('summary', {}))
        row['pi_ap_signal_dbm_median'] = (pi.get('ap_signal_dbm') or {}).get('median')
        row['pi_ap_signal_dbm_min'] = (pi.get('ap_signal_dbm') or {}).get('min')
        row['pi_ap_tx_bitrate_median'] = (pi.get('ap_tx_bitrate_mbps') or {}).get('median')
        row['pi_ap_tx_retries'] = pi.get('ap_tx_retries_delta')
        row['pi_ap_tx_failed'] = pi.get('ap_tx_failed_delta')
        t = pi.get('tcp_8082') or {}
        row['pi_tcp8082_rtt_ms_median'] = (t.get('rtt_ms') or {}).get('median')
        row['pi_tcp8082_rtt_ms_p95'] = (t.get('rtt_ms') or {}).get('p95')
        row['pi_tcp8082_retrans'] = t.get('retransmits_total')
        row['pi_soc_c_max'] = pi.get('soc_c_max')
        RESULTS.append(row)
        try:
            import wifi_range_test as W
            W.write_tables(RESULTS, SESSION)
        except Exception as exc:
            log(f'table write failed: {exc!r}')
    log(f'result saved: {label} #{rep} (client {ip}, signal at Pi {row["pi_ap_signal_dbm_median"]})')
    return {'ok': True, 'ap_signal_dbm': row['pi_ap_signal_dbm_median']}


# ------------------------------------------------------------------ control server
class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *a):
        pass

    def send(self, body, ctype='application/json', code=200):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        if u.path == '/ping':
            return self.send({'t': time.time()})
        if u.path == '/':
            return self.send((HERE / 'wifi_range_page.html').read_bytes(), 'text/html; charset=utf-8')
        if u.path == '/begin':
            rec = {'t': time.time(), 'label': q.get('label', ''), 'event': 'start', 'client': self.client_address[0]}
            with open(SESSION / 'markers.jsonl', 'a') as fh:
                fh.write(json.dumps(rec) + '\n')
            log(f'begin: {rec["label"]} from {rec["client"]}')
            return self.send(rec)
        if u.path == '/blob':
            n = int(float(q.get('mb', '4')) * (1 << 20))
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Length', str(n))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                sent = 0
                while sent < n:
                    chunk = BLOCK[:min(len(BLOCK), n - sent)]
                    self.wfile.write(chunk)
                    sent += len(chunk)
            except (BrokenPipeError, ConnectionResetError):
                self.close_connection = True
            return
        if u.path.startswith('/app/'):
            return self.proxy(u.path[4:] + ('?' + u.query if u.query else ''))
        if u.path == '/results.json':
            return self.send(RESULTS)
        if u.path == '/results.csv' or u.path == '/results.md':
            f = SESSION / ('results_all_columns.csv' if u.path.endswith('csv') else 'results_table.md')
            return self.send(f.read_bytes() if f.exists() else b'no results yet', 'text/plain; charset=utf-8')
        if u.path == '/mark':
            rec = {'t': time.time(), 'label': q.get('label', ''), 'event': q.get('event', ''), 'note': q.get('note', ''), 'client': self.client_address[0]}
            with open(SESSION / 'markers.jsonl', 'a') as fh:
                fh.write(json.dumps(rec) + '\n')
            log(f'marker: {rec["label"]} {rec["event"]} {rec["note"]}')
            return self.send(rec)
        if u.path == '/station':
            return self.send(summary(float(q.get('t0', 0)), float(q.get('t1', time.time()))))
        if u.path == '/info':
            return self.send({'session': str(SESSION), 'iperf3': bool(shutil.which('iperf3')), 'iw': bool(shutil.which('iw')), 'hotspot': S.hotspot.get('status'), 'time': time.time(),
                              'iwconfig': sh(['iwconfig', 'wlan0'], 4)[1], 'ap_profile': S.hotspot.get('profile', ''), 'soc_c': T.soc_temp()})
        if u.path == '/wifi_range_test.py':
            return self.send((HERE / 'wifi_range_test.py').read_bytes(), 'text/x-python')
        if u.path == '/logs.zip':
            tmp = tempfile.NamedTemporaryFile(suffix='.zip', delete=False)
            with zipfile.ZipFile(tmp.name, 'w', zipfile.ZIP_DEFLATED) as z:
                for f in SESSION.rglob('*'):
                    if f.is_file():
                        z.write(f, f.relative_to(SESSION.parent))
            data = Path(tmp.name).read_bytes()
            os.unlink(tmp.name)
            return self.send(data, 'application/zip')
        if u.path == '/finish':
            self.send({'ok': True})
            STOP.set()
            return
        self.send({'error': 'not found'}, code=404)


def _proxy(self, path):
    """Forward a viewer-style request to the pipeline's streamer on this Pi, keeping conditional-GET headers (the radio link is the only difference to a direct request)."""
    try:
        if getattr(self, 'up', None) is None:
            self.up = http.client.HTTPConnection('127.0.0.1', 8080, timeout=10)
        h = {'Accept-Encoding': self.headers.get('Accept-Encoding', 'gzip')}
        if self.headers.get('If-None-Match'):
            h['If-None-Match'] = self.headers['If-None-Match']
        self.up.request('GET', path, headers=h)
        r = self.up.getresponse()
        body = r.read()
        self.send_response(r.status)
        for k in ('ETag', 'X-Cloud-Stamp', 'X-Server-Time', 'X-Quality-Levels', 'X-Formats', 'Content-Type'):
            if r.getheader(k):
                self.send_header(k, r.getheader(k))
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)
    except (BrokenPipeError, ConnectionResetError):
        self.close_connection = True
    except Exception as exc:
        self.up = None
        self.send({'error': repr(exc)}, code=502)


def _do_post(self):
    u = urlparse(self.path)
    n = int(self.headers.get('Content-Length') or 0)
    data = self.rfile.read(n)
    if u.path == '/sink':
        return self.send({'bytes': len(data)})
    if u.path == '/save':
        return self.send(save_result(json.loads(data), self.client_address[0]))
    self.send({'error': 'not found'}, code=404)


Handler.proxy = _proxy
Handler.do_POST = _do_post


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    global SESSION
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-hotspot', action='store_true', help='do not switch WiFi (home testing of the scripts)')
    ap.add_argument('--ap-band', default='', help='a = 5 GHz, bg = 2.4 GHz; blank = the profile default (2.4 GHz)')
    ap.add_argument('--ap-channel', default='')
    ap.add_argument('--return-home', action='store_true', help='switch back to home WiFi when finished')
    args = ap.parse_args()
    SESSION = HERE / 'wifi_range' / time.strftime('%Y%m%d_%H%M%S')
    SESSION.mkdir(parents=True, exist_ok=True)
    STOP_FILE.unlink(missing_ok=True)
    S.out = SESSION
    S.markers_path = str(SESSION / 'markers_internal.jsonl')
    S.args = SimpleNamespace(ap_band=args.ap_band, ap_channel=args.ap_channel)
    T.P('14_wireless')
    signal.signal(signal.SIGTERM, lambda *a: STOP.set())
    signal.signal(signal.SIGINT, lambda *a: STOP.set())
    shutil.copy(HERE / 'wifi_range_test.py', SESSION / 'wifi_range_test.py') if (HERE / 'wifi_range_test.py').exists() else None
    log(f'wifi range test, Pi side. Session folder: {SESSION}')
    if not args.no_hotspot:
        ok = T.hotspot_up()
        log(f'hotspot: {S.hotspot.get("status")}  band={args.ap_band or "profile default (2.4 GHz)"}')
        T.jwrite(SESSION / 'hotspot_status.json', S.hotspot)
    else:
        S.hotspot = {'requested': False, 'status': 'skipped (--no-hotspot)'}
    for fn in (station_loop, tcp_loop, wlan_loop, iperf_loop):
        threading.Thread(target=fn, daemon=True).start()
    srv = Server(('0.0.0.0', CONTROL_PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    log(f'control server on port {CONTROL_PORT}; iperf3 server on 5201. Ready: run the client script now.')
    T.jwrite(SESSION / 'READY.json', {'time': time.time(), 'hotspot': S.hotspot.get('status'), 'iperf3': bool(shutil.which('iperf3')), 'iw': bool(shutil.which('iw'))})
    while not STOP.is_set():
        if STOP_FILE.exists():
            STOP.set()
        time.sleep(1)
    log('stopping')
    time.sleep(2)
    srv.shutdown()
    with zipfile.ZipFile(SESSION.parent / f'{SESSION.name}_pi_logs.zip', 'w', zipfile.ZIP_DEFLATED) as z:
        for f in SESSION.rglob('*'):
            if f.is_file():
                z.write(f, f.relative_to(SESSION.parent))
    T.jwrite(SESSION / 'DONE.json', {'finished': time.time(), 'return_home': args.return_home})
    if args.return_home and S.hotspot.get('requested'):
        T.hotspot_down()
    log('done')
    os._exit(0)


if __name__ == '__main__':
    main()
