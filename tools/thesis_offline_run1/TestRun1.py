#!/usr/bin/env python3
"""TestRun1: unattended, offline measurement run with the thermal camera physically disconnected.

Reuses the samplers, probe, safety gates, hotspot logic and benchmark of TestWithLidV1.py; adds the Run 1 phases:
inventory, scene (snapshots + images), rates, combinations (LiDAR only / LiDAR + RGB), range_accuracy, cpu_mem, stage_timing,
freshness, alignment (indirect RGB/LiDAR sweep) and repo_extract. Resumable (state.json), safe to kill, never repeats the countdown.

Started by thesis-offline.service (see arm.sh) or by hand:  python3 TestRun1.py --profile quick --countdown 20 --run-dir DIR --no-hotspot
"""
import argparse
import csv
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import TestWithLidV1 as T      # noqa: E402
import run1_geo as G           # noqa: E402

S = T.S
P = T.P


def SD(section, sub):
    """Sub-folder inside a numbered section (created)."""
    d = T.P(section) / sub
    d.mkdir(parents=True, exist_ok=True)
    return d
log = T.log
HOME = Path.home()
ROOT = HOME / 'thesis_tests'
HERE = Path(__file__).resolve().parent
SITE_FILE = ROOT / 'site_notes.env'
ARMED = ROOT / 'ARMED_1'
STOP_FILE = ROOT / 'STOP'

ORDER = ['inventory', 'scene', 'rates', 'combinations', 'range_accuracy', 'cpu_mem', 'stage_timing', 'freshness', 'alignment', 'repo_extract']
PNUM = {name: f'P{i + 1}' for i, name in enumerate(ORDER)}
EST_MIN = {'inventory': 1, 'scene': 11, 'rates': 10, 'combinations': 18, 'range_accuracy': 3, 'cpu_mem': 10, 'stage_timing': 4, 'freshness': 5, 'alignment': 4, 'repo_extract': 1}
RANGE_BANDS = [(2, 5), (5, 10), (10, 15), (15, 20), (20, 30)]
ALIGN_BANDS = [('below_10m', 0, 10), ('10_to_20m', 10, 20), ('above_20m', 20, 1e9)]
NEW_KEYS = {
    'scene_mean_range_m': ('m', 'mean range of merged points (outdoor scene)'), 'scene_share_15_25m_pct': ('%', 'share of merged points between 15 and 25 m'),
    'plane_rms_mm_2_5m': ('mm', ''), 'plane_rms_mm_5_10m': ('mm', ''), 'plane_rms_mm_10_15m': ('mm', ''), 'plane_rms_mm_15_20m': ('mm', ''), 'plane_rms_mm_20_30m': ('mm', ''),
    'plane_repeat_mm_5_10m': ('mm', ''), 'plane_repeat_mm_15_20m': ('mm', ''), 'truth_error_mean_abs_mm': ('mm', 'mean absolute error over supplied tape distances'),
    'combo_lidar_only_cpu_pct_of_core': ('%', ''), 'combo_lidar_rgb_cpu_pct_of_core': ('%', ''), 'combo_lidar_rgb_fused_hz': ('Hz', ''),
    'combo_lidar_rgb_merged_hz': ('Hz', ''), 'combo_lidar_rgb_coloured_pct': ('%', ''), 'combo_lidar_only_current_a': ('A', ''), 'combo_lidar_rgb_current_a': ('A', ''),
    'align_score_ratio_deployed_to_peak_below10': ('', ''), 'align_score_ratio_deployed_to_peak_10_20': ('', ''), 'align_score_ratio_deployed_to_peak_above20': ('', ''),
    'age_fused_p50_ms': ('ms', 'ROS-level age'), 'age_merged_p50_ms_ros': ('ms', 'ROS-level age'),
}


# ------------------------------------------------------------------ site notes and state
def read_site():
    d = {}
    if SITE_FILE.exists():
        for line in SITE_FILE.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith('#') and '=' in line:
                k, v = line.split('=', 1)
                d[k.strip().upper()] = v.strip().strip('"').strip("'")
    return d


def truth_items(site):
    out = []
    for item in (site.get('TRUTH') or '').split(','):
        if ':' in item:
            lab, val = item.rsplit(':', 1)
            try:
                out.append((lab.strip(), float(val)))
            except ValueError:
                pass
    return out


class RunState:
    def __init__(self, path):
        self.path = path
        self.d = {'created': time.strftime('%Y-%m-%dT%H:%M:%S'), 'countdown_done': False, 'phases': {}, 'boots': [], 'finalized': False, 'windows': {}}
        if path.exists():
            try:
                self.d.update(json.load(open(path)))
            except Exception:
                pass

    def save(self):
        tmp = str(self.path) + '.tmp'
        json.dump(self.d, open(tmp, 'w'), indent=1)
        os.replace(tmp, self.path)


RS = None


def save_numbers():
    T.jwrite(S.out / 'numbers_state.json', S.numbers)


def numx(key, value, unit='', source='', note=''):
    """Like T.num but keys that are new in Run 1 have no previous value."""
    T.num(key, value, unit or NEW_KEYS.get(key, ('', ''))[0], source, note or NEW_KEYS.get(key, ('', ''))[1])


def thermal_usb_present():
    return bool(re.search('Thermal', T.sh('ls /dev/v4l/by-id 2>&1', 5)[1], re.I))


def thermal_label():
    st, hz = T.thermal_status()
    return f"camera {'connected (USB device present)' if thermal_usb_present() else 'not connected'}; data stream {st} ({hz} Hz); thermal content is not used or reported in Run 1"


def cond(t0, t1):
    c = T.conditions(t0, t1)
    c['thermal'] = thermal_label()
    c['thermal_status_probe'] = c.pop('thermal_status', '')
    c['ambient_from_site_notes'] = read_site().get('AMBIENT_C') or 'not supplied'
    c['ap_stations_now'] = S.ap_clients
    return c


def finish(section, sub, summary, readme_title, readme_lines, t0):
    d = SD(section, sub) if sub else P(section)
    summary['conditions'] = cond(t0, time.time())
    T.jwrite(d / 'summary.json', summary)
    with open(d / 'README.md', 'a') as fh:
        fh.write(f'## {readme_title}\n\n' + '\n'.join(readme_lines) + f'\n\nConditions: thermal {thermal_label()}; see summary.json for temperatures, throttle flags and hotspot state.\n\n')
    S.summaries[readme_title] = summary
    return summary


def csv_rows(path):
    with open(path) as fh:
        return list(csv.DictReader(fh))


def ages_ms(key, a, b):
    w = T.topic_window(key, a, b)
    return (w[:, 0] - w[:, 1]) * 1000.0 if len(w) else np.array([])


def cov_window(a, b):
    p = P('02_rates', 'coverage_raw.csv')
    if not p.exists():
        return None
    try:
        rows = [r for r in csv_rows(p) if a <= float(r['t_arrival']) <= b]
        return float(np.mean([float(r['pct_rgb']) for r in rows])) if rows else None
    except Exception:
        return None


# ================================================================== P1 inventory
def phase_inventory():
    t0 = time.time()
    T.phase_meta()
    site = read_site()
    shutil.copy(SITE_FILE, P('00_meta', 'site_notes.env')) if SITE_FILE.exists() else None
    usb = T.sh('lsusb', 10)[1]
    v4l = T.sh('ls -l /dev/v4l/by-id 2>&1; ls /dev/video* 2>&1', 10)[1]
    present = bool(re.search(r'Thermal', v4l + usb, re.I))
    st_th, hz_th = T.thermal_status()
    th = {'usb_device_present': present, 'stream_status_probe': st_th, 'stream_hz_10s': hz_th, 'lsusb': usb, 'v4l_by_id': v4l, 'time': time.strftime('%Y-%m-%dT%H:%M:%S'),
          'note': 'Thermal camera state as found at the start of Run 1 (USB device present or not, and whether it was streaming). Run 1 results do not use or report thermal image content.'}
    T.jwrite(P('00_meta', 'thermal_state.json'), th)
    log(f'thermal camera: USB device present={present}, stream status {st_th} ({hz_th} Hz): recorded')
    extra = {}
    for repo in (HOME / 'PitFusion3D_repo', HOME / 'ros2_ws/src', HERE):
        extra[repo.name] = T.sh(f'git -C {repo} rev-parse HEAD 2>/dev/null; git -C {repo} branch --show-current 2>/dev/null', 10)[1]
    extra['site_notes'] = site
    extra['hotspot'] = S.hotspot
    T.jwrite(P('00_meta', 'run1_inventory.json'), extra)
    T.num('ambient_c', float(site['AMBIENT_C']) if site.get('AMBIENT_C') else None, 'C', '00_meta/site_notes.env', 'from site notes' if site.get('AMBIENT_C') else 'not supplied')
    finish('00_meta', '', {'thermal_usb_present': present, 'thermal_stream_status': st_th, 'site_notes': site, 'ap_hotspot': S.hotspot.get('status')}, 'Run 1 inventory',
           ['system_inventory.json, node_parameters.json, calibration and launch files in use, git hashes, USB list and thermal camera state, site notes, hotspot status.'], t0)


# ================================================================== P2 scene snapshots and images
def viewer_screenshot(path):
    ff = shutil.which('firefox')
    if not ff:
        return 'firefox not installed'
    host = '10.42.0.1' if S.hotspot.get('status') == 'up' else '127.0.0.1'
    prof = tempfile.mkdtemp(prefix='ffprof_', dir=str(HOME))     # a snap-packaged firefox can only use profiles under the home folder
    try:
        rc, out = T.sh([ff, '--headless', '--no-remote', '--profile', prof, '--window-size', '1280,800', '--screenshot', str(path), f'http://{host}:8080/'], 90)
        if not Path(path).exists():
            return f'screenshot failed (rc {rc}): {out[-200:]}'
        import cv2
        im = cv2.imread(str(path))
        if im is None or im.std() < 3:
            return 'captured but the page is blank (headless browser could not draw the viewer)'
        return 'page captured (viewer panel only: the headless browser does not draw the 3D point cloud)'
    finally:
        shutil.rmtree(prof, ignore_errors=True)


def range_stats(xyz):
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    r = np.linalg.norm(xyz, axis=1)
    edges = [0, 5, 10, 15, 25, 1e9]
    names = ['0_5m', '5_10m', '10_15m', '15_25m', 'over_25m']
    out = {'n_points': int(len(r)), 'mean_range_m': float(r.mean()), 'median_range_m': float(np.median(r)), 'p5_m': float(np.percentile(r, 5)), 'p95_m': float(np.percentile(r, 95)),
           'share_15_25m_pct': float(100 * np.mean((r >= 15) & (r <= 25)))}
    for nm, lo, hi in zip(names, edges[:-1], edges[1:]):
        out[f'share_{nm}_pct'] = float(100 * np.mean((r >= lo) & (r < hi)))
    return out, r


def scene_snapshot(tag, cal):
    d = SD('15_fusion_info', 'scene')
    img_dir = S.out / 'images'
    img_dir.mkdir(exist_ok=True)
    cap = T.capture('lidar:5,fused:1,merged:1,rgb:1,caminfo:1', f'scene_{tag}', 90)
    if not cap:
        raise RuntimeError(f'snapshot {tag}: capture failed')
    raw = S.out / 'raw_captures'
    raw.mkdir(exist_ok=True)
    z = np.load(cap, allow_pickle=True)
    t_snap = time.time()
    res = {'tag': tag, 'time': time.strftime('%H:%M:%S'), 'soc_c': round(T.soc_temp(), 1)}
    import cv2
    img = z['rgb_img'][0] if 'rgb_img' in z.files else None
    if img is not None:
        cv2.imwrite(str(img_dir / f'rgb_{tag}.png'), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    lidar = z['lidar_xyz'] if 'lidar_xyz' in z.files else None
    if lidar is not None:
        lidar = lidar[np.isfinite(lidar).all(axis=1)]
    mxyz = z['merged_xyz'].astype(np.float32) if 'merged_xyz' in z.files else None
    mrest = z['merged_rest'] if 'merged_rest' in z.files else None
    mrgb = mrest[:, 1].astype(np.float32).view(np.uint32) if mrest is not None else None
    fxyz = z['fused_xyz'].astype(np.float32) if 'fused_xyz' in z.files else None
    frest = z['fused_rest'] if 'fused_rest' in z.files else None
    frgb = frest[:, 1].astype(np.float32).view(np.uint32) if frest is not None else None
    if mxyz is not None:
        np.savez_compressed(d / f'merged_{tag}.npz', xyz=mxyz, rgb=mrgb, intensity=mrest[:, 0].astype(np.float32), scan_time_unix=float(z['merged_arrival'][0]),
                            scan_stamp=float(z['merged_stamps'][0]))
        st, r = range_stats(mxyz)
        res['merged'] = st
        hist, edges = np.histogram(r, bins=np.arange(0, 61, 1.0))
        T.write_csv(d / f'range_hist_{tag}.csv', ['range_lo_m', 'range_hi_m', 'n_points', 'share_pct'], [[edges[i], edges[i + 1], int(hist[i]), f'{100 * hist[i] / max(1, hist.sum()):.3f}'] for i in range(len(hist))])
        G.sensor_view(mxyz, mrgb, img_dir / f'merged_view_{tag}.png', f'merged cloud, sensor view ({tag})')
        G.sensor_view(mxyz, None, img_dir / f'merged_range_{tag}.png', f'merged cloud coloured by range ({tag})')
        G.plan_views(mxyz, mrgb, img_dir / f'merged_topdown_{tag}.png', img_dir / f'merged_side_{tag}.png', f'merged cloud ({tag})')
    if fxyz is not None:
        np.savez_compressed(d / f'fused_{tag}.npz', xyz=fxyz, rgb=frgb, intensity=frest[:, 0].astype(np.float32), scan_time_unix=float(z['fused_arrival'][0]), scan_stamp=float(z['fused_stamps'][0]))
        G.sensor_view(fxyz, frgb, img_dir / f'fused_view_{tag}.png', f'live fused scan, sensor view ({tag})')
        res['fused_coloured_pct'] = float(100 * np.mean(frgb != 0))
    if lidar is not None and img is not None:
        res['lidar_points_on_image'] = G.overlay_lidar_on_rgb(img, lidar, cal, img_dir / f'lidar_on_rgb_{tag}.png', title=f'LiDAR (5 scans) projected onto the RGB frame, coloured by range ({tag})')
        res['lidar_scan_range'] = range_stats(lidar)[0]
    res['viewer_screenshot'] = viewer_screenshot(img_dir / f'viewer_{tag}.png')
    shutil.move(str(cap), str(raw / f'scene_{tag}.npz'))
    res['seconds'] = round(time.time() - t_snap, 1)
    return res


def phase_scene():
    t0 = time.time()
    cal = G.load_calibration()
    snaps = []
    offsets = [('min00', 0), ('min05', T.D(300, 20)), ('min10', T.D(600, 40))]
    for tag, off in offsets:
        while time.time() < t0 + off:
            T.check_stop()
            time.sleep(1)
        try:
            snaps.append(scene_snapshot(tag, cal))
        except T.Stop:
            raise
        except Exception as exc:
            log(f'scene snapshot {tag} failed: {exc!r}')
            snaps.append({'tag': tag, 'error': repr(exc)})
    good = [s for s in snaps if 'merged' in s]
    summ = {'snapshots': snaps}
    if good:
        keys = [k for k in good[0]['merged'] if k.startswith('share_') or k in ('mean_range_m', 'median_range_m')]
        summ['mean_over_snapshots'] = {k: float(np.mean([s['merged'][k] for s in good])) for k in keys}
        numx('scene_mean_range_m', round(summ['mean_over_snapshots']['mean_range_m'], 2), 'm', '15_fusion_info/scene/summary.json')
        numx('scene_share_15_25m_pct', round(summ['mean_over_snapshots']['share_15_25m_pct'], 2), '%', '15_fusion_info/scene/summary.json')
    T.write_csv(SD('15_fusion_info', 'scene') / 'range_summary.csv', ['tag', 'n_points', 'mean_range_m', 'median_range_m', 'share_0_5m_pct', 'share_5_10m_pct', 'share_10_15m_pct', 'share_15_25m_pct', 'share_over_25m_pct'],
                [[s['tag']] + [f"{s['merged'][k]:.3f}" for k in ('n_points', 'mean_range_m', 'median_range_m', 'share_0_5m_pct', 'share_5_10m_pct', 'share_10_15m_pct', 'share_15_25m_pct', 'share_over_25m_pct')] for s in good])
    finish('15_fusion_info', 'scene', summ, 'Scene snapshots (P2)',
           ['Three snapshots (0, 5, 10 min). merged_*.npz / fused_*.npz: xyz, rgb (uint32 0xRRGGBB, 0 = no colour), intensity, scan time. range_hist_*.csv: 1 m range histogram of the merged cloud.',
            'Images are in ../../images (RGB frame, LiDAR on RGB, fused view, merged view, range-coloured, top-down, side, viewer screenshot if the headless browser could draw it).'], t0)


# ================================================================== P3 rates
def series_10s(key, a, b):
    w = T.topic_window(key, a, b)
    rows = []
    t = a
    while t < b:
        n = int(np.sum((w[:, 0] >= t) & (w[:, 0] < t + 10))) if len(w) else 0
        rows.append([f'{t:.1f}', f'{n / 10.0:.2f}'])
        t += 10
    return rows


def rate_block(a, b, keys):
    rows, summ = [], {}
    for key in keys:
        st = T.rate_stats(key, a, b)
        summ[key] = st
        rows.append([key, f'{st.get("hz_mean", 0):.3f}', f'{st.get("interval_p50_ms", "")}', f'{st.get("interval_p95_ms", "")}', f'{st.get("interval_std_ms", "")}', st['n'],
                     st.get('points_per_msg_median', ''), st.get('kbytes_per_msg_median', '')])
    return rows, summ


def phase_rates():
    t0 = time.time()
    T.sleep_checked(T.D(600, 30))
    t1 = time.time()
    RS.d['windows']['rates'] = [t0, t1]
    RS.save()
    keys = ('lidar_raw', 'lidar_restamped', 'rgb', 'thermal', 'fused', 'merged')
    rows, summ = rate_block(t0, t1, keys)
    T.write_csv(P('02_rates', 'rates.csv'), ['topic', 'hz_mean', 'interval_p50_ms', 'interval_p95_ms', 'interval_std_ms', 'n', 'points_per_msg_median', 'kbytes_per_msg_median'], rows)
    for key in ('lidar_raw', 'rgb', 'fused', 'merged'):
        T.write_csv(P('02_rates', f'rate_series_10s_{key}.csv'), ['t_start_unix', 'hz'], series_10s(key, t0, t1))
    lr = T.topic_window('lidar_restamped', t0, t1)
    loss = None
    if len(lr) > 10:
        expected = (lr[-1, 1] - lr[0, 1]) / 0.1 + 1
        loss = max(0.0, 100.0 * (1 - len(lr) / expected))
        summ['lidar_scans_lost_pct'] = loss
        summ['lidar_gaps_over_150ms'] = int(np.sum(np.diff(lr[:, 0]) > 0.15))
    for key, src in (('rate_lidar_hz', 'lidar_raw'), ('rate_rgb_fps', 'rgb'), ('rate_fused_hz', 'fused'), ('rate_merged_hz', 'merged')):
        hz = summ[src].get('hz_mean')
        T.num(key, round(hz, 3) if hz is not None else None, 'Hz', '02_rates/rates.csv', 'no viewer connected')
    T.num('lidar_scans_lost_pct', loss, '%', '02_rates/rates.csv')
    T.num('merged_points_before_cap', summ['merged'].get('points_per_msg_median'), 'points', '02_rates/rates.csv', 'median of the merged topic (before the viewer cap)')
    T.num('rate_thermal_fps', round(summ['thermal'].get('hz_mean', 0.0), 3), 'Hz', '02_rates/rates.csv', 'rate of the thermal topic only (' + thermal_label() + ')')
    summ['window'] = [t0, t1]
    finish('02_rates', '', summ, 'Rates (P3)', ['rates.csv: rate and interval statistics from independent raw subscribers over the window; rate_series_10s_*.csv: rate per 10 s; topic_*.csv: every message for the whole run.'], t0)


# ================================================================== P4 sensor combinations
def clean_shm():
    T.sh([sys.executable, str(HOME / 'ros2_ws/src/pitfusion3d_bringup/scripts/clean_stale_shm.py')], 30)


def window_metrics(a, b):
    s = T.sampler_slice(a, b)
    have = len(s.get('t', [])) > 0
    m = {'seconds': round(b - a, 1), 'soc_start_c': round(float(s['soc_c'][0]), 1) if have else None, 'soc_end_c': round(float(s['soc_c'][-1]), 1) if have else None}
    m['soc_rise_c'] = round(m['soc_end_c'] - m['soc_start_c'], 1) if have else None
    for key in ('lidar_raw', 'lidar_restamped', 'rgb', 'fused', 'merged'):
        st = T.rate_stats(key, a, b)
        m[f'{key}_hz'] = round(st.get('hz_mean', 0.0), 3)
        m[f'{key}_interval_p95_ms'] = st.get('interval_p95_ms')
        if key in ('lidar_restamped', 'fused', 'merged'):
            ag = ages_ms(key, a, b)
            m[f'{key}_age_p50_ms'] = round(float(np.median(ag)), 1) if len(ag) else None
            m[f'{key}_age_p95_ms'] = round(float(np.percentile(ag, 95)), 1) if len(ag) else None
    if have:
        m['pipeline_cpu_pct_of_core'] = round(float(np.nanmean(s['pipeline_cpu_pct_of_core'])), 1)
        m['pipeline_rss_mb'] = round(float(np.nanmean(s['pipeline_rss_mb'])), 0)
        for name in T.PIPE_PROCS:
            col = s.get(f'{name}_cpu')
            m[f'{name}_cpu_pct'] = round(float(np.nanmean(col)), 1) if col is not None and len(col) and not np.all(np.isnan(col)) else None
        for i in range(4):
            m[f'cpu{i}_pct'] = round(float(np.nanmean(s[f'cpu{i}'])), 1)
    m['coloured_pct'] = cov_window(a, b)
    m.update({f'power_{k}': round(v, 3) for k, v in T.power_summary(a, b).items() if k in ('est_input_a_mean', 'est_input_a_max', 'power_w_mean', 'ext5v_min')})
    return m


def stop_proc(proc):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 25), (signal.SIGTERM, 20)):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(wait)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def wait_topic(key, hz_min, timeout=150):
    t0 = time.time()
    while time.time() - t0 < timeout:
        T.check_stop()
        if ((T.probe_status().get(key) or {}).get('hz_10s') or 0) >= hz_min:
            return True
        time.sleep(3)
    return False


def build_variants(d):
    src = HOME / 'ros2_ws/src/pitfusion3d_bringup/launch'
    rs = (src / 'real_sensors.launch.py').read_text()
    full = (src / 'pitfusion3d_full.launch.py').read_text()
    lidar_only = rs.replace('        rgb_camera_delayed,\n', '', 1).replace('        thermal_camera_delayed,\n', '', 1)
    lidar_rgb = rs.replace('        thermal_camera_delayed,\n', '', 1)
    if lidar_only == rs or lidar_rgb == rs:
        raise RuntimeError('could not build the launch variants (real_sensors.launch.py changed)')
    (d / 'real_sensors_lidar_only.launch.py').write_text(lidar_only)
    (d / 'real_sensors_lidar_rgb.launch.py').write_text(lidar_rgb)
    new, n = re.subn(r"PathJoinSubstitution\(\s*\[FindPackageShare\('pitfusion3d_bringup'\), 'launch', 'real_sensors.launch.py'\]\s*\)", repr(str(d / 'real_sensors_lidar_rgb.launch.py')), full)
    if n != 1:
        raise RuntimeError('could not build the LiDAR + RGB launch variant')
    (d / 'pitfusion3d_lidar_rgb.launch.py').write_text(new)
    return d / 'real_sensors_lidar_only.launch.py', d / 'pitfusion3d_lidar_rgb.launch.py'


def phase_combinations():
    t0 = time.time()
    d = SD('15_fusion_info', 'combinations')
    var_dir = d / 'variants'
    var_dir.mkdir(exist_ok=True)
    lidar_only, lidar_rgb = build_variants(var_dir)
    res = {}
    if 'rates' in RS.d['windows']:
        a, b = RS.d['windows']['rates']
        res['deployed_pipeline'] = window_metrics(a, b)
        res['deployed_pipeline']['note'] = ('deployed pipeline as it runs, with the thermal node and watchdog running (' + thermal_label() + '); '
                                                          'P3 window, no viewer')
    proc = None
    try:
        for name, launch, ready_key, ready_hz, note in (
                ('lidar_only', lidar_only, 'lidar_raw', 8.0,
                 'LiDAR driver + restamp + calibration publisher only. The fusion node needs RGB frames to emit a cloud and the merge node reads the fused (colour) cloud fields, '
                 'so neither can run on LiDAR alone: fused and merged are not produced in this configuration.'),
                ('lidar_rgb', lidar_rgb, 'fused', 8.0,
                 'full pipeline (driver, restamp, RGB camera, calibration, fusion, merge, streamer) with the thermal camera node removed from the launch.')):
            T.sh(['systemctl', '--user', 'stop', T.UNIT], 90)
            time.sleep(5)
            clean_shm()
            env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
            lf = open(var_dir / f'launch_{name}.log', 'w')
            proc = subprocess.Popen(['ros2', 'launch', str(launch)], stdout=lf, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            try:
                ok = wait_topic(ready_key, ready_hz)
                T.sleep_checked(T.D(60, 15))
                a = time.time()
                T.sleep_checked(T.D(360, 30))
                b = time.time()
                res[name] = window_metrics(a, b)
                res[name]['launch_ok'] = ok
                res[name]['what_ran'] = note
                res[name]['window'] = [a, b]
            finally:
                stop_proc(proc)
                proc = None
                time.sleep(5)
    finally:
        stop_proc(proc)
        T.sh(['systemctl', '--user', 'start', T.UNIT], 90)
        up = T.wait_ready(240)
        time.sleep(max(T.D(60, 35), 35))
        b = time.time()
        a = b - 20
        chk = {'cloud_after_s': up, 'lidar_hz': T.hz_between('lidar_raw', a, b), 'rgb_hz': T.hz_between('rgb', a, b), 'fused_hz': T.hz_between('fused', a, b)}
        chk['nominal'] = bool(chk['lidar_hz'] >= 9.0 and chk['rgb_hz'] >= 8.5 and chk['fused_hz'] >= 8.5)
        res['restore_check'] = chk
        T.jwrite(d / 'restore_check.json', chk)
        log(f'service restored; rates {chk}')
    cols = ['configuration', 'lidar_raw_hz', 'rgb_hz', 'fused_hz', 'merged_hz', 'fused_age_p50_ms', 'merged_age_p50_ms', 'pipeline_cpu_pct_of_core', 'cpu0_pct', 'cpu1_pct', 'cpu2_pct', 'cpu3_pct',
            'coloured_pct', 'power_est_input_a_mean', 'power_est_input_a_max', 'soc_rise_c']
    rows = [[k] + [res[k].get(c, '') for c in cols[1:]] for k in ('lidar_only', 'lidar_rgb', 'deployed_pipeline') if k in res]
    T.write_csv(d / 'combinations.csv', cols, rows)
    per_proc = [[k] + [res[k].get(f'{n}_cpu_pct', '') for n in T.PIPE_PROCS] for k in ('lidar_only', 'lidar_rgb', 'deployed_pipeline') if k in res]
    T.write_csv(d / 'combinations_cpu_per_process.csv', ['configuration'] + T.PIPE_PROCS, per_proc)
    lo, lr = res.get('lidar_only', {}), res.get('lidar_rgb', {})
    numx('combo_lidar_only_cpu_pct_of_core', lo.get('pipeline_cpu_pct_of_core'), '%', '15_fusion_info/combinations/combinations.csv')
    numx('combo_lidar_rgb_cpu_pct_of_core', lr.get('pipeline_cpu_pct_of_core'), '%', '15_fusion_info/combinations/combinations.csv')
    numx('combo_lidar_rgb_fused_hz', lr.get('fused_hz'), 'Hz', '15_fusion_info/combinations/combinations.csv')
    numx('combo_lidar_rgb_merged_hz', lr.get('merged_hz'), 'Hz', '15_fusion_info/combinations/combinations.csv')
    numx('combo_lidar_rgb_coloured_pct', lr.get('coloured_pct'), '%', '15_fusion_info/combinations/combinations.csv')
    numx('combo_lidar_only_current_a', lo.get('power_est_input_a_mean'), 'A', '15_fusion_info/combinations/combinations.csv', 'estimated from PMIC rails')
    numx('combo_lidar_rgb_current_a', lr.get('power_est_input_a_mean'), 'A', '15_fusion_info/combinations/combinations.csv', 'estimated from PMIC rails')
    finish('15_fusion_info', 'combinations', res, 'Sensor combinations (P4)',
           ['Two windows (60 s settle + 6 min each) with generated launch variants (originals untouched, in variants/). The deployed service is stopped for each and restored afterwards (restore_check.json).',
            'combinations.csv: rates, data age at the ROS level, CPU (pipeline total and per core), % points coloured, estimated current, SoC rise. combinations_cpu_per_process.csv: per-process CPU.',
            'The deployed-pipeline row is the P3 window (thermal node still launching and retrying).'], t0)


# ================================================================== P5 range accuracy
def phase_range_accuracy():
    t0 = time.time()
    d = SD('08_accuracy', 'range_accuracy')
    n_scans = max(40, int(200 * max(S.scale, 0.2)))
    cap = T.capture(f'lidar:{n_scans}', 'range_acc', 120)
    if not cap:
        raise RuntimeError('LiDAR capture failed')
    raw = S.out / 'raw_captures'
    raw.mkdir(exist_ok=True)
    z = np.load(cap)
    xyz, off = z['lidar_xyz'], z['lidar_offsets']
    rng = np.random.default_rng(7)
    chunk = 5
    chunk_rows, planes = [], []
    n_chunks = (len(off) - 1) // chunk
    for ci in range(n_chunks):
        pts = xyz[off[ci * chunk]:off[(ci + 1) * chunk]].astype(np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        r = np.linalg.norm(pts, axis=1)
        for bi, (lo, hi) in enumerate(RANGE_BANDS):
            sel = pts[(r >= lo) & (r < hi)]
            thr = 0.03 + 0.003 * (lo + hi) / 2.0
            for pl in G.find_planes(sel, rng, max_planes=3, thr=thr):
                n, dd, rms, ninl, cen, ext = pl
                planes.append({'chunk': ci, 'band': bi, 'n': n, 'dist': -dd, 'rms': rms, 'inliers': ninl, 'centroid_range': float(np.linalg.norm(cen)), 'extent': ext})
                chunk_rows.append([ci, f'{lo}-{hi}', f'{-dd:.4f}', f'{n[0]:.4f}', f'{n[1]:.4f}', f'{n[2]:.4f}', ninl, f'{rms * 1000:.2f}', f'{np.linalg.norm(cen):.2f}', f'{ext:.2f}'])
    T.write_csv(d / 'plane_chunks.csv', ['chunk(5 scans, 0.5 s)', 'range_band_m', 'distance_to_plane_m', 'nx', 'ny', 'nz', 'inliers', 'rms_mm', 'centroid_range_m', 'extent_m'], chunk_rows)
    groups = []
    for pl in planes:
        for g in groups:
            if g['band'] == pl['band'] and np.degrees(np.arccos(np.clip(abs(float(g['n0'] @ pl['n'])), -1, 1))) < 6.0 and abs(g['d0'] - pl['dist']) < 0.4:
                g['m'].append(pl)
                break
        else:
            groups.append({'band': pl['band'], 'n0': pl['n'], 'd0': pl['dist'], 'm': [pl]})
    grows, gsum = [], []
    for gi, g in enumerate(groups):
        m = g['m']
        det = len({p['chunk'] for p in m}) / max(1, n_chunks)
        dist = np.array([p['dist'] for p in m])
        nmean = np.mean([p['n'] for p in m], axis=0)
        nmean /= np.linalg.norm(nmean)
        rec = {'group': gi, 'band_m': f'{RANGE_BANDS[g["band"]][0]}-{RANGE_BANDS[g["band"]][1]}', 'detected_in_chunks_pct': round(100 * det, 1), 'distance_median_m': float(np.median(dist)),
               'distance_std_mm_repeatability': float(dist.std() * 1000), 'rms_median_mm': float(np.median([p['rms'] for p in m]) * 1000), 'inliers_median': float(np.median([p['inliers'] for p in m])),
               'centroid_range_m': float(np.median([p['centroid_range'] for p in m])), 'normal': [float(v) for v in nmean], 'plane_equation': f'{nmean[0]:.4f} x + {nmean[1]:.4f} y + {nmean[2]:.4f} z = {-np.median(dist):.4f} m'}
        gsum.append(rec)
        grows.append([gi, rec['band_m'], rec['detected_in_chunks_pct'], f'{rec["distance_median_m"]:.4f}', f'{rec["distance_std_mm_repeatability"]:.2f}', f'{rec["rms_median_mm"]:.2f}', f'{rec["inliers_median"]:.0f}',
                      f'{rec["centroid_range_m"]:.2f}', rec['plane_equation']])
    T.write_csv(d / 'planes_by_band.csv', ['group', 'range_band_m', 'detected_in_chunks_pct', 'distance_median_m', 'repeatability_std_mm', 'rms_median_mm', 'inliers_median', 'centroid_range_m', 'plane_equation'], grows)
    strong = [g for g in gsum if g['detected_in_chunks_pct'] >= 30.0]
    band_rows = []
    for bi, (lo, hi) in enumerate(RANGE_BANDS):
        gb = [g for g in strong if g['band_m'] == f'{lo}-{hi}']
        if gb:
            best = max(gb, key=lambda g: g['inliers_median'])
            band_rows.append([f'{lo}-{hi}', len(gb), f'{best["rms_median_mm"]:.2f}', f'{best["distance_std_mm_repeatability"]:.2f}', f'{best["inliers_median"]:.0f}', f'{best["distance_median_m"]:.3f}'])
            numx(f'plane_rms_mm_{lo}_{hi}m', round(best['rms_median_mm'], 2), 'mm', '08_accuracy/range_accuracy/planes_by_band.csv', 'strongest plane in the band (dominant plane, not necessarily a flat wall)')
            if f'{lo}_{hi}' in ('5_10', '15_20'):
                numx(f'plane_repeat_mm_{lo}_{hi}m', round(best['distance_std_mm_repeatability'], 2), 'mm', '08_accuracy/range_accuracy/planes_by_band.csv')
        else:
            band_rows.append([f'{lo}-{hi}', 0, '', '', '', 'no plane found in this band'])
    T.write_csv(d / 'band_summary.csv', ['range_band_m', 'planes_found', 'plane_rms_mm', 'repeatability_std_mm', 'inliers_median', 'distance_median_m'], band_rows)
    truth = truth_items(read_site())
    trows, errs = [], []
    for lab, tm in truth:
        if strong:
            near = min(strong, key=lambda g: abs(g['distance_median_m'] - tm))
            e = near['distance_median_m'] - tm
            if abs(e) <= 1.0:
                trows.append([lab, f'{tm:.3f}', f'{near["distance_median_m"]:.3f}', f'{e * 1000:.1f}', near['plane_equation'], near['detected_in_chunks_pct']])
                errs.append(abs(e) * 1000)
                continue
        trows.append([lab, f'{tm:.3f}', '', '', 'no detected plane within 1 m of the supplied distance', ''])
    if not truth:
        trows = [['not supplied', '', '', '', 'no --truth distances in site_notes.env', '']]
    T.write_csv(d / 'truth_errors.csv', ['object', 'tape_distance_m', 'lidar_plane_distance_m', 'error_mm(lidar - tape)', 'plane / note', 'detected_in_chunks_pct'], trows)
    if errs:
        numx('truth_error_mean_abs_mm', round(float(np.mean(errs)), 1), 'mm', '08_accuracy/range_accuracy/truth_errors.csv')
    shutil.move(str(cap), str(raw / 'range_accuracy_lidar.npz'))
    finish('08_accuracy', 'range_accuracy', {'scans': n_scans, 'chunks_of_5_scans': n_chunks, 'groups': gsum, 'bands': RANGE_BANDS, 'truth': truth, 'n_truth_matched': len(errs),
                                              'caveats': 'distance = perpendicular distance from the LiDAR origin to the plane; the tape distance is measured from the LiDAR window and carries its own error (a few mm to cm); the LiDAR origin sits behind the window.'},
           'Range accuracy (P5)', ['plane_chunks.csv: every plane found in every 0.5 s chunk (5 scans accumulated, rig static). planes_by_band.csv: planes grouped across chunks (same normal within 6 deg and distance within 0.4 m).',
                                   'band_summary.csv: strongest plane per band. truth_errors.csv: nearest detected plane (within 1 m) to each tape distance, signed error.'], t0)


# ================================================================== P6 CPU, threads, memory
def phase_cpu_mem():
    t0 = time.time()
    T.sleep_checked(T.D(600, 30))
    t1 = time.time()
    s = T.sampler_slice(t0, t1)
    if not len(s.get('t', [])):
        raise RuntimeError('no 1 Hz samples in the window')
    rows = []
    for n in T.PIPE_PROCS:
        c = s.get(f'{n}_cpu')
        r = s.get(f'{n}_rss_mb')
        th = s.get(f'{n}_threads')
        if c is None or not len(c) or np.all(np.isnan(c)):
            rows.append([n, '', '', '', '', '', ''])
            continue
        rows.append([n, f'{np.nanmean(c):.1f}', f'{np.nanpercentile(c, 95):.1f}', f'{np.nanmax(c):.1f}', f'{np.nanmean(r):.0f}', f'{np.nanmax(r):.0f}', f'{np.nanmax(th):.0f}'])
    T.write_csv(P('05_cpu_memory', 'cpu_per_process.csv'), ['process', 'cpu_mean_pct_of_core', 'cpu_p95', 'cpu_max', 'rss_mean_mb', 'rss_max_mb', 'threads_max'], rows)
    core_rows = [[f'core{i}', f'{np.nanmean(s[f"cpu{i}"]):.1f}', f'{np.nanpercentile(s[f"cpu{i}"], 95):.1f}', f'{np.nanmax(s[f"cpu{i}"]):.1f}'] for i in range(4)]
    T.write_csv(P('05_cpu_memory', 'cpu_per_core.csv'), ['core', 'load_mean_pct', 'load_p95', 'load_max'], core_rows)
    bt = np.asarray(s['busiest_thread_pct'], dtype=float)
    procs = [p for p in s['busiest_thread_proc'] if p]
    top = max(set(procs), key=procs.count) if procs else ''
    tt = np.asarray(s['t'], dtype=float)
    rss = np.asarray(s['pipeline_rss_mb'], dtype=float)
    slope = float(np.polyfit((tt - tt[0]) / 60.0, rss, 1)[0]) if len(tt) > 20 else None
    summ = {'pipeline_cpu_pct_of_core_mean': float(np.nanmean(s['pipeline_cpu_pct_of_core'])), 'pipeline_cpu_pct_of_core_p95': float(np.nanpercentile(s['pipeline_cpu_pct_of_core'], 95)),
            'pipeline_rss_mb_mean': float(np.nanmean(rss)), 'rss_slope_mb_per_min': slope, 'busiest_thread_pct_mean': float(np.nanmean(bt)), 'busiest_thread_pct_max': float(np.nanmax(bt)),
            'busiest_thread_process_most_often': top, 'mem_available_mb_min': float(np.nanmin(s['mem_available_mb'])), 'swap_used_mb_max': float(np.nanmax(s['swap_used_mb'])),
            'soc_c_min': float(np.nanmin(s['soc_c'])), 'soc_c_max': float(np.nanmax(s['soc_c'])), 'cpu_total_pct_mean': float(np.nanmean(s['cpu_total_pct'])),
            'note': 'baseline without any viewer; thermal node and watchdog running (' + thermal_label() + ')', 'window': [t0, t1]}
    T.num('pipeline_cpu_pct_of_core', round(summ['pipeline_cpu_pct_of_core_mean'], 1), '% of one core', '05_cpu_memory/cpu_per_process.csv', 'no viewer; thermal node running')
    T.num('pipeline_mem_mb', round(summ['pipeline_rss_mb_mean'], 0), 'MB', '05_cpu_memory/cpu_per_process.csv')
    T.num('soak_mem_slope_mb_per_min', round(slope, 4) if slope is not None else None, 'MB/min', '00_meta/system_1hz.csv', f'{(t1 - t0) / 60:.0f} min window')
    finish('05_cpu_memory', '', summ, 'CPU, threads, memory (P6)', ['cpu_per_process.csv, cpu_per_core.csv from system_1hz.csv over the window (1 Hz samples); busiest-thread CPU is the largest single-thread load among pipeline processes.'], t0)


# ================================================================== P7 stage timing (bench.py, as before)
def phase_stage_timing():
    T.phase_stage_timing()


# ================================================================== P8 freshness
def phase_freshness():
    t0 = time.time()
    T.sleep_checked(T.D(300, 30))
    t1 = time.time()
    d = P('04_freshness')
    rows, res = [], {}
    for key, what in (('lidar_restamped', 'LiDAR scan (host-clock stamp) to arrival at the probe'), ('rgb', 'RGB frame stamp to arrival'),
                      ('fused', 'fused cloud (carries the LiDAR scan stamp): LiDAR scan to fused cloud arrival'), ('merged', 'merged cloud: stamp to arrival')):
        a = ages_ms(key, t0, t1)
        if len(a):
            res[key] = T.pstats(a)
            rows.append([key, what, len(a), f'{np.median(a):.1f}', f'{np.percentile(a, 95):.1f}', f'{np.percentile(a, 99):.1f}', f'{a.max():.1f}'])
            T.write_csv(d / f'age_samples_{key}.csv', ['age_ms'], [[f'{v:.2f}'] for v in a])
    T.write_csv(d / 'freshness_stages.csv', ['topic', 'meaning', 'n', 'median_ms', 'p95_ms', 'p99_ms', 'max_ms'], rows)
    if 'merged' in res:
        res['merged_content_age_s_est'] = (res['merged']['median'] / 1000.0) + 0.25
        T.num('age_merged_p50_ms', round(res['merged']['median'], 1), 'ms', '04_freshness/freshness_stages.csv', 'ROS-level (stamp to arrival on the Pi); excludes streamer encode and network')
        T.num('age_merged_p95_ms', round(res['merged']['p95'], 1), 'ms', '04_freshness/freshness_stages.csv', 'ROS-level')
        T.num('merged_content_age_s', round(res['merged_content_age_s_est'], 3), 's', '04_freshness/freshness_stages.csv', 'calculated: ROS-level age + half the 0.5 s merge window')
    if 'fused' in res:
        numx('age_fused_p50_ms', round(res['fused']['median'], 1), 'ms', '04_freshness/freshness_stages.csv')
        T.num('age_live_p50_ms', round(res['fused']['median'], 1), 'ms', '04_freshness/freshness_stages.csv', 'ROS-level age of the fused (live) cloud; no viewer, no streamer encode, no network')
        T.num('age_live_p95_ms', round(res['fused']['p95'], 1), 'ms', '04_freshness/freshness_stages.csv', 'ROS-level')
    finish('04_freshness', '', res, 'Freshness (P8)',
           ['Data age from message stamps versus arrival time at the independent probe on the Pi. Stage ages only: no viewer was connected, so streamer encode and network delay are not included (see 03_timing_stages for encode time).'], t0)


# ================================================================== P9 alignment sweep
def phase_alignment():
    t0 = time.time()
    d = SD('08_accuracy', 'alignment')
    bags = S.out / 'bags'
    bags.mkdir(exist_ok=True)
    raw = S.out / 'raw_captures'
    raw.mkdir(exist_ok=True)
    free_gb = shutil.disk_usage(S.out).free / 1e9
    if free_gb < 10:
        raise RuntimeError(f'disk free {free_gb:.1f} GB < 10 GB: alignment bag not recorded')
    secs = int(T.D(60, 12))
    bag = bags / f'alignment_{time.strftime("%H%M%S")}'
    topics = ['/landfill/livox/points', '/landfill/rgb/image_raw', '/landfill/rgb/camera_info', '/tf_static']
    bp = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag)] + topics, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    time.sleep(3)
    cap = T.capture('lidar:60,rgb:6,caminfo:1', 'alignment', 90)
    left = secs - 3 - (time.time() - t0)
    if left > 0:
        T.sleep_checked(left)
    try:
        os.killpg(bp.pid, signal.SIGINT)
        bp.wait(40)
    except Exception:
        stop_proc(bp)
    if not cap:
        raise RuntimeError('alignment capture failed')
    z = np.load(cap, allow_pickle=True)
    xyz = z['lidar_xyz']
    xyz = xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64)
    frames = [z['rgb_img'][i] for i in range(len(z['rgb_img']))]
    cal = G.load_calibration()
    import cv2
    grads = [G.gradient_map(f) for f in frames]
    edges = G.edge_points(xyz, cal)
    ex = xyz[edges]
    er = np.linalg.norm(ex, axis=1)
    res = {'n_points': int(len(xyz)), 'n_edge_points': int(len(edges)), 'frames': len(frames), 'bag': str(bag), 'bag_seconds': secs,
           'method': ('indirect: mean normalised image-gradient magnitude at the projected LiDAR depth-discontinuity points; higher = LiDAR depth edges coincide with image edges. '
                      'Offsets are artificial changes of the deployed lidar-to-camera transform, applied in the camera frame (x forward, y left, z up). Not ground-truth accuracy.')}
    sweeps = [('x_forward', 'm', [i * 0.01 for i in range(-5, 6)], lambda o: ((o, 0, 0), None)), ('y_left', 'm', [i * 0.01 for i in range(-5, 6)], lambda o: ((0, o, 0), None)),
              ('z_up', 'm', [i * 0.01 for i in range(-5, 6)], lambda o: ((0, 0, o), None)), ('roll_about_x', 'deg', [i * 0.5 for i in range(-6, 7)], lambda o: ((0, 0, 0), ('x', o))),
              ('pitch_about_y', 'deg', [i * 0.5 for i in range(-6, 7)], lambda o: ((0, 0, 0), ('y', o))), ('yaw_about_z', 'deg', [i * 0.5 for i in range(-6, 7)], lambda o: ((0, 0, 0), ('z', o)))]
    # baseline: gradient at ALL projected points (the expected score for a random alignment)
    pb = []
    u, v, ok = G.project(G.to_camera(xyz[::20], cal), cal['intr'])
    for g in grads:
        pb.append(float(g[v[ok].astype(int), u[ok].astype(int)].mean()))
    res['baseline_gradient_all_points'] = float(np.mean(pb))
    rows, summary_bands = [], {}
    base_proj = {}
    for bname, lo, hi in ALIGN_BANDS:
        sel = (er >= lo) & (er < hi)
        base_proj[bname] = sel
        n_edge = int(sel.sum())
        sc0, n0, c0 = G.score_offset(ex[sel], grads, cal) if n_edge else (float('nan'), 0, (np.nan, np.nan))
        info = {'n_edge_points': n_edge, 'score_deployed': sc0, 'enough_edges': n_edge >= 300 and np.isfinite(sc0)}
        peaks = {}
        for axis, unit, vals, mk in sweeps:
            sc = []
            for o in vals:
                tr, rot = mk(o)
                s_, n_, c_ = G.score_offset(ex[sel], grads, cal, tr, rot) if n_edge else (float('nan'), 0, (np.nan, np.nan))
                sc.append(s_)
                rows.append([bname, axis, unit, f'{o:.3f}', f'{s_:.5f}' if np.isfinite(s_) else '', n_, f'{c_[0]:.2f}', f'{c_[1]:.2f}'])
            arr = np.array(sc)
            if np.isfinite(arr).any():
                k = int(np.nanargmax(arr))
                tr, rot = mk(vals[k])
                pc0 = G.to_camera(ex[sel], cal)
                pc1 = G.to_camera(ex[sel], cal, tr, rot)
                u0, v0, ok0 = G.project(pc0, cal['intr'], False)
                u1, v1, ok1 = G.project(pc1, cal['intr'], False)
                both = ok0 & ok1
                shift = float(np.hypot(u1[both] - u0[both], v1[both] - v0[both]).mean()) if both.any() else None
                peaks[axis] = {'peak_offset': vals[k], 'unit': unit, 'score_at_peak': float(arr[k]), 'mean_pixel_shift_at_peak': shift}
        info['peaks'] = peaks
        best_axis = max(peaks, key=lambda a: peaks[a]['score_at_peak']) if peaks else None
        info['best_single_axis'] = best_axis
        info['score_peak'] = peaks[best_axis]['score_at_peak'] if best_axis else None
        info['score_ratio_deployed_to_peak'] = (sc0 / info['score_peak']) if best_axis and info['score_peak'] else None
        info['score_over_random_baseline'] = (sc0 / res['baseline_gradient_all_points']) if np.isfinite(sc0) else None
        if not info['enough_edges']:
            info['verdict'] = 'scene has too few edges for a reliable alignment score in this range band'
        else:
            info['verdict'] = ('deployed alignment is at (or within one sweep step of) the sweep maximum' if info['score_ratio_deployed_to_peak'] and info['score_ratio_deployed_to_peak'] > 0.98
                               else 'a different offset scores higher than the deployed calibration; see peaks (indirect measure)')
        summary_bands[bname] = info
    T.write_csv(d / 'sweep_scores.csv', ['range_band', 'axis', 'unit', 'offset', 'score', 'n_points_in_image', 'mean_u_px', 'mean_v_px'], rows)
    res['bands'] = summary_bands
    for bname, key in (('below_10m', 'align_score_ratio_deployed_to_peak_below10'), ('10_to_20m', 'align_score_ratio_deployed_to_peak_10_20'), ('above_20m', 'align_score_ratio_deployed_to_peak_above20')):
        info = summary_bands[bname]
        v_ = info['score_ratio_deployed_to_peak'] if info['enough_edges'] else None
        numx(key, round(v_, 4) if v_ is not None else None, '', '08_accuracy/alignment/sweep_scores.csv', 'indirect alignment measure' if v_ is not None else 'scene has too few edges in this band')
    # example overlays: deployed versus the combination of per-axis peaks (if that scores higher)
    try:
        best_tr, best_rot_axes = [0.0, 0.0, 0.0], {}
        allb = {'n': 0}
        pk = {}
        for ax_ in ('x_forward', 'y_left', 'z_up'):
            vals = [summary_bands[b]['peaks'].get(ax_, {}).get('peak_offset') for b, _, _ in ALIGN_BANDS if summary_bands[b]['enough_edges']]
            vals = [v_ for v_ in vals if v_ is not None]
            pk[ax_] = float(np.median(vals)) if vals else 0.0
        best_tr = (pk['x_forward'], pk['y_left'], pk['z_up'])
        img = frames[0]
        sub = xyz[::6]
        import matplotlib
        matplotlib.use('Agg')
        plt = G._mpl()
        fig, axs = plt.subplots(1, 2, figsize=(6.4, 2.6))
        for ax, tr, ttl in ((axs[0], (0, 0, 0), 'deployed calibration'), (axs[1], best_tr, 'best single-axis offsets (median of range bands)')):
            pc = G.to_camera(sub, cal, tr)
            u, v, ok = G.project(pc, cal['intr'], False)
            ax.imshow(img)
            ax.grid(False)
            ax.scatter(u[ok], v[ok], c=np.clip(np.linalg.norm(sub[ok], axis=1), 0, 30), s=0.4, cmap='turbo', linewidths=0)
            ax.set_title(ttl, fontsize=7)
            ax.set_xticks([])
            ax.set_yticks([])
        fig.tight_layout()
        fig.savefig(S.out / 'images' / 'alignment_example.png', dpi=300)
        plt.close(fig)
        res['example_offsets_m'] = best_tr
    except Exception as exc:
        res['example_error'] = repr(exc)
    shutil.move(str(cap), str(raw / 'alignment_capture.npz'))
    finish('08_accuracy', 'alignment', res, 'RGB / LiDAR alignment sweep (P9)',
           ['sweep_scores.csv: score versus artificial offset, per axis and range band (INDIRECT measure: LiDAR depth edges versus image gradient edges, not ground-truth accuracy).',
            'The 60 s bag (LiDAR points, RGB, camera info, tf) is kept on the SD card under bags/ (not in the bundle); the analysis used a simultaneous capture of 60 scans and 6 frames (raw_captures/).'], t0)


# ================================================================== P10 repo extract
def phase_repo_extract():
    t0 = time.time()
    d = SD('00_meta', 'repo_extract')
    repo = HOME / 'PitFusion3D_repo'
    rows = []
    pat = re.compile(r'(before|after|→|->|was|now|reduced|improv|faster|slower|from \d)', re.I)
    num_re = re.compile(r'\d')
    files = []
    for sub in ('docs',):
        files += [f for f in (repo / sub).rglob('*') if f.is_file() and f.suffix.lower() in ('.md', '.txt', '.csv', '.json')] if (repo / sub).exists() else []
    files += [f for f in repo.rglob('final_campaign_*') if f.is_file()]
    for f in sorted(set(files)):
        try:
            for i, line in enumerate(f.read_text(errors='ignore').splitlines(), 1):
                if re.search(r'before', line, re.I) and re.search(r'after', line, re.I) and num_re.search(line) and len(line) < 400:
                    rows.append(['file', str(f.relative_to(repo)), i, '', line.strip()[:300]])
                elif re.search(r'(optimi|speed|latency|cpu|fps|ms\b)', line, re.I) and re.search(r'→|->|from [\d.]+ .* to [\d.]+', line) and len(line) < 300:
                    rows.append(['file', str(f.relative_to(repo)), i, '', line.strip()[:300]])
        except Exception:
            pass
    rc, out = T.sh(f'git -C {repo} log --all --format="%H%x09%ad%x09%s%n%b%x00" --date=short -i --grep="before" --grep="after" --grep="optimi" --grep="faster" --grep="reduce" 2>/dev/null', 60)
    for block in out.split('\x00'):
        block = block.strip()
        if not block:
            continue
        head, _, body = block.partition('\n')
        parts = head.split('\t')
        if len(parts) < 3:
            continue
        for line in (parts[2] + '\n' + body).splitlines():
            if num_re.search(line) and pat.search(line) and len(line) < 300:
                rows.append(['commit', parts[0][:10], '', parts[1], line.strip()[:300]])
    T.write_csv(d / 'repo_extract_candidates.csv', ['source_kind', 'source (file or commit)', 'line', 'date', 'text'], rows)
    (d / 'repo_extract.md').write_text('# Candidate before/after optimisation results found in the repository\n\nAutomatic search of docs/, final_campaign_* files and commit messages for lines that state a before and an after (or a change with numbers).\n'
                                       'These are CANDIDATES for the TODO in 3.3.1: a person must check each against Figure 12 and Tables 7 and 8 (not available to the script). Nothing here is a measurement of Run 1.\n\n'
                                       + (f'{len(rows)} candidate lines; see repo_extract_candidates.csv (source file or commit, line, text).\n' if rows else 'No candidate lines found.\n'))
    finish('00_meta', 'repo_extract', {'candidates': len(rows), 'files_searched': len(files)}, 'Repo extract (P10)', ['repo_extract_candidates.csv: metric lines with a before and an after, with the file or commit they came from. No test is run.'], t0)


PHASE_FN = {'inventory': phase_inventory, 'scene': phase_scene, 'rates': phase_rates, 'combinations': phase_combinations, 'range_accuracy': phase_range_accuracy,
            'cpu_mem': phase_cpu_mem, 'stage_timing': phase_stage_timing, 'freshness': phase_freshness, 'alignment': phase_alignment, 'repo_extract': phase_repo_extract}


# ================================================================== run control
def stop_watcher():
    while True:
        if STOP_FILE.exists() and not S.stop_flag:
            log('STOP file found: finishing early')
            S.stop_reason = 'stopfile'
            S.stop_flag = True
        time.sleep(2)


def finalize_run(args):
    import run1_report
    S.phase = 'finalising'
    T.kill_stray_workers()
    try:
        T.stop_probe()
    except Exception:
        pass
    for th in S.threads:
        th.stop_flag = True
    time.sleep(2)
    try:
        T.power_by_phase()
    except Exception as exc:
        log(f'power_by_phase failed: {exc!r}')
    raw = S.out / 'raw_captures'
    raw.mkdir(exist_ok=True)
    for f in (S.out / '00_meta').glob('_capture_*.npz'):
        shutil.move(str(f), str(raw / f.name))
    save_numbers()
    try:
        run1_report.build_all(S.out, args)
    except Exception as exc:
        log(f'report build failed: {exc!r}\n{traceback.format_exc()}')
    T.jwrite(S.out / 'DONE.json', {'finished': time.time(), 'iso': time.strftime('%Y-%m-%dT%H:%M:%S'), 'hotspot_left_running': S.hotspot.get('status') == 'up', 'phases': RS.d['phases']})
    RS.d['finalized'] = True
    RS.save()
    if not args.dry_run:
        ARMED.unlink(missing_ok=True)
        T.sh(['systemctl', '--user', 'disable', 'thesis-offline.service'], 20)
    log('Run 1 finished: results in ' + str(S.out))


def main():
    global RS
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', choices=['full', 'quick'], default='full')
    ap.add_argument('--countdown', type=int, default=120)
    ap.add_argument('--run-dir', default='')
    ap.add_argument('--no-hotspot', action='store_true')
    ap.add_argument('--dry-run', action='store_true', help='do not touch the ARMED flag or the service')
    ap.add_argument('--report-only', default='', help='rebuild reports for an existing run directory and exit')
    args = ap.parse_args()
    if args.report_only:
        import run1_report
        run1_report.build_all(Path(args.report_only), args)
        return
    run_dir = Path(args.run_dir) if args.run_dir else Path(ARMED.read_text().strip())
    run_dir.mkdir(parents=True, exist_ok=True)
    RS = RunState(run_dir / 'state.json')
    if RS.d.get('finalized'):
        if not args.dry_run:
            ARMED.unlink(missing_ok=True)
        print('this run is already finished')
        return
    site = read_site()
    S.scale = {'full': 1.0, 'quick': 0.04}[args.profile]
    S.out = run_dir
    S.markers_path = str(run_dir / '00_meta' / 'markers.jsonl')
    S.stop_reason = ''
    ns = SimpleNamespace(profile=args.profile, countdown=args.countdown, no_hotspot=args.no_hotspot, return_home=False, battery_run=False, only='', skip='',
                         ambient=float(site['AMBIENT_C']) if site.get('AMBIENT_C') else None, enclosure=('closed, clear lid' if site.get('LID_FITTED', 'yes').lower().startswith('y') else site.get('LID_FITTED', 'not supplied')),
                         power_source='Anker 737 via PD trigger + 52Pi', cold_wait_min=0.0, ap_band='', ap_channel='', run_id=run_dir.name, dry_run=args.dry_run)
    S.args = ns
    for dname in ('00_meta', '02_rates', '03_timing_stages', '04_freshness', '05_cpu_memory', '06_tradeoffs', '08_accuracy', '09_power', '14_wireless', '15_fusion_info', '16_human_tests', 'scripts', 'images', 'figures', 'tables'):
        P(dname)
    for f in ('TestRun1.py', 'run1_geo.py', 'run1_report.py', 'auto_report.py', 'TestWithLidV1.py', 'probe.py', 'bench.py', 'arm.sh'):
        if (HERE / f).exists():
            shutil.copy(HERE / f, run_dir / 'scripts' / f)
    if (run_dir / 'phase_status.json').exists():
        try:
            S.status_log = json.load(open(run_dir / 'phase_status.json'))
        except Exception:
            pass
    if (run_dir / 'numbers_state.json').exists():
        try:
            S.numbers = json.load(open(run_dir / 'numbers_state.json'))
        except Exception:
            pass
    S.phases = ORDER
    RS.d['boots'].append(time.strftime('%Y-%m-%dT%H:%M:%S'))
    RS.save()
    resumed = len(RS.d['boots']) > 1
    signal.signal(signal.SIGTERM, lambda *a: (setattr(S, 'stop_reason', S.stop_reason or 'signal'), setattr(S, 'stop_flag', True)))
    threading.Thread(target=stop_watcher, daemon=True).start()
    log(f'Run 1 start (boot #{len(RS.d["boots"])}, resumed={resumed}), profile {args.profile}, output {run_dir}')
    for cls in (T.PowerThread, T.SystemSampler, T.WifiMonitor):
        th = cls()
        th.start()
        S.threads.append(th)
    threading.Thread(target=T.status_thread, daemon=True).start()
    hs = None
    if not args.no_hotspot:
        hs = threading.Thread(target=lambda: T.hotspot_up(), daemon=True)
        hs.start()
    else:
        S.hotspot = {'requested': False, 'status': 'skipped (--no-hotspot)'}
    try:
        if not RS.d['countdown_done']:
            end = time.time() + args.countdown
            while time.time() < end:
                T.check_stop()
                left = int(end - time.time())
                T.jwrite(run_dir / 'COUNTDOWN.json', {'seconds_left': left, 'hotspot': S.hotspot.get('status')})
                if left % 10 == 0:
                    log(f'countdown: {left} s left (hotspot: {S.hotspot.get("status")})')
                time.sleep(1)
            RS.d['countdown_done'] = True
            RS.save()
        else:
            log('countdown already done in an earlier boot: not repeated')
        if hs:
            hs.join(timeout=600)
        if T.wait_ready(240) is None:
            log('WARNING: the pipeline is not serving clouds yet; continuing (phases record what happens)')
        os.environ['PROBE_RGB_DIR'] = str(run_dir / 'images' / 'rgb_every_minute')
        os.environ['PROBE_RGB_EVERY'] = str(max(60.0 * S.scale, 5.0) if args.profile == 'quick' else 60.0)
        T.start_probe()
        time.sleep(15)
        log('thermal camera at start: ' + thermal_label())
        T.settle_gate()
        for name in ORDER:
            if RS.d['phases'].get(name):
                log(f'phase {name} already done in an earlier boot ({RS.d["phases"][name]}): skipped')
                continue
            status = T.run_phase(name, PHASE_FN[name], EST_MIN[name])
            RS.d['phases'][name] = status
            RS.save()
            save_numbers()
            T.probe_alive()
    except T.Stop:
        if S.stop_reason == 'stopfile':
            log('stop requested by the STOP file: finalising early')
        else:
            log('terminated (shutdown or service stop): leaving the run resumable; it continues at the next start')
            save_numbers()
            os._exit(0)
    except Exception as exc:
        log(f'FATAL in main loop: {exc!r}\n{traceback.format_exc()}')
    finally:
        pass
    finalize_run(args)
    if S.stop_reason == 'stopfile':
        STOP_FILE.unlink(missing_ok=True)
    os._exit(0)


if __name__ == '__main__':
    main()
