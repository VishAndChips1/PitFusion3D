#!/usr/bin/env python3
"""Thesis-quality figures and infographics from the campaign data.

Output: thesis_figures/<name>.png (200 dpi) and .pdf (vector), plus thesis_figures/CAPTIONS.md.
Conventions: one fixed colour per entity in every figure (LiDAR blue, RGB green, thermal red,
fused violet, merged orange, viewer yellow); one y-axis per chart; direct labels; text in neutral
ink; status colours always paired with a symbol.
"""
import collections
import glob
import json
import os
import re
import subprocess
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, Rectangle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rb_common import DATA, HERE, load_csv, phase_windows, pj, sub_markers   # noqa: E402

OUT = f'{HERE}/thesis_figures'
os.makedirs(OUT, exist_ok=True)

INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
LIDAR, RGB, THERMAL, FUSED, MERGED, VIEWER, NEUTRAL = '#2a78d6', '#1baf7a', '#e34948', '#6250d6', '#eb6834', '#eda100', '#8a8984'
GOOD, WARN, SERIOUS, CRIT = '#0ca30c', '#fab219', '#ec835a', '#d03b3b'
BLUE_RAMP = ['#cfe0f7', '#9fc2ee', '#6ea3e4', '#2a78d6', '#1c57a0', '#143d73']

plt.rcParams.update({
    'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF, 'font.size': 10, 'text.color': INK,
    'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2, 'axes.edgecolor': GRID, 'axes.grid': True,
    'grid.color': GRID, 'grid.linewidth': 0.6, 'axes.spines.top': False, 'axes.spines.right': False,
    'axes.titlesize': 11, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left', 'legend.frameon': False,
    'font.family': 'DejaVu Sans',
})
CAPTIONS = []
ENTITY = {'LiDAR': LIDAR, 'RGB': RGB, 'thermal': THERMAL, 'fused': FUSED, 'merged': MERGED, 'viewer': VIEWER}


def save(fig, name, title, caption, source='Source: campaign 2026-11-12 (Raspberry Pi 5 rig).'):
    try:
        fig.tight_layout(rect=(0, 0.025, 1, 1))
    except Exception:
        pass
    fig.text(0.01, 0.005, source, fontsize=7, color=INK2, va='bottom')
    fig.savefig(f'{OUT}/{name}.png', dpi=200)
    fig.savefig(f'{OUT}/{name}.pdf')
    plt.close(fig)
    CAPTIONS.append((name, title, caption))
    print('figure', name)


def box(ax, x, y, w, h, text, colour, fs=8, text_colour='#ffffff', sub=None):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.08', fc=colour, ec='none'))
    ax.text(x + w / 2, y + h / 2 + (0.12 if sub else 0), text, ha='center', va='center', fontsize=fs, color=text_colour, weight='bold')
    if sub:
        ax.text(x + w / 2, y + h / 2 - 0.2, sub, ha='center', va='center', fontsize=fs - 1.5, color=text_colour)


def arrow(ax, x0, y0, x1, y1, label=None, colour=INK2, off=(0, 0.12)):
    ax.annotate('', xy=(x1, y1), xytext=(x0, y0), arrowprops=dict(arrowstyle='-|>', color=colour, lw=1.3))
    if label:
        ax.text((x0 + x1) / 2 + off[0], (y0 + y1) / 2 + off[1], label, fontsize=6.5, color=INK2, ha='center')


def active_minutes(t):
    """Minutes since start with long gaps (power loss) removed, plus the mapping function."""
    gaps = [(t[i], t[i + 1]) for i in np.where(np.diff(t) > 30)[0]]

    def am(x):
        x = np.asarray(x, float)
        out = (x - t[0]) / 60.0
        for a, b in gaps:
            out = out - np.clip(x - a, 0, b - a) / 60.0
        return out
    return am


# --------------------------------------------------------------------------------------------- 01
def fig_architecture():
    fig, ax = plt.subplots(figsize=(13, 6.2))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 6.2)
    ax.axis('off')
    ax.grid(False)
    ax.set_title('System architecture: sensors, processes and data flow (Raspberry Pi 5)', pad=10)
    # sensors
    for i, (name, sub, col) in enumerate([('Livox Avia LiDAR', 'Ethernet/UDP, 24 000 pts @ 10 Hz', LIDAR), ('Pi HQ camera (IMX477)', 'CSI, 1280x720 @ 10 fps', RGB), ('Thermal camera', 'USB UVC, 640x512 @ 30 fps', THERMAL)]):
        box(ax, 0.1, 4.6 - i * 1.7, 2.3, 1.0, name, col, 8, sub=sub)
    # nodes
    box(ax, 3.0, 4.6, 2.1, 1.0, 'livox driver', LIDAR, 8, sub='vendor C++, device clock')
    box(ax, 5.5, 4.6, 2.0, 1.0, 'restamp node', LIDAR, 8, sub='min-delay offset filter')
    box(ax, 3.0, 2.9, 2.1, 1.0, 'rgb_camera_node', RGB, 8, sub='rpicam-vid pipe + thread')
    box(ax, 3.0, 1.2, 2.1, 1.0, 'thermal_camera_node', THERMAL, 8, sub='V4L2 + self-healing thread')
    box(ax, 8.0, 2.9, 2.2, 1.6, 'sensor_fusion_node', FUSED, 9, sub='time-sync, project + colour\nfused cloud 576 kB @ 10 Hz')
    box(ax, 10.6, 4.6, 2.3, 1.0, 'registration_merge', MERGED, 8, sub='worker thread, ICP')
    box(ax, 10.6, 2.9, 2.3, 1.0, 'fusion_web_streamer', VIEWER, 8, text_colour=INK, sub='HTTP :8080, lazy encode')
    box(ax, 10.6, 1.2, 2.3, 1.0, 'browser viewer', NEUTRAL, 8, sub='laptop / phone pulls data')
    box(ax, 8.0, 0.35, 2.2, 0.8, 'calibration_publisher', NEUTRAL, 7, sub='TF + CameraInfo (static)')
    for y in (5.1, 3.4, 1.7):
        arrow(ax, 2.4, y if y != 3.4 else 3.4, 3.0, y)
    arrow(ax, 5.1, 5.1, 5.5, 5.1)
    arrow(ax, 7.5, 5.1, 8.9, 4.5, '10 Hz, 432 kB', LIDAR, off=(0.3, 0.15))
    arrow(ax, 5.1, 3.4, 8.0, 3.7, 'rgb8 2.76 MB @10 Hz', RGB, off=(-0.6, 0.35))
    arrow(ax, 5.1, 1.7, 8.0, 3.1, 'mono16 @30 Hz', THERMAL, off=(0.3, -0.25))
    arrow(ax, 10.2, 4.0, 10.6, 5.0, None, FUSED)
    arrow(ax, 10.2, 3.5, 10.6, 3.4, None, FUSED)
    arrow(ax, 11.75, 4.6, 11.75, 3.9, 'merged ~2 MB @1.5 Hz', MERGED, off=(0.9, 0))
    arrow(ax, 11.75, 2.9, 11.75, 2.2, 'gzip, 11 B/pt', VIEWER, off=(0.7, 0))
    ax.text(0.1, 0.62, 'Between nodes: Fast DDS shared memory (64 MB segments), BEST_EFFORT QoS.', fontsize=8.5, color=INK2, style='italic')
    ax.text(0.1, 0.25, 'Every node is a separate OS process under one systemd user service and respawns within 2 s.', fontsize=8.5, color=INK2)
    save(fig, 'F01_architecture', 'System architecture', 'Block diagram of the whole system. Colour identifies the data stream (LiDAR blue, RGB green, thermal red, fused violet, merged orange, viewer yellow) and is used for the same streams in every other figure. Use as the overview figure of the methods chapter.')


# --------------------------------------------------------------------------------------------- 02
def fig_processes():
    ip = pj('phase_idle_pipeline') or {}
    S = load_csv(f'{DATA}/system_1hz.csv')
    mk = dict(sub_markers('idle_pipeline'))
    if not S or 'steady_begin' not in mk:
        return
    m = (S['t'] >= mk['steady_begin']) & (S['t'] <= mk['steady_end'])
    names = [('livox_driver', 'Livox driver'), ('restamp', 'restamp'), ('rgb_camera', 'RGB camera'), ('thermal_camera', 'thermal camera'), ('calibration', 'calibration'),
             ('fusion', 'fusion'), ('merge', 'merge (ICP)'), ('streamer', 'web streamer'), ('rpicam_vid', 'rpicam-vid')]
    cpu = [np.nanmean(S[f'{k}_cpu'][m]) for k, _ in names]
    rss = [np.nanmean(S[f'{k}_rss_mb'][m]) for k, _ in names]
    thr = [np.nanmax(S[f'{k}_threads'][m]) for k, _ in names]
    order = np.argsort(cpu)
    fig, ax = plt.subplots(1, 3, figsize=(13, 4.2), sharey=True)
    y = np.arange(len(names))
    for a, vals, label, col in ((ax[0], cpu, 'CPU (% of one core)', FUSED), (ax[1], rss, 'resident memory (MB)', LIDAR), (ax[2], thr, 'threads', NEUTRAL)):
        a.barh(y, [vals[i] for i in order], color=col, height=0.62)
        for yi, i in zip(y, order):
            a.text(vals[i], yi, f' {vals[i]:.0f}' if label != 'CPU (% of one core)' else f' {vals[i]:.1f}', va='center', fontsize=8, color=INK)
        a.set_xlabel(label)
        a.set_xlim(0, max(vals) * 1.18)
        a.grid(axis='y', visible=False)
    ax[0].set_yticks(y)
    ax[0].set_yticklabels([names[i][1] for i in order])
    ax[0].set_title('Per-process cost of the running pipeline (no viewer)')
    fig.suptitle(f'Total pipeline CPU {sum(cpu):.0f} % of 400 (4 cores) - {sum(rss):.0f} MB RSS - 9 processes, one GIL each', x=0.01, ha='left', fontsize=9, color=INK2, y=0.02)
    save(fig, 'F02_process_cost', 'CPU, memory and threads per process', 'Steady-state cost of each pipeline process with no viewer connected. Merge (ICP) and fusion dominate; every node is its own process, so no Python GIL is shared between nodes.')


# --------------------------------------------------------------------------------------------- 03
def fig_latency():
    ip = pj('phase_idle_pipeline') or {}
    allt = {}
    for k in ('topics_group_lidar', 'topics_rgb', 'topics_fused_merged'):
        allt.update(ip.get(k) or {})
    rows = [('LiDAR (restamped)', '/landfill/livox/points', LIDAR), ('RGB camera', '/landfill/rgb/image_raw', RGB), ('fused cloud', '/landfill/fusion/points_rgbt', FUSED), ('merged cloud', '/landfill/fusion/points_merged', MERGED)]
    fig, ax = plt.subplots(figsize=(9, 3.6))
    for i, (lab, tp, col) in enumerate(rows):
        v = allt.get(tp) or {}
        if 'stamp_age_ms' not in v:
            continue
        a = v['stamp_age_ms']
        ax.barh(i, a['p50'], color=col, height=0.55)
        ax.plot([a['p95'], a['p95']], [i - 0.3, i + 0.3], color=INK, lw=1.5)
        ax.text(a['p95'] + 6, i, f"median {a['p50']:.0f} ms, p95 {a['p95']:.0f} ms", va='center', fontsize=8, color=INK)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([r[0] for r in rows])
    ax.invert_yaxis()
    ax.set_xlabel('age of the message when a subscriber receives it (ms after its header stamp)')
    ax.set_title('Pipeline latency by stage (bar = median, tick = 95th percentile)')
    ax.grid(axis='y', visible=False)
    save(fig, 'F03_latency_by_stage', 'Latency by stage', 'Age of each message at the moment it reaches a subscriber, measured against its own header stamp. The merged cloud is the oldest because it deliberately contains the scans of a 0.5-0.6 s window and carries the last scan\'s stamp.')


# --------------------------------------------------------------------------------------------- 04
def fig_wire():
    w = ((pj('phase_accuracy') or {}).get('wire') or {}).get('points_merged') or {}
    fig, ax = plt.subplots(figsize=(12, 3.8))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 3.8)
    ax.axis('off')
    ax.grid(False)
    ax.set_title('Viewer wire format: 24-byte header + 11 bytes per point, then gzip')
    ax.text(0.1, 3.25, 'header (24 B)', fontsize=9, weight='bold', color=INK)
    for i, (lab, wd) in enumerate([('n\nuint32', 1.1), ('seq\nuint32', 1.1), ('origin x,y,z\n3 x float32', 2.2), ('scale\nfloat32', 1.1)]):
        x0 = 0.1 + sum([1.1, 1.1, 2.2, 1.1][:i])
        box(ax, x0, 2.35, wd - 0.06, 0.8, lab, NEUTRAL, 7)
    ax.text(6.0, 3.25, 'one point (11 B), repeated n times', fontsize=9, weight='bold', color=INK)
    parts = [('x,y,z\n3 x int16 (6 B)', 3.6, LIDAR), ('r,g,b\n3 x uint8 (3 B)', 1.8, RGB), ('thermal\nint16 (2 B)', 1.2, THERMAL)]
    x0 = 6.0
    for lab, wd, col in parts:
        box(ax, x0, 2.35, wd - 0.06, 0.8, lab, col, 7)
        x0 += wd
    ax.text(0.1, 1.75, 'position: offset + scale quantisation, step = extent / 32000 (0.16 mm for a 5.2 m scene); max error = sqrt(3) x step / 2', fontsize=8.5, color=INK2)
    ax.text(0.1, 1.35, 'colour: lossless 8-bit.  thermal: value x 100 as int16, NaN sentinel.', fontsize=8.5, color=INK2)
    if w:
        t = [('ROS message', w['ros_msg_bytes'] / 1e6, NEUTRAL), ('wire, uncompressed', w['wire_uncompressed_bytes'] / 1e6, VIEWER), ('wire, gzip level 6', w['gzip']['gzip6']['bytes'] / 1e6, MERGED)]
        mx = max(x[1] for x in t)
        for i, (lab, v, col) in enumerate(t):
            ax.add_patch(Rectangle((2.4, 0.9 - i * 0.32), 9.0 * v / mx, 0.22, fc=col))
            ax.text(2.3, 1.0 - i * 0.32, lab, ha='right', va='center', fontsize=8, color=INK)
            ax.text(2.5 + 9.0 * v / mx, 1.0 - i * 0.32, f'{v:.2f} MB', va='center', fontsize=8, color=INK)
        ax.text(11.9, 0.1, f"measured on a real merged cloud ({w['points']:,} points)\nmax position error {w['position_error_mm']['max']:.3f} mm - colours exact", ha='right', fontsize=8, color=INK2)
    save(fig, 'F04_wire_format', 'Viewer wire format', 'Byte layout of the data served to the browser, with the size reduction measured on a real merged cloud and the measured quantisation error. The format adds about 0.14 mm of position error, far below the LiDAR\'s own noise.')


# --------------------------------------------------------------------------------------------- 05/06
def fig_power():
    P = load_csv(f'{DATA}/power_raw.csv')
    if not P:
        return
    t = P['t']
    est, cons, ext, watts, temp = P['est_in_a'], P['est_in_cons_a'], P['EXT5V_V'], P['watts'], P['temp_c']
    am = active_minutes(t)
    tt = am(t)
    W = phase_windows()
    fig, ax = plt.subplots(3, 1, figsize=(12, 7.5), sharex=True)
    step = max(1, len(t) // 25000)
    ax[0].plot(tt[::step], cons[::step], lw=.4, color=SERIOUS, label='conservative estimate (includes USB + regulator loss)')
    ax[0].plot(tt[::step], est[::step], lw=.4, color=LIDAR, label='estimated input current')
    ax[0].axhline(5, color=CRIT, lw=1.4)
    ax[0].text(tt[0], 5.08, '5 A shutdown limit', color=CRIT, fontsize=8)
    ax[0].set_ylim(0, 5.6)
    ax[0].set_ylabel('A')
    ax[0].legend(loc='center right', fontsize=8)
    ax[0].set_title('Input current, supply voltage and temperature over the whole campaign')
    ax[1].plot(tt[::step], ext[::step], lw=.4, color=RGB)
    ax[1].set_ylabel('EXT5V (V)')
    ax[2].plot(tt[::step], temp[::step], lw=.6, color=MERGED)
    ax[2].set_ylabel('SoC temperature (C)')
    ax[2].set_xlabel('active minutes since start (the 2-hour battery-loss gap is removed)')
    for name, (a, b) in W.items():
        if name in ('stress', 'soak', 'sweep', 'corner'):
            for x in ax:
                x.axvspan(float(am(a)), float(am(b)), color=NEUTRAL, alpha=.10)
            ax[0].text((float(am(a)) + float(am(b))) / 2, 5.3, name, ha='center', fontsize=8, color=INK2)
    save(fig, 'F05_power_timeline', 'Power timeline', 'Estimated input current (blue), a conservative bound (orange) and the 5 A shutdown limit (red) over 1 h 45 min of testing, with supply voltage and SoC temperature below. Shaded bands mark the stress ramp, sweep, soak and corner-case phases.')
    # histogram + headroom
    fig, ax = plt.subplots(1, 2, figsize=(12, 3.8), gridspec_kw={'width_ratios': [1.3, 1]})
    ax[0].hist(est, bins=110, color=LIDAR, log=True)
    ax[0].axvline(5, color=CRIT, lw=1.4)
    ax[0].text(4.9, ax[0].get_ylim()[1] * 0.5, '5 A limit', color=CRIT, ha='right', fontsize=8)
    ax[0].set_xlim(0, 5.3)
    ax[0].set_xlabel('estimated input current (A)')
    ax[0].set_ylabel('samples (log scale)')
    ax[0].set_title(f'Distribution of {len(est):,} current samples (~30 Hz)')
    pk, cp = est.max(), cons.max()
    for i, (lab, v, col) in enumerate([('median', np.median(est), LIDAR), ('99.9th percentile', np.percentile(est, 99.9), LIDAR), ('highest sample', pk, LIDAR), ('conservative highest', cp, SERIOUS)]):
        ax[1].barh(i, v, color=col, height=0.55)
        ax[1].text(v + 0.05, i, f'{v:.2f} A', va='center', fontsize=9, color=INK)
    ax[1].axvline(5, color=CRIT, lw=1.4)
    ax[1].text(4.95, 3.55, '5 A', color=CRIT, ha='right', fontsize=8)
    ax[1].set_yticks(range(4))
    ax[1].set_yticklabels(['median', '99.9th percentile', 'highest sample', 'conservative highest'])
    ax[1].invert_yaxis()
    ax[1].set_xlim(0, 5.3)
    ax[1].set_xlabel('A')
    ax[1].set_title(f'Headroom: at least {5 - cp:.1f} A below the limit')
    ax[1].grid(axis='y', visible=False)
    save(fig, 'F06_current_headroom', 'Current distribution and headroom', 'Left: distribution of every current sample. Right: the typical and extreme values against the 5 A limit. Sampling is about 30 ms, so spikes shorter than that cannot be seen; the conservative estimate bounds the unmetered USB and regulator losses.')


# --------------------------------------------------------------------------------------------- 07
def fig_stress():
    P = load_csv(f'{DATA}/power_raw.csv')
    S = load_csv(f'{DATA}/system_1hz.csv')
    sm = sub_markers('stress')
    steps = {}
    for e, tm in sm:
        if e.endswith('_begin'):
            steps.setdefault(e[:-6], [None, None])[0] = tm
        if e.endswith('_end'):
            steps.setdefault(e[:-4], [None, None])[1] = tm
    labels, cur, peak, w, tmax, cpu = [], [], [], [], [], []
    for name, (a, b) in steps.items():
        if name.startswith('cooldown') or a is None or b is None:
            continue
        m = (P['t'] >= a + 10) & (P['t'] <= b)
        ms = (S['t'] >= a + 10) & (S['t'] <= b)
        if m.sum() < 10:
            continue
        labels.append(name.replace('cpu_', '').replace('_workers', ' workers').replace('_worker', ' worker').replace('baseline_pipeline_only', 'pipeline only').replace('plus_512MB_mem', '+ memory'))
        cur.append(P['est_in_a'][m].mean())
        peak.append(P['est_in_a'][m].max())
        w.append(P['watts'][m].mean())
        tmax.append(P['temp_c'][m].max())
        cpu.append(np.nanmean(S['cpu_total_pct'][ms]))
    if not labels:
        return
    fig, ax = plt.subplots(1, 4, figsize=(14, 3.8))
    x = np.arange(len(labels))
    ax[0].bar(x - .2, cur, .4, color=LIDAR, label='mean')
    ax[0].bar(x + .2, peak, .4, color=SERIOUS, label='peak')
    ax[0].axhline(5, color=CRIT, lw=1.2)
    ax[0].set_ylim(0, 5.4)
    ax[0].set_title('Input current (A)')
    ax[0].legend(fontsize=7, loc='center right')
    ax[1].plot(x, w, 'o-', color=MERGED, lw=2)
    ax[1].set_title('SoC power (W)')
    ax[2].plot(x, tmax, 'o-', color=THERMAL, lw=2)
    ax[2].set_title('SoC peak temperature (C)')
    ax[3].plot(x, cpu, 'o-', color=FUSED, lw=2)
    ax[3].set_title('system CPU (% of 400)')
    for a in ax:
        a.set_xticks(x)
        a.set_xticklabels(labels, rotation=50, ha='right', fontsize=7)
    fig.suptitle('Stress ramp: power and temperature rise with load, then saturate once all four cores are busy; the highest spike was NOT under stress', x=0.01, ha='left', fontsize=9, color=INK2, y=0.02)
    save(fig, 'F07_stress_ramp', 'CPU stress ramp', 'Pipeline running while 0 to 4 busy workers (plus memory traffic) were added. Mean current and power plateau at two workers because the CPU is capped at 2.0 GHz and the cores are already saturated; the highest single current spike of the campaign occurred at idle, so spikes are not caused by sustained load.')


# --------------------------------------------------------------------------------------------- 08
def sweep_rows(tag):
    p = f'{HERE}/results_{tag}.jsonl'
    return [json.loads(line) for line in open(p)] if os.path.exists(p) else []


def fig_sweep():
    M = sweep_rows('final_merged')
    L = sweep_rows('final_live')
    nt = pj('phase_net') or {}
    if not M:
        return
    fig, ax = plt.subplots(2, 3, figsize=(14, 7))
    x = [r['cap'] for r in M]
    cov5 = [(r.get('quality') or {}).get('coverage_5cm_pct') for r in M]
    cov2 = [(r.get('quality') or {}).get('coverage_2cm_pct') for r in M]
    if any(v is not None for v in cov5):
        ax[0, 0].plot(x, cov5, 'o-', color=FUSED, lw=2, label='5 cm voxels')
        ax[0, 0].plot(x, cov2, 's--', color=LIDAR, lw=2, label='2 cm voxels')
        ax[0, 0].legend(fontsize=8)
    ax[0, 0].set_title('Fidelity vs full cloud (%)')
    ax[0, 1].plot(x, [r['quality']['nn_median_cm'] for r in M], 'o-', color=FUSED, lw=2)
    ax[0, 1].set_title('Median point spacing (cm)')
    ax[0, 2].plot(x, [r['viewer_mbit_s'] for r in M], 'o-', color=VIEWER, lw=2)
    dl = min((d['mbit_s'] for d in (nt.get('download') or {}).values()), default=None)
    ax[0, 2].set_title('Bandwidth to one viewer (Mbit/s)')
    if dl is not None:
        ax[0, 2].axhline(dl, color=CRIT, lw=1.2)
        ax[0, 2].text(x[0], dl + 0.15, f'measured WiFi download in the rig room: {dl:.1f} Mbit/s', color=CRIT, fontsize=7.5)
    ax[1, 0].plot(x, [r['cpu']['pipeline_pct'] for r in M], 'o-', color=MERGED, lw=2, label='whole pipeline')
    ax[1, 0].plot(x, [r['cpu']['streamer_pct'] for r in M], 's--', color=VIEWER, lw=2, label='web streamer')
    ax[1, 0].legend(fontsize=8)
    ax[1, 0].set_title('CPU (% of one core)')
    ax[1, 1].plot(x, [r['power']['est_input_a']['max'] for r in M], 'o-', color=LIDAR, lw=2, label='estimated peak')
    ax[1, 1].plot(x, [r['power']['est_input_conservative_a_max'] for r in M], 's--', color=SERIOUS, lw=2, label='conservative peak')
    ax[1, 1].axhline(5, color=CRIT, lw=1.2)
    ax[1, 1].set_ylim(0, 5.4)
    ax[1, 1].legend(fontsize=8, loc='center right')
    ax[1, 1].set_title('Peak input current (A)')
    ax[1, 2].plot(x, [r['resp_ms']['p95'] for r in M], 'o-', color=MERGED, lw=2)
    ax[1, 2].set_title('Viewer latency p95 (ms)')
    for a in ax.flat:
        a.set_xlabel('point cap (merged view)')
        a.axvspan(55000, 65000, color=GOOD, alpha=.12)
    ax[0, 0].text(60000, ax[0, 0].get_ylim()[0] + 2, 'suggested\ncap', ha='center', va='bottom', fontsize=7, color=INK2)
    save(fig, 'F08_pointcap_tradeoff', 'Point-cap trade-off', 'Effect of the number of points served per merged cloud on fidelity, bandwidth, CPU, peak current and latency (green band = suggested 60 000-point cap). Quality improves steadily while bandwidth and CPU rise; input current does not depend on the cap at all. Above about 70 000 the merged cloud itself contains no more points than the cap, so the 80 000 and 105 000 settings serve nearly the same cloud. The measured WiFi download speed in the rig room (red line) shows the radio, not the Pi, is the practical limit.')
    if L:
        fig, ax = plt.subplots(1, 3, figsize=(12, 3.6))
        x = [r['points_per_cloud'] for r in L]
        ax[0].plot(x, [r['viewer_mbit_s'] for r in L], 'o-', color=VIEWER, lw=2)
        ax[0].set_title('Bandwidth (Mbit/s)')
        ax[1].plot(x, [r['updates_per_s'] for r in L], 'o-', color=LIDAR, lw=2)
        ax[1].set_title('Updates per second to the viewer')
        ax[2].plot(x, [r['resp_ms']['p95'] for r in L], 'o-', color=MERGED, lw=2)
        ax[2].set_title('Request latency p95 (ms)')
        for a in ax:
            a.set_xlabel('points per live cloud')
        save(fig, 'F09_live_view_sweep', 'Live-view sweep', 'The live (single-scan, ~10 Hz) view for caps of 5 000 to 25 000 points: bandwidth grows roughly linearly while the update rate rises to about 8 per second.')


# --------------------------------------------------------------------------------------------- 10
def fig_rates():
    ip = pj('phase_idle_pipeline') or {}
    lt = pj('phase_latency') or {}
    allt = {}
    for k in lt:
        allt.update(lt[k])
    if not allt:
        for k in ('topics_group_lidar', 'topics_rgb', 'topics_fused_merged'):
            allt.update(ip.get(k) or {})
    rows = [('LiDAR', '/livox/lidar', LIDAR, 10), ('RGB', '/landfill/rgb/image_raw', RGB, 10), ('fused', '/landfill/fusion/points_rgbt', FUSED, 10), ('merged', '/landfill/fusion/points_merged', MERGED, 1.5)]
    fig, ax = plt.subplots(1, 2, figsize=(12, 3.8))
    for i, (lab, tp, col, nom) in enumerate(rows):
        v = allt.get(tp)
        if not v or 'interval_ms' not in v:
            continue
        ax[0].barh(i, v['hz'], color=col, height=0.55)
        ax[0].text(v['hz'] + 0.15, i, f"{v['hz']:.2f} Hz", va='center', fontsize=9)
        i95 = v['interval_ms']['p95'] / v['interval_ms']['p50']
        ax[1].barh(i, i95, color=col, height=0.55)
        ax[1].text(i95 + 0.03, i, f"p95 interval = {v['interval_ms']['p95']:.0f} ms ({i95:.2f}x median)", va='center', fontsize=8)
    for a in ax:
        a.set_yticks(range(4))
        a.set_yticklabels([r[0] for r in rows])
        a.invert_yaxis()
        a.grid(axis='y', visible=False)
    ax[0].set_xlabel('measured rate (Hz)')
    ax[0].set_title('Delivered message rates (steady state)')
    ax[1].set_xlabel('p95 inter-arrival interval relative to the median (1.0 = perfectly regular)')
    ax[1].set_title('Regularity (jitter)')
    ax[1].set_xlim(0, 5)
    save(fig, 'F10_rates_and_jitter', 'Rates and jitter', 'Left: delivered rate of each stream. Right: how irregular the arrivals are (95th-percentile interval relative to the median). The RGB stream is the most irregular because frames are stamped in bursts after a variable decode delay.')


# --------------------------------------------------------------------------------------------- 11
def fig_robustness():
    comps = ['LiDAR driver', 'restamp', 'RGB camera node', 'thermal camera node', 'calibration', 'sensor fusion', 'registration merge', 'web streamer', 'whole service']
    cols = ['hard kill\n(SIGKILL)', 'graceful\nrestart', 'NaN / empty /\ndegenerate data', 'clock jump /\nNTP step', 'overload /\nmany clients', 'sensor or\ndevice missing']
    # 2 pass, 1 partial, 0 fail, -1 not tested, 3 not applicable
    M = [[2, 2, 3, 3, 3, 3], [2, 2, 3, 2, 3, 3], [2, 2, 3, 3, 3, 2], [-1, 2, 3, 3, 3, 2], [2, 2, 3, 3, 3, 3], [2, 2, 2, 2, 3, 1], [2, 2, 2, 2, 2, 3], [2, 2, 2, 3, 2, 3], [-1, 2, 3, 2, 3, 3]]
    sym = {2: ('PASS', GOOD, '✓'), 1: ('PARTIAL', WARN, '!'), 0: ('FAIL', CRIT, '✗'), -1: ('not tested', NEUTRAL, '-'), 3: ('n/a', GRID, '')}
    fig, ax = plt.subplots(figsize=(10, 5.2))
    ax.set_xlim(0, len(cols))
    ax.set_ylim(len(comps), 0)
    ax.grid(False)
    for i in range(len(comps)):
        for j in range(len(cols)):
            _, c, s = sym[M[i][j]]
            ax.add_patch(Rectangle((j + .04, i + .06), .92, .88, fc=c, ec='none', alpha=1 if M[i][j] in (2, 1, 0) else .35))
            ax.text(j + .5, i + .5, s, ha='center', va='center', fontsize=13, color='#ffffff' if M[i][j] in (2, 0, -1) else INK, weight='bold')
    ax.set_xticks(np.arange(len(cols)) + .5)
    ax.set_xticklabels(cols, fontsize=8)
    ax.xaxis.tick_top()
    ax.set_yticks(np.arange(len(comps)) + .5)
    ax.set_yticklabels(comps)
    ax.tick_params(length=0)
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.set_title('Robustness matrix (✓ pass, ! partial, - not tested on purpose, blank = not applicable)', pad=34)
    fig.text(0.01, 0.06, 'Partial: thermal flapping at exactly the 1 s fallback timeout halves fused output. Not tested: hard-killing the thermal camera node (it left the camera unable to stream until power-cycled).', fontsize=7.5, color=INK2)
    save(fig, 'F11_robustness_matrix', 'Robustness matrix', 'Which failure classes were exercised against which component, from the corner-case suites, the SIGKILL/respawn test, the restart tests and a real NTP clock step. Status colours are always paired with a symbol.')


# --------------------------------------------------------------------------------------------- 12
def fig_camera_history():
    p = '/home/vish/power_logs/usb_events.log'
    if not os.path.exists(p):
        return
    ev = [line for line in open(p) if 'Thermal' in line and 'DISCONNECTED' in line]
    ts = [time.mktime(time.strptime(line[:19], '%Y-%m-%d %H:%M:%S')) for line in ev]
    ts = [x for x in ts if x >= time.mktime(time.strptime('2026-11-11 07:00:00', '%Y-%m-%d %H:%M:%S'))]
    t0 = time.mktime(time.strptime('2026-11-11 07:00:00', '%Y-%m-%d %H:%M:%S'))
    end = time.time()
    hrs = np.arange(0, (end - t0) / 3600 + 1)
    h, _ = np.histogram([(x - t0) / 3600 for x in ts], bins=np.append(hrs, hrs[-1] + 1))
    fig, ax = plt.subplots(figsize=(12, 3.8))
    ax.bar(hrs, h, width=0.9, color=THERMAL)
    ax.set_ylabel('camera USB disconnects per hour')
    labs = []
    for hh in range(0, int(hrs[-1]) + 1, 6):
        labs.append(time.strftime('%d %b %H:%M', time.localtime(t0 + hh * 3600)))
    ax.set_xticks(range(0, int(hrs[-1]) + 1, 6))
    ax.set_xticklabels(labs, rotation=35, ha='right', fontsize=8)
    marks = [('2026-11-11 16:53', 'worked 56 min'), ('2026-11-12 14:19', 'worked 8 min,\nthen hard-killed'), ('2026-11-12 20:23', 'battery loss ->\nworking again')]
    for s, lab in marks:
        x = (time.mktime(time.strptime(s, '%Y-%m-%d %H:%M')) - t0) / 3600
        ax.axvline(x, color=GOOD, lw=1.3)
        ax.text(x + 0.3, ax.get_ylim()[1] * 0.85, lab, color=INK, fontsize=7.5)
    ax.set_title('Thermal camera reliability: a stuck-state loop (about every 34 s) that only a full power loss has cleared')
    save(fig, 'F12_thermal_camera_history', 'Thermal camera reliability', 'USB disconnects per hour for the thermal camera. Each high bar is the stuck state: the camera enumerates, never sends frames and resets about every 34 s. Green lines mark the times it streamed correctly, each time after a full power loss. The watchdog added on 12 Nov detects the state and will auto-recover once the one-time permission step is done.')


# --------------------------------------------------------------------------------------------- 13
def fig_history():
    labels = ['before', '+ shared\nmemory', '+ merge\nthread', '+ lazy streamer\n+ RGB 10 fps', 'after']
    rgb = [3.6, 15.1, 15.0, 10.1, 10.1]
    fused = [2.42, 9.76, 9.75, 10.02, 10.03]
    merged = [0.66, 1.49, 1.50, 1.55, 1.53]
    cpu = [76, 159, 167, 117, 137]
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
    x = np.arange(5)
    ax[0].plot(x, fused, 'o-', color=FUSED, lw=2.2, label='fused cloud')
    ax[0].plot(x, rgb, 's-', color=RGB, lw=2.2, label='RGB reaching fusion')
    ax[0].set_title('Rates (Hz)')
    ax[0].legend(fontsize=8)
    ax[1].plot(x, merged, 'o-', color=MERGED, lw=2.2)
    ax[1].set_title('Merged cloud rate (Hz)')
    ax[2].bar(x, cpu, color=NEUTRAL, width=.6)
    for xi, c, f in zip(x, cpu, fused):
        ax[2].text(xi, c + 3, f'{c / f:.0f} %-core\nper fused Hz', ha='center', fontsize=7)
    ax[2].set_title('Pipeline CPU (% of 400)')
    ax[2].set_ylim(0, 200)
    for a in ax:
        a.set_xticks(x)
        a.set_xticklabels(labels, fontsize=7)
    save(fig, 'F13_before_after', 'Before and after the performance work', 'The staged improvements of the 2026-11-11 performance work (values from docs/performance_2026-11-11). Fused output rose 4x and merged 2.3x; CPU per delivered fused Hz fell from 31 to about 13 %-core.', source='Source: earlier campaign, 2026-11-11 (independent subscriber loading the pipeline\'s DDS profile).')


# --------------------------------------------------------------------------------------------- 14
def fig_gallery():
    files = {k: f'{DATA}/sample_{k}.npz' for k in ('lidar_scan', 'fused_cloud', 'merged_cloud')}
    if not all(os.path.exists(v) for v in files.values()):
        return
    L = np.load(files['lidar_scan'])
    F = np.load(files['fused_cloud'])
    M = np.load(files['merged_cloud'])

    def rgb_of(z):
        c = z['rgb']
        return np.stack([(c >> 16) & 255, (c >> 8) & 255, c & 255], 1) / 255.0
    fig, ax = plt.subplots(2, 3, figsize=(14, 8.4))
    x = L['xyz']
    s = ax[0, 0].scatter(x[:, 1], x[:, 2], c=L['intensity'], s=.5, cmap='viridis')
    ax[0, 0].set_title('LiDAR scan (front view), colour = intensity')
    ax[0, 0].set_aspect('equal')
    ax[0, 1].scatter(x[:, 0], x[:, 1], c=np.linalg.norm(x, axis=1), s=.5, cmap='plasma')
    ax[0, 1].set_title('LiDAR scan (top view), colour = range')
    ax[0, 1].set_aspect('equal')
    ax[0, 2].hist(np.linalg.norm(x, axis=1), bins=60, color=LIDAR)
    ax[0, 2].set_title('range histogram of one scan (m)')
    xf = F['xyz']
    ax[1, 0].scatter(xf[:, 1], xf[:, 2], c=rgb_of(F), s=.6)
    ax[1, 0].set_title('fused cloud, RGB colour')
    ax[1, 0].set_aspect('equal')
    th = F['thermal']
    ax[1, 1].scatter(xf[:, 1], xf[:, 2], c=(th if np.isfinite(th).any() else 'lightgrey'), s=.6, cmap='inferno')
    ax[1, 1].set_title('fused cloud, thermal value' + ('' if np.isfinite(th).any() else ' (no thermal frames)'))
    ax[1, 1].set_aspect('equal')
    xm = M['xyz']
    ax[1, 2].scatter(xm[:, 0], xm[:, 1], c=rgb_of(M), s=.2)
    ax[1, 2].set_title(f'merged cloud ({len(xm):,} points), top view')
    ax[1, 2].set_aspect('equal')
    for a in ax.flat:
        a.grid(False)
    save(fig, 'F14_pointcloud_gallery', 'Point cloud gallery', 'One real LiDAR scan, the same scan after RGB and thermal colouring, and the ICP-merged cloud. Use these as the visual results figure.')


# --------------------------------------------------------------------------------------------- 15
def fig_startup():
    su = pj('phase_startup') or {}
    fm = su.get('first_message_s_after_start') or {}
    if not fm:
        return
    rows = [('LiDAR scan', '/livox/lidar', LIDAR), ('LiDAR restamped', '/landfill/livox/points', LIDAR), ('RGB frame', '/landfill/rgb/image_raw', RGB), ('fused cloud', '/landfill/fusion/points_rgbt', FUSED), ('merged cloud', '/landfill/fusion/points_merged', MERGED)]
    fig, ax = plt.subplots(figsize=(10, 3.8))
    for i, (lab, tp, col) in enumerate(rows):
        if tp in fm:
            ax.barh(i, fm[tp], color=col, height=0.55)
            ax.text(fm[tp] + 0.2, i, f'{fm[tp]:.1f} s', va='center', fontsize=9)
    v = su.get('viewer_first_merged_data_s')
    if v:
        ax.axvline(v, color=VIEWER, lw=2)
        ax.text(v + 0.2, len(rows) - 0.45, f'first data in the viewer: {v:.1f} s', color=INK, fontsize=9)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([r[0] for r in rows])
    ax.invert_yaxis()
    ax.set_xlabel('seconds after the pipeline service is started (the OS itself boots in 41 s)')
    ax.set_title('Start-up: time to the first message of each stage')
    ax.grid(axis='y', visible=False)
    save(fig, 'F15_startup', 'Start-up timeline', 'Time from starting the pipeline service to the first message at each stage, measured with a subscriber waiting beforehand. The viewer shows data about 13 s after the service starts; a graceful restart takes the same time.')


# --------------------------------------------------------------------------------------------- 16
def fig_ntp_step():
    p = f'{DATA}/real_ntp_step_evidence.txt'
    if not os.path.exists(p):
        return
    L = open(p).read().splitlines()
    pubs = []
    for line in L:
        m = re.search(r'(\d\d):(\d\d):(\d\d\.\d+).*Published merged cloud: (\d+) pts \((\d+) scans', line)
        if m:
            pubs.append((int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3)), int(m.group(4)), int(m.group(5))))
    step = [line for line in L if 'Host clock stepped' in line]
    if not pubs or not step:
        return
    ms = re.search(r'(\d\d):(\d\d):(\d\d\.\d+)', step[0])
    t_step = int(ms.group(1)) * 3600 + int(ms.group(2)) * 60 + float(ms.group(3))
    mag = re.search(r'stepped by ([+-][\d.]+)s', step[0]).group(1)
    fig, ax = plt.subplots(figsize=(10, 3.8))
    ax.vlines([p[0] - t_step for p in pubs], 0, [p[1] / 1000 for p in pubs], color=MERGED, lw=3)
    ax.axvline(0, color=CRIT, lw=2)
    ax.text(0.2, ax.get_ylim()[1] * 0.92, f'NTP clock step {float(mag) / 3600:+.2f} h\n(restamp filter reset immediately)', color=INK, fontsize=9)
    ax.set_xlabel('seconds relative to the clock step')
    ax.set_ylabel('merged cloud size (thousand points)')
    ax.set_title('A real NTP step (+2 h) hit the running pipeline - merged clouds kept flowing')
    save(fig, 'F16_real_ntp_step', 'Real clock step', 'Each orange bar is one merged cloud published around the moment NTP stepped the Pi\'s clock forward by about two hours, 36 s after boot. The new step detector reset the LiDAR offset filter at once; with the previous code fusion would have produced nothing for about 20 s.', source='Source: pipeline log, 2026-11-12 20:23:53.')


# --------------------------------------------------------------------------------------------- 17
def fig_dashboard():
    P = load_csv(f'{DATA}/power_raw.csv')
    ip = pj('phase_idle_pipeline') or {}
    sen = pj('phase_sensor') or {}
    allt = {}
    for k in ('topics_group_lidar', 'topics_rgb', 'topics_fused_merged'):
        allt.update(ip.get(k) or {})
    wire = ((pj('phase_accuracy') or {}).get('wire') or {}).get('points_merged') or {}
    su = pj('phase_startup') or {}
    tiles = []
    if P:
        tiles.append((f"{P['est_in_a'].max():.2f} A", 'peak input current\n(5 A limit)', LIDAR))
        tiles.append((f"{P['watts'].mean():.1f} W", 'mean power\n(SoC rails)', MERGED))
        tiles.append((f"{P['temp_c'].max():.0f} C", 'peak SoC temperature', THERMAL))
    for tp, lab, col in (('/livox/lidar', 'LiDAR rate', LIDAR), ('/landfill/fusion/points_rgbt', 'fused rate', FUSED), ('/landfill/fusion/points_merged', 'merged rate', MERGED)):
        v = allt.get(tp)
        if v and 'hz' in v:
            tiles.append((f"{v['hz']:.1f} Hz", lab, col))
    pf = ((sen.get('lidar') or {}).get('plane_fit') or {}).get('rms_mm', {}).get('mean')
    if pf:
        tiles.append((f'{pf:.0f} mm', 'LiDAR plane-fit\nprecision (upper bound)', LIDAR))
    if wire:
        tiles.append((f"{wire['position_error_mm']['max']:.2f} mm", 'viewer wire-format\nmax error', VIEWER))
    if su.get('viewer_first_merged_data_s'):
        tiles.append((f"{su['viewer_first_merged_data_s']:.0f} s", 'start to first\nviewer data', NEUTRAL))
    tiles.append(('0', 'tracebacks in\nthe whole campaign', GOOD))
    tiles.append(('7/7', 'nodes recover from\nSIGKILL in 2 s', GOOD))
    n = len(tiles)
    cols = 4
    rows = int(np.ceil(n / cols))
    fig, ax = plt.subplots(figsize=(12, 2.4 * rows + 0.6))
    ax.set_xlim(0, cols)
    ax.set_ylim(rows, 0)
    ax.axis('off')
    ax.grid(False)
    ax.set_title('Key results at a glance', pad=10)
    for i, (val, lab, col) in enumerate(tiles):
        r, c = divmod(i, cols)
        ax.add_patch(FancyBboxPatch((c + .04, r + .06), .92, .88, boxstyle='round,pad=0.01,rounding_size=0.05', fc='#ffffff', ec=GRID))
        ax.add_patch(Rectangle((c + .04, r + .06), .05, .88, fc=col, ec='none'))
        ax.text(c + .5, r + .42, val, ha='center', va='center', fontsize=26, weight='bold', color=INK)
        ax.text(c + .5, r + .76, lab, ha='center', va='center', fontsize=9, color=INK2)
    save(fig, 'F17_key_results_dashboard', 'Key results dashboard', 'A summary panel of the headline numbers, each tile coloured by the entity it describes. Suitable as a results-overview infographic.')


# --------------------------------------------------------------------------------------------- 18
def fig_battery():
    pl = '/home/vish/power_logs/power_log.csv'
    if not os.path.exists(pl):
        return
    out = subprocess.run(['bash', '-c', f"grep -E '^2026-11-12 (1[5-9]|20):' {pl}"], capture_output=True, text=True).stdout.splitlines()
    rows = [r.split(',') for r in out if len(r.split(',')) > 5]
    if len(rows) < 100:
        return
    ts = np.array([time.mktime(time.strptime(r[0][:19], '%Y-%m-%d %H:%M:%S')) for r in rows])
    ext = np.array([float(r[1]) for r in rows])
    wat = np.array([float(r[3]) for r in rows])
    gap = np.where(np.diff(ts) > 120)[0]
    if not len(gap):
        return
    g = gap[-1]
    prev = gap[-2] + 1 if len(gap) > 1 else 0
    t, e, w = ts[prev:g + 1], ext[prev:g + 1], wat[prev:g + 1]
    x = (t - t[0]) / 3600
    fig, ax = plt.subplots(2, 1, figsize=(10, 4.6), sharex=True)
    ax[0].plot(x, e, lw=.4, color=RGB)
    ax[0].set_ylabel('EXT5V (V)')
    ax[0].set_title(f'Anker power bank run-out: {x[-1]:.2f} h of operation, then an abrupt cut-off with no voltage warning')
    ax[1].plot(x, w, lw=.4, color=MERGED)
    ax[1].set_ylabel('SoC power (W)')
    ax[1].set_xlabel('hours since the Pi booted on the bank')
    ax[0].annotate('power lost', xy=(x[-1], e[-1]), xytext=(x[-1] - 0.6, e.min() - 0.01), color=CRIT, fontsize=9, arrowprops=dict(arrowstyle='->', color=CRIT))
    save(fig, 'F18_battery_runout', 'Battery run-out', 'Supply voltage and SoC power for the whole session on the power bank. The voltage stays flat at about 5.0 V until the bank cuts out, so software cannot predict the shutdown from the supply voltage; plan runtime from the energy figure instead.')


# --------------------------------------------------------------------------------------------- 19
def fig_network():
    nt = pj('phase_net') or {}
    ht = pj('phase_http') or {}
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
    sig = nt.get('signal_dbm')
    items = []
    for k, lab in (('ping_gateway_200', 'gateway'), ('ping_1.1.1.1_50', '1.1.1.1'), ('ping_8.8.8.8_50', '8.8.8.8')):
        d = nt.get(k) or {}
        if 'rtt_avg' in d:
            items.append((lab, d['rtt_avg'], d['rtt_max']))
    for i, (lab, avg, mx) in enumerate(items):
        ax[0].barh(i, avg, color=LIDAR, height=0.5)
        ax[0].plot([mx, mx], [i - .25, i + .25], color=INK)
        ax[0].text(mx + 20, i, f'avg {avg:.0f}, max {mx:.0f} ms', va='center', fontsize=8)
    ax[0].set_yticks(range(len(items)))
    ax[0].set_yticklabels([i[0] for i in items])
    ax[0].invert_yaxis()
    ax[0].set_title('WiFi round-trip time (ms)')
    ax[0].grid(axis='y', visible=False)
    dl = nt.get('download') or {}
    ul = nt.get('upload') or {}
    vals = [(f'download\n{k.split("_")[0]}', d['mbit_s']) for k, d in dl.items()] + [(f'upload\n{k.split("_")[0]}', d['mbit_s']) for k, d in ul.items()]
    ax[1].bar(range(len(vals)), [v[1] for v in vals], color=VIEWER, width=.6)
    for i, v in enumerate(vals):
        ax[1].text(i, v[1] + 0.05, f'{v[1]:.1f}', ha='center', fontsize=9)
    ax[1].set_xticks(range(len(vals)))
    ax[1].set_xticklabels([v[0] for v in vals], fontsize=8)
    ax[1].set_title(f'Internet throughput (Mbit/s), signal {sig} dBm')
    xs, p95, mb = [], [], []
    for k, v in sorted(ht.items(), key=lambda kv: int(re.findall(r'\d+', kv[0])[0]) if kv[0].startswith('clients_') else 0):
        if k.startswith('clients_') and k.endswith('localhost') and 'paths' in v:
            xs.append(int(re.findall(r'\d+', k)[0]))
            p95.append(max(d['latency_ms']['p95'] for d in v['paths'].values()))
            mb.append(sum(d['wire_mbit_s_total'] for d in v['paths'].values()))
    if xs:
        ax[2].plot(xs, mb, 'o-', color=VIEWER, lw=2)
        ax[2].set_xscale('log', base=2)
        ax[2].set_xlabel('concurrent viewer clients')
        ax[2].set_ylabel('Mbit/s served by the Pi')
        ax[2].set_title(f'Viewer scaling (p95 latency stays {min(p95):.0f}-{max(p95):.0f} ms, 0 errors)')
    save(fig, 'F19_network_and_viewer_scaling', 'Network and viewer scaling', 'Left and middle: the WiFi link in the rig room (weak signal, long round-trip times, under 0.5 Mbit/s download). Right: the Pi itself serves 16 concurrent clients with flat latency, so the radio, not the Pi, limits what a remote viewer sees.')


# --------------------------------------------------------------------------------------------- 20
def fig_sensors():
    s = pj('phase_sensor') or {}
    L = s.get('lidar') or {}
    R = s.get('rgb') or {}
    ts = pj('phase_thermal_supplement') or {}
    T = ((ts.get('sensor') or {}).get('thermal')) or {}
    F = ((ts.get('sensor') or {}).get('fused')) or (s.get('fused') or {})
    fig, ax = plt.subplots(1, 4, figsize=(14, 3.6))
    if L.get('range_hist_m'):
        h = L['range_hist_m']
        ax[0].bar(range(len(h)), [v * 100 for v in h.values()], color=LIDAR)
        ax[0].set_xticks(range(len(h)))
        ax[0].set_xticklabels(list(h), fontsize=7)
        ax[0].set_title('LiDAR returns by range (%)')
        ax[0].set_xlabel('range bin (m)')
    if L.get('points_by_line'):
        pb = L['points_by_line']
        ax[1].bar(range(len(pb)), list(pb.values()), color=LIDAR)
        ax[1].set_title('points per scan line')
        ax[1].set_xticks(range(len(pb)))
        ax[1].set_xticklabels([f'line {k}' for k in pb], fontsize=7)
    names, vals, cols = [], [], []
    if F.get('valid_rgb_frac'):
        names += ['RGB', 'thermal', 'both']
        vals += [F['valid_rgb_frac']['mean'] * 100, F['valid_thermal_frac']['mean'] * 100, F['both_valid_frac']['mean'] * 100]
        cols += [RGB, THERMAL, FUSED]
        ax[2].bar(names, vals, color=cols)
        for i, v in enumerate(vals):
            ax[2].text(i, v + 1, f'{v:.0f}%', ha='center', fontsize=9)
        ax[2].set_ylim(0, 105)
        ax[2].set_title('LiDAR points with a colour sample (%)')
    rates = [('LiDAR', 10.0 if not L.get('rate_hz') else L['rate_hz'], LIDAR), ('RGB', R.get('rate_hz', 0), RGB), ('thermal', T.get('rate_hz', 0), THERMAL)]
    ax[3].bar([r[0] for r in rates], [r[1] for r in rates], color=[r[2] for r in rates])
    for i, r in enumerate(rates):
        ax[3].text(i, r[1] + 0.5, f'{r[1]:.1f} Hz', ha='center', fontsize=9)
    ax[3].set_title('sensor frame rates')
    for a in ax:
        a.grid(axis='x', visible=False)
    save(fig, 'F20_sensor_characterisation', 'Sensor characterisation', 'LiDAR range and line distribution, how many LiDAR points fall inside each camera image (field-of-view overlap, not colour accuracy) and the sensors\' delivered frame rates.')


def write_captions():
    lines = ['# Thesis figures: captions and notes\n', 'Every figure exists as a 200 dpi PNG and a vector PDF in this folder. Colour identifies the stream in every figure: LiDAR blue, RGB green, thermal red, fused violet, merged orange, viewer yellow. Source data is in `../data/`; regenerate with `python3 make_thesis_figures.py`.\n']
    for name, title, cap in CAPTIONS:
        lines.append(f'## {name} - {title}\n\n![{title}]({name}.png)\n\n**Suggested caption:** {cap}\n')
    open(f'{OUT}/CAPTIONS.md', 'w').write('\n'.join(lines))


if __name__ == '__main__':
    for fn in (fig_architecture, fig_processes, fig_latency, fig_wire, fig_power, fig_stress, fig_sweep, fig_rates, fig_robustness, fig_camera_history, fig_history, fig_gallery, fig_startup, fig_ntp_step, fig_dashboard, fig_battery, fig_network, fig_sensors):
        try:
            fn()
        except Exception as exc:
            print('FAILED', fn.__name__, repr(exc))
    write_captions()
    print(len(CAPTIONS), 'figures written to', OUT)
