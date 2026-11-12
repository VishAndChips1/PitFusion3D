#!/usr/bin/env python3
"""Day-2 (2026-11-13) thesis figures. Same conventions as the main figure set: one fixed colour per entity, direct labels, one y-axis per chart,
status colours always paired with a symbol. Output: figures/D*.png + .pdf and figures/CAPTIONS_DAY2.md"""
import csv
import json
import os
import time
import textwrap

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = f'{HERE}/figures'
os.makedirs(OUT, exist_ok=True)
INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
LIDAR, RGB, THERMAL, FUSED, MERGED, VIEWER, NEUTRAL = '#2a78d6', '#1baf7a', '#e34948', '#6250d6', '#eb6834', '#eda100', '#8a8984'
GOOD, WARN, CRIT = '#0ca30c', '#fab219', '#d03b3b'
plt.rcParams.update({'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF, 'font.size': 10, 'text.color': INK, 'axes.labelcolor': INK2,
                     'xtick.color': INK2, 'ytick.color': INK2, 'axes.edgecolor': GRID, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
                     'axes.spines.top': False, 'axes.spines.right': False, 'axes.titlesize': 11, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left',
                     'legend.frameon': False})
CAPS = []


def J(name):
    p = f'{HERE}/data/{name}.json'
    return json.load(open(p)) if os.path.exists(p) else None


def save(fig, name, title, caption, source='Source: Raspberry Pi 5 rig, 2026-11-13.'):
    try:
        fig.tight_layout(rect=(0, 0.025, 1, 1))
    except Exception:
        pass
    fig.text(0.01, 0.005, source, fontsize=7, color=INK2, va='bottom')
    fig.savefig(f'{OUT}/{name}.png', dpi=200)
    fig.savefig(f'{OUT}/{name}.pdf')
    plt.close(fig)
    CAPS.append((name, title, caption))
    print('figure', name)


def box(ax, x, y, w, h, text, colour, fs=8, tc='#ffffff', sub=None):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02,rounding_size=0.08', fc=colour, ec='none'))
    ax.text(x + w / 2, y + h / 2 + (0.1 if sub else 0), text, ha='center', va='center', fontsize=fs, color=tc, weight='bold')
    if sub:
        ax.text(x + w / 2, y + h / 2 - 0.17, sub, ha='center', va='center', fontsize=fs - 1.5, color=tc)


def d01_lidar_paths():
    sl = J('lidar_sudden_loss') or {}
    fig, ax = plt.subplots(figsize=(13, 5.2))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 5.2)
    ax.axis('off')
    ax.grid(False)
    ax.set_title('What happens to the LiDAR when the Pi powers down and up')
    rows = [(4.1, 'Pi shut down from the OS (poweroff / reboot)', GOOD, [('pipeline stops\n(driver releases LiDAR)', 0.2, 2.6), ('lidar-power.service\nStandby command', 3.0, 2.6), ('LiDAR in Standby\n0.9 s', 5.8, 2.2), ('Pi off', 8.2, 1.2)], 'handled in software (tested through systemd; real power-down order verified at the reboot test)'),
            (2.5, 'Pi powered up', GOOD, [('Pi boots\n(user session at 13.8 s)', 0.2, 2.6), ('wake service runs BEFORE\nthe pipeline: Standby -> Normal 0.9 s', 3.0, 3.4), ('pipeline driver connects,\nstreams', 6.6, 2.6), ('first merged cloud\n~30 s', 9.4, 2.4)], 'handled in software; adds ~5 s to boot (Standby) or ~15 s (Power-Saving)'),
            (0.9, 'Pi loses power suddenly (flat battery, cable pulled)', CRIT, [('host vanishes\n(no goodbye)', 0.2, 2.2), ('data still arriving\nfor %.1f s' % (sl.get('last_data_packet_at_s') or 3.2), 2.6, 2.6), ('LiDAR re-broadcasts every 1 s,\nwaits for a host', 5.4, 3.2), ('stays in NORMAL mode\n(idle, not scanning)', 8.8, 3.0)], 'cannot be done in software: nothing is left running to send the command. Hardware fix: switch the LiDAR supply from the Pi 5 V (relay/MOSFET)')]
    for y, title, col, boxes, note in rows:
        ax.text(0.1, y + 0.95, title, fontsize=9.5, weight='bold', color=INK)
        for t, x, w in boxes:
            box(ax, x, y - 0.1, w - 0.1, 0.85, t, col, 7.5)
        ax.text(0.1, y - 0.3, note, fontsize=7.5, color=INK2, style='italic')
    save(fig, 'D01_lidar_power_paths', 'LiDAR behaviour on power-down and power-up', 'The three ways the Pi can lose or gain power and what the LiDAR does in each. Green = handled automatically by the installed service. Red = physically impossible to handle from the Pi; measured on the real unit with the controlling process killed without warning.')


def d02_camera():
    ha = J('heat_analysis')
    if not ha:
        return
    eps = ha['episodes']
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.8), gridspec_kw={'width_ratios': [1.0, 1.1, 1.0], 'wspace': 0.55})
    xs = list(range(len(eps)))
    ax[0].bar(xs, [e['soc_before_C'] for e in eps], color=THERMAL, width=0.6, label='SoC temperature just before a stuck episode began')
    ax[0].axhline(ha['good_mean'], color=GOOD, lw=2)
    ax[0].text(-0.4, 78, f"green line = mean while streaming fine: {ha['good_mean']:.1f} C", color=INK, fontsize=8)
    ax[0].set_xticks(xs)
    ax[0].set_xticklabels([e['start'] for e in eps], rotation=60, ha='right', fontsize=7)
    ax[0].set_ylabel('SoC temperature (C)')
    ax[0].set_ylim(0, 85)
    ax[0].set_title('Stuck episodes vs temperature')
    ax[0].grid(axis='x', visible=False)
    exp = [('port power-off/on\n(8 cycles, quiet 30 s)', 0, 8), ('watchdog ladder:\nport cycle / deauth / usbreset', 0, 3), ('camera self-reset +\n2 min hands-off', 0, 1), ('full power loss\n(cold starts, 2 days)', 2, 4)]
    for i, (lab, ok, n) in enumerate(exp):
        ax[1].barh(i, n, color=GRID, height=0.55)
        ax[1].barh(i, ok, color=GOOD, height=0.55)
        ax[1].text(n + 0.15, i, f'{ok} of {n} cleared it', va='center', fontsize=9, color=INK)
    ax[1].set_yticks(range(len(exp)))
    ax[1].set_yticklabels([e[0] for e in exp], fontsize=8)
    ax[1].invert_yaxis()
    ax[1].set_xlim(0, 11)
    ax[1].set_xlabel('attempts')
    ax[1].set_title('What clears the stuck state')
    ax[1].grid(axis='y', visible=False)
    ax[2].axis('off')
    ax[2].grid(False)
    ax[2].set_title('Conclusions')
    for i, t in enumerate(['Heat does not explain it: stuck episodes began at 37-48 C as often as at 70 C; the SoC averaged 61.5 C while streaming fine and 64.4 C while stuck (ranges overlap).',
                           'Port-level USB switching does not cut power on the Pi 5: all 4 ports share one power group.',
                           'Only a real loss of power has cleared it, and only about half the time.',
                           'Fix options: cut power to the whole USB group from software (needs root), a switchable USB hub, or replug / replace the camera.']):
        ax[2].text(0.0, 0.93 - i * 0.24, '• ' + t, fontsize=8, va='top', wrap=True, color=INK2, transform=ax[2].transAxes)
    save(fig, 'D02_thermal_camera_investigation', 'Thermal camera: heat theory and recovery experiments', 'Left: SoC temperature at the start of each of the 12 stuck episodes against the average while the camera streams fine. Middle: every recovery method tried and how many attempts cleared the stuck state. Right: what follows from the evidence.')


def d03_icp():
    a = J('icp_accuracy')
    p = J('icp_param_study')
    if not a:
        return
    sc = a['scene']
    fig, ax = plt.subplots(1, 3, figsize=(14.5, 4.2))
    mot = [f"{r['translation_cm']:.0f} cm\n{r['rotation_deg']:g} deg" for r in sc]
    x = np.arange(len(sc))
    ax[0].bar(x, [r['success_pct'] for r in sc], color=LIDAR, width=0.6)
    for xi, r in zip(x, sc):
        ax[0].text(xi, r['success_pct'] + 2, f"{r['success_pct']:.0f}%", ha='center', fontsize=8)
    ax[0].set_xticks(x)
    ax[0].set_xticklabels(mot, fontsize=7)
    ax[0].set_ylim(0, 118)
    ax[0].set_title('ICP recovers the motion (success %)')
    ax[0].set_xlabel('true motion between two scans')
    ax[0].grid(axis='x', visible=False)
    ax[0].axvline(3.5, color=NEUTRAL, ls=':')
    ax[0].text(3.6, 104, 'rig moves at most 3.5 cm between scans', fontsize=7, color=INK2)
    ok = [r for r in sc if r['t_err_mm_median'] is not None]
    ax[1].plot([sc.index(r) for r in ok], [r['t_err_mm_median'] for r in ok], 'o-', color=MERGED, lw=2, label='position error (mm, median)')
    ax[1].set_xticks(x)
    ax[1].set_xticklabels(mot, fontsize=7)
    ax[1].set_ylabel('mm')
    ax[1].set_title('Residual error (previous 0.2 m voxel setting)')
    ax[1].set_xlabel('true motion between two scans')
    ax[1].legend(fontsize=8)
    if p:
        base = [r for r in p if r['voxel_m'] == 0.2 and r['max_src'] == 1500 and r['max_tgt'] == 10000 and r['iterations'] == 10][0]
        new = [r for r in p if r['voxel_m'] == 0.1 and r['max_src'] == 1500 and r['max_tgt'] == 10000 and r['iterations'] == 10][0]
        cols = {0.3: NEUTRAL, 0.2: LIDAR, 0.1: GOOD, 0.05: MERGED}
        for r in p:
            ax[2].scatter(r['total_ms_median'], r['t_err_mm_median'], color=cols[r['voxel_m']], s=40)
        for v, c in cols.items():
            ax[2].scatter([], [], color=c, label=f'voxel {v} m')
        ax[2].annotate('was', xy=(base['total_ms_median'], base['t_err_mm_median']), xytext=(base['total_ms_median'] + 15, base['t_err_mm_median'] + 8), arrowprops=dict(arrowstyle='->'), fontsize=8)
        ax[2].annotate('now (0.1 m)', xy=(new['total_ms_median'], new['t_err_mm_median']), xytext=(new['total_ms_median'] + 25, new['t_err_mm_median'] + 14), arrowprops=dict(arrowstyle='->'), fontsize=8)
        ax[2].set_xlabel('ICP cost per scan (ms)')
        ax[2].set_ylabel('position error (mm)')
        ax[2].set_title('Accuracy vs cost: 24 settings tried')
        ax[2].legend(fontsize=7)
    save(fig, 'D03_icp_accuracy', 'Registration (ICP) accuracy with known ground truth', 'A real merged cloud was moved by known amounts and the production ICP had to recover the motion. Left: success rate. Middle: residual error with the previous 0.2 m voxel (the 0.1 m now in production roughly halves it). Right: the accuracy/cost trade-off of 24 parameter combinations; the 0.1 m voxel now in production halves the error for about 11 ms more per scan.')


def d04_calibration():
    est = [('wide search (4 frames)', [3.5, 3.5, 3.5, 3.5]), ('fine search (5 frames)', [3.5, 3.4, 3.5, 3.5, 3.4]), ('survey (2 frames)', [2.5, 2.5]), ('validation (5 frames)', [-2.25, -2.0, -2.0, -2.25, -2.0])]
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.2), gridspec_kw={'width_ratios': [1.1, 1]})
    for i, (lab, v) in enumerate(est):
        ax[0].scatter(v, [i] * len(v), s=60, color=LIDAR, alpha=.7)
    ax[0].axvline(0, color=NEUTRAL, ls=':')
    ax[0].set_yticks(range(len(est)))
    ax[0].set_yticklabels([e[0] for e in est], fontsize=8)
    ax[0].invert_yaxis()
    ax[0].set_xlabel('best yaw offset found (degrees)')
    ax[0].set_title('Calibration check: the estimate was NOT stable')
    ax[0].grid(axis='y', visible=False)
    ax[1].axis('off')
    ax[1].grid(False)
    ax[1].set_title('Why, and what to do')
    for i, t in enumerate(['Within each run the answer was consistent (3.4-3.5 deg) - then a later run gave 2.5 deg and another -2 deg.',
                           'The scene explains it: sheer curtains. The LiDAR hits the curtain; the camera also sees the trees behind it, and the daylight changed. Depth edges and image edges are largely unrelated here.',
                           'A paired test on identical frames showed the first correction was not better than none (0.045 vs 0.065), so it was NOT applied. The optional correction support stays in the code, disabled.',
                           'To measure the RGB-LiDAR alignment reliably: a checkerboard / corner target, or an edge-rich static scene (shelves, doorframe, boxes).']):
        ax[1].text(0.0, 0.92 - i * 0.235, '• ' + t, fontsize=8.3, va='top', color=INK2, transform=ax[1].transAxes, wrap=True)
    save(fig, 'D04_calibration_check', 'RGB-LiDAR calibration check: inconclusive on this scene', 'Estimated LiDAR-to-camera yaw error from edge alignment across four runs. The estimate jumped between runs because the scene (sheer curtains with daylight behind) is unsuitable; an honest negative result that defines what a proper calibration needs.')


def d05_stability():
    p = f'{HERE}/data/lidar_stability.csv'
    s = J('lidar_stability_summary')
    if not os.path.exists(p) or not s:
        return
    r = list(csv.DictReader(open(p)))
    t = np.array([float(x['t_s']) for x in r]) / 60
    cr = np.array([float(x['centre_range_m']) for x in r])
    soc = np.array([float(x['soc_c']) for x in r])
    rms = np.array([float(x['plane_rms_mm']) for x in r])
    good = np.abs(cr - np.median(cr)) < 0.20
    fig, ax = plt.subplots(1, 3, figsize=(14.5, 4))
    ax[0].plot(t[good], (cr[good] - np.median(cr[good])) * 1000, 'o-', color=LIDAR, ms=3, lw=1)
    ax[0].axhline(0, color=NEUTRAL, lw=1)
    ax[0].set_xlabel('minutes after the test started')
    ax[0].set_ylabel('range change (mm)')
    ax[0].set_title(f"Fixed-window range: {s['robust_std_mm']:.0f} mm spread, {s['trend_mm_per_h']:+.0f} mm/h trend")
    ax[1].plot(t, soc, color=THERMAL, lw=2)
    ax[1].set_xlabel('minutes')
    ax[1].set_ylabel('Pi SoC temperature (C)')
    ax[1].set_title('Pi temperature during the same period')
    ax[2].plot(t, rms, 'o-', color=LIDAR, ms=3, lw=1)
    ax[2].set_ylim(0, 25)
    ax[2].set_xlabel('minutes')
    ax[2].set_ylabel('plane-fit RMS (mm)')
    ax[2].set_title('LiDAR noise: steady at ~16.6 mm')
    save(fig, 'D05_lidar_stability', 'LiDAR range stability over 25 minutes', 'A fixed 6x6 degree window straight ahead and the dominant plane were tracked every 30 s. Noise (plane RMS) did not change as the sensor warmed. The measured range drifted about 3-4 cm shorter over 25 minutes (consistent with warm-up drift, though scene motion cannot be excluded). One sample with a person in the window was excluded.')


def d06_planning():
    n = J('stakeholder_numbers')
    if not n:
        return
    fig, ax = plt.subplots(1, 2, figsize=(14, 4.6), gridspec_kw={'width_ratios': [1.25, 1]})
    vol = n['data_volume']
    names = [('LiDAR scans (raw)', LIDAR), ('RGB frames (raw rgb8)', RGB), ('thermal frames (raw mono16)', THERMAL), ('fused cloud (XYZ+I+RGB+T)', FUSED), ('merged cloud (full resolution)', MERGED), ('merged cloud, quantised 11 B/pt + gzip (viewer format, full res)', VIEWER)]
    for i, (k, c) in enumerate(names):
        v = vol[k]['gb_per_hour']
        ax[0].barh(i, v, color=c, height=0.6)
        ax[0].text(v * 1.05, i, f"{v:.1f} GB/h  ({n['sd']['free_gb'] / v:.1f} h of free SD)", va='center', fontsize=8.5)
    ax[0].set_xscale('log')
    ax[0].set_xlim(1, 600)
    ax[0].set_yticks(range(len(names)))
    ax[0].set_yticklabels([k.replace(' (viewer format, full res)', '').replace('merged cloud, quantised 11 B/pt + gzip', 'merged, quantised + gzip') for k, _ in names], fontsize=8)
    ax[0].invert_yaxis()
    ax[0].set_title('If you record it: data volume (SD card: 70 GB free, 53 MB/s write)')
    ax[0].grid(axis='y', visible=False)
    ax[0].text(1.1, 5.75, '', fontsize=8)
    ax[0].set_xlabel('gigabytes per hour (log scale).  Recording all three raw sensors needs 52 MB/s: at the card\'s 53 MB/s write limit.')
    caps = list(n['runtime_h'])
    w = 0.38
    xs = np.arange(len(caps))
    asleep, awake = 'Pi + sensors, LiDAR asleep', 'Pi + sensors + LiDAR scanning'
    ax[1].bar(xs - w / 2, [n['runtime_h'][c][asleep]['conservative'] for c in caps], w, color=NEUTRAL, label='Pi + sensors, LiDAR asleep')
    ax[1].bar(xs + w / 2, [n['runtime_h'][c][awake]['conservative'] for c in caps], w, color=LIDAR, label='Pi + sensors + LiDAR scanning')
    for x, c in zip(xs, caps):
        ax[1].text(x - w / 2, n['runtime_h'][c][asleep]['conservative'] + 1, f"{n['runtime_h'][c][asleep]['conservative']:.0f} h", ha='center', fontsize=8)
        ax[1].text(x + w / 2, n['runtime_h'][c][awake]['conservative'] + 1, f"{n['runtime_h'][c][awake]['conservative']:.0f} h", ha='center', fontsize=8)
    ax[1].set_xticks(xs)
    ax[1].set_xticklabels(caps)
    ax[1].set_ylabel('runtime (hours, conservative)')
    ax[1].set_title('Battery runtime by energy capacity')
    ax[1].legend(fontsize=8)
    ax[1].grid(axis='x', visible=False)
    save(fig, 'D06_data_and_battery_planning', 'Data volume and battery runtime', 'Left: gigabytes per hour of each stream, and how long the free SD space lasts. Right: runtime for typical battery capacities, using the measured Pi power (conservative bound 7.3 W) plus the manufacturer\'s typical 9 W for the LiDAR (not measured here).')


def d07_freshness():
    f = J('freshness_headers')
    if not f:
        return
    fig, ax = plt.subplots(figsize=(10, 3.6))
    rows = [('Live view (single scan, ~10 Hz)', f['live']['age_at_client_s'], 0.0, FUSED), ('Merged view (ICP-merged, ~1.5 Hz)', f['merged']['age_at_client_s'], 0.3, MERGED)]
    for i, (lab, a, extra, col) in enumerate(rows):
        ax.barh(i, a['p50'] * 1000, color=col, height=0.5)
        ax.plot([a['p95'] * 1000, a['p95'] * 1000], [i - 0.28, i + 0.28], color=INK, lw=1.6)
        if extra:
            ax.barh(i, extra * 1000, left=a['p50'] * 1000, color=col, alpha=0.35, height=0.5)
        ax.text(a['p95'] * 1000 + extra * 1000 + 25, i, f"newest data {a['p50'] * 1000:.0f} ms old (p95 {a['p95'] * 1000:.0f} ms)" + (f"\ncontent averages ~{(a['p50'] + extra) * 1000:.0f} ms" if extra else ''), va='center', fontsize=8.5)
    ax.set_yticks(range(2))
    ax.set_yticklabels([r[0] for r in rows])
    ax.invert_yaxis()
    ax.set_xlim(0, 1800)
    ax.set_xlabel('age of the data when it reaches the viewer (ms)  |  pale bar = extra age of the merged window\'s content')
    ax.set_title('How fresh is what the viewer shows?')
    ax.grid(axis='y', visible=False)
    save(fig, 'D07_data_freshness', 'End-to-end data freshness', 'Age of the newest data in each payload when the client receives it, measured with the streamer\'s own timestamp header (X-Cloud-Stamp / X-Server-Time). The live view is about 0.08 s old; the merged view is about 0.47 s old and summarises a 0.6 s window, so its content averages about 0.8 s.')


def d08_heat():
    p = f'{HERE}/data/heat_endurance.csv'
    if not os.path.exists(p):
        return
    r = list(csv.DictReader(open(p)))
    if len(r) < 50:
        return
    t = np.array([float(x['t_s']) for x in r]) / 60
    f = lambda k: np.array([float(x[k]) for x in r])
    phase = [x['phase'] for x in r]
    fig, ax = plt.subplots(4, 1, figsize=(11, 8.6), sharex=True)
    ax[0].plot(t, f('soc_c'), color=THERMAL, lw=1.6)
    ax[0].axhline(80, color=CRIT, ls=':')
    ax[0].text(t[0], 80.5, 'firmware throttling begins around 80 C', fontsize=7.5, color=CRIT)
    ax[0].set_ylabel('SoC (C)')
    ax[0].set_title('Sustained heat test: 4 busy cores for 25 minutes with the pipeline running')
    ax[1].plot(t, f('fan_rpm'), color=NEUTRAL, lw=1.6)
    ax[1].set_ylabel('fan (RPM)')
    ax[2].plot(t, f('arm_mhz'), color=LIDAR, lw=1.6)
    ax[2].set_ylabel('CPU clock (MHz)')
    ax[3].plot(t, f('est_in_a'), color=MERGED, lw=1, label='estimated input')
    ax[3].plot(t, f('est_cons_a'), color=VIEWER, lw=1, label='conservative')
    ax[3].axhline(5, color=CRIT)
    ax[3].set_ylim(0, 5.4)
    ax[3].set_ylabel('current (A)')
    ax[3].set_xlabel('minutes')
    ax[3].legend(fontsize=7, loc='center right')
    edges = [i for i in range(1, len(phase)) if phase[i] != phase[i - 1]]
    for e in edges:
        for a in ax:
            a.axvline(t[e], color=GRID, lw=1.2)
    for i, (s, e) in enumerate(zip([0] + edges, edges + [len(phase) - 1])):
        ax[0].text((t[s] + t[e]) / 2, ax[0].get_ylim()[0] + 1, phase[s], ha='center', fontsize=8, color=INK2)
    save(fig, 'D08_heat_endurance', 'Sustained-heat test', 'SoC temperature, fan speed, CPU clock and input current through a baseline, 25 minutes of all four cores at 100 % (with the pipeline running) and a cool-down. Shows the thermal headroom of the system in this room and how the fan and clock respond.')


def d09_boot():
    ev = [('kernel start', 0.0), ('pipeline service starts', 13.8), ('Ethernet (LiDAR) link up', 14.4), ('restamp node ready', 19.0), ('fusion node ready', 21.8), ('first merged cloud', 25.3), ('RGB / thermal node open', 25.8)]
    fig, ax = plt.subplots(figsize=(11, 3.6))
    cols = [NEUTRAL, VIEWER, LIDAR, LIDAR, FUSED, MERGED, RGB]
    for i, ((lab, tt), c) in enumerate(zip(ev, cols)):
        ax.barh(i, tt, color=c, height=0.55)
        ax.text(tt + 0.4, i, f'{tt:.1f} s', va='center', fontsize=9)
    ax.set_yticks(range(len(ev)))
    ax.set_yticklabels([e[0] for e in ev])
    ax.invert_yaxis()
    ax.set_xlabel('seconds after power-on (cold boot, 2026-11-13 07:59)')
    ax.set_title('Cold boot: data is flowing 25 s after power-on (the desktop takes 63 s - the pipeline does not wait for it)')
    ax.grid(axis='y', visible=False)
    save(fig, 'D09_cold_boot_timeline', 'Cold-boot timeline', 'Time from power-on to each pipeline stage, from the kernel journal\'s monotonic clock (immune to the Pi\'s stale clock). The installed LiDAR wake step runs before the pipeline and adds about 5 s from Standby.')


def d10_scorecard():
    items = [('Real-time performance', 'Fused 10 Hz, merged 1.5 Hz, 0.08-0.5 s data age; no backlog', GOOD, '✓'),
             ('Power and current', 'Peak 2.0 A of 5 A; 4.1 W mean; 0 throttling', GOOD, '✓'),
             ('Software robustness', '7/7 nodes recover in 2 s; 0 tracebacks; real NTP step handled', GOOD, '✓'),
             ('Restart / boot', '25 s to data; graceful restarts 20+ with no failure', GOOD, '✓'),
             ('LiDAR power management', 'Standby/Normal on OS shutdown/boot; sudden loss needs hardware', WARN, '!'),
             ('Registration accuracy', '18 mm after tuning (was 32 mm); robust to 10 cm motion', GOOD, '✓'),
             ('Colour/thermal alignment', 'Calibration approximate and unverified (needs a target)', WARN, '!'),
             ('Thermal camera', 'Stuck state needs power loss to clear; not heat related', CRIT, '✗'),
             ('Remote viewing (WiFi)', 'Weak link in rig room (-67 dBm, <0.5 Mbit/s)', CRIT, '✗'),
             ('Recording to SD', 'Fused/merged OK; all raw sensors at the card limit', WARN, '!')]
    fig, ax = plt.subplots(figsize=(11.5, 5.6))
    ax.set_xlim(0, 11.5)
    ax.set_ylim(len(items), 0)
    ax.axis('off')
    ax.grid(False)
    ax.set_title('Project scorecard (2026-11-13)')
    for i, (a, b, c, s) in enumerate(items):
        ax.add_patch(Rectangle((0.0, i + 0.08), 0.55, 0.84, fc=c, ec='none'))
        ax.text(0.275, i + 0.5, s, ha='center', va='center', color='#ffffff' if c != WARN else INK, fontsize=14, weight='bold')
        ax.text(0.8, i + 0.35, a, fontsize=10, weight='bold', color=INK, va='center')
        ax.text(0.8, i + 0.7, b, fontsize=8.5, color=INK2, va='center')
    save(fig, 'D10_scorecard', 'Project scorecard', 'One-page status by area. Green = meets the goal, amber = works with a stated limitation, red = needs action. Symbols are repeated beside the colours so the chart reads in greyscale.')


def write_caps():
    L = ['# Day-2 figures: captions\n']
    for n, t, c in CAPS:
        L.append(f'## {n} - {t}\n\n![{t}]({n}.png)\n\n**Suggested caption:** {c}\n')
    open(f'{OUT}/CAPTIONS_DAY2.md', 'w').write('\n'.join(L))


if __name__ == '__main__':
    for fn in (d01_lidar_paths, d02_camera, d03_icp, d04_calibration, d05_stability, d06_planning, d07_freshness, d08_heat, d09_boot, d10_scorecard):
        try:
            fn()
        except Exception as e:
            print('FAILED', fn.__name__, repr(e))
    write_caps()
    print(len(CAPS), 'figures')
