"""Run 1 reports: figures, tables, image captions, data dictionary, RESULTS_RUN1.md, thesis_numbers.json, PAPER_MAP.md and results_bundle.zip.
Works only from the files in the run directory, so it can be re-run at any time (auto_report.py) and produces whatever the data allows."""
import csv
import glob
import json
import time
import zipfile
from pathlib import Path

import numpy as np

import run1_geo as G
import TestWithLidV1 as T

C = G.COLOURS
PHASE_LABEL = {'inventory': 'P1 inventory', 'scene': 'P2 scene', 'rates': 'P3 rates', 'combinations': 'P4 combinations', 'range_accuracy': 'P5 range accuracy', 'cpu_mem': 'P6 CPU/memory',
               'stage_timing': 'P7 stage timing', 'freshness': 'P8 freshness', 'alignment': 'P9 alignment', 'repo_extract': 'P10 repo extract'}


# ------------------------------------------------------------------ small loaders
def rows_of(path):
    try:
        with open(path) as fh:
            return list(csv.DictReader(fh))
    except Exception:
        return []


def col(rows, key):
    out = []
    for r in rows:
        try:
            out.append(float(r[key]))
        except (ValueError, KeyError, TypeError):
            out.append(np.nan)
    return np.array(out)


def topic(run, key):
    p = Path(run) / '02_rates' / f'topic_{key}.csv'
    if not p.exists():
        return np.zeros((0, 4))
    try:
        a = np.loadtxt(p, delimiter=',', skiprows=1, ndmin=2)
        return a if a.size else np.zeros((0, 4))
    except Exception:
        return np.zeros((0, 4))


def jload(path, default=None):
    try:
        return json.load(open(path))
    except Exception:
        return default


def write_fig_csv(path, header, rows):
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)


def md_table(header, rows):
    out = ['| ' + ' | '.join(str(h) for h in header) + ' |', '|' + '---|' * len(header)]
    for r in rows:
        out.append('| ' + ' | '.join('' if v is None else str(v) for v in r) + ' |')
    return '\n'.join(out) + '\n'


class Report:
    def __init__(self, run):
        self.run = Path(run)
        self.fig = self.run / 'figures'
        self.tab = self.run / 'tables'
        self.fig.mkdir(exist_ok=True)
        self.tab.mkdir(exist_ok=True)
        self.captions_fig, self.captions_tab, self.made = [], [], []
        self.phases = jload(self.run / 'phase_status.json', [])
        self.t0 = None
        sysr = rows_of(self.run / '00_meta' / 'system_1hz.csv')
        self.sys = sysr
        if sysr:
            self.t0 = float(sysr[0]['t'])

    # ---- tables
    def table(self, name, header, rows, caption):
        with open(self.tab / f'{name}.csv', 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(header)
            w.writerows(rows)
        (self.tab / f'{name}.md').write_text(f'**{caption}**\n\n' + md_table(header, rows))
        self.captions_tab.append(f'- `{name}`: {caption}')

    # ---- figures
    def save(self, plt, fig, name, header, rows, caption):
        fig.tight_layout()
        fig.savefig(self.fig / f'{name}.png', dpi=300)
        plt.close(fig)
        write_fig_csv(self.fig / f'{name}.csv', header, rows)
        self.captions_fig.append(f'- `{name}.png` (data: `{name}.csv`): {caption}')
        self.made.append(name)

    def bands(self, ax):
        if self.t0 is None:
            return
        for i, ph in enumerate(self.phases):
            ax.axvspan((ph['start'] - self.t0) / 60, (ph['end'] - self.t0) / 60, color=C[3], alpha=0.35 if i % 2 == 0 else 0.15, lw=0)
            ax.text((ph['start'] - self.t0) / 60, 1.01, ph['phase'], transform=ax.get_xaxis_transform(), fontsize=5, rotation=45, ha='left', va='bottom')


def build_figures(R):
    plt = G._mpl()
    run, t0 = R.run, R.t0
    W = 6.4
    # F01 rates over time
    try:
        if t0 is None:
            raise ValueError('no system log')
        fig, ax = plt.subplots(figsize=(W, 3.2))
        data = {}
        end = max(float(R.sys[-1]['t']), t0 + 60)
        edges = np.arange(t0, end + 10, 10)
        for key, colr in (('lidar_raw', C[0]), ('rgb', C[1]), ('fused', C[2]), ('merged', '#555555')):
            a = topic(run, key)
            if len(a):
                h = np.histogram(a[:, 0], edges)[0] / 10.0
                data[key] = h
                ax.plot((edges[:-1] - t0) / 60, h, lw=0.8, color=colr, label=key)
        R.bands(ax)
        ax.set_xlabel('minutes from start')
        ax.set_ylabel('messages per second (10 s bins)')
        ax.legend(fontsize=6, loc='lower right')
        rows = [[f'{(edges[i] - t0) / 60:.2f}'] + [f'{data[k][i]:.2f}' if k in data else '' for k in ('lidar_raw', 'rgb', 'fused', 'merged')] for i in range(len(edges) - 1)]
        R.save(plt, fig, 'F01_rates_over_time', ['minute', 'lidar_raw_hz', 'rgb_hz', 'fused_hz', 'merged_hz'], rows, 'Topic rates over the whole run (10 s bins) with phase bands.')
    except Exception as exc:
        T.log(f'figure F01: {exc!r}')
    # F02 interval histograms for the rates window
    try:
        w = jload(run / 'state.json', {}).get('windows', {}).get('rates')
        fig, axs = plt.subplots(1, 2, figsize=(W, 2.6))
        rows = []
        for ax, key, colr in ((axs[0], 'fused', C[0]), (axs[1], 'merged', C[1])):
            a = topic(run, key)
            if w:
                a = a[(a[:, 0] >= w[0]) & (a[:, 0] <= w[1])]
            iv = np.diff(a[:, 0]) * 1000 if len(a) > 2 else np.array([])
            if len(iv):
                h, e = np.histogram(iv, bins=40, range=(0, np.percentile(iv, 99.5) * 1.1))
                ax.bar(e[:-1], h, width=np.diff(e), color=colr, align='edge')
                rows += [[key, f'{e[i]:.1f}', int(h[i])] for i in range(len(h))]
            ax.set_xlabel(f'{key} message interval (ms)')
            ax.set_ylabel('count')
        R.save(plt, fig, 'F02_interval_histograms', ['topic', 'interval_bin_start_ms', 'count'], rows, 'Distribution of the interval between fused and merged messages in the rates window (jitter).')
    except Exception as exc:
        T.log(f'figure F02: {exc!r}')
    sysr = R.sys
    if sysr and t0 is not None:
        t = (col(sysr, 't') - t0) / 60
        try:
            fig, ax = plt.subplots(figsize=(W, 3.2))
            names = [n for n in T.PIPE_PROCS if not np.all(np.isnan(col(sysr, f'{n}_cpu')))]
            stack = [np.nan_to_num(col(sysr, f'{n}_cpu')) for n in names]
            pal = [C[0], C[1], C[2], C[3], '#6b6b6b', '#8c6d46', '#3d7a63', '#a35d8a']
            ax.stackplot(t, stack, labels=[n.replace('_node', '').replace('livox_ros2_', 'livox_') for n in names], colors=pal[:len(names)], lw=0)
            R.bands(ax)
            ax.set_xlabel('minutes from start')
            ax.set_ylabel('CPU (% of one core)')
            ax.legend(fontsize=5, ncol=4, loc='upper right')
            R.save(plt, fig, 'F03_cpu_per_process', ['minute'] + names, [[f'{t[i]:.3f}'] + [f'{stack[j][i]:.1f}' for j in range(len(names))] for i in range(0, len(t), 5)], 'CPU per pipeline process, stacked (every 5th second).')
        except Exception as exc:
            T.log(f'figure F03: {exc!r}')
        try:
            fig, ax = plt.subplots(figsize=(W, 2.8))
            for i in range(4):
                ax.plot(t, col(sysr, f'cpu{i}'), lw=0.5, color=[C[0], C[1], C[2], '#555555'][i], label=f'core {i}')
            R.bands(ax)
            ax.set_xlabel('minutes from start')
            ax.set_ylabel('load (%)')
            ax.legend(fontsize=6, ncol=4)
            R.save(plt, fig, 'F04_cpu_per_core', ['minute', 'core0', 'core1', 'core2', 'core3'], [[f'{t[k]:.3f}'] + [f'{col(sysr, f"cpu{i}")[k]:.1f}' for i in range(4)] for k in range(0, len(t), 5)], 'Load on each of the four cores (every 5th second).')
        except Exception as exc:
            T.log(f'figure F04: {exc!r}')
        try:
            fig, (a1, a2) = plt.subplots(2, 1, figsize=(W, 4.2), sharex=True)
            a1.plot(t, col(sysr, 'pipeline_rss_mb'), color=C[0], lw=0.8)
            a1.set_ylabel('pipeline RSS (MB)')
            a2.plot(t, col(sysr, 'busiest_thread_pct'), color=C[1], lw=0.5)
            a2.set_ylabel('busiest thread (% of a core)')
            a2.set_xlabel('minutes from start')
            R.bands(a1)
            R.save(plt, fig, 'F05_memory_and_busiest_thread', ['minute', 'pipeline_rss_mb', 'busiest_thread_pct'], [[f'{t[k]:.3f}', f'{col(sysr, "pipeline_rss_mb")[k]:.0f}', f'{col(sysr, "busiest_thread_pct")[k]:.1f}'] for k in range(0, len(t), 5)],
                   'Pipeline resident memory and the busiest single thread over the run.')
        except Exception as exc:
            T.log(f'figure F05: {exc!r}')
        try:
            fig, (a1, a2) = plt.subplots(2, 1, figsize=(W, 4.2), sharex=True)
            a1.plot(t, col(sysr, 'soc_c'), color=C[1], lw=0.8)
            a1.set_ylabel('SoC temperature (C)')
            a2.plot(t, col(sysr, 'arm_mhz'), color=C[0], lw=0.8)
            a2.set_ylabel('ARM clock (MHz)')
            a2.set_xlabel('minutes from start')
            R.bands(a1)
            R.save(plt, fig, 'F06_temperature_clock', ['minute', 'soc_c', 'arm_mhz'], [[f'{t[k]:.3f}', f'{col(sysr, "soc_c")[k]:.1f}', f'{col(sysr, "arm_mhz")[k]:.0f}'] for k in range(0, len(t), 5)], 'SoC temperature and ARM clock over the run.')
        except Exception as exc:
            T.log(f'figure F06: {exc!r}')
    try:
        p = run / '09_power' / 'power_raw.csv'
        pa = np.loadtxt(p, delimiter=',', skiprows=1, ndmin=2)[::15]
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(W, 4.2), sharex=True)
        tt = (pa[:, 0] - (t0 if t0 else pa[0, 0])) / 60
        a1.plot(tt, pa[:, 1], color=C[0], lw=0.5)
        a1.axhline(4.75, color=C[1], ls='--', lw=0.8)
        a1.set_ylabel('EXT5V (V)')
        a2.plot(tt, pa[:, 2], color=C[0], lw=0.5, label='estimated input')
        a2.plot(tt, pa[:, 3], color=C[1], lw=0.5, label='conservative')
        a2.axhline(5.0, color='#888888', ls='--', lw=0.8)
        a2.set_ylabel('current (A, estimated)')
        a2.set_xlabel('minutes from start')
        a2.legend(fontsize=6)
        R.bands(a1)
        R.save(plt, fig, 'F07_power', ['minute', 'ext5v_v', 'est_input_a', 'est_input_cons_a'], [[f'{tt[k]:.3f}', f'{pa[k, 1]:.3f}', f'{pa[k, 2]:.3f}', f'{pa[k, 3]:.3f}'] for k in range(len(tt))],
               'EXT5V and estimated input current (PMIC rails, no input shunt) over the run; dashed lines: 4.75 V gate and 5 A limit.')
    except Exception as exc:
        T.log(f'figure F07: {exc!r}')
    try:
        rows = rows_of(run / '15_fusion_info' / 'combinations' / 'combinations.csv')
        rows = [r for r in rows if r['configuration'] in ('lidar_only', 'lidar_rgb', 'deployed_thermal_not_connected')]
        names = ['LiDAR only', 'LiDAR + RGB', 'deployed (thermal absent)']
        lab = {'lidar_only': 0, 'lidar_rgb': 1, 'deployed_thermal_not_connected': 2}
        fig, axs = plt.subplots(1, 4, figsize=(W, 2.6))
        spec = [('fused_hz', 'fused rate (Hz)'), ('pipeline_cpu_pct_of_core', 'pipeline CPU (% of a core)'), ('coloured_pct', 'points coloured (%)'), ('power_est_input_a_mean', 'est. current (A)')]
        out = []
        for ax, (k, ttl) in zip(axs, spec):
            vals = [col([r], k)[0] for r in rows]
            xs = [lab[r['configuration']] for r in rows]
            ax.bar(xs, np.nan_to_num(vals), color=[C[0], C[1], C[2]][:len(xs)] if len(xs) == 3 else [C[i] for i in xs])
            ax.set_xticks(xs)
            ax.set_xticklabels([names[i] for i in xs], rotation=40, ha='right', fontsize=6)
            ax.set_title(ttl, fontsize=7)
            for r, v in zip(rows, vals):
                out.append([r['configuration'], k, v])
        R.save(plt, fig, 'F08_sensor_combinations', ['configuration', 'metric', 'value'], out, 'Sensor combinations: fused rate, CPU, share of coloured points and estimated current (blank bar = not produced in that configuration).')
    except Exception as exc:
        T.log(f'figure F08: {exc!r}')
    try:
        rows = rows_of(run / '08_accuracy' / 'range_accuracy' / 'band_summary.csv')
        fig, ax = plt.subplots(figsize=(W, 2.8))
        x = np.arange(len(rows))
        rms, rep = col(rows, 'plane_rms_mm'), col(rows, 'repeatability_std_mm')
        ax.bar(x - 0.2, np.nan_to_num(rms), 0.4, color=C[0], label='plane RMS (mm)')
        ax.bar(x + 0.2, np.nan_to_num(rep), 0.4, color=C[1], label='repeatability, std of distance (mm)')
        ax.set_xticks(x)
        ax.set_xticklabels([r['range_band_m'] + ' m' for r in rows], fontsize=7)
        ax.set_ylabel('mm')
        ax.legend(fontsize=6)
        R.save(plt, fig, 'F09_range_accuracy_by_band', ['range_band_m', 'plane_rms_mm', 'repeatability_std_mm'], [[r['range_band_m'], r['plane_rms_mm'], r['repeatability_std_mm']] for r in rows], 'Plane RMS and distance repeatability by range band (strongest plane in each band).')
    except Exception as exc:
        T.log(f'figure F09: {exc!r}')
    try:
        rows = [r for r in rows_of(run / '08_accuracy' / 'range_accuracy' / 'truth_errors.csv') if r.get('error_mm(lidar - tape)')]
        if rows:
            fig, ax = plt.subplots(figsize=(W, 2.6))
            ax.bar(range(len(rows)), col(rows, 'error_mm(lidar - tape)'), color=C[0])
            ax.axhline(0, color='#555555', lw=0.6)
            ax.set_xticks(range(len(rows)))
            ax.set_xticklabels([f"{r['object']}\n{float(r['tape_distance_m']):.1f} m" for r in rows], fontsize=7)
            ax.set_ylabel('LiDAR minus tape (mm)')
            R.save(plt, fig, 'F10_truth_error_per_object', ['object', 'tape_distance_m', 'error_mm'], [[r['object'], r['tape_distance_m'], r['error_mm(lidar - tape)']] for r in rows], 'Signed distance error of the detected plane against the tape distance, per object.')
    except Exception as exc:
        T.log(f'figure F10: {exc!r}')
    try:
        files = sorted(glob.glob(str(run / '15_fusion_info' / 'scene' / 'range_hist_*.csv')))
        if files:
            fig, ax = plt.subplots(figsize=(W, 2.8))
            out = []
            for f, colr in zip(files, C):
                r = rows_of(f)
                lo = col(r, 'range_lo_m')
                ax.plot(lo + 0.5, col(r, 'share_pct'), color=colr, lw=1.0, label=Path(f).stem.replace('range_hist_', ''))
                out += [[Path(f).stem.replace('range_hist_', ''), a, b] for a, b in zip(lo, col(r, 'share_pct'))]
            ax.axvspan(15, 25, color=C[3], alpha=0.4, lw=0)
            ax.set_xlabel('range (m)')
            ax.set_ylabel('share of merged points (%)')
            ax.legend(fontsize=6)
            R.save(plt, fig, 'F11_scene_range_histogram', ['snapshot', 'range_lo_m', 'share_pct'], out, 'Range histogram of the merged cloud at the three snapshots; the shaded band is 15 to 25 m.')
    except Exception as exc:
        T.log(f'figure F11: {exc!r}')
    try:
        rows = rows_of(run / '08_accuracy' / 'alignment' / 'sweep_scores.csv')
        if rows:
            axes = []
            for r in rows:
                if r['axis'] not in axes:
                    axes.append(r['axis'])
            fig, axs = plt.subplots(2, 3, figsize=(W, 4.2))
            out = []
            for ax, axname in zip(axs.flat, axes):
                for bi, band in enumerate(('below_10m', '10_to_20m', 'above_20m')):
                    sub = [r for r in rows if r['axis'] == axname and r['range_band'] == band and r['score']]
                    if sub:
                        x, y = col(sub, 'offset'), col(sub, 'score')
                        ax.plot(x, y, color=C[bi] if bi < 3 else '#555555', lw=0.9, label=band.replace('_', ' '))
                        out += [[band, axname, a, b] for a, b in zip(x, y)]
                ax.axvline(0, color='#999999', lw=0.5)
                ax.set_title(axname.replace('_', ' '), fontsize=7)
                ax.set_xlabel(sub[0]['unit'] if rows and sub else '', fontsize=6)
            axs.flat[0].legend(fontsize=5)
            R.save(plt, fig, 'F12_alignment_sweep', ['range_band', 'axis', 'offset', 'score'], out, 'Alignment score versus artificial offset per axis and range band (indirect measure; vertical line = deployed calibration).')
    except Exception as exc:
        T.log(f'figure F12: {exc!r}')
    try:
        rows = rows_of(run / '03_timing_stages' / 'stage_timing.csv')
        keep = [r for r in rows if r['stage'] in ('projection_rgb', 'colouring_rgb', 'fusion_step_total', 'icp_registration_per_scan', 'merge_window', 'streamer_encode_v1', 'streamer_encode_v2')]
        if keep:
            fig, ax = plt.subplots(figsize=(W, 3.6))
            lab = [f"{r['stage']} | {r['setting']}" for r in keep]
            med, p95 = col(keep, 'median_ms'), col(keep, 'p95_ms')
            y = np.arange(len(keep))
            ax.barh(y, med, color=C[0])
            ax.errorbar(med, y, xerr=[np.zeros(len(keep)), np.maximum(p95 - med, 0)], fmt='none', ecolor=C[1], lw=0.8)
            ax.set_yticks(y)
            ax.set_yticklabels(lab, fontsize=5)
            ax.set_xscale('log')
            ax.set_xlabel('ms (median, bar to p95), log scale')
            R.save(plt, fig, 'F13_stage_timing', ['stage', 'setting', 'median_ms', 'p95_ms'], [[r['stage'], r['setting'], r['median_ms'], r['p95_ms']] for r in keep], 'Time per processing stage (isolated benchmark on real captured data).')
    except Exception as exc:
        T.log(f'figure F13: {exc!r}')
    try:
        fig, ax = plt.subplots(figsize=(W, 2.8))
        out = []
        for key, colr in (('lidar_restamped', C[0]), ('fused', C[1]), ('merged', C[2])):
            f = run / '04_freshness' / f'age_samples_{key}.csv'
            if f.exists():
                v = np.sort(col(rows_of(f), 'age_ms'))
                if len(v):
                    ax.plot(v, np.arange(1, len(v) + 1) / len(v), color=colr, lw=1.0, label=f'{key} (median {np.median(v):.0f} ms)')
                    out += [[key, f'{a:.2f}', f'{b:.4f}'] for a, b in zip(v[::max(1, len(v) // 400)], (np.arange(1, len(v) + 1) / len(v))[::max(1, len(v) // 400)])]
        if out:
            ax.set_xlabel('age at the probe (ms)')
            ax.set_ylabel('cumulative share')
            ax.legend(fontsize=6)
            R.save(plt, fig, 'F14_data_age_distribution', ['topic', 'age_ms', 'cumulative_share'], out, 'Data age (message stamp to arrival) at ROS level; no viewer, no streamer encode, no network.')
        else:
            plt.close(fig)
    except Exception as exc:
        T.log(f'figure F14: {exc!r}')


def build_tables(R):
    run = R.run

    def add(name, src, caption, cols=None):
        rows = rows_of(run / src)
        if not rows:
            return
        header = cols or list(rows[0].keys())
        R.table(name, header, [[r.get(h, '') for h in header] for r in rows], caption)
    add('T01_rates', '02_rates/rates.csv', 'Message rates and intervals per topic over the rates window (no viewer, thermal camera not connected).')
    add('T02_sensor_combinations', '15_fusion_info/combinations/combinations.csv', 'Sensor combinations: LiDAR only, LiDAR + RGB, and the deployed pipeline with the thermal camera not connected.')
    add('T03_combination_cpu_per_process', '15_fusion_info/combinations/combinations_cpu_per_process.csv', 'CPU per process (% of one core) for each configuration.')
    add('T04_cpu_per_process', '05_cpu_memory/cpu_per_process.csv', 'Per-process CPU, resident memory and threads over the CPU window.')
    add('T05_cpu_per_core', '05_cpu_memory/cpu_per_core.csv', 'Per-core load over the CPU window.')
    add('T06_range_accuracy_by_band', '08_accuracy/range_accuracy/band_summary.csv', 'Plane RMS and repeatability of the strongest plane in each range band.')
    add('T07_plane_groups', '08_accuracy/range_accuracy/planes_by_band.csv', 'Every detected plane group with its equation, detection share and statistics.')
    add('T08_truth_errors', '08_accuracy/range_accuracy/truth_errors.csv', 'Detected plane distance against tape-measured distance (the tape carries its own error).')
    add('T09_stage_timing', '03_timing_stages/stage_timing.csv', 'Time per processing stage.')
    add('T10_freshness_stages', '04_freshness/freshness_stages.csv', 'Data age at ROS level per stage.')
    add('T11_power_by_phase', '09_power/power_by_phase.csv', 'Estimated input current, power and supply voltage per phase (estimated from the PMIC rails).')
    al = jload(run / '08_accuracy' / 'alignment' / 'summary.json')
    if al and 'bands' in al:
        rows = []
        for b, info in al['bands'].items():
            rows.append([b, info['n_edge_points'], f"{info['score_deployed']:.4f}" if info['score_deployed'] == info['score_deployed'] else '', f"{info['score_peak']:.4f}" if info.get('score_peak') else '',
                         f"{info['score_ratio_deployed_to_peak']:.4f}" if info.get('score_ratio_deployed_to_peak') else '', info.get('best_single_axis') or '', info['verdict']])
        R.table('T12_alignment_summary', ['range_band', 'n_depth_edge_points', 'score_deployed', 'score_peak_of_sweep', 'deployed/peak', 'axis of peak', 'verdict'], rows,
                'Alignment sweep summary per range band (indirect measure: LiDAR depth edges against image gradient edges).')
    sc = jload(run / '15_fusion_info' / 'scene' / 'summary.json')
    if sc and 'mean_over_snapshots' in sc:
        m = sc['mean_over_snapshots']
        R.table('T13_scene_range_bands', ['quantity', 'value'], [[k, f'{v:.2f}'] for k, v in m.items()], 'Range statistics of the merged cloud, mean of the three snapshots.')
    (R.tab / 'CAPTIONS.md').write_text('# Table captions\n\n' + '\n'.join(R.captions_tab) + '\n')


def build_images(R):
    run = R.run
    img = run / 'images'
    caps = []
    spec = {'rgb_': 'RGB frame at the snapshot (full resolution)', 'lidar_on_rgb_': 'LiDAR points (5 scans) projected onto the RGB frame with the deployed calibration, coloured by range',
            'fused_view_': 'live fused scan rendered from the sensor viewpoint, RGB-coloured (grey = no colour)', 'merged_view_': 'merged cloud rendered from the sensor viewpoint, RGB-coloured',
            'merged_range_': 'merged cloud rendered from the sensor viewpoint, coloured by range', 'merged_topdown_': 'merged cloud, top-down view (x forward, y left), 5 m grid',
            'merged_side_': 'merged cloud, side view (x forward, z up), 5 m grid', 'viewer_': 'viewer page screenshot (headless browser; blank if it could not draw the viewer)',
            'alignment_example': 'LiDAR projected on the RGB frame: deployed calibration (left) and best single-axis offsets (right), from the alignment sweep'}
    for f in sorted(img.glob('*.png')):
        for pre, text in spec.items():
            if f.name.startswith(pre):
                tag = f.stem[len(pre):] if pre.endswith('_') else ''
                caps.append(f'- `{f.name}`: {text}' + (f' ({tag})' if tag else ''))
                break
    mins = sorted((img / 'rgb_every_minute').glob('*.jpg')) if (img / 'rgb_every_minute').exists() else []
    if mins:
        caps.append(f'- `rgb_every_minute/` ({len(mins)} frames): one RGB frame about every 60 s through the run (JPEG, quality 85); file name = Unix time')
        if G.contact_sheet(mins, img / 'rgb_contact_sheet.jpg'):
            caps.append('- `rgb_contact_sheet.jpg`: contact sheet of the once-a-minute RGB frames')
    (img / 'CAPTIONS.md').write_text('# Image captions\n\n' + '\n'.join(caps) + '\n')
    return len(caps)


# ------------------------------------------------------------------ data dictionary
REGISTRY = [
    ('00_meta/system_1hz.csv', 'all phases', 'master 1 Hz log: temperature (C), ARM clock (MHz), throttle flags, per-core load (%), per-process CPU (% of a core), RSS (MB), threads, memory, WiFi counters', '1 Hz'),
    ('00_meta/markers.jsonl', 'all phases', 'phase start/end markers with Unix time', 'event'),
    ('00_meta/gate_events.jsonl', 'safety gates', 'safety gate trips (reason, phase)', 'event'),
    ('00_meta/top_processes.log', 'status thread', 'busiest processes every 30 s', '30 s'),
    ('00_meta/system_inventory.json', 'P1', 'system, software, git hashes, WiFi/power-saving state', 'once'),
    ('00_meta/thermal_not_connected.json', 'P1', 'evidence that the thermal USB device was absent', 'once'),
    ('02_rates/topic_*.csv', 'probe (whole run)', 'every message: t_arrival (Unix s), stamp (s), n (points or pixels), nbytes', 'per message'),
    ('02_rates/rates.csv', 'P3', 'rate and interval statistics per topic (Hz, ms)', 'window'),
    ('02_rates/rate_series_10s_*.csv', 'P3', 'rate per 10 s bin (Hz)', '0.1 Hz'),
    ('02_rates/coverage_raw.csv', 'probe', 'share of fused points with RGB colour (%, thermal always 0 in Run 1)', 'every 5th fused cloud'),
    ('02_rates/lidar_plane.csv', 'probe', 'dominant plane of every 10th restamped scan: distance (m), normal, inliers, rms (m)', 'every 10th scan'),
    ('03_timing_stages/*.csv', 'P7', 'stage timings (ms), merge windows, ICP trials', 'benchmark'),
    ('04_freshness/*.csv', 'P8', 'data age at ROS level (ms) per message and per stage', 'per message'),
    ('05_cpu_memory/*.csv', 'P6', 'CPU per process (% of a core), per core (%), RSS (MB), threads', 'window'),
    ('08_accuracy/range_accuracy/*.csv', 'P5', 'planes by range band: distance (m), RMS (mm), repeatability (mm), equations, truth errors (mm)', '0.5 s chunks'),
    ('08_accuracy/alignment/*.csv', 'P9', 'alignment sweep: score versus artificial offset (m or deg) per axis and range band', 'sweep'),
    ('09_power/power_raw.csv', 'all phases', 'EXT5V (V), estimated input current (A), power (W), SoC (C); current is estimated from PMIC rails', '~30 Hz'),
    ('09_power/power_by_phase.csv', 'all phases', 'power statistics per phase', 'per phase'),
    ('14_wireless/*.csv', 'WiFi monitor', 'hotspot link counters, client TCP info, server stats (empty if no client connected)', '1 Hz'),
    ('15_fusion_info/scene/*.npz', 'P2', 'merged/fused clouds: xyz (m, sensor frame x fwd y left z up), rgb (uint32 0xRRGGBB, 0 = none), intensity, scan_time_unix, scan_stamp', 'snapshots'),
    ('15_fusion_info/scene/range_hist_*.csv', 'P2', '1 m range histogram of the merged cloud', 'snapshot'),
    ('15_fusion_info/combinations/*.csv', 'P4', 'sensor combination metrics (Hz, ms, %, A, C)', 'window'),
    ('figures/*.csv', 'report', 'the data behind each figure', ''),
    ('tables/*.csv', 'report', 'every table as CSV (and .md)', ''),
]


def build_dictionary(R):
    run = R.run
    lines = ['# Data dictionary (Run 1)\n', 'Thermal camera not connected for this run. Times are Unix seconds unless stated. Sampling/producing phase from the registry below; columns read from the file headers.\n']
    seen = set()
    for pattern, phase, desc, rate in REGISTRY:
        for f in sorted(glob.glob(str(run / pattern))):
            f = Path(f)
            if f in seen or not f.is_file():
                continue
            seen.add(f)
            rel = f.relative_to(run)
            ext = f.suffix.lower()
            cols = ''
            if ext == '.csv':
                try:
                    cols = ', '.join(open(f).readline().strip().split(','))
                except Exception:
                    pass
                samp = rate
                try:
                    r = rows_of(f)
                    key = 't' if r and 't' in r[0] else 't_arrival' if r and 't_arrival' in r[0] else None
                    if key and len(r) > 5:
                        d = np.diff(col(r, key))
                        samp = f'{rate} (median interval {np.nanmedian(d):.3g} s, {len(r)} rows)'
                except Exception:
                    pass
            elif ext == '.npz':
                try:
                    z = np.load(f, allow_pickle=True)
                    cols = 'arrays: ' + ', '.join(f'{k}{tuple(z[k].shape)}' for k in z.files)
                except Exception:
                    pass
                samp = rate
            else:
                samp = rate
            lines.append(f'### `{rel}`\n- phase: {phase}\n- contents: {desc}\n- sampling: {samp}\n' + (f'- columns: {cols}\n' if cols else ''))
    lines.append('\n### Other\n- `images/`: PNG/JPG with captions in `images/CAPTIONS.md`\n- `bags/`, `raw_captures/`: raw recordings and message captures (not in the bundle)\n- `state.json`, `phase_status.json`, `numbers_state.json`: run control\n')
    (run / 'DATA_DICTIONARY.md').write_text('\n'.join(lines))


# ------------------------------------------------------------------ results, numbers, paper map
def fmt(v):
    if v is None:
        return ''
    return f'{v:.4g}' if isinstance(v, float) else str(v)


def numbers_dump(run):
    nums = jload(Path(run) / 'numbers_state.json', {})
    keys = list(dict.fromkeys(list(T.ALL_KEYS) + list(nums)))
    out = {}
    for k in keys:
        v = nums.get(k)
        if v is not None and v.get('value') is not None:
            v = dict(v)
            v['status'] = 'measured' if 'calculated' not in (v.get('note') or '') else 'calculated'
            out[k] = v
        else:
            out[k] = {'value': None, 'unit': (v or {}).get('unit', ''), 'previous': T.PREVIOUS.get(k), 'source_file': '', 'status': 'not measured in Run 1',
                      'note': 'thermal camera not connected in Run 1' if 'thermal' in k else 'not part of Run 1 (see section 7 of the brief)'}
    json.dump(out, open(Path(run) / 'thesis_numbers.json', 'w'), indent=1)
    return out


def build_results(R, nums, args=None):
    run = R.run
    st = jload(run / 'state.json', {})
    lines = ['# RESULTS: Run 1 (thermal camera not connected)', '',
             f'Generated {time.strftime("%Y-%m-%d %H:%M")} from the files in this folder. Boots: {len(st.get("boots", []))}. All numbers trace to the source file shown.', '', '## Phase status', '',
             md_table(['phase', 'status', 'minutes', 'note'], [[PHASE_LABEL.get(p['phase'], p['phase']), p['status'], p.get('minutes', ''), p.get('detail', '')] for p in R.phases])]
    th = jload(run / '00_meta' / 'thermal_not_connected.json', {})
    lines += ['## Thermal camera', '', f'Thermal USB device confirmed absent at the start: **{th.get("confirmed_absent")}**. No phase reads or reports thermal data. All thermal-related items are therefore not measured in this run.', '']
    hs = jload(run / '14_wireless' / 'hotspot_status.json', {})
    ap = rows_of(run / '14_wireless' / 'wifi_link.csv')
    ap_n = col(ap, 'ap_stations')
    lines += ['## Hotspot and clients', '', f'Hotspot (PitFusion3D-AP): **{hs.get("status", "not recorded")}**. Maximum stations seen on the access point: {int(np.nanmax(ap_n)) if len(ap_n) and not np.all(np.isnan(ap_n)) else "no data"}. '
              'No viewer was required; any real client that connected is recorded in 14_wireless but nothing here relies on it.', '']
    gates = []
    gp = run / '00_meta' / 'gate_events.jsonl'
    if gp.exists():
        gates = [json.loads(l) for l in gp.read_text().splitlines() if l.strip()]
    lines += ['## Safety gate events', '', md_table(['time', 'phase', 'reason'], [[time.strftime('%H:%M:%S', time.localtime(g['t'])), g['phase'], g['reason']] for g in gates]) if gates else 'None.\n']
    site = jload(run / '00_meta' / 'run1_inventory.json', {}).get('site_notes', {})
    lines += ['## Site notes', '', md_table(['item', 'value'], [[k, v if v else 'not supplied'] for k, v in site.items()]) if site else 'Not supplied (site_notes.env was empty).\n']
    lines += ['## Numbers (value, unit, previous campaign value, source, measured or calculated)', '',
              md_table(['key', 'value', 'unit', 'previous', 'source', 'status', 'note'], [[k, fmt(v['value']), v.get('unit', ''), fmt(v.get('previous')), v.get('source_file', ''), v.get('status', ''), v.get('note', '')] for k, v in nums.items() if v['value'] is not None]),
              '## Not measured, and why', '',
              md_table(['key', 'previous', 'reason'], [[k, fmt(v.get('previous')), v.get('note', '')] for k, v in nums.items() if v['value'] is None])]
    al = jload(run / '08_accuracy' / 'alignment' / 'summary.json', {})
    if al.get('bands'):
        lines += ['## Alignment sweep (indirect)', '', al.get('method', ''), '', md_table(['band', 'edge points', 'deployed score', 'peak score', 'deployed/peak', 'verdict'],
                                                                                          [[b, i['n_edge_points'], fmt(i['score_deployed']), fmt(i.get('score_peak')), fmt(i.get('score_ratio_deployed_to_peak')), i['verdict']] for b, i in al['bands'].items()])]
    cb = jload(run / '15_fusion_info' / 'combinations' / 'summary.json', {})
    if cb:
        lines += ['## Combinations: what ran', '', '\n'.join(f'- **{k}**: {v.get("what_ran", v.get("note", ""))}' for k, v in cb.items() if isinstance(v, dict) and (v.get('what_ran') or v.get('note'))), '',
                  f'Restore check after the variants: {cb.get("restore_check")}', '']
    lines += ['## Figures, tables, images', '', 'figures/ (PNG 300 dpi + CSV behind each), tables/ (CSV + Markdown, CAPTIONS.md), images/ (CAPTIONS.md). Full file list: DATA_DICTIONARY.md.', '',
              '## Not covered by Run 1', '', 'Anything thermal (Run 2); handheld or moving-platform tests, including the physical-movement check of merging; photographs, mass, dimensions and costs; the USB-C meter reading; '
              'stopwatch capture-to-display delay; WiFi range tests with a second device; full-size board calibration of all three sensors together; the thermal lens focal length; the hypothesis statement; the submission date; the thermal camera reference publisher.']
    (run / 'RESULTS_RUN1.md').write_text('\n'.join(lines) + '\n')


def build_paper_map(R, nums):
    run = R.run

    def val(k):
        v = nums.get(k, {})
        return (f"{fmt(v['value'])} {v.get('unit', '')}".strip(), v.get('source_file', '')) if v.get('value') is not None else ('not measured', '')
    al = jload(run / '08_accuracy' / 'alignment' / 'summary.json', {}).get('bands', {})
    rows = [
        ('Abstract: mounted (tripod) result', ', '.join(f'{k} {val(k)[0]}' for k in ('rate_fused_hz', 'rate_merged_hz', 'lidar_scans_lost_pct', 'truth_error_mean_abs_mm')), '08_accuracy/range_accuracy/truth_errors.csv; 02_rates/rates.csv',
         'State the mounted-rig fused/merged rates and mean absolute distance error against the tape distances (see truth_errors.csv for which objects).'),
        ('Outdoor ~20 m test (2.4.1)', f'mean range {val("scene_mean_range_m")[0]}, share 15-25 m {val("scene_share_15_25m_pct")[0]}', '15_fusion_info/scene/range_summary.csv; images/*',
         'Replace the scene description with the measured range histogram of the merged cloud; cite the images.'),
        ('Table 4: spatial accuracy 2 to 20 m', ', '.join(f'{lo}-{hi} m RMS {val(f"plane_rms_mm_{lo}_{hi}m")[0]}' for lo, hi in ((2, 5), (5, 10), (10, 15), (15, 20), (20, 30))), '08_accuracy/range_accuracy/band_summary.csv',
         'Fill the 2-20 m rows with plane RMS and repeatability by band; state truth distances are tape-measured with their own error.'),
        ('3.3.1 Optimisation (before/after)', 'candidate lines only', '00_meta/repo_extract/repo_extract_candidates.csv', 'A person must check candidates against Figure 12 and Tables 7 and 8; nothing is measured here.'),
        ('3.3.3 CPU baseline', f'{val("pipeline_cpu_pct_of_core")[0]} of one core, RSS {val("pipeline_mem_mb")[0]}', '05_cpu_memory/cpu_per_process.csv', 'Use as the no-viewer, no-thermal baseline; thermal node was retrying at the time.'),
        ('3.4 Spatial accuracy and alignment at range', ', '.join(f'{b}: deployed/peak {fmt(i.get("score_ratio_deployed_to_peak"))} ({i["verdict"]})' for b, i in al.items()) or 'not measured', '08_accuracy/alignment/summary.json',
         'Describe as an INDIRECT edge-agreement measure per range band, not ground-truth accuracy.'),
        ('Table 10, row 1 (LiDAR only)', f'CPU {val("combo_lidar_only_cpu_pct_of_core")[0]}, current {val("combo_lidar_only_current_a")[0]}', '15_fusion_info/combinations/combinations.csv',
         'Fusion and merge cannot run on LiDAR alone (see summary.json "what_ran"): report driver + restamp load only.'),
        ('Table 10, row 2 (LiDAR + RGB)', f'fused {val("combo_lidar_rgb_fused_hz")[0]}, merged {val("combo_lidar_rgb_merged_hz")[0]}, CPU {val("combo_lidar_rgb_cpu_pct_of_core")[0]}, coloured {val("combo_lidar_rgb_coloured_pct")[0]}, current {val("combo_lidar_rgb_current_a")[0]}',
         '15_fusion_info/combinations/combinations.csv', 'Fill the LiDAR + RGB row.')]
    (run / 'PAPER_MAP.md').write_text('# PAPER MAP (Run 1)\n\nOne row per paper item: new value, file, implied wording change. Check each value in its source file before use.\n\n' + md_table(['paper item', 'new value (Run 1)', 'file', 'implied wording change'], rows))


def build_bundle(run):
    run = Path(run)
    z = run / 'results_bundle.zip'
    z.unlink(missing_ok=True)
    skip = {'bags', 'raw_captures'}
    with zipfile.ZipFile(z, 'w', zipfile.ZIP_DEFLATED) as zf:
        for f in run.rglob('*'):
            if f.is_file() and f != z and not (set(f.relative_to(run).parts) & skip):
                zf.write(f, f.relative_to(run.parent))
    return z


def build_all(run, args=None):
    run = Path(run)
    T.S.out = T.S.out or run
    R = Report(run)
    build_figures(R)
    (R.fig / 'CAPTIONS.md').write_text('# Figure captions\n\n' + '\n'.join(R.captions_fig) + '\n')
    build_tables(R)
    build_images(R)
    nums = numbers_dump(run)
    build_dictionary(R)
    build_results(R, nums, args)
    build_paper_map(R, nums)
    for d in ('16_human_tests',):
        (run / d).mkdir(exist_ok=True)
    (run / '16_human_tests' / 'NOT_COVERED_BY_RUN1.md').write_text('Not covered by Run 1: anything thermal (Run 2); handheld or moving-platform tests including the physical-movement check of merging; photographs, mass, dimensions, costs; '
                                                                    'USB-C meter reading; stopwatch capture-to-display delay; WiFi range tests with a second device; full-size board calibration of all three sensors together; thermal lens focal length; '
                                                                    'hypothesis statement; submission date; thermal camera reference publisher.\n')
    z = build_bundle(run)
    T.log(f'reports written; bundle {z} ({z.stat().st_size / 1e6:.1f} MB); {len(R.made)} figures')
