#!/usr/bin/env python3
"""Render the optimization report (PDF + PNG figures) from benchmark results.

    python3 benchmark/bench_offline.py --json benchmark/results.json
    python3 benchmark/make_report.py

Writes benchmark/report/optimization_report.pdf plus standalone PNGs that
the README embeds.
"""

import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, 'report')

INK = '#1b1f24'
MUTED = '#5b6470'
BASE_C = '#c2503a'
OPT_C = '#2f7d6d'
ACCENT = '#3b6ea5'
GRID = '#d7dbe0'

STAGE_ORDER = [
    'cloud_read', 'cloud_write', 'voxel_downsample', 'fusion_projection', 'window_merge',
]

SHORT_NAMES = {
    'cloud_read': 'PointCloud2\ndecode',
    'cloud_write': 'PointCloud2\nencode',
    'voxel_downsample': 'Voxel\ndownsample',
    'fusion_projection': 'RGB+thermal\nfusion',
    'window_merge': 'Windowed\nICP merge',
}

OPTIMIZATIONS = [
    ('PointCloud2 decode',
     'sensor_msgs_py rebuilt an intermediate structured array per message.',
     'Map msg.data with a numpy dtype carrying the real field offsets and take '
     'the x/y/z/intensity block as one strided view - zero copies.'),
    ('PointCloud2 encode',
     'pc2.create_cloud re-packs the payload field by field in Python.',
     'Write the float32 matrix straight to msg.data with .tobytes(), since the '
     'output layout is already known and dense.'),
    ('Voxel downsample',
     'np.unique(keys, axis=0) lexsorts an (N,3) array - the dominant cost of '
     'every merge window.',
     'Hash the 3 integer voxel coordinates into one int64 key so the dedupe '
     'becomes a 1-D sort. Selection is bit-identical.'),
    ('RGB + thermal fusion',
     'Converted the entire 1280x720 thermal frame to Kelvin (921k pixels) when '
     'only ~24k points sample it, in float64, via cv_bridge.',
     'Gather the raw uint16 at the projected pixels first and convert only '
     'those ~24k values; stay in float32; fold the extrinsic hops into one '
     '(M, b) precomputed at startup; reuse the RGB projection when the rig is '
     'co-located.'),
    ('Windowed ICP merge',
     'Each scan rebuilt a KD-tree over an ever-growing raw map (O(N^2)), and '
     'the map was re-voxelised from scratch every iteration.',
     'Keep the running map voxel-downsampled and grow it incrementally, cap '
     'the ICP source/target sizes, and use distance_upper_bound so the tree '
     'prunes. Correspondence search is serial below 20k points because '
     "scipy's per-call thread spawn cost exceeded the parallel gain."),
]


def style_axes(ax):
    ax.set_facecolor('white')
    for spine in ('top', 'right'):
        ax.spines[spine].set_visible(False)
    for spine in ('left', 'bottom'):
        ax.spines[spine].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(axis='y', color=GRID, linewidth=0.7, alpha=0.9)
    ax.set_axisbelow(True)


def load_results(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return json.load(handle)


def fig_stage_times(payload):
    results = payload['results']
    stages = [s for s in STAGE_ORDER if s in results]
    base = [results[s]['baseline']['median_ms'] for s in stages]
    opt = [results[s]['optimized']['median_ms'] for s in stages]
    labels = [SHORT_NAMES.get(s, s) for s in stages]

    fig, ax = plt.subplots(figsize=(9, 4.8))
    x = np.arange(len(stages))
    width = 0.38
    ax.bar(x - width / 2, base, width, label='Baseline', color=BASE_C)
    ax.bar(x + width / 2, opt, width, label='Optimized', color=OPT_C)
    ax.set_yscale('log')
    ax.set_ylabel('median time per call (ms, log scale)', color=MUTED, fontsize=10)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_title('Per-stage cost: baseline vs optimized', color=INK, fontsize=13,
                 fontweight='bold', pad=14)
    style_axes(ax)
    ax.legend(frameon=False, fontsize=9)

    for xi, (b, o) in enumerate(zip(base, opt)):
        ax.text(xi + width / 2, o * 1.15, '%.2fx' % (b / o), ha='center',
                fontsize=8.5, color=OPT_C, fontweight='bold')
    fig.tight_layout()
    return fig


def fig_speedups(payload):
    results = payload['results']
    stages = [s for s in STAGE_ORDER if s in results]
    speed = [results[s]['baseline']['median_ms'] / results[s]['optimized']['median_ms']
             for s in stages]
    labels = [SHORT_NAMES.get(s, s).replace('\n', ' ') for s in stages]
    order = np.argsort(speed)

    fig, ax = plt.subplots(figsize=(9, 4.2))
    ypos = np.arange(len(stages))
    ax.barh(ypos, [speed[i] for i in order], color=ACCENT, height=0.55)
    ax.set_yticks(ypos)
    ax.set_yticklabels([labels[i] for i in order], fontsize=9)
    ax.axvline(1.0, color=BASE_C, linewidth=1.2, linestyle='--')
    ax.text(1.03, -0.7, 'no change', color=BASE_C, fontsize=8)
    ax.set_xlabel('speedup (x faster)', color=MUTED, fontsize=10)
    ax.set_title('Speedup by stage', color=INK, fontsize=13, fontweight='bold', pad=14)
    style_axes(ax)
    ax.grid(axis='x', color=GRID, linewidth=0.7)
    ax.grid(axis='y', visible=False)
    for i, idx in enumerate(order):
        ax.text(speed[idx] * 1.02, i, '%.2fx' % speed[idx], va='center',
                fontsize=9, color=INK, fontweight='bold')
    ax.set_xlim(0, max(speed) * 1.18)
    fig.tight_layout()
    return fig


def fig_window_budget(payload):
    """Where the time goes for one 1s merge window, baseline vs optimized."""
    results = payload['results']
    scans = 11
    base_total = (
        results['window_merge']['baseline']['median_ms']
        + scans * results['fusion_projection']['baseline']['median_ms']
        + scans * (results['cloud_read']['baseline']['median_ms']
                   + results['cloud_write']['baseline']['median_ms'])
    )
    opt_total = (
        results['window_merge']['optimized']['median_ms']
        + scans * results['fusion_projection']['optimized']['median_ms']
        + scans * (results['cloud_read']['optimized']['median_ms']
                   + results['cloud_write']['optimized']['median_ms'])
    )

    fig, ax = plt.subplots(figsize=(9, 4.4))
    parts_base = [
        results['window_merge']['baseline']['median_ms'],
        scans * results['fusion_projection']['baseline']['median_ms'],
        scans * (results['cloud_read']['baseline']['median_ms']
                 + results['cloud_write']['baseline']['median_ms']),
    ]
    parts_opt = [
        results['window_merge']['optimized']['median_ms'],
        scans * results['fusion_projection']['optimized']['median_ms'],
        scans * (results['cloud_read']['optimized']['median_ms']
                 + results['cloud_write']['optimized']['median_ms']),
    ]
    names = ['ICP merge', 'fusion x%d' % scans, 'cloud I/O x%d' % scans]
    colors = ['#c2503a', '#e0a33e', '#3b6ea5']

    left_b = left_o = 0.0
    for value_b, value_o, name, color in zip(parts_base, parts_opt, names, colors):
        ax.barh(1, value_b, left=left_b, color=color, height=0.42, label=name)
        ax.barh(0, value_o, left=left_o, color=color, height=0.42)
        left_b += value_b
        left_o += value_o

    ax.axvline(1000.0, color=INK, linestyle='--', linewidth=1.4)
    ax.text(1060, 0.62, '1s real-time budget', fontsize=9, color=INK, fontweight='bold')
    ax.set_yticks([0, 1])
    ax.set_yticklabels(['Optimized\n%.0f ms' % opt_total, 'Baseline\n%.0f ms' % base_total],
                       fontsize=9)
    ax.set_xlabel('CPU time to process one 1-second window (ms)', color=MUTED, fontsize=10)
    ax.set_title('Real-time budget for one merge window', color=INK, fontsize=13,
                 fontweight='bold', pad=14)
    style_axes(ax)
    ax.grid(axis='y', visible=False)
    ax.legend(frameon=False, fontsize=9, ncol=3, loc='lower right')
    ax.set_xlim(0, max(base_total, 1000) * 1.12)
    fig.tight_layout()
    return fig


def fig_scan_pattern(csv_path, sample=9000):
    """Grid-snapped sampling vs true-angle reconstruction of the Avia pattern."""
    fig, axes = plt.subplots(1, 2, figsize=(9.5, 4.6))
    if not csv_path or not os.path.exists(csv_path):
        for ax in axes:
            ax.text(0.5, 0.5, 'avia.csv not found', ha='center', va='center')
            ax.set_axis_off()
        return fig

    raw = np.loadtxt(csv_path, delimiter=',', skiprows=1, usecols=(1, 2),
                     max_rows=sample)
    az = np.radians(raw[:, 0])
    zen = np.radians(raw[:, 1]) - math.pi / 2.0
    el = -zen

    h_samples, v_samples = 200, 120
    hmin, hmax = -math.radians(70.4) / 2, math.radians(70.4) / 2
    vmin, vmax = -math.radians(77.2) / 2, math.radians(77.2) / 2

    def to_index(a, lo, hi, n):
        return np.round(np.clip((a - lo) / (hi - lo), 0, 1) * (n - 1))

    az_snapped = to_index(az, hmin, hmax, h_samples) / (h_samples - 1) * (hmax - hmin) + hmin
    el_snapped = to_index(el, vmin, vmax, v_samples) / (v_samples - 1) * (vmax - vmin) + vmin

    axes[0].scatter(np.degrees(az_snapped), np.degrees(el_snapped), s=0.35,
                    c=BASE_C, alpha=0.5, linewidths=0)
    axes[0].set_title('Before: snapped to the %dx%d grid\n(only %d distinct elevations)'
                      % (h_samples, v_samples, v_samples),
                      fontsize=10.5, color=INK, fontweight='bold')

    axes[1].scatter(np.degrees(az), np.degrees(el), s=0.35, c=OPT_C, alpha=0.5,
                    linewidths=0)
    axes[1].set_title('After: true per-ray angles\n(non-repetitive rosette)',
                      fontsize=10.5, color=INK, fontweight='bold')

    for ax in axes:
        ax.set_xlabel('azimuth (deg)', fontsize=9, color=MUTED)
        ax.set_aspect('equal')
        style_axes(ax)
        ax.grid(color=GRID, linewidth=0.6, alpha=0.7)
    axes[0].set_ylabel('elevation (deg)', fontsize=9, color=MUTED)
    fig.suptitle('Livox Avia scan pattern reconstruction (%d rays)' % sample,
                 fontsize=13, color=INK, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    return fig


def fig_text_page(payload):
    fig = plt.figure(figsize=(8.5, 11))
    fig.text(0.07, 0.995, 'What changed, and why it was slow', fontsize=17,
             fontweight='bold', color=INK, va='top')

    y = 0.945
    for title, before, after in OPTIMIZATIONS:
        fig.text(0.07, y, title, fontsize=12, fontweight='bold', color=ACCENT, va='top')
        y -= 0.028
        fig.text(0.09, y, 'Before', fontsize=9, fontweight='bold', color=BASE_C, va='top')
        wrapped = _wrap(before, 92)
        fig.text(0.155, y, wrapped, fontsize=9, color=INK, va='top', linespacing=1.5)
        y -= 0.021 * (wrapped.count('\n') + 1) + 0.012
        fig.text(0.09, y, 'After', fontsize=9, fontweight='bold', color=OPT_C, va='top')
        wrapped = _wrap(after, 92)
        fig.text(0.155, y, wrapped, fontsize=9, color=INK, va='top', linespacing=1.5)
        y -= 0.021 * (wrapped.count('\n') + 1) + 0.03

    meta = payload['meta']
    fig.text(0.07, 0.045,
             'Measured on %s | %d cores | Python %s | numpy %s'
             % (meta['machine'], meta['cpu_count'], meta['python'], meta['numpy']),
             fontsize=8.5, color=MUTED)
    return fig


def _wrap(text, width):
    words = text.split()
    lines, line = [], ''
    for word in words:
        candidate = (line + ' ' + word).strip()
        if len(candidate) > width:
            lines.append(line)
            line = word
        else:
            line = candidate
    if line:
        lines.append(line)
    return '\n'.join(lines)


def fig_title_page(payload):
    results = payload['results']
    meta = payload['meta']
    total_base = sum(r['baseline']['median_ms'] for r in results.values())
    total_opt = sum(r['optimized']['median_ms'] for r in results.values())

    fig = plt.figure(figsize=(8.5, 11))
    fig.text(0.07, 0.94, 'PitFusion3D', fontsize=30, fontweight='bold', color=INK)
    fig.text(0.07, 0.90, 'Pipeline optimization report', fontsize=17, color=ACCENT)
    fig.text(0.07, 0.865,
             'Raspberry Pi 5 target: RGB + thermal + Livox Avia fusion',
             fontsize=11, color=MUTED)

    fig.patches.append(plt.Rectangle((0.07, 0.60), 0.86, 0.22, transform=fig.transFigure,
                                     facecolor='#f2f5f7', edgecolor=GRID))
    fig.text(0.5, 0.755, '%.2fx' % (total_base / total_opt), fontsize=52,
             fontweight='bold', color=OPT_C, ha='center')
    fig.text(0.5, 0.712, 'overall speedup across benchmarked stages', fontsize=11,
             color=MUTED, ha='center')
    fig.text(0.5, 0.655, '%.0f ms  ->  %.0f ms   per merge window'
             % (total_base, total_opt), fontsize=13, color=INK, ha='center')

    rows = [('stage', 'baseline', 'optimized', 'speedup')]
    for key in STAGE_ORDER:
        if key not in results:
            continue
        entry = results[key]
        b = entry['baseline']['median_ms']
        o = entry['optimized']['median_ms']
        rows.append((entry['label'], '%.2f ms' % b, '%.2f ms' % o, '%.2fx' % (b / o)))

    y = 0.53
    for i, row in enumerate(rows):
        weight = 'bold' if i == 0 else 'normal'
        color = MUTED if i == 0 else INK
        fig.text(0.07, y, row[0][:46], fontsize=9.5, color=color, fontweight=weight)
        fig.text(0.62, y, row[1], fontsize=9.5, color=color, fontweight=weight, ha='right')
        fig.text(0.78, y, row[2], fontsize=9.5, color=color, fontweight=weight, ha='right')
        fig.text(0.92, y, row[3], fontsize=9.5,
                 color=OPT_C if i else MUTED, fontweight='bold' if i else 'bold', ha='right')
        if i == 0:
            y -= 0.012
            fig.lines.append(plt.Line2D([0.07, 0.92], [y, y], transform=fig.transFigure,
                                        color=GRID, linewidth=1))
        y -= 0.026

    fig.text(0.07, 0.20, 'Correctness fix included in this release', fontsize=12,
             fontweight='bold', color=INK)
    fig.text(0.07, 0.17,
             _wrap('The Livox resampler previously snapped every return to the nearest '
                   'cell of the uniform gpu_lidar grid, quantising the scan onto only 120 '
                   'distinct elevations and rendering as horizontal banding rather than '
                   'the Avia rosette. Points are now rebuilt along each pattern ray\'s own '
                   'azimuth/zenith using bilinear range interpolation.', 96),
             fontsize=9.5, color=INK, va='top', linespacing=1.6)

    fig.text(0.07, 0.05, 'Measured on %s | %d cores | numpy %s'
             % (meta['platform'][:60], meta['cpu_count'], meta['numpy']),
             fontsize=8, color=MUTED)
    return fig


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--json', default=os.path.join(HERE, 'results.json'))
    parser.add_argument('--pattern-csv',
                        default=os.path.join(os.path.dirname(HERE), 'config',
                                             'scan_patterns', 'avia.csv'))
    parser.add_argument('--out-dir', default=OUT_DIR)
    args = parser.parse_args()

    if not os.path.exists(args.json):
        sys.exit('No results at %s - run bench_offline.py first' % args.json)

    payload = load_results(args.json)
    os.makedirs(args.out_dir, exist_ok=True)

    figures = [
        ('title', fig_title_page(payload)),
        ('stage_times', fig_stage_times(payload)),
        ('speedups', fig_speedups(payload)),
        ('window_budget', fig_window_budget(payload)),
        ('scan_pattern', fig_scan_pattern(args.pattern_csv)),
        ('details', fig_text_page(payload)),
    ]

    pdf_path = os.path.join(args.out_dir, 'optimization_report.pdf')
    with PdfPages(pdf_path) as pdf:
        for _, fig in figures:
            pdf.savefig(fig, facecolor='white')

    for name, fig in figures:
        if name in ('title', 'details'):
            continue
        fig.savefig(os.path.join(args.out_dir, '%s.png' % name), dpi=150,
                    facecolor='white', bbox_inches='tight')
        plt.close(fig)

    print('Wrote %s' % pdf_path)
    print('Wrote PNGs to %s' % args.out_dir)


if __name__ == '__main__':
    main()
