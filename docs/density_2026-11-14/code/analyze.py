#!/usr/bin/env python3
"""Tabulate and plot the density campaign: points shown vs refresh, link cost, CPU, power, temperature."""
import glob, json, os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__)); DATA = os.path.join(HERE, '..', 'data'); OUT = os.path.join(HERE, '..', 'results')
os.makedirs(OUT, exist_ok=True)
rows = []
for f in sorted(glob.glob(f'{DATA}/results_w*.jsonl')):
    window = float(os.path.basename(f)[len('results_w'):-len('.jsonl')])
    for line in open(f):
        r = json.loads(line); r['window'] = window; rows.append(r)
rows.sort(key=lambda r: (r['window'], r['cap']))


def g(r, *keys):
    for k in keys:
        r = (r or {}).get(k) if isinstance(r, dict) else None
    return r


lines = ['| window | cap | points shown | merged Hz (pipeline) | viewer updates/s | wire KB | Mbit/s | data age p50 ms | streamer CPU % | pipeline CPU % | est. input A (max) | temp max C | 5 cm coverage % | cloud available |',
         '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
for r in rows:
    lines.append('| {w} s | {cap} | {pts:,.0f} | {mhz} | {ups} | {kb} | {mb} | {age:.0f} | {sc} | {pc} | {amp:.2f} | {tc} | {cov} | {full} |'.format(
        w=r['window'], cap=r['cap'], pts=r['points_per_cloud'] or 0, mhz=g(r, 'topics', 'points_merged', 'hz'), ups=r['updates_per_s'],
        kb=r['wire_kb_mean'], mb=r['viewer_mbit_s'], age=g(r, 'data_age_ms', 'p50') or 0, sc=g(r, 'cpu', 'streamer_pct'), pc=g(r, 'cpu', 'pipeline_pct'),
        amp=g(r, 'power', 'est_input_conservative_a_max') or 0, tc=g(r, 'power', 'temp_c_max'), cov=g(r, 'quality', 'coverage_5cm_pct'), full=g(r, 'quality', 'full_points')))
open(f'{OUT}/ladder_table.md', 'w').write('\n'.join(lines) + '\n'); print('\n'.join(lines))

if rows:
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))
    for w in sorted({r['window'] for r in rows}):
        rs = [r for r in rows if r['window'] == w]; pts = [r['points_per_cloud'] for r in rs]
        ax[0].plot(pts, [r['updates_per_s'] for r in rs], 'o-', label=f'window {w} s')
        ax[1].plot(pts, [r['viewer_mbit_s'] for r in rs], 'o-', label=f'window {w} s')
        ax[2].plot(pts, [g(r, 'power', 'est_input_conservative_a_max') for r in rs], 'o-', label=f'window {w} s')
    ax[0].set_ylabel('viewer updates per second'); ax[1].set_ylabel('link needed (Mbit/s)'); ax[2].set_ylabel('estimated input current, worst case (A)')
    ax[2].axhline(5.0, color='r', ls='--', lw=1); ax[2].text(0.02, 0.93, '5 A limit', transform=ax[2].transAxes, color='r')
    for a in ax: a.set_xlabel('points shown per update'); a.grid(alpha=.3); a.legend()
    fig.tight_layout(); fig.savefig(f'{OUT}/ladder_overview.png', dpi=120); print('saved plot')
