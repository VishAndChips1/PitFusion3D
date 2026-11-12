"""Common loaders/helpers for build_report.py"""
import os, sys, json, csv, glob, re, time
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); DATA = f'{HERE}/data'; FIG = f'{HERE}/figures'; LOGD = f'{HERE}/logs'
plt.rcParams.update({'figure.dpi': 110, 'font.size': 9, 'axes.grid': True, 'grid.alpha': .3, 'axes.spines.top': False, 'axes.spines.right': False})
C = ['#4A5D7A', '#B5532A', '#3E8E7E', '#C9A227', '#7A4A7A', '#5A5A5A', '#2A7AB5', '#B52A5A']
def pj(name):
    p = f'{DATA}/{name}.json'; return json.load(open(p)) if os.path.exists(p) else None
def load_csv(path, numeric=None):
    """-> dict col -> numpy array (float where possible else str)."""
    if not os.path.exists(path): return {}
    rows = list(csv.reader(open(path))); 
    if len(rows) < 2: return {}
    hdr = rows[0]; n = len(hdr); body = [r for r in rows[1:] if len(r) == n]; out = {}
    for i, h in enumerate(hdr):
        col = [r[i] for r in body]
        try: out[h] = np.array([float(x) if x != '' else np.nan for x in col])
        except ValueError: out[h] = np.array(col)
    return out
def markers():
    p = f'{DATA}/markers.jsonl'; return [json.loads(l) for l in open(p)] if os.path.exists(p) else []
def phase_windows():
    w = {}; 
    for m in markers():
        if m['event'] == 'begin': w.setdefault(m['phase'], [m['t'], None])[0] = m['t']
        elif m['event'] == 'end' and m['phase'] in w: w[m['phase']][1] = m['t']
    return {k: tuple(v) for k, v in w.items() if v[1]}
def sub_markers(phase):
    return [(m['event'], m['t']) for m in markers() if m['phase'] == phase]
def fmt(x, d=2, unit=''):
    if x is None or (isinstance(x, float) and (np.isnan(x) or np.isinf(x))): return '–'
    return f'{x:,.{d}f}{unit}'
def stats_row(a):
    a = np.asarray(a, float); a = a[np.isfinite(a)]
    if len(a) == 0: return dict(n=0)
    return dict(n=len(a), mean=a.mean(), std=a.std(), min=a.min(), p50=np.percentile(a, 50), p95=np.percentile(a, 95), p99=np.percentile(a, 99), max=a.max())
def table(headers, rows, align=None):
    L = ['| ' + ' | '.join(headers) + ' |', '|' + '|'.join(['---'] * len(headers)) + '|']
    for r in rows: L.append('| ' + ' | '.join(str(x) for x in r) + ' |')
    return '\n'.join(L) + '\n'
def kv_table(d, keys=None, headers=('Metric', 'Value')):
    keys = keys or list(d); return table(headers, [(k, d[k]) for k in keys])
def savefig(name):
    plt.tight_layout(); plt.savefig(f'{FIG}/{name}', dpi=130); plt.close(); return f'figures/{name}'
def inv(key):
    d = pj('inventory'); return (d or {}).get(key, {}).get('output', '') if d else ''
def slice_t(arrs, t0, t1, key='t'):
    t = arrs[key]; m = (t >= t0) & (t <= t1); return {k: v[m] for k, v in arrs.items()}
