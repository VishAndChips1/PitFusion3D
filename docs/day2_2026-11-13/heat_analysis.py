import numpy as np, time, json, csv
def ts(s): return time.mktime(time.strptime(s[:19], '%Y-%m-%d %H:%M:%S'))
ev = sorted(ts(l) for l in open('/home/vish/power_logs/usb_events.log') if 'Thermal' in l and 'DISCONNECTED' in l)
rows = list(csv.reader(open('data/soc_temp_1hz.csv'))); T = np.array([ts(r[0]) for r in rows]); C = np.array([float(r[1]) for r in rows])
# drop the stale-clock stretch right after reboots (times that go backwards)
ok = np.concatenate([[True], np.diff(T) > 0]); T, C = T[ok], C[ok]
# episodes: runs of disconnects with gaps < 150 s and at least 4 events
ep = []; cur = [ev[0]]
for t in ev[1:]:
    if t - cur[-1] < 150: cur.append(t)
    else:
        if len(cur) >= 4: ep.append(cur)
        cur = [t]
if len(cur) >= 4: ep.append(cur)
def temp_at(t0, t1):
    m = (T >= t0) & (T <= t1); return C[m] if m.any() else np.array([np.nan])
out = []
for e in ep:
    a, b = e[0], e[-1]; pre = temp_at(a - 300, a); dur = temp_at(a, b)
    out.append(dict(start=time.strftime('%d %b %H:%M', time.localtime(a)), minutes=round((b - a) / 60, 1), n=len(e), soc_before_C=round(float(np.nanmean(pre)), 1), soc_during_mean_C=round(float(np.nanmean(dur)), 1), soc_during_max_C=round(float(np.nanmax(dur)), 1)))
print(f'{len(ev)} camera disconnects, {len(ep)} stuck episodes'); 
for o in out: print(o)
# temperature of the SoC while the camera is known to stream fine (hand-labelled from today's/yesterday's tests)
good = [('2026-11-11 16:53:40', '2026-11-11 17:48:30'), ('2026-11-12 14:19:10', '2026-11-12 14:26:45'), ('2026-11-12 20:24:30', '2026-11-12 22:06:00')]
gt = np.concatenate([temp_at(ts(a), ts(b)) for a, b in good]); st = np.concatenate([temp_at(e[0], e[-1]) for e in ep])
from scipy.stats import mannwhitneyu
print('SoC temp while streaming fine: mean %.1f, p5-p95 %.1f-%.1f, max %.1f (n=%d)' % (gt.mean(), *np.percentile(gt, [5, 95]), gt.max(), len(gt)))
print('SoC temp during stuck episodes: mean %.1f, p5-p95 %.1f-%.1f, max %.1f (n=%d)' % (st.mean(), *np.percentile(st, [5, 95]), st.max(), len(st)))
print('Mann-Whitney p =', mannwhitneyu(gt, st).pvalue)
json.dump(dict(episodes=out, good_mean=float(gt.mean()), stuck_mean=float(st.mean()), n_events=len(ev)), open('data/heat_analysis.json', 'w'), indent=1)
