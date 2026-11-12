from rb_common import *
def srow(d, keys=('mean', 'std', 'p50', 'p95', 'max'), digits=1):
    return [fmt(d.get(k), digits) if isinstance(d, dict) and d else '–' for k in keys]
def topic_table(dd, title_map=None):
    rows = []
    for tp, v in dd.items():
        if not isinstance(v, dict) or 'interval_ms' not in v: rows.append((f'`{tp}`', v.get('n', 0) if isinstance(v, dict) else '–', fmt(v.get('hz') if isinstance(v, dict) else None, 2), *(['–'] * 8))); continue
        i = v['interval_ms']; a = v['stamp_age_ms']; si = v['stamp_interval_ms']
        dev = a['p50'] > 1e9   # raw LiDAR stamps are in the sensor's own clock domain
        rows.append((f'`{tp}`', v['n'], fmt(v['hz'], 2), fmt(i['p50'], 0), fmt(i['p95'], 0), fmt(i['p99'], 0), fmt(i['max'], 0), v['gaps_over_2x_nominal'], 'device clock' if dev else fmt(a['p50'], 0), 'device clock' if dev else fmt(a['p95'], 0), fmt(v['msg_bytes_mean'] / 1e3, 0)))
    return table(['topic', 'msgs', 'rate Hz', 'interval p50 ms', 'p95 ms', 'p99 ms', 'max ms', 'gaps >2× median', 'stamp age p50 ms', 'p95 ms', 'msg kB'], rows)
def build_sensors():
    md = ['## 4.3 Sensor and point-cloud characterisation\n']; ph = pj('phase_sensor') or {}; res = {}
    L = ph.get('lidar') or {}
    md.append('### 4.3.1 LiDAR (Livox Avia, raw `/livox/lidar`)\n')
    if L and 'scans' in L:
        md.append(table(['quantity', 'value'], [('Scans captured / window', f"{L['scans']} in 60 s ⇒ **{L['rate_hz']:.2f} Hz received** (nominal 10 Hz)"), ('Points per message', f"{L['points_per_scan']['mean']:.0f} (std {L['points_per_scan']['std']:.0f}); valid (finite, non-zero) {L['valid_points_per_scan']['mean']:.0f} ± {L['valid_points_per_scan']['std']:.0f}"), ('Point throughput', f"{L['points_per_second_mean']:,.0f} points/s received (Avia nominal 240 000 points/s at 10 Hz)"),
            ('Range (m) mean ± std / p5–p95 / max', f"{L['range_m']['mean']:.2f} ± {L['range_m']['std']:.2f} / {L['range_m']['p5']:.2f}–{L['range_m']['p95']:.2f} / {L['range_m']['max']:.2f}"), ('Range distribution', ', '.join(f"{k} m: {v*100:.1f}%" for k, v in L['range_hist_m'].items())),
            ('Intensity mean / p5–p95 / max', f"{L['intensity']['mean']:.1f} / {L['intensity']['p5']:.0f}–{L['intensity']['p95']:.0f} / {L['intensity']['max']:.0f}"), ('Azimuth span', f"{L['azimuth_deg']['min']:.1f}° to {L['azimuth_deg']['max']:.1f}° (this scene; sensor FOV 70.4° H)"), ('Elevation span', f"{L['elevation_deg']['min']:.1f}° to {L['elevation_deg']['max']:.1f}° (sensor FOV 77.2° V)"),
            ('Points per scan line (6 lines)', ', '.join(f"line {k}: {v:,}" for k, v in L['points_by_line'].items())), ('Message inter-arrival ms (mean / p50 / p95 / max)', ' / '.join(srow(L['interarrival_ms'], ('mean', 'p50', 'p95', 'max'), 0))), ('Device-stamp interval ms (mean / p50 / p95 / max)', ' / '.join(srow(L['device_stamp_interval_ms'], ('mean', 'p50', 'p95', 'max'), 0))),
            ('Plane-fit precision (RANSAC, 3 cm band)', (f"RMS **{L['plane_fit']['rms_mm']['mean']:.1f} mm** (std {L['plane_fit']['rms_mm']['std']:.2f}, n={L['plane_fit']['n_planes']} planes), inlier fraction {L['plane_fit']['inlier_frac']['mean']*100:.0f}%" if L.get('plane_fit') else '–'))]))
        md.append('*Plane-fit RMS is an upper bound on LiDAR range precision: it includes real surface roughness and the RANSAC band choice. A gap between the median device-stamp interval (≈100 ms) and its mean means whole scans are missing before they reach the subscriber (see §4.4.2).*\n'); res['lidar'] = L
        if os.path.exists(f'{DATA}/sample_lidar_scan.npz'):
            z = np.load(f'{DATA}/sample_lidar_scan.npz'); x = z['xyz']; fig, ax = plt.subplots(1, 3, figsize=(13, 4)); sc = ax[0].scatter(x[:, 0], x[:, 1], c=z['intensity'], s=.3, cmap='viridis'); ax[0].set_title('top-down (x fwd, y left), colour = intensity'); ax[0].set_aspect('equal'); ax[0].set_xlabel('x m'); ax[0].set_ylabel('y m')
            ax[1].scatter(x[:, 0], x[:, 2], c=np.linalg.norm(x, axis=1), s=.3, cmap='plasma'); ax[1].set_title('side view, colour = range'); ax[1].set_aspect('equal'); ax[1].set_xlabel('x m'); ax[1].set_ylabel('z m'); ax[2].hist(np.linalg.norm(x, axis=1), bins=60, color=C[0]); ax[2].set_title('range histogram (one scan)'); ax[2].set_xlabel('m')
            md.append(f"![lidar scan]({savefig('lidar_scan.png')})\n")
    else: md.append('_LiDAR characterisation unavailable._\n')
    R = ph.get('rgb') or {}; md.append('### 4.3.2 RGB camera (`/landfill/rgb/image_raw`)\n')
    if R and 'frames' in R:
        md.append(table(['quantity', 'value'], [('Frames / rate', f"{R['frames']} in 30 s ⇒ **{R['rate_hz']:.2f} Hz** (configured 10 fps)"), ('Resolution / encoding / frame size', f"{R['resolution'][0]}×{R['resolution'][1]}, `{R['encoding']}`, {R['bytes_per_frame']/1e6:.2f} MB"), ('Mean luma (0–255)', f"{R['mean_luma']['mean']:.1f}"), ('Sharpness (variance of Laplacian)', f"{R['sharpness_laplacian_var']['mean']:.1f}"),
            ('Clipped pixels (≤2 or ≥253)', f"{R['clipped_pixel_frac']['mean']*100:.1f} %"), ('Frame-to-frame median abs difference (noise/motion)', f"{R['frame_diff_median_abs']['mean']:.2f} grey levels"), ('Channel means (R,G,B)', ', '.join(f'{v:.1f}' for v in (R['channel_means_bgr_or_rgb'] or []))),
            ('Inter-arrival ms (mean / p50 / p95 / max)', ' / '.join(srow(R['interarrival_ms'], ('mean', 'p50', 'p95', 'max'), 0))), ('Header-stamp interval ms (mean / p50 / p95 / max)', ' / '.join(srow(R['stamp_interval_ms'], ('mean', 'p50', 'p95', 'max'), 0))), ('Stamp age at arrival ms (mean / p50 / p95 / max)', ' / '.join(srow(R['stamp_age_at_arrival_ms'], ('mean', 'p50', 'p95', 'max'), 0)))]))
        md.append('*A stamp-interval p50 well below the mean (≈20–50 ms vs ≈100 ms) means frames are stamped in clustered pairs: the capture thread stamps at publish time, after a variable decode/convert delay, rather than at exposure.*\n'); res['rgb'] = R
        if os.path.exists(f'{HERE}/frames/rgb_sample.jpg'): md.append('![rgb sample](frames/rgb_sample.jpg)\n')
    T = ph.get('thermal') or {}; md.append('### 4.3.3 Thermal camera (`/landfill/thermal/image_raw`)\n')
    if T and T.get('frames'):
        md.append(table(['quantity', 'value'], [('Frames / rate', f"{T['frames']} in 20 s ⇒ **{T['rate_hz']:.2f} Hz** (requested 30)"), ('Resolution / encoding', f"{T['resolution'][0]}×{T['resolution'][1]}, `{T['encoding']}` (Y·256)"), ('8-bit dynamic range in scene', f"{T['dynamic_range_8bit']['min']:.0f}–{T['dynamic_range_8bit']['max']:.0f}, mean {T['dynamic_range_8bit']['mean']:.0f}, spatial std {T['dynamic_range_8bit']['std_spatial']:.1f}"), ('Temporal std per pixel p10 / median / p90 (counts)', f"{T['temporal_std_per_pixel']['p10']:.2f} / {T['temporal_std_per_pixel']['median']:.2f} / {T['temporal_std_per_pixel']['p90']:.2f} (p10 ≈ noise floor)"), ('Stuck 0/255 pixels', f"{T['stuck_pixel_frac']*100:.3f} %"), ('Unique-frame fraction', f"{T['unique_frame_frac']*100:.0f} %")]))
        res['thermal'] = T
        if os.path.exists(f'{HERE}/frames/thermal_sample.png'): md.append('![thermal sample](frames/thermal_sample.png)\n')
    else: md.append(f"**No thermal frames were received in the characterisation window** (`{T.get('note', 'no data')}`). Camera state in this campaign: see §4.11. All thermal-dependent results (3-way fusion, thermal coverage) are therefore reported as *LiDAR+RGB fallback* where applicable.\n"); res['thermal'] = T
    F = ph.get('fused') or {}; md.append('### 4.3.4 Fused cloud (`/landfill/fusion/points_rgbt`)\n')
    if F and 'fused_clouds' in F:
        rows = [('Fused clouds / rate', f"{F['fused_clouds']} in 60 s ⇒ **{F['fused_rate_hz']:.2f} Hz**"), ('Inputs seen by this subscriber (Hz)', f"LiDAR(restamped) {F['lidar_in_rate_hz']:.2f}, RGB {F['rgb_rate_hz']:.2f}, thermal {F['thermal_rate_hz']:.2f}"), ('Fused clouds per LiDAR scan', f"{F['fused_per_lidar_scan']:.2f} (>1 only because this subscriber loses some LiDAR messages)"), ('Points per fused cloud', f"{F['points_per_fused_cloud']['mean']:.0f}"),
            ('Points with a valid RGB sample', f"{F['valid_rgb_frac']['mean']*100:.1f} % (std {F['valid_rgb_frac']['std']*100:.2f})"), ('Points with a valid thermal sample', f"{F['valid_thermal_frac']['mean']*100:.1f} %"), ('Points with both', f"{F['both_valid_frac']['mean']*100:.1f} %"), ('Mean point range in fused cloud', f"{F['mean_range_m']['mean']:.2f} m")]
        for k, lab in (('dt_fused_to_nearest_rgb_ms', 'Nearest RGB frame to each fused scan (ms)'), ('dt_fused_to_nearest_thermal_ms', 'Nearest thermal frame to each fused scan (ms)')):
            if F.get(k): rows.append((lab, f"mean {F[k]['mean']:.0f}, p50 {F[k]['p50']:.0f}, p95 {F[k]['p95']:.0f}, max {F[k]['max']:.0f}"))
        md.append(table(['quantity', 'value'], rows)); md.append('*"Valid" means the LiDAR point projects inside the camera image, so it measures field-of-view overlap, not colour accuracy.*\n'); res['fused'] = F
        if os.path.exists(f'{DATA}/sample_fused_cloud.npz'):
            z = np.load(f'{DATA}/sample_fused_cloud.npz'); x = z['xyz']; rgb = z['rgb']; col = np.stack([(rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255], 1) / 255.0; th = z['thermal']
            fig, ax = plt.subplots(1, 3, figsize=(14, 4)); ax[0].scatter(x[:, 1], x[:, 2], c=col, s=.6); ax[0].set_title('fused cloud, RGB colour (front view y/z)'); ax[0].set_aspect('equal'); ax[1].scatter(x[:, 1], x[:, 2], c=(th if np.isfinite(th).any() else 'lightgrey'), s=.6, cmap='inferno'); ax[1].set_title('thermal value (NaN = none)' if np.isfinite(th).any() else 'thermal: no data (camera not streaming)'); ax[1].set_aspect('equal'); ax[2].scatter(x[:, 0], x[:, 1], c=col, s=.6); ax[2].set_title('RGB colour, top-down'); ax[2].set_aspect('equal'); md.append(f"![fused cloud]({savefig('fused_cloud.png')})\n")
    M = ph.get('merged') or {}; md.append('### 4.3.5 Merged cloud (`/landfill/fusion/points_merged`)\n')
    if M and 'merged_clouds' in M:
        rows = [('Merged clouds / rate', f"{M['merged_clouds']} in 45 s ⇒ **{M['merged_rate_hz']:.2f} Hz**"), ('Points per merged cloud', f"mean {M['points_per_merged_cloud']['mean']:.0f}, p5–p95 {M['points_per_merged_cloud']['p5']:.0f}–{M['points_per_merged_cloud']['p95']:.0f}, max {M['points_per_merged_cloud']['max']:.0f}"), ('Bytes per merged cloud', f"{M['bytes_per_merged_cloud']['mean']/1e6:.2f} MB"), ('Inter-arrival s (mean / p50 / p95)', ' / '.join(srow(M['interarrival_s'], ('mean', 'p50', 'p95'), 2))), ('Stamp age at arrival s (mean / p50 / p95)', ' / '.join(srow(M['stamp_age_at_arrival_s'], ('mean', 'p50', 'p95'), 3))), ('Scene extent (x,y,z m)', ', '.join(f'{v:.2f}' for v in M.get('extent_m', []))), ('Wire quantisation step at this extent', f"{M.get('quantisation_step_mm_at_this_extent', float('nan')):.3f} mm")]
        if M.get('consecutive_merged_clouds_icp'): g = M['consecutive_merged_clouds_icp']; rows.append(('Residual motion between consecutive merged clouds (ICP, static rig ⇒ ~0)', f"translation {g['translation_mm']['mean']:.1f} mm mean (p95 {g['translation_mm']['p95']:.1f}, max {g['translation_mm']['max']:.1f}); rotation {g['rotation_deg']['mean']:.3f}° mean (max {g['rotation_deg']['max']:.3f}); converged {g['converged_frac']*100:.0f}% of {g['n']}"))
        md.append(table(['quantity', 'value'], rows)); res['merged'] = M
        if os.path.exists(f'{DATA}/sample_merged_cloud.npz'):
            z = np.load(f'{DATA}/sample_merged_cloud.npz'); x = z['xyz']; rgb = z['rgb']; col = np.stack([(rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255], 1) / 255.0; fig, ax = plt.subplots(1, 3, figsize=(14, 4)); ax[0].scatter(x[:, 1], x[:, 2], c=col, s=.2); ax[0].set_aspect('equal'); ax[0].set_title('merged, front view'); ax[1].scatter(x[:, 0], x[:, 1], c=col, s=.2); ax[1].set_aspect('equal'); ax[1].set_title('merged, top-down'); ax[2].scatter(x[:, 0], x[:, 2], c=(z['thermal'] if np.isfinite(z['thermal']).any() else 'lightgrey'), s=.2, cmap='inferno'); ax[2].set_aspect('equal'); ax[2].set_title('merged, side, thermal' if np.isfinite(z['thermal']).any() else 'merged side (thermal: no data)'); md.append(f"![merged cloud]({savefig('merged_cloud.png')})\n")
    return '\n'.join(md), res
def build_pipeline():
    md = ['## 4.4 Pipeline performance: rates, jitter, latency, CPU, memory and threads\n']; res = {}; ip = pj('phase_idle_pipeline') or {}; lt = pj('phase_latency') or {}
    md.append('### 4.4.1 Topic rates and jitter (steady state, no viewer)\n')
    allt = {}
    for k in ('topics_group_lidar', 'topics_rgb', 'topics_thermal', 'topics_fused_merged'): allt.update(ip.get(k) or {})
    md.append('Measured by independent subscriber processes that load the **same Fast DDS profile as the pipeline** (one process per topic group so a slow subscriber cannot lose messages for another). *Stamp age* = receive time − message header stamp (pipeline latency up to that topic).\n')
    md.append(topic_table(allt)); res['topics'] = allt
    if lt:
        md.append('**Repeat measurement used for the jitter histograms (60 s each, `latency` phase):**\n'); l2 = {}
        for k in lt: l2.update(lt[k])
        md.append(topic_table(l2)); res['latency_topics'] = l2
        arr = {}
        for fn in glob.glob(f'{DATA}/stamps_*.npz'):
            z = np.load(fn)
            for kk in z.files:
                if kk.endswith('_arr'): arr[kk[:-4]] = (z[kk], z[kk[:-4] + '_stamp'])
        if arr:
            nm = {'livox_lidar': 'LiDAR raw', 'landfill_livox_points': 'LiDAR restamped', 'landfill_rgb_image_raw': 'RGB', 'landfill_thermal_image_raw': 'thermal', 'landfill_fusion_points_rgbt': 'fused', 'landfill_fusion_points_merged': 'merged'}
            keys = [k for k in nm if k in arr and len(arr[k][0]) > 5]
            fig, ax = plt.subplots(2, 3, figsize=(13, 6)); 
            for i, k in enumerate(keys[:6]):
                a, s = arr[k]; d = np.diff(a) * 1000; x = ax.flat[i]; x.hist(d, bins=np.linspace(0, min(d.max(), np.percentile(d, 99.5) * 1.2), 70), color=C[i % len(C)]); x.set_title(f'{nm[k]}  ({len(a)/(a[-1]-a[0]):.2f} Hz)'); x.set_xlabel('inter-arrival ms'); x.set_yscale('log')
            md.append(f"![interarrival]({savefig('interarrival_hist.png')})\n")
            fig, ax = plt.subplots(figsize=(11, 3.5))
            for i, k in enumerate(keys):
                a, s = arr[k]; bins = np.arange(0, a[-1] - a[0] + 1, 1.0); h, _ = np.histogram(a - a[0], bins=bins); ax.plot(bins[:-1], h, lw=.8, label=nm[k], c=C[i % len(C)])
            ax.set_xlabel('s'); ax.set_ylabel('messages per second'); ax.legend(ncol=3, fontsize=7); ax.set_title('Per-second message count (rate stability)'); md.append(f"![rate stability]({savefig('rate_stability.png')})\n")
            fig, ax = plt.subplots(figsize=(8, 3.5)); ages = [((arr[k][0] - arr[k][1]) * 1000, nm[k]) for k in keys if k != 'livox_lidar']; ax.boxplot([a for a, _ in ages], labels=[n for _, n in ages], showfliers=False); ax.set_ylabel('stamp age at arrival (ms)'); ax.set_title('Latency from header stamp to subscriber'); md.append(f"![latency]({savefig('latency_box.png')})\n")
    # LiDAR loss probes
    ll = ip.get('lidar_loss') or []
    if ll and 'rx_rate_hz' in ll[0]:
        md.append('### 4.4.2 LiDAR scan loss (device-stamp analysis, 3 × 25 s)\n'); md.append(table(['run', 'rx rate Hz', 'scans missing %', 'stamp gaps >150 ms', 'max gap ms', 'UDP datagrams', 'UDP rcvbuf errors'], [(i + 1, fmt(d['rx_rate_hz'], 2), fmt(d['scans_missing_pct'], 1), d['stamp_gap_over_150ms'], fmt(d['stamp_gap_max_ms'], 0), f"{d['udp_datagrams']:,}", d['udp_rcvbuf_errors']) for i, d in enumerate(ll)]))
        md.append('*Scans missing = 1 − received ÷ (device-stamp span ÷ 100 ms). The LiDAR counter ticks at exactly 100 ms per scan, so gaps in the device stamps identify scans that never reached the subscriber.*\n'); res['lidar_loss'] = ll
    # CPU per node from system_1hz
    S = load_csv(f'{DATA}/system_1hz.csv'); sm = dict(sub_markers('idle_pipeline')); W = phase_windows()
    if S and 'steady_begin' in sm and 'steady_end' in sm:
        a, b = sm['steady_begin'], sm['steady_end']; m = (S['t'] >= a) & (S['t'] <= b)
        names = ['livox_driver', 'restamp', 'rgb_camera', 'thermal_camera', 'calibration', 'fusion', 'merge', 'streamer', 'rpicam_vid']; rows = []; tot = 0; res['cpu'] = {}
        for n in names:
            c = S.get(f'{n}_cpu'); r = S.get(f'{n}_rss_mb'); th = S.get(f'{n}_threads')
            if c is None: continue
            cm = c[m]; rm = r[m]; tm = th[m]; res['cpu'][n] = dict(cpu_mean=np.nanmean(cm), cpu_p95=np.nanpercentile(cm, 95), cpu_max=np.nanmax(cm), rss=np.nanmean(rm), threads=np.nanmax(tm)); tot += np.nanmean(cm)
            rows.append((n, fmt(np.nanmean(cm), 1), fmt(np.nanpercentile(cm, 95), 1), fmt(np.nanmax(cm), 1), fmt(np.nanmean(rm), 0), fmt(np.nanmax(tm), 0)))
        rows.append(('**pipeline total**', fmt(tot, 1), '', '', fmt(sum(res['cpu'][n]['rss'] for n in res['cpu']), 0), ''))
        md.append('### 4.4.3 CPU, memory and threads per process (steady state, no viewer; 1 Hz samples)\n'); md.append('CPU is percent of one core (the Pi has 4, so 400 % is the ceiling).\n'); md.append(table(['process', 'CPU mean %', 'CPU p95 %', 'CPU max %', 'RSS MB', 'threads'], rows))
        md.append(f"System-wide: total CPU **{np.nanmean(S['cpu_total_pct'][m]):.0f} % of 400** (per-core mean {', '.join(f'{np.nanmean(S[f'cpu_c{i}'][m]):.0f}' for i in range(4))} %), load average {np.nanmean(S['load1'][m]):.2f}, memory used {np.nanmean(S['mem_used_mb'][m]):.0f} MB (available {np.nanmean(S['mem_avail_mb'][m]):.0f} MB), `/dev/shm` {np.nanmean(S['shm_mb'][m]):.0f} MB, CPU clock {np.nanmean(S['arm_clock_mhz'][m]):.0f} MHz. Non-pipeline processes (IDE, desktop, system services) used {np.nanmean(S['other_cpu_pct'][m]):.0f} % of one core on average (measurement overhead of this campaign's own samplers is included here).\n")
        fig, ax = plt.subplots(1, 2, figsize=(12, 3.8)); ax[0].barh([r[0] for r in rows[:-1]], [float(res['cpu'][n]['cpu_mean']) for n in res['cpu']], color=C[0]); ax[0].set_xlabel('CPU % of one core'); ax[0].set_title('Per-process CPU (idle pipeline)'); ax[1].barh([r[0] for r in rows[:-1]], [res['cpu'][n]['rss'] for n in res['cpu']], color=C[2]); ax[1].set_xlabel('RSS MB'); ax[1].set_title('Per-process memory'); md.append(f"![cpu per node]({savefig('cpu_per_node.png')})\n")
        if 'idle_os' in W:
            a2, b2 = [x for e, x in sub_markers('idle_os') if e == 'steady_begin'][0], [x for e, x in sub_markers('idle_os') if e == 'steady_end'][0]; m2 = (S['t'] >= a2) & (S['t'] <= b2)
            md.append(f"**OS baseline (pipeline stopped):** total CPU {np.nanmean(S['cpu_total_pct'][m2]):.0f} % of 400, memory used {np.nanmean(S['mem_used_mb'][m2]):.0f} MB, `/dev/shm` {np.nanmean(S['shm_mb'][m2]):.0f} MB. **Pipeline adds ≈ {np.nanmean(S['cpu_total_pct'][m]) - np.nanmean(S['cpu_total_pct'][m2]):.0f} % of one core and ≈ {np.nanmean(S['mem_used_mb'][m]) - np.nanmean(S['mem_used_mb'][m2]):.0f} MB.**\n")
    # threads
    th = pj('phase_threads')
    if th and th.get('runs') and 'processes' in th['runs'][0]:
        md.append('### 4.4.4 Per-thread CPU accounting (2 × 60 s, from `/proc/<pid>/task`)\n'); P = th['runs'][0]['processes']; rows = []
        for n, v in P.items():
            b = v['busiest_thread']; rows.append((n, v['pid'], v['n_threads'], fmt(v['total_cpu_pct'], 1), f"{b['name']} (tid {b['tid']})", fmt(b['user_pct'], 1), fmt(b['sys_pct'], 1), fmt(b['vol_ctx_per_s'], 0), fmt(b['invol_ctx_per_s'], 0)))
        md.append(table(['process', 'pid', 'threads', 'total CPU %', 'busiest thread', 'its user %', 'its sys %', 'vol. ctx sw/s', 'invol. ctx sw/s'], rows))
        md.append('**Top threads overall (run 1):**\n'); allth = [(n, t) for n, v in P.items() for t in v['threads']]; allth.sort(key=lambda x: -x[1]['total_pct']); md.append(table(['process', 'thread name', 'tid', 'total %', 'user %', 'sys %', 'last core', 'vol ctx/s', 'invol ctx/s'], [(n, t['name'], t['tid'], fmt(t['total_pct'], 1), fmt(t['user_pct'], 1), fmt(t['sys_pct'], 1), t['last_core'], fmt(t['vol_ctx_per_s'], 0), fmt(t['invol_ctx_per_s'], 0)) for n, t in allth[:20]]))
        md.append('*Reading guide:* the busiest thread of each Python node is its executor/callback thread; `merge` additionally shows the ICP worker thread; the streamer shows per-request threads only while a client is connected. Because each node is a separate process the GIL never couples nodes. Voluntary context switches are the thread blocking on I/O or a lock; involuntary ones are pre-emptions by other runnable threads (CPU contention).\n'); res['threads'] = P
        fig, ax = plt.subplots(figsize=(10, 4)); names = list(P); data = [sorted([t['total_pct'] for t in P[n]['threads']], reverse=True)[:6] + [0] * 6 for n in names]; bot = np.zeros(len(names))
        for j in range(6): v = np.array([d[j] for d in data]); ax.bar(names, v, bottom=bot, color=plt.cm.tab10(j), label=f'thread #{j+1}'); bot += v
        ax.set_ylabel('CPU % of one core'); ax.legend(fontsize=7, ncol=3); ax.tick_params(axis='x', rotation=40); ax.set_title('Per-process CPU split over its 6 busiest threads'); md.append(f"![threads]({savefig('threads.png')})\n")
    # startup
    su = pj('phase_startup'); rt = pj('phase_restart_timing')
    if su and 'first_message_s_after_start' in su:
        md.append('### 4.4.5 Start-up and restart timing\n'); fm = su['first_message_s_after_start']; cn = su.get('message_counts_100s') or {}; rows = [(k, fmt(v, 1), cn.get(k, '–')) for k, v in sorted(fm.items(), key=lambda kv: kv[1])]
        md.append(f"**Cold service start** (service `start` issued at t=0; probe subscribed beforehand): first message per topic —\n"); md.append(table(['topic', 'first message (s after start)', 'messages in the 100 s probe'], rows)); md.append(f"Viewer first returned a non-empty merged cloud **{fmt(su.get('viewer_first_merged_data_s'), 1)} s** after start; {su.get('nodes_up')} pipeline processes up. Thermal camera at that point: {su.get('camera_after')}.\n"); res['startup'] = su
    if rt and rt.get('runs'):
        fv = lambda d, k: ('n/a' if (d.get(k) is None or d.get(k) < 0.5) else fmt(d.get(k), 1))
        md.append('**Graceful restarts (`systemctl --user restart`, 3 runs):**\n'); md.append(table(['run', 'viewer data (s)', 'first LiDAR (s)', 'first RGB (s)', 'first fused (s)', 'first merged (s)', 'nodes up'], [(r['run'], fmt(r['viewer_data_s'], 1), fv(r['first_message_s'], '/livox/lidar'), fv(r['first_message_s'], '/landfill/rgb/image_raw'), fv(r['first_message_s'], '/landfill/fusion/points_rgbt'), fv(r['first_message_s'], '/landfill/fusion/points_merged'), r['nodes_up']) for r in rt['runs']])); md.append('*In a restart the probe is subscribed to the still-running old pipeline, so per-topic first-message times that come from the outgoing processes are shown as n/a; the **viewer-data time** (first non-empty merged cloud served over HTTP) is the reliable restart figure.*\n'); res['restart'] = rt
    return '\n'.join(md), res
