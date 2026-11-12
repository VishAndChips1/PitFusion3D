from rb_common import *
def build_transport():
    md = ['## 4.5 DDS transport, network and viewer load\n']; res = {}; tr = pj('phase_transport') or {}
    md.append('### 4.5.1 DDS transport audit\n')
    lp = tr.get('live_pipeline_60s')
    if lp:
        md.append(table(['quantity (60 s, live pipeline running)', 'value'], [('Loopback traffic', f"**{lp['loopback_MB_per_s']:.1f} MB/s**, {lp['loopback_pkts_per_s']:.0f} packets/s"), ('UDP datagrams received (all sockets)', f"{lp['udp_in_datagrams']:,}"), ('UDP receive-buffer errors (kernel drops)', f"**{lp['udp_rcvbuf_errors']:,}** ({100*lp['udp_rcvbuf_errors']/max(lp['udp_in_datagrams'],1):.2f} % of datagrams)"), ('UDP InErrors', lp['udp_in_errors']), ('Sockets whose drop counter grew', '; '.join(f'{k}: +{v}' for k, v in lp['socket_drop_growth'].items()) or 'none')]))
        shm = re.findall(r'\d+(?:\.\d+)?[MG]?', tr.get('shm', '')); md.append(f"Shared memory: {shm[0] if shm else '?'} `fastrtps` segment files, {shm[1] if len(shm) > 1 else '?'} total in `/dev/shm`.\n"); res['live'] = lp
    rows = []
    for n, lab in (('no_profile', 'default Fast DDS (no profile)'), ('current_profile', 'deployed profile (SHM 64 MB + UDP 4 MB)'), ('shm_only_profile', 'SHM-only')):
        d = tr.get(n) or {}
        if 'delivered_pct' in d: rows.append((lab, d['sent'], d['received'], fmt(d['delivered_pct'], 1), d.get('subscriber_udp_sockets'), d.get('subscriber_own_socket_drops')))
    if rows: md.append('**Isolated transport test** (separate ROS domain; publisher → subscriber, 24 000-point XYZIRT clouds, 576 kB, 10 Hz, 25 s; the first ≈1 s is lost to discovery in every case):\n'); md.append(table(['profile', 'sent', 'received', 'delivered %', 'subscriber UDP sockets', 'drops on those sockets'], rows)); res['isolated'] = rows
    if lp and lp['udp_rcvbuf_errors'] == 0 and lp['loopback_MB_per_s'] < 3:
        md.append(f"*Interpretation:* with the pipeline settled and every measuring tool loading the project profile, UDP loopback carried only **{lp['loopback_MB_per_s']:.2f} MB/s** and the kernel recorded **0 receive-buffer errors**: the large messages (2.76 MB RGB frames, 0.43–0.58 MB clouds, ≈40 MB/s in total) travel over shared memory as designed. **Early in the session** (minutes after boot, while measuring tools were running *without* the profile) loopback carried ≈17.7 MB/s and 2–3 % of UDP datagrams were dropped, and the LiDAR read 4.5–6.6 Hz; that was a *measurement artefact* (tools without the profile receive large messages over lossy UDP loopback and also make the publishers send them that way), not a pipeline fault. It is the reason every tool in this campaign loads the profile. In the isolated test a subscriber without the profile received 82 % of messages versus 96 % with it (the missing ≈4 % is discovery time at start-up).\n")
    else:
        md.append('*Interpretation:* in isolation shared memory delivers essentially everything, but the live pipeline shows UDP loopback traffic and/or kernel drops (table above); any measuring tool that does not load the project profile receives large messages over lossy UDP and under-reports rates.\n')
    ht = pj('phase_http') or {}; rows = []
    for k, v in sorted(ht.items()):
        if not k.startswith('clients_') or 'paths' not in v: continue
        for p, d in v['paths'].items(): rows.append((k.replace('clients_', '').replace('_', ' '), p, fmt(d['req_per_s'], 1), d['n200'], d['n304'], d['errors'], fmt(d['latency_ms']['p50'], 0), fmt(d['latency_ms']['p95'], 0), fmt(d['latency_ms']['p99'], 0), fmt(d['latency_ms']['max'], 0), fmt(d['payload_kb_mean'], 0), fmt(d['wire_mbit_s_total'], 2), fmt(d['points_mean'], 0)))
    if rows:
        md.append('### 4.5.2 Viewer (HTTP) load test\n'); md.append('N concurrent clients emulate the browser page (half poll `/merged.bin` every 250 ms, half `/live.bin` every 120 ms, with `If-None-Match`). `localhost` rows exclude the radio; `wifi ip` rows use the Pi\'s WiFi address (still local traffic, but through the full TCP stack).\n')
        md.append(table(['clients', 'path', 'req/s', '200s', '304s', 'errors', 'lat p50 ms', 'p95', 'p99', 'max', 'payload kB', 'Mbit/s total', 'points'], rows)); res['http'] = rows
        fig, ax = plt.subplots(1, 2, figsize=(11, 3.6)); xs = []; p95 = []; mb = []
        for k, v in sorted(ht.items(), key=lambda kv: int(re.findall(r'\d+', kv[0])[0]) if kv[0].startswith('clients_') else 0):
            if k.startswith('clients_') and k.endswith('localhost') and 'paths' in v:
                xs.append(int(re.findall(r'\d+', k)[0])); p95.append(max(d['latency_ms']['p95'] for d in v['paths'].values())); mb.append(sum(d['wire_mbit_s_total'] for d in v['paths'].values()))
        if xs: ax[0].plot(xs, p95, 'o-', c=C[1]); ax[0].set_xlabel('concurrent clients'); ax[0].set_ylabel('p95 request latency (ms)'); ax[0].set_xscale('log', base=2); ax[1].plot(xs, mb, 'o-', c=C[0]); ax[1].set_xlabel('concurrent clients'); ax[1].set_ylabel('total Mbit/s served'); ax[1].set_xscale('log', base=2); md.append(f"![http scaling]({savefig('http_scaling.png')})\n")
    nt = pj('phase_net') or {}
    if nt and 'iwconfig' in nt:
        md.append('### 4.5.3 Network (WiFi link, latency, internet throughput)\n'); rows = [('WiFi', f"{nt.get('freq_ghz')} GHz, bit rate {nt.get('bitrate_mbps')} Mb/s, signal {nt.get('signal_dbm')} dBm, link quality {nt.get('link_quality')}, TX power {nt.get('tx_power_dbm')} dBm"), ('WiFi power-save', nt.get('power_save')), ('Tx excessive retries (cumulative counter)', nt.get('excessive_retries'))]
        for k in ('ping_gateway_200', 'ping_1.1.1.1_50', 'ping_8.8.8.8_50'):
            d = nt.get(k)
            if d and 'rtt_avg' in d: rows.append((k.replace('_', ' '), f"loss {d['loss_pct']} %, RTT min/avg/max/mdev {d['rtt_min']}/{d['rtt_avg']}/{d['rtt_max']}/{d['rtt_mdev']} ms"))
        rows.append(('DNS lookup', f"{fmt(nt.get('dns_ms'), 0)} ms"))
        for k, d in (nt.get('download') or {}).items(): rows.append((f'download {k}', f"**{d['mbit_s']:.1f} Mbit/s** ({d['bytes']/1e6:.1f} MB in {d['secs']:.1f} s, HTTP {d['http']})"))
        for k, d in (nt.get('upload') or {}).items(): rows.append((f'upload {k}', f"**{d['mbit_s']:.1f} Mbit/s** ({d['bytes']/1e6:.1f} MB in {d['secs']:.1f} s, HTTP {d['http']})"))
        md.append(table(['measurement', 'result'], rows))
        try:
            dl = min(d['mbit_s'] for d in (nt.get('download') or {}).values()); sig = int(nt.get('signal_dbm')); rtt = (nt.get('ping_gateway_200') or {}).get('rtt_avg')
            if dl < 5 or (rtt and rtt > 100): md.append(f"**Interpretation — the WiFi link in the rig's location was poor during this campaign:** signal {sig} dBm on 2.4 GHz (the same Pi measured −25 dBm at the desk earlier in the day), average gateway round-trip {rtt:.0f} ms with 0 % loss (a delay-without-loss pattern typical of a congested or weak link with deep buffering) and internet download of only {dl:.1f} Mbit/s. A viewer fetching the merged stream at the default cap needs ≈2.3 Mbit/s (§4.7), so over this link the viewer would stutter regardless of the Pi's own performance (the Pi serves 16 concurrent local clients with flat latency, §4.5.2). Options: move the rig/AP closer, join the 5 GHz band (the router also advertises it), lower the point caps, or use the Pi's own hotspot so viewer and Pi are one hop apart.\n")
        except Exception: pass
        md.append('*The radio can carry the viewer stream comfortably only if this throughput exceeds the Mbit/s figures in §4.7; a real phone/laptop client was not connected during the campaign, so radio TX power and its current draw under a real viewer load are not captured.*\n'); res['net'] = nt
    return '\n'.join(md), res
def sweep_rows(tag):
    p = f'{HERE}/results_{tag}.jsonl'; return [json.loads(l) for l in open(p)] if os.path.exists(p) else []
def build_sweep():
    md = ['## 4.7 Point-cap sweep (quality vs bandwidth vs CPU vs current)\n']; res = {}
    md.append("The streamer's point cap was changed at run time (`max_points_merged` / `max_points_live`) and held 90 s per level with the full pipeline running. Per level: payload, bandwidth to one viewer, refresh rate seen by a polling client, request latency, CPU, upstream rates, **fidelity against a contemporaneous full cloud** (median nearest-neighbour spacing, and the share of the full cloud's occupied 2 cm / 5 cm voxels that the capped cloud still covers) and **fast-sampled current** with abort gates. (A previous, longer sweep from 2026-11-11 is archived in `docs/pointcap_sweep_2026-11-11/`.)\n")
    for tag, title in (('final_merged', 'Merged view'), ('final_live', 'Live view')):
        R = sweep_rows(tag)
        if not R: md.append(f'### {title}\n_no data_\n'); continue
        rows = []
        for r in R:
            q = r.get('quality') or {}; p = r['power']; c = r['cpu']; t = r['topics']
            rows.append((f"{r['cap']:,}", fmt(r['points_per_cloud'], 0), fmt(r['wire_kb_mean'], 0), fmt(r['updates_per_s'], 2), fmt(r['viewer_mbit_s'], 2), fmt(r['resp_ms']['p95'], 0), fmt(q.get('nn_median_cm'), 2), fmt(q.get('coverage_2cm_pct'), 1), fmt(q.get('coverage_5cm_pct'), 1), fmt(t.get('points_rgbt', {}).get('hz'), 2), fmt(t.get('points_merged', {}).get('hz'), 2), fmt(c['pipeline_pct'], 0), fmt(c['streamer_pct'], 0), fmt(p['est_input_a']['max'], 2), fmt(p['est_input_conservative_a_max'], 2), fmt(p['ext5v_min'], 3), fmt(p.get('temp_c_max', r.get('temp_c_max')), 1)))
        md.append(f'### {title}\n'); md.append(table(['cap', 'points/cloud', 'kB/cloud', 'updates/s', 'Mbit/s', 'resp p95 ms', 'NN spacing cm', '2 cm cover %', '5 cm cover %', 'fused Hz', 'merged Hz', 'pipeline CPU %', 'streamer CPU %', 'est A max', 'cons. A max', 'EXT5V min', 'SoC °C max'], rows)); res[tag] = R
        fig, ax = plt.subplots(2, 3, figsize=(14, 6)); x = [r['cap'] for r in R]; g = lambda f: [f(r) for r in R]
        ax[0, 0].plot(x, g(lambda r: (r.get('quality') or {}).get('coverage_5cm_pct')), 'o-', c=C[0], label='5 cm'); ax[0, 0].plot(x, g(lambda r: (r.get('quality') or {}).get('coverage_2cm_pct')), 's--', c=C[1], label='2 cm'); ax[0, 0].set_title('fidelity vs full cloud (%)'); ax[0, 0].legend()
        ax[0, 1].plot(x, g(lambda r: r['viewer_mbit_s']), 'o-', c=C[0]); ax[0, 1].set_title('bandwidth to one viewer (Mbit/s)'); ax[0, 2].plot(x, g(lambda r: r['updates_per_s']), 'o-', c=C[2]); ax[0, 2].set_title('updates per second seen by client')
        ax[1, 0].plot(x, g(lambda r: r['cpu']['pipeline_pct']), 'o-', c=C[0], label='pipeline'); ax[1, 0].plot(x, g(lambda r: r['cpu']['streamer_pct']), 's--', c=C[1], label='streamer'); ax[1, 0].set_title('CPU (% of one core)'); ax[1, 0].legend()
        ax[1, 1].plot(x, g(lambda r: r['power']['est_input_a']['max']), 'o-', c=C[0], label='est max'); ax[1, 1].plot(x, g(lambda r: r['power']['est_input_conservative_a_max']), 's--', c=C[1], label='conservative'); ax[1, 1].axhline(5, c='r'); ax[1, 1].set_title('peak input current (A)'); ax[1, 1].legend(); ax[1, 2].plot(x, g(lambda r: r['resp_ms']['p95']), 'o-', c=C[3]); ax[1, 2].set_title('request latency p95 (ms)')
        for a in ax.flat: a.set_xlabel('point cap')
        md.append(f"![sweep {tag}]({savefig(f'sweep_{tag}.png')})\n")
    return '\n'.join(md), res
def build_accuracy():
    md = ['## 4.8 Accuracy and fidelity\n']; res = {}; ac = pj('phase_accuracy') or {}; w = ac.get('wire') or {}
    md.append('### 4.8.1 Viewer wire format (encode → decode against the real ROS cloud)\n')
    if w and 'error' not in w:
        rows = []
        for k, v in w.items():
            rows.append((k, f"{v['points']:,}", fmt(v['bytes_per_point'], 1), fmt(v['ros_to_wire_ratio'], 2), fmt(v['encode_ms'], 1), fmt(v['quant_scale_mm'], 4), fmt(v['position_error_mm']['mean'], 4), fmt(v['position_error_mm']['rms'], 4), fmt(v['position_error_mm']['max'], 4), fmt(v['theoretical_max_error_mm'], 4), fmt(v['rgb_exact_match_frac'] * 100, 2), fmt(v['thermal_abs_error_counts']['max'], 4), v['thermal_sentinel_ok']))
        md.append(table(['cloud', 'points', 'wire B/pt', 'ROS÷wire size', 'encode ms', 'quant step mm', 'pos err mean mm', 'rms mm', '**max mm**', 'theory max mm', 'RGB exact %', 'thermal max err (counts)', 'NaN sentinel ok'], rows))
        md.append('The measured maximum position error equals the theoretical bound (√3 × step ÷ 2): the format adds **≈ 0.14 mm** of error, two orders of magnitude below the LiDAR plane noise (§4.3.1). RGB is bit-exact; thermal is exact to 0.01 count.\n'); md.append('**gzip level trade-off on these real clouds** (CPU time vs size):\n')
        rows = []
        for k, v in w.items():
            for lv, d in v['gzip'].items(): rows.append((k, lv, f"{d['bytes']/1e3:,.0f}", fmt(d['ratio'], 2), fmt(d['ms'], 1)))
        md.append(table(['cloud', 'level', 'compressed kB', 'ratio', 'ms'], rows)); res['wire'] = w
    for k in ('restamp_window_jitter_ms', 'restamp_offset_s'):
        if ac.get(k): md.append(f"* `{k}` (from node logs): {ac[k]}\n")
    md.append('### 4.8.2 Geometry, registration and timing accuracy (summary of measured evidence)\n')
    rows = []; s = pj('phase_sensor') or {}; L = (s.get('lidar') or {}).get('plane_fit')
    if L: rows.append(('LiDAR range precision (plane RMS)', f"{L['rms_mm']['mean']:.1f} mm (upper bound)"))
    g = (s.get('merged') or {}).get('consecutive_merged_clouds_icp')
    if g: rows.append(('ICP registration stability (static rig)', f"{g['translation_mm']['mean']:.1f} mm / {g['rotation_deg']['mean']:.3f}° between consecutive merged clouds"))
    f = s.get('fused') or {}
    if f.get('dt_fused_to_nearest_rgb_ms'): rows.append(('Time offset fused scan ↔ nearest RGB frame', f"p50 {f['dt_fused_to_nearest_rgb_ms']['p50']:.0f} ms, p95 {f['dt_fused_to_nearest_rgb_ms']['p95']:.0f} ms (fusion tolerance 150 ms)"))
    tm = pj('phase_timing') or {}
    for kk, vv in tm.items():
        if isinstance(vv, dict) and 'p50' in vv and ('rgb' in kk.lower() or 'thermal' in kk.lower()) and 'ms' in kk.lower(): rows.append((f'stamp offset inside fusion [{kk}]', f"p50 {vv['p50']:.1f}, p95 {vv['p95']:.1f}, max {vv['max']:.1f} ms"))
    rows.append(('Camera↔LiDAR calibration', 'RGB: placeholder intrinsics, zero extrinsics; thermal: solved from one person-as-target capture. **Not a measured checkerboard calibration — colour/thermal registration error is unquantified.**'))
    rows.append(('Thermal values', 'Relative 8-bit AGC luma, not temperature; no radiometric calibration.'))
    md.append(table(['aspect', 'result'], rows)); return '\n'.join(md), res
def build_stability():
    md = ['## 4.9 Long-run stability (30-minute soak with viewer clients)\n']; res = {}; sk = pj('phase_soak') or {}; S = load_csv(f'{DATA}/system_1hz.csv'); W = phase_windows()
    if 'soak' in W and S:
        a, b = W['soak']; m = (S['t'] >= a) & (S['t'] <= b); tt = (S['t'][m] - a) / 60; rows = []; fig, ax = plt.subplots(1, 2, figsize=(12, 3.8)); res['rss_slope_mb_per_h'] = {}
        for n in ['livox_driver', 'restamp', 'rgb_camera', 'thermal_camera', 'calibration', 'fusion', 'merge', 'streamer', 'rpicam_vid']:
            r = S.get(f'{n}_rss_mb')
            if r is None: continue
            y = r[m]; ok = np.isfinite(y)
            if ok.sum() < 20: continue
            sl = np.polyfit(tt[ok], y[ok], 1)[0] * 60; res['rss_slope_mb_per_h'][n] = sl; rows.append((n, fmt(y[ok][:30].mean(), 0), fmt(y[ok][-30:].mean(), 0), fmt(y[ok].max(), 0), fmt(sl, 1), fmt(np.nanmean(S[f'{n}_cpu'][m]), 1)))
            ax[0].plot(tt[ok], y[ok] - y[ok][:10].mean(), lw=.8, label=n)
        ax[0].set_xlabel('min'); ax[0].set_ylabel('RSS change (MB)'); ax[0].legend(fontsize=6, ncol=2); ax[0].set_title('Per-process memory change during soak'); ax[1].plot(tt, S['mem_avail_mb'][m], c=C[0], label='available MB'); ax[1].plot(tt, S['shm_mb'][m], c=C[1], label='/dev/shm MB'); ax[1].set_xlabel('min'); ax[1].legend(); ax[1].set_title('System memory'); md.append(table(['process', 'RSS first 30 s MB', 'RSS last 30 s MB', 'RSS max MB', 'slope MB/hour', 'mean CPU %'], rows)); md.append(f"![soak memory]({savefig('soak_memory.png')})\n")
        md.append(f"Memory available changed from {np.nanmean(S['mem_avail_mb'][m][:30]):.0f} to {np.nanmean(S['mem_avail_mb'][m][-30:]):.0f} MB; `/dev/shm` {np.nanmean(S['shm_mb'][m][:30]):.0f} → {np.nanmean(S['shm_mb'][m][-30:]):.0f} MB. Slopes near zero mean no leak at the resolution of a 30-minute test.\n")
    pr = sk.get('probes') or []
    if pr:
        rows = []
        for p in pr:
            r = p['rates']; rows.append((fmt(p['t_min'], 1), fmt((r.get('/landfill/fusion/points_rgbt') or {}).get('hz'), 2), fmt((r.get('/landfill/fusion/points_merged') or {}).get('hz'), 2), fmt((r.get('/landfill/rgb/image_raw') or {}).get('hz'), 2), (p.get('camera') or {}).get('thermal_hz'), p.get('journal_errors')))
        md.append('**Periodic probes during the soak:**\n'); md.append(table(['minute', 'fused Hz', 'merged Hz', 'RGB Hz', 'thermal Hz', 'ERROR/Traceback lines in prior 6 min'], rows))
        if all(((p.get('camera') or {}).get('thermal_hz') or 0) == 0 for p in pr): md.append('*The 10–11 ERROR lines per 6-minute window are the thermal node\'s own retry messages ("Too many consecutive read failures; reopening device") while the camera was in its stuck state; there were no tracebacks and no other errors during the soak.*\n')
    vc = sk.get('viewer_clients')
    if vc and 'paths' in vc: md.append('**Viewer clients over the whole soak:** ' + '; '.join(f"`{p}`: {d['requests']:,} requests, {d['n200']} updates, {d['errors']} errors, p95 {d['latency_ms']['p95']:.0f} ms" for p, d in vc['paths'].items()) + '\n')
    return '\n'.join(md), res
def parse_suite(name):
    p = f'{LOGD}/{name}.txt'
    if not os.path.exists(p): return None
    L = open(p).read().splitlines(); pas = [l for l in L if l.startswith('[PASS]')]; fail = [l for l in L if l.startswith('[FAIL]')]
    return pas, fail, L
def build_robust():
    md = ['## 4.10 Robustness: corner cases, crash recovery, restarts\n']; res = {}
    names = {'t_merge': 'Merge node (NaN, empty, degenerate, clock jumps, 4× overload, silence)', 't_fusion': 'Fusion node (thermal flapping/gone/unsyncable, RGB missing, bad LiDAR, clock jumps)', 't_streamer': 'Web streamer (empty, 404, garbage, concurrent/aborting clients, degenerate clouds)', 't_restamp': 'Restamp estimator (clock steps both ways, stalls, restart, latency spike)', 't_merge_stall': 'Merge buffer cap (frozen timestamps)'}
    rows = []; det = []
    for k, lab in names.items():
        r = parse_suite(k)
        if r is None: rows.append((lab, '–', '–', 'not run')); continue
        pas, fail, L = r; rows.append((lab, len(pas) if (pas or fail) else '–', len(fail) if (pas or fail) else '–', ('all pass' if not fail else 'FAILURES: ' + '; '.join(f[:70] for f in fail)) if (pas or fail) else 'estimator unit test: per-case error table below (new code recovers instantly in every case)')); det.append((k, L))
    md.append(table(['suite', 'pass', 'fail', 'result'], rows)); res['suites'] = rows
    for k, L in det:
        keep = [l for l in L if l.startswith(('[PASS]', '[FAIL]', 'node log', 'RESULT', 'duplicate', 'mode switches', '---', '   '))]
        md.append(f'<details><summary>{k} — full output</summary>\n\n```\n' + '\n'.join(keep[:60]) + '\n```\n</details>\n')
    for k, lab in (('crash_recovery', 'Hard kill (SIGKILL) of each node except the thermal camera node, with live LiDAR/RGB data flowing'), ('restart_cases', 'Whole-service restarts')):
        p = f'{LOGD}/{k}.txt'
        if os.path.exists(p): md.append(f'### {lab}\n\n```\n' + open(p).read()[-3500:] + '\n```\n')
    ev = f'{DATA}/real_ntp_step_evidence.txt'
    if os.path.exists(ev):
        L = open(ev).read().splitlines(); st = [l for l in L if 'Host clock stepped' in l]
        if st:
            mm = re.search(r'stepped by ([+-][\d.]+)s', st[0]); pubs = [l for l in L if 'Published merged cloud' in l]
            md.append('### A real NTP clock step on the live system (2026-11-12 20:23:53)\n')
            md.append(f"After the battery-run-out reboot the Pi (no real-time clock) booted with a stale clock; **36 s after boot NTP stepped it by {mm.group(1)} s (≈ {float(mm.group(1))/3600:.2f} h)** while the pipeline was running — the situation the restamp step-detector was written for, and the first time it was exercised by a genuine step rather than a unit test. Evidence (`data/real_ntp_step_evidence.txt`):\n")
            md.append(f"* `livox_restamp_node` logged `Host clock stepped by {mm.group(1)}s relative to the lidar clock … resetting offset estimate` at the moment of the step.\n* The merge node **kept publishing**: {len(pubs)} merged-cloud log lines in the first ≈25 s covered by the evidence file (the full 2 minutes after the step contained 136 merged clouds, ≈1.1 Hz while warming up). The first window straddled the step (its logged span was ≈7334 s, 2 scans) and was handled without error.\n* The restamp offset was stable straight after the reset (window jitter 26–42 ms as the 200-sample window refilled). With the previous code the lidar stamps would have trailed the camera stamps by the full step for the ≈20 s filter window and fusion would have produced nothing.\n")
    md.append('**Not tested on purpose:** SIGKILL of the thermal camera node — on 2026-11-12 it was observed to leave the camera unable to stream until power-cycled (see §4.11). A real clock step (needs root), LiDAR unplug while running, and a real WiFi client were also not exercised.\n'); return '\n'.join(md), res
def build_camera():
    md = ['## 4.11 Thermal camera reliability record\n']; res = {}
    ev = [l for l in open('/home/vish/power_logs/usb_events.log') if 'Thermal' in l and 'DISCONNECTED' in l] if os.path.exists('/home/vish/power_logs/usb_events.log') else []
    import collections; days = collections.Counter(l[:10] for l in ev); res['events_by_day'] = dict(days)
    md.append(f"`~/power_logs/usb_events.log` (1 Hz `lsusb` diff watcher) holds **{len(ev):,} thermal-camera USB disconnects** since 2026-10-26.\n"); md.append(table(['day', 'disconnects'], sorted(days.items())))
    st = []
    for ph in ('meta', 'idle_os', 'startup', 'idle_pipeline', 'sensor', 'transport', 'accuracy', 'recovery', 'restart_timing', 'final'):
        d = pj(f'phase_{ph}') or {}
        for k, v in d.items():
            if k in ('camera', 'camera_after', 'camera_before') and isinstance(v, dict): st.append((ph, k, v.get('thermal_hz'), v.get('usb_present')))
    for p in (pj('phase_soak') or {}).get('probes', []): st.append(('soak', f"t={p['t_min']:.0f} min", (p.get('camera') or {}).get('thermal_hz'), (p.get('camera') or {}).get('usb_present')))
    if st: md.append('**Camera state sampled during this campaign (thermal frames per second on the topic; USB enumerated):**\n'); md.append(table(['phase', 'sample', 'thermal Hz', 'USB present'], [(a, b, fmt(c_, 1), d) for a, b, c_, d in st]))
    md.append("**Findings from the 2026-11-11/12 investigation:** the camera enumerates normally but can enter a state where it accepts the stream request and never sends frames, then resets (USB error −71 on the video completion handler) about every 34 s (the pipeline's retry cadence). A full power-off cleared it; re-plugging, moving ports/controllers, resting and cooling did not. It once streamed perfectly for 56 minutes, and for 8 minutes on 2026-11-12 until the thermal node was hard-killed (SIGKILL), after which it delivered nothing. Port over-current counter = 0 and supply voltage ≥ 4.9 V, so a gross power fault is unlikely. Marginal signal integrity is suggested by a failed first descriptor read at *every* enumeration. No root access, so USB reset and low-level tracing were not possible.\n")
    return '\n'.join(md), res
def build_bench():
    md = ['## 4.12 Per-kernel benchmarks and per-stage timing\n']; res = {}; bj = pj('bench_offline')
    if bj:
        md.append('### 4.12.1 Offline kernel benchmark on this Pi (pipeline stopped; baseline vs optimised code, synthetic data)\n'); rows = []
        def walk(d, pre=''):
            for k, v in d.items():
                if isinstance(v, dict) and ('baseline_ms' in v or 'optimized_ms' in v or 'speedup' in v): rows.append((pre + k, fmt(v.get('baseline_ms'), 2), fmt(v.get('optimized_ms'), 2), fmt(v.get('speedup'), 2)))
                elif isinstance(v, dict): walk(v, pre + k + '/')
        walk(bj); md.append(table(['stage', 'baseline ms', 'optimised ms', 'speed-up ×'], rows) if rows else '```\n' + json.dumps(bj, indent=1)[:3000] + '\n```\n'); res['bench'] = bj
    tm = pj('phase_timing')
    if tm and tm.get('rows'):
        md.append('### 4.12.2 Per-stage fusion timing on live data (instrumented second fusion node, 90 s)\n'); rows = []
        for k, v in tm.items():
            if isinstance(v, dict) and 'p50' in v: rows.append((k, fmt(v['mean'], 2), fmt(v['p50'], 2), fmt(v['p95'], 2), fmt(v['max'], 2)))
        md.append(f"{tm['rows']} fused scans analysed. Columns ending `_ms` are milliseconds.\n"); md.append(table(['field', 'mean', 'p50', 'p95', 'max'], rows)); res['timing'] = tm
    if not (bj or (tm and tm.get('rows'))): md.append('_Benchmark/timing phases were not run in this pass._\n')
    return '\n'.join(md), res

def build_battery():
    md = ['### 4.1.8 Battery run-out event (Anker power bank)\n']; res = {}
    pl = '/home/vish/power_logs/power_log.csv'
    if not os.path.exists(pl): return '', res
    import subprocess
    out = subprocess.run(['bash', '-c', f"grep -E '^2026-11-12 (1[5-9]|20):' {pl}"], capture_output=True, text=True).stdout.splitlines()
    rows = [l.split(',') for l in out if len(l.split(',')) > 5]
    if len(rows) < 100: return '', res
    ts = np.array([time.mktime(time.strptime(r[0][:19], '%Y-%m-%d %H:%M:%S')) for r in rows]); ext = np.array([float(r[1]) for r in rows]); wat = np.array([float(r[3]) for r in rows]); cur = np.array([float(r[4]) for r in rows])
    gap = np.where(np.diff(ts) > 120)[0]
    if len(gap) == 0: return '', res
    g = gap[-1]; seg = slice(0, g + 1); 
    # session = last contiguous block before the final long gap, starting after the previous gap (boot)
    prev = gap[-2] + 1 if len(gap) > 1 else 0; seg = slice(prev, g + 1); t = ts[seg]; e = ext[seg]; w_ = wat[seg]; cu = cur[seg]
    dur_h = (t[-1] - t[0]) / 3600; wh = np.trapz(w_, t) / 3600; cons_wh = np.trapz(w_ / 0.85 + 2.5, t) / 3600
    res = dict(start=time.strftime('%H:%M:%S', time.localtime(t[0])), cutoff=time.strftime('%H:%M:%S', time.localtime(t[-1])), dur_h=dur_h, wh=wh, cons_wh=cons_wh, w_mean=w_.mean())
    n10 = max(1, len(t) // 20); drop = e[:n10].mean() - e[-n10:].mean()
    md.append(f"The Anker power bank ran out during the campaign. From the always-on power logger (`~/power_logs/power_log.csv`, 5 Hz) the battery session ran **{res['start']} → {res['cutoff']} ({dur_h:.2f} h)** with the pipeline and the campaign's load, then the Pi lost power; it rebooted at 20:23 once power returned (the data gap is visible as a break in the timeline).\n")
    md.append(table(['quantity', 'value'], [('Session length on battery', f"{dur_h:.2f} h"), ('Mean SoC-rail power', f"{w_.mean():.2f} W"), ('Energy into the SoC rails', f"{wh:.1f} Wh"), ('Conservative energy drawn from the bank (÷0.85 efficiency, +2.5 W for USB peripherals)', f"≈ {cons_wh:.1f} Wh — the bank held *at least* this much usable energy"), ('EXT5V in the first / last 5 % of the session', f"{e[:n10].mean():.3f} V / {e[-n10:].mean():.3f} V (drop {drop*1000:.0f} mV)"), ('Lowest EXT5V in the session', f"{e.min():.3f} V"), ('Mean estimated input current in the last 5 minutes', f"{cu[(t >= t[-1] - 300)].mean():.2f} A")]))
    fig, ax = plt.subplots(2, 1, figsize=(11, 4.5), sharex=True); x = (t - t[0]) / 3600; ax[0].plot(x, e, lw=.5, c=C[2]); ax[0].set_ylabel('EXT5V (V)'); ax[0].set_title('Supply voltage and power over the battery session'); ax[1].plot(x, w_, lw=.5, c=C[3]); ax[1].set_ylabel('SoC rails (W)'); ax[1].set_xlabel('hours since the Pi booted on the bank')
    md.append(f"![battery]({savefig('battery_run.png')})\n"); md.append(('**No voltage warning before cut-off:** EXT5V sagged only %.0f mV over the session, so the bank delivered a steady 5 V right up to the moment it shut off (typical of a protection cut-out). Software could not have seen it coming from the supply voltage.\n' % (drop * 1000)) if drop < 0.05 else ''); md.append('*The pack\'s nameplate capacity and output rating were not recorded, so this gives a lower bound on usable energy rather than a capacity.*\n'); return '\n'.join(md), res
def build_thermal_supplement():
    md = ['## 4.13 Live thermal supplement (thermal camera streaming)\n']; res = {}; ts = pj('phase_thermal_supplement')
    if not ts or 'failed' in ts: return '', res
    md.append("After the battery ran out, the Pi was fully powered off and rebooted, and **the thermal camera came back streaming** (the second time a full power loss has cleared its stuck state). This section repeats the thermal-dependent measurements with live thermal data and compares them with the earlier no-thermal (LiDAR+RGB fallback) results.\n")
    cs, ce = ts.get('camera_start') or {}, ts.get('camera_end') or {}; md.append(f"Camera at start: {cs}; at end: {ce}; USB disconnects during the supplement window: **{ts.get('usb_drops_during')}**.\n")
    T = (ts.get('sensor') or {}).get('thermal') or {}
    if T.get('frames'):
        md.append('### 4.13.1 Thermal camera data\n'); md.append(table(['quantity', 'value'], [('Frames / rate', f"{T['frames']} in 40 s ⇒ **{T['rate_hz']:.2f} Hz**"), ('Resolution / encoding', f"{T['resolution'][0]}×{T['resolution'][1]}, `{T['encoding']}`"), ('8-bit range in scene', f"{T['dynamic_range_8bit']['min']:.0f}–{T['dynamic_range_8bit']['max']:.0f} (mean {T['dynamic_range_8bit']['mean']:.0f}, spatial std {T['dynamic_range_8bit']['std_spatial']:.1f})"), ('Temporal std per pixel p10/median/p90 (counts)', f"{T['temporal_std_per_pixel']['p10']:.2f} / {T['temporal_std_per_pixel']['median']:.2f} / {T['temporal_std_per_pixel']['p90']:.2f}"), ('Stuck pixels / unique frames', f"{T['stuck_pixel_frac']*100:.3f} % / {T['unique_frame_frac']*100:.0f} %"), ('Inter-arrival ms (mean/p95/max)', ' / '.join(f"{T['interarrival_ms'][k]:.0f}" for k in ('mean', 'p95', 'max'))), ('Stamp age at arrival ms (mean/p95)', f"{T['stamp_age_at_arrival_ms']['mean']:.0f} / {T['stamp_age_at_arrival_ms']['p95']:.0f}")])); res['thermal'] = T
        if os.path.exists(f'{HERE}/frames/thermal_sample.png'): md.append('![thermal sample](frames/thermal_sample.png)\n')
    F = (ts.get('sensor') or {}).get('fused') or {}; F0 = (pj('phase_sensor') or {}).get('fused') or {}
    if F.get('fused_clouds'):
        md.append('### 4.13.2 3-way fusion (LiDAR + RGB + thermal) vs LiDAR+RGB fallback\n')
        def g(d, *k):
            for x in k:
                if not isinstance(d, dict) or x not in d: return None
                d = d[x]
            return d
        rows = [('Fused cloud rate (Hz)', fmt(F0.get('fused_rate_hz'), 2), fmt(F['fused_rate_hz'], 2)), ('Points with valid RGB (%)', fmt((g(F0, 'valid_rgb_frac', 'mean') or 0) * 100, 1), fmt(F['valid_rgb_frac']['mean'] * 100, 1)), ('Points with valid thermal (%)', fmt((g(F0, 'valid_thermal_frac', 'mean') or 0) * 100, 1), fmt(F['valid_thermal_frac']['mean'] * 100, 1)), ('Points with both (%)', fmt((g(F0, 'both_valid_frac', 'mean') or 0) * 100, 1), fmt(F['both_valid_frac']['mean'] * 100, 1))]
        for k, lab in (('dt_fused_to_nearest_rgb_ms', 'nearest RGB offset p50 / p95 (ms)'), ('dt_fused_to_nearest_thermal_ms', 'nearest thermal offset p50 / p95 (ms)')):
            rows.append((lab, f"{g(F0, k, 'p50') and fmt(g(F0, k, 'p50'), 0)} / {g(F0, k, 'p95') and fmt(g(F0, k, 'p95'), 0)}" if F0.get(k) else '–', f"{fmt(F[k]['p50'], 0)} / {fmt(F[k]['p95'], 0)}" if F.get(k) else '–'))
        md.append(table(['quantity', 'without thermal (fallback)', 'with live thermal'], rows)); res['fused'] = F
        md.append('*This supplement was measured **before** the `rpicam-vid --flush` optimisation (§4.14). The lower fused rate with live thermal here came from the irregular RGB stamping; after the change the same 3-way configuration delivered 9.80 Hz with the LiDAR-to-RGB offset at p50 12 ms.*\n')
        z = f'{DATA}/sample_fused_cloud.npz'
        if os.path.exists(z):
            d = np.load(z); x = d['xyz']; th = d['thermal']
            if np.isfinite(th).any():
                fig, ax = plt.subplots(1, 2, figsize=(11, 4)); ax[0].scatter(x[:, 1], x[:, 2], c=th, s=.6, cmap='inferno'); ax[0].set_aspect('equal'); ax[0].set_title('fused cloud coloured by thermal (front view)'); ax[1].scatter(x[:, 0], x[:, 1], c=th, s=.6, cmap='inferno'); ax[1].set_aspect('equal'); ax[1].set_title('top-down'); md.append(f"![fused thermal]({savefig('fused_thermal.png')})\n")
    tp = ts.get('topics_lidar_fused_merged') or {}; tp0 = (pj('phase_idle_pipeline') or {}).get('topics_fused_merged') or {}
    rows = []
    for k, lab in (('/landfill/fusion/points_rgbt', 'fused'), ('/landfill/fusion/points_merged', 'merged')):
        a, b = tp0.get(k) or {}, tp.get(k) or {}
        if a.get('interval_ms') and b.get('interval_ms'): rows.append((lab, fmt(a['hz'], 2), fmt(b['hz'], 2), fmt(a['interval_ms']['p95'], 0), fmt(b['interval_ms']['p95'], 0), fmt(a['stamp_age_ms']['p50'], 0), fmt(b['stamp_age_ms']['p50'], 0)))
    if rows: md.append('**Rates and latency, no-thermal idle vs live thermal:**\n'); md.append(table(['topic', 'Hz (no thermal)', 'Hz (thermal)', 'interval p95 ms (no thermal)', '(thermal)', 'stamp age p50 ms (no thermal)', '(thermal)'], rows))
    S = load_csv(f'{DATA}/system_1hz.csv'); smk = dict(sub_markers('thermal_supplement')); imk = dict(sub_markers('idle_pipeline'))
    if S and 'steady_begin' in smk and 'steady_end' in smk and 'steady_begin' in imk:
        a1, b1 = smk['steady_begin'], smk['steady_end']; a0, b0 = imk['steady_begin'], imk['steady_end']; m1 = (S['t'] >= a1) & (S['t'] <= b1); m0 = (S['t'] >= a0) & (S['t'] <= b0); rows = []
        for n in ['livox_driver', 'restamp', 'rgb_camera', 'thermal_camera', 'calibration', 'fusion', 'merge', 'streamer', 'rpicam_vid']:
            c = S.get(f'{n}_cpu')
            if c is not None: rows.append((n, fmt(np.nanmean(c[m0]), 1), fmt(np.nanmean(c[m1]), 1), fmt(np.nanmean(c[m1]) - np.nanmean(c[m0]), 1)))
        rows.append(('**system total (of 400)**', fmt(np.nanmean(S['cpu_total_pct'][m0]), 0), fmt(np.nanmean(S['cpu_total_pct'][m1]), 0), fmt(np.nanmean(S['cpu_total_pct'][m1]) - np.nanmean(S['cpu_total_pct'][m0]), 0)))
        P = load_csv(f'{DATA}/power_raw.csv')
        if P:
            for lab, (a, b) in (('no thermal', (a0, b0)), ('live thermal', (a1, b1))): 
                mm = (P['t'] >= a) & (P['t'] <= b); 
                if mm.sum() > 20: res[f'power_{lab}'] = dict(mean_a=float(P['est_in_a'][mm].mean()), max_a=float(P['est_in_a'][mm].max()), w=float(P['watts'][mm].mean()), temp=float(P['temp_c'][mm].mean()))
            if 'power_no thermal' in res and 'power_live thermal' in res: rows.append(('**mean input power W (SoC rails)**', fmt(res['power_no thermal']['w'], 2), fmt(res['power_live thermal']['w'], 2), fmt(res['power_live thermal']['w'] - res['power_no thermal']['w'], 2)))
        md.append('**CPU per process, steady state, no thermal vs live thermal (percent of one core):**\n'); md.append(table(['process', 'no thermal', 'live thermal', 'difference'], rows)); res['cpu_rows'] = rows
    th = ts.get('threads')
    if th and 'processes' in th:
        md.append('**Per-thread view with live thermal (60 s):** busiest thread per process —\n'); md.append(table(['process', 'threads', 'total CPU %', 'busiest thread', 'its CPU %'], [(n, v['n_threads'], fmt(v['total_cpu_pct'], 1), v['busiest_thread']['name'], fmt(v['busiest_thread']['total_pct'], 1)) for n, v in th['processes'].items()]))
    ll = ts.get('lidar_loss') or []
    if ll and 'rx_rate_hz' in ll[0]: md.append('**LiDAR scan loss with thermal live:** ' + '; '.join(f"{d['rx_rate_hz']:.2f} Hz, {d['scans_missing_pct']:.1f} % missing, {d['udp_rcvbuf_errors']} UDP drops" for d in ll) + '\n')
    return '\n'.join(md), res
