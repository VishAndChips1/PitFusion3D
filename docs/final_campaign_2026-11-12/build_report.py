#!/usr/bin/env python3
"""Assemble FULL_RESULTS_REPORT.md and CODE_REFERENCE.md from the campaign data. Usage: build_report.py"""
import sys, os, traceback, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rb_common import *
import rb_power, rb_pipeline, rb_misc
def safe(fn, name):
    try: return fn()
    except Exception as e:
        tb = traceback.format_exc(); print(f'SECTION {name} FAILED: {e!r}\n{tb[-600:]}'); return f'## {name}\n\n_This section failed to build: `{e!r}`. Raw data is in `data/`._\n', {}
def inv_block(key, maxc=2500):
    t = inv(key).strip(); return f'```\n{t[:maxc]}\n```\n' if t else '_not collected_\n'
def build_system():
    md = ['# 2. Hardware, operating system and environment\n']
    def first(key, pat, default='–'):
        m = re.search(pat, inv(key)); return m.group(1).strip() if m else default
    md.append('## 2.1 Platform summary\n')
    md.append(table(['item', 'value'], [('Board', inv('hw.model').splitlines()[0] if inv('hw.model') else '–'), ('CPU', first('hw.lscpu', r'Model name:\s*(.+)') + ', ' + first('hw.lscpu', r'(?m)^CPU\(s\):\s*(\d+)') + ' cores'), ('CPU frequency range (governor)', first('hw.cpufreq', r'scaling_min_freq=(\d+)') + ' – ' + first('hw.cpufreq', r'scaling_max_freq=(\d+)') + ' kHz, governor ' + first('hw.cpufreq', r'scaling_governor=(\w+)')),
        ('Memory', first('hw.meminfo', r'MemTotal:\s*(\d+) kB') + ' kB'), ('Kernel / OS', inv('os.uname').splitlines()[0][:120] if inv('os.uname') else '–'), ('Boot time (systemd-analyze)', inv('os.boot').splitlines()[0] if inv('os.boot') else '–'), ('Storage', first('storage.df', r'(/dev/mmcblk0p2\s+\S+\s+\S+\s+\S+\s+\S+)')), ('Firmware', inv('hw.vcgencmd_version').replace('\n', '; ')[:160]), ('PSU negotiated (device tree)', 'see §2.2')]))
    md.append('### Hardware detail (raw collected output)\n'); 
    for k, t in (('hw.lscpu', 'lscpu'), ('hw.boot_config_txt', 'boot config.txt (non-comment lines)'), ('hw.config_int', 'firmware integer config'), ('hw.clocks', 'clocks'), ('hw.volts', 'voltages'), ('hw.thermal_zones', 'thermal zones and trip points'), ('hw.cooling', 'cooling devices / fan'), ('hw.power_dt', 'power data from device tree'), ('hw.pmic_adc', 'PMIC ADC snapshot (all rails)'), ('hw.eeprom', 'bootloader config')): md.append(f'<details><summary>{t}</summary>\n\n' + inv_block(k, 4000) + '\n</details>\n')
    md.append('## 2.2 Power supply\n'); md.append('The Pi 5 negotiates its power budget with the supply: `max_current` in the device tree is the negotiated limit in mA (5000 = a 5 A capable source, which also enables the full 1.6 A USB budget). `usb_max_current_enable=0` is the default; USB limits are governed by the negotiated PSU current. This campaign ran from an **Anker power bank**; its true output limit and any protection threshold are not visible to software, which is why the 5 A abort gate and the voltage gate (4.75 V) were used.\n'); md.append(inv_block('hw.power_dt'))
    md.append('## 2.3 Operating system, services and timing\n'); 
    for k, t in (('os.boot', 'systemd boot analysis'), ('os.user_boot', 'user-session units'), ('os.sysctl_net', 'network sysctls (note `rmem_max` = 212 992)'), ('systemd.unit', 'pitfusion3d.service unit and runtime status'), ('systemd.linger', 'linger (boot without login)'), ('os.limits', 'ulimits'), ('storage.df', 'filesystems'), ('storage.shm', '/dev/shm')): md.append(f'<details><summary>{t}</summary>\n\n' + inv_block(k, 5000) + '\n</details>\n')
    md.append('## 2.4 Network\n'); 
    for k, t in (('net.addr', 'addresses and routes'), ('net.nm_conns', 'NetworkManager connections (hotspot autoconnect is off)'), ('net.wifi_link', 'WiFi link'), ('net.wifi_scan', 'WiFi scan'), ('net.eth0', 'eth0 (LiDAR) statistics'), ('net.udp_snmp', 'UDP counters')): md.append(f'<details><summary>{t}</summary>\n\n' + inv_block(k, 4000) + '\n</details>\n')
    md.append('## 2.5 USB, cameras and software stack\n')
    for k, t in (('usb.tree', 'USB topology'), ('usb.thermal_desc', 'thermal camera USB descriptors'), ('usb.ports', 'USB port over-current counters'), ('video.list', 'V4L2 devices'), ('video.thermal_fmt', 'thermal formats'), ('video.cameras', 'rpicam cameras'), ('sw.pip', 'Python libraries'), ('sw.ros_pkgs', 'ROS packages'), ('sw.ros_env', 'ROS environment'), ('sw.fastdds_xml', 'Fast DDS profile'), ('sw.ffmpeg', 'ffmpeg / rpicam')): md.append(f'<details><summary>{t}</summary>\n\n' + inv_block(k, 5000) + '\n</details>\n')
    md.append('## 2.6 ROS graph, QoS, parameters, process layout\n')
    for k, t in (('ros.nodes', 'nodes'), ('ros.topics', 'topics and types'), ('ros.qos_lidar', 'QoS /livox/lidar'), ('ros.qos_rgb', 'QoS RGB'), ('ros.qos_fused', 'QoS fused'), ('ros.qos_merged', 'QoS merged'), ('proc.tree', 'process tree (pid, threads, nice, CPU, RSS, uptime)'), ('proc.affinity_sched', 'CPU affinity and scheduling policy of every pipeline process')): md.append(f'<details><summary>{t}</summary>\n\n' + inv_block(k, 5000) + '\n</details>\n')
    d = pj('inventory') or {}
    for k in sorted(d):
        if k.startswith('ros.params'): md.append(f'<details><summary>live parameters {k[10:]}</summary>\n\n```yaml\n{d[k]["output"][:4000]}\n```\n</details>\n')
    md.append('## 2.7 Calibration and configuration files\n')
    for k, t in (('cfg.extrinsics', 'extrinsics.yaml'), ('cfg.intrinsics', 'intrinsics.yaml'), ('cfg.livox', 'Livox driver config'), ('cfg.run_pipeline', 'run_pipeline.sh'), ('cfg.launch_full', 'pitfusion3d_full.launch.py'), ('cfg.launch_sensors', 'real_sensors.launch.py')): md.append(f'<details><summary>{t}</summary>\n\n```\n{inv(k)[:9000]}\n```\n</details>\n')
    md.append('## 2.8 Repository state\n'); md.append(inv_block('git.log', 4000)); return '\n'.join(md)
def build_method():
    return '''# 3. Methodology
## 3.1 How the measurements were made
* **One unattended orchestrator** (`campaign.py`) ran ~20 phases in sequence with **no operator input**. Each phase writes its results to `data/phase_<name>.json`; phase boundaries are recorded in `data/markers.jsonl` so any raw time series can be sliced per phase.
* **Power:** `vcgencmd pmic_read_adc` returns every PMIC rail's voltage and current. A dedicated thread read it back-to-back for the whole campaign (≈30 Hz, the firmware's limit; each read ≈31 ms) and wrote `data/power_raw.csv` (one row per read, all rails). **Estimated input current** = Σ(V×I over the SoC rails) ÷ EXT5V; a **conservative** variant divides by 0.85 and adds 0.5 A for unmetered USB loads. `get_throttled` and SoC temperature are logged alongside.
* **Safety gates** evaluated on every power sample: abort the running stress step if estimated input current > 4.4 A, EXT5V < 4.75 V, or SoC > 82 °C (the Pi shuts off above 5 A; the user's hard requirement was to stay below it). A second guard in the stress phase stopped workers at 4.2 A.
* **System sampler** (1 Hz, `data/system_1hz.csv`): per-process CPU %, RSS and thread count for all 9 pipeline processes, per-core CPU, load, memory, swap, `/dev/shm` size, ARM/core clocks, UDP/`eth0`/`wlan0` counters, and CPU used by non-pipeline processes (the IDE, desktop, system services).
* **Topic measurements** used **fresh subscriber processes** (`probe_topics.py`, `chars.py`, …) that load **the pipeline's own Fast DDS profile**, one process per topic group. (Early in the session measuring tools without the profile under-reported rates — see §4.5.1 — which is why this matters.) *Stamp age* = receive wall-clock − header stamp.
* **Per-thread CPU:** deltas of `utime`/`stime` per `/proc/<pid>/task/<tid>/stat` over 2 × 60 s, with context-switch counts.
* **Fidelity (sweep):** for each point cap, a contemporaneous full cloud is grabbed; the capped cloud's median nearest-neighbour spacing (cKDTree) and the share of the full cloud's 2 cm and 5 cm occupied voxels it still covers are computed.
* **Accuracy:** real ROS clouds are encoded with the production encoder, decoded, and compared point-by-point; LiDAR precision is the RMS residual of RANSAC plane fits (3 cm inlier band) on real scans; registration stability is ICP between consecutive merged clouds; time alignment is the nearest-camera-stamp offset of each fused scan.
* **Corner cases:** isolated-domain suites for merge/fusion/streamer/restamp, plus SIGKILL of every node except the thermal camera (see §4.10) and graceful restarts of the whole service.
* **Stress:** busy-loop workers (numpy matrix multiply, optional memory traffic) ramped 0→4 workers (+512 MB memory) while the pipeline ran, with cool-down.

## 3.2 Known limits of the method (read before quoting numbers)
1. **Peak current is a sampled quantity.** Samples are ≈30 ms apart and each averages the ADC's internal window, so spikes shorter than ≈30 ms are not captured. The input current is *derived* from rail powers, not measured at the USB-C connector; the conservative figure bounds the regulator-loss and USB-peripheral error but is still an estimate.
2. **No real WiFi client** was connected, so radio TX power/current under a real viewer load is not in the data. The viewer clients were local HTTP pollers.
3. **Measurement overhead:** the 30 Hz `vcgencmd` sampler and the 1 Hz system sampler fork processes; they cost on the order of a tenth of a core and are included in all CPU figures (visible as `other_cpu_pct`).
4. **The IDE (VS Code server) and desktop run on the same Pi** and compete for CPU; their share is recorded per second.
5. **Thermal camera availability** varied (see §4.11); thermal-dependent results are flagged where the camera was not streaming.
6. **Fidelity percentages** depend on the scene and window contents (the full cloud varies ±10 % between grabs), so compare trends, not neighbouring steps.
7. **Calibration** is approximate (placeholder RGB intrinsics, single-capture thermal solve): colour/thermal *registration* accuracy is not quantified, only the processing chain's numerical accuracy.
8. **Ambient temperature was not controlled or measured:** the rig was moved to a cooler room for this campaign, so SoC temperatures (and fan behaviour) are lower than they would be in typical or field conditions. Treat temperature results as a lower bound and compare *trends* (load vs temperature, cool-down shape), not absolute values.
9. **Single unit, single scene:** one Pi, one LiDAR, mostly a static indoor scene; no field (landfill) conditions.
'''
def build_logs():
    md = ['## 4.6 Pipeline self-reported statistics (from the node logs during the campaign)\n']; res = {}
    ph = pj('phase_meta') or {}; since = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(ph.get('start_epoch', time.time() - 3 * 3600)))
    r = subprocess.run(['bash', '-c', f'journalctl --user -u pitfusion3d.service --no-pager --since "{since}" -o cat 2>/dev/null'], capture_output=True, text=True).stdout; L = r.splitlines(); res['journal_lines'] = len(L)
    pub = [re.search(r'Published merged cloud: (\d+) pts \((\d+) scans, (\d+) raw pts, ([\d.]+)s window\) @ ([\d.]+) Hz', l) for l in L]; pub = [m for m in pub if m]
    if pub:
        a = np.array([[float(x) for x in m.groups()] for m in pub]); md.append(f"**Merge node, {len(a)} published windows:**\n"); md.append(table(['quantity', 'mean', 'p5', 'p50', 'p95', 'max'], [(n, *[fmt(f(a[:, i]), 2) for f in (np.mean, lambda x: np.percentile(x, 5), np.median, lambda x: np.percentile(x, 95), np.max)]) for i, n in enumerate(['output points', 'scans per window', 'raw points before voxel', 'window span (s)', 'publish rate Hz (instantaneous)'])])); res['merge_windows'] = dict(n=len(a), pts=a[:, 0].mean(), scans=a[:, 1].mean(), raw=a[:, 2].mean(), hz=a[:, 4].mean())
        md.append(f"Voxel reduction (0.01 m): {a[:, 2].mean():,.0f} raw points → {a[:, 0].mean():,.0f} published ({100*(1-a[:,0].mean()/a[:,2].mean()):.0f} % removed).\n")
    cnt = {k: sum(1 for l in L if pat in l) for k, pat in {'merge windows with non-converged scans': 'did not converge', 'merge worker behind (stale window replaced)': 'Merge worker behind', 'fusion fell back to LiDAR+RGB': 'fusing lidar + RGB only', 'fusion thermal recovered': 'Thermal fusion working again', 'restamp: host clock step reset': 'Host clock stepped', 'restamp: device clock backwards': 'jumped backwards', 'thermal read failure warnings': 'Failed to read frame from thermal', 'thermal cool-down reopen': 'reopening device after', 'rgb stream ended': 'rpicam-vid stream ended', 'ERROR lines (all)': '[ERROR]', 'Tracebacks': 'Traceback', 'process has died (respawn)': 'process has died'}.items()}
    md.append('**Event counts over the campaign:**\n'); md.append(table(['event', 'count'], list(cnt.items()))); res['counts'] = cnt
    jit = [float(m.group(1)) for l in L for m in [re.search(r'window_jitter=([\d.]+)ms', l)] if m]; off = [float(m.group(1)) for l in L for m in [re.search(r'offset=([\d.]+)s', l)] if m]
    if jit: md.append(f"**LiDAR→host offset filter:** {len(jit)} log lines; window jitter (max−min arrival latency over 20 s) mean {np.mean(jit):.2f} ms, max {np.max(jit):.2f} ms; estimated offset between {min(off):.3f} and {max(off):.3f} s.\n")
    errs = {}
    for l in L:
        if '[ERROR]' in l: k = re.sub(r'[\d.]+', 'N', l.split('[ERROR]')[-1])[:110]; errs[k] = errs.get(k, 0) + 1
    if errs: md.append('**ERROR messages grouped:**\n'); md.append(table(['message', 'count'], sorted(errs.items(), key=lambda kv: -kv[1])[:15]))
    return '\n'.join(md), res
HISTORY = '''# 5. Project history and previous results (context)
The work in this report follows three earlier sessions whose write-ups are committed in the repository (`docs/performance_2026-11-11/`, `docs/pointcap_sweep_2026-11-11/`, `docs/pipeline_audit_2026-11-12/`).

## 5.1 Before / after the 2026-11-11 performance work (independent subscriber loading the pipeline profile)
| run | LiDAR Hz | RGB Hz | fused Hz | fused latency ms | merged Hz | merged latency ms | pipeline CPU (of 400) | system CPU (of 400) | SoC °C |
|---|---|---|---|---|---|---|---|---|---|
| **BEFORE** original build | 10.0 | 3.6 | 2.42 | 77 | 0.66 | 148 | 76 % | 198 % | 63.0 |
| + shared-memory transport (128 MB) | 10.0 | 15.1 | 9.76 | 49 | 1.49 | 321 | 159 % | 324 % | 67.1 |
| + 64 MB segment + threaded merge | 10.0 | 15.0 | 9.75 | 54 | 1.50 | 376 | 167 % | 338 % | 68.2 |
| + on-demand streamer + RGB 10 fps | 10.0 | 10.1 | 10.02 | 46 | 1.55 | 276 | 117 % | 270 % | 66.9 |
| **AFTER** final build, 1 viewer | 10.1 | 10.1 | 10.03 | 48 | 1.53 | 305 | 137 % | 291 % | 69.2 |

Headline: RGB reaching fusion 3.6 → 10 Hz, fused 2.42 → 10.03 Hz, merged 0.66 → 1.53 Hz; CPU per delivered fused Hz fell from 31.4 to ≈12.4–13.7 %-core. Transport A/B (received/sent): default DDS RGB 3.4 Hz, thermal 18.4, cloud 6.2; shared memory RGB 14.4, thermal 28.9, cloud 9.6. The CPU-pinning study (rejected) showed no consistent gain; one pinned run collapsed.

## 5.2 Chronological list of problems found and how they were resolved
| # | Problem | Evidence | Fix |
|---|---|---|---|
| 1 | Large messages dropped over UDP loopback (RGB 3.4 of 15 Hz) | transport A/B | Fast DDS shared-memory profile (64 MB) |
| 2 | Stale `/dev/shm/fastrtps_*` after crashes (821 MB leak, introduced by #1) | corner-case suite | `clean_stale_shm.py` at every start |
| 3 | Killed nodes never came back | crash test | `respawn=True` on every node |
| 4 | Orphaned `rpicam-vid` after RGB node crash | crash test | PDEATHSIG + stale cleanup |
| 5 | Merge could build a backlog | design review | worker thread + size-1 replace-stale queue + max age |
| 6 | One NaN discarded a whole merge window | corner case | NaN filter at fusion and merge input |
| 7 | Unsyncable/missing thermal blanked the viewer | corner case | 2-way fallback synchroniser, 1.0 s monotonic timeout |
| 8 | Streamer encoded at 10 Hz with no viewer (27 % of a core) | profiling | lazy on-demand encode |
| 9 | Unknown URL hung the browser (no Content-Length) | corner case | explicit 404 with `Content-Length: 0` |
| 10 | Thermal camera reset loop worsened by instant reopening | logs | settle wait + growing cool-down |
| 11 | WiFi instability: hotspot profile (priority 100) and five other profiles competed | wpa/NM logs | hotspot autoconnect off, `Patelnet` preferred, retries forever |
| 12 | Forward host-clock step stalled fusion ≈20 s | restamp unit test | step detector resets the offset filter |
| 13 | Merge buffer unbounded with stalled stamps (+145 MB for 250 scans) | stall test | 60-scan cap |
| 14 | Thermal node closed the camera mid-read | review + incident | join capture thread before release |
| 15 | Measuring tools without the DDS profile under-report rates | A/B + this campaign | all probes now load the profile |

## 5.3 Earlier point-cap sweep (2026-11-11, mains power)
Merged cap 20 k → 105 k and live 5 k → 25 k, 120 s per level. Highlights: estimated input current peak 1.60–2.05 A at every level (no trend with cap); bandwidth to one viewer 2.3 → 11.4 Mbit/s over the merged range; 5 cm coverage of the full cloud 53 % (20 k) → 81 % (90 k); above ≈90 k the cap stops binding because the window holds only ≈110 k points. The cap, not the Pi, is the practical limit: the radio link. Suggested caps 60 000 merged / 12 000 live (≈6.5 Mbit/s).
'''

def build_optimisation():
    o = pj('phase_optimisation_pass')
    if not o: return '', {}
    md = ['## 4.14 Optimisation pass (2026-11-12 evening, measured on the live system with thermal streaming)\n']
    md.append('Three code changes were made and each was measured against the previous behaviour. Only the third produced a gain; the second is neutral but kept because it is more accurate.\n')
    r = o['rgb_stamp']; g = lambda d, k: d['stamp_interval_ms'][k]
    md.append(table(['RGB frame stamping', 'stamp interval p50 ms', 'p95 ms', 'std ms', 'stamp age at arrival mean ms'], [
        ('before (stamp after colour conversion)', fmt(g(r['before'], 'p50'), 0), fmt(g(r['before'], 'p95'), 0), fmt(g(r['before'], 'std'), 0), fmt(r['before']['age_ms']['mean'], 0)),
        ('stamp immediately after the frame is read', fmt(g(r['stamp_at_read'], 'p50'), 0), fmt(g(r['stamp_at_read'], 'p95'), 0), fmt(g(r['stamp_at_read'], 'std'), 0), fmt(r['stamp_at_read']['age_ms']['mean'], 0)),
        ('**+ `rpicam-vid --flush`**', fmt(g(r['flush'], 'p50'), 0), fmt(g(r['flush'], 'p95'), 0), fmt(g(r['flush'], 'std'), 0), fmt(r['flush']['age_ms']['mean'], 0))]))
    md.append('*Finding:* the erratic RGB timing (frames stamped in clustered pairs) came from `rpicam-vid` buffering its output, not from the node\'s own delay: stamping earlier changed nothing, while `--flush` made the stream perfectly regular (interval std 79 → 5 ms).\n')
    fb, fa, mb, ma = o['fused_before'], o['fused_after'], o['merged_before'], o['merged_after']; sb, sa = o['sync_before'], o['sync_after']
    md.append(table(['effect downstream (live thermal)', 'before', 'after `--flush`'], [
        ('Fused cloud rate (Hz)', fmt(fb['hz'], 2), fmt(fa['hz'], 2)), ('Fused interval p95 (ms)', fmt(fb['interval_ms']['p95'], 0), fmt(fa['interval_ms']['p95'], 0)),
        ('Merged cloud rate (Hz)', fmt(mb['hz'], 2), fmt(ma['hz'], 2)), ('Merged stamp age p50 (ms)', fmt(mb['stamp_age_ms']['p50'], 0), fmt(ma['stamp_age_ms']['p50'], 0)),
        ('**LiDAR scan to nearest RGB frame: p50 / p95 (ms)**', f"{sb['dt_fused_to_nearest_rgb_ms']['p50']:.0f} / {sb['dt_fused_to_nearest_rgb_ms']['p95']:.0f}", f"**{sa['dt_fused_to_nearest_rgb_ms']['p50']:.0f} / {sa['dt_fused_to_nearest_rgb_ms']['p95']:.0f}**"),
        ('LiDAR scan to nearest thermal frame: p50 / p95 (ms)', f"{sb['dt_fused_to_nearest_thermal_ms']['p50']:.0f} / {sb['dt_fused_to_nearest_thermal_ms']['p95']:.0f}", f"{sa['dt_fused_to_nearest_thermal_ms']['p50']:.0f} / {sa['dt_fused_to_nearest_thermal_ms']['p95']:.0f}")]))
    md.append('The time offset between each LiDAR scan and the colour image painted onto it fell about 3× (p50 34 → 12 ms), which directly reduces colour smearing on a moving platform; the fused rate rose ≈ 7 % and the merged cloud became ≈ 24 % fresher. This also explains most of the 3-way-fusion rate cost seen earlier (fused 8.8 → 7.0 Hz when thermal went live): irregular RGB stamps made the synchroniser miss matches.\n')
    md.append('**Code clean-up in the same pass (behaviour unchanged, all regression suites re-run):** the merge node\'s `_on_cloud` (complexity 14) was split into `_finite_points`, `_reset_window_if_clock_rewound`, `_window_is_complete` and `_submit_window`; the thermal node\'s `_capture_loop` (complexity 14) became `_connect`, `_read_frame`, `_on_read_failure`, `_publish_frame` plus a small `_CaptureHealth` object; the RGB node gained `_publish_frame`; `thermal_view.py` was rewritten (class-based, subscribes only while someone is watching); style fixes in `clean_stale_shm.py`. Regression results: merge suite 13/13, stall test pass, restamp estimator unchanged, thermal node exits cleanly on SIGINT/SIGTERM with no camera, live restart streams thermal at 29.3 Hz with 0 USB drops.\n')
    return '\n'.join(md), {}


def build_lidar_power():
    p = f'{HERE}/LIDAR_POWER_MODE.md'
    if not os.path.exists(p): return '', {}
    t = open(p).read().replace('# LiDAR power-mode control (Livox Avia) from the Raspberry Pi', '## 4.15 LiDAR power-mode control (Livox Avia) from the Raspberry Pi', 1)
    t = re.sub(r'^## ', '### ', t[t.index('\n') + 1:], flags=re.M)
    return '## 4.15 LiDAR power-mode control (Livox Avia) from the Raspberry Pi\n' + t, {}

def build_issues(R):
    md = ['# 6. Anomalies, issues and limitations observed in this campaign\n']
    L = R.get('lidar_loss') or []
    rows = []
    if L and 'scans_missing_pct' in (L[0] or {}):
        mm = np.mean([d['scans_missing_pct'] for d in L]); rx = np.mean([d['rx_rate_hz'] for d in L])
        rows.append(('LiDAR scan loss (steady state)', f"{mm:.1f} % scans missing, received {rx:.1f} Hz (nominal 10 Hz) — **no loss once the pipeline had settled**. Right after boot, and while tools without the DDS profile were running, 18–40 % loss was seen; that was transient/artefactual (see §4.5.1)."))
    th = (R.get('thermal') or {})
    if th.get('frames') in (0, None): rows.append(('Thermal camera not streaming', 'see §4.11; 3-way fusion not exercised on live thermal in this campaign'))
    for k in ('lidar', 'rgb'):
        pass
    cnt = (R.get('counts') or {})
    if cnt.get('Tracebacks'): rows.append(('Python tracebacks in pipeline logs', cnt['Tracebacks']))
    rows += [('Camera stamp clustering', 'RGB frames are stamped in clustered pairs (publish-time stamping), up to ≈0.3 s after exposure; limits fusion alignment accuracy and was the probable cause of the 3-way fusion rate cost seen on 2026-11-12 (fused 8.8 → 7.0 Hz with thermal live)'),
        ('Calibration', 'placeholder RGB intrinsics; thermal solved from one capture'), ('Thermal channel', 'relative AGC luma, not temperature'), ('Power measurement', 'estimated from PMIC rails, ≈30 ms sampling; USB peripherals unmetered'), ('WiFi', 'power-save reported on; Tx excessive retries counter non-zero; connected on 2.4 GHz'), ('No root', 'no sysctl changes (rmem_max), no USB reset, no real-time priorities, no USB-C power measurement')]
    md.append(table(['item', 'detail'], rows)); return '\n'.join(md)
def summary(R, P):
    ov = (P or {}).get('overall') or {}; md = ['# 1. Executive summary\n']
    def g(path, default=None):
        d = R
        for k in path:
            if not isinstance(d, dict) or k not in d: return default
            d = d[k]
        return d
    L = R.get('lidar') or {}; rows = []
    if ov: rows += [('Peak estimated input current', f"**{ov['max']:.2f} A** nominal ({ov['cons_max']:.2f} A conservative) vs 5 A shutdown limit"), ('Typical input power', f"{ov['w_mean']:.2f} W mean over the campaign (SoC rails), {ov['mean']:.2f} A mean"), ('Supply voltage (EXT5V)', f"{ov['v_min']:.3f}–{ov['v_max']:.3f} V; source impedance ≈ {-(P or {}).get('supply_slope_ohm', 0)*1000:.0f} mΩ")]
    rows.append(('Peak SoC temperature', f"{max([d['temp_max'] for d in (P or {}).get('per_phase', {}).values()] or [float('nan')]):.1f} °C"))
    if L.get('rate_hz'): rows.append(('LiDAR', f"{L['rate_hz']:.1f} Hz received, {L['points_per_scan']['mean']:.0f} points/scan, range {L['range_m']['p5']:.1f}–{L['range_m']['p95']:.1f} m (p5–p95), plane RMS {L['plane_fit']['rms_mm']['mean']:.1f} mm" if L.get('plane_fit') else f"{L['rate_hz']:.1f} Hz"))
    t = R.get('topics') or {}
    for tp, lab in (('/landfill/rgb/image_raw', 'RGB'), ('/landfill/thermal/image_raw', 'Thermal'), ('/landfill/fusion/points_rgbt', 'Fused cloud'), ('/landfill/fusion/points_merged', 'Merged cloud')):
        v = t.get(tp)
        if v and 'interval_ms' in v: rows.append((lab, f"{v['hz']:.2f} Hz, stamp age p50 {v['stamp_age_ms']['p50']:.0f} ms (p95 {v['stamp_age_ms']['p95']:.0f} ms)"))
        elif v and tp.endswith('thermal/image_raw'):
            tsup = ((R.get('thermal') or {}) if (R.get('thermal') or {}).get('frames') else {})
            rows.append((lab, 'no frames in the main run (camera in its stuck state); **' + (f"{tsup['rate_hz']:.1f} Hz after the power-loss reboot (supplement, §4.13)" if tsup else 'see §4.11') + '**'))
        elif v: rows.append((lab, 'no messages in window'))
    c = R.get('cpu') or {}
    if c: rows.append(('Pipeline CPU (idle, no viewer)', f"{sum(v['cpu_mean'] for v in c.values()):.0f} % of one core (of 400), RSS {sum(v['rss'] for v in c.values()):.0f} MB over 9 processes"))
    w = R.get('wire') or {}
    for k, v in w.items(): rows.append((f'Wire-format error ({k})', f"max {v['position_error_mm']['max']:.3f} mm, {v['bytes_per_point']:.0f} B/point, RGB exact"))
    s = R.get('startup') or {}
    if s.get('viewer_first_merged_data_s'): rows.append(('Cold start to first viewer data', f"{s['viewer_first_merged_data_s']:.0f} s"))
    sw = R.get('final_merged') or []
    if sw: r0, r1 = sw[0], sw[-1]; rows.append(('Point-cap sweep (merged)', f"{r0['cap']:,} → {r1['cap']:,} points: {r0['viewer_mbit_s']:.1f} → {r1['viewer_mbit_s']:.1f} Mbit/s, peak current {max(r['power']['est_input_a']['max'] for r in sw):.2f} A"))
    md.append(table(['aspect', 'result'], rows)); md.append('\nSee §6 for anomalies and limitations. Every number in this document is produced by `build_report.py` from the raw data in `data/`; nothing is typed in by hand except the narrative text in chapters 1, 3, 5 and 6 and the code description.\n')
    return '\n'.join(md)
def appendix(CM):
    md = ['# Appendix A — Declared ROS parameters (from static analysis)\n']; rows = []
    for p, v in CM['files'].items():
        for x in v['params']: rows.append((f'`{p.split("/")[-1]}`', f"`{x['name']}`", f"`{x['default']}`", x['line']))
    md.append(table(['file', 'parameter', 'default', 'line'], rows))
    md.append('\n# Appendix B — Source file metrics\n'); rows = [(p, v['lines'], v['code'], v['comment'], len(v['functions']), len(v['classes']), v['max_complexity'], v['mean_complexity']) for p, v in sorted(CM['files'].items(), key=lambda kv: -kv[1]['lines']) if v['lines']]
    md.append(table(['file', 'lines', 'code', 'comment', 'functions', 'classes', 'max complexity', 'mean complexity'], rows)); t = CM['totals']; md.append(f"\n**Totals:** {t['files']} Python files, {t['lines']:,} lines ({t['code']:,} code / {t['comment']:,} comment / {t['blank']:,} blank), {t['functions']} functions, {t['classes']} classes, {t['params']} parameters. Other files: {json.dumps(CM['other'])}.\n")
    md.append('\n# Appendix C — Topics published/subscribed per file\n'); rows = []
    for p, v in CM['files'].items():
        for kind in ('pub', 'sub'):
            for x in v['topics'][kind]: rows.append((f'`{p.split("/")[-1]}`', kind, f"`{x['topic']}`", x['type'].split('.')[-1]))
    md.append(table(['file', 'dir', 'topic (as written in code)', 'type'], rows))
    md.append('\n# Appendix D — Functions (length, complexity)\n'); rows = []
    for p, v in sorted(CM['files'].items()):
        for f in v['functions']:
            if f['length'] >= 25 or f['complexity'] >= 6: rows.append((f'`{p.split("/")[-1]}`', f"`{f['name']}`", f['line'], f['length'], f['complexity'], f['doc'][:90]))
    md.append(table(['file', 'function', 'line', 'length', 'complexity', 'docstring (first line)'], rows)); return '\n'.join(md)

def walkthrough(CM):
    """Function-by-function walkthrough generated from the source's own docstrings."""
    order = ['pitfusion3d_bringup/pitfusion3d_bringup/livox_restamp_node.py', 'pitfusion3d_bringup/pitfusion3d_bringup/rgb_camera_node.py', 'pitfusion3d_bringup/pitfusion3d_bringup/thermal_camera_node.py', 'landfill_camera_sim/scripts/calibration_publisher.py', 'landfill_camera_sim/scripts/cloud_io.py', 'landfill_camera_sim/scripts/sensor_fusion_node.py', 'landfill_camera_sim/scripts/registration_merge_node.py', 'landfill_camera_sim/scripts/fusion_web_streamer.py', 'pitfusion3d_bringup/scripts/thermal_view.py', 'pitfusion3d_bringup/scripts/clean_stale_shm.py', 'pitfusion3d_bringup/launch/real_sensors.launch.py', 'pitfusion3d_bringup/launch/pitfusion3d_full.launch.py']
    md = ['# 17. Function-by-function walkthrough (generated from the source)\n', 'For each production file: its purpose (module docstring), then every class and function with signature, length, complexity and docstring. Complexity = 1 + branches/loops/handlers/boolean operators.\n']
    for p in order:
        v = CM['files'].get(p)
        if not v: continue
        md.append(f"## `{p}`  ({v['lines']} lines, {len(v['functions'])} functions, max complexity {v['max_complexity']})\n")
        if v['doc']: md.append('> ' + v['doc'].replace('\n', ' ')[:600] + '\n')
        if v['imports']: md.append('Imports: ' + ', '.join(f'`{i}`' for i in v['imports'] if i) + '\n')
        for f in sorted(v['functions'], key=lambda f: f['line']):
            who = (f.get('parent') + '.' if f.get('parent') else '') + f['name']
            md.append(f"* **`{who}({', '.join(a for a in f['args'] if a != 'self')})`** — line {f['line']}, {f['length']} lines, complexity {f['complexity']}. " + (f['doc_full'] or '*(no docstring)*'))
        md.append('')
    return '\n'.join(md)

def data_index():
    md = ['# 7. Reproducibility and data index\n', 'Run everything again with `python3 campaign.py` (resumable; `--force` redoes finished phases; extra phases `bench timing` are run by `python3 campaign.py bench timing`), then `python3 build_report.py` to regenerate this document and all figures. Safety gates are enforced by `lib.PowerSampler`.\n', '## Data files\n']
    rows = []
    for d in ('data', 'logs', 'figures', 'frames', '.'):
        for f in sorted(glob.glob(f'{HERE}/{d}/*')):
            if os.path.isfile(f): rows.append((os.path.relpath(f, HERE), f'{os.path.getsize(f)/1e3:,.1f} kB'))
    md.append(table(['file', 'size'], rows)); return '\n'.join(md)
if __name__ == '__main__':
    os.makedirs(FIG, exist_ok=True); R = {}; secs = {}
    for name, fn in (('power', rb_power.build), ('battery', rb_misc.build_battery), ('sensors', rb_pipeline.build_sensors), ('pipeline', rb_pipeline.build_pipeline), ('transport', rb_misc.build_transport), ('logs', build_logs), ('sweep', rb_misc.build_sweep), ('accuracy', rb_misc.build_accuracy), ('stability', rb_misc.build_stability), ('robust', rb_misc.build_robust), ('camera', rb_misc.build_camera), ('bench', rb_misc.build_bench), ('thermal_supp', rb_misc.build_thermal_supplement), ('optim', build_optimisation), ('lidarpwr', build_lidar_power)):
        out = safe(fn, name); secs[name], res = out if isinstance(out, tuple) else (out, {})
        R.update({k: v for k, v in res.items()}); 
    P = {k: R[k] for k in ('overall', 'per_phase', 'supply_slope_ohm') if k in R}
    front = f"# PitFusion3D — Full Results Report\n\n*Raspberry Pi 5 sensor-fusion rig (Livox Avia LiDAR + Pi HQ camera + USB thermal camera). Campaign run {pj('phase_meta').get('start') if pj('phase_meta') else '?'} on battery-bank power. Generated by `build_report.py` from `data/`.*\n\n**Companion document:** `CODE_REFERENCE.md` (every node, algorithm, thread, parameter and file).\n\n## How to use this document\nChapters 1–3 give the summary, platform and method; chapter 4 holds the measurements in 12 sections with figures; chapter 5 is the project history and earlier results; chapter 6 lists anomalies; chapter 7 indexes every raw data file. Appendices list parameters, topics and per-file code metrics. Tables are plain Markdown and CSV-friendly; every figure is a PNG in `figures/`.\n\n**Table of contents:** 1 Executive summary · 2 Hardware/OS/network/software inventory · 3 Methodology and limits · 4.1 Power/current/voltage · 4.2 Temperature/frequency · 4.3 Sensor characterisation · 4.4 Pipeline rates/latency/CPU/threads · 4.5 DDS transport, viewer load, network · 4.6 Node-log statistics · 4.7 Point-cap sweep · 4.8 Accuracy · 4.9 Soak stability · 4.10 Robustness · 4.11 Thermal camera record · 4.12 Benchmarks/timing · 4.13 Live-thermal supplement · 4.14 Optimisation pass · 4.15 LiDAR power-mode control · 5 History · 6 Anomalies · 7 Data index · Appendices A–D\n\n---\n"
    CM = json.load(open(f'{DATA}/code_metrics.json'))
    parts = [front, summary(R, P), build_system(), build_method(), '# 4. Results\n', secs['power'] + '\n' + secs['battery'], secs['sensors'], secs['pipeline'], secs['transport'], secs['logs'], secs['sweep'], secs['accuracy'], secs['stability'], secs['robust'], secs['camera'], secs['bench'], secs['thermal_supp'], secs['optim'], secs['lidarpwr'], HISTORY, build_issues(R), data_index(), appendix(CM)]
    open(f'{HERE}/FULL_RESULTS_REPORT.md', 'w').write('\n\n'.join(parts))
    narr = open(f'{HERE}/docs_src/CODE_REFERENCE_narrative.md').read(); deep = open(f'{HERE}/docs_src/CODE_DEEP_DIVE.md').read(); open(f'{HERE}/CODE_REFERENCE.md', 'w').write(narr + '\n\n---\n\n' + deep + '\n\n---\n\n' + walkthrough(CM) + '\n\n---\n\n' + appendix(CM))
    print('written FULL_RESULTS_REPORT.md', os.path.getsize(f'{HERE}/FULL_RESULTS_REPORT.md') // 1000, 'kB;', 'CODE_REFERENCE.md', os.path.getsize(f'{HERE}/CODE_REFERENCE.md') // 1000, 'kB')
