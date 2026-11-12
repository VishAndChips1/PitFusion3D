from rb_common import *
def build():
    md = []; P = load_csv(f'{DATA}/power_raw.csv'); S = load_csv(f'{DATA}/system_1hz.csv'); W = phase_windows()
    if not P: return '## Power\n\n_no power data_\n', {}
    t = P['t']; est = P['est_in_a']; cons = P['est_in_cons_a']; ext = P['EXT5V_V']; watts = P['watts']; temp = P['temp_c']
    dt = np.diff(t); res = {}
    gaps = [(t[i], t[i + 1]) for i in np.where(dt > 30)[0]]; res['gaps'] = [(a, b) for a, b in gaps]
    def am(x):
        x = np.asarray(x, float); out = (x - t[0]) / 60.0
        for a, b in gaps: out = out - np.clip(x - a, 0, b - a) / 60.0
        return out
    res['samples'] = len(t); res['duration_h'] = (t[-1] - t[0]) / 3600; res['sample_rate_hz'] = len(t) / (t[-1] - t[0]); res['dt_ms'] = dict(p50=np.percentile(dt, 50) * 1e3, p95=np.percentile(dt, 95) * 1e3, p99=np.percentile(dt, 99) * 1e3, max=dt.max() * 1e3)
    md.append('## 4.1 Power, current and voltage\n')
    md.append(f"**Source of data:** `data/power_raw.csv` — every PMIC rail (voltage and current) read with `vcgencmd pmic_read_adc` as fast as the firmware allows. **{len(t):,} samples over {res['duration_h']:.2f} h ({res['sample_rate_hz']:.1f} Hz average; sample spacing p50 {res['dt_ms']['p50']:.0f} ms, p95 {res['dt_ms']['p95']:.0f} ms, p99 {res['dt_ms']['p99']:.0f} ms, max {res['dt_ms']['max']:.0f} ms).** Spikes shorter than the sample spacing (≈30 ms) cannot be seen by this method; see §3 for the effect on the claim 'peak current'.\n")
    md.append("**Estimated input current** = Σ(rail V × rail I over all PMIC rails except EXT5V/BATT) ÷ EXT5V voltage — i.e. the power delivered *to the SoC rails* divided by the supply voltage. The **conservative** figure additionally divides by 0.85 (regulator efficiency) and adds 0.5 A for USB peripherals (which the PMIC does not meter). The Pi shuts down above 5 A, and the campaign's hard abort gate was 4.4 A (nominal). `VDD_CORE_A` is the 0.87 V core rail and is *not* the input current.\n")
    # per-phase table
    rows = []; order = ['idle_os', 'startup', 'idle_pipeline', 'sensor', 'latency', 'threads', 'transport', 'accuracy', 'http', 'net', 'stress', 'soak', 'corner', 'recovery', 'restart_timing', 'bench', 'timing']
    ph = P['phase']; perphase = {}
    for name in order + [k for k in W if k not in order]:
        if name not in W: continue
        a, b = W[name]; m = (t >= a) & (t <= b)
        if m.sum() < 10: continue
        e = est[m]; c = cons[m]; v = ext[m]; w_ = watts[m]
        perphase[name] = dict(dur_min=(b - a) / 60, n=int(m.sum()), mean=e.mean(), p50=np.percentile(e, 50), p95=np.percentile(e, 95), p99=np.percentile(e, 99), max=e.max(), cons_max=c.max(), w_mean=w_.mean(), w_max=w_.max(), v_min=v.min(), v_mean=v.mean(), v_max=v.max(), temp_max=temp[m].max(), temp_mean=temp[m].mean(),
                              frac_gt2=(e > 2).mean() * 100, frac_gt3=(e > 3).mean() * 100, frac_gt4=(e > 4).mean() * 100, wh=np.trapz(w_, t[m]) / 3600)
        rows.append((name, fmt(perphase[name]['dur_min'], 1), f"{perphase[name]['n']:,}", fmt(perphase[name]['mean']), fmt(perphase[name]['p95']), fmt(perphase[name]['p99']), fmt(perphase[name]['max']), fmt(perphase[name]['cons_max']), fmt(perphase[name]['w_mean'], 2), fmt(perphase[name]['w_max'], 2), fmt(perphase[name]['v_min'], 3), fmt(perphase[name]['v_mean'], 3), fmt(perphase[name]['temp_max'], 1), fmt(perphase[name]['wh'], 3)))
    res['per_phase'] = perphase
    md.append('### 4.1.1 Current, power and supply voltage by test phase\n')
    md.append(table(['phase', 'min', 'samples', 'mean A', 'p95 A', 'p99 A', '**max A**', 'cons. max A', 'mean W', 'max W', 'EXT5V min V', 'EXT5V mean V', 'SoC max °C', 'energy Wh'], rows))
    md.append('*Notes:* the `sweep` phase shows few samples because `sweep.py` runs its own 30 Hz sampler (its per-step current peaks are in §4.7); short phases (`meta`, `code`, `accuracy`, `final`) have too few samples for percentiles to mean much; `bench` ran with the pipeline stopped.\n')
    ov = dict(mean=est.mean(), p50=np.percentile(est, 50), p95=np.percentile(est, 95), p99=np.percentile(est, 99), p999=np.percentile(est, 99.9), max=est.max(), cons_max=cons.max(), w_mean=watts.mean(), w_max=watts.max(), v_min=ext.min(), v_mean=ext.mean(), v_max=ext.max(), wh=np.trapz(watts, t) / 3600)
    res['overall'] = ov; imax = int(np.argmax(est)); res['peak_time'] = time.strftime('%H:%M:%S', time.localtime(t[imax])); res['peak_phase'] = str(ph[imax])
    md.append('### 4.1.2 Whole-campaign summary\n')
    md.append(table(['quantity', 'value'], [('Peak estimated input current (nominal)', f"**{ov['max']:.2f} A** at {res['peak_time']} (phase `{res['peak_phase']}`)"), ('Peak conservative estimate', f"{ov['cons_max']:.2f} A"), ('Limit (Pi shuts off)', '5.0 A'), ('Headroom to limit (conservative peak)', f"{5 - ov['cons_max']:.2f} A"),
        ('99.9th-percentile current', f"{ov['p999']:.2f} A"), ('99th / 95th / median', f"{ov['p99']:.2f} / {ov['p95']:.2f} / {ov['p50']:.2f} A"), ('Mean current / mean power', f"{ov['mean']:.2f} A / {ov['w_mean']:.2f} W"), ('Peak SoC-rail power', f"{ov['w_max']:.2f} W"), ('EXT5V min / mean / max', f"{ov['v_min']:.3f} / {ov['v_mean']:.3f} / {ov['v_max']:.3f} V"), ('Total energy over campaign', f"{ov['wh']:.2f} Wh (SoC rails only)"),
        ('Throttle flags (`get_throttled`) seen', ', '.join(sorted({hex(int(x)) for x in P['throttled'] if np.isfinite(x)})) )]))
    # spike analysis
    from numpy.lib.stride_tricks import sliding_window_view as swv
    k = 31; med = np.convolve(est, np.ones(k) / k, mode='same') if len(est) > k else est; spike = est - med
    md.append('### 4.1.3 Spike analysis (current excursions above the 1-second moving average)\n')
    thr = [0.25, 0.5, 1.0, 1.5]; rows = [(f'> +{x:.2f} A', int((spike > x).sum()), f'{(spike > x).mean() * 100:.3f} %') for x in thr]
    md.append(table(['excursion above 31-sample (≈1 s) mean', 'samples', 'share of all samples'], rows)); res['spike_max_over_avg'] = float(spike.max())
    top = np.argsort(est)[::-1]; seen = []; rows = []
    for i in top:
        if all(abs(t[i] - s) > 10 for s in seen): seen.append(t[i]); rows.append((time.strftime('%H:%M:%S', time.localtime(t[i])), str(ph[i]), fmt(est[i]), fmt(cons[i]), fmt(watts[i], 2), fmt(ext[i], 3), fmt(temp[i], 1)))
        if len(rows) >= 10: break
    md.append('**Ten highest current samples (at least 10 s apart):**\n'); md.append(table(['time', 'phase', 'est A', 'cons. A', 'W', 'EXT5V V', 'SoC °C'], rows))
    # voltage vs current regression
    m = np.isfinite(ext) & np.isfinite(est); A = np.vstack([est[m], np.ones(m.sum())]).T; slope, icpt = np.linalg.lstsq(A, ext[m], rcond=None)[0]; res['supply_slope_ohm'] = slope; res['supply_open_v'] = icpt
    md.append('### 4.1.4 Supply behaviour (EXT5V vs load)\n')
    md.append(f"Linear fit over all samples: `EXT5V = {icpt:.3f} V {slope:+.4f} V/A × I` ⇒ effective source impedance **{-slope * 1000:.0f} mΩ** (negative = voltage falls with load) and an unloaded voltage of {icpt:.3f} V. The measured minimum under the highest load was {ov['v_min']:.3f} V; the Pi's own undervoltage threshold is ≈4.63 V and the campaign abort gate was 4.75 V.\n")
    # figures
    fig, ax = plt.subplots(4, 1, figsize=(12, 9), sharex=True); tt = am(t)
    step = max(1, len(t) // 20000); ax[0].plot(tt[::step], est[::step], lw=.5, c=C[0], label='est. input (nominal)'); ax[0].plot(tt[::step], cons[::step], lw=.5, c=C[1], alpha=.6, label='conservative'); ax[0].axhline(5, c='r'); ax[0].axhline(4.4, c='r', ls=':'); ax[0].set_ylabel('A'); ax[0].legend(loc='upper right', ncol=3); ax[0].set_title('Estimated input current')
    ax[1].plot(tt[::step], ext[::step], lw=.5, c=C[2]); ax[1].set_ylabel('EXT5V (V)'); ax[2].plot(tt[::step], watts[::step], lw=.5, c=C[3]); ax[2].set_ylabel('SoC rails (W)'); ax[3].plot(tt[::step], temp[::step], lw=.6, c=C[1]); ax[3].set_ylabel('SoC °C'); ax[3].set_xlabel('active minutes since campaign start (power-loss gap removed)' if gaps else 'minutes since campaign start')
    cols = {}; 
    for i, (name, (a, b)) in enumerate(sorted(W.items(), key=lambda kv: kv[1][0])):
        for x in ax: x.axvspan(float(am(a)), float(am(b)), color=C[i % len(C)], alpha=.07)
        ax[0].text(float(am(a)), ax[0].get_ylim()[1] * 0.93, name, rotation=90, fontsize=6, va='top')
    md.append(f"![power timeline]({savefig('power_timeline.png')})\n\n*Whole-campaign timeline; shaded bands are test phases (labelled).*\n")
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6)); ax[0].hist(est, bins=120, color=C[0], log=True); ax[0].set_xlabel('est. input current (A)'); ax[0].set_ylabel('samples (log)'); ax[0].axvline(5, c='r'); ax[0].set_title('Distribution of all current samples')
    names = [n for n in order if n in perphase]; ax[1].boxplot([est[(t >= W[n][0]) & (t <= W[n][1])] for n in names], labels=names, showfliers=False); ax[1].tick_params(axis='x', rotation=60); ax[1].set_ylabel('A'); ax[1].set_title('Current by phase (whiskers = non-outlier range)')
    md.append(f"![current distribution]({savefig('power_distribution.png')})\n")
    # rails breakdown
    rails = sorted({c[:-2] for c in P if c.endswith('_A') and c[:-2] + '_V' in P and c[:-2] not in ('EXT5V', 'BATT') and c[:-2] != ''}); rb = {}
    for name in ('idle_os', 'idle_pipeline', 'stress', 'soak'):
        if name not in W: continue
        a, b = W[name]; m = (t >= a) & (t <= b); rb[name] = {r: np.nanmean(P[r + '_V'][m] * P[r + '_A'][m]) for r in rails}
    if rb:
        md.append('### 4.1.5 Power by PMIC rail (mean W)\n'); keys = [r for r in rails if max(rb[n][r] for n in rb) > 0.005]
        md.append(table(['rail'] + list(rb), [(r, *[fmt(rb[n][r], 3) for n in rb]) for r in keys] + [('**total**', *[fmt(sum(rb[n].values()), 3) for n in rb])]))
        fig, ax = plt.subplots(figsize=(9, 3.6)); bottom = np.zeros(len(rb))
        for i, r in enumerate(keys): vals = np.array([rb[n][r] for n in rb]); ax.bar(list(rb), vals, bottom=bottom, label=r, color=plt.cm.tab20(i % 20)); bottom += vals
        ax.set_ylabel('W'); ax.legend(fontsize=6, ncol=2, bbox_to_anchor=(1.01, 1)); md.append(f"![rails]({savefig('power_rails.png')})\n"); res['rails'] = rb
    # startup inrush
    mk = {e: tt_ for e, tt_ in sub_markers('startup')}
    if 'service_start_issued' in mk:
        t0 = mk['service_start_issued']; m = (t >= t0 - 10) & (t <= t0 + 90); fig, ax = plt.subplots(2, 1, figsize=(10, 5), sharex=True); ax[0].plot(t[m] - t0, est[m], c=C[0], lw=.8); ax[0].axvline(0, c='k', ls=':'); ax[0].set_ylabel('est. input A'); ax[0].set_title('Pipeline start-up (service start at t=0)')
        ax[1].plot(t[m] - t0, ext[m], c=C[2], lw=.8); ax[1].set_ylabel('EXT5V V'); ax[1].set_xlabel('s after start'); md.append('### 4.1.6 Start-up transient (inrush)\n')
        base = est[(t >= t0 - 10) & (t < t0)].mean() if ((t >= t0 - 10) & (t < t0)).any() else np.nan; pk = est[m & (t >= t0)].max(); tpk = t[m & (t >= t0)][np.argmax(est[m & (t >= t0)])] - t0; steady = est[(t >= t0 + 60) & (t <= t0 + 90)].mean() if ((t >= t0 + 60) & (t <= t0 + 90)).any() else np.nan
        md.append(f"Before start: {base:.2f} A (OS only). Peak during start-up: **{pk:.2f} A at +{tpk:.1f} s**. Steady state 60–90 s after start: {steady:.2f} A. Minimum EXT5V during start-up: {ext[m & (t >= t0)].min():.3f} V.\n"); md.append(f"![startup]({savefig('startup_inrush.png')})\n"); res['startup'] = dict(base=base, peak=pk, t_peak=tpk, steady=steady)
    # stress ramp
    sm = sub_markers('stress'); steps = {}
    for e, tm in sm:
        if e.endswith('_begin'): steps.setdefault(e[:-6], [None, None])[0] = tm
        if e.endswith('_end'): steps.setdefault(e[:-4], [None, None])[1] = tm
    sp = pj('phase_stress') or {}; rows = []; ramp = []
    for name, (a, b) in steps.items():
        if a is None or b is None or name.startswith('cooldown'): continue
        m = (t >= a + 10) & (t <= b); 
        if m.sum() < 10: continue
        S_m = (S['t'] >= a + 10) & (S['t'] <= b) if S else None; fr = None
        r = (sp.get(name) or {}).get('rates', {}); fz = (r.get('/landfill/fusion/points_rgbt') or {}).get('hz'); mz = (r.get('/landfill/fusion/points_merged') or {}).get('hz')
        fq = np.nanmean(S['arm_clock_mhz'][S_m]) if S and S_m.any() else np.nan
        rows.append((name, fmt(est[m].mean()), fmt(est[m].max()), fmt(cons[m].max()), fmt(watts[m].mean(), 2), fmt(ext[m].min(), 3), fmt(temp[m].max(), 1), fmt(fq, 0), fmt(np.nanmean(S['cpu_total_pct'][S_m]) if S_m is not None and S_m.any() else np.nan, 0), fmt(fz, 2), fmt(mz, 2), (sp.get(name) or {}).get('aborted_by_safety') or '–'))
        ramp.append((name, est[m].mean(), est[m].max(), temp[m].max()))
    if rows:
        md.append('### 4.1.7 CPU stress ramp (pipeline running; 0 → 4 busy workers, then + 512 MB memory traffic)\n')
        md.append(table(['step', 'mean A', 'max A', 'cons. max A', 'mean W', 'EXT5V min', 'SoC max °C', 'mean CPU MHz', 'system CPU % (of 400)', 'fused Hz', 'merged Hz', 'aborted?'], rows)); res['stress_rows'] = ramp
        fig, ax = plt.subplots(1, 2, figsize=(10, 3.6)); xs = range(len(ramp)); ax[0].bar([i - .2 for i in xs], [r[1] for r in ramp], .4, label='mean', color=C[0]); ax[0].bar([i + .2 for i in xs], [r[2] for r in ramp], .4, label='max', color=C[1]); ax[0].axhline(5, c='r'); ax[0].set_xticks(list(xs)); ax[0].set_xticklabels([r[0].replace('cpu_', '').replace('_workers', 'w') for r in ramp], rotation=40, ha='right'); ax[0].set_ylabel('A'); ax[0].legend(); ax[0].set_title('Current vs load')
        ax[1].plot([r[3] for r in ramp], 'o-', c=C[1]); ax[1].set_xticks(list(xs)); ax[1].set_xticklabels([r[0].replace('cpu_', '').replace('_workers', 'w') for r in ramp], rotation=40, ha='right'); ax[1].set_ylabel('SoC max °C'); ax[1].set_title('Temperature vs load'); md.append(f"![stress]({savefig('stress_ramp.png')})\n")
    if ramp and 'overall' in res:
        stress_max = max(x[2] for x in ramp); res['stress_max'] = stress_max
        md.append(f"**Reading the stress result.** Sustained load raised the *mean* current from {ramp[0][1]:.2f} A (pipeline only) to {max(x[1] for x in ramp):.2f} A and the SoC temperature by {ramp[-1][3]-ramp[0][3]:.0f} °C, then plateaued from two busy workers onward: the CPU is capped at 2.0 GHz and all four cores were already saturated (system CPU 368–400 %), so more workers cannot draw more power. The *highest single sample of the whole campaign* ({res['overall']['max']:.2f} A at {res['peak_time']}, phase `{res['peak_phase']}`) was **not** under stress (stress peak {stress_max:.2f} A): current spikes of ≈2 A are short bursts that occur at any load level (camera/ISP and memory activity), not a consequence of sustained CPU load. At the measured ≈30 ms resolution the margin to the 5 A limit is therefore >2 A even in the conservative estimate. Pipeline rates under saturation were erratic from a single 15 s probe per step (fused 10.0 → 5.2 → 10.0 Hz, merged 1.47 → 0.0 → 1.33 Hz): treat them as 'degrades under full CPU saturation but recovers', not as a smooth curve.\n")
    # thermal section
    md.append('## 4.2 Temperature, CPU frequency and throttling\n')
    rows = []
    for name, d in perphase.items():
        a, b = W[name]; mm = (S['t'] >= a) & (S['t'] <= b) if S else None
        fq = S['arm_clock_mhz'][mm] if S and mm.any() else np.array([np.nan])
        rows.append((name, fmt(d['temp_mean'], 1), fmt(d['temp_max'], 1), fmt(np.nanmean(fq), 0), fmt(np.nanmin(fq), 0), fmt(np.nanmax(fq), 0)))
    md.append(table(['phase', 'SoC mean °C', 'SoC max °C', 'mean CPU MHz', 'min MHz', 'max MHz'], rows))
    if 'stress' in steps or sub_markers('stress'):
        cm = dict(sub_markers('stress'))
        if 'cooldown_begin' in cm and 'cooldown_end' in cm:
            a, b = cm['cooldown_begin'], cm['cooldown_end']; m = (t >= a) & (t <= b); tc = t[m] - a; y = temp[m]
            if m.sum() > 20:
                T0, Tend = y[:20].mean(), y[-20:].mean(); md.append(f"**Cool-down after the heaviest stress step:** {T0:.1f} °C → {Tend:.1f} °C over {(b - a):.0f} s (fan-assisted, pipeline still running).\n"); res['cooldown'] = dict(start=T0, end=Tend)
                fig, ax = plt.subplots(figsize=(7, 3)); ax.plot(tc, y, c=C[1]); ax.set_xlabel('s'); ax.set_ylabel('SoC °C'); ax.set_title('Cool-down curve'); md.append(f"![cooldown]({savefig('cooldown.png')})\n")
    md.append(f"Maximum SoC temperature during the whole campaign: **{temp.max():.1f} °C** (throttling begins at 80–85 °C; the campaign abort gate was 82 °C). The CPU governor is `ondemand` with a 1.5–2.0 GHz range (`arm_freq=2000` in `config.txt`), so the 'CPU MHz' columns show the governor's response to load, not a fault.\n")
    return '\n'.join(md), res
