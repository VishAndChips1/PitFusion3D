# PitFusion3D — final results folder (2026-11-12)

Everything produced by the full measurement campaign, the code documentation and the thesis figures. Start here.

| What | File / folder | Size / contents |
|---|---|---|
| **Main results report** (all measurements, 4 chapters of results, history, anomalies, appendices) | [`FULL_RESULTS_REPORT.md`](FULL_RESULTS_REPORT.md) | ≈ 160 kB; 25+ figures, ≈ 300 tables |
| **Code reference** (architecture, every node, threading, state machines, algorithms, failure handling, tests, function-by-function walkthrough, parameters) | [`CODE_REFERENCE.md`](CODE_REFERENCE.md) | ≈ 75 kB |
| **Thesis figures and infographics** (20 figures, PNG + vector PDF, with suggested captions) | [`thesis_figures/`](thesis_figures/) · captions in [`thesis_figures/CAPTIONS.md`](thesis_figures/CAPTIONS.md) | F01 architecture … F20 sensor characterisation |
| **LiDAR power-mode control** (how the Pi puts the Livox Avia into low-power mode and wakes it) | [`LIDAR_POWER_MODE.md`](LIDAR_POWER_MODE.md) · tool in `~/lidar_ctl/` · command `~/lidar` | tested on the real unit |
| Figures used inside the report | [`figures/`](figures/) · [`frames/`](frames/) | |
| Raw data | [`data/`](data/) | `power_raw.csv` (all PMIC rails, ≈ 30 Hz, 1.8 h), `system_1hz.csv` (per-process CPU/RSS/threads, per-core, memory, clocks, network counters), `markers.jsonl` (phase boundaries), one `phase_*.json` per test phase, `inventory.json` (77 system queries), `code_metrics.json`, saved point-cloud samples (`sample_*.npz`) and stamp traces (`stamps_*.npz`) |
| Logs | [`logs/`](logs/) | campaign log, corner-case suite outputs, regression runs after the code clean-up |
| Sweep results | `results_final_merged.jsonl`, `results_final_live.jsonl`, `power_raw_final_*.csv`, `sweep_final_*.log` | earlier runs kept in `sweep_run1_nofidelity/`, `sweep_run2_disturbed/` |
| Test and measurement code | `campaign.py` (orchestrator), `lib.py`, `probe_topics.py`, `chars.py`, `threads.py`, `http_bench.py`, `net_bench.py`, `accuracy.py`, `transport_test.py`, `lidar_loss.py`, `stress.py`, `sweep.py`, `inventory.py`, `code_metrics.py`, `build_report.py`, `rb_*.py`, `make_thesis_figures.py`, `corner/` | everything is re-runnable |

## Re-running
```bash
python3 campaign.py                 # whole campaign, unattended and resumable (safety gates: 4.4 A / 4.75 V / 82 C)
python3 campaign.py thermal_supplement timing recovery restart_timing bench final   # the second-pass phases
python3 build_report.py             # regenerate FULL_RESULTS_REPORT.md, CODE_REFERENCE.md and report figures
python3 make_thesis_figures.py      # regenerate the 20 thesis figures
```

## The most important results in ten lines
1. Peak estimated input current **2.02 A** (2.88 A conservative) against the 5 A shutdown limit; mean power 4.1 W; supply held ≥ 4.84 V; SoC peak 75.5 °C (cooler room — treat temperatures as a lower bound).
2. LiDAR 10.0 Hz (24 000 points/scan), fused cloud 9.8–10 Hz, merged cloud 1.5 Hz; viewer data 13 s after start; 16 concurrent viewer clients with flat latency and no errors.
3. Pipeline cost ≈ 94 % of one core (of 400) and 1.5 GB over 9 processes; no memory leak in a 30-minute soak; no tracebacks in the whole campaign.
4. Viewer wire format: 11 bytes/point, **0.14 mm** maximum position error, colours exact. LiDAR plane-fit precision ≈ 16 mm (upper bound).
5. Robustness: 7/7 nodes recover from SIGKILL in 2 s; all corner-case suites pass (one documented edge case); **a real +7 333 s NTP clock step** hit the running pipeline and was handled with no stall.
6. Optimisation found today: `rpicam-vid --flush` makes RGB stamps perfectly regular (std 79 → 5 ms) and tightens LiDAR↔colour alignment 3× (p50 34 → 12 ms).
7. Thermal camera: works (30 Hz) after a full power loss; its stuck-state loop only ever cleared that way; a watchdog now detects it and can auto-recover once its one-time permission step is installed (done).
8. LiDAR can be put in low-power mode from the Pi (`~/lidar low`, confirmed by the sensor in 0.9 s; wake-up 9.7 s) and stays there when the host disconnects.
9. The rig room's WiFi was poor (−67 dBm, <0.5 Mbit/s): the radio, not the Pi, limits what a remote viewer sees.
10. The 2-hour power-bank session ended by an abrupt cut-off with no voltage warning (supply flat at 5.0 V until it died).
