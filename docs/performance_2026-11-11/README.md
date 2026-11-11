# PitFusion3D performance & robustness work, 2026-11-11

Measured on the real rig (Raspberry Pi 5, 4 cores) before and after each change. The rate / CPU / latency tables are generated directly from the raw
JSON/CSV files in [`evidence/measurements/`](evidence/measurements/); the test counts and figures quoted in docs 05-07 are copied from the saved outputs in [`evidence/`](evidence/) (each is linked next to its claim).

## Result in one table (original build -> final build)

| | BEFORE | AFTER (final) |
|---|---|---|
| RGB frames reaching fusion | **3.61 Hz** (camera captured 15) | **10.06 Hz** (captured 10 = LiDAR rate) |
| Live fused cloud | **2.42 Hz**, 77 ms | **10.03 Hz**, 48 ms |
| Merged dense cloud | **0.66 Hz**, ~3 scans / 48,961 pts | **1.53 Hz**, ~7 scans / 99,818 pts (91-100k in most runs; 56k in the soak run) |
| Latency trend over 8 min | n/a | flat (see [RESULTS.md](RESULTS.md#realtime--no-backlog-8-minute-soak)) |
| CPU used per delivered fused cloud | 31.4 %-core per Hz | **13.7** (soak: 12.4) %-core per Hz, ~2.3x more efficient |
| Pipeline CPU (sum of its processes) | 76% of 400 *(starved: 70% of camera frames were being lost)* | 137% of 400 (109-137% across runs) |
| Crash of any node | node stays dead, viewer half-working | respawns in 2-4 s |
| Whole-service crash | restart by hand | auto-restart, data back in ~25 s, no memory leak |
| Thermal camera missing / flaky | **viewer blank** | viewer keeps working (LiDAR+RGB), thermal auto-resumes |

**The one-line finding:** the pipeline was *transport-bound, not CPU-bound*. The Pi's 208 KB kernel UDP buffer was dropping 2.7 MB camera frames between
processes. Fixing that (no root needed) multiplied output ~4x, which briefly pushed the cores to ~84% busy; the remaining changes trimmed wasted load back to
~30% of the machine and made the system crash-proof. Details and evidence per change below.

## Documents

| | |
|---|---|
| [RESULTS.md](RESULTS.md) | all before/after results, per-process CPU, pinning study, soak, **corner-case report summary** |
| [BEST_CONFIGURATION.md](BEST_CONFIGURATION.md) | the configuration selected, and the evidence for each choice (and what was rejected) |
| [01_shared_memory_transport.md](01_shared_memory_transport.md) | DDS transport fix (the big one) |
| [02_merge_worker_thread_and_realtime_queue.md](02_merge_worker_thread_and_realtime_queue.md) | threaded ICP, no backlog by design |
| [03_streamer_on_demand_encoding_and_404_fix.md](03_streamer_on_demand_encoding_and_404_fix.md) | viewer server efficiency + HTTP bug |
| [04_rgb_10fps_and_cpu_pinning_study.md](04_rgb_10fps_and_cpu_pinning_study.md) | RGB rate, CPU affinity tested and rejected, GPU note |
| [05_thermal_resilience_and_fusion_fallback.md](05_thermal_resilience_and_fusion_fallback.md) | USB thermal camera faults, graceful degradation |
| [06_crash_recovery_respawn_and_shm_cleanup.md](06_crash_recovery_respawn_and_shm_cleanup.md) | node respawn, orphan camera process, shared-memory leak |
| [07_corner_case_report.md](07_corner_case_report.md) | every corner-case test, what failed, what was fixed, what is NOT covered |

## Reproduce
`evidence/harness/run_case.sh <label> 120 [viewer]` measures a run (per-process/per-core CPU, topic rates, latency); `evidence/transport/transport_bench.py`
is the transport A/B; `evidence/corner_cases/t_*.py` are the isolated-domain test suites. Code changes: [`evidence/code/code_changes_vs_HEAD.diff`](evidence/code/code_changes_vs_HEAD.diff).

## Caveats that apply to everything
- Harness and test scripts contain absolute paths for this Pi (`/home/vish/...`); adjust before reusing elsewhere.
- The **thermal camera was down for every measurement** (a USB hardware fault, see doc 05), so fusion ran LiDAR+RGB; thermal decode cost (a few ms) is excluded.
- ~150% of the system CPU in each run is desktop apps (VS Code, browser, GNOME); a headless field unit will be lighter.
- The viewer was a simulated poller at the real page's cadence over loopback, **not** real WiFi radio load. Single runs, one machine, run-to-run noise is real (see RESULTS).
