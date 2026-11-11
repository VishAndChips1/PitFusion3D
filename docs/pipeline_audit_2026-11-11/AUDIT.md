# Pipeline audit (2026-11-11): optimisation, efficiency, layout, threading

Scope: review of the running system plus micro-benchmarks. Measured on the live Pi (4 cores, 8 GB) while the pipeline ran with the 20k/5k caps.

## 1. Process and thread layout (snapshot)
| Process | Threads | CPU % of one core | RSS MB | Role |
|---|---|---|---|---|
| registration_merge_node | 15 | 33 | 263 | executor + 1 ICP worker thread (queue size 1, replace-stale) |
| sensor_fusion_node | 14 | 24 | 345 | executor; 2 synchronisers (3-way, and LiDAR+RGB fallback) |
| rgb_camera_node (+ rpicam-vid) | 15 (+8) | 19 (+4) | 256 | capture thread + executor; camera child process |
| fusion_web_streamer | 15 | 16 | 287 | executor + HTTP server thread + one thread per client |
| livox driver | 17 | 13 | 110 | vendor driver |
| livox_restamp_node | 14 | 6 | 148 | restamp |
| thermal / calibration | 14-15 | under 1 | 163-240 | thermal currently has no device |

**Verdict:** every node is its own process, so there is no Python GIL contention between nodes. The work is spread over the 4 cores; the busiest process uses about a third of one core (total about 1.1 to 1.3 cores of 4, consistent with the sweep). No process is a hot spot, and there is no cross-process backlog (merge uses replace-stale; the streamer encodes lazily).

## 2. Threading review
- **Merge node:** ICP runs in a worker thread fed by `queue.Queue(maxsize=1)`; a full queue drops the stale item, so the callback never blocks. Shared state is limited to that queue. Correct.
- **Streamer `_StreamState`:** `offer()` is lock-free (message then counter). `refresh()` reads under `encode_lock` and re-checks the counter. I walked the interleavings: the worst case is one redundant re-encode of the same cloud, never a stale or torn payload. `set_payload` swaps the buffer and sequence number together under `lock`, so readers always get a matching pair. Correct. The `encode_lock` also makes concurrent clients share one encode instead of repeating it.
- **Param callback (runtime caps):** validates all values before applying any, and the caps are plain ints read once per encode. Correct.
- **Camera nodes:** capture runs in a daemon thread with a try/except around processing, so one bad frame cannot silently kill the thread. rpicam-vid has PDEATHSIG, so it cannot be orphaned.
- **Known cost:** HTTP handler threads do the encode, so a very slow encode would be paid on a client request thread. Measured encode is small (below), so this is acceptable.

## 3. Micro-benchmarks (110k-point cloud, this Pi)
| Item | Result | Decision |
|---|---|---|
| `pc2.read_points_numpy` vs `cloud_io.read_fields` | 1.2 ms vs 0.03 ms, identical output | Negligible in absolute terms; not worth a change |
| `_encode_cloud` at cap 20k / 60k / 105k | 15 / 42 / 90 ms | Fine (1.5 clouds/s means about 7 to 14% of a core at the larger caps) |
| gzip level 1 / 3 / 6 at 60k points | 57 / 33 / 47 ms (timing noisy, test data incompressible) | Level 6 kept; level 3 would save a little CPU only. Measure on a real viewer session before changing |
| Random subsampling (`rng.choice`) at 60k | 3 ms | Fine |

**No change was made to the code in this audit.** The hot paths measured are not bottlenecks; the system-level limit is the ICP merge rate (about 1.5 Hz) and, per viewer, the WiFi link.

## 4. Findings and open items (honest list)
1. **Fast DDS shared memory uses about 620 MB RAM** (9 participants x 64 MB segments). Available RAM is 3.4 GB, so it is harmless today. Halving the segment size (32 MB still holds more than 10 of the largest messages) would save about 300 MB, but it needs a regression test; not changed.
2. **Stale SHM files:** a dry run of `clean_stale_shm.py` found 62 stale small files (12 MB), left by short-lived test processes. They are cleaned at every service start, so they cannot accumulate across restarts. Low priority.
3. **Clock step after boot (no RTC):** earlier testing showed the running pipeline can stall when NTP steps the clock. Mitigation proposed, **not yet implemented**: (a) reset the restamp window when the offset jumps by more than 0.5 s, (b) a watchdog in `run_pipeline.sh` that restarts the stack once after the first big clock step. Needs its own test.
4. **Thermal camera:** a USB hardware fault remains. Fusion falls back to LiDAR+RGB after `thermal_timeout_s`, so the pipeline keeps running, but the thermal channel is empty.
5. **Live code is not in git:** `~/ros2_ws/src` is the running copy and the repo holds a mirror. A drift-check script would prevent them diverging. Not yet written.
6. **Sweep recommendation not applied:** the service still runs 20k merged / 5k live. See `docs/pointcap_sweep_2026-11-11/POINTCAP_SWEEP_RESULTS.md`.
