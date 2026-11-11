# Best configuration: what was selected and why

Selected from the measurements in [RESULTS.md](RESULTS.md). All code is in this branch; originals are in git history (base commit) and the diff is [`evidence/code/code_changes_vs_HEAD.diff`](evidence/code/code_changes_vs_HEAD.diff).

| setting | selected | previously | evidence | alternatives rejected |
|---|---|---|---|---|
| DDS transport | **shared memory, 64 MB segment** + UDP 4 MB buffers (`fastdds_large_buffers.xml`) | UDP-only (kernel-capped at 208 KB) | RGB 3.6 -> 10.1 Hz received; transport A/B 3.4 -> 14.4 Hz ([doc 01](01_shared_memory_transport.md)) | default (3.40 Hz); original UDP (3.44 Hz); `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` (RGB 6.44 vs 7.00 Hz); SHM 128 MB (same rates, 1.1 GB RAM); SHM 32 MB (thermal 26.6 vs 28.9 Hz) |
| RGB frame rate | **10 fps** | 15 fps | rgb_node 28% -> 19%, fused unchanged ([doc 04](04_rgb_10fps_and_cpu_pinning_study.md)) | 15 fps (frames above the 10 Hz LiDAR are never fused) |
| Merge execution | **worker thread**; 1-window queue, **replace stale**, drop > 1.0 s old, NaN filter at intake | ICP inline in the callback | no throughput change (1.49 -> 1.50 Hz); bounded latency under 4x overload (1.8 s peak, 0.38 s after) ([doc 02](02_merge_worker_thread_and_realtime_queue.md)) | drop-newest (would publish stale windows); unbounded queue (backlog) |
| Streamer encoding | **on demand**, newest cloud only | every cloud at 10 Hz regardless | streamer 26% -> 6% idle ([doc 03](03_streamer_on_demand_encoding_and_404_fix.md)) | encode-always |
| Point caps | **unchanged** 20,000 merged / 5,000 live | same | set from earlier real-WiFi power tests; **not re-validated** at the new fused rate | - |
| CPU affinity | **none** (scheduler decides) | none | 3 pinned vs 3 unpinned runs, no consistent gain, one collapse ([doc 04](04_rgb_10fps_and_cpu_pinning_study.md)) | role pinning (LiDAR/cameras/fusion+streamer/merge on 4 cores) |
| GPU | **not used** | not used | VideoCore VII cannot accelerate the numpy/scipy stages; ISP already used | - |
| Fusion fallback | **LiDAR+RGB when 3-way fusion fails > 1.0 s**; auto-resume | none (blank viewer) | 3.0 s: 67% -> 1.0 s: 89% (0.5 s: 94%) of scans on realistic dropouts ([doc 05](05_thermal_resilience_and_fusion_fallback.md)) | 3.0 s (stalls); 0.5 s (flicker risk, unmeasured) |
| Thermal node recovery | **wait 3 s for stable device; cool-down 3 -> 20 s** | reopen immediately | no more reopen storms; hardware fault itself remains | immediate reopen |
| Node supervision | **`respawn=True`, 2 s** on all 8 nodes | none | 8/8 respawn in 2-4 s ([doc 06](06_crash_recovery_respawn_and_shm_cleanup.md)) | - |
| Camera child process | **dies with its node** + stale cleanup | could be orphaned | stale `rpicam-vid` observed | - |
| Shared-memory hygiene | **stale-segment cleanup on every start** | none (would leak 8 x 64 MB per crash) | 821 MB leaked -> 0; flat 542 MB across 2 hard kills | - |
| Service restart policy | unchanged (`Restart=on-failure`, 5 s) | same | auto-restart verified: data back in ~25 s | - |
| Sync / ICP / window params | unchanged (slop 0.15 s, queue 5, window 0.5 s, ICP voxel 0.2 m, 10 it, caps 1500 / 10000) | same | not varied here | - |

## Resulting operating point (final build, 1 viewer)
LiDAR 10.1 Hz, RGB 10.1 Hz, fused **10.03 Hz** at 48 ms, merged **1.53 Hz**, pipeline 137% of 400 (109-137% across runs), 69.2 C, no throttling, ~0.54 GB shared memory.
Headroom: the pipeline is ~30% of the machine; the rest in these runs was desktop apps.

## Caveats
Thermal camera down for every measurement; simulated viewer; single runs with visible run-to-run noise ([RESULTS.md](RESULTS.md#excluded--unusual-runs-kept-in-evidence-not-hidden)); thermal-mode viewer behaviour with NaN thermal not checked in a browser.
