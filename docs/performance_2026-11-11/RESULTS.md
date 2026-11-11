# Results: before and after

Rates are received by an independent subscriber (loading the same DDS profile as the pipeline). Latency = receive time minus message header stamp.
Source data: [`evidence/measurements/`](evidence/measurements/) (`*.json` summary, `*_rates.csv` per-10 s windows, `summary_table.csv`).

## Stage-by-stage: how each change moved the numbers
| run | LiDAR Hz | RGB Hz | fused Hz | fused lat ms | merged Hz | merged lat ms | pipeline CPU (of 400) | whole system CPU (of 400) | per-core % | SoC C |
|---|---|---|---|---|---|---|---|---|---|---|
| **BEFORE** original build | 10.0 | 3.6 | 2.42 | 77 | 0.66 | 148 | 76% | 198% | 53/48/49/49 | 63.0 |
| + shared-memory transport (128 MB segment) | 10.0 | 15.1 | 9.76 | 49 | 1.49 | 321 | 159% | 324% | 85/79/80/80 | 67.1 |
| + 64 MB segment + threaded merge | 10.0 | 15.0 | 9.75 | 54 | 1.50 | 376 | 167% | 338% | 88/83/83/84 | 68.2 |
| + on-demand streamer + RGB 10 fps (no viewer) | 10.0 | 10.1 | 10.02 | 46 | 1.55 | 276 | 117% | 270% | 71/66/67/66 | 66.9 |
| same, with 1 simulated viewer, 8-min soak | 10.0 | 10.1 | 9.82 | 44 | 1.50 | 266 | 122% | 282% | 74/69/70/70 | 68.4 |
| same, after 8 x kill -9 node-crash test | 10.0 | 10.1 | 10.08 | 44 | 1.55 | 269 | 109% | 270% | 70/67/65/67 | 67.0 |
| **AFTER** final build, 1 viewer | 10.1 | 10.1 | 10.03 | 48 | 1.53 | 305 | 137% | 291% | 76/72/71/72 | 69.2 |

- Fusion output rose 2.42 -> 10.03 Hz and merged 0.66 -> 1.53 Hz because RGB frames now actually arrive.
- The middle rows show the cost: after the transport fix alone the cores sat at ~80-88% (pipeline 167% of 400, 68 C). The next two changes removed
  work that didn't need doing, bringing the pipeline to 117-137% at full rate.
- Merged latency is higher after (305 vs 148 ms) because each merged cloud now contains ~6-7 scans (typically 91-100k points) instead of ~3; its header carries the *last* scan's stamp.
- CPU efficiency: 31.4 %-core per delivered fused Hz before vs 12.4-13.7 after.

## Per-process CPU (percent of one core)
- BEFORE: rgb_node 23% · fusion 15% · livox_driver 14% · merge 8% · streamer 6% · rpicam_vid 6% · livox_restamp 5% · calib_pub 0% · thermal_node 0%
- + transport fix only: merge 40% · fusion 35% · rgb_node 28% · streamer 27% · livox_driver 21% · rpicam_vid 8% · livox_restamp 7% · calib_pub 1% · thermal_node 0%
- AFTER (idle, no viewer): merge 34% · fusion 29% · rgb_node 19% · livox_driver 17% · livox_restamp 7% · streamer 6% · rpicam_vid 4% · calib_pub 1% · thermal_node 0%
- AFTER (viewer, 8-min soak): merge 29% · fusion 26% · streamer 20% · rgb_node 18% · livox_driver 18% · livox_restamp 7% · rpicam_vid 4% · calib_pub 0% · thermal_node 0%

The merge and fusion rows grow because they now process ~4x as many frames; streamer/RGB shrink because of changes 3 and 4. Per-core load AFTER: 76/72/71/72% (whole system, incl. desktop).

## Transport A/B (synthetic traffic at the real sizes/rates, received / sent)
From [`evidence/transport/transport_bench_results.txt`](evidence/transport/transport_bench_results.txt), live pipeline running concurrently:

| profile | RGB 1280x720 (15 Hz sent) | thermal 640x512 (30 Hz) | cloud 24k pts (10 Hz) |
|---|---|---|---|
| A. default Fast DDS | 3.40 | 18.40 | 6.24 |
| B. original UDP-only profile (deployed before) | 3.44 | 17.40 | 5.04 |
| C. shared-memory 64 MB (deployed now) | **14.44** | **28.92** | **9.64** |

## CPU pinning study (rejected)
| run | LiDAR Hz | fused Hz | merged Hz | merged lat ms | pipeline CPU |
|---|---|---|---|---|---|
| unpinned (tuned_viewer2) | 8.3 | 8.30 | 1.27 | 281 | 109% |
| unpinned (none_viewer3) | 10.0 | 9.74 | 1.50 | 219 | 122% |
| pinned (roles_viewer) | 5.7 | 4.87 | 0.14 | 242 | 102% |
| pinned (roles_viewer2) | 10.0 | 9.81 | 1.50 | 223 | 114% |
| pinned (roles_viewer3) | 10.0 | 9.78 | 1.50 | 236 | 120% |

No consistent gain; one pinned run collapsed and one unpinned run also dipped. Not adopted. See [doc 04](04_rgb_10fps_and_cpu_pinning_study.md).

## Realtime / no backlog: 8-minute soak
`soak8min`: LiDAR 10.01, fused 9.82, merged 1.50 Hz. Memory flat (shared memory 746 -> 746 MB; available RAM 3490 -> 3497 MB; merge process +11 MB, others unchanged);
one stale merge window replaced in 8 min. Merged latency steady ~200-250 ms for 34 windows, then **one ~50 s excursion to 530-680 ms (fused 31 -> 124 ms at the same moment)** that
recovered by itself. Fusion rising together with merge means a system-wide stall, not a queue; the cause was not identified (the thermal camera's USB resets occur every ~30 s without this effect).
Per-window series: [`evidence/measurements/soak8min_rates.csv`](evidence/measurements/soak8min_rates.csv).

## Excluded / unusual runs (kept in evidence, not hidden)
- `tuned_viewer`: merged topic read 0 Hz although the node was publishing 1.5 Hz (361 clouds in 4 min) and the viewer received 20,000 points: a measurement-subscriber glitch, repeated as `tuned_viewer2`.
- `tuned_viewer2` LiDAR/fused 8.3 Hz and `roles_viewer` LiDAR 5.7 Hz: transient dips upstream of every change here (the LiDAR chain); cause unidentified. USB resets did not line up with them.

---

# Corner-case report (summary)
Full detail, raw outputs and the list of what is **not** covered: [07_corner_case_report.md](07_corner_case_report.md). Outputs are in [`evidence/corner_cases/`](evidence/corner_cases/).

| area | tests | result | bug found -> fix |
|---|---|---|---|
| Node crash (kill -9 each of 8 nodes) | 8 | **8/8 respawn in 2-4 s**, 1 copy each, data flows | BEFORE: nodes never came back -> `respawn=True` in launch |
| Camera orphan after node crash | 1 | stale `rpicam-vid` found holding the camera | -> child dies with parent (PDEATHSIG) + stale cleanup |
| Whole-service SIGKILL x2 | 2 | auto-restart, merged data back in 25-26 s, 8/8 nodes | - |
| Restart / stop-start / repeat | 4 | data in 15-20 s each, 8/8 nodes, 1 camera process | - |
| Shared-memory leak after crashes | 1 | **821 MB stale RAM** after crash/restart tests | introduced by the SHM change -> auto-cleanup on every start; flat 542 MB after 2 more crashes |
| Merge node (12 phases + shutdown) | 13 | **13/13 pass**: empty/NaN/single-point/identical clouds, stamps back/forward, 4x overload (max 1.8 s, recovers to 0.38 s), silence/resume | NaN window was dropped whole (0 outputs) -> NaN filter at intake (8 outputs) |
| Fusion node (12 phases + shutdown) | 14 | **13/14 pass**: thermal flapping/gone/unsyncable, RGB missing, bad LiDAR, clock rewind/forward | unsyncable thermal left viewer blank + 3 s stall -> trigger on "fusion not succeeding", timeout 3 s -> 1 s |
| Fusion fallback timeout selection | 3 | 3.0 s: 180/270; **1.0 s: 241/270**; 0.5 s: 255/270 | chose 1.0 s (see doc 05) |
| Web streamer (15 checks) | 15 | **15/15 pass**: empty/zero-extent/NaN clouds, 16 concurrent clients, 60 mid-response disconnects, garbage requests, idle-then-fresh | unknown path **hung** (no Content-Length) -> fixed; 45 disconnect tracebacks -> silenced |
| Shutdown on SIGINT/SIGTERM | 8 | all 4 nodes exit in 0.4-0.5 s on both signals | one test process once ignored SIGTERM, not reproduced in 2 attempts |
| Thermal node, device absent | 1 | alive, 0% CPU, no spam | silent wait -> warns every 30 s |

**Known limits / failures kept honest:** fusion with *sub-second* thermal flapping (1 s on / 1 s off) still halves output to ~5 Hz (never blank or stalled); the thermal camera hardware fault is unresolved;
no real reboot, LiDAR unplug, real-WiFi or real-thermal-frame test was possible this session. See doc 07.
