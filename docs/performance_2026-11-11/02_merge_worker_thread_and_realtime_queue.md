# 02. Merge node: ICP in a worker thread, real-time queueing

## Problem
`registration_merge_node` ran its ICP merge *inside* the subscription callback. On the Pi one 0.5 s window costs ~358 ms of ICP for 5 scans
([`benchmark/results_pi.json`](../../src/landfill_camera_sim/benchmark/results_pi.json)), so for most of every window the node was not receiving scans and dropped them. Windows stretched in wall time and merged output was capped at ~1.5 Hz even when everything else was fast.

## Change (`registration_merge_node.py`)
- The callback only accumulates scans. A closed window is handed to a **worker thread** (numpy/scipy release the GIL in the heavy parts, so it runs on another core).
- **No backlog by design:** the hand-over queue holds one window. If the worker is busy and a window is already waiting, that waiting window is *replaced by the newer one*, and a window older than 1.0 s is discarded.
- Defence in depth: non-finite points are filtered at intake (one NaN used to discard a whole 5-scan window).

## Before / after
| | merged Hz | scans per merged cloud | merged lat ms |
|---|---|---|---|
| before (original) | 0.66 | ~3 (48,961 pts) | 148 |
| transport fix only (before worker) | 1.49 | ~7 (93,224 pts) | 321 |
| after worker (final, with viewer) | 1.53 | ~7 (99,818 pts) | 305 |

**Honest reading:** the worker thread did **not** raise merged throughput. The merged rate was already 1.49 Hz with the transport fix alone (before this change existed, [`shm.json`](evidence/measurements/shm.json)) and is 1.50 Hz after it ([`shm64_worker.json`](evidence/measurements/shm64_worker.json)); cloud size was ~93k points both times. The rise from 0.66 Hz is doc 01 (RGB frames now arrive, so windows fill).
Merged output is limited to ~1.5 Hz by the ICP cost of each window. What this change buys is **robustness**: scan intake can no longer be blocked by ICP, stale work is replaced instead of queued, and latency is bounded under overload (below). The *old* node was not re-run under the 4x overload test, so there is no measured before/after for that; its blocking behaviour is from code reading and the node's own comments.

## Corner cases ([doc 07](07_corner_case_report.md))
12 phases + shutdown all pass: empty / NaN / single-point / identical-point clouds, timestamps jumping back 100 s and forward 200 s, **4x overload** (peak latency 1.8 s, back to 0.38 s right after, memory capped at ~230 MB),
silence then resume, SIGINT exit in 0.5 s. Before the NaN filter a NaN-contaminated window produced 0 outputs; after, 8 ([before](evidence/corner_cases/merge_suite_BEFORE_nan_fix.txt) / [after](evidence/corner_cases/merge_suite_AFTER.txt)).

## Caveat
Under sustained overload the node deliberately drops windows (counted, logged) rather than queue: freshness over completeness. In the 8-min soak exactly one window was replaced.
