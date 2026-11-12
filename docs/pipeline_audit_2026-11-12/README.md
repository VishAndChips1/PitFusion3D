# Full pipeline audit and corner-case testing, 2026-11-12

Written for: the project owner / thesis reader. All tests ran on the real Pi with the real RGB and thermal cameras.
**The LiDAR was not connected**, so LiDAR data was synthetic (see "What was not tested").

## Summary
| Area | Result |
|---|---|
| Merge node corner cases (12 phases: NaN, empty, degenerate, clock jumps both ways, 4x overload, silence) | 12/12 pass |
| Fusion corner cases (13 phases: sensors missing/flapping/unsyncable, bad LiDAR, clock jumps) | 12/13 pass; the 1 s on/1 s off thermal flap is a known, accepted edge case (see below) |
| Streamer corner cases (15 checks: empty, 404, garbage, 16 clients, 60 aborted clients, degenerate clouds) | 15/15 pass |
| Hard-kill (SIGKILL) of 7 nodes with live data flowing | 7/7 respawn in 2 s, 1 copy each, viewer keeps serving 5000 live points |
| Whole-service restart / stop-start / repeat (live system) | 3/3 pass; data back in 13-15 s; 8/8 nodes; `/dev/shm` files stable (98) |
| 5 further graceful restarts in a row | camera reopened every time, 0 kernel USB errors |
| End-to-end with real cameras + synthetic LiDAR | whole chain works, viewer serves merged (20000) and live (5000) points |

## Problems found and fixed
1. **Forward host-clock step stalled fusion for ~20 s** (`livox_restamp_node.py`). The offset filter takes the minimum over a 20 s window, so after an NTP forward step (the Pi has no RTC, so this happens after every boot) the old minimum kept winning and LiDAR stamps stayed behind the camera stamps. Fix: reset the filter when the host clock and LiDAR counter disagree by more than 0.1 s (below the fusion's 0.15 s slop). Test `t_restamp.py`, old vs new: forward +300 s: wrong for 19.9 s -> 0 s; +3 s: 19.9 s -> 0; +0.2 s: 19.9 s -> 0; backward, stall, restart and latency-spike cases unchanged (still correct). Warnings are rate-limited to one per 5 s.
2. **Merge buffer could grow without limit if scan stamps stalled** (`registration_merge_node.py`). Fix: also close a window after 60 scans. Test `t_merge_stall.py` (250 scans, frozen stamp): old = 0 outputs, memory +145 MB and growing; new = outputs continue, memory bounded.
3. **Thermal node closed the camera while its capture thread was still reading** (`thermal_camera_node.py`). Fix: stop and join the thread (3 s) before releasing. SIGINT and SIGTERM (what systemd sends) both exit with code 0.
4. `thermal_release.sh` (helper) used SIGKILL; changed to SIGINT (see camera finding).

## Thermal camera findings (hardware)
- Works when healthy: 23-30 Hz, 640x512, every frame unique, correct thermal contrast; 30-minute soak with 0 camera kernel events; 5/5 graceful restarts recover it.
- **After a hard kill (SIGKILL) of the thermal node at 14:26:49 the camera enumerated but delivered no frames, with no USB disconnect, until a full power-off.** Port over-current count was 0; Pi supply 4.92 V or higher throughout; no root, so software cannot reset it. Treat the camera as sensitive to unclean termination. The crash test therefore excludes the thermal node.
- Every enumeration shows a failed first descriptor read (-71) followed by success: marginal link, cable not excluded.

## Observations, not changed
- **3-way fusion costs fused-cloud rate.** Same synthetic scans, thermal paused (LiDAR+RGB fallback) vs live: fused 8.8 Hz vs 7.0 Hz, merged 1.50 Hz vs 1.31 Hz. Fusion is not CPU-bound. The RGB node stamps frames in clustered pairs (median spacing 98 ms, 50% of gaps above 150 ms; arrival lag up to 0.3 s), which is the probable cause. A/B tests of queue_size and sync_slop were inconclusive (the extra instances skewed the result), so no setting was changed. A proper fix would use rpicam-vid frame timestamps in `rgb_camera_node.py`.
- **Known edge case F2a:** thermal flapping 1 s on/1 s off gives 50% of fused clouds. The 1.0 s fallback timeout is deliberate: normal thermal frame gaps reach 0.47 s.
- Fast DDS shared memory uses about 620 MB RAM (9 x 64 MB segments); harmless with 3.4 GB free.

## What was not tested
- Real LiDAR data (not connected). Fused-cloud counts here come from 24k-point synthetic scans.
- A real NTP step of the system clock (no root). The node-level clock-jump suites and the estimator unit test cover it; an end-to-end check needs a real boot.
- Radio/WiFi-client effects (see the point-cap sweep).

Evidence: `evidence/tests` (scripts), `evidence/code_before` (pre-fix files), `evidence/results`.
