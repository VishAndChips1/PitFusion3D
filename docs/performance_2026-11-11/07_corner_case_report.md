# 07. Corner-case report

All suites run on an **isolated ROS domain/port** (live system untouched) except the crash/restart tests, which ran on the live service and restored it. Scripts and raw output: [`evidence/corner_cases/`](evidence/corner_cases/).

## Bugs the testing found (and what happened to each)
| # | bug | found by | status |
|---|---|---|---|
| 1 | killed nodes never respawn | kill -9 each node | **fixed** (respawn) |
| 2 | orphaned `rpicam-vid` holds the camera after a node crash | stale process observed | **fixed** (PDEATHSIG + cleanup) |
| 3 | **stale shared memory leaks 8 x 64 MB per crash** (caused by my own SHM change) | memory check after the restart tests | **fixed** (auto cleanup at every start) |
| 4 | unknown HTTP path hangs under keep-alive (every browser's `/favicon.ico`) | streamer suite S3 | **fixed** (Content-Length: 0); pre-existing |
| 5 | merge drops a whole window if one point is NaN | merge suite P2 (0 outputs) | **fixed** (filter at intake -> 8 outputs) |
| 6 | fusion stayed blank if thermal frames arrive but aren't time-matched; 3 s stall on dropout | design review + F2 | **fixed** (trigger on fusion success, 3 s -> 1 s) |
| 7 | disconnecting viewers flood the journal with tracebacks (45 in one run) | streamer log | **fixed** (ignored) |
| 8 | thermal node waits silently when its device is absent | missing-device test | **fixed** (warn every 30 s) |
| - | thermal camera USB resets (hardware) | kernel log | **open**, see doc 05 |
| - | sub-second thermal flapping halves output | F2a | **open / accepted**, never blank |

## Suites
**Merge node** ([after](evidence/corner_cases/merge_suite_AFTER.txt), [before NaN fix](evidence/corner_cases/merge_suite_BEFORE_nan_fix.txt)): 13/13. P1 normal 24k pts (15 outputs, 0.35 s max latency); P2 10% NaN (**0 -> 8 outputs after fix**); P3 empty / single-point / 24k identical points; P4 recovery (12 outputs, worker alive); P5 stamps -100 s; P6 stamps +200 s; P7 back to real time; **P8 4x overload 40 Hz: max latency 1.82 s, RSS 231 MB**; P9 right after overload: 0.38 s; P10 silence 6 s then resume; SIGINT exit 0.5 s; 0 tracebacks.

**Fusion node** ([after](evidence/corner_cases/fusion_suite_AFTER.txt), [before](evidence/corner_cases/fusion_suite_BEFORE_timeout_change.txt)): 13/14 pass. F1 all sensors (80 outputs/8 s); F2a flapping 1 s/1 s (**FAIL**: 103 of ~200, accepted); F2b realistic dropouts (242/270); F3 thermal gone (80); **F4 thermal unsyncable (100)**; F5 thermal back (80); F6 RGB missing then returns (0 then 60); F7 NaN/empty LiDAR then normal (60); F8 clock rewind -100 s / forward +200 s / back (79/80/79); SIGINT 0.4 s; 0 duplicate stamps, 0 tracebacks, 19 degrade/recover switches logged.

**Web streamer** ([after](evidence/corner_cases/streamer_suite_AFTER.txt), [before 404 fix](evidence/corner_cases/streamer_suite_BEFORE_404_fix.txt)): 15/15. Details in [doc 03](03_streamer_on_demand_encoding_and_404_fix.md). The first run crashed at S3 on the 404 hang; the log also held 45 `ConnectionResetError` tracebacks before they were silenced (0 after).

**Crash and restart** ([doc 06](06_crash_recovery_respawn_and_shm_cleanup.md)): 8/8 nodes respawn, 2 whole-service SIGKILLs recover in ~25 s, 4 restart variants 15-20 s, shared memory flat.

**Shutdown signals** ([output](evidence/corner_cases/signal_shutdown_before_fix.txt)): thermal, merge, streamer, fusion each exit in 0.4-0.5 s on SIGINT and SIGTERM. One backgrounded thermal test process once survived its `timeout` SIGTERM for 6+ minutes; two reproduction attempts did not recur, so it is recorded as unexplained, not fixed.

**Thermal node, device absent:** alive, 0.0% CPU, no log spam (now warns every 30 s).

**Fusion fallback timeout** ([output](evidence/corner_cases/fusion_timeout_selection.txt)): 3.0 s -> 180/270, 1.0 s -> 241/270, 0.5 s -> 255/270; 1.0 s selected.

## Final live verification (after the last code change)
10.06 / 10.06 / 10.03 / 1.53 Hz (LiDAR / RGB / fused / merged), merged latency 310 -> 305 ms first-to-last third, 1 camera process, 542 MB shared memory, no throttling ([`final_viewer.json`](evidence/measurements/final_viewer.json)).

## NOT covered (stated plainly)
- **A real reboot / cold boot**, power loss, brown-out (no root to reboot; the 5 A supply itself was validated in an earlier campaign).
- **LiDAR unplugged and replugged at runtime**; LiDAR behaviour with the cable out (only synthetic silence was tested).
- **Real thermal frames** resuming, and the thermal camera's own USB fault.
- **Real WiFi radio load / real viewer devices / the hotspot under client load** (viewer was a loopback poller). The 20,000 / 5,000 point caps were set from earlier real-WiFi power testing and were **not re-validated** against the new higher fused rate.
- A real **system clock step on the live system** (no root to set the clock); clock jumps were tested synthetically on the fusion and merge nodes only.
- Long soaks beyond 8 minutes; thermal throttle behaviour at the 2.4 GHz setting (the `arm_freq=2000` cap is still in place).
- Interaction between the new fallback and the viewer's thermal colour mode when thermal is NaN was not checked in a real browser.
