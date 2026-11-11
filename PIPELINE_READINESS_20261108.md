# Pipeline readiness check -- 2026-11-08

Full pass over the real-hardware pipeline and its supporting tooling,
done on request ahead of the 5A RPi board arriving, so the full test
battery can just be run when it does. Pipeline was kept on mains power
(RPi) + Anker (LiDAR) throughout, per Vish's note. Nothing destructive
was done; two fixes need Vish's own action (listed in #1 and #2 below)
since they need root / touch persistent system config, which this
session can't do non-interactively.

## 1. REAL, STILL-ACTIVE BUG: the DDS large-buffer fix was never actually completed

`rgb_camera_node.py`'s own comment claims this was fixed "at the launch
level (FASTRTPS_DEFAULT_PROFILES_FILE...) plus a kernel sysctl change --
nothing to fix here." Checked both halves directly:

- FASTRTPS_DEFAULT_PROFILES_FILE -> config/fastdds_large_buffers.xml: in
  place, correctly wired into pitfusion3d_full.launch.py. This half is done.
- The kernel half is NOT done: `sysctl net.core.rmem_max net.core.wmem_max`
  measured **212992** (the exact "~208KB, far too small" value the config
  file's own header comment describes as the problem) -- the ceiling the
  DDS profile's 4MB buffer request gets silently capped back down to.

Confirmed this is the actual live cause, not just a stale comment: measured
rgb/image_raw at only **~3Hz** against its configured 15Hz, with
`bench_live.py` (new tool this run, see #4) both with and without the DDS
profile env var active -- near-identical result either way (profile alone,
without the kernel change, does nothing), which is exactly what the
config file's own comment predicts.

**Fix (needs your password, couldn't run this non-interactively):**
```
sudo sysctl -w net.core.rmem_max=8388608 net.core.wmem_max=8388608
echo -e 'net.core.rmem_max=8388608\nnet.core.wmem_max=8388608' | sudo tee /etc/sysctl.d/99-pitfusion3d-dds.conf
sudo sysctl --system
```
Then re-run `python3 src/landfill_camera_sim/benchmark/bench_live.py --duration 30`
against the live pipeline and check rgb/image_raw's Hz climbed toward 15.

## 2. Gap: pipeline auto-start service exists but was never enabled

`pitfusion3d.service` (systemd --user unit) + `loginctl enable-linger` are
both correctly set up to auto-start the whole pipeline headless on boot --
but `systemctl --user is-enabled pitfusion3d.service` came back
**disabled**. Right now the pipeline only runs because it's launched
manually every session (which is what this session has been doing).
Linger is already `yes`, so enabling it is the only missing step:

```
systemctl --user enable pitfusion3d.service
```
(Deliberately not started now -- the pipeline's already running manually;
starting the service on top of that would double-launch every node.)

## 3. Real OOM event during this session -- pipeline went down silently, no app-level error

Confirmed via kernel log (not guessed): at **15:28:36**, the kernel OOM-killer
fired and killed PID 1117646 (a python3 process, 4.86GB resident) under
genuine memory pressure (`Normal free:0kB`, swap nearly exhausted at the
time). The entire 8-node pipeline went dark at that exact moment with
**zero error, traceback, or shutdown message in its own log** -- the log
just stops mid-stream. The only trace of what happened is in `dmesg`/
`journalctl -k`, not anywhere this project already logs to.

This machine has 7.7GB RAM + only 1GB swap. I can't be certain from the
kernel log alone whether the killed process was this session's own
benchmark run (`bench_offline.py`, run concurrently with the live
pipeline while investigating #4/#5 below) or a genuine leak in one of the
long-running pipeline nodes -- the process was gone by the time I found
this, so I didn't fabricate an attribution either way. What IS certain:
**running a heavy CPU/memory benchmark concurrently with the live
pipeline on this board is enough to OOM the whole system**, and the
pipeline has no self-detection for this (no memory column in
power_logger.py, no watchdog restart).

**Already done:** relaunched the pipeline (confirmed all 8 nodes healthy,
web streamer responding HTTP 200 again).

**Recommended before the big test campaign:**
- Don't run `bench_offline.py` (or anything else CPU/memory-heavy)
  concurrently with the live pipeline -- run it standalone.
- Consider adding a `mem_used_mb`/`mem_available_mb` column to
  `power_logger.py` (currently has loadavg but no memory figures at all)
  so a future OOM leaves a trace in the project's own log, not just
  `dmesg`.
- If a long (multi-hour) unattended soak test is planned once the 5A
  board arrives, watch `free -h` periodically, or add a `systemd`
  watchdog so the pipeline restarts itself rather than staying silently
  dead for hours (which is what happened here -- it had been down for
  ~4 hours before this was even noticed, via an unrelated test script
  failing to connect to localhost:8080).

## 4. Re-ran the existing benchmark suite on the REAL Pi (it had only ever been run on a dev workstation)

`src/landfill_camera_sim/benchmark/{bench_offline,bench_live,make_report}.py`
already existed and are genuinely well-built -- but `results.json`'s
`meta` showed `Linux-...-WSL2-x86_64`, `20` cores: a dev laptop, not the
Pi. Also found `bench_offline.py`'s merge-stage parameters
(`icp_voxel=0.15`, `max_iter=20`, no source/target point caps) didn't
match `registration_merge_node.py`'s actual current deployed defaults
(`icp_voxel=0.2`, `max_iter=10`, caps of 1500/10000) -- those were tuned
down for real-time performance after the benchmark was last run, per that
file's own "# was X" comments, but the benchmark was never updated to
match. Fixed both (now `results_pi.json`, `report_pi/`), matching the
real deployed config (5 scans/window at the real 0.5s window_duration /
10Hz Livox rate, not the old 11-scan/1.0s figures).

**Real numbers, on the actual Pi, for the thesis report:**

| stage | baseline | optimized | speedup |
|---|---|---|---|
| PointCloud2 decode | 0.22ms | 0.01ms | 25.1x |
| PointCloud2 encode | 0.68ms | 0.70ms | 0.97x |
| Voxel downsample (120k pts) | 304.5ms | 62.6ms | 4.9x |
| **RGB+thermal fusion (colourisation)** | **26.5ms** | **8.6ms** | **3.1x** |
| Windowed ICP merge (5 scans) | 2375.2ms | 358.2ms | 6.6x |
| **Total per 0.5s window** | **2707ms** | **405ms** | **6.3x** |

Direct answer to the original "colourisation: if it was 400ms" question:
**it's not the bottleneck at all -- 8.6ms, about 2% of the per-window
budget.** ICP merge dominates (88% of processing time), and the optimized
pipeline uses 405ms of its 500ms real-time budget (81%, ~95ms margin) --
comfortably real-time now, but not by a huge margin; the unoptimized
baseline would have been >5x over budget. See `report_pi/window_budget.png`
for the figure.

Also ran `bench_live.py` (works unmodified against real hardware -- it
subscribes to the same topic names the real launch files use) against the
live pipeline for real Hz/point-count numbers -- see `results_live_pi.json`.
This is also what surfaced finding #1 above.

## 5. Full review of every node actually in the real-hardware pipeline

Read in full: `livox_restamp_node.py`, `rgb_camera_node.py`,
`thermal_camera_node.py`, `fusion_web_streamer.py`,
`registration_merge_node.py`, `cloud_io.py` (plus `sensor_fusion_node.py`
and `calibration_publisher.py`, reviewed in an earlier session). No new
correctness bugs found -- this code is already unusually well-engineered,
with real documented root-causes behind most of its non-obvious choices
(QoS policy, threading, retry/backoff, the real-time ICP cost cuts, the
HTTP keep-alive/quantization changes for WiFi power). Confirmed
`livox_pattern_filter.py`, `calib_solve.py`, `calib_capture_pose.py`,
`platform_motion_controller.py`, `scene_traffic_controller.py` are
simulator-only (not wired into any real-hardware launch file) -- not
reviewed in depth since they're not part of what the 5A board will
actually run.

## 6. Found: an entire pre-existing power/thermal testing toolkit (`tools/power_monitoring/`)

`power_logger.py` (continuous PMIC rail logging -- voltage, current,
throttle flags, temp, loadavg), `power_summary.py`, `burst_monitor.py`
(short high-rate sampling for startup transients), `post_mortem.py`,
`usb_watch.py`, `kernel_log_tail.py`. `power_logger.py` has been running
continuously since 2026-11-07 16:20 (confirmed still running) --
`/home/vish/power_logs/power_log.csv` now covers Oct 23 through today,
1.97M rows. Also found an extensive history of prior real-hardware tests
there already (battery cutout tests, cold-start, real-WiFi sweeps,
point-count tuning) from Oct 23 - Nov 2 -- this project already has much
more test history than this session initially assumed.

One operational note: `power_summary.py` took ~49s to summarize the full
1.97M-row log (not broken, just slow at this size now) -- fine for a
one-off check, but worth keeping in mind if it's run often during the big
campaign; consider a `--since <timestamp>` flag if that becomes annoying.

## 7. New tools added this session (fill the two gaps that genuinely didn't exist)

- **`tools/power_monitoring/cpu_logger.py`** -- per-core CPU%, per-core
  clock, SoC temp, loadavg, continuous CSV (same crash-safe fsync'd
  pattern as `power_logger.py`, so the two logs join on timestamp).
  `power_logger.py` only had an aggregate loadavg and cpu0's own clock --
  nothing broken down by core. Tested: ran for several seconds against
  the live pipeline, confirmed real per-core data (including DVFS
  throttling from 2.0GHz down to 1.5GHz under the earlier benchmark load).
  Directly fills the "CPU usage: core and percentage" report ask.

- **`tools/network_monitoring/wifi_bandwidth_latency_test.py`** -- ping
  latency sweep + iperf3 bidirectional throughput + an HTTP fetch-timing
  proxy against `fusion_web_streamer`'s own `/merged.bin`. Confirmed
  `iperf3` is **not installed** on this Pi (`sudo apt install iperf3`
  needed before the throughput half can run) -- the script detects this
  and tells you so rather than failing unhelpfully. Dry-ran the
  ping+HTTP-proxy half (works: 4.21ms avg round-trip to the router).
  **Needs a second device on the same WiFi, with an operator present to
  run `iperf3 -s` on it** -- this is the one report section that
  genuinely cannot be fully automated or finished solo, same conclusion
  as the earlier session's test tracker already flagged.

## What's still blocking full readiness

1. Run the `sudo sysctl` fix in #1, then re-verify rgb/thermal/pointcloud
   Hz actually recovers.
2. Run `systemctl --user enable pitfusion3d.service` from #2.
3. Install `iperf3` (`sudo apt install iperf3`) and, once an operator +
   second device are available, run `wifi_bandwidth_latency_test.py` for
   real.
4. Everything else (power/temp/CPU logging infra, the benchmark suite,
   the 30-minute power-vs-output-quality and temperature-vs-output-quality
   report sections) is ready to just run once the 5A board is in and the
   full campaign starts -- see `thesis_tests/MASTER_TEST_TRACKER.md` for
   the full, now-updated test list.
