# PitFusion3D — Code Reference

*Everything about how the software is built: structure, every node, algorithms, threading, configuration, deployment, tooling and test harness. Written 2026-11-12 from the live source in `~/ros2_ws/src` (mirrored in the git repository `PitFusion3D_repo`). Numbers quoted here are code facts; measured behaviour is in `FULL_RESULTS_REPORT.md`.*

---

## 1. What the software is

PitFusion3D is a ROS 2 (Jazzy) sensor-fusion pipeline for a three-sensor survey rig aimed at landfill inspection. It runs on a **Raspberry Pi 5 (8 GB)** and combines:

| Sensor | Device | What it provides |
|---|---|---|
| **LiDAR** | Livox Avia (Ethernet/UDP) | 24 000-point scans, nominally 10 Hz, non-repetitive Risley-prism pattern, FOV 70.4° × 77.2° |
| **RGB camera** | Raspberry Pi HQ camera (IMX477, CSI) via `rpicam-vid` | 1280×720 colour frames, 10 fps |
| **Thermal camera** | USB UVC module (VID 3474, PID 43e2) | 640×512 8-bit auto-gain (AGC) image, 30 fps requested — a *relative* heat signal, not calibrated temperature |

Every LiDAR point is **projected into both camera images** and gets an RGB colour and a thermal value ("RGBT cloud"). A rolling window of RGBT scans is then **registered with ICP and merged** into a denser cloud. A small **web server on the Pi** serves both clouds to any browser (laptop/phone) as compact quantised binary, so nothing needs to be installed on the viewing device and nothing is pushed anywhere: the Pi is an HTTP server on port 8080 and a viewer *pulls* data.

The repository started as a **Gazebo simulation** of the rig (landfill scene, sensor truck, Livox scan-pattern emulation, calibration tooling) so that the pipeline could be developed without hardware; the same processing nodes run unmodified on the real sensors through the `pitfusion3d_bringup` package.

## 2. Repository / workspace layout

```
~/ros2_ws/src/
  landfill_camera_sim/            original sim project + the processing nodes that run on the Pi
    scripts/                      fusion, merge, web streamer, calibration publisher, cloud_io, sim helpers
    launch/                       sensor_fusion.launch.py, web_stream.launch.py, sim scenario launches
    config/                       sensor profiles, calibration/{extrinsics,intrinsics}.yaml, Avia scan table
    worlds/                       Gazebo SDF landfill scene and truck
    benchmark/                    offline + live benchmarks, baseline implementations, report generators
  pitfusion3d_bringup/            real-hardware package
    pitfusion3d_bringup/          livox_restamp_node, rgb_camera_node, thermal_camera_node, camera_viewer
    launch/                       real_sensors.launch.py, pitfusion3d_full.launch.py
    config/                       fastdds_large_buffers.xml
    scripts/                      run_pipeline.sh, clean_stale_shm.py, thermal_view.py, hotspot start/stop scripts
  livox_ros2_avia/                vendor C++ driver (≈29 k lines, 72 files), launched via real_sensors.launch.py
~/.config/systemd/user/pitfusion3d.service   auto-start unit
~/power_logs/                     power_logger.py, usb_watch.py, kernel_log_tail.py (continuous system logging)
```

Static-analysis totals (see §9 and `data/code_metrics.json`): 20 Python files, ≈4 000 lines (≈3 000 code, ≈550 comment), 113 functions, 19 classes, 86 declared ROS parameters; plus ≈29 000 lines of vendored C++ (the Livox driver).

## 3. Runtime architecture

### 3.1 Dataflow

```
 Livox Avia ──UDP──► livox_ros2_avia_node ──/livox/lidar (device clock)──► livox_restamp_node
                                                                              │ /landfill/livox/points (host clock)
 CSI camera ► rpicam-vid ─pipe─► rgb_camera_node ──/landfill/rgb/image_raw──┐  │
 USB thermal ─V4L2─► thermal_camera_node ──/landfill/thermal/image_raw──────┤  ▼
                                                                    sensor_fusion_node  (time-sync, project, colour)
 calibration_publisher ─ tf static + CameraInfo (30 Hz)                          │ /landfill/fusion/points_rgbt  (≈10 Hz)
                                                                                 ├──────────────► fusion_web_streamer  /live.bin
                                                                                 ▼
                                                                    registration_merge_node  (window → ICP → voxel)
                                                                                 │ /landfill/fusion/points_merged (≈1.5 Hz)
                                                                                 ▼
                                                                    fusion_web_streamer  /merged.bin  +  viewer page  :8080
```

### 3.2 Processes and threads

The system is **9 OS processes plus the `rpicam-vid` child**, all started by one `ros2 launch`. Every node is its own process, so the Python GIL never couples two nodes. Inside a process the model is: one rclpy executor thread (callbacks) plus the threads below.

| Process | Threads you wrote / start | Role of each |
|---|---|---|
| `livox_ros2_avia_node` | vendor threads (≈17 observed) | UDP receive, frame assembly, publish |
| `livox_restamp_node` | executor only | one callback per scan |
| `rgb_camera_node` | **capture thread** + executor; child process `rpicam-vid` | capture thread blocks reading exactly one frame from the pipe, converts, publishes |
| `thermal_camera_node` | **capture thread** + executor | V4L2 read loop with self-healing reopen/cool-down |
| `calibration_publisher.py` | executor (30 Hz timer) | republish CameraInfo + static TF |
| `sensor_fusion_node.py` | executor (two `ApproximateTimeSynchronizer`s, callbacks run on it) | fuse |
| `registration_merge_node.py` | **ICP worker thread** + executor | executor only buffers scans; the worker does the heavy ICP so the callback never blocks |
| `fusion_web_streamer.py` | **HTTP server thread** + one thread per client (ThreadingTCPServer) + executor | encode on demand in request threads |
| `rpicam-vid` | vendor (≈8 threads) | camera ISP/encoder |

Thread-count limits: the launch files export `OMP_NUM_THREADS=OPENBLAS_NUM_THREADS=MKL_NUM_THREADS=NUMEXPR_NUM_THREADS=OPENCV_NUM_THREADS=1`, and each OpenCV-using node calls `cv2.setNumThreads(1)`, so numpy/OpenCV never spawn extra worker pools on the 4-core board. No CPU pinning is used (a pinning study showed no consistent gain; see the performance docs).

### 3.3 Transport and QoS

* All sensor and cloud topics use `qos_profile_sensor_data` (**BEST_EFFORT**, VOLATILE, shallow KEEP_LAST): a dropped frame is never retransmitted, which keeps latency bounded and removes retransmission bookkeeping. (The Livox driver publishes `/livox/lidar` RELIABLE; a BEST_EFFORT subscriber is compatible with that.)
* **RMW:** `rmw_fastrtps_cpp` (Fast DDS 2.14). Discovery is pinned to the machine: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (so the pipeline does not depend on any WiFi subnet, and keeps working when the Pi switches between client and hotspot modes).
* **Shared-memory transport:** `config/fastdds_large_buffers.xml` defines a 64 MB SHM segment (listed first) plus a UDPv4 transport with 4 MB buffers (kept for discovery and for peers that do not load the profile). Reason: a 1280×720 `rgb8` frame is 2.76 MB and a fused cloud is 0.58 MB; over default UDP loopback the kernel socket buffer ceiling (`net.core.rmem_max = 212 992`, not raisable without root) silently drops fragments. **Any tool that measures a topic must load the same profile** (`FASTRTPS_DEFAULT_PROFILES_FILE`) or it will see artificially low rates.
* `clean_stale_shm.py` removes `/dev/shm/fastrtps_*` segments no live process maps (they leak after unclean exits, ≈0.5 GB per crash with 64 MB segments); it runs at every service start.

## 4. Node-by-node

### 4.1 `livox_ros2_avia_node` (vendor driver, node name `livox_lidar_publisher`)
Launched from `real_sensors.launch.py` with `xfer_format: 0` (PointCloud2), `publish_freq: 10.0`, `data_src: 0` (live), `frame_id: camera_platform/lidar_frame`, `respawn=True, respawn_delay=2.0`. Config JSON (`livox_lidar_config.json`) enables the connected unit by broadcast code, first-return mode, Cartesian coordinates, and **disables hardware time-sync**, which is why the driver stamps clouds with the lidar's own free-running counter (seconds since the sensor booted) — the reason `livox_restamp_node` exists. Output point layout (`/livox/lidar`): `x,y,z` (float32), `intensity` (float32), `tag` (uint8), `line` (uint8) — 18 bytes/point; 24 000 points ⇒ 432 kB per message.

### 4.2 `livox_restamp_node` (`pitfusion3d_bringup/livox_restamp_node.py`)
**Purpose:** move LiDAR timestamps from the sensor's clock into the Pi's clock so the fusion's time-synchroniser can match them against camera stamps.
**Method:** for each scan, `sample_offset = arrival_host_ns − device_ns`. The true offset plus network/scheduling latency (always ≥ 0) is what is observed, so the **minimum over a rolling window of 200 samples (≈20 s)** is the best estimate (the NTP/PTP "min-delay" filter). Corrected stamp = `device_ns + min_offset`. This preserves the LiDAR's hardware-precise packet-to-packet timing (≈100.00 ms ± <1 ms) instead of folding ~8 ms of arrival jitter into every stamp.
**Resets (added 2026-11-12):** (a) device counter going backwards by >1 s (sensor/driver restart); (b) **host-clock step**: the host clock and the device counter normally advance equally between scans; if they disagree by more than `clock_step_threshold_s` (0.1 s, deliberately below the fusion's 0.15 s slop) the window is cleared immediately, because a *forward* NTP step (this Pi has no real-time clock, so NTP steps it after every boot) would otherwise be hidden by the min-filter for a whole window (~20 s) and LiDAR stamps would trail the camera stamps by the full step. Warnings are rate-limited (5 s).
**Interface:** sub `livox/lidar`, pub `/landfill/livox/points`, both sensor-data QoS. Parameters: `input_topic`, `output_topic`, `offset_window=200`, `log_period=50`, `clock_step_threshold_s=0.1`. `_estimate()` is a pure function of its inputs plus filter state, so it can be unit-tested without a clock (`t_restamp.py`).

### 4.3 `rgb_camera_node` (`rgb_camera_node.py`)
Spawns `rpicam-vid -t 0 --width 1280 --height 720 --framerate 10 --codec yuv420 --saturation 1.8 --nopreview --flush -o -` (`--flush` makes it write each frame out immediately; without it frames reached the pipe in bursts and were stamped in clustered pairs — adding it made the stamp interval std fall from 79 ms to 5 ms and tightened LiDAR-to-RGB alignment 3×) and reads **exactly one frame (1280×720×3/2 bytes) at a time** from its stdout pipe in a **dedicated capture thread**. Each frame is converted with `cv2.cvtColor(..., COLOR_YUV2RGB_I420)`, wrapped by `cv_bridge` as `rgb8` and published with the **host-clock stamp taken the instant the complete frame has been read** (not after conversion) on `/landfill/rgb/image_raw` (2 764 800 bytes/frame). Robustness: the child is started with `PR_SET_PDEATHSIG` so it dies with the node; stale `rpicam-vid` processes from a previous crash are killed at start; if the stream ends the loop respawns it with exponential back-off (1→10 s); per-frame exceptions are caught so one bad frame cannot kill the thread. Frame rate is 10 because the LiDAR is 10 Hz and frames above that are never fused (it was 15 earlier). Parameters: `topic`, `frame_id`, `width`, `height`, `framerate`, `saturation`.

### 4.4 `thermal_camera_node` (`thermal_camera_node.py`)
Opens the UVC device **by its `/dev/v4l/by-id/…` path** (stable across the changing `/dev/videoN` numbering caused by the CSI camera's many nodes) with OpenCV/V4L2 (`YUYV`, 640×512, `CONVERT_RGB=0`). Because this module only exposes an 8-bit AGC'd YUYV stream (no radiometric mode), the node takes the **Y (luma) channel**, rotates it 180° (`vflip`+`hflip`, the module is mounted upside down), and publishes it left-shifted as `mono16` (`value = Y·256`) so downstream can recover `raw·thermal_resolution + thermal_min_temp`; the launch sets resolution `1/256` and min `0`, giving back the 0–255 AGC value — **a relative hot/cold signal, not Kelvin**. Self-healing capture thread (structured as `_connect`, `_read_frame`, `_on_read_failure`, `_publish_frame`, with a small `_CaptureHealth` object holding the back-off counters): waits for the device node to exist continuously for `device_settle_s` (3 s), then opens; after 10 consecutive read failures it releases the device and waits a cool-down that doubles from 3 s to `recovery_cooldown_max_s` (20 s) (reopening instantly was observed to keep a misbehaving camera in a reset loop); cool-down resets after 30 good frames; the missing-device warning is throttled to every 30 s. **Shutdown (2026-11-12):** `destroy_node()` stops the capture thread and **joins it (3 s) before releasing the device**; an unclean close of a streaming session was observed to leave this camera unable to stream until power-cycled. Optional `publish_preview` (colourised) is off by default.

### 4.5 `calibration_publisher.py`
Reads `config/calibration/extrinsics.yaml` and `intrinsics.yaml`, publishes the transforms as **tf2 static transforms** and republishes `sensor_msgs/CameraInfo` for RGB and thermal at 30 Hz (`publish_rate`). **Calibration status (important for accuracy claims):** the RGB intrinsics are placeholders (`fx=fy=1065`, `cx=640`, `cy=360`, no distortion; a ≈62° HFOV guess for a 6 mm lens), RGB and LiDAR extrinsics are zero (co-located assumption), and the thermal camera was **solved from a single person-as-target capture** (`calib_solve.py`): `fx=fy=919`, `cx=320`, `cy=256`, translation `(0, −0.003, +0.061)` m relative to RGB. A checkerboard calibration has not been done; colour/thermal alignment therefore has an unquantified systematic error that grows with the true sensor offsets.

### 4.6 `sensor_fusion_node.py`
**Inputs:** `/landfill/livox/points`, `/landfill/rgb/image_raw`, `/landfill/thermal/image_raw`. **Output:** `/landfill/fusion/points_rgbt` (`x,y,z,intensity,rgb,thermal`, 24 bytes/point; `rgb` is a packed 0xRRGGBB held in a float32 slot via bit reinterpretation, `0` = no colour; `thermal` is float32, `NaN` = no sample).
**Synchronisation:** two `message_filters.ApproximateTimeSynchronizer`s with `queue_size=5`, `slop=0.15 s`: a **3-way** one (LiDAR+RGB+thermal) and a **fallback 2-way** one (LiDAR+RGB). The 2-way callback is ignored while 3-way fusion is succeeding; if no 3-way fusion has succeeded for `thermal_timeout_s` (1.0 s, on a **monotonic clock**, so immune to NTP steps) the node logs a warning once and fuses LiDAR+RGB with `thermal = NaN` until thermal returns ("recovered" is logged). The timeout was chosen from a test (3.0 s lost ~33 % of output under realistic dropouts; 1.0 s lost ~11 %; 0.5 s ~6 % but would flip modes on normal thermal frame gaps of up to ≈0.47 s). A duplicate guard (`_last_fused_stamp`) prevents the same scan being fused via both paths.
**Per-scan algorithm (`_process_synced`):** (1) map the cloud's bytes with a numpy dtype carrying the real field offsets and take x,y,z,intensity as one strided view (`cloud_io.read_fields`) — no per-message structured-array rebuild; (2) **drop non-finite points once, at the entry of the pipeline** (a NaN reaching ICP/SVD or the streamer's quantiser used to poison whole clouds); (3) decode the two images to arrays; (4) project: `uv = K·(xyz @ M + b)` with the extrinsic hops **folded into one (M, b) at start-up**; when RGB and thermal share a transform and intrinsics the RGB projection is **reused** for thermal; (5) **gather only at projected pixels** (≈24 k samples) instead of converting the whole 0.9 Mpx frame; (6) convert thermal raw → value, pack RGB, build the (N,6) float32 array and publish with the LiDAR's header. Exceptions in the callback are caught (one bad triple is skipped, the node survives).
**Clock handling:** `_watch_clock` clears the synchroniser queues if LiDAR stamps jump backwards by more than 1 s (sim restart / large NTP step) so stale future-stamped messages are not matched.
**Instrumentation:** earlier versions could write a per-stage timing CSV (decode / project / colour / publish) with `PF3D_TIMING=1`; that code was removed on 2026-11-14 to keep the node clean. The instrumented version used for the stage-timing results is kept in `tools/instrumented/sensor_fusion_node_timing.py` in the repository.
**Parameters (defaults):** see Appendix A. The launch overrides `thermal_min_temp=0.0`, `thermal_resolution=0.00390625`, `window_duration=0.5`.

### 4.7 `registration_merge_node.py`
**Purpose:** turn ~10 sparse scans/second into one dense merged cloud per window. **Buffering:** the subscription callback (split into `_finite_points`, `_reset_window_if_clock_rewound`, `_window_is_complete`, `_submit_window`) appends `(stamp, xyz, [intensity,rgb,thermal])` to a list (after the same NaN filter) and closes the window when `stamp − window_start ≥ window_duration` (0.5 s in the full launch; default 1.0 s) **or the buffer reaches 60 scans** (safety valve against stalled stamps, added 2026-11-12). A backward stamp jump of >1 s resets the window (small NTP backward jitter is tolerated on purpose).
**Threading:** the closed window goes to a `queue.Queue(maxsize=1)` consumed by a **single worker thread**. If the worker is still busy, the waiting (stale) window is **replaced** by the newest one and counted — so the node can never build a backlog, and the callback never blocks. The worker drops windows older than `_max_window_age` (1.0 s) rather than publishing stale data. A window that raises is logged and skipped.
**Algorithm (`_register_and_publish`):** sort scans by stamp; the first scan seeds the map; for each later scan: voxel-downsample (0.2 m) → subsample to ≤1500 source / ≤10 000 target points → **point-to-point ICP** (cKDTree nearest neighbours with `distance_upper_bound` 0.5 m, Kabsch SVD, ≤10 iterations, convergence when mean error changes <1e-5, ≥200 correspondences; serial KD-tree queries below 20 000 points because scipy's per-call thread-spawn cost exceeded the parallel gain) → apply the rigid transform to the full-resolution scan → grow the downsampled map. Non-converged scans are merged with identity and counted in a warning. The union is **voxel-downsampled at 0.01 m** (int64-hashed voxel keys; selection bit-identical to `np.unique(axis=0)` but ~4.8× faster) and published with the **last scan's stamp**. Each publish logs `Published merged cloud: N pts (S scans, R raw pts, W s window) @ X Hz`.
**Output:** `/landfill/fusion/points_merged` (24 bytes/point).

### 4.8 `fusion_web_streamer.py`
A ROS node plus an HTTP server in one process (default port 8080, bind `0.0.0.0`). `ThreadingTCPServer` (one thread per connection, `http_keepalive=True`), connection errors from disappearing clients are swallowed.
**Endpoints:** `/` — single-page 3-D viewer (static assets from `web_assets/`); `/merged.bin` and `/live.bin` — binary clouds from `points_merged` and `points_rgbt`; anything else → `404` with `Content-Length: 0` (a missing length once hung browsers).
**Lazy encoding:** the ROS callbacks only *remember* the newest message (`_StreamState.offer`); quantisation and gzip happen **only when a client asks** for newer data (`refresh()` under an encode lock, so concurrent clients share one encode). With no viewer connected the node costs ≈ nothing (it previously encoded every cloud at 10 Hz regardless: ≈27 % of a core idle).
**Wire format (before gzip):** `uint32 n, uint32 seq`, `float32 ox, oy, oz, scale` (24-byte header), then **11 bytes/point**: `int16 x,y,z` (offset+scale quantised, scale = extent/32000 ⇒ ≤ ≈0.14 mm error at the observed scene sizes), `uint8 r,g,b` (lossless), `int16 thermal×100` (sentinel for NaN). Compared with 28 B/point float32 this is 2.55× smaller before gzip (level 6, compressed once per data update, not per poll). Over the limit, points are randomly subsampled to `max_points_merged` (default 20 000) / `max_points_live` (5 000). **Both caps are runtime-settable** (`ros2 param set /fusion_web_streamer max_points_merged N`, validated to [100, 1 000 000]).
**Client protocol:** `ETag = seq`; a request with the same `If-None-Match` gets `304`; the page polls merged every 250 ms or live every 120 ms (one view at a time).

### 4.9 Helpers and tools
* `cloud_io.py` — zero-copy PointCloud2 ⇄ numpy (`structured_view`, `read_fields` with a strided-block fast path, `make_cloud`); field sets `XYZI_FIELDS`, `XYZIRT_FIELDS`.
* `thermal_view.py` — standalone thermal-only viewer: subscribes to the thermal topic **only while a client is asking** (drops the subscription 10 s after the last request), colourises (INFERNO), serves `http://<pi>:8081/` page, `/stream.mjpg`, `/frame.jpg`; shows **NO SIGNAL** after 3 s without frames; encodes only while a client is connected; does not touch the USB device.
* `camera_viewer.py` — debug: side-by-side RGB+thermal with sync-offset read-out.
* `clean_stale_shm.py`, `start_hotspot_viewing.sh` / `stop_hotspot_viewing.sh` (switch the Pi to/from its own WiFi access point; currently the hotspot does **not** auto-start).
* Simulation-only: `livox_pattern_filter.py` (resamples a Gazebo uniform-grid lidar into the true Avia rosette using bilinear range sampling), `platform_motion_controller.py`, `scene_traffic_controller.py`, `calib_capture_pose.py`, `calib_solve.py`.
* `benchmark/` — `bench_offline.py` (per-kernel baseline vs optimised on synthetic data), `bench_live.py`, report/figure generators; the pre-optimisation implementations are kept verbatim in `benchmark/baseline/`.


### 4.10 Operational tools added around the pipeline
* **`~/lidar` / `~/lidar_ctl/lidar_ctl.cpp`** — controls the Livox Avia's work mode (Normal / Power-Saving / Standby) over Ethernet using the vendor SDK's `LidarSetMode`; the sensor confirms each change (0.9 s into low power, 9.7 s back to Normal). It stops the pipeline gracefully first, because the LiDAR serves one host at a time. Full description, protocol references and test log: `LIDAR_POWER_MODE.md`.
* **`~/thermal_fix/`** — `thermal_watchdog.py` (user service `thermal-watchdog`) detects the thermal camera's stuck state (≥ 4 USB disconnects in 5 minutes, from `~/power_logs/usb_events.log`) and, if it persists 10 minutes, tries port power-cycle → deauthorise/reauthorise → `usbreset`; `install_once.sh` + `thermal-usb-perms.sh` install the one-time udev rule that makes those controls writable by the user. Status in `status.json`, log in `watchdog.log`.
* **`~/check_thermal.sh [minutes]`** — one-command pass/fail check of the thermal stream (rate, gaps, frame uniqueness, camera resets); `~/thermal_release.sh` / `~/thermal_resume.sh` free and return the camera for direct use (graceful stop).
* **Measurement harness** — see section 8; `make_thesis_figures.py` regenerates all thesis figures from the data.

## 5. Deployment

`run_pipeline.sh` (exec'd by the systemd user unit) sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, sources ROS and the workspace, runs `clean_stale_shm.py`, then `exec ros2 launch pitfusion3d_bringup pitfusion3d_full.launch.py`. The launch exports the thread-limit variables and the Fast DDS profile, starts `real_sensors.launch.py` immediately, `sensor_fusion.launch.py` after 3 s and `web_stream.launch.py` after 6 s. **Every node has `respawn=True, respawn_delay=2.0`.** `pitfusion3d.service` is a *user* unit with `Restart=on-failure`, `RestartSec=5`, `StartLimitBurst=10/600 s`, `WantedBy=default.target`, and the user has `linger` enabled so it starts at boot with nobody logged in; it deliberately does **not** wait for `network-online.target` (the pipeline needs no network). Stopping the service sends SIGTERM to every process; rclpy handles it (exit code 0), so nodes close devices cleanly.
Networking: NetworkManager profile `Patelnet` is preferred (priority 100, retries forever); the `PitFusion3D-AP` hotspot profile (10.42.0.1/24) is kept but has autoconnect off. Viewer URL on the home network: `http://vish-desktop.local:8080` (mDNS via avahi) or the DHCP address.

## 6. Resilience features (all in code)

| Failure | Handling |
|---|---|
| Any node crashes / is killed | `respawn=True` (2 s) in every launch entry; `rpicam-vid` dies with its parent and stale ones are killed at start |
| Whole service dies | systemd `Restart=on-failure` |
| Thermal camera absent/reset loop | settle wait, growing cool-down, throttled warnings; fusion falls back to LiDAR+RGB after 1 s |
| RGB stream ends | respawn `rpicam-vid` with back-off |
| NaN/Inf/empty/degenerate clouds | filtered once at fusion input and again at merge input; per-callback exception guards |
| Clock steps / rewinds | restamp window reset (both directions), fusion queue clear on rewind, merge window reset on rewind, monotonic timeouts |
| Slow ICP | queue of 1, replace-stale, max window age, no backlog |
| Many/slow/aborting HTTP clients | per-connection threads, swallowed connection errors, shared encode, 304 caching, lazy encode |
| Leaked shared memory | cleaned at each start |

## 7. Design decisions and rationale (short)

1. **Shared memory instead of UDP loopback** — default transport lost most large messages (RGB 3.4 of 15 Hz received); SHM raised RGB to 14.4 Hz, thermal to 28.9 Hz, clouds to 9.6 Hz.
2. **Merge in a worker thread with a size-1 queue** — real-time guarantee (no backlog) rather than throughput; it did *not* raise merged throughput (1.49 → 1.50 Hz), which is limited by ICP time.
3. **Lazy streamer encode** — removes the idle cost and makes the cost proportional to viewers.
4. **RGB at 10 fps** — matches the LiDAR; frames above 10 Hz are never fused.
5. **No CPU pinning** — a study found no consistent gain and one pinned run collapsed.
6. **Single-thread BLAS/OpenCV** — avoids oversubscription of 4 cores by many processes.
7. **Quantised int16 wire format** — position error far below the sensor precision (≈0.14 mm vs ≈16 mm plane noise).
8. **Min-delay offset filter for LiDAR stamps** — preserves hardware timing; reset on clock steps.

## 8. Measurement and test harness written for this work

| Path | Purpose |
|---|---|
| `~/thesis_tests/perf_20261111/` | pipeline performance harness (`pipeline_perf.py`, `run_case.sh`), before/after JSON/CSV |
| `~/thesis_tests/corner_cases_20261111/` | corner-case suites `t_merge.py` (12 phases), `t_fusion.py` (13), `t_streamer.py` (15), `crash_recovery.py`, `restart_cases.py` |
| `~/thesis_tests/pointcap_sweep_20261111/` | point-cap sweep (`sweep.py`, `grab_full.py`, `analyze.py`), raw 30 Hz power traces |
| `~/thesis_tests/audit_20261112/` | `t_restamp.py`, `t_merge_stall.py`, e2e synthetic-LiDAR tests, A/B fusion test |
| `~/thesis_tests/thermal_debug_20261112/`, `~/check_thermal.sh`, `~/thermal_release.sh`, `~/thermal_resume.sh` | thermal-camera diagnosis and soak |
| `~/thesis_tests/final_campaign_20261112/` | **the full campaign**: `campaign.py` (orchestrator), `lib.py` (30 Hz PMIC sampler with safety gates, 1 Hz system sampler), `probe_topics.py`, `chars.py`, `http_bench.py`, `net_bench.py`, `threads.py`, `stress.py`, `accuracy.py`, `transport_test.py`, `lidar_loss.py`, `inventory.py`, `code_metrics.py`, `build_report.py` |
| `~/PitFusion3D_repo/docs/…` | committed write-ups: performance 2026-11-11, point-cap sweep, pipeline audit 2026-11-12 |

## 9. Quality indicators from static analysis
Computed by `code_metrics.py` (AST): lines/code/comment ratios, per-function length and a branch-count complexity (cyclomatic-style: 1 + branches, loops, handlers, boolean operators). Appendix B lists every file; the most complex functions are in `sensor_fusion_node.py`, `thermal_camera_node.py` (self-healing loop) and `registration_merge_node.py`. Comment density is ≈14 % of lines, concentrated on *why* (root-cause notes next to defensive code).



---

# Deep dive: how the whole system works, end to end

*This chapter explains the software as a single machine: what happens to one LiDAR scan from the photons to the pixel in the browser, which threads run when, which state machines decide what the system does when something goes wrong, and how each algorithm works numerically. It complements the per-node reference above and uses numbers measured in the 2026-11-12 campaign.*

---

## 10. The life of one LiDAR scan (≈ 100 ms of data, ≈ 0.25 s of latency)

| # | Where | What happens | Cost / timing (measured) |
|---|---|---|---|
| 1 | Livox Avia | The Risley-prism scanner accumulates 100 ms of returns (24 000 points, a non-repetitive rosette), timestamps the packet stream with its own free-running clock, and sends ≈ 2 400 UDP datagrams/s to the Pi. | 240 000 points/s |
| 2 | `livox_ros2_avia_node` (C++) | Assembles packets into one `PointCloud2` (x,y,z,intensity,tag,line; 18 B/point; 432 kB) per 100 ms and publishes it on `/livox/lidar`. Does **not** start sampling until the sensor reports Normal state. Stamps use the sensor clock (seconds since sensor boot). | ≈ 11 % of a core |
| 3 | `livox_restamp_node` | `offset = min over last 200 scans of (arrival − device_stamp)`; publishes the same cloud with `stamp = device_stamp + offset` on `/landfill/livox/points`. If the host clock and device counter disagree by > 0.1 s between scans (NTP step) the window is cleared. | ≈ 5 % of a core; stamp age at the next subscriber ≈ 7 ms (median) |
| 4 | `sensor_fusion_node` — synchroniser | Two `ApproximateTimeSynchronizer`s (queue 5, slop 0.15 s) wait for an RGB frame and a thermal frame whose stamps are within 0.15 s of the scan. The 3-way match wins; if none has succeeded for 1.0 s the LiDAR+RGB match is used instead. | wait ≈ 0–100 ms (depends on camera stamps) |
| 5 | `sensor_fusion_node` — fuse | Drops non-finite points; decodes the two images; projects the points with the pinhole model through the folded extrinsic `(M,b)`; **gathers** colour and thermal at the projected pixels only (≈ 24 k samples instead of converting 0.9 Mpx); packs `(x,y,z,intensity,rgb,thermal)` into 24 B/point and publishes on `/landfill/fusion/points_rgbt`. | ≈ 21 % of a core; fused stamp age ≈ 35–65 ms |
| 6 | `registration_merge_node` — callback | Appends the scan to the window list; when the scans span 0.5 s (≈ 5–7 scans) or 60 scans accumulate, the closed window is put on a size-1 queue (replacing any waiting window). Never blocks. | microseconds |
| 7 | `registration_merge_node` — worker thread | For each later scan: voxel-downsample (0.2 m) → subsample (≤ 1 500 source / ≤ 10 000 target points) → point-to-point **ICP** against the growing downsampled map (cKDTree nearest neighbours within 0.5 m, Kabsch SVD, ≤ 10 iterations) → apply the rigid transform to the full-resolution scan. Concatenate, voxel-downsample at 1 cm (int64-hashed keys), publish with the last scan's stamp. | ≈ 31 % of a core; ≈ 0.6 s per window ⇒ 1.5 Hz merged output; merged stamp age ≈ 250 ms |
| 8 | `fusion_web_streamer` — callback | Remembers the newest merged/fused message (`offer`) — no copy, no encode. | microseconds |
| 9 | `fusion_web_streamer` — request thread | When a browser asks for `/merged.bin` and a newer cloud exists: (under the encode lock) randomly subsample to the cap, quantise to int16 + uint8 + int16, gzip level 6 once, publish `(buffer, seq)`; otherwise answer `304` from the ETag. | encode ≈ 42 ms + gzip ≈ 47 ms at 60 k points (one per update, shared by all clients) |
| 10 | Browser | Polls every 250 ms (merged) or 120 ms (live), decompresses, rebuilds positions from `offset + scale·int16`, renders. | n/a |

**Where latency comes from:** sensor accumulation (100 ms), time-sync wait for the cameras (0–100 ms), fusion compute, then for the merged view the 0.5–0.6 s window itself (the merged cloud's stamp is its *last* scan, so its *age* is ≈ 250 ms but its *content* spans 0.6 s).

## 11. Concurrency model, in detail

**Processes (9) and what each blocks on**

| Process | Threads that do work | Blocks on | Shares state with |
|---|---|---|---|
| livox driver | UDP receive, frame assembly, publish (vendor) | socket | nothing (publishes via DDS) |
| restamp | executor | DDS take | none |
| rgb_camera_node | **capture thread** (blocking pipe read of exactly one frame) + executor | `rpicam-vid` stdout | none — the capture thread publishes directly (rclpy publishers are thread-safe) |
| thermal_camera_node | **capture thread** (V4L2 `read`) + executor | USB frame | `_cap`, `_running` (single writer each) |
| fusion | executor (both synchronisers' callbacks) | DDS | none |
| merge | executor (buffer) + **worker thread** (ICP) | `Queue.get(timeout=0.5)` | `queue.Queue(maxsize=1)` only; `_dropped_windows` is a counter written by both threads (benign race, only used for logging) |
| streamer | executor + HTTP server thread + one thread per client | sockets / locks | `_StreamState` (below) |

**The streamer's three locks and why they are enough.** `offer()` is deliberately lock-free: it writes `latest_msg` *then* increments `latest_id`. `refresh()` takes `encode_lock`, re-reads `(latest_id, latest_msg)` and encodes. Because the message is written before the counter, a reader can see a new message with an old counter (it encodes the newer message and one redundant re-encode follows) but never an old message with a new counter, so a payload is never stale or torn. `set_payload()` swaps `(buffer, seq)` together under `lock`, and `get_buffer_and_seq()` reads them together, so a client always receives a matching pair. The encode lock makes N concurrent clients share one encode instead of repeating it.

**Why no thread pools and no CPU pinning.** The Python GIL only matters inside one process, and every node is its own process, so the OS already spreads the work over four cores. BLAS/OpenCV thread pools are forced to 1 thread (`OMP_NUM_THREADS=1`, `cv2.setNumThreads(1)`) so ten processes do not oversubscribe four cores. A pinning study (roles to cores) showed no consistent gain, and one pinned run collapsed, so scheduling is left to the kernel. Measured balance: per-core load 43–46 % with the pipeline idle.

**Real-time policy, summarised.** Every hand-off drops rather than queues: DDS best-effort (a lost frame is gone), the merge queue (`maxsize=1`, replace-stale, max window age 1 s), lazy streamer encode (only the newest cloud is ever encoded), subscription queue depth 5. The system therefore never accumulates latency; under overload it loses *freshness of rate*, not *age of data* (tested at 4× overload: max latency 1.2–1.4 s, recovering to 0.2 s once the load stops).

## 12. State machines

**Thermal capture loop** (`ThermalCameraNode._capture_loop`)
```
        ┌──────────────────────────── shutdown (running=False / rclpy down) ───────────► EXIT
        ▼
  WAIT_DEVICE ──device node present continuously for 3 s──► OPEN ──open fails──► backoff 1,2,4,8,10 s ─┐
        ▲                                                     │ ok                                     │
        │                                                     ▼                                        │
        │                                              STREAMING ──read fails (1st, 10th logged)        │
        │                                                     │ 10 consecutive failures                │
        │                                                     ▼                                        │
        └──────── cool-down 3→6→12→20 s (reset after 30 good frames) ◄── release device ◄──────────────┘
```
Reopening instantly after a failed streak was observed to keep a misbehaving camera in a reset loop for hours, hence the settle wait and growing cool-down. On shutdown the capture thread is **joined before the device is released**.

**Fusion mode** (`SensorFusionNode`): `FULL` (3-way matches arriving) ⇄ `DEGRADED` (no 3-way fusion for `thermal_timeout_s` = 1.0 s on a monotonic clock → LiDAR+RGB with `thermal = NaN`). Re-entering FULL is logged. Keyed on *success of fusion*, not on whether thermal frames arrive, so unsyncable thermal (e.g. clock skew) also degrades gracefully instead of blanking the viewer.

**Merge window** (`RegistrationMergeNode`): `EMPTY` → (first scan) `ACCUMULATING` → (span ≥ 0.5 s **or** 60 scans) `CLOSED` → queue → worker → `PUBLISHED`; a backward stamp jump > 1 s returns `ACCUMULATING` to `EMPTY` (buffer dropped); a waiting window is replaced by a newer one; a window older than 1 s when the worker reaches it is discarded.

**Restamp filter** (`LivoxRestampNode._estimate`): `TRACKING` (min over a sliding window) → on *device counter went backwards by > 1 s* or *host/device disagreement > 0.1 s* → `RESET` (clear, take the next sample) → `TRACKING`.

## 13. Algorithms and the numbers behind them

* **Pinhole projection.** For a point `p` in the LiDAR frame: `q = p·M + b` (the extrinsic chain folded once at start-up), `u = fx·qx/qz + cx`, `v = fy·qy/qz + cy`; a point is *valid* if `qz > 0` and `0 ≤ u < W`, `0 ≤ v < H`. RGB and thermal reuse one projection when their intrinsics and extrinsic are identical (they are co-located here). Calibration is approximate (placeholder RGB intrinsics; thermal solved from one capture).
* **Min-delay clock offset.** `observed = true_offset + latency`, latency ≥ 0 ⇒ the window minimum is the best estimate (NTP/PTP filtering). Window 200 scans ≈ 20 s tracks crystal drift; the measured window jitter is 25–42 ms (max − min arrival latency), i.e. the stamp error is bounded by the *minimum* latency, not the average.
* **ICP (point-to-point).** Iterate: nearest neighbours of the working source in the target (cKDTree, distance bound 0.5 m) → Kabsch (SVD of the cross-covariance) → accumulate `(R,t)` → stop when the mean error changes < 1e-5. Subsampling to 1 500/10 000 points and a 0.2 m voxel keep a window at ≈ 0.6 s. Measured: consecutive merged clouds of a static scene differ by ≈ 10 mm translation (p50 9.5 mm) — the practical registration repeatability floor.
* **Voxel downsample.** `key = (ix << 42) | (iy << 21) | iz` (hashed int64) then `np.unique(return_index)`; bit-identical to `np.unique(axis=0)` but ≈ 4.8× faster. 0.01 m output voxel removes ≈ 35 % of the merged points (152 k → 98 k).
* **Quantisation.** `scale = extent / 32000`, `q = round((p − min)/scale)` as int16. Error ≤ `√3·scale/2`: 0.14 mm for a 5 m scene (measured max 0.144 mm).
* **Lazy encode cost.** `encode(n) ≈ 0.7 µs·n + gzip(n)`; at 60 000 points ≈ 42 ms + 47 ms, paid once per update regardless of client count.

## 14. Failure-handling taxonomy (what the code does for each class of fault)

| Fault class | Detection | Response | Tested by |
|---|---|---|---|
| Process crash / SIGKILL | launch `respawn=True` | restart in 2 s; stale shm cleaned at service start; child `rpicam-vid` dies with parent | crash-recovery suite (7 nodes, all 2 s) |
| Service death | systemd `Restart=on-failure` | restart after 5 s (≤ 10 in 10 min) | restart cases |
| Bad data (NaN/Inf/empty/identical points) | filter at fusion entry and merge entry; per-callback `try/except` | drop the point/scan/window, keep running | merge P2–P4, fusion F7, streamer S11–S14 |
| Clock rewind / step | restamp (both directions), fusion queue clear, merge window reset, monotonic timeouts | recover immediately | restamp unit test, merge P5–P7, fusion F8, **a real +7 333 s NTP step** |
| Frozen stamps | scan-count cap (60) | window closes, memory bounded | merge-stall test |
| Sensor missing / unsyncable | fusion fallback timeout | LiDAR+RGB with thermal NaN | fusion F2–F6 |
| Camera device loss / stuck | settle wait, growing cool-down, throttled warnings; external watchdog | node stays alive; watchdog power-cycles the port once permitted | thermal-absent test, camera soak, watchdog |
| Overload | size-1 queue, replace-stale, drop-oldest | bounded latency | merge P8–P9 (4×) |
| Many/slow/aborting HTTP clients | thread per client, swallowed connection errors, shared encode, 304 | no errors | streamer S8–S10 (16 clients, 60 aborts) |
| Leaked shared memory | `clean_stale_shm.py` at start | freed | restart cases (shm count stable) |
| Unclean device close | join capture thread before release; SIGTERM/SIGINT handled | camera closed cleanly | restart loop (6/6 recoveries) |

## 15. Test architecture

Isolated **component suites** run each node as a real subprocess in its own ROS domain and drive it with synthetic topics (so they can inject faults no sensor will produce on demand): `t_merge` (13 phases), `t_fusion` (14), `t_streamer` (15 HTTP-level checks), `t_restamp` (estimator, old vs new code), `t_merge_stall`. **System tests** act on the live service: SIGKILL each node and verify respawn, data flow and a single `rpicam-vid`; graceful restarts; the stress ramp; a 30-minute soak with viewer clients; the point-cap sweep. **Measurement tools** (`probe_topics`, `chars`, `threads`, `http_bench`, `net_bench`, `accuracy`) are separate processes that load the pipeline's DDS profile, because a subscriber without it receives large messages over lossy UDP loopback and under-reports every rate.

## 16. Maintaining and extending the code

1. **Adding a node:** copy the pattern of `livox_restamp_node.py` — declare parameters, create the publisher/subscriber with `qos_profile_sensor_data`, keep callbacks short, wrap work in `try/except`, hand heavy work to a worker through a size-1 replace-stale queue, handle SIGTERM by returning from `spin`, add `respawn=True` to its launch entry, add a corner-case suite.
2. **Changing a transport-sensitive message size:** re-run `transport_test.py` for the new size, and keep the DDS profile loaded in every tool.
3. **Tuning:** the supported runtime knobs are the streamer caps (`ros2 param set /fusion_web_streamer max_points_merged N`), fusion `sync_slop`/`queue_size`/`thermal_timeout_s`, merge `window_duration` and ICP limits, restamp `clock_step_threshold_s` (keep it below `sync_slop`).
4. **Known limits and future work:** RGB frames are stamped on arrival rather than at exposure (use `rpicam-vid` timestamps); calibration is approximate (run a checkerboard calibration); the thermal channel is relative AGC luma (a radiometric camera would remove that); the streamer sends a random subsample (a spatially uniform or importance-based subsample would retain more structure at the same cap); merged output is ICP-limited to ≈ 1.5 Hz.


---

# 17. Function-by-function walkthrough (generated from the source)

For each production file: its purpose (module docstring), then every class and function with signature, length, complexity and docstring. Complexity = 1 + branches/loops/handlers/boolean operators.

## `pitfusion3d_bringup/pitfusion3d_bringup/livox_restamp_node.py`  (149 lines, 4 functions, max complexity 4)

> Re-stamp the Livox driver's PointCloud2 into the host's clock domain.

Imports: `collections`, `rclpy`, `sensor_msgs`

* **`LivoxRestampNode.__init__()`** — line 51, 38 lines, complexity 1. *(no docstring)*
* **`LivoxRestampNode._estimate(arrival_ns, device_ns)`** — line 90, 27 lines, complexity 4. Update the offset estimate for one scan; returns the corrected stamp (ns). Pure function of its inputs plus the filter state, so it can be unit-tested without a running clock.
* **`LivoxRestampNode._on_cloud(msg)`** — line 118, 15 lines, complexity 2. *(no docstring)*
* **`main(args)`** — line 135, 11 lines, complexity 3. *(no docstring)*

## `pitfusion3d_bringup/pitfusion3d_bringup/rgb_camera_node.py`  (234 lines, 9 functions, max complexity 10)

> Publish the Raspberry Pi (imx477) camera as sensor_msgs/Image.

Imports: `ctypes`, `cv2`, `cv_bridge`, `numpy`, `rclpy`, `sensor_msgs`, `signal`, `subprocess`, `threading`, `time`

* **`_die_with_parent()`** — line 41, 8 lines, complexity 2. *(no docstring)*
* **`RgbCameraNode.__init__()`** — line 52, 59 lines, complexity 1. *(no docstring)*
* **`RgbCameraNode._spawn()`** — line 112, 21 lines, complexity 1. *(no docstring)*
* **`RgbCameraNode._read_exact(n)`** — line 134, 10 lines, complexity 4. *(no docstring)*
* **`RgbCameraNode._publish_frame(raw, stamp)`** — line 145, 17 lines, complexity 2. Convert one yuv420 frame to rgb8 and publish it with the given stamp. This runs in a daemon thread, so an uncaught exception (a malformed frame, a shape mismatch) would silently kill only this thread: no exception reaches rclpy.spin(), no process exits for systemd to notice, and the node would look healthy while never publishing again. Catch and skip the frame instead.
* **`RgbCameraNode._capture_loop()`** — line 163, 42 lines, complexity 10. *(no docstring)*
* **`RgbCameraNode._terminate_process()`** — line 206, 7 lines, complexity 4. *(no docstring)*
* **`RgbCameraNode.destroy_node()`** — line 214, 4 lines, complexity 1. *(no docstring)*
* **`main(args)`** — line 220, 11 lines, complexity 3. *(no docstring)*

## `pitfusion3d_bringup/pitfusion3d_bringup/thermal_camera_node.py`  (314 lines, 15 functions, max complexity 8)

> Publish the UVC thermal camera (640x512 YUYV) as sensor_msgs/Image.

Imports: `cv2`, `cv_bridge`, `numpy`, `os`, `rclpy`, `sensor_msgs`, `threading`, `time`

* **`_CaptureHealth.__init__()`** — line 52, 5 lines, complexity 1. *(no docstring)*
* **`_CaptureHealth.on_opened()`** — line 58, 4 lines, complexity 1. *(no docstring)*
* **`_CaptureHealth.on_good_frame()`** — line 63, 5 lines, complexity 2. *(no docstring)*
* **`ThermalCameraNode.__init__()`** — line 71, 78 lines, complexity 2. *(no docstring)*
* **`ThermalCameraNode._sleep_while_running(seconds)`** — line 150, 4 lines, complexity 3. *(no docstring)*
* **`ThermalCameraNode._wait_for_device()`** — line 155, 22 lines, complexity 8. Block until the device node has existed continuously for device_settle_s (i.e. USB enumeration has finished, not just begun).
* **`ThermalCameraNode._open_capture()`** — line 178, 11 lines, complexity 2. *(no docstring)*
* **`ThermalCameraNode._capture_loop()`** — line 190, 16 lines, complexity 6. Self-healing capture loop: wait for the device, open it, read, publish, recover. On boot (systemd/lingering, nobody watching) this USB device can take seconds to enumerate, and a cable hiccup in the field must not need a human. The loop therefore retries instead of raising: an exception would kill only this thread (or, if raised in __init__, the node's process, which ros2 launch does not propagate), leaving a healthy
* **`ThermalCameraNode._connect(health)`** — line 207, 18 lines, complexity 3. Wait for the device and open it. Returns True once streaming; False means retry/shutdown.
* **`ThermalCameraNode._read_frame(health)`** — line 226, 8 lines, complexity 3. Read one raw frame, or None after handling a failed read.
* **`ThermalCameraNode._on_read_failure(health)`** — line 235, 22 lines, complexity 3. Count a failed read; after 10 in a row release the device and cool down. A handful of isolated drops is normal; many in a row means the device is gone or misbehaving. Reopen after a growing cool-down (3 s doubling to the configured maximum) rather than busy-looping on a dead capture: reopening instantly was observed to keep a faulty camera in a reset loop for hours.
* **`ThermalCameraNode._publish_frame(raw)`** — line 258, 21 lines, complexity 5. Convert a raw YUYV frame to rotated mono16 and publish it. Runs in a daemon thread, so an uncaught exception (e.g. raw not actually being height*width*2 bytes although read() reported success) would silently kill only this thread; catch and skip the frame instead.
* **`ThermalCameraNode._publish_image(publisher, array, encoding, stamp)`** — line 280, 5 lines, complexity 1. *(no docstring)*
* **`ThermalCameraNode.destroy_node()`** — line 286, 12 lines, complexity 4. *(no docstring)*
* **`main(args)`** — line 300, 11 lines, complexity 3. *(no docstring)*

## `landfill_camera_sim/scripts/calibration_publisher.py`  (168 lines, 8 functions, max complexity 6)

Imports: `ament_index_python`, `geometry_msgs`, `rclpy`, `scipy`, `sensor_msgs`, `tf2_ros`, `yaml`

* **`_package_path(relative)`** — line 15, 2 lines, complexity 1. *(no docstring)*
* **`load_extrinsics(path)`** — line 19, 24 lines, complexity 5. Read config/calibration/extrinsics.yaml. Returns (reference_frame, [(child_frame, (x, y, z), (roll, pitch, yaw)), ...]).
* **`load_camera_intrinsics(path, camera_key)`** — line 45, 30 lines, complexity 6. Read one camera's block (rgb_camera/thermal_camera) from intrinsics.yaml.
* **`make_camera_info(intrinsics, frame_id)`** — line 77, 12 lines, complexity 1. *(no docstring)*
* **`CalibrationPublisher.__init__()`** — line 92, 38 lines, complexity 2. *(no docstring)*
* **`CalibrationPublisher._build_transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw)`** — line 132, 13 lines, complexity 1. *(no docstring)*
* **`CalibrationPublisher._on_timer()`** — line 146, 6 lines, complexity 1. *(no docstring)*
* **`main()`** — line 154, 11 lines, complexity 3. *(no docstring)*

## `landfill_camera_sim/scripts/cloud_io.py`  (110 lines, 3 functions, max complexity 10)

> Zero-copy PointCloud2 <-> numpy helpers.

Imports: `numpy`, `sensor_msgs`

* **`structured_view(msg)`** — line 28, 23 lines, complexity 5. View msg.data as a structured array honouring the real field offsets. Zero-copy: padded layouts (e.g. gz-sim's 32-byte stride) are described by the dtype's itemsize rather than being repacked.
* **`read_fields(msg, field_names, dtype)`** — line 53, 38 lines, complexity 10. Return an (N, len(field_names)) array of the requested fields. When the fields are consecutive float32 values (the layout every cloud in this package uses) the whole block is taken as one strided slice, which costs a single bulk copy instead of one gather per field.
* **`make_cloud(header, field_names, payload)`** — line 93, 18 lines, complexity 2. Build a dense PointCloud2 from an (N, F) float32 array.

## `landfill_camera_sim/scripts/sensor_fusion_node.py`  (427 lines, 14 functions, max complexity 15)

> Colorize lidar points with time-synced RGB + thermal camera samples.

Imports: `ament_index_python`, `cloud_io`, `csv`, `message_filters`, `numpy`, `os`, `rclpy`, `scipy`, `sensor_msgs`, `time`, `yaml`

* **`_package_path(relative)`** — line 32, 2 lines, complexity 1. *(no docstring)*
* **`_load_extrinsics(path)`** — line 36, 21 lines, complexity 5. *(no docstring)*
* **`_load_camera_intrinsics(path, camera_key)`** — line 59, 13 lines, complexity 3. *(no docstring)*
* **`lidar_to_camera_transform(extrinsic_frames, lidar_frame, camera_frame)`** — line 74, 12 lines, complexity 1. Collapse rig-frame hops into one (M, b) so points map with xyz @ M + b. extrinsic_frames maps child_frame -> (R, t) with p_reference = R @ p_child + t, so p_camera = R_cam.T @ (R_lidar @ p_lidar + t_lidar - t_cam), which is the same as p_lidar @ (R_lidar.T @ R_cam) + (t_lidar - t_cam) @ R_cam.
* **`project_to_pixels(points_camera_frame, fx, fy, cx, cy, width, height)`** — line 88, 15 lines, complexity 1. Rig/robot axis convention (x forward, y left, z up) -> pixel coords.
* **`image_to_array(msg, channels, dtype)`** — line 105, 12 lines, complexity 5. View an Image message's buffer without copying, honouring msg.step.
* **`SensorFusionNode.__init__()`** — line 120, 120 lines, complexity 4. *(no docstring)*
* **`SensorFusionNode._watch_clock(cloud_msg)`** — line 241, 21 lines, complexity 7. Drop cached messages when the clock rewinds. Restarting the simulator resets sim time; the synchronizer would otherwise keep matching against queued messages stamped in the future and never emit again.
* **`SensorFusionNode._on_synced_full(cloud_msg, rgb_msg, thermal_msg)`** — line 263, 6 lines, complexity 2. *(no docstring)*
* **`SensorFusionNode._thermal_stale()`** — line 270, 6 lines, complexity 2. *(no docstring)*
* **`SensorFusionNode._on_synced_no_thermal(cloud_msg, rgb_msg)`** — line 277, 9 lines, complexity 3. *(no docstring)*
* **`SensorFusionNode._on_synced(cloud_msg, rgb_msg, thermal_msg)`** — line 287, 18 lines, complexity 4. *(no docstring)*
* **`SensorFusionNode._process_synced(cloud_msg, rgb_msg, thermal_msg)`** — line 306, 105 lines, complexity 15. *(no docstring)*
* **`main()`** — line 413, 11 lines, complexity 3. *(no docstring)*

## `landfill_camera_sim/scripts/registration_merge_node.py`  (403 lines, 13 functions, max complexity 9)

> Merge a rolling window of fused (RGB+thermal-colored) lidar scans.

Imports: `cloud_io`, `numpy`, `queue`, `rclpy`, `scipy`, `sensor_msgs`, `std_msgs`, `threading`, `time`

* **`voxel_downsample_indices(xyz, voxel_size)`** — line 32, 22 lines, complexity 4. Indices of one representative point per voxel. Hashes the 3 integer voxel coordinates into a single int64 key so the dedupe is a 1-D sort; np.unique(..., axis=0) instead lexsorts an (N, 3) array, which is markedly slower for the ~1e5-point clouds seen here.
* **`_kabsch(source, target)`** — line 56, 13 lines, complexity 1. Rigid R, t minimizing ||target - (source @ R.T + t)||^2.
* **`icp_point_to_point(source_xyz, target_xyz, max_iterations, max_correspondence_dist, min_correspondences, workers)`** — line 76, 37 lines, complexity 9. Point-to-point ICP. Returns (R, t, converged) aligning source onto target.
* **`RegistrationMergeNode.__init__()`** — line 116, 91 lines, complexity 1. *(no docstring)*
* **`RegistrationMergeNode._on_cloud(msg)`** — line 208, 13 lines, complexity 4. Accumulate one fused scan; hand the window to the worker when it is complete.
* **`RegistrationMergeNode._finite_points(msg)`** — line 223, 13 lines, complexity 4. Return the cloud's (N, 6) array without non-finite points, or None if nothing is left. Defence in depth: fusion already drops non-finite points, but one NaN reaching ICP/SVD used to discard the whole 5-scan window.
* **`RegistrationMergeNode._reset_window_if_clock_rewound(stamp_sec)`** — line 237, 16 lines, complexity 3. Start a fresh window if stamps jumped backwards by more than 1 s. A sim restart rewinds the clock; without this the window never closes again because stamp_sec stays below the stale start. The 1 s tolerance (matching sensor_fusion_node.py) is deliberate: this Pi has no battery-backed RTC, so NTP steps the clock after every boot, and small backward jitter used to reset the window every few seconds, tanking the merged 
* **`RegistrationMergeNode._window_is_complete(stamp_sec)`** — line 254, 4 lines, complexity 2. A window closes when its scans span window_duration, or at the safety scan cap.
* **`RegistrationMergeNode._submit_window()`** — line 259, 30 lines, complexity 5. Hand the closed window to the worker thread, replacing a stale waiting one. Registration is real numerical code with edge cases (degenerate or NaN input); it runs in the worker inside a try/except so one bad window never kills the node (systemd's restart is only a much slower backstop).
* **`RegistrationMergeNode._worker_loop()`** — line 290, 15 lines, complexity 5. *(no docstring)*
* **`RegistrationMergeNode._subsample(xyz, limit)`** — line 306, 4 lines, complexity 3. *(no docstring)*
* **`RegistrationMergeNode._register_and_publish(buffer)`** — line 311, 76 lines, complexity 7. *(no docstring)*
* **`main()`** — line 389, 11 lines, complexity 3. *(no docstring)*

## `landfill_camera_sim/scripts/fusion_web_streamer.py`  (802 lines, 19 functions, max complexity 8)

> Serve the fused RGB+thermal point cloud to any browser on the network.

Imports: `gzip`, `http`, `numpy`, `os`, `rcl_interfaces`, `rclpy`, `sensor_msgs`, `sensor_msgs_py`, `socketserver`, `struct`, `sys`, `threading`

* **`_ReusableThreadingTCPServer.handle_error(request, client_address)`** — line 375, 7 lines, complexity 2. *(no docstring)*
* **`_StreamState.__init__()`** — line 392, 14 lines, complexity 1. *(no docstring)*
* **`_StreamState.offer(msg)`** — line 407, 3 lines, complexity 1. *(no docstring)*
* **`_StreamState.refresh()`** — line 411, 10 lines, complexity 6. *(no docstring)*
* **`_StreamState.set_payload(n_points, payload_bytes)`** — line 422, 12 lines, complexity 2. *(no docstring)*
* **`_StreamState.get_buffer_and_seq()`** — line 435, 3 lines, complexity 2. *(no docstring)*
* **`_SharedState.__init__()`** — line 441, 3 lines, complexity 1. *(no docstring)*
* **`_Handler.log_message(format_)`** — line 490, 2 lines, complexity 1. *(no docstring)*
* **`_Handler._serve_bytes(body, content_type)`** — line 493, 11 lines, complexity 2. *(no docstring)*
* **`_Handler._serve_cloud(stream_state)`** — line 505, 45 lines, complexity 4. Conditional GET keyed on the stream's sequence number. Point clouds only actually change at ~1-10Hz while browsers poll faster than that (to catch an update quickly once it lands), so most polls would otherwise re-download an unchanged multi-hundred- KB payload for nothing -- fatal to reliability on a real WiFi link (this is the difference between the viewer being unusable and smooth from another room). Standard HTTP
* **`_Handler.do_GET()`** — line 551, 17 lines, complexity 6. *(no docstring)*
* **`_encode_cloud(msg, max_points, rng)`** — line 573, 56 lines, complexity 3. Pack a cloud into a compact quantized wire format. Was 7 x float32 = 28 bytes/point. Over a real WiFi link (hotspot or a router-mediated LAN), transfer time of that payload -- not the pipeline's own production rate -- turned out to be the actual bottleneck on the viewer's observed refresh rate (measured: pipeline producing merged clouds at ~1.4Hz internally, browser only seeing ~0.7Hz). Point position only needs to b
* **`_load_static_assets()`** — line 634, 16 lines, complexity 3. Read three.min.js/OrbitControls.js off disk once at startup. See this module's docstring for why these are files here instead of CDN URLs in the page.
* **`FusionWebStreamer.__init__()`** — line 653, 104 lines, complexity 3. *(no docstring)*
* **`FusionWebStreamer._on_param_change(params)`** — line 758, 11 lines, complexity 8. *(no docstring)*
* **`FusionWebStreamer._on_merged(msg)`** — line 770, 9 lines, complexity 1. *(no docstring)*
* **`FusionWebStreamer._on_live(msg)`** — line 780, 2 lines, complexity 1. *(no docstring)*
* **`FusionWebStreamer.destroy_node()`** — line 783, 3 lines, complexity 1. *(no docstring)*
* **`main()`** — line 788, 11 lines, complexity 3. *(no docstring)*

## `pitfusion3d_bringup/scripts/thermal_view.py`  (161 lines, 14 functions, max complexity 6)

> Thermal-camera-only viewer.

Imports: `cv2`, `http`, `numpy`, `rclpy`, `sensor_msgs`, `socketserver`, `threading`, `time`

* **`LatestFrame.__init__()`** — line 43, 5 lines, complexity 1. *(no docstring)*
* **`LatestFrame.set(image)`** — line 49, 3 lines, complexity 2. *(no docstring)*
* **`LatestFrame.touch()`** — line 53, 3 lines, complexity 1. Record that a client asked for a picture (keeps the ROS subscription alive).
* **`LatestFrame.wanted()`** — line 57, 3 lines, complexity 1. True while a client has asked for a picture recently.
* **`LatestFrame.get()`** — line 61, 6 lines, complexity 4. Return the newest frame, or None if there is none or it is stale.
* **`no_signal_picture()`** — line 72, 7 lines, complexity 1. A 640x512 black picture explaining that the camera is not delivering frames.
* **`encode_jpeg()`** — line 81, 9 lines, complexity 2. Colourise the newest frame (INFERNO palette) and return it as JPEG bytes.
* **`Handler.log_message()`** — line 93, 2 lines, complexity 1. Silence the default per-request logging.
* **`Handler._reply(status, content_type, body)`** — line 96, 7 lines, complexity 2. *(no docstring)*
* **`Handler._stream()`** — line 104, 10 lines, complexity 2. *(no docstring)*
* **`Handler.do_GET()`** — line 115, 12 lines, complexity 5. *(no docstring)*
* **`on_image(msg)`** — line 134, 3 lines, complexity 2. *(no docstring)*
* **`main()`** — line 139, 19 lines, complexity 6. *(no docstring)*
* **`manage_subscription()`** — line 144, 7 lines, complexity 5. Subscribe while someone is watching, unsubscribe when nobody has asked for a while.

## `pitfusion3d_bringup/scripts/clean_stale_shm.py`  (46 lines, 0 functions, max complexity 0)

> Remove Fast DDS shared-memory files in /dev/shm that no running process has mapped/open. Fast DDS only deletes its segments on a clean exit, so a crash / kill -9 / OOM leaves one segment per participant behind; with the 64MB segments used here that is ~0.5GB of RAM per unclean shutdown. Only touches this user's fastrtps files and never a file a live process uses (so it is safe even if another ROS tool is running). Usage: clean_stale_shm.py [--dry-run]

Imports: `glob`, `os`, `psutil`, `sys`, `time`


## `pitfusion3d_bringup/launch/real_sensors.launch.py`  (174 lines, 1 functions, max complexity 2)

> Bring up the physical sensor rig: Livox Avia + RPi RGB camera + UVC thermal camera.

Imports: `ament_index_python`, `launch`, `launch_ros`, `os`

* **`generate_launch_description()`** — line 34, 141 lines, complexity 2. *(no docstring)*

## `pitfusion3d_bringup/launch/pitfusion3d_full.launch.py`  (132 lines, 1 functions, max complexity 1)

> One-shot launch for the whole real-hardware PitFusion3D pipeline:

Imports: `launch`, `launch_ros`

* **`generate_launch_description()`** — line 23, 110 lines, complexity 1. *(no docstring)*


---

# Appendix A — Declared ROS parameters (from static analysis)

| file | parameter | default | line |
|---|---|---|---|
| `calibration_publisher.py` | `extrinsics_path` | `_package_path('config/calibration/extrinsics.yaml')` | 95 |
| `calibration_publisher.py` | `intrinsics_path` | `_package_path('config/calibration/intrinsics.yaml')` | 96 |
| `calibration_publisher.py` | `image_frame_id` | `camera_platform/camera_link` | 97 |
| `calibration_publisher.py` | `rgb_camera_info_topic` | `/landfill/rgb/camera_info` | 98 |
| `calibration_publisher.py` | `thermal_camera_info_topic` | `/landfill/thermal/camera_info` | 99 |
| `calibration_publisher.py` | `publish_rate` | `30.0` | 100 |
| `fusion_web_streamer.py` | `merged_topic` | `/landfill/fusion/points_merged` | 656 |
| `fusion_web_streamer.py` | `live_topic` | `/landfill/fusion/points_rgbt` | 657 |
| `fusion_web_streamer.py` | `http_port` | `8080` | 658 |
| `fusion_web_streamer.py` | `max_points_merged` | `20000` | 700 |
| `fusion_web_streamer.py` | `max_points_live` | `5000` | 701 |
| `fusion_web_streamer.py` | `input_topic` | `` | 704 |
| `fusion_web_streamer.py` | `http_keepalive` | `True` | 710 |
| `livox_pattern_filter.py` | `input_points_topic` | `/landfill/livox/raw_scan/points` | 94 |
| `livox_pattern_filter.py` | `output_points_topic` | `/landfill/livox/points` | 95 |
| `livox_pattern_filter.py` | `frame_id` | `camera_platform/camera_link` | 96 |
| `livox_pattern_filter.py` | `horizontal_samples` | `200` | 97 |
| `livox_pattern_filter.py` | `vertical_samples` | `120` | 98 |
| `livox_pattern_filter.py` | `points_per_scan` | `24000` | 99 |
| `livox_pattern_filter.py` | `downsample` | `1` | 100 |
| `livox_pattern_filter.py` | `horizontal_min_angle` | `-0.614356` | 101 |
| `livox_pattern_filter.py` | `horizontal_max_angle` | `0.614356` | 102 |
| `livox_pattern_filter.py` | `vertical_min_angle` | `-0.673697` | 103 |
| `livox_pattern_filter.py` | `vertical_max_angle` | `0.673697` | 104 |
| `livox_pattern_filter.py` | `range_min` | `0.1` | 105 |
| `livox_pattern_filter.py` | `range_max` | `200.0` | 106 |
| `livox_pattern_filter.py` | `scan_pattern_csv` | `default_csv` | 107 |
| `livox_pattern_filter.py` | `interpolation_depth_tolerance` | `0.5` | 111 |
| `platform_motion_controller.py` | `cmd_topic` | `/landfill/platform/cmd_vel` | 15 |
| `platform_motion_controller.py` | `axis` | `[0.0, 1.0, 0.0]` | 16 |
| `platform_motion_controller.py` | `travel_distance` | `8.0` | 17 |
| `platform_motion_controller.py` | `linear_speed` | `0.35` | 18 |
| `platform_motion_controller.py` | `angular_speed` | `0.18` | 19 |
| `platform_motion_controller.py` | `update_rate` | `20.0` | 20 |
| `registration_merge_node.py` | `input_topic` | `/landfill/fusion/points_rgbt` | 119 |
| `registration_merge_node.py` | `output_topic` | `/landfill/fusion/points_merged` | 120 |
| `registration_merge_node.py` | `window_duration` | `1.0` | 121 |
| `registration_merge_node.py` | `icp_voxel_size` | `0.2` | 140 |
| `registration_merge_node.py` | `max_icp_iterations` | `10` | 141 |
| `registration_merge_node.py` | `max_correspondence_distance` | `0.5` | 142 |
| `registration_merge_node.py` | `min_correspondences` | `200` | 143 |
| `registration_merge_node.py` | `output_voxel_size` | `0.01` | 147 |
| `registration_merge_node.py` | `max_icp_target_points` | `10000` | 150 |
| `registration_merge_node.py` | `max_icp_source_points` | `1500` | 154 |
| `scene_traffic_controller.py` | `update_rate` | `20.0` | 88 |
| `scene_traffic_controller.py` | `topic_template` | `/model/{name}/cmd_vel` | 89 |
| `scene_traffic_controller.py` | `vehicles` | `[row[0] for row in DEFAULT_ROUTES]` | 90 |
| `sensor_fusion_node.py` | `lidar_points_topic` | `/landfill/livox/points` | 123 |
| `sensor_fusion_node.py` | `rgb_image_topic` | `/landfill/rgb/image_raw` | 124 |
| `sensor_fusion_node.py` | `thermal_image_topic` | `/landfill/thermal/image_raw` | 125 |
| `sensor_fusion_node.py` | `output_topic` | `/landfill/fusion/points_rgbt` | 126 |
| `sensor_fusion_node.py` | `extrinsics_path` | `_package_path('config/calibration/extrinsics.yaml')` | 127 |
| `sensor_fusion_node.py` | `intrinsics_path` | `_package_path('config/calibration/intrinsics.yaml')` | 128 |
| `sensor_fusion_node.py` | `lidar_frame` | `camera_platform/lidar_frame` | 129 |
| `sensor_fusion_node.py` | `rgb_frame` | `camera_platform/rgb_optical_frame` | 130 |
| `sensor_fusion_node.py` | `thermal_frame` | `camera_platform/thermal_optical_frame` | 131 |
| `sensor_fusion_node.py` | `thermal_min_temp` | `253.15` | 132 |
| `sensor_fusion_node.py` | `thermal_resolution` | `0.01` | 133 |
| `sensor_fusion_node.py` | `sync_slop` | `0.15` | 134 |
| `sensor_fusion_node.py` | `queue_size` | `5` | 135 |
| `sensor_fusion_node.py` | `thermal_timeout_s` | `1.0` | 141 |
| `camera_viewer.py` | `rgb_topic` | `/landfill/rgb/image_raw` | 24 |
| `camera_viewer.py` | `thermal_topic` | `/landfill/thermal/image_raw` | 25 |
| `livox_restamp_node.py` | `input_topic` | `livox/lidar` | 53 |
| `livox_restamp_node.py` | `output_topic` | `/landfill/livox/points` | 54 |
| `livox_restamp_node.py` | `offset_window` | `200` | 55 |
| `livox_restamp_node.py` | `log_period` | `50` | 56 |
| `livox_restamp_node.py` | `clock_step_threshold_s` | `0.1` | 57 |
| `rgb_camera_node.py` | `topic` | `/landfill/rgb/image_raw` | 55 |
| `rgb_camera_node.py` | `frame_id` | `camera_platform/rgb_optical_frame` | 56 |
| `rgb_camera_node.py` | `width` | `1280` | 57 |
| `rgb_camera_node.py` | `height` | `720` | 58 |
| `rgb_camera_node.py` | `framerate` | `15` | 64 |
| `rgb_camera_node.py` | `saturation` | `1.8` | 86 |
| `thermal_camera_node.py` | `topic` | `/landfill/thermal/image_raw` | 74 |
| `thermal_camera_node.py` | `frame_id` | `camera_platform/thermal_optical_frame` | 75 |
| `thermal_camera_node.py` | `device` | `/dev/v4l/by-id/usb-Thermal_Cam_Co._Ltd_Camera_202206223-video-index0` | 76 |
| `thermal_camera_node.py` | `width` | `640` | 80 |
| `thermal_camera_node.py` | `height` | `512` | 81 |
| `thermal_camera_node.py` | `framerate` | `30` | 89 |
| `thermal_camera_node.py` | `publish_preview` | `False` | 93 |
| `thermal_camera_node.py` | `preview_topic` | `/landfill/thermal/image_preview` | 94 |
| `thermal_camera_node.py` | `vflip` | `True` | 106 |
| `thermal_camera_node.py` | `hflip` | `True` | 107 |
| `thermal_camera_node.py` | `device_settle_s` | `3.0` | 140 |
| `thermal_camera_node.py` | `recovery_cooldown_max_s` | `20.0` | 141 |


# Appendix B — Source file metrics

| file | lines | code | comment | functions | classes | max complexity | mean complexity |
|---|---|---|---|---|---|---|---|
| landfill_camera_sim/scripts/fusion_web_streamer.py | 802 | 598 | 132 | 19 | 5 | 8 | 2.68 |
| landfill_camera_sim/scripts/sensor_fusion_node.py | 427 | 330 | 50 | 14 | 1 | 15 | 4.0 |
| landfill_camera_sim/scripts/registration_merge_node.py | 403 | 294 | 53 | 13 | 1 | 9 | 3.92 |
| landfill_camera_sim/scripts/livox_pattern_filter.py | 342 | 276 | 15 | 8 | 1 | 9 | 4.0 |
| pitfusion3d_bringup/pitfusion3d_bringup/thermal_camera_node.py | 314 | 238 | 41 | 15 | 2 | 8 | 3.13 |
| pitfusion3d_bringup/pitfusion3d_bringup/rgb_camera_node.py | 234 | 153 | 54 | 9 | 1 | 10 | 3.11 |
| landfill_camera_sim/scripts/calib_solve.py | 218 | 161 | 26 | 7 | 0 | 13 | 4.71 |
| pitfusion3d_bringup/launch/real_sensors.launch.py | 174 | 129 | 29 | 1 | 0 | 2 | 2.0 |
| landfill_camera_sim/scripts/calibration_publisher.py | 168 | 142 | 1 | 8 | 1 | 6 | 2.5 |
| pitfusion3d_bringup/scripts/thermal_view.py | 161 | 126 | 1 | 14 | 3 | 6 | 2.5 |
| pitfusion3d_bringup/pitfusion3d_bringup/livox_restamp_node.py | 149 | 123 | 9 | 4 | 1 | 4 | 2.5 |
| pitfusion3d_bringup/launch/pitfusion3d_full.launch.py | 132 | 68 | 52 | 1 | 0 | 1 | 1.0 |
| landfill_camera_sim/scripts/scene_traffic_controller.py | 131 | 100 | 6 | 6 | 2 | 5 | 2.5 |
| landfill_camera_sim/scripts/cloud_io.py | 110 | 94 | 2 | 3 | 0 | 10 | 5.67 |
| pitfusion3d_bringup/pitfusion3d_bringup/camera_viewer.py | 91 | 71 | 4 | 4 | 1 | 3 | 1.5 |
| landfill_camera_sim/scripts/platform_motion_controller.py | 86 | 67 | 1 | 3 | 1 | 5 | 4.67 |
| landfill_camera_sim/scripts/calib_capture_pose.py | 83 | 67 | 1 | 5 | 1 | 6 | 2.6 |
| pitfusion3d_bringup/scripts/clean_stale_shm.py | 46 | 43 | 1 | 0 | 0 | 0 | 0.0 |
| pitfusion3d_bringup/setup.py | 33 | 30 | 0 | 0 | 0 | 0 | 0.0 |


**Totals:** 20 Python files, 4,104 lines (3,110 code / 478 comment / 516 blank), 134 functions, 21 classes, 86 parameters. Other files: {"yaml": {"files": 5, "lines": 420}, "xml": {"files": 6, "lines": 188}, "json": {"files": 4, "lines": 209}, "sh": {"files": 4, "lines": 189}, "html": {"files": 0, "lines": 0}, "md": {"files": 7, "lines": 881}, "cfg": {"files": 1, "lines": 4}, "txt": {"files": 4, "lines": 241}, "cpp_vendor_driver": {"files": 72, "lines": 29134}}.


# Appendix C — Topics published/subscribed per file

| file | dir | topic (as written in code) | type |
|---|---|---|---|
| `calib_capture_pose.py` | sub | `/landfill/livox/points` | PointCloud2 |
| `calib_capture_pose.py` | sub | `/landfill/thermal/image_raw` | Image |
| `calibration_publisher.py` | pub | `rgb_topic` | CameraInfo |
| `calibration_publisher.py` | pub | `thermal_topic` | CameraInfo |
| `fusion_web_streamer.py` | sub | `merged_topic` | PointCloud2 |
| `fusion_web_streamer.py` | sub | `live_topic` | PointCloud2 |
| `livox_pattern_filter.py` | pub | `self.output_points_topic` | PointCloud2 |
| `livox_pattern_filter.py` | sub | `self.input_points_topic` | PointCloud2 |
| `platform_motion_controller.py` | pub | `self.cmd_topic` | Twist |
| `registration_merge_node.py` | pub | `output_topic` | PointCloud2 |
| `registration_merge_node.py` | sub | `input_topic` | PointCloud2 |
| `scene_traffic_controller.py` | pub | `topic` | Twist |
| `sensor_fusion_node.py` | pub | `output_topic` | PointCloud2 |
| `sensor_fusion_node.py` | sub | `lidar_topic` | PointCloud2 |
| `sensor_fusion_node.py` | sub | `rgb_topic` | Image |
| `sensor_fusion_node.py` | sub | `thermal_topic` | Image |
| `livox_restamp_node.py` | pub | `str(self.get_parameter('output_topic').value)` | PointCloud2 |
| `livox_restamp_node.py` | sub | `str(self.get_parameter('input_topic').value)` | PointCloud2 |
| `rgb_camera_node.py` | pub | `str(self.get_parameter('topic').value)` | Image |
| `thermal_camera_node.py` | pub | `str(self.get_parameter('topic').value)` | Image |
| `thermal_camera_node.py` | pub | `str(self.get_parameter('preview_topic').value)` | Image |
| `thermal_view.py` | sub | `TOPIC` | Image |


# Appendix D — Functions (length, complexity)

| file | function | line | length | complexity | docstring (first line) |
|---|---|---|---|---|---|
| `calib_capture_pose.py` | `main` | 55 | 25 | 6 |  |
| `calib_solve.py` | `lidar_person_centroid` | 45 | 27 | 5 |  |
| `calib_solve.py` | `thermal_person_centroid` | 74 | 38 | 3 |  |
| `calib_solve.py` | `main` | 129 | 58 | 8 |  |
| `calib_solve.py` | `_apply` | 189 | 26 | 13 |  |
| `calibration_publisher.py` | `load_camera_intrinsics` | 45 | 30 | 6 | Read one camera's block (rgb_camera/thermal_camera) from intrinsics.yaml. |
| `calibration_publisher.py` | `__init__` | 92 | 38 | 2 |  |
| `cloud_io.py` | `read_fields` | 53 | 38 | 10 | Return an (N, len(field_names)) array of the requested fields. |
| `fusion_web_streamer.py` | `_encode_cloud` | 573 | 56 | 3 | Pack a cloud into a compact quantized wire format. |
| `fusion_web_streamer.py` | `refresh` | 411 | 10 | 6 |  |
| `fusion_web_streamer.py` | `_serve_cloud` | 505 | 45 | 4 | Conditional GET keyed on the stream's sequence number. |
| `fusion_web_streamer.py` | `do_GET` | 551 | 17 | 6 |  |
| `fusion_web_streamer.py` | `__init__` | 653 | 104 | 3 |  |
| `fusion_web_streamer.py` | `_on_param_change` | 758 | 11 | 8 |  |
| `livox_pattern_filter.py` | `_load_scan_pattern` | 51 | 28 | 9 | Load (azimuth, zenith) radians from a Livox-SDK scan_mode CSV. |
| `livox_pattern_filter.py` | `__init__` | 82 | 101 | 3 |  |
| `livox_pattern_filter.py` | `_on_points` | 207 | 67 | 8 |  |
| `livox_pattern_filter.py` | `_sample_range` | 275 | 51 | 1 | Bilinearly interpolate the range image at fractional grid coords. |
| `platform_motion_controller.py` | `__init__` | 12 | 35 | 5 |  |
| `registration_merge_node.py` | `icp_point_to_point` | 76 | 37 | 9 | Point-to-point ICP. Returns (R, t, converged) aligning source onto target. |
| `registration_merge_node.py` | `__init__` | 116 | 91 | 1 |  |
| `registration_merge_node.py` | `_submit_window` | 259 | 30 | 5 | Hand the closed window to the worker thread, replacing a stale waiting one. |
| `registration_merge_node.py` | `_register_and_publish` | 311 | 76 | 7 |  |
| `scene_traffic_controller.py` | `__init__` | 86 | 25 | 5 |  |
| `sensor_fusion_node.py` | `__init__` | 120 | 120 | 4 |  |
| `sensor_fusion_node.py` | `_watch_clock` | 241 | 21 | 7 | Drop cached messages when the clock rewinds. |
| `sensor_fusion_node.py` | `_process_synced` | 306 | 105 | 15 |  |
| `pitfusion3d_full.launch.py` | `generate_launch_description` | 23 | 110 | 1 |  |
| `real_sensors.launch.py` | `generate_launch_description` | 34 | 141 | 2 |  |
| `camera_viewer.py` | `synced_callback` | 44 | 27 | 1 |  |
| `livox_restamp_node.py` | `__init__` | 51 | 38 | 1 |  |
| `livox_restamp_node.py` | `_estimate` | 90 | 27 | 4 | Update the offset estimate for one scan; returns the corrected stamp (ns). |
| `rgb_camera_node.py` | `__init__` | 52 | 59 | 1 |  |
| `rgb_camera_node.py` | `_capture_loop` | 163 | 42 | 10 |  |
| `thermal_camera_node.py` | `__init__` | 71 | 78 | 2 |  |
| `thermal_camera_node.py` | `_wait_for_device` | 155 | 22 | 8 | Block until the device node has existed continuously for |
| `thermal_camera_node.py` | `_capture_loop` | 190 | 16 | 6 | Self-healing capture loop: wait for the device, open it, read, publish, recover. |
| `thermal_view.py` | `main` | 139 | 19 | 6 |  |
