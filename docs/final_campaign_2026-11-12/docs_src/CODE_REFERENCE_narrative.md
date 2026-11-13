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

