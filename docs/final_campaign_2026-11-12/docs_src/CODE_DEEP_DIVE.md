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
