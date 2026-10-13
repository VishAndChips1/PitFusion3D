# pitfusion3d_bringup

Real-hardware bringup for [PitFusion3D](https://github.com/VishAndChips1/PitFusion3D)
(`landfill_camera_sim`) -- replaces its Gazebo simulation with the actual
rig: Livox Avia, Raspberry Pi (imx477) RGB camera, and a UVC thermal
camera, feeding the same fusion/merge/web-stream nodes unmodified. Also
includes a standalone-WiFi-hotspot deployment mode for viewing the fused
cloud from a laptop with no other network infrastructure involved.

## Nodes

- **`rgb_camera_node`** -- captures `rpicam-vid -t 0 --width 1280 --height 720
  --framerate 15 --codec yuv420 -o -` over a pipe, converts YUV420 -> RGB,
  publishes `sensor_msgs/Image` (`rgb8`) on `/landfill/rgb/image_raw`.
  15fps, not 30: the Livox only fuses at 10Hz, so capturing/converting
  faster than that is CPU spent on frames nothing consumes.
- **`thermal_camera_node`** -- opens the UVC thermal module (640x512 YUYV)
  via OpenCV/V4L2, publishes the Y (luma) channel as `mono16` on
  `/landfill/thermal/image_raw`. Opened by its stable `/dev/v4l/by-id/...`
  path, not a raw `/dev/videoN` number -- the Pi's own camera ISP
  competes for the same device numbering and can bump this camera to a
  different number across reboots. **This camera only exposes an AGC'd
  8-bit video stream over UVC -- there is no vendor SDK here to switch it
  into a raw/TLinear radiometric mode.** Published values are a relative
  hot/cold signal, not absolute Kelvin. Launch with
  `thermal_min_temp:=0.0 thermal_resolution:=0.00390625` (done
  automatically by `pitfusion3d_full.launch.py`) so the fused cloud's
  `thermal` field comes back as the original 0-255 value. The
  `image_preview` topic (colormapped `bgr8`) is off by default -- pure
  overhead once the web viewer does its own thermal colorization; flip
  `publish_preview:=true` for direct debugging.
- **`livox_restamp_node`** -- re-stamps the Livox driver's `PointCloud2`
  from the lidar's own internal hardware clock into the host's clock
  domain, using a rolling minimum-offset estimate (not just arrival
  time, which would fold network/scheduling jitter into every point's
  timestamp) -- see the node's docstring. Without this,
  `sensor_fusion_node`'s time synchronizer never matches a lidar scan to
  a camera frame, and `points_rgbt` silently stays empty forever even
  though every individual topic looks fine.
- **`camera_viewer`** -- debug-only: opens a synced RGB/thermal side-by-side
  `cv2.imshow` window. Not part of the pipeline.

## Launch

```bash
# sensors only
ros2 launch pitfusion3d_bringup real_sensors.launch.py

# sensors + fusion + merge + web viewer (http://<host>:8080/), one shot
ros2 launch pitfusion3d_bringup pitfusion3d_full.launch.py
```

Both launch files force `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and cap
OpenCV/BLAS/OpenMP to one thread per process -- see "Performance" and
"Standalone WiFi hotspot" below for why both matter.

## Auto-start on boot

For field deployment -- power on, no keyboard/mouse/login, pipeline just
comes up:

```bash
scripts/install_autostart.sh
```

This installs `systemd/pitfusion3d.service` as a **user** systemd unit
(`~/.config/systemd/user/`, no root needed) and enables lingering
(`loginctl enable-linger`) so that user's systemd instance -- and this
service with it -- starts at boot independent of any login. Neither step
needs `sudo`: polkit allows the logged-in user to manage their own
lingering state and systemd --user units directly.

**Stability is layered, deliberately:**
- `rgb_camera_node`/`thermal_camera_node` retry opening their device
  internally (backoff capped at 10s) instead of crashing on a failed
  open, and `rgb_camera_node` respawns `rpicam-vid` the same way if it
  ever dies mid-run. This matters because a node whose capture thread
  dies but whose main thread doesn't **keeps running** -- no exception
  reaches `rclpy.spin()`, so neither the node itself nor `ros2 launch`
  (which doesn't by default propagate a child's exit into its own)
  would otherwise notice or recover. Verified live: killing `rpicam-vid`
  directly gets it respawned and publishing again within ~2s, logged.
- The systemd unit itself has `Restart=on-failure` (`RestartSec=5`) as
  the outer safety net, for failures the nodes' own retry logic can't
  cover (e.g. the whole `ros2 launch` process being killed). Verified
  live: `kill -9` on the main PID gets the whole pipeline back up within
  ~8s. `StartLimitBurst=10` over `StartLimitIntervalSec=600` caps a
  truly broken install at 10 restarts in 10 minutes rather than
  spinning forever, without getting in the way of normal boot-time
  hardware-enumeration variability.
- `systemctl --user stop` (and a crash-triggered restart) correctly
  tears down the *entire* process tree via systemd's cgroup tracking --
  verified no node is ever left behind, which the manual `pkill` pattern
  used everywhere else in this README can't guarantee.

Useful commands:

```bash
systemctl --user status pitfusion3d.service     # is it running, since when
journalctl --user -u pitfusion3d.service -f     # watch logs live
systemctl --user restart pitfusion3d.service    # apply a config change
systemctl --user stop pitfusion3d.service       # stop (stays enabled for next boot)
systemctl --user disable pitfusion3d.service    # turn off auto-start entirely
```

Currently starts on whatever WiFi the Pi normally connects to (so you
keep internet during development) -- auto-starting the standalone
hotspot too is a planned follow-up for actual field deployment, once
everything else is settled.

### Merged cloud crawls for the first few minutes after every boot -- fixed

First real symptom of a genuine reboot test (auto-start itself worked):
the merged cloud's refresh rate collapsed to ~0.2-0.4Hz, way down from
its normal ~1.3-1.8Hz, immediately after power-on.

Root cause, found via `journalctl --user -u pitfusion3d.service`:
`registration_merge_node` was logging `Timestamps jumped backwards;
resetting merge window` every 15-30 seconds, each one discarding
whatever scans had already accumulated in that window and starting
over from nothing -- directly tanking the refresh rate. This Pi has no
battery-backed RTC (`journalctl -u systemd-timesyncd` shows "clock
jumped backwards, restored from recorded timestamp" on *every* boot,
followed by an NTP correction a minute or so later), and
`registration_merge_node`'s reset check had **zero tolerance** -- any
backward movement at all, even a millisecond, triggered a full reset.
`sensor_fusion_node.py` has the identical check but with a sensible
1-second tolerance; `registration_merge_node.py` just never got the
same treatment. Fixed by matching it.

Worth being precise about what actually caused each backward tick,
since "faulty RTC" isn't the whole story: the observed jumps were
small and suspiciously uniform (~100ms), which points more at
`livox_restamp_node`'s own min-offset estimator finding a new
lowest-latency sample in its rolling window (expected behavior of any
min-tracking filter) than at NTP making a correction of that exact,
repeated size. Either way -- post-boot clock settling or ordinary
estimator jitter -- the fix is the same and the 1-second tolerance
covers both. This can recur any time during a long session, not just
at boot, so the fix is a tolerance in the code rather than something
boot-ordering (e.g. `After=time-sync.target`) could fully solve --
that target is also, on testing, not reliably signalled on this system
regardless, and waiting on it would actively break the no-internet
hotspot/field case, where NTP sync can never complete at all.

## Viewing the fused cloud

`fusion_web_streamer` (in `landfill_camera_sim`) serves a self-contained
three.js viewer at `http://<host>:8080/` with a **Merged (dense, ~1.5Hz)**
/ **Live (fast, ~10Hz)** toggle. Locally on the Pi:

```bash
firefox http://localhost:8080/
```

### From another device on the same LAN

Just open `http://<pi-lan-ip>:8080/` from that device. Needs:
- both devices genuinely on the same L2 segment (not a guest network, not
  a different band the router treats as a separate network -- some
  routers/mesh systems do this even when the SSIDs look related)
- the Pi's firewall to allow the port: `sudo ufw allow 8080/tcp`
- no AP/client isolation enabled on the router (common default on mesh
  systems -- lets devices reach the internet but not each other)

`ERR_CONNECTION_TIMED_OUT` in the browser (not `ERR_CONNECTION_REFUSED`)
means packets aren't arriving at all -- that's a network-path problem
(isolation, wrong band/SSID), not anything fixable on the Pi.

### Standalone WiFi hotspot (no router involved at all)

For a field deployment with no existing network, turn the Pi's own WiFi
into an access point instead of relying on someone else's router:

```bash
scripts/start_hotspot_viewing.sh   # switches wlan0 to hotspot, restarts the pipeline
scripts/stop_hotspot_viewing.sh    # reverts to normal WiFi + internet
```

This drops the Pi's own internet access while the hotspot is up (`wlan0`
becomes a standalone AP with no uplink) -- run `stop_hotspot_viewing.sh`
before you need internet on the Pi again. Connect the viewing device to
WiFi `PitFusion3D` (password in the script) and open `http://10.42.0.1:8080/`.

**Two non-obvious bugs had to be fixed to make this work**, both worth
knowing about if you extend this:

1. **ROS2 discovery breaks when `wlan0` changes subnet.** The environment
   ships with `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, which makes DDS
   discovery depend on the current WiFi subnet. Switching `wlan0` from a
   normal client (e.g. `192.168.1.0/24`) to the hotspot (`10.42.0.0/24`)
   is a full driver-level mode change, not just an IP change -- nodes
   that were already running (or even freshly started, in one case)
   would keep publishing to their own topics individually but the web
   streamer's subscriptions would go silent, which looks exactly like
   "the page loads but shows nothing." Every node in this pipeline runs
   on one machine, so there's no reason discovery should touch any
   physical interface at all -- both launch files force
   `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, and the hotspot scripts
   `export` it before `ros2 launch` runs (setting it via the launch
   file's own `SetEnvironmentVariable` action applied inconsistently
   across nested `IncludeLaunchDescription`s when tested -- exporting in
   the shell first is the reliable fix).
2. **The viewer page loaded Three.js from a CDN.** A device connected to
   the standalone hotspot has no internet route at all, so it can never
   reach `cdn.jsdelivr.net`. `THREE` stays undefined and the page's
   entire script throws on its first line (`new THREE.Scene()`) --
   before the fetch/poll loop that would show an error (or anything)
   ever runs. Totally silent, blank screen, nothing in the served HTML
   to suggest why. Fixed by bundling `three.min.js` and
   `OrbitControls.js` in `landfill_camera_sim/scripts/web_assets/` and
   serving them from the same local HTTP server -- the page now has zero
   internet dependency, which is the actual requirement for a standalone
   deployment.

Also worth knowing: the client's polling `fetch()` originally had no
timeout. A single hung request (plausible on a NAT'd connection -- a
connection-tracking timeout silently dropping a TCP session with no
FIN/RST) would block forever, and since `setTimeout(poll, ...)` only
runs after the fetch settles, the *entire* polling loop would die
silently and permanently on the first hang. Fixed with an
`AbortController` + 4s timeout so any single bad request costs one
missed refresh instead of ending every future one, and the server was
moved back from HTTP/1.1 keep-alive to HTTP/1.0 (fresh connection per
request) since a persistent connection is exactly the kind of thing a
NAT'd hotspot link can silently kill without either side noticing.

## Livox Avia networking

The static IP needs to be on whichever NIC the Avia's cable is actually
in. On this rig that turned out to matter twice over: once because the
cable was in a second, add-on PCIe Ethernet card (`enp1s0`, chip:
Realtek RTL8168h) instead of the Pi's onboard port (`eth0`), and later
because that add-on card's port itself turned out to be flaky -- it held
electrical link (`LOWER_UP`) but delivered zero clean packets with a
climbing NIC-level `missed` counter, the signature of a marginal PHY
rather than a cabling or software issue. Both were root-caused by
sampling `ip -s link show <iface>` twice a few seconds apart: an
interface with a real, healthy connection shows RX bytes/packets
climbing steadily and `missed` flat at zero; if bytes/packets are frozen
while `missed` still ticks up, something's arriving corrupted at the
physical layer specifically on that port. Verified over a 60s soak
(600+ messages, zero packet loss) once moved to the Pi's onboard `eth0`.

Reassigning the static address between NICs, if you ever need to:

```bash
nmcli connection modify eth0 ipv4.method manual ipv4.addresses 192.168.1.5/24
nmcli connection up eth0
nmcli connection down "Wired connection 1"   # whichever profile now has the wrong interface
nmcli connection modify "Wired connection 1" connection.autoconnect no
```

(No `sudo` needed -- polkit allows the logged-in user to manage
NetworkManager connections directly.)

## Image looked "pixely"/blobby despite good point density -- fixed

Not a sensor resolution limitation -- the render point *size* was
roughly 3-4x larger than the data actually needed. Measured directly:
fetched the live served cloud and computed real nearest-neighbor point
spacing (`scipy.spatial.cKDTree`) rather than guessing -- merged mode
~1.4cm median spacing, live mode ~2.1cm (sparser; no ICP densification).
The previous render size (0.05 world units = 5cm for merged) was ~3.5x
the actual point spacing, so adjacent points overlapped heavily enough
to merge into solid blobs, hiding real structure the data already had.
Confirmed visually, not just by the numbers: re-rendering the identical
fetched cloud at progressively smaller sizes went from "mostly solid
black blobs, no visible structure" to individual points and real
geometric edges being distinguishable, with zero change to the
underlying data. Fixed by sizing each mode to ~1.3-1.5x its own measured
spacing (the standard point-cloud-rendering target -- enough overlap to
read as a continuous surface, not enough to blob): merged 0.05 -> 0.018,
live 0.06 -> 0.028. If you change sensor resolution, lidar density,
`max_points_*`, or `window_duration` later, re-measure spacing rather
than assuming these sizes still apply -- they're calibrated to this
specific scene's density, not a universal constant.

## Performance

Measured end to end on the Pi: reducing every OpenCV-using node to
`cv2.setNumThreads(1)`, capping `OMP_NUM_THREADS`/`OPENBLAS_NUM_THREADS`/
etc. to 1 pipeline-wide, disabling the thermal preview topic by default,
and halving `registration_merge_node`'s `window_duration` (1.0s -> 0.5s,
doubling the merged cloud's refresh rate and roughly halving its jitter
by giving each ICP window fewer scans to register) took the pipeline's
total CPU from ~228% to ~159% (of 400% available on this 4-core board)
while every measured rate held steady or improved. The single biggest
factor: OpenCV defaults to a core-count-sized internal thread pool
*per process*; with several OpenCV-using nodes running independently,
that's severe oversubscription that shows up as high CPU with
*declining* throughput under load, not just "slow."

The web viewer's point-count caps (`max_points_merged`/`max_points_live`
in `fusion_web_streamer`) are a different kind of cost -- bandwidth and a
cheap numpy gather, not extra ICP compute -- so they can be tuned up for
a denser/better-looking view without touching the Pi's tighter compute
budget.

A second pass took total CPU from ~159% to ~103%: every publisher and
subscriber pipeline-wide was switched from ROS2's default `RELIABLE` QoS
to `BEST_EFFORT` (`rclpy.qos.qos_profile_sensor_data`) -- correct for a
live sensor stream where a dropped frame is superseded almost
immediately, and it removes DDS's ACK/retransmission bookkeeping
entirely. **QoS mismatches fail silently** (a stricter subscriber request
than what's offered just never connects, with no error) -- every
publisher/subscriber pair on every topic was changed together and
verified afterward with `ros2 topic info <topic> --verbose` to confirm
matching reliability and the expected publisher/subscriber counts on
both ends. `calibration_publisher`'s republish rate was also cut from
30Hz to 2Hz -- it's static data loaded once at startup, so redoing that
15x more often than the data could possibly change was pure waste.

If you touch QoS again: verify with `ros2 topic info --verbose`, not
just "does data still arrive" -- a slow/naive test subscriber can itself
under-count on `BEST_EFFORT` + shallow queues (older unconsumed messages
get dropped instead of buffered) and look like a regression that isn't
really there. Trust the producing node's own logged rate
(`registration_merge_node` logs its publish rate on every message) or
the actual served endpoint over an ad hoc subscriber script.

**Balancing quality against that CPU budget**: with the headroom the QoS
pass freed up (~103% of 400% available, load average settling around
2.5-3 on the 4-core board), `max_points_merged` was raised 28000 -> 35000
for a denser/crisper merged view -- safe to tune because, as above, it's
bandwidth and a numpy gather in `fusion_web_streamer`, not extra ICP
compute. `window_duration` (0.5s) was left alone rather than pushed
further for speed: it already lands ~1.3-1.8Hz with 5-7 scans per merge,
which reads as the right trade-off between "updates often enough" and
"dense enough to look like something" -- there wasn't clear evidence a
tighter window would improve the balance rather than just trade one for
the other.

## Calibration

`landfill_camera_sim/config/calibration/intrinsics.yaml` and
`extrinsics.yaml` were updated for the real sensors (thermal resolution
640x512, both cameras' fx/fy set from assumed FOVs) but **are
placeholders, not a measured calibration** -- see the comments in those
files for how to get real numbers (checkerboard calibration for
intrinsics, tape-measure offsets for extrinsics, since the three sensors
are physically separate devices here, unlike the co-located simulated
rig).

## Still open (needs your input, not a software fix)

- **Calibration is placeholder-only** -- see above.
- **Thermal is relative, not radiometric.** No vendor SDK access to this
  module's raw14/TLinear mode -- see `thermal_camera_node.py`'s docstring.
- **The Avia's near-range blind zone (~0.5-1m) is optical**, not a
  config parameter -- its transmit/receive optics are physically offset
  (parallax), so very close returns don't overlap and get lost
  regardless of software.
