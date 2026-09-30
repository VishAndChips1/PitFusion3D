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
