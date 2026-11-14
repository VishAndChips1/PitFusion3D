# PitFusion3D: how the code works (second level of detail)

Written for someone who doesn't code. Every claim below comes from the code in `~/ros2_ws/src`.

---

## 1. The goal in one paragraph

A Raspberry Pi 5 carries three sensors. The code combines them into a live 3D point cloud, where every point has a position, a colour and (when the thermal camera works) a temperature. You watch it in a browser.

| Sensor | Device | Rate | What one reading is |
|---|---|---|---|
| LiDAR | Livox Avia, over Ethernet | **10 Hz** (set in the launch file, `publish_freq: 10.0`) | A "scan" of 24,000 points: x, y, z and intensity |
| RGB camera | Pi HQ camera (IMX477) | **10 fps** (launch file; the node's own default is 15) | A 1280×720 colour image |
| Thermal camera | USB camera, 640×512 | **30 fps requested** (cameras of this type only offer 30 or 60) | A heat image. **Currently not delivering frames** |
| LiDAR motion sensor (IMU) | Inside the Avia | ~200 Hz | Recorded in bags; not used by the pipeline |

The rates are deliberately matched: the camera runs at the LiDAR's 10 Hz because extra frames would never be used.

---

## 2. The framework: ROS 2

The code is built on **ROS 2 (Jazzy)**, a free robotics framework. It gives you:
- **Nodes:** small independent programs, each with one job.
- **Topics:** named channels between them (e.g. `/landfill/rgb/image_raw`). A node *publishes* to a topic and others *subscribe*.
- **Launch files:** a recipe that starts all the nodes together.

The messaging underneath is **Fast DDS**. All the sensor streams use "best effort" delivery: if a frame is lost it is not re-sent, because a newer one is always about to arrive. The nodes run on the same Pi, so the big messages (a 2.7 MB image) pass through shared memory rather than the network.

---

## 3. The pipeline, step by step

```
LiDAR ─► driver ─► [1 restamp] ─┐
RGB cam ─► rgb node ────────────┼─► [2 matcher/fusion] ─► coloured scans ─┬─► [3 stitcher] ─► merged cloud ─┐
Thermal ─► thermal node ────────┘      (10 per second)                   │      (~1.5 per second)           │
                                                                          └─────────────────────────────────┤
                                                                                  [4 web streamer] ─► browser
```

### Step 1: Putting the sensors on the same clock (`livox_restamp_node`)

**The problem.** Each coloured point needs a camera image from the *same instant* as the LiDAR scan. But the LiDAR stamps its scans with its own internal counter (seconds since it was switched on), while the cameras use the Pi's clock. The two numbers can't be compared.

**The fix.** For each scan, the node compares the sensor's stamp with the Pi's clock at the moment the scan arrived. That difference includes a small and variable network delay, so the node keeps the **smallest difference seen in the last 200 scans (~20 s)**. The smallest one contains the least delay, so it is the best estimate of the true clock offset. (This is the same trick used by network time protocols.) It then re-stamps every scan with the sensor's stamp plus that offset.

This keeps the LiDAR's own precise 100 ms spacing between scans. The node also resets itself if the Pi's clock suddenly jumps. The Pi has no battery clock, so it corrects its time shortly after every boot.

The RGB node stamps each frame with the Pi's clock at the moment the full frame is read. So after this step all sensors speak the same time.

### Step 2: Matching and colouring (`sensor_fusion_node`)

**Time matching.** This uses a standard ROS library, `message_filters`, and its `ApproximateTimeSynchronizer`. It holds the last few messages (queue of 5) from each stream. When it finds a LiDAR scan, an RGB image and a thermal image whose stamps are all within **0.15 s** of each other, it hands them over as a set.

A second synchroniser matches LiDAR + RGB only. If no three-way match has happened for 1 s (e.g. the thermal camera is dead), the node switches to LiDAR + RGB and leaves the temperature empty. It switches back automatically when thermal returns. This is why the pipeline still works with the broken thermal camera.

**Colouring the points.** For each scan, the node does this to all 24,000 points at once:

1. **Move each point into the camera's point of view.** The calibration file `extrinsics.yaml` says where the camera sits relative to the LiDAR (a shift and a rotation; measured 14 Nov: about 4 cm back and 5 cm down, tilted about 2°). One matrix multiply converts every point from "LiDAR coordinates" to "camera coordinates".
2. **Project it onto the image** with the pinhole camera model. A point `(x forward, y left, z up)` lands at pixel `u = fx · (−y / x) + cx`, `v = fy · (−z / x) + cy`. `fx`, `fy` (focal length in pixels) and `cx`, `cy` (the image centre) come from `intrinsics.yaml`. The lens bends the image (barrel distortion), so the node also applies two distortion numbers, `k1` and `k2`, before converting to pixels. For the RGB camera, `fx` was **measured on 14 Nov from your checkerboard bags: 1330.7 px**, with `k1 = -0.476`, `k2 = +0.223`. The earlier placeholder (1065 px, no distortion) was about 25 % off, which is why colours used to land in the wrong place.
3. **Discard points that can't be seen:** behind the camera, or landing outside the image.
4. **Copy the pixel's colour onto the point.** The node looks up the RGB value at that pixel and packs it into one number. For thermal it looks up the heat value and converts it to a temperature.

So **the match between a LiDAR point and a colour is purely geometric**: where the point would appear in the picture. It depends on the calibration numbers being right. That is why you are calibrating with the checkerboard: the board shows where the sensors really sit relative to each other.

The output is a point cloud with 6 values per point (x, y, z, intensity, colour, temperature), about 10 per second.

### Step 3: The stitcher (`registration_merge_node`)

**Why it exists.** One LiDAR scan is only 24,000 points, which is sparse. Over half a second you collect 5 scans. If the rig were perfectly still, you could just pile them on top of each other and get a denser picture. But the platform wobbles or moves a little, so the scans don't line up exactly.

**What it does.**
1. Collect scans for a **0.5 s window** (about 5 scans).
2. Take the first scan as the reference "map".
3. For each later scan, work out the small shift and turn needed to line it up with the map. This is called **ICP** (Iterative Closest Point): for every point in the new scan, find the nearest point in the map, work out the best rigid move that brings the pairs closer, apply it and repeat up to 10 times.
4. Add the aligned scan to the map.
5. Remove near-duplicate points: where scans overlap, only one point is kept per 1 cm cube.
6. Publish one denser cloud, about **1.5 times a second**.

**Speed tricks.** ICP is the heaviest part of the whole pipeline, so it runs on a separate background thread. The live feed never waits for it. ICP also uses only a thinned-out sample of the points (a 10 cm grid, capped at 1,500 source and 10,000 map points); the full-detail points are still moved and merged. If it falls behind, old windows are dropped so the view stays current.

**Why point counts vary.** The merged cloud's size depends on the scene. A flat wall seen by 5 overlapping scans merges heavily (many duplicates), while a varied scene keeps more points. A typical window has around 105,000 points. That's normal behaviour, not a fault.

### Step 4: Sending to your screen (`fusion_web_streamer`)

The Pi runs a small **web server on port 8080**. Any device on the same network opens `http://<pi-address>:8080` in a browser and sees the viewer. No software is installed on your laptop or phone.

To keep WiFi traffic and the Pi's power use low, it:
- **Caps the number of points:** up to 80,000 for the merged cloud (raised from 20,000 on 14 Nov) and 5,000 for the live one, picked at random. Each viewer asks for as many as its link can take, so a weak connection automatically gets fewer points instead of a blank screen.
- **Packs each point into 11 bytes:** position as three 16-bit numbers (the cloud is scaled to fit, with sub-millimetre error), colour as three bytes, temperature as one 16-bit number.
- **Compresses it with gzip**, and only encodes a new frame when a viewer asks for one.

The viewer page (`web_assets/viewer.html`) uses a 3D library called three.js. It repeatedly asks the Pi for new data, shows the point count and refresh rate, lets you switch between RGB and Thermal colouring and between Merged (dense, ~1.5 Hz) and Live (sparse, ~10 Hz), and lets you rotate, zoom and "Fit view". All its scripts come from the Pi itself, so it works with no internet connection (for example on the Pi's own hotspot).

> **About the receiver app with light and dark mode.** I couldn't find it. This Pi has only one Claude chat history (this one), and the only viewer in the code is the browser page above. That page is **dark only** and has no light mode. The receiver app you remember must live on another computer or in a different chat. If you tell me where it is, or paste its code, I'll describe how it connects. Nothing in the Pi's code needs to change for it, as long as it fetches the same data from port 8080: the 11-byte point format, with `/live.bin` and `/merged.bin` as the two data addresses.

---

## 4. What keeps it running

| Piece | Job |
|---|---|
| `pitfusion3d.service` | Starts the whole pipeline when the Pi boots and restarts it if it crashes. Before starting it wakes the LiDAR from Standby. |
| `lidar-power.service` + the `lidar` command | Puts the LiDAR into Standby at shutdown and back to Normal at boot (uses Livox's own control library). `lidar status / low / standby / on` |
| `thermal-watchdog.service` | Watches the thermal camera and tries to restart it if it stops. Currently unable to bring it back. |
| `clean_stale_shm.py` | Clears left-over shared memory after a crash |
| `calibration_publisher` | Broadcasts the calibration numbers to the other nodes |
| `run_checks.sh` | Style check plus 20 automatic tests (the maths, launch files) |

## 5. Calibration and bag recordings

- **Calibration** = finding the numbers that go into `extrinsics.yaml` and `intrinsics.yaml`: where each sensor sits, and how each lens bends the image. The checkerboard supplies a shape of known size (squares 58 mm) that every sensor can find.
- **Bag files** are replayable recordings (an `.mcap` file and a `metadata.yaml`), made with `~/checkerboard_bags/record_bags.py`. Each holds 5 s of all topics. The calibration project (`SensorCalibration`) reads them. Yours are on GitHub in `updated_bags/`.

## 6. Where things live

| Place | Contents |
|---|---|
| `~/ros2_ws/src/landfill_camera_sim/scripts/` | Fusion node, stitcher, web streamer, calibration publisher |
| `~/ros2_ws/src/pitfusion3d_bringup/` | Camera nodes, restamp node, launch files, start-up scripts |
| `~/ros2_ws/src/livox_ros2_avia/` | The LiDAR maker's own driver (not our code) |
| `~/lidar_ctl/`, `~/thermal_fix/` | LiDAR power control; thermal diagnosis and watchdog |
| `~/PitFusion3D_repo/` | Everything under version control, with test campaigns and docs |
| `~/thesis_tests/` | Test reports and figures |
| `~/SensorCalibration/` | Calibration project (GitHub) |

## 7. Honest status

- **Working:** LiDAR, RGB, time matching, colouring, stitching, web viewer, auto-start, LiDAR standby and wake, bag recording.
- **Not working:** the thermal camera. It connects, but any stream request gives zero frames and then it resets. No software fix worked. The test left is plugging it into another computer.
- **Done 14 Nov:** calibration from the checkerboard bags is applied. On poses the solver never saw, the overlap between LiDAR points and the real board rose from 55 % to 83 % (report: `~/checkerboard_analysis/results/CALIBRATION_REPORT.md`). Re-recording with the board tilted would tighten the rotation estimate.
- **Next:** review the sync/window sweep results and test raising the viewer's point caps over real WiFi.
