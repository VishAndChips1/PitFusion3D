# 05. Thermal camera faults and graceful degradation

## Problem (observed on the live system)
After this morning's boot the pipeline showed a loaded viewer page with **no data**. Cause chain, from the kernel and service logs:
1. The USB thermal camera (3474:43e2) kept dropping off the bus: `USB disconnect` / re-enumeration with error -71, **69 times between 08:24:47 and 12:51:08** on one boot (device number reached 72). Pi power was clean (EXT5V 5.04 V, `throttled=0x0`), so it is not a supply fault.
2. The thermal node reopened the camera immediately after each failure ("capture started" then 10 failed reads, every ~12 s) and logged a warning per frame (640 in the boot log).
3. `sensor_fusion_node` only published when lidar, RGB **and** thermal were all present, so a dead thermal camera silenced the whole viewer. That is the "restart a few times until it works" behaviour.

## Changes
**`thermal_camera_node.py`:** waits until the device node has existed continuously for 3 s (enumeration finished) before opening; after each failed streak cools down 3 -> 6 -> 12 -> 20 s (reset after 30 good frames) instead of reopening instantly; logs the 1st and 10th failure instead of every one; warns every 30 s while the device is absent.

**`sensor_fusion_node.py`:** a second synchroniser (lidar + RGB only) publishes fused clouds with `thermal = NaN` (the viewer already draws NaN thermal as grey) whenever **3-way fusion has not succeeded for `thermal_timeout_s` (default 1.0 s)**; full fusion resumes automatically when it works again. Keyed on "fusion not succeeding", not "no thermal frame arrived", so thermal frames that arrive but can't be time-matched (clock skew/step) are also covered. Duplicate scans are suppressed; sync queues are cleared on a clock rewind.

## Evidence
Live, with the camera still faulty: viewer served 5,000 live / 20,000 merged real points ([doc 07](07_corner_case_report.md)); all performance runs in [RESULTS.md](RESULTS.md) were made in this degraded mode.
Isolated-domain fusion suite with synthetic sensors ([`fusion_suite_AFTER.txt`](evidence/corner_cases/fusion_suite_AFTER.txt)): thermal gone, **thermal arriving but unsyncable (stamps 30 s off)**, thermal flapping, RGB missing, bad LiDAR, clock rewind/forward all keep the node alive and publishing (13/14 phases; the one miss is below).

**Timeout selection** ([`fusion_timeout_selection.txt`](evidence/corner_cases/fusion_timeout_selection.txt)): realistic dropouts (6 s up / 3 s down, 27 s, 270 scans ideal):

| fallback timeout | scans published | |
|---|---|---|
| 3.0 s (first version) | 180 (67%) | viewer stalls ~3 s every dropout |
| **1.0 s (selected)** | **241 (89%)** | |
| 0.5 s | 255 (94%) | more output, but a shorter timer risks flickering thermal on ordinary CPU hiccups; that false-switch rate was **not measured**, so 1.0 s was kept |

## Not solved / not tested
- **The hardware fault is unresolved**: kernel USB resets continued at ~2 per minute after these changes. Recommended: fully power off (unplug the 5 A supply ~10 s; a warm reboot likely leaves the camera's firmware in a stuck state, a hypothesis not confirmed), reseat the USB cable, try the other port, then replace the cable. No root, so no USB reset or autosuspend change from software.
- Sub-second flapping (1 s on / 1 s off) still halves output to ~5 Hz (103 of ~200 scans), never blank. Faster than any fallback timer can track; not realistic for this fault.
- Real thermal frames resuming after recovery were only tested synthetically; the camera never came back during the session.
