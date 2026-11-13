# PitFusion3D: Day-2 results (2026-11-13)

Follow-up to the 2026-11-12 campaign (`../final_campaign_20261112/FULL_RESULTS_REPORT.md`). Figures: `figures/D01..D10` (PNG + PDF, captions in `figures/CAPTIONS_DAY2.md`). Raw data: `data/`. Code: the scripts in this folder (all re-runnable).

## 1. LiDAR standby on power-down, wake on power-up
| Case | Result | Evidence |
|---|---|---|
| Pi shut down / rebooted from the OS | LiDAR put in **Standby** (confirmed by the sensor in 0.9 s) after the pipeline releases it | `lidar-power.service` stop action; `~/lidar_ctl/power_events.log` (`shutdown_sleep`: Normal -> Standby, 0.9 s) |
| Pi powered up | Service runs **before** the pipeline: Standby -> **Normal** in 0.9 s (whole step 4.8 s), then the pipeline connects | `boot_state_seen` / `boot_wake_to_normal` entries |
| Pi loses power suddenly | **Cannot be handled in software.** Data stops after 3.2 s; the LiDAR re-broadcasts every 1 s (238 packets seen) and stays in Normal mode | `data/lidar_sudden_loss.json` (8672 data packets, last at 3.206 s) |

Installed: `/etc/systemd/system/lidar-power.service` (enabled), hook `~/lidar_ctl/lidar_power_hook.sh`, copies in the repo `tools/lidar_ctl/`. Ordering: `Before=user@1000.service` (starts before the pipeline, stops after it) and `After=NetworkManager.service` (stops before the network goes down). The start/stop actions were tested through systemd. A full real power-off/boot cycle has NOT yet been run (it ends the working session); to verify, power off or reboot the Pi, then check `~/lidar_ctl/power_events.log` for `shutdown_sleep` (Normal -> Standby) and, after boot, `boot_state_seen` (Standby) followed by `boot_wake_to_normal` (Standby -> Normal).
**For sudden power loss** the only fix is hardware: power the LiDAR through a switch (relay or MOSFET) driven from the Pi's 5 V rail, so it turns off when the Pi's supply disappears. Figure D01.

## 2. Thermal camera
- Heat is **not** the explanation (D02, `data/heat_analysis.json`): 12 stuck episodes began at SoC 37-71 C; the SoC averaged 61.5 C while streaming fine and 64.4 C while stuck (ranges overlap).
- Heat endurance test (section 8): the camera state was stuck before and after, so it did not change with 25 min at 84 C either.
- Recovery attempts: USB port power-off/on 0/8 (`cam_cycle_loop1.txt`), watchdog ladder 0/3, camera self-reset plus 2 min hands-off 0/1 (`cam_patience_run1.txt`). Only complete power loss has cleared it, and not every time (2 of 4).
- On the Pi 5 the four USB ports share one power group, and the `disable` switch does not remove VBUS.
- **Status now: still stuck (no thermal frames) since the cold start this morning.** It will need a real power loss to the camera.
- Options: (a) unplug/replug the camera or power-cycle the Pi; (b) a switchable USB hub or a relay in the camera's USB power line, driven by the existing watchdog; (c) replace the camera/cable (a hardware fault is possible and the pattern, "stays stuck until power is removed", fits a latch-up in the camera).
The watchdog service stays enabled and will recover the camera automatically after (a)/(b).

## 3. Registration (ICP) accuracy against known ground truth (D03)
A real merged cloud (115 375 points) was moved by known amounts, with 16 mm LiDAR noise added, and the production ICP had to recover the motion (25 trials each).
- 100 % success up to 5 cm / 0.5 deg between scans (the rig moves at most about 3.5 cm between scans at 0.35 m/s); 84 % at 10 cm, 56 % at 20 cm, 40 % at 30 cm, 8 % at 50 cm.
- Residual error with the previous 0.2 m voxel: 24-36 mm at small motion. Insensitive to sensor noise 5-60 mm (34 -> 38 mm).
- Parameter study (24 combinations, `data/icp_param_study.json`): voxel 0.1 m gives about **18 mm instead of 32 mm** for about 11 ms more per scan. **Adopted** (`icp_voxel_size` 0.1). Voxel 0.05 m gives about 9 mm but costs 70-145 ms per scan (not adopted: would load the CPU).
- Live A/B on the running pipeline (`data/ab_icp_voxel_A.json` / `_B.json`): merge-node CPU 38.1 -> 43.0 %; fused 10.0 Hz and merged 1.5 Hz unchanged; merged age p50 271 -> 294 ms.

## 4. RGB-LiDAR calibration check (D04): inconclusive
The edge-correlation check gave 3.5 deg, 3.5 deg, 2.5 deg and -2 deg yaw in four runs. The scene (sheer curtains, changing daylight) breaks the method, so the previous calibration was **left unchanged**; a paired test on identical frames (0.045 vs 0.065) did not show a benefit of the estimated correction. Optional boresight-correction support is in `sensor_fusion_node.py` but disabled (no `boresight_correction_deg` in `extrinsics.yaml`). To quantify alignment properly, use a checkerboard or an edge-rich static scene.

## 5. LiDAR stability over 25 minutes (D05)
Fixed 6x6 deg window and dominant plane every 30 s (49 of 50 samples kept): spread 25 mm (robust std), plane-fit noise **16.7 -> 16.6 mm** (no change as it warmed), range 4.635 -> 4.600 m (-96 mm/h, CI -151 to -34), which is consistent with warm-up drift but a moving scene cannot be excluded.

## 6. Planning numbers (D06, `data/stakeholder_numbers.json`)
SD card 70 GB free, 53 MB/s write. Per hour: LiDAR 15.6 GB, RGB raw 99.5 GB, thermal raw 70.8 GB, fused cloud 20.7 GB, merged cloud 16.0 GB, merged in viewer format (11 B/pt + gzip) 4.9 GB. Recording all three raw sensors (52 MB/s) is at the card's write limit and fills the free space in 23 min. Power (Pi + sensors) 4.1 W typical, 7.3 W conservative; LiDAR adds the manufacturer's typical 9 W (not measured). Runtime (conservative): 50 / 100 / 150 / 300 Wh give 7 / 14 / 20 / 41 h with the LiDAR asleep and 3 / 6 / 9 / 18 h scanning.

## 7. Data freshness (D07)
Measured with the streamer's `X-Cloud-Stamp` / `X-Server-Time` headers (new): live view data is **81 ms** old at the client (p95 137 ms); merged **467 ms** (p95 638 ms), and since a merged cloud covers a 0.5 s window its content averages about 0.77 s.

## 8. Sustained heat test (D08, `data/heat_endurance.csv`)
3 min baseline, then 4 busy cores for about 25 min **on top of the running pipeline** (a deliberately unrealistic load), then cool-down.
- Baseline (pipeline only): 66-70 C, fan 6.8k RPM, 2000 MHz, no throttling.
- Stress: SoC rose to 76 C in 5 min, **reached 80 C at about 13 min** (first firmware throttling flag at 802 s), settled at **81-83 C**, peak **84.2 C** where the test's own safety limit ended the stress phase 11 s early. Fan went to its maximum (about 8.8-9.4k RPM, PWM 250) within 20 s of the load.
- CPU clock: held at 2000 MHz until minute 15, then dipped to about 1850-1950 MHz (soft throttling, about 3-8 %); ending flag `0xe0000` records that throttling had occurred.
- Pipeline under that overload: fused cloud held 8.8-10 Hz; the **merged cloud fell to 0.6-0.7 Hz** at first and recovered to 1.2-1.3 Hz; no crashes.
- Current stayed under 1.8 A (conservative 2.6 A), supply >= 4.85 V, no USB drops.
- Cool-down: back to 65 C within 8 minutes; fan relaxed to 5.3k RPM.
Meaning: normal operation has about **10 C of headroom** to where throttling starts in this (cool) room. In a room 10 C warmer, or in direct sun, the pipeline alone would approach the throttling range; a heatsink/shade is cheap insurance. Heat does not explain the thermal-camera fault (section 2). This room was not measured for ambient temperature.

## 9. Cold boot (D09)
Data flowing 25 s after power-on; the desktop takes 63 s and is not waited for. The LiDAR wake step runs before the pipeline and adds about 5 s from Standby (about 15 s from Power-Saving).

## 10. Scorecard (D10) and open items
Green: real-time performance, power, robustness, boot, registration accuracy. Amber: LiDAR power management (sudden loss needs hardware), colour/thermal alignment (needs a target), SD recording. Red: thermal camera stuck state, WiFi in the rig room.
Open: real shutdown/boot cycle of `lidar-power.service`, calibration target, camera power switch, LiDAR power switch, ambient temperature logging.

## 11. Thermal camera follow-up (2026-11-13, midday)
Evidence (`data/cam_rest_test.txt`, kernel journal):
- The camera enumerates normally and stays connected indefinitely (100 s and 20 min tested) while nothing opens it.
- Any stream request, even plain `v4l2-ctl` with no ROS, yields **0 frames** and the camera resets itself **about 10 s later** (the UVC endpoint is bulk; the kernel logs -71 errors). With the pipeline running, its reopen loop makes this repeat every ~34 s (227 disconnects between 09:30 and 11:45).
- Resting the camera untouched for 20 minutes did not clear it. A restart of the host-side stack or USB port/authorisation/usbreset does not either.
- Pi 5 USB power is ganged: VBUS drops only if all onboard hubs' ports are off together (uhubctl README, Raspberry Pi 5 section); my earlier cycles disabled one port, so they never cut power to the camera. This explains why only real power loss ever helped.
Implemented: the watchdog (`~/thermal_fix/thermal_watchdog.py`) has a new first step, `vbus_cycle`: disable all onboard USB ports for 30 s, re-enable, then check the camera stays up for 90 s. It needs a one-time permission step: `sudo bash ~/thermal_fix/install_ganged_power.sh` (not yet run). Manual test afterwards: `python3 ~/thermal_fix/thermal_watchdog.py --vbus-test` (briefly drops USB keyboard/mouse).
If vbus_cycle does not restore streaming, the camera itself (or its cable) is faulty and should be replaced.

### 11b. Result of the ganged power cut (12:38-12:47)
Permission step done by the user. `vbus_cycle` works at USB level (keyboard, mouse and camera all disconnect and re-enumerate). A 30 s cut and a 300 s cut (`thermal_fix/vbus_long_test.py`) both left the camera failing again 13-14 s after streaming resumed, and a manual unplug/replug at about 11:26-11:31 also failed. **Conclusion: the camera (or its cable/connector) is probably faulty now.** To confirm: try a different USB cable and port, and ideally a second camera. The watchdog now limits the power cut to once per 6 h unless it ever works. (Also seen: LiDAR Ethernet link down 11:26-12:4x while cables were being moved; restored.)
