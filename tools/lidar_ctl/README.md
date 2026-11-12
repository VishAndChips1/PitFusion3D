# LiDAR power-mode control (Livox Avia) from the Raspberry Pi

**Short answer:** yes. The Livox Avia has three work modes — **Normal**, **Power-Saving** and **Standby** — that the host sets over the same Ethernet link that carries the point data. This folder contains a small tool built on the vendor's own Livox SDK that lets the Pi switch the sensor between them, with the sensor itself confirming each change. It was built and tested on 2026-11-12 against the real unit (broadcast code `3JEDLAJ001G0651`, firmware 11.8.0.6, IP 192.168.1.165).

## Use it
```bash
~/lidar status     # ask the LiDAR what state it is in
~/lidar low        # Power-Saving mode  (scanning stops; lowest-power state the sensor offers)
~/lidar standby    # Standby mode       (fast wake-up)
~/lidar on         # back to Normal mode (the pipeline's ROS driver resumes streaming by itself)
# options:  --no-restart  leave the ROS pipeline stopped afterwards     --no-stop  do not touch the pipeline
```
Each command prints one JSON line, e.g. `{"ok":true,"command":"low","initial_state":"Normal","final_state":"PowerSaving","switch_seconds":0.9,...}`; `ok:true` means **the LiDAR itself reported the new state**, not merely that the command was sent. A log of every call (including the SDK's own output) is `~/lidar_ctl/lidar.log`.

## What the modes are (Livox SDK protocol, LiDAR command set 0x01 / cmd 0x00 "Set Mode")
| Mode | Value | Meaning | Measured switch time |
|---|---|---|---|
| Normal | 0x01 | scanning and streaming points | from Power-Saving **9.7 s** (the sensor has to spin up), from Standby **0.9 s** |
| Power-Saving | 0x02 | scanning stopped; sensor stays on the network and answers heartbeats | Normal → Power-Saving **0.9 s** |
| Standby | 0x03 | scanning stopped, kept ready for a quick restart | Normal → Standby **0.9 s** |

The state is reported in the 1 Hz heartbeat (general command 0x03, `work_state` field). The Livox documentation does not publish per-mode wattage, and the sensor's own supply could not be metered from the Pi, so **power saved was not measured** — only the state change, the stop of point data and the timings above. Power-Saving is the deepest state the protocol offers; the sensor remains powered (fan/electronics) and cannot be fully switched off over Ethernet — that needs a power switch on its 12 V supply.

## How it works
`lidar_ctl` (C++, `lidar_ctl.cpp`, linked against `liblivox_sdk_static.a`) does what the ROS driver does up to the point of streaming: it initialises the SDK, hears the LiDAR's broadcast, connects (handshake), waits for a heartbeat that reports a valid state, calls `LidarSetMode(handle, mode)`, then waits until the *device state callback* shows the target state (the SDK notes that a successful command callback only means the switch has *started*). Exit code 0 = confirmed, 2 = timeout, 3 = LiDAR not found, 4 = SDK would not start.

**Why the pipeline is stopped first:** the LiDAR serves one host at a time, so the pipeline's `livox_ros2_avia_node` must release it. The `lidar` wrapper stops `pitfusion3d.service` **gracefully** (SIGTERM; nodes close their devices cleanly — important for the thermal camera), runs the tool, and restarts the service unless `--no-restart` is given. A mode change therefore costs about 25–40 s of pipeline downtime, plus 10 s of wake-up when turning on.

## Verified behaviour (all on the real unit, 2026-11-12 21:38–21:43)
| Test | Result |
|---|---|
| `status` | Normal, firmware 11.8.0.6 |
| Normal → Power-Saving | confirmed by the sensor in 0.9 s |
| State persists after the tool disconnects (no heartbeats) | still Power-Saving after >15 s, and after a ≈45 s run of the ROS pipeline |
| ROS pipeline started while the LiDAR is in Power-Saving | driver connects, **no points, no errors, no tracebacks**; it does *not* wake the sensor (it only starts sampling when the sensor reports Normal) |
| Power-Saving → Normal | confirmed in 9.7 s; after restarting the pipeline `/livox/lidar` 9.8 Hz and fused cloud flowing |
| Normal → Standby → Normal | 0.9 s each way |
| Repeat Power-Saving → Normal | 9.7 s again (repeatable) |
| Final state left for the night | **Power-Saving** |

## Automatic Standby at shutdown and wake at boot (installed 2026-11-13)
**What you asked for:** when the Pi is powered down the LiDAR goes into Standby; when the Pi gets power again the LiDAR comes out of Standby into normal operation.

| Situation | What happens | Status |
|---|---|---|
| Pi shut down or rebooted from the OS (`poweroff`, `reboot`, desktop menu, `shutdown`) | `lidar-power.service` stops **after** the ROS pipeline has released the sensor and **before** the network goes down, and puts the LiDAR into **Standby** | installed, tested through systemd; **real power-down ordering verified at the reboot test (see `power_events.log`)** |
| Pi powered up | the same service starts **before** the pipeline's user session, reads the LiDAR's state (logging whether the previous shutdown left it in Standby) and wakes it to **Normal**; the ROS driver then connects to an already-awake sensor and streams | installed, tested through systemd |
| **Pi loses power suddenly** (battery flat, cable pulled) | **cannot be handled by software**: nothing is left running to send the command. Measured on the real unit: after the host disappears the LiDAR **stops sending point data within 3.2 s**, starts re-broadcasting once a second to find a new host, and **stays in Normal mode** (it does *not* enter Standby by itself). It is awake and idle, not scanning | measured 2026-11-13 |

**For sudden power loss the only reliable fix is hardware:** switch the LiDAR's own supply from the Pi's 5 V, for example a small relay/MOSFET module whose coil is driven from a Pi 5 V pin or USB port, so the LiDAR is unpowered whenever the Pi is. (Or accept the idle state: with no host the sensor is not scanning.)

**Files and controls**
* `lidar_power_hook.sh start|stop` — the action run by the service; every action is appended as one JSON line to `~/lidar_ctl/power_events.log` (state seen, state after, seconds taken).
* `/etc/systemd/system/lidar-power.service` — `Type=oneshot`, `RemainAfterExit=yes`, `User=vish`, `After=NetworkManager.service`, `Before=user@1000.service`, `TimeoutStartSec=75`, `TimeoutStopSec=60`; enabled for multi-user.target.
* Choose the sleep mode: `LIDAR_SLEEP_MODE=standby` (default, 0.9 s wake) or `low` (Power-Saving, 9.7 s wake) — set it with `sudo systemctl edit lidar-power.service` (`[Service] Environment=LIDAR_SLEEP_MODE=low`).
* Disable it: `sudo systemctl disable --now lidar-power.service`.
* Boot cost: the wake step runs before the pipeline starts, so first data appears about 5 s later (Standby) or about 15 s later (Power-Saving) than the 25 s measured without it.
* The manual `~/lidar` commands still work; they stop the pipeline, change the mode and restart it.

**Honest limits.** (1) Standby vs Power-Saving power draw was not measured (the sensor's supply is not visible to the Pi); Power-Saving is nominally the deeper state, Standby the faster to wake. (2) If the LiDAR itself lost power overnight it restarts in Normal; the boot log line `boot_state_seen` shows which case occurred.

## Behaviour to know about
* After a Pi reboot the pipeline auto-starts, its driver connects to the sleeping LiDAR and simply publishes nothing; run `~/lidar on` to start scanning again. The fused/merged outputs stay silent until then (the viewer shows its last cloud / empty).
* If the sensor loses power entirely it starts in Normal mode again (power-up default).
* `SDK Init failed` from the tool means the ROS driver still holds the connection — use the `lidar` wrapper, not the binary directly.
* "LiDAR not found": check the Ethernet link (`ip -br addr show eth0` should show 192.168.1.5), the sensor's power, and that the broadcast code in `LIDAR_CODE` matches the label (default `3JEDLAJ001G0651`).

## Rebuild
`g++ -std=c++14 -O2 lidar_ctl.cpp -I/usr/local/include -L/home/vish/Livox-SDK/build/sdk_core -llivox_sdk_static -lpthread -o lidar_ctl`

## Possible improvement
Doing this *inside* the ROS driver (a `/livox/set_mode` subscriber that calls `LidarSetMode` on the existing connection) would remove the need to stop the pipeline. It requires modifying and rebuilding the vendor driver, so it was not done.

## References
Livox SDK Communication Protocol (LiDAR command set 0x01, 0x00 *Set Mode*; general command 0x03 *Heartbeat*): https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol · Livox SDK `livox_sdk.h`: `LidarSetMode`, `LidarMode` (`kLidarModeNormal/PowerSaving/Standby`), `LidarState`.
