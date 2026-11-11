# 06. Crash recovery: node respawn, orphaned camera, shared-memory leak

## Problems found by testing (not by reading code)
1. **Killed nodes never came back.** `ros2 launch` does not respawn crashed nodes by default. `kill -9` of the fusion node left it dead for 45 s+ and the viewer silently served a stale cached payload ([before](evidence/corner_cases/crash_recovery_BEFORE_respawn.txt)).
2. **A killed RGB node would leave `rpicam-vid` running** holding the camera (a stale one was seen), so a respawned node could not open it.
3. **Hard crashes leaked shared memory.** Introduced by the 64 MB segments of [doc 01](01_shared_memory_transport.md): Fast DDS only deletes segments on a clean exit, so each crash left 8 x 64 MB behind. After the crash/restart tests `/dev/shm` held **1,363 MB, of which 821 MB were stale** ([`shm_leak_before_after.txt`](evidence/corner_cases/shm_leak_before_after.txt)): repeated crashes would exhaust RAM on a board that has OOM'd before.

## Changes
- `respawn=True, respawn_delay=2.0` on all 8 nodes in `real_sensors.launch.py`, `sensor_fusion.launch.py`, `web_stream.launch.py`.
- `rgb_camera_node.py`: `rpicam-vid` is started with `PR_SET_PDEATHSIG` (dies with its parent) and any stale `rpicam-vid` is cleared at start.
- `scripts/clean_stale_shm.py` removes this user's `fastrtps_*` shm files that no live process has mapped (never a file in use, never one < 3 s old); called from `run_pipeline.sh` before every launch.

## Before / after
| test | before | after |
|---|---|---|
| `kill -9` each of 8 nodes | not respawned (fusion: 45 s+, dead) | **8/8 respawned in 2-4 s** (restamp 2, calib 2, fusion 2, merge 2, streamer 2, thermal 4, rgb 2, LiDAR driver 2), 1 copy each, then measured rates 10.0/10.1/10.1/1.55 Hz, 1 `rpicam-vid` ([after](evidence/corner_cases/crash_recovery_AFTER_respawn.txt)) |
| SIGKILL every process of the service, x2 | (manual restart) | systemd auto-restart; merged data back after **26 s / 25 s**; 8/8 node types; 1 `rpicam-vid`; `/dev/shm` 542 -> 542 MB ([output](evidence/corner_cases/hard_kill_test_output.txt)) |
| restart / stop-start / repeat | - | live data after 17 / 19 / 15 s, merged 17 / 20 / 15 s, 8/8 nodes ([output](evidence/corner_cases/service_restart_cases.txt)) |
| stale shm after crashes | 821 MB leaked | 0, removed 861 MB live, then flat across 2 more crashes |

## Caveats
- The first "whole-service crash" in `service_restart_cases.txt` (case 1) did **not actually kill anything** (`systemctl kill` rejected the signal); it is replaced by the cgroup-level hard-kill test above. Kept for transparency.
- A loose `pgrep` in the hard-kill output reports "nodes 10" because it also counts the checking shell; the exact count appended at the end of that file shows 8/8 types (the second thermal copy there was a leftover of my own test process, since removed).
- Not tested: a real power loss or reboot (no root to reboot), LiDAR physically unplugged at runtime.
