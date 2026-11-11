# 01. Shared-memory transport (Fast DDS profile)

## Problem (measured)
The RGB camera was captured at 15 Hz but only **3.61 Hz** reached the fusion node, so fusion output was 2.42 Hz and the four cores sat ~53% busy.
Not a CPU problem: the 1280x720 rgb8 frame is 2.7 MB, Fast DDS's built-in shared-memory segment is far smaller, so large messages fell back to UDP over loopback, where this Pi's
kernel socket buffer (`net.core.rmem_max` = 212,992 B, unraisable without root) silently dropped fragments. The existing `fastdds_large_buffers.xml` requested 4 MB buffers, but the kernel caps the request.
(An earlier test of `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` did not help either: RGB 7.00 Hz vs 6.44 Hz, [`evidence/earlier_campaign/B3_transport_rates_T0_vs_T2.csv`](evidence/earlier_campaign/B3_transport_rates_T0_vs_T2.csv).)

## Change
`src/pitfusion3d_bringup/config/fastdds_large_buffers.xml` (same file name, so the launch file is unchanged) now defines a **shared-memory transport with a 64 MB segment**, kept alongside UDP for discovery.
No root required. Original profile kept at [`evidence/transport/original_udp_only_profile.xml`](evidence/transport/original_udp_only_profile.xml).

## Before / after
Synthetic benchmark at the real sizes and rates ([`transport_bench_results.txt`](evidence/transport/transport_bench_results.txt)), received/sent:

| profile | RGB (15 Hz) | thermal (30 Hz) | cloud (10 Hz) |
|---|---|---|---|
| default Fast DDS | 3.40 | 18.40 | 6.24 |
| original UDP-only profile | 3.44 | 17.40 | 5.04 |
| **shared memory 64 MB** | **14.44** | **28.92** | **9.64** |

On the live pipeline: RGB 3.61 -> 15.05 Hz, fused 2.42 -> 9.76 Hz, merged 0.66 -> 1.49 Hz ([`baseline.json`](evidence/measurements/baseline.json) vs [`shm.json`](evidence/measurements/shm.json)).

## Segment size selected: 64 MB
128 MB, 64 MB and 32 MB all gave the same rates in the benchmark (RGB 14.0-14.4 Hz; thermal 28.9 / 28.9 / 26.6 Hz). Memory differs: 8 participants x 128 MB held **1.1 GB** of RAM, 64 MB holds ~0.54 GB.
64 MB keeps full rates with half the memory; 32 MB showed a small thermal-rate drop. (Files: `shm_large.xml`, `shm_32mb.xml`.)

## Costs and caveats
- The extra throughput made the system busier: whole-system CPU 198% -> 324% of 400 (85-79% per core) until docs 02-04 removed wasted work.
- **Anything run from a shell must load the same profile** (`export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/install/pitfusion3d_bringup/share/pitfusion3d_bringup/config/fastdds_large_buffers.xml`), otherwise `ros2 topic hz` talks over the lossy UDP path and reports low rates.
- Crashes leave the segments behind in `/dev/shm` (a leak this change introduced): fixed in [doc 06](06_crash_recovery_respawn_and_shm_cleanup.md).
