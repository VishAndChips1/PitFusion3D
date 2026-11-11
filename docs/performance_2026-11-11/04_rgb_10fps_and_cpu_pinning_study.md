# 04. RGB at 10 fps, and the CPU-pinning / GPU question

## RGB 15 -> 10 fps (`real_sensors.launch.py`)
The LiDAR publishes at 10 Hz, so RGB frames above 10 Hz are never fused. After the transport fix the RGB node was the third-biggest consumer.

| | rgb_node CPU | rpicam-vid CPU |
|---|---|---|
| 15 fps ([`shm64_worker`](evidence/measurements/shm64_worker.json)) | 28% | 8% |
| 10 fps ([`tuned_idle`](evidence/measurements/tuned_idle.json)) | 19% | 4% |

Fused output stays at the LiDAR rate (10.02 Hz).

## CPU pinning: tested, **not adopted**
Layout tried (applied live with `taskset`, [`apply_pinning.sh`](evidence/harness/apply_pinning.sh)): LiDAR chain -> core 0, cameras -> core 1, fusion + streamer -> core 2, merge -> core 3.
Interleaved with unpinned runs of the same build, 120 s each, 1 simulated viewer (LiDAR / fused / merged):

| run | result |
|---|---|
| unpinned `tuned_viewer2` | 8.3 / 8.30 / 1.27 Hz, merged 281 ms, pipeline 109% *(LiDAR dip)* |
| unpinned `none_viewer3` | 10.0 / 9.74 / 1.50 Hz, merged 219 ms, pipeline 122% |
| pinned `roles_viewer` | 5.7 / 4.87 / 0.14 Hz, merged 242 ms, pipeline 102% **(collapsed)** |
| pinned `roles_viewer2` | 10.0 / 9.81 / 1.50 Hz, merged 223 ms, pipeline 114% |
| pinned `roles_viewer3` | 10.0 / 9.78 / 1.50 Hz, merged 236 ms, pipeline 120% |

The clean pinned and unpinned runs are indistinguishable (the Linux scheduler already holds every core at 62-72%), one pinned run collapsed, and one unpinned run also dipped, so run-to-run noise is comparable to any effect.
Pinning also removes the scheduler's freedom to dodge desktop/IRQ load, i.e. makes the system *tighter*, which was explicitly not wanted. All-core affinity restored.

## GPU: not used
The Pi 5's VideoCore VII has no CUDA-style path for these numpy/scipy stages. The camera ISP already does YUV420 conversion/scaling in hardware (rpicam-vid at ~4% CPU), the Pi 5 has no hardware video encoder, and the 3D rendering runs on the viewing laptop's GPU.

## Caveat
The pinning conclusion rests on 3 runs per arm on a machine also running a desktop; it rules out a *large* benefit, not a small one.
