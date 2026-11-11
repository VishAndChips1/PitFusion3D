# Point-cap sweep (2026-11-11): what happens as the cap on points sent to the viewer is raised

Raw data: `results_merged*.jsonl`, `results_live.jsonl`, 31 Hz power traces `power_raw_*.csv`, per-step rate traces `_rates_*.csv`, logs `sweep_*.log`.
Tables: `table_merged.md`, `table_live.md`. Plot: `pointcap_sweep.png`. Code: `sweep.py`, `grab_full.py`, `analyze.py`.

## Method
- The cap (`max_points_merged` / `max_points_live` on `/fusion_web_streamer`) was changed at runtime, 120 s per step, with the full pipeline running on the real sensors.
- Merged view: 20k to 105k (12 steps). Live view: 5k to 25k (8 steps).
- Measured per step: points per cloud, gzip wire KB per cloud, clouds per second, Mbit/s to one viewer, gap and response latency, CPU (pipeline, streamer, system), upstream topic rates (fused / merged), temperature, and fidelity against a contemporaneous full cloud.
- Fidelity metrics: median nearest-neighbour spacing, and the percentage of the full cloud's 2 cm and 5 cm occupied voxels that the capped cloud covers.
- Current: `vcgencmd pmic_read_adc` at about 31 Hz. Estimated input current = sum of rail V*I / EXT5V; the conservative figure divides by 0.85 and adds 0.5 A for USB.
- Safety gates were active the whole time: abort if estimated input current > 4.5 A, EXT5V < 4.8 V, SoC > 78 C, or VDD_CORE well above its baseline. **No gate tripped in any step.**

## Headline results
| Quantity | Result across all steps |
|---|---|
| Estimated input current peak | 1.60 to 2.05 A (conservative 2.4 to 2.9 A). Limit is 5 A, so there is at least about 2 A of margin. **No trend with point cap.** |
| EXT5V minimum | 4.91 to 4.93 V |
| SoC temperature max | 70.5 to 76.0 C (rises slowly with cap) |
| Throttling | 0x0 throughout |
| Fused rate | about 10 Hz at every cap |
| Merged update rate | 1.5 to 1.65 per second at every cap. It is set by the ICP merge, **not by the cap**. |

### Merged view
| Cap | Wire KB/cloud | Mbit/s | Streamer CPU % | Pipeline CPU % | 5 cm coverage % | NN spacing cm |
|---|---|---|---|---|---|---|
| 20,000 (current) | 172 | 2.3 | 12 | 117 | 53 | 1.83 |
| 40,000 | 342 | 4.6 | 17 | 118 | 65 | 1.51 |
| 60,000 | 512 | 6.5 | 23 | 124 | 73 | 1.32 |
| 80,000 | 682 | 8.8 | 30 | 132 | 78 | 1.22 |
| 90,000 | 768 | 9.8 | 30 | 128 | 81 | 1.15 |
| 105,000 | 892 | 11.4 | 34 | 133 | n/a (see caveats) | n/a |

### Live view
Bandwidth scales roughly linearly (1.7 Mbit/s at 5k up to 10.9 Mbit/s at 25k). Clouds per second rises from 6 to 8.2, and response p95 rises from 20 ms to about 70 ms. Streamer CPU goes from 11% to 31%.

## Levels found
1. **Up to about 40k:** cheap. Under 5 Mbit/s and streamer CPU under 17%; 5 cm coverage rises from 53% to 65%.
2. **40k to 90k:** the quality gain continues almost linearly (coverage 65% to 81%, spacing 1.5 to 1.15 cm), but bandwidth doubles (4.6 to 9.8 Mbit/s) and streamer CPU roughly doubles (17% to 30%). Response p95 rises from 105 ms to 200 to 415 ms (the 80k figure is a single high outlier step).
3. **Above about 90k:** the cap stops binding. The merged window holds only about 110k points, so a 105k cap sends nearly everything. Bandwidth is 11.4 Mbit/s, with no further quality available.
4. **Nothing broke at any level.** The current, rail voltage, temperature and refresh rate showed no cap dependence.

## Recommendation
- Safe on every measured criterion up to 105k merged and 25k live. The 5 A limit is never close.
- The practical constraint is the **link**, not the Pi. Per viewer, 60k costs about 6.5 Mbit/s and 90k about 10 Mbit/s; a weak WiFi link or several viewers is where it would hurt first.
- Suggested default: **merged 60,000, live 12,000**. That is about three times the points, 73% 5 cm coverage versus 53%, 6.5 Mbit/s, streamer CPU about 23%. Use 90,000 when a strong link is guaranteed.
- This is a recommendation only. **The live service still runs the previous caps (20,000 merged / 5,000 live)**; the sweep script restored them.

## Caveats (important for the thesis)
- **No real WiFi client was connected** (clients 0-0 in every step). The streamer encoded and served the measurement client's requests, but the Pi's WiFi radio TX power, and the extra current it draws, is not in these numbers. Repeat one or two levels with a real laptop or phone connected before quoting a total system current.
- Current is sampled at about 32 Hz, so very short spikes (sub-30 ms) could be missed. VDD_CORE_A is the 0.87 V core rail, not input current; its max (about 7 to 9.7 A) must not be compared with 5 A.
- Fidelity percentages are noisy at about plus or minus 5 points, because the scene and the full-cloud size (108k to 125k points) change between steps. Compare trends, not neighbouring steps.
- 70k merged: the fused-rate probe returned 0.0 Hz; a live check showed the topic alive and the 80k step was normal. Treat the fused rate at 70k as a probe failure.
- 105k merged: the fidelity sample captured only a partial cloud (39k points), so the 105k coverage and spacing values are invalid. Wire size, bandwidth, CPU and power for that step are valid.
- 20k merged coverage (53%) and 30k (52%) show the same noise; the 5 cm coverage is not strictly monotonic below 40k.
