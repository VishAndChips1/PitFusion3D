# Livox scan pattern data

`avia.csv` is the non-repetitive Avia scan pattern table from
[Livox-SDK/livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)
(`scan_mode/avia.csv`, commit `dffebcbb057d37376ebcc2550cadfa498a8490c8`),
used under that project's MIT license.

Columns are `Time/s, Azimuth/deg, Zenith/deg`, one row per emitted point.
960000 rows cover the full non-repeat cycle (24000 points/scan at the
upstream 10 Hz rate == a 40-scan / 4 s period before the exact ray pattern
repeats). `livox_pattern_filter.py` walks this table the same way the
upstream `LivoxPointsPlugin` does: a running index into the table advances
by `points_per_scan` every update and wraps with modulo, so consecutive
scans use consecutive, non-repeating slices of the real Avia pattern.
