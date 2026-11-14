# Merged point cloud: from 20,000 to 80,000 points (14 Nov 2026)

Scope: the merged (dense) view only, LiDAR + RGB pipeline on the real Pi 5.

## Result
The viewer now receives **up to 80,000 points per update (4x before) at the same ~1.5 updates per second**, and adapts down automatically on a weak link. Over a 20-minute soak with WiFi-radio load: 79,888 points per update, 1.52 updates/s, about 7.5 Mbit/s, data age 0.50 s (p95 0.64 s), worst-case input current 3.05 A (limit 5 A), SoC 71 to 74 C, no throttling, no memory growth, no warnings.

## What limits the point count (three separate things)
1. **The cap** (`max_points_merged`): what the server is willing to send. It was 20,000.
2. **What the scene provides.** The merge window (0.5 s) produces a different number of points depending on the scene because overlapping points merge. Measured here: 54k to 107k at 0.5 s. A cap above that has no effect.
3. **The link.** About 8.7 Mbit/s per viewer at 80,000 points. This is the real constraint, not the Pi.
Refresh rate is set by the merge window only (not by the cap): 0.25 s gives 2.5 Hz, 0.5 s gives 1.55 Hz, 0.75 s gives 1.1 Hz, 1.0 s gives 0.86 Hz.

## Ladder: windows x caps (one simulated viewer, WiFi radio load, safety gates on)
| window | cap | points shown | refresh | link | data age (median) | worst input A | max temp |
|---|---|---|---|---|---|---|---|
| 0.5 s | 20,000 (before) | 20,000 | 1.56 Hz | 2.2 Mbit/s | 0.48 s | 2.64 | 69 C |
| 0.5 s | 40,000 | 40,000 | 1.58 | 4.3 | 0.44 | 2.77 | 67 |
| **0.5 s** | **80,000 (chosen)** | **80,000** | **1.59** | **8.7** | **0.53** | **2.76** | **68** |
| 0.5 s | all (100k) | 100,075 | 1.56 | 9.0 | 0.61 | 2.56 | 68 |
| 0.25 s | all | 65,563 | 2.52 | 9.6 | 0.39 | 2.97 | 70 |
| 0.75 s | all | 132,531 | 1.11 | 8.3 | 0.64 | 2.71 | 70 |
| 1.0 s | all | 150,000 | 0.90 | 9.1 | 0.84 | 2.63 | 71 |
No step showed aborts, HTTP errors, merge-worker backlog, ICP non-convergence, a dip in the 5 V rail (minimum 4.90 V) or throttling. Fused rate stayed at 10 Hz throughout. Coverage of the full cloud's 5 cm cells rose from 57% (20k) to 74% (40k) to 86% (80k) to 89% (all).
The estimated input current shows **no trend with point count** (2.6 to 3.0 A across every step). The WiFi rail averaged about 0.1 A (max 0.4 to 0.46 A) at every level.

Choosing 80,000 at 0.5 s: it keeps the refresh the same as before. Going to 0.25 s buys 2.5 Hz but only about 65k points in this scene; 0.75 to 1.0 s buys 130k+ points but halves the refresh.

## Weak links (new page with adaptive quality vs the old page)
| link | new page: points | new page: updates/s | old page (fixed 80k, 4 s abort) |
|---|---|---|---|
| 20 Mbit/s | 80,000 | 1.59 | |
| 10 Mbit/s | about 69,000 | 1.50 | |
| 5 Mbit/s | about 30,000 | 1.65 | |
| 3 Mbit/s | about 14,000 | 1.77 | |
| 2 Mbit/s | about 13,000 | 1.58 | 0.40 updates/s, data 3.2 s old |
| 1 Mbit/s | 8,000 | 1.70 | **nothing: 20 aborted downloads** |
Without the change, a high cap would have blanked the viewer on a weak link (the old page abandons any download over 4 s and retries forever). Now each viewer picks its own quality.

## Several viewers
- 3 simultaneous viewers: each 1.45 to 1.48 updates/s, 15.7 Mbit/s combined, streamer 33% of a core, worst-case input current 2.39 A, max temp 76 C.
- An accidental run with 120 simulated viewers (a typo in my arguments) also held: merged 1.51 Hz, fused 9.9 Hz, no errors, 2.4 A. Each cloud level is encoded once and shared, so extra viewers cost the server almost nothing.

## Code changes (all in the repo, 31 tests pass)
1. `fusion_web_streamer.py`: clients may request `?n=<points>`; each (cloud, level) is encoded once and cached; the cap is the ceiling; default raised to 80,000. The live cloud is only received while someone asks for it.
2. `viewer.html`: adaptive quality (steps down quickly, up slowly), 8 s abort instead of 4 s, point discs sized to the density, and two new HUD lines (quality level; download / decode ms) to read on a real device.
3. `test_streamer_tiers.py`: 6 new tests (levels, caching, sequence numbers, HTTP 200/304/404). The page's controller code passes 11 logic tests in a JavaScript engine; decoding 80,000 points takes about 20 ms there.

## Second audit (same day) and what it fixed
A line-by-line review after the first write-up found and fixed four things, all covered by tests (33 pass):
1. The page counted cached "unchanged" replies as fast downloads, so it would have climbed back to a heavy level too eagerly after a slow link. Only a newly received cloud now moves the level (the simulator already behaved this way).
2. A runtime change of the ceiling could reuse a sequence number. Each encoded payload now gets its own increasing number, and the latest cloud is stored as one object so readers never see a mixed pair.
3. The lens-distortion maths now ignores points more than about 76 degrees off-axis, so an extreme point can never overflow into a valid-looking pixel.
4. My test harness reset the live cap to a fixed 20,000 when it finished. It now restores whatever value it found, and the production cap was put back to 80,000 and checked (the stream serves 80,000 points).
Re-verified after the fixes: live 80,000-point run at 1.54 updates/s, 7.4 Mbit/s, zero aborts, worst-case input current 2.4 A, 74 C.

## Limits (read before quoting)
- No real WiFi client was connected. The viewer ran on the Pi, so I generated real radio traffic to the router out of wlan0, matching the viewer's byte rate (confirmed by the wlan0 transmit counter, e.g. 7.79 Mbit/s sent vs 7.71 measured). In the 3-viewer run the radio only carried about 9 of 15.7 Mbit/s. A final check with a real phone or laptop is still worth doing.
- Browser rendering on a real device was not measured. 80,000 points is small for any GPU, but check smoothness on your device; the new HUD line shows download and decode time.
- One indoor scene. Scenes with close objects merge more duplicates and provide fewer points, so the cap may not be reached.
- Current is estimated from the PMIC rails (conservative figure) at about 31 Hz, not measured at the input.
- Streamer CPU at 80k was 24% in the soak vs 28% in the earlier ladder, consistent with the live-stream saving but not isolated.
- The Pi's clock was stepped once by +0.3 s by network time during the testing (the restamp node reset itself as designed). The Pi has no battery clock and corrects by steps, so an occasional step is normal.
- The 11 Nov sweep's suggestion of 60,000 is superseded by these results.

## Files
`code/` (campaign harness, runner scripts, analysis), `data/` (results, 31 Hz power traces, logs), `results/` (this report, ladder table and plot, page logic test), `staging/`, `backup_before_adaptive/` (previous streamer and page).
