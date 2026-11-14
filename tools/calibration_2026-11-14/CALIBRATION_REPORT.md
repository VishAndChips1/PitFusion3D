# RGB camera + LiDAR calibration from the 5 checkerboard bags (14 Nov 2026)

Scope: RGB camera and LiDAR. The thermal path is not part of this analysis.

## What was done
- Board: plain checkerboard, 5 x 7 squares of 58 mm, top-left square black, so 4 x 6 inner corners.
- Data: `updated_bags/` (close, far, left, right, centre), 5 s each, 50 RGB frames and 50 LiDAR scans per bag.
- All 251 RGB frames detected the board (sub-pixel corners, frame-to-frame jitter at most 0.1 px).
- The board's light squares are dim in the camera image, so detection needed a contrast boost (CLAHE) and a per-bag region of interest.

## Result 1: the camera lens (intrinsics)
| | Placeholder used until now | Measured |
|---|---|---|
| Focal length | 1065 px (assumed ~62 deg FOV) | **1330.7 px** |
| Principal point | (640, 360) | (616.7, 369.3) |
| Lens distortion | none | **k1 = -0.476, k2 = +0.223** (strong barrel distortion) |
| Reprojection error | n/a | **0.11 px** |

Checks:
- Leave-one-bag-out: focal length 1307 to 1352 px (+-1.7 %). The old ChArUco attempt swung from 970 to 2640 px.
- Independent check with the LiDAR: the board's distance from the camera model agrees with the LiDAR-measured distance to about 1 % (close +0.7 cm, far +4.6 cm at 3.6 m, left -0.9 cm, centre -0.9 cm). The placeholder focal length would be wrong by 25 %.
- A variant with the principal point fixed at the image centre fits worse (0.14 px) and gives f = 1153 px, which the LiDAR distances reject.

## Result 2: where the LiDAR sits relative to the camera (extrinsics)
Solved from board plane distance plus the known board outline (29 x 40.6 cm):
- Rotation (roll, pitch, yaw): -1.99, -0.50, -0.55 deg.
- Translation (x, y, z): -3.8, +0.5, -5.1 cm.
- LiDAR scatter about the board plane: 1.8 cm, which is the sensor's own range noise.
- Uncertainty from leaving out each pose in turn: roughly +-0.6 deg and +-2 cm (roll +-0.6, yaw +-0.7, y +-3 cm, z +-1.5 cm).

## Result 3: does it work? (held-out test)
For each pose, the extrinsic was solved WITHOUT that pose. LiDAR board points were then projected into the image and compared with the true board outline.

| Model | Overlap with true board (IoU) | Points inside outline | 90th percentile miss |
|---|---|---|---|
| Before (placeholder lens, no extrinsic) | **54.9 %** | 76.1 % | 20.5 px |
| Calibrated lens only | 77.8 % | 87.8 % | 3.2 px |
| Calibrated lens + new extrinsic (held out) | **83.0 %** | 90.8 % | 1.4 px |

Every pose improved (left: 33 % to 81 %). The ceiling is around 85-90 %, because LiDAR points are sparse and noisy at the board edges. Figure: `E01_board_overlay_before_after.png`.
Running the production code and config files on the same data gives 83.7 % (in-sample), so what is deployed matches what was tested.

## What changed in the pipeline
- `intrinsics.yaml` (rgb_camera) and `extrinsics.yaml` (lidar_frame) hold the new values; the old files are in `backup_before_calibration/`.
- `sensor_fusion_node.py` now applies the lens distortion (default is none, so other cameras are unaffected). Two new tests check it against OpenCV (within 1 px); all 22 tests pass.
- Pipeline restarted and checked: fused clouds 10.1 Hz, merged 1.4 Hz, same as before.

## Limits (read before quoting)
1. All five poses had the board facing the sensors (no tilt). That pins down distance, scale and the lens well, but constrains the LiDAR-to-camera rotation (especially roll) only moderately, hence the +-0.6 deg and +-2 cm.
2. The right pose has the board's right edge cut off by the image border; the inner corners were all still found.
3. The principal point moved about 23 px from the centre. Leaving one bag out moves it by up to 15 px, so treat it as less certain than the focal length.
4. Indoor scene, low-contrast board. This is not a field validation.
5. The RGB-to-thermal offset in `extrinsics.yaml` was set relative to the old LiDAR frame and has not been re-derived here.

## To tighten it further (needs 10 minutes with the board)
Record a second set with the board tilted about 30 deg left/right and up/down at 2-3 m. That adds the missing rotation information, and I can rerun everything with `results/*.py`.

## Files
`extract.py` (bags to arrays), `detect_final.py` (corner detection), `intrinsics.py`, `lidar_board.py`, `extrinsic.py`, `evaluate.py`, `run_safe.sh`, `evaluation.json`, `intrinsics_variants.json`.
