# Offline MediaPipe 3D hand analysis

This pipeline reads the original demonstration and existing CSVs. It never
imports ROS, commands the robot, edits the working baseline, or starts training.
Results are written only under `outputs/hand3d/`.

## Reproduce on this Jetson

The project-local `.venv-hand3d` uses Python 3.10 and the official ARM64 MediaPipe
0.10.18 wheel. The previously used Windows 0.10.21 version has no matching
Python 3.10 ARM64 wheel. Native system OpenCV 4.5.4, NumPy 1.21.5, SciPy and
Matplotlib are reused through `--system-site-packages`; ROS/Torch environments
are left alone. The minimal legacy inference path was exercised directly;
optional MediaPipe Tasks/JAX features are not part of this environment.

```bash
cd /home/mohammad/Documents/Groot2-1
python3 -m venv --system-site-packages .venv-hand3d
.venv-hand3d/bin/python -m pip install --no-deps -r analysis/hand3d/requirements-runtime.txt
.venv-hand3d/bin/python analysis/hand3d/test_geometry.py
.venv-hand3d/bin/python analysis/hand3d/extract.py
.venv-hand3d/bin/python analysis/hand3d/extract.py --static --out outputs/hand3d/static
.venv-hand3d/bin/python analysis/hand3d/analyze.py
.venv-hand3d/bin/python analysis/hand3d/render.py
```

The source is `data/raw/2026-11-15 20-59-05.mkv`. The original landmarks and joined
phase CSV are read for comparison and primary-hand selection. Source hashes,
bundled model hashes, versions, inference settings and elapsed time are in each
extraction's `metadata.json`. Raw inputs are never overwritten. Approximate
phase labels are inherited from the existing project, not newly verified contact.

## Outputs

| File under outputs/hand3d | Contents |
| --- | --- |
| `video/landmarks.csv` | All 21 image and `multi_hand_world_landmarks` XYZ points for every detected hand |
| `video/hand_pose_flexion.csv` | Per-hand timestamp, raw handedness score, wrist/palm position estimates, palm R/quaternion, 15 bend proxies |
| `video/frames.csv` | Exactly one row per decoded frame, including zero-hand frames |
| `video/landmarks.npz` | Same image/world arrays for rendering and quantitative inspection |
| `static/` | Independent per-frame detector outputs using the same full model |
| `primary_pose_quality.csv` | One selected hand or missing row per source frame, raw jumps/identity/label flags, separate display-filter quaternion |
| `phase_quality.csv` | Phase detection coverage, raw angular steps, mode disagreement and width variation |
| `summary.json` | Aggregate metrics and recorded robot comparison provenance |
| `tracking_quality.png` | Missing data, raw/filtered steps, independent-mode disagreement, finger bends |
| `robot_baseline_fk.csv` | Offline forward kinematics from the recorded successful robot joints and existing URDF |
| `robot_comparison.png` | Relative rotation magnitudes and normalized cup progress, aligned only by phase fraction |
| `hand3d_review.mp4` | Image overlay beside raw wrist-centered 3D skeleton; 15 fps, every second source frame |
| `review_keyframes.jpg` | Representative stable/unstable frames for visual review |

The legacy `multi_hand_world_landmarks` graph is used directly. MediaPipe
timestamps are assigned from decoded frame index / actual source FPS. No frames
are deliberately skipped during extraction; the separate review video is
subsampled for display. Missing detections have no fabricated landmark positions.

## Coordinate and confidence contract

- `mp_world_*_model_m` is nominal meter-scale, hand-centered model output. It is
  not a metrically reconstructed camera/world point or tabletop trajectory.
- Wrist XYZ and palm-center XYZ retain the returned model origin. Rendering
  subtracts the wrist only to make articulation visible; it never adds an
  invented global translation.
- Palm center is the mean of points 0, 5, 9, 13 and 17. Let `y` be normalized
  wrist→middle-MCP; project little-MCP→index-MCP perpendicular to `y` to obtain
  `x`; let `z=x×y` and re-orthogonalize. `R=[x y z]` maps the constructed palm
  frame into the uncalibrated model-output axes. Quaternion order is **xyzw**.
  This attached palm frame is a wrist-orientation proxy, not an independent
  wrist/forearm joint pose.
- Collinear/degenerate palms are rejected. `orientation_valid` means the basis
  can be constructed, not that its physical orientation is accurate.
- Finger bends are `180° − angle(a−b, c−b)`: 0° means geometrically straight.
  MCP/CMC fields are proxies; thumb opposition, anatomical joint axes, signed
  flexion and tendon/force information are not recovered.
- Raw handedness labels are preserved; no horizontal flip is applied by this
  pipeline, and the source's original mirror convention is unverified.
- `handedness_score` is a classifier score, **not tracking/pose confidence**.
  This legacy API exposes no per-frame tracking/detection or populated
  per-landmark confidence in this run. Those fields are blank, not invented.
- Hand association is heuristic 2D-wrist matching with a 0.25 s gap limit. The
  primary candidate is chosen nearest the existing image wrist where available;
  this does not provide identity ground truth. Track/label changes are flagged.

## Evaluation contract

Raw orientation jumps are flagged above 30° between adjacent frames in the same
track (a review threshold, not a measured biological bound). Missing frames,
identity boundaries and handedness flips remain explicit. The separate causal
five-sample rotation mean is for display only; it resets at gaps, track changes,
label flips and jump flags. Quaternion hemisphere adjustment changes only q/-q
representation. It does not correct physical 180° model flips.

Filtered step statistics exclude reset boundaries and are not evidence that
flips or occlusion have been solved. Video/static agreement measures sensitivity
to inference mode; two passes can agree and still be wrong. Width variation is
a consistency warning, not a calibrated metric-scale error.

Robot comparison uses the preserved successful baseline's recorded joints and
plain URDF forward-transform composition. Neither the simulation nor its
controllers are launched. Different human/robot frames and timing are not
registered: only phase-relative scalar rotation changes and normalized cup
progress are plotted. No human-to-robot absolute pose error is identifiable.

For findings and suitability, see `docs/HAND_3D_ANALYSIS.md` from the project root.
