# MediaPipe 3D hand extraction and suitability assessment

**Date:** November 16, 2026 — Asia/Amman  
**Status:** Offline extraction and evaluation complete. The working ROS 2/Gazebo implementation was preserved. No training or robot control was performed.

## Outcome

The original clip now has image landmarks **and all 21 `multi_hand_world_landmarks` XYZ estimates for each detected hand**, geometric palm orientation, 15 individual finger bend proxies, timestamps, missing-detection rows, association/label flags and available handedness scores.

The additional 3D output is useful for visual inspection and relative motion cues during stable grasp/drag/hold segments. **It is not suitable for direct robot end-effector pose replay.** Large flips occur outside those stable segments; the hand-centered origin does not recover the human wrist's global translation; model-output axes are not calibrated to the camera or robot.

## Inputs, environment and reproducibility

- Original video: `data/raw/2026-11-15 20-59-05.mkv`, SHA256 `bc44b754a047e1a552e9ea1b9e8ed712dfe4f835c65499485aaff1fd9eaaca0d`.
- Existing image landmark CSV: `data/raw/2026-11-15 20-59-05_hand_landmarks.csv`.
- Existing joined tracking/approximate phases: `data/raw/human_demonstration_phases.csv`.
- Dedicated `.venv-hand3d`, Python 3.10, official **MediaPipe 0.10.18 ARM64 wheel**. The previously used Windows 0.10.21 package has no matching Python 3.10 ARM64 wheel. This is therefore not a bit-for-bit rerun of that Windows environment.
- Existing native OpenCV 4.5.4, NumPy 1.21.5, SciPy and Matplotlib reused through an isolated environment with system-package access. No ROS/Torch environment was overwritten.
- Legacy `mp.solutions.hands.Hands`: full landmark model (`model_complexity=1`), two-hand maximum, detection/tracking thresholds 0.5. A sequential video pass and an independent static-image pass both process every decoded frame.
- Source and model hashes, runtime/settings and software versions are recorded in each pass's metadata. First video-pass inference took about 46.5 s for the 14.23 s decoded video; this was offline CPU processing.

Commands and field definitions: [analysis pipeline README](../analysis/hand3d/README.md).

## Main artifacts

| Artifact | Purpose |
| --- | --- |
| [Primary pose/quality CSV](../outputs/hand3d/primary_pose_quality.csv) | 427 rows: timestamps, selected hand or missing row, model wrist/palm XYZ, raw R/quaternion, 15 bend proxies, scores, flags and separate display-filter quaternions |
| [All-hand pose/flexion CSV](../outputs/hand3d/video/hand_pose_flexion.csv) | Pose/finger estimates for all detections, with explicit missing-frame records |
| [All image/world landmarks](../outputs/hand3d/video/landmarks.csv) | 5,460 rows: 260 detected hand instances × 21 paired image/world landmarks |
| [Frame coverage](../outputs/hand3d/video/frames.csv) | One row for every decoded frame, including no-hand frames |
| [3D review video](../outputs/hand3d/hand3d_review.mp4) | Original image overlay beside raw wrist-centered 3D skeleton and palm axes |
| [Representative frames](../outputs/hand3d/review_keyframes.jpg) | Stable drag and problematic approach/retraction frames |
| [Tracking-quality figure](../outputs/hand3d/tracking_quality.png) | Detection gaps, angular steps, mode disagreement and finger bends |
| [Robot comparison figure](../outputs/hand3d/robot_comparison.png) | Phase-relative rotation magnitudes and normalized cup motion |
| [Phase quality CSV](../outputs/hand3d/phase_quality.csv) | Coverage, jitter, jumps, disagreement and width consistency by phase |
| [Summary/provenance JSON](../outputs/hand3d/summary.json) | Quantitative results and comparison hashes |
| [Recorded robot FK CSV](../outputs/hand3d/robot_baseline_fk.csv) | End-effector pose derived offline from recorded successful robot joints and the existing URDF |

The review video is 1280×560, 15 fps, approximately 14.27 s, displaying every second source frame. Extraction itself processes all 427 frames at the source's 30 fps. Missing hands are shown explicitly; no global translation or missing landmarks are fabricated.

## Definitions and confidence limitations

The palm center is the average of wrist and MCP points 0, 5, 9, 13 and 17. The palm basis uses wrist→middle MCP as its longitudinal axis; the across-palm axis is little MCP→index MCP projected perpendicular to it. Their cross product supplies the third axis. Matrix columns are the palm axes expressed in MediaPipe's model-output coordinates; quaternion order is **xyzw**.

The resulting frame also acts as a wrist-orientation proxy. There is no independent forearm reference, so anatomical wrist flexion, pronation and roll relative to the forearm are not measured. Orthonormality/determinant checks verify the construction's mathematics, not physical accuracy.

Each finger has three unsigned geometric bend estimates, with 0° meaning straight. Thumb CMC and finger MCP values are explicitly named proxies. These angles do not recover signed anatomical joint axes, thumb opposition, tendon state, grip force or contact. Occluded digits remain model predictions.

The legacy API returned **handedness classification scores**, but no populated per-landmark visibility/presence or per-frame detection/tracking scores in this run. Unavailable confidence columns are blank. A high handedness score does not validate the 3D orientation: the clipped-hand frame at approximately 10.33 s has a score near 1.0 despite a large orientation jump. Threshold settings of 0.5 are configuration values, not measured frame confidence.

Raw left/right labels are retained. The pipeline applies no mirror operation; the original recording's mirror convention and anatomical side are unverified. Nine adjacent-frame label changes were observed.

Primary selection uses the existing image wrist where available and otherwise 2D wrist continuity. There are two associated track IDs, with three primary identity switches around 9.47–9.53 s. These association changes are flagged and excluded from adjacent-orientation comparisons. This heuristic is not identity ground truth.

## Detection and orientation results

- **427** decoded frames processed in each pass.
- Video tracking: **253/427** frames with a hand, matching the existing CSV's distinct-frame coverage. The task has gained 3D estimates and diagnostics; detection coverage has not improved.
- During the approximately hand-present Approach–Retract phases: **253/255** frames detected, about **99.2%**. The two gaps are at approximately 1.90 and 10.37 s, the phase boundaries. The 172 idle/no-hand frames should not be counted as tracking failures.
- New video pass: **260** hand instances, versus 266 in the older extraction. Seven frames contain secondary detections, all during retraction; these require review rather than treating them as verified additional hands.
- Static-image pass: **214/427** frames detected. Video/static orientation comparisons use 205 shared primary matches with image wrists less than 80 pixels apart. Mode agreement is a consistency test, not accuracy against ground truth.
- Old/new matched image-landmark repeatability: median approximately **0.74 px**, 95th percentile **4.84 px**. Both are model estimates; this does not establish improved image or 3D accuracy.
- Raw adjacent-frame orientation change: median **3.42°**, 95th percentile **26.01°**, maximum **179.77°**.
- **11** raw orientation jumps exceed the review threshold of 30° per adjacent frame: three in Approach, one in Release and seven in Retract. None occur in Grip, Drag or Hold.

| Phase | Detected / phase frames | Median raw step | 95th-percentile raw step | Jumps >30°/frame | Video/static median disagreement | Video/static 95th percentile |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Approach | 70/71 | 9.96° | 25.85° | 3 | 11.77° | 66.65° |
| Grip | 33/33 | 1.06° | 4.89° | 0 | 8.89° | 18.43° |
| Drag | 52/52 | 2.33° | 6.22° | 0 | 3.69° | 9.11° |
| Hold | 24/24 | 1.18° | 1.91° | 0 | 3.87° | 5.74° |
| Release | 14/14 | 4.12° | 49.12° | 1 | 6.75° | 91.42° |
| Retract | 60/61 | 6.10° | 107.32° | 7 | 4.33° | 51.37° |

These step values contain both actual hand motion and estimator noise; they are not pure jitter or angular-error measurements. Important review intervals include approximately 3.87–3.93 s, 8.23 s, 9.33–9.43 s and 10.20–10.33 s. Side/edge views, partial visibility, changing labels and association ambiguity accompany the worst cases.

Nominal model palm-width variation is another warning. Its coefficient of variation is approximately 43% in Approach, 45% in Release and 40% in Retract; even Drag is approximately 12%. Such output is not a stable metric-scale measurement. Perspective/occlusion and learned depth/shape uncertainty cannot be separated from these data alone.

A separate causal five-sample rotation mean reduces within-segment display noise. It resets at gaps, identity changes, label flips and flagged jumps. Its median step is approximately 0.98°, but this excludes reset boundaries and **does not mean the flips were corrected**. Raw estimates and flags remain available, and the review animation shows raw 3D geometry.

## Individual finger estimates

Median geometric bends support a broad closing/opening interpretation:

| Approximate phase | Index PIP | Middle PIP | Ring PIP | Little PIP |
| --- | ---: | ---: | ---: | ---: |
| Approach | 34.5° | 24.6° | 22.0° | 21.3° |
| Grip | 66.6° | 57.4° | 48.1° | 32.6° |
| Drag | 51.7° | 60.4° | 43.7° | 34.9° |
| Release | 19.6° | 21.0° | 7.4° | 16.8° |

This is useful candidate phase/gesture information, not a measured human grip state. The thumb MCP estimate near 4° during Drag does not imply absence of thumb contact: opposition and the thumb's multi-axis anatomy are not described by that one bend proxy. Exact per-joint values, especially for fingers hidden by the cup, should not be used as measured anatomical angles or converted directly into gripper aperture.

## Comparison with the preserved successful robot demonstration

Comparison reads `outputs/successful_baseline/contact_trace.csv` and its successful report, without starting ROS/Gazebo or changing the baseline. Offline URDF transform composition supplies the robot end-effector orientation from its recorded joint positions. The robot model and trace hashes are stored in `summary.json`.

- Human labelled drag interval: approximately **1.7 s**. Recorded robot baseline drag: **10 s**.
- Human model-palm orientation change relative to each continuous drag segment's start: median **10.57°**, maximum **19.86°**.
- Robot end-effector change relative to drag start: median **0.129°**, maximum **0.429°**; its intended grasp orientation remains approximately fixed.
- Robot cup displacement in that preserved baseline: approximately **0.299642 m** along world +Y, with contact-based grasp/release already verified by the existing experiment.
- Human cup-image progress rises earlier within its drag interval than the programmed robot profile. The figure aligns intervals only by their duration fraction; it does not establish matching temporal dynamics.

The palm and robot tool are different physical frames, with no camera-to-world calibration or hand-to-tool mapping. A palm/tool angular error or 3D trajectory RMSE would therefore be misleading. The model wrist's hand-centered XYZ changes by about 21.7 model mm during the human drag; that change is **not the wrist's tabletop travel** and must not replace the independently estimated approximately 30 cm cup displacement.

## Reliability and suitability for our task

**Usable as reviewed relative cues:** image-space motion, broad finger closing/opening trends, and lower-frequency palm orientation changes inside stable Grip/Drag/Hold segments. These can help propose phase boundaries or a relative orientation prior after explicit frame registration and validation.

**Inaccurate or unverified:** absolute camera/world translation; metric hand size/depth; full-video orientation continuity; exact hidden-finger anatomy and thumb opposition; anatomical side without mirror verification; contact/force; absolute robot pose mapping. No ground-truth 3D measurements were available, so angular accuracy cannot be established from smoothness or cross-mode agreement.

**Recommendation:** preserve the successful robot's fixed downward grasp orientation and measured contact logic. Do not feed these raw palm quaternions or model wrist XYZ directly into robot control. For this horizontal drag, the current fixed tool orientation is already sufficient; copying the estimated palm's roughly 20° variation could disturb the grasp rather than improve task success.

Before using relative orientation in a separate retargeting experiment:

1. Manually verify stable primary-hand identity and choose a reference pose; reject jump/edge/ambiguous segments rather than filling them with smooth-looking estimates.
2. Define and validate the hand-to-tool rotation offset and camera/robot frame relationship. A table homography alone does not establish 3D wrist pose.
3. Use independent known orientations, a rigid marker/reference, depth or multiple views to measure error and confirm the required axes.
4. Limit and time-scale any tested orientation changes, preserve IK/collision/contact gates, and compare task success against the untouched fixed-orientation baseline.

## Checks and preservation

- Analytic geometry tests passed: rigid-transform equivariance/right-handedness, q/-q equivalence, known 0°/90° bends, and degenerate-palm rejection.
- Each detected hand has 21 paired image/world landmarks; missing frames remain explicit.
- Three-dimensional rotation matrices and quaternions were checked numerically for consistency.
- Output video metadata and decoding were checked.
- Existing tracked ROS/Gazebo scripts, launch files, controllers, world, handoff, baseline reports and input files were not edited. New code is under `analysis/hand3d`; new results are under `outputs/hand3d`. The only existing tracked-file change is an ignore entry for `.venv-hand3d`.
- No model training, robot execution, commit or push was performed for this task.

## Primary API references

- [MediaPipe Hands output definitions and mirror/handedness convention](https://chuoling.github.io/mediapipe/solutions/hands.html#output).
- [Google AI Edge Hand Landmarker documentation](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker?tab=Python).
- [Official PyPI release metadata used to verify the ARM64 wheel](https://pypi.org/pypi/mediapipe/0.10.18/json).

The legacy documentation describes the approximate hand-center origin of world landmarks. This project retains explicit `model` labels and does not promote those learned coordinates to calibrated camera/world measurements.
