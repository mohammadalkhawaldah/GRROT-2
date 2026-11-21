# Human-to-Gazebo Sandwich Assembly — complete Codex handoff

**Scope:** Four-step sandwich assembly from one real human demonstration, transferred to the **existing Kinova arm + Robotiq gripper in ROS 2 Humble / Gazebo Fortress on Jetson Orin NX 16 GB**. **Do not replace the working cup-drag baseline.**

## What's in this archive

- `data/raw/2026-11-21 11-02-55.mkv`: **original human video**, 1920×1080 at 30 FPS, 1,637 decoded frames (~54.57 s).
- `data/mediapipe/2026-11-21 11-02-55_hand_landmarks.csv`: actual output of the **working Windows MediaPipe 0.10.21** extraction; 32,067 landmark rows; hands detected in 1,476/1,637 frames (90.2%). Columns include `frame,time_s,hand_index,handedness,landmark_index,x_px,y_px,z_relative_not_meters`.
- `data/review/2026-11-21 11-02-55_hand_preview.mp4`: annotation of detected hand points.
- `data/derived/sandwich_hand_object_timeline_10hz.csv`: 546 frame-sampled records, approximate thumb/index midpoint, wrist, tomato HSV candidates, rough plate centers, phase names. **Tomato candidates are unverified, not tracked ground truth.**
- `data/derived/sandwich_ingredient_events_review.csv`: **manually estimated**, non-contact-verified approach/pickup/release times.
- `data/derived/sandwich_object_keyframes_review.csv`: **manually approximated image pixels** for ingredient sources/destinations; bread and cheese are NOT continuously tracked.
- `data/review/`: annotated video and contact sheets for manual inspection.
- `docs/SCENE_SPEC.json`: proposed simulator task with explicitly unknown calibration, geometry, object mass and friction.
- `docs/CODEX_INSTRUCTIONS.md`: staged implementation instructions and acceptance gates.
- `scripts/validate_bundle.py`: dependency-free integrity/data checks.
- `MANIFEST_SHA256.txt`: SHA-256 checksums of packaged inputs and documentation.

## Explicit limits / evidence quality

1. The media are user-recorded human actions. No measurements of actual physical object contact, grip force, or human wrist 6DoF are present.
2. Current data are **uncalibrated image pixels**. `z_relative_not_meters` is MediaPipe's relative model output, **not depth in meters**. No camera intrinsics, homography, world coordinates, measured bread/cheese/tomato dimensions, or lift heights are provided.
3. Ingredient event times and source/destination keyframes are **approximate visual review**, not precise physical contact timestamps. Tomato HSV detections contain unverified false-positive risk.
4. The ROS/Gazebo cup-drag success (physics-contact grasp and drag) was reported by Codex in the conversation; **its Jetson project files are not in this archive**. Inspect the existing project on Jetson rather than assuming paths, controller names, URDF, or service interfaces.
5. The first sandwich deliverable is a **simulation demonstration following human-derived task cues**, not an AI model trained from human demonstrations. Do not claim autonomy, human-to-robot learning, or GR00T fine-tuning.

## Start on Jetson

1. Copy the zip to Jetson, `unzip sandwich_codex_experiment.zip` into a working directory.
2. Run `python3 sandwich_codex_experiment/scripts/validate_bundle.py sandwich_codex_experiment`.
3. Ask Codex to read `sandwich_codex_experiment/docs/CODEX_INSTRUCTIONS.md`, then inspect its **existing working cup project**.
4. Start with a scene/coordinate audit, not with rebuilding ROS, installing competing Gazebo versions, or training.

**Long-term goal:** capture many varied human demonstrations, validate human-derived robot simulation rollouts, then investigate reproducible pre-trained robot policy fine-tuning/transfer (including GR00T 2 when officially available). The current archive **does not** conform to a verified GR00T 2 training schema.
