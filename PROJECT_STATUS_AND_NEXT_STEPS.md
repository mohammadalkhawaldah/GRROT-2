# Human cup demonstration → ROS 2 / Gazebo → robot demonstration dataset

**Updated:** November 15, 2026 — Asia/Amman
**Project:** `/home/mohammad/Documents/Groot2-1`
**Current milestone:** A simulated arm completes a programmed cup grasp, approximately 30 cm tabletop drag, release, and retraction. The visible Gazebo replay passes at approximately real time, and the user has confirmed it works.

## 1. Objective and current boundary

Our objective is to reproduce the recorded human cup-grasp, horizontal drag, and release demonstration with a simulated robotic arm using **ROS 2 Humble and Gazebo Fortress on Jetson Orin NX 16 GB**. We then want to collect robot demonstrations suitable for evaluation against the official **NVIDIA GR00T 2** adaptation/fine-tuning workflow.

The working result today is a **programmed simulation baseline using measured contact dynamics**. It establishes that the robot can manipulate the cup proxy through contact. The original human trajectory's timing has **not yet driven the robot**, and no learned policy, complete camera/action dataset, or GR00T 2 training result exists.

## 2. What we accomplished

### Environment and supported stack

- Audited the actual Jetson before installation: ARM64/aarch64, Ubuntu 22.04.5, JetPack 6.2.1, ROS 2 Humble, and Gazebo Fortress 6.18.0.
- Reused the installed ROS/Gazebo bridge and installed missing native control, robot-description, and MoveIt dependencies. The dependency manifest records 73 package additions across setup; existing packages were not upgraded or removed.
- Selected the **Kinova Gen3 7-DoF arm with Robotiq 2F-85 gripper**, using established vendor descriptions, meshes, and ROS packages.
- Established `gz_ros2_control`, arm/gripper joint trajectory controllers, joint-state feedback, TF, and native MoveIt planning. The final contact configuration uses local `pick_ik` with a displacement penalty.
- Kept the work inside this project. Existing CUDA, Torch/Python environments, Scout, ROS2-SWARM, and other projects were not rebuilt.

### Human demonstration files

All five supplied files were copied from the Seagate drive into `data/raw/` and SHA256-verified. The drive initially disconnected, but reconnection resolved access; the local copies now remove that dependency for analysis.

| File | Verified contents |
| --- | --- |
| `2026-11-15 20-59-05.mkv` | Original video: 1920 × 1080, 30 fps, 14.3 s |
| `2026-11-15 20-59-05_hand_landmarks.csv` | 5,586 landmark rows; 266 complete hand/frame groups; 253 distinct detected frames |
| `2026-11-15 20-59-05_hand_preview.mp4` | Hand overlay video with 427 frames |
| `human_demonstration_phases.csv` | 427 joined tracking/phase rows, from 0 to 14.2 s |
| `cup_rough_scale_5cm.csv` | 427 rows of approximate displacement estimates |

The hand CSV has image-space XY and relative/model Z, not calibrated human world coordinates. All joined human `gripper_state_verified` values remain `unknown`. Phase boundaries are approximate.

Validation evidence: [`outputs/demo_validation.json`](outputs/demo_validation.json).

### Approximate human target reference

Created [`data/derived/tabletop_reference.csv`](data/derived/tabletop_reference.csv) and its [metadata](data/derived/reference_metadata.json).

The preparation script:

1. Selects 52 Drag-labelled cup samples, approximately 5.3667–7.0667 s, with template score ≥ 0.7.
2. Applies a five-sample median filter, projects onto the observed 2D movement direction, and enforces monotonic progress.
3. Normalizes the movement to **0.30 m along simulated world +Y**.
4. Retains source timestamps and labels robot-specific approach/retraction geometry, gripper angles, and orientation as assumptions.
5. Excludes post-release tracking from motion estimation, including the known approximately 9.5 s tracking jump.

This is a heuristic tabletop target reference. It is not a camera calibration, recovered human 6DoF trajectory, executed action dataset, or learned skill.

### Robot control and physical contact baseline

- Created a world containing a table, cup proxy, fixed-base arm, and parallel gripper.
- Verified advancing simulation time, active controllers, arm tracking, and all five coupled finger joints.
- Added table/cup collision geometry to MoveIt and require complete collision-checked Cartesian paths before execution.
- Used a tested pre-grasp joint target to keep the arm on a consistent configuration. MoveIt plans and checks the approach route on every run.
- Verified the downward end-effector pose before closing and require recent **bilateral fingertip contact** before dragging.
- Completed grasp → horizontal drag → opening → retraction, with measured cup motion and cleared finger contacts afterward.
- Recorded actual robot/cup traces and requested robot trajectories. No cup attachment constraint or cup pose command is used during the trial.

### Visible Gazebo and real-time operation

Opened the native Gazebo GUI on the Jetson desktop and replayed the complete test. Rendering uses the NVIDIA Tegra Orin GPU.

The original 1 ms physics configuration ran at approximately 0.3× real time. Direct startup at larger steps failed the gripper initialization check, so those configurations were rejected.

The validated visible mode now:

1. Initializes the robot/controllers at **1 ms**.
2. Opens the gripper and verifies the controller action.
3. Changes Gazebo's physics step to **4 ms** through its native `set_physics` service.
4. Runs the complete manipulation test and leaves the GUI open afterward.

| Latest validated visible-run measurement | Result |
| --- | ---: |
| Complete trial | PASS |
| Mean sampled real-time factor during motion | 1.00007× |
| Sampled real-time-factor range | 0.9954–1.0081× |
| Requested cup +Y travel | 0.300000 m |
| Achieved final cup +Y travel | 0.299523 m / 29.95 cm |
| Cross-track X displacement | −0.335 mm |
| Maximum cup-center height change during drag | 0.565 mm |
| Finger contacts clear after release/retraction | Yes |

These measurements describe the assumed Gazebo model, not accuracy against calibrated human world motion or a real robot. Real-time physics means simulation time advances with wall time; the robot's motions still use deliberate baseline trajectory durations rather than the video's human timing.

Evidence: [`outputs/realtime_validation.json`](outputs/realtime_validation.json), [`outputs/realtime_stats_motion.json`](outputs/realtime_stats_motion.json), and [`outputs/visible_demo_result.json`](outputs/visible_demo_result.json).

## 3. Important implementation choices and limitations

- **Cup proxy:** rigid solid cylinder, measured diameter approximately 0.05 m. Height 0.10 m, mass 0.02 kg, and friction coefficient 0.4 are assumed. Hollow/tapered/deformable paper-cup behavior is not modeled.
- **Scene:** table top at world z = 0.4 m; grasp end-effector target z = 0.615 m; pre-grasp/retraction target z = 0.75 m.
- **Vendor files:** installed descriptions remain untouched. Local compatibility copies remove two unsupported optional macro arguments and expose measured mimic-joint interfaces.
- **Arm bounds:** the vendor `use_external_cable=true` option supplies ±2π revolute bounds for joints 1, 3, 5, and 7, avoiding continuous-angle wrapping during the planner/controller handoff.
- **Gripper:** starts/opens at 0.1 rad; empty-space closing target is 0.4 rad. Closure permits 0.08 rad error because contact stops the fingers before that angle, but bilateral contact is independently required. Opening retains 0.03 rad tolerance. This is position-servo contact, not regulated grip force.
- **Arm checks:** explicit goal tolerances and independent measured endpoint checks prevent action status alone from being counted as success.
- **Startup consistency:** the runner generates the model before launching and verifies that MoveIt uses the same description as the simulator.
- **Physics resolution:** the validated 4 ms mode trades temporal resolution for speed. The original 1 ms world and baseline are retained for comparison; performance is not guaranteed for future, more complex scenes.
- **Data:** robot traces are asynchronous snapshots and command timestamps are client-side times. There is no synchronized robot-camera episode dataset yet.
- **Evaluation:** nominal successes have been demonstrated. A systematic success rate over repeated and perturbed starts has not been established.
- **GR00T 2:** no official dataset compatibility, training recipe, checkpoint, Jetson fine-tuning feasibility, or closed-loop learned performance has been validated in this project.

Detailed assumptions: [`docs/ASSUMPTIONS.md`](docs/ASSUMPTIONS.md). Failed attempts and their diagnostic logs are retained in `outputs/`.

## 4. How to run what works now

Run commands from the project directory. Avoid running two copies in the same ROS domain/Gazebo partition. Close the existing visible demo before starting a new one.

### Watch the full test at approximately real time

```bash
cd /home/mohammad/Documents/Groot2-1
bash scripts/run_visible.sh
```

The scene opens, waits approximately 10 seconds for the desktop, then runs the sequence. Startup briefly uses the finer physics step. The GUI stays open after completion; closing it stops this demo's server and planner.

### Run the original 1 ms contact verification without the GUI

```bash
cd /home/mohammad/Documents/Groot2-1
bash scripts/verify_contact.sh
```

In a fresh shell, the default remains 1 ms throughout. Clear any manually exported `CUP_RUNTIME_STEP_S` or `CUP_PHYSICS_STEP_S` overrides when comparing configurations.

### Watch with 1 ms physics throughout

```bash
CUP_RUNTIME_STEP_S='' bash scripts/run_visible.sh
```

### Control-only smoke test

```bash
bash scripts/verify_headless.sh
```

Scripts source `/opt/ros/humble/setup.bash`. Defaults are `ROS_DOMAIN_ID=61` and `IGN_PARTITION=human_cup_drag`; both can be overridden. No colcon build is needed for these project launch scripts.

## 5. Planned next steps

### Step 1 — Execute the human-derived reference

**Immediate next implementation task:** connect `data/derived/tabletop_reference.csv` to the verified robot contact sequence instead of using the current programmed drag profile.

- Use the human-derived cup progress and approximate phase boundaries for approach, closure, drag, hold, release, and retraction.
- Preserve relative timing where feasible; apply and record an explicit global time scale to respect joint velocity/acceleration limits and contact stability.
- Retain robot-specific grasp geometry, measured bilateral contact, collision checking, and failure gates.
- Compare measured cup position, displacement, timing, and release behavior against the normalized reference.
- Record all filtering, normalization, and time-scaling assumptions. Treat the result as geometric/temporal retargeting, not learning.

**Acceptance:** a complete Gazebo rollout actually driven by the human reference, with a reproducible command trace, measured reference-tracking errors, and an honest report of timing deviations and failures.

### Step 2 — Record one synchronized robot demonstration episode

- Add a native Gazebo camera and verify headless/GUI rendering performance on this Jetson.
- Record camera images, robot joint positions/velocities, end-effector poses, gripper state, requested actions, cup pose, contact observations, and task instruction.
- Use a common simulation time base and document camera calibration, coordinate frames, action representation, and episode metadata.
- Validate serialization, timestamp alignment, image/state coverage, and reloadability before collecting more episodes.
- Keep failed trials as diagnostics, separate from clean successful demonstrations.

**Acceptance:** one complete, reloadable, synchronized image/state/action/object episode with success evidence. Do not label its format GR00T 2-compatible before checking official requirements.

### Step 3 — Establish repeatability and collect a small clean dataset

- Automate fresh episode resets and deterministic initial conditions.
- Repeat nominal trials and test explicitly recorded perturbations of cup starting position.
- Evaluate displacement error, height change, persistent contact during drag, release, collisions, and failure reasons.
- Report trial counts and success rates; compare the human-derived replay with the existing programmed baseline.
- Save only validated successful rollouts as clean demonstrations, with provenance and configuration hashes.

**Acceptance:** a documented evaluation and a small validated demonstration collection. Fixed replay remains an open-loop baseline unless feedback-driven adaptation is implemented and tested.

### Step 4 — Check official NVIDIA GR00T 2 integration

When proceeding to this stage, verify the then-current official release, code, weights, license, embodiment/action support, data schema, training instructions, and hardware requirements.

- Determine whether the original human video/keypoints can enter the official adaptation path, or whether additional 6DoF/wearable data or robot-action-labelled episodes are required.
- Validate conversion of one episode before scaling dataset preparation.
- Assess training hardware separately; Jetson 16 GB has not been established as a fine-tuning target.
- Use the official GR00T 2 method if available and compatible. Do not substitute another GR00T version and call it GR00T 2.
- Evaluate any trained policy in closed-loop Gazebo trials with unseen starts and compare against the replay baselines.

**Acceptance:** verified official compatibility first; only afterward, a reproducible training run/checkpoint and measured closed-loop evaluation. Report concrete release, format, or hardware blockers if they prevent this stage.

## 6. Main project artifacts

| Location | Purpose |
| --- | --- |
| `data/raw/` | Local verified copies of the five human-source files |
| `data/derived/tabletop_reference.csv` | Approximate human-derived robot target reference |
| `data/derived/reference_metadata.json` | Source hash, filtering, normalization, and limitations |
| `worlds/cup_drag.sdf` | Original 1 ms world and cup/table assumptions |
| `config/` | Controller configuration, tested pre-grasp joint target, visible GUI configuration |
| `launch/` | Native ROS/Gazebo scene and MoveIt planning launch files |
| `scripts/` | Import/validation, reference preparation, control/contact tests, GUI runner, plotting |
| `outputs/successful_baseline/` | Preserved original successful baseline artifacts |
| `outputs/contact_trial.json` | Most recent trial outcome; overwritten by subsequent trials |
| `outputs/contact_trace.csv` | Measured joint/cup states with simulation timestamps and stages |
| `outputs/contact_commands.json` | Requested trajectories and client simulation timestamps |
| `outputs/visible_demo_result.json` | Most recent visible replay result |
| `outputs/realtime_validation.json` | Validated faster-mode performance and manipulation metrics |
| `docs/ENV_AUDIT.md` | Initial machine audit and data-transfer findings |
| `docs/EXPERIMENTS.md` | Detailed results, failures, fixes, and real-time validation |
| `docs/ASSUMPTIONS.md` | Explicit simulation and measurement boundaries |
| `docs/dependency_manifest.json` | Installed dependency versions, sources, and checksums |

Output reports may be overwritten by later trials. Preserve configuration, source hashes, reports, commands, and traces together when promoting a run into an episode or publication artifact.
