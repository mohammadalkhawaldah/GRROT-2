# Explicit Codex execution request — sandwich experiment

Proceed with implementing the **next more complex physical manipulation demonstration** on the existing Jetson **ROS 2 Humble + Gazebo Fortress** setup using the *already working* Kinova + Robotiq arm/gripper from our cup-drag experiment. This archive contains the original human video, MediaPipe extraction, visual review, and approximate ingredient-event/keyframe data. Work autonomously within the existing sandbox/workspace while preserving the original data and the working cup demonstration.

## Constraints

- **NO new robot or simulator**. Reuse the existing working robot model, ROS controllers, Gazebo contacts setup, ROS<->Gazebo bridges, and launch stack. Find their actual paths first; do not invent any paths.
- **NO PyBullet/Isaac Sim substitution; NO arbitrary MLP/CNN training**. No training is requested yet.
- **NO software-gripper attachments, fixed joints joining food to tool, teleportation, direct physics pose overrides, or canned motion of ingredients** counted as a successful grasp. Robot joint control is programmed; ingredient motion must come from Gazebo contact/dynamics.
- No claim that MediaPipe pixels represent physical centimeters or 3D hand pose. Do not silently fabricate calibration. Where data are missing, define explicit test assumptions in config and report them. Robot lift Z/grasp height may be designed for the simplified proxies, but must be labeled **assumed**, not reconstructed from human.
- First get a single stable physical episode. Do not repeatedly collect hundreds of rollouts or train an AI policy.
- Record commands vs measured robot movement, collisions, object poses and gripper contact for every attempted stage.
- Work **headless** if needed; avoid rebuilding unrelated workspaces or modifying system packages unless justified and approved.

## Intended human sequence

1. Pick bottom bread from right-hand stack, place on left-hand plate.
2. Pick tomato slice from right-hand plate, place on bottom bread.
3. Pick cheese slice from right-hand plate, place on tomato.
4. Pick top bread from right-hand stack, place on top.

See `data/derived/sandwich_ingredient_events_review.csv` for coarse timing and `sandwich_object_keyframes_review.csv` for **approximate visual pixels**. Identify actual camera coordinate orientation from video and **verify** the left/right references yourself.

## Work in short gated stages

**Stage 0 — read-only checks (do first):** run `scripts/validate_bundle.py`, inspect raw video plus CSV rows and detection quality; locate cup ROS/Gazebo sources, Kinova/Robotiq URDF/Xacro, existing launch/controllers/bridges, camera and contact topics; report disk/memory. Map source video observations to the intended scene; flag occlusions and untrusted coordinates.

**Stage 1 — scene specification & geometry:** create **a separate sandwich scene**, not an edit to the working cup baseline. Use four individually movable rigid food proxies, two shallow trays/plates, table, Kinova robot. Simplified bread, tomato and cheese geometries **thick enough** for the two-finger gripper; sizes, mass, friction, clearances and visual appearance recorded as assumptions. Set source and destination positions reachable with no self-collision. A stacking fixture or shallow support lips are okay if disclosed; physics must still hold pieces together.

**Stage 2 — one bread test:** plan a feasible approach, contact-based two-finger physical grasp, lift, transport, placement and release of bottom bread. Verify joint commands/feedback, both fingertips on correct object, actual item movement caused by dynamics, no table/plate collision and release confirmed. Stop and troubleshoot if this stage fails. Use existing IK/motion-planning packages; do not hardcode a completely unrelated set of joint trajectories.

**Stage 3 — full sequential sandwich:** repeat for tomato, cheese and top bread, maintaining stacking and stable physics. Convert hand/time/approximate target cues into normalized task-space phases. The user video can inform order, travel direction and relative timing; for missing height/pose/contact use documented robot/scene assumptions. Implement reset/retry and explicit failure reporting. Do NOT report a 'human trajectory exact reproduction' where data are not calibrated.

**Stage 4 — evidence-based evaluation:** output short mp4 or saved screenshots, ROS/Gazebo logs/bag, commanded and measured joint/gripper traces, object poses/contact evidence, failure table and whether final stack is physically stable. Run at least three resets at nominal placement to test repeatability; document success count. Perturb initial object positions only after nominal success; no scripted replay may be called learned generalization.

**Stage 5 — next decision, not implementation now:** propose requirements for collecting 30 varied human sandwich demonstrations and training **a pre-trained policy or well-grounded approach** to produce task-level goals transferable across arm types. Explicitly distinguish human-only supervision vs robot-rollout data; retain original source video for future models (GR00T 2 or alternatives). Do not promise that MediaPipe-only data directly fine-tunes GR00T 2.

## Deliverables expected from Codex

1. Actual ROS 2/Gazebo project path and exact launch commands, plus concise `README`.
2. Isolated sandwich-world SDF and robot/food models, controller/bridge config, reproducible test scripts.
3. Simulation recording of at least bottom-bread operation; subsequent four-step run if prior gate passes.
4. CSV/rosbag logs of joint actual vs command, gripper aperture, contact events, individual ingredient world pose, timestamped pass/fail and restart behavior.
5. Brief statement of **what was human-derived**, **what was geometrically assumed**, **what was manually programmed**, **what was physically verified**, and **what remains unproven**.

## Report style

Concise and evidence-based. At each milestone report what passed, failed, blockers and the **single next step**. No long setup lists when existing tooling can be reused.
