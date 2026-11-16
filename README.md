# Human cup drag in ROS 2 / Gazebo

For the current accomplishments, verified results, limitations, and roadmap, see
[Project status and next steps](PROJECT_STATUS_AND_NEXT_STEPS.md).

Isolated Jetson experiment using the installed ROS 2 Humble / Gazebo Fortress
stack, Kinova Gen3 7-DoF, and Robotiq 2F-85. Direct launch requires no colcon
build and does not source other project overlays.

The native control smoke test has passed. See `outputs/smoke_test.json` for
measured errors, five coupled-finger checks and cup/table contact evidence.
The five human-source files have been copied locally and validated. A labelled
approximate target reference is available in `data/derived`; it has not been
executed or validated as a physical grasp/drag.

The complete **programmed contact baseline passed** from a fresh world: measured
cup travel was 0.299642 m, with maximum center-height variation 0.571 mm during
drag, followed by opening and retraction. This uses a rigid cup proxy and a
programmed 10 s drag, with measured bilateral fingertip contact and no attach
constraint. See `docs/EXPERIMENTS.md`, `outputs/contact_trial.json` and
`outputs/contact_motion.png`. It is not human timing replay or a learned skill.

## Run the headless scene

```bash
cd /home/mohammad/Documents/Groot2-1
bash scripts/run_headless.sh
```

To watch the complete test in the native Gazebo window on the Jetson desktop:

```bash
bash scripts/run_visible.sh
```

This opens the scene, waits 10 seconds, then runs grasp/drag/release/retract.
The window stays open at the end; closing it stops this demo's server and planner.
The earlier successful measurements are preserved in `outputs/successful_baseline`;
the visible run result is saved as `outputs/visible_demo_result.json`.
Visible mode initializes at 1 ms, then switches to a validated 4 ms physics step
after the gripper is active. The full GUI trial passed with measured real-time
factor 0.995–1.008 (mean 1.000). This changes simulation resolution; commanded
robot motions still use the deliberate baseline durations. The original 1 ms
world and headless verification remain available. To keep 1 ms throughout a
visible run, use `CUP_RUNTIME_STEP_S='' bash scripts/run_visible.sh`.

In another terminal:

```bash
cd /home/mohammad/Documents/Groot2-1
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=61 IGN_PARTITION=human_cup_drag
python3 scripts/smoke_test.py
```

The bounded smoke test checks advancing simulated time, active controllers,
measured arm tracking, gripper close/open, opposite-finger movement and cup pose
availability. It writes `outputs/smoke_test.json` and exits nonzero on failure.
The scene remains running until Ctrl+C. Start a new server for a fresh world.
This test does not perform a cup grasp, drag, human replay or learning.

For an automated fresh-world test (with no other instance of this scene running):

```bash
bash scripts/verify_headless.sh
```

To import and validate the five supplied human demonstration files once the
external drive is readable:

```bash
python3 scripts/import_demo.py --source '/media/mohammad/Seagate Expansion Drive/files-to-jetson'
```

This preserves the originals, verifies local copies by SHA256 and checks CSV
schemas, timestamps, complete landmark groups and video metadata. The report is
`outputs/demo_validation.json`; missing or unreadable files cause failure.

## Planning and contact feasibility

With a fresh headless scene running, start the native planner in another terminal:

```bash
CUP_IK_SOLVER=pick_ik bash scripts/run_planning.sh
```

Then, in a sourced Humble shell with `ROS_DOMAIN_ID=61`:

```bash
python3 scripts/plan_approach.py  # Plan only; no motion
python3 scripts/contact_trial.py  # Programmed contact feasibility trial
```

The trial collision-checks table/cup geometry, commands a pregrasp and descent,
requires bilateral finger contact before dragging, and records measured robot/cup
traces. It is a programmed baseline; it does not reproduce the human timing.
Check `outputs/contact_trial.json` for the actual outcome, including failures.
The KDL configuration remains available by omitting `CUP_IK_SOLVER`. The current
contact configuration uses `pick_ik` with a displacement penalty. This packaged solver
currently receives basic upstream maintenance rather than feature development.
See https://github.com/PickNikRobotics/pick_ik.

For a complete fresh-world trial that owns and stops its own processes, with no
other instance running in the same domain/partition:

```bash
bash scripts/verify_contact.sh
```

Raw joint/cup traces are in `outputs/contact_trace.csv`; exact requested
trajectories and client simulation timestamps are in `outputs/contact_commands.json`.
These logs do not yet include robot-camera images or an expert dataset schema.

## Files and evidence

- `docs/ENV_AUDIT.md`: original environment and source-data findings.
- `docs/ASSUMPTIONS.md`: proxy dimensions, integration changes and limitations.
- `docs/dependency_manifest.json`: exact official package sources and checksums.
- `outputs/package_install.log`: native package installation log.
- `outputs/robot.urdf`, `robot_manifest.json`: generated simulation model.
- `outputs/headless.log`, `smoke_test.json`: runtime evidence when generated.

`scripts/fetch_dependencies.py` caches and verifies dependencies using the local
APT index. It does not install packages. Native package installation requires
sudo; no credential is stored by project scripts.

## Next gates

1. Use the validated human reference to drive the verified contact sequence with
   an explicit safe time scale, and measure error against that reference.
2. Record synchronized images, robot states/actions, object poses and success
   evidence. Rendering/camera acquisition is a later gate, not enabled initially.
3. Evaluate repeated and perturbed starts and keep clean successful episodes.
4. Assess official GR00T 2 dataset/training requirements separately when available.
