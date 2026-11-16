# Experiment results — 2026-11-15

## Outcome

**PASS: one complete programmed contact baseline from a fresh headless world.**
Kinova Gen3 7-DoF + Robotiq 2F-85 grasped a rigid cup proxy, dragged it along the
table, opened the gripper, and retracted. Both fingertips made measured cup
contact before dragging. No artificial cup attachment or cup pose command was
used. This is a programmed physics feasibility result, not human timing replay,
policy learning, a generalization result or GR00T 2 training.

| Measurement | Latest result |
| --- | ---: |
| Requested +Y drag | 0.300000 m |
| Final cup +Y displacement | 0.299641907 m |
| Final cross-track X displacement | -0.449 mm |
| Maximum cup center height change during drag | 0.571 mm |
| Grasp target position error from measured joint FK | 0.945 mm |
| Gripper close target / achieved | 0.4 / 0.358277 rad |
| Maximum final arm joint error during drag | 0.000992184 rad |
| Maximum final arm joint error after retraction | 0.000055112 rad |
| Finger/cup contacts clear after release/retract | True |
| Measured trace samples | 789 |

These are measurements inside the assumed simulation model. They do not measure
accuracy against a calibrated human world trajectory or establish real-cup
friction, deformation, grip forces or repeatability across starting positions.
Only the final configuration's single complete nominal rollout is reported here.

## Reproduce and inspect

Run `bash scripts/verify_contact.sh` with no other instance in the same domain
and partition. The runner generates the model first, starts a fresh server and
planner, checks that their model descriptions match, runs the trial and stops
only its own processes. The final run exited successfully and its server shut
down cleanly.

- `outputs/contact_trial.json`: measured stage outcomes and contact pairs.
- `outputs/contact_trace.csv`: approximately 20 Hz measured joints and cup poses,
  with simulated timestamps and stage labels.
- `outputs/contact_commands.json`: exact requested trajectories and client-side
  simulation timestamps before sending each request; these are not exact
  controller-acceptance timestamps.
- `outputs/contact_motion.png`: measured translation and center-height plot.
- `outputs/contact_headless.log`, `contact_planning.log`,
  `contact_verification_console.log`: native runtime evidence.

The world-scoped SceneBroadcaster message has an empty header frame string;
its absolute world poses are interpreted in the named cup_drag world. Model FK
and Gazebo fingertip poses were compared during debugging and agreed. These
asynchronous trace snapshots are not yet a synchronized image/action dataset.

## Installed stack and integration choices

73 native packages were added: the initial 71 control/robot/MoveIt dependencies,
then pick_ik and librange-v3-dev. No existing packages were upgraded or removed;
`dpkg --audit` was clean. No CUDA, Torch, Python environment or unrelated project
was rebuilt. Exact versions and sources are in docs/dependency_manifest.json.

The model reuses installed vendor meshes, kinematics and inertias. Only local
compatibility copies remove two optional macro arguments missing from the
packaged Robotiq description. Measured state interfaces are exposed for all five
mimic joints. See ASSUMPTIONS.md and outputs/robot_manifest.json.

The vendor external-cable option supplies +/-2pi revolute bounds for arm joints
1, 3, 5 and 7. A tested pregrasp joint target fixes the arm branch; MoveIt still
collision-checks the route on every run. Local pick_ik interpolation with a
displacement penalty handles approach, drag and retraction. pick_ik currently
receives basic upstream maintenance: https://github.com/PickNikRobotics/pick_ik.

Object closure has an explicit 0.08 rad goal tolerance because cup contact stops
the driver before its empty-space angle; recent bilateral fingertip contact is
independently required. Opening retains 0.03 rad tolerance. Arm goals require
0.03 rad error or less, with independent measured endpoint checks. This is
position-servo contact, not measured or regulated grip force.

Earlier failed trials are retained. The checks caught a sideways quaternion
target, interference between the cup and the gripper base at too-low approach
height, a planner/model startup mismatch, continuous-angle retraction error, and
a poor arm branch near a joint limit. The final target height is 0.615 m; the
pregrasp/retraction height is 0.75 m. No incomplete Cartesian path is executed.

The control-only smoke test separately passed in multiple fresh worlds before
the bounded-joint update. The final contact rollout exercised the updated arm,
gripper and pose/contact feedback. Earlier observed real-time factor was about
0.29–0.37; the simulator runs slower than wall-clock real time. About 9.5 GiB RAM
remained available during contact testing.

## Human data and approximate reference

All five supplied source files were copied locally and SHA256-verified after the
Seagate drive was reconnected. See outputs/demo_validation.json. The video is
1920 x 1080, 30 fps, 14.3 s. Both trajectory tables have 427 rows; landmarks have
5,586 rows in 266 complete hand/frame groups covering 253 detected frames. All
human grip/contact labels remain unknown.

`data/derived/tabletop_reference.csv` uses 52 Drag-labelled cup samples with
score >= 0.7, a five-sample median, endpoint-direction projection, monotonic
progress and normalization to 0.30 m along world +Y. Relative source timestamps
are retained. All post-release tracking, including the 9.5 s jump, is excluded
from motion estimation. Gripper angles, downward orientation and vertical
approach/retraction are robot-specific assumptions. There is no homography or
recovered human 6DoF pose. Provenance and normalization are recorded in
`data/derived/reference_metadata.json`.

The successful contact trial above uses a programmed 10 s drag, not the human
1.7 s drag timing/profile. The human-derived target reference has not yet driven
a physical rollout.

## Remaining work

1. Drive the verified contact sequence from the human-derived reference with a
   documented global time scale that respects robot velocity/acceleration limits;
   compare the logged cup path and timing to that reference.
2. Add headless robot-camera observations and synchronized state/action/object
   recording with calibration/frame metadata. Validate one complete serialized
   episode before collecting more.
3. Evaluate repeated episodes and perturbed cup starts, preserving failure logs
   and collecting only successful rollouts as clean demonstrations.
4. Validate against official GR00T 2 tools/schema/hardware requirements separately.
   No compatibility, checkpoint, training result or learned closed-loop skill is
   established by this experiment.

## Visible real-time validation

The complete native GUI trial passed after 1 ms controller initialization followed
by a runtime change to a 4 ms physics step via Gazebo set_physics. Moving RTF
was 0.9954–1.0081 (mean 1.0001). Final +Y cup travel was 0.299522673 m.
The original 1 ms success artifacts are preserved in outputs/successful_baseline.
Direct startup at 4 or 5 ms failed the initial gripper goal; these configurations
were rejected. No motion/contact tolerances were relaxed for this speed change.
See outputs/realtime_validation.json, realtime_stats_motion.json and
visible_demo_result.json. GUI mode now uses the validated warm-start configuration;
the original world file and headless defaults remain at 1 ms.
