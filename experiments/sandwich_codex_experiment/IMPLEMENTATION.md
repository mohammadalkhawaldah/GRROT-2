# Sandwich experiment — implementation and measured results

The working cup project remains at `/home/mohammad/Documents/Groot2-1`.
This experiment is isolated in `experiments/sandwich_codex_experiment`.
The original package documents and media remain unchanged. No system packages,
training jobs, object attachments, fixed grasp joints, or object-pose commands
were introduced.

## Validated inputs and environment

- All supplied SHA-256 checksums pass. Original video decodes to 1,637 frames,
  1920×1080, 30 FPS. The CSV has 32,067 rows, 1,527 complete 21-point hand groups,
  and 1,476 detected frames (90.16%). All coordinates are finite and timestamps
  agree with frame/30. There are 51 two-hand frames and 395 landmark rows outside
  the image bounds. Detection does not establish accuracy of occluded fingers.
- All four approximate manipulation windows have hand detections in every frame.
  The bread stack hides individual slices; ingredient pickup/release times are
  visual estimates, not verified contact times. The tomato HSV output is unverified.
- Visual review confirms image-right sources and image-left destination. The
  mapping to robot +Y is an explicit simulator choice, not camera calibration.
- Jetson ARM64, ROS 2 Humble, Fortress 6.18.0, gz_ros2_control 0.7.21,
  MoveIt 2.5.10, Kortex description 0.2.3, Robotiq description 0.0.1,
  pick_ik 1.1.2, ros_gz bridge/sim 0.244.26 are already installed. No missing
  dependencies were encountered. Initially about 10 GiB RAM was available and
  143 GiB disk was free. See `outputs/dataset_audit.json` and `bundle_validation.log`.

## Reproduce

From the cup project root:

```bash
bash experiments/sandwich_codex_experiment/scripts/run.sh bread_new_01 --gui --record
bash experiments/sandwich_codex_experiment/scripts/run.sh sandwich_new_01 --full --gui --record
```

Choose an unused run name; existing run directories are never overwritten.
Omit `--gui --record` for a headless test. Each invocation creates a fresh world
and shuts down its own processes afterward. Domain 62 and partition
`sandwich_experiment` keep this separate from cup domain 61.
The launch files can also be started with `ros2 launch` after the runner has
generated the experiment-local robot URDF, but the runner ensures matching models
and records the source/configuration snapshot used for a trial.

## Implementation and assumptions

- `scripts/scene.py`, `worlds/sandwich.sdf`, and `config/assumptions.json` define
  four independent rigid ingredients with box collision proxies and recognizable
  food meshes for rendering, two flat 6 mm plate proxies, and the existing
  table. Bread is 50×70×11 mm, tomato is 50×60×6 mm and cheese is 50×60×5 mm. Their masses are 5.5 g, 2.57 g and 1.71 g respectively.
  Food/support friction is 0.8. Every dimension, mass, friction and world pose is
  assumed. Soft food and deformable grasping are not modeled.
  `scripts/food_visuals.py` generates loaf-shaped bread with crust and porous
  crumb, seeded tomato cut faces and rounded perforated cheese. These visuals
  move with their existing dynamic bodies; see `models/food/README.md` for the
  explicit rendering/physics distinction. No earlier video or result was replaced.
- Two bread pieces begin stacked. The upper source piece is named `bread_bottom`
  because it becomes the sandwich base. The other ingredients begin separately
  on the source plate. The world-fixed robot base has 1 mm mounting clearance
  above the table, avoiding a constant base/table contact stream.
- The installed Kinova/Robotiq geometry, controller configuration, description
  compatibility fixes, bounded joint branch, and native MoveIt/JTC helpers are
  reused. Compatibility generation writes only experiment-local outputs.
- Native OMPL plans the initial validated branch. Collision-checked native
  Cartesian paths handle approach/lift/transport/place/retract with a downward
  tool quaternion. No unrelated arm trajectory is substituted.
- Grasping uses the unchanged gripper position controller and dynamic friction
  contacts. Both **finger-tip links** must contact the selected item recently
  before lifting. An observed lift of at least 70 mm and maintained bilateral
  contact are required before transport.
- During carry, the held ingredient's stale world-fixed **planning proxy** is
  removed; there is no MoveIt attached object. The Gazebo body and all physics
  collisions remain active. An independent sampled box envelope checks payload
  clearance against supports and other foods; native MoveIt checks robot
  collisions. Envelope checks use fixed orientation, 4 mm sampling, 0.5 mm box
  margin, and 2 mm tolerance for initial/support contact. This is a simplified
  clearance check, not a complete swept-volume guarantee for arbitrary motions.
- For thin slices, pickup tool height is 161 mm above the selected slice's
  bottom face for bread and 154 mm for toppings. The tomato and cheese each
  start on a 10 mm high rest, 24 mm wide, which leaves room under their edges
  for the fingertip pads. During placement the controller uses the measured
  grasp offset to release bread 5 mm and toppings 9 mm above their support. The tool travels at 760 mm world height between pickups and placements.
- `config/human_task_cues.json` retains coarse event times and normalized phases.
  Transport duration minima preserve coarse relative carry timing with a safety
  scale and floor; native velocity limits can lengthen them. This is programmed,
  human-guided task reproduction, not an exact trajectory replay or learned policy.

## Acceptance and evidence

Each transfer requires measured endpoint joint/FK agreement, bilateral fingertip
contact, an observed lift, transport without slipping, placement within 15 mm XY
and 8 mm Z, tilt below 0.15 rad, released finger contacts, less than 3 mm settling
motion over three simulated seconds, and no robot/support collision. The complete
stack also checks all final ingredient heights/order, adjacent contacts, and five
seconds of final stability. These are explicit test tolerances, not measurement
accuracy claims.

Each run saves commands, controller desired/actual/error feedback, joint states,
joint-derived gripper aperture estimate, ingredient world poses/quaternions,
contact pairs/positions, actions/results, failures, simulator/planner logs, and
source snapshots. GUI recordings capture the actual Gazebo window on the desktop;
they are not reconstructed animation. The contact bridge's wrench arrays are
empty, so no grip-force measurement or force-control claim is made.
The independent recorded-contact audit checks every robot link, including the
gripper base, against table/plate sensors; only the anchored arm base is exempt.
The ordinary runner includes this audit after execution. The nominal trials were
also independently rechecked against their complete saved contact traces.

Bottom-bread gate: `outputs/bottom_attempt_04/result.json` **passed**. Observed
lift was 146.4 mm; final XY error 0.72 mm; final Z error below 0.001 mm; release
contacts cleared; no arm/table or arm/plate contacts were recorded; three-second
settling position range was zero at the logged precision. Video:
`outputs/bottom_attempt_04/gazebo_recording.mp4`.

Earlier attempts remain as failure evidence: attempts 01/02 physically grasped,
lifted and transported, but a stale planning obstacle blocked descent; attempt 03
exposed a repeated-remove planning-scene update error. These are failures, not
successful complete transfers. Both software issues were corrected before gate 04.

The first full trial (`full_nominal_01`) passed the first three transfers but
failed top-bread settling: downward placement pressure slid the cheese out,
leaving the top bread 35 mm too low. No complete success is claimed for that run.
`top_debug_01` tested the 5 mm gap correction against an initially stacked lower
three ingredients and passed with 0.59 mm top-bread XY error. That initialized
stack diagnostic does **not** count as a four-transfer episode. The corrected
implementation is then tested from the original source layout in three fresh
full trials. All earlier evidence is retained.

`full_nominal_04` was interrupted by signal 15 during bread transport. It has no
completed trial result and its in-memory CSV buffers were lost; partial native
video and ROS/Gazebo logs are retained in that folder with `aborted.json`.
Its remaining owned processes were stopped and the replacement test was launched
through `scripts/detached_job.py`. Interrupted runs are listed separately from
completed physics trials in the summary, and do not count as successful resets.

Full-sequence and repeatability results are recorded in `outputs/experiment_summary.json`
when evaluation finishes. Consult that file for actual counts rather than assuming
the remaining stages passed. Cup preservation hashes are in
`outputs/baseline_preservation.json`.

## Later dataset decision

After repeatable nominal success, evaluate small, documented source-position
perturbations with measured starting poses. Scripted success would demonstrate
engineering robustness, not learned generalization. Before collecting 30 varied
human demonstrations, specify calibrated views/depth or fiducials, ingredient
geometry, task/event annotations, occlusion review, and variation coverage.
Retain original video and distinguish human observations from robot action,
state, contact and outcome data. Selecting a pretrained policy, action convention,
and verified training schema is a later decision; these pixel CSVs alone do not
establish GR00T training compatibility.

A concrete collection plan is ten documented source layouts with three human
takes each, maintaining the four-step order. Record camera calibration and raw
video, measured ingredient/tray geometry, approximate events with reviewer
confidence, occlusions and failed actions. Capture corresponding robot rollouts
separately with synchronized robot-view images, command/state timestamps,
contacts, initial poses and outcomes. Human video supplies visual/task supervision;
robot rollouts supply embodiment-specific actions and execution evidence. Choose
a pretrained policy only after checking its actual input/action schema and compute
requirements; no model or training job is part of this experiment.
