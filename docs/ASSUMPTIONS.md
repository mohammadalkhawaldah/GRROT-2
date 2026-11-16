# Simulation assumptions and boundaries

- Existing Kinova Gen3 7-DoF and Robotiq 2F-85 assets come from official ROS ARM64
  packages. Their meshes, kinematics, inertias and collision geometry are reused.
- `scripts/prepare_robot.py` generates a local URDF; installed vendor files stay
  untouched. It mounts the base at world z=0.4 m, uses the current
  gz_ros2_control plugin names, supplies a 100 Hz controller configuration, and
  omits rendering sensors during the initial control smoke test.
- Local compatibility copies omit two optional arguments absent from the
  packaged Robotiq macro. Measured position/velocity interfaces are added to
  its five coupled joints; gz_ros2_control reports these names with `_mimic`.
  Both arm and gripper use installed joint trajectory controllers. The gripper
  starts and reopens at 0.1 rad, away from the exact joint limit; 0.4 rad is the
  free-space close target. These are joint angles, not measured aperture.
- The vendor `use_external_cable=true` option bounds arm joints 1, 3, 5 and 7 as
  revolute joints within +/-2pi. This simulation choice avoids continuous-angle
  normalization during the planner/controller handoff; no robot geometry is
  invented. Every arm joint has an explicit controller goal tolerance of 0.03 rad.
- Object closure allows 0.08 rad driver error on that goal only, since cup
  contact prevents reaching the empty-space 0.4 rad angle. Both fingertips must
  show recent cup contact before a drag command. Opening retains 0.03 rad
  tolerance. This is a position-servo contact approximation, not force control.
- Table top is world z=0.4 m; table dimensions are 1.6 x 1.2 x 0.4 m.
- The cup is a solid rigid cylinder proxy. Radius 0.025 m follows the reported
  base diameter; height 0.1 m, mass 0.02 kg and friction coefficient 0.4 are
  assumptions. Cylinder inertia is computed from those assumed dimensions/mass.
  This does not model a hollow, tapered or deformable paper cup.
- Cup initial center is (0.55, 0, 0.451) m, settling under gravity onto the table.
- ROS_DOMAIN_ID defaults to 61 and IGN_PARTITION to human_cup_drag to keep this
  experiment separate from other ROS/Gazebo projects. Both can be overridden.
- No fixed joint, attach service, pose teleport, fake hardware or kinematic cup
  following is used. The arm is anchored to the world as a fixed-base robot.
- Joint commands and finger angles are robot commands, not recovered human
  gripper aperture or contact measurements.
- Smoke-test finger motion is not a physical grasp test. The five human source
  files are now imported and validated. The approximate reference does not
  establish executed human retargeting. No expert image/action episode dataset,
  learning result or GR00T 2 compatibility is established.
