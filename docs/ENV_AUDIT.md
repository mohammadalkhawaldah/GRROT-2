# Environment audit — 2026-11-15

Read-only audit preceded installation. Host: NVIDIA Jetson Orin NX Seeed
reComputer classic Super; Ubuntu 22.04.5 LTS, aarch64, JetPack 6.2.1,
L4T 36.4.3. `free -h`: 15 GiB RAM, approximately 10 GiB available at audit.
`df -h /`: 145 GiB available on the NVMe root filesystem.

`/opt/ros/humble` exists. Sourcing its setup makes `ros2` available and sets
ROS_DISTRO=humble. The initial shell had no sourced ROS environment.
`ign gazebo --versions` returned 6.18.0 (Fortress).
ARM64 `ros_gz` 0.244.26 and bridge/image/sim/demos were already installed.
`robot_state_publisher` and `xacro` were already installed.
MoveIt, ros2_control controller manager, gz_ros2_control, manipulator descriptions,
and manipulation controllers were absent at audit.

APT's existing signed ROS index (InRelease dated October 12) has ARM64 candidates
for the missing packages. A dependency simulation resolved 71 new packages,
zero upgrades and zero removals. See `dependency_manifest.json` for exact
versions, source URLs, architectures, byte sizes and SHA256 checksums.
The subsequent implementation also installed pick_ik and librange-v3-dev
(two additional packages, zero upgrades/removals). The final manifest has 73
entries; its control and contact results are in EXPERIMENTS.md.
No APT source configuration, CUDA, Torch or existing workspace was changed.
See `../outputs/package_install.log` for the installation result.

Existing Scout mobile robot and ROS2-SWARM workspaces were found and left alone.
No existing arm/gripper simulation stack was found in the inspected projects.
The project directory was initially empty.

## Human source data

No named source video, hand-landmark CSV, cup trajectory CSV or joined phase CSV
from the handoff was found in user project/data directories under /home/mohammad
(environment/cache directories were excluded). No external data volume was
identified in /media/mohammad. This is not a claim about every filesystem path.

Other project CSVs exist in /home/mohammad/telio_humi:

| File | Data rows | Schema |
| --- | ---: | --- |
| human_demo_01.csv | 1401 | timestamp, hand_x/y/z, rot_x/y/z, gripper_width |
| training_transitions.csv | 106046 | episode_id, timestamp, dt, hand pose, target motion increments |
| vision_data/all/vision_motion_manifest.csv | 17631 | episode/split/frame/time, image path, pose, gripper, target dx/dy/dz |
| vision_data/episode89/vision_motion.csv | 115 | image, time, pose, gripper, target dx/dy/dz |

Their provenance, units, calibration and relationship to this recording are
unverified. They are not used as the cup-drag source.

### Supplied external-drive paths — subsequent check

The user supplied five paths in
`/media/mohammad/Seagate Expansion Drive/files-to-jetson`.
The directory was initially readable, and file metadata showed a 2,928,745-byte
source MKV and a 256,977-byte landmark CSV. Reading the CSV returned errno 5
(Input/output error). The drive then disconnected and all five supplied paths
became unavailable. Kernel logs at 22:54 on November 15 show repeated USB resets,
descriptor error -110, read I/O errors, failure to read the partition table,
and reported capacity changing to zero. No filesystem repair, format or write
to this drive was attempted. These symptoms establish an unavailable USB
device; they do not identify whether the cause is power, cable, port or media.

The user subsequently reconnected the drive. All five supplied files were then
copied to project-local `data/raw` and SHA256-verified. The source MKV probes as
1920 x 1080, 30 fps, duration 14.3 s. The hand-preview MP4 has 427 frames.
The landmark CSV has 5,586 rows, 253 detected frames, 266 hand/frame groups and
no incomplete 21-landmark groups. Both joined phase and rough-scale CSVs have
427 rows over 0–14.2 s. All joined gripper_state_verified values are `unknown`.
The original drive files were not modified. See `../outputs/demo_validation.json`.

## Supported stack evidence

- Humble/Fortress pairing: https://gazebosim.org/docs/fortress/ros_installation/
- Control plugin: https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html
- Kinova simulation: https://github.com/Kinovarobotics/ros2_kortex/blob/humble/README.md

The Kinova Humble documentation describes Fortress simulation and Robotiq mimic
joint issues. Package availability alone does not establish working contact.
Runtime evidence is recorded separately in outputs/smoke_test.json.
