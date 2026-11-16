#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
export IGN_PARTITION="${IGN_PARTITION:-human_cup_drag}"
export ROS_LOG_DIR="$project_dir/outputs/ros_logs"
exec ros2 launch "$project_dir/launch/headless.launch.py" "$@"
