#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
export IGN_PARTITION="${IGN_PARTITION:-human_cup_drag}"
export ROS_LOG_DIR="$project_dir/outputs/ros_logs"
export CUP_PHYSICS_STEP_S="${CUP_PHYSICS_STEP_S:-0.001}"
export CUP_RUNTIME_STEP_S="${CUP_RUNTIME_STEP_S-0.004}"
exec python3 "$project_dir/scripts/visible_demo.py" "$@"
