#!/usr/bin/env bash
# Start a fresh world, run the measured smoke test, and stop only this launch.
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
export IGN_PARTITION="${IGN_PARTITION:-human_cup_drag}"
export ROS_LOG_DIR="$project_dir/outputs/ros_logs"
exec python3 "$project_dir/scripts/verify_headless.py"
