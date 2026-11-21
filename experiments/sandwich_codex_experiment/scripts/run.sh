#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
experiment_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$experiment_dir/scripts/run.py" "$@"
