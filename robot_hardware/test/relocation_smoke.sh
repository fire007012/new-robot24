#!/usr/bin/env bash
# Validate a copied install while the original prefix is unavailable.
set -eo pipefail
robot_relocation_source="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
robot_original_prefix="$(realpath "$1")"
robot_hidden_prefix="${robot_original_prefix}-relocation-hold"
if [[ -e "$robot_hidden_prefix" ]]; then
  echo 'another relocation is already in progress' >&2; exit 1
fi
robot_relocated_prefix="$(mktemp -d "${robot_original_prefix}-relocated-XXXXXX")"
echo "Relocated install: $robot_relocated_prefix"
cp -a "$robot_original_prefix/." "$robot_relocated_prefix/"
mv "$robot_original_prefix" "$robot_hidden_prefix"
trap 'mv "$robot_hidden_prefix" "$robot_original_prefix"' EXIT
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u LD_LIBRARY_PATH -u PYTHONPATH \
  bash -s -- "$robot_relocated_prefix" "$robot_relocation_source" <<'SCRIPT'
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$1/local_setup.bash"
python3 "$2/robot_hardware/scripts/audit_config.py" "$1"
export ROS_LOCALHOST_ONLY=1
export ROBOT_OFFLINE_REPORT="$1-single-axis.log"
python3 "$2/robot_hardware/test/hybrid_smoke.py" SingleAxisSmoke
export ROBOT_OFFLINE_REPORT="$1-groups.log"
python3 "$2/robot_hardware/test/group_smoke.py"
SCRIPT
