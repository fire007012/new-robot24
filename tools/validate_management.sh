#!/usr/bin/env bash
# Unit suites also run in core CI; ROS integration requires the full install.
set -eo pipefail
robot_source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
robot_prefix="${1:?usage: validate_management.sh OUTPUT_PREFIX core|full}"
robot_mode="${2:-core}"
export PYTHONDONTWRITEBYTECODE=1
python3 -m unittest discover -s "$robot_source_root/host_bridge/tests" -v 2>&1 | tee "${robot_prefix}-bridge-unit.log"
python3 -m unittest discover -s "$robot_source_root/robot_manager/tests" -v 2>&1 | tee "${robot_prefix}-manager-unit.log"
if [[ "$robot_mode" == full ]]; then
  source "${robot_prefix}-install/setup.bash"
  ROS_DOMAIN_ID=96 ROS_LOCALHOST_ONLY=1 python3 "$robot_source_root/host_bridge/tests/ros_p0_mock.py" > "${robot_prefix}-p0-ros.json"
  python3 "$robot_source_root/robot_manager/tests/integration.py" \
    --setup "${robot_prefix}-install/setup.bash" \
    --output "${robot_prefix}-manager-config" --rounds 2 --config-transactions
  python3 "$robot_source_root/robot_manager/tests/integration.py" \
    --setup "${robot_prefix}-install/setup.bash" \
    --output "${robot_prefix}-manager-recovery" --rounds 2 --optional-video --restart-during-start
  python3 "$robot_source_root/robot_manager/tests/integration.py" \
    --setup "${robot_prefix}-install/setup.bash" \
    --output "${robot_prefix}-manager-running" --rounds 1 --exercise-running
fi
