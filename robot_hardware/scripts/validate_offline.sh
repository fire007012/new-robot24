#!/usr/bin/env bash
set -eo pipefail
robot_source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
robot_validation_prefix="${1:-/tmp/robot-canopen-offline}"
source /opt/ros/humble/setup.bash
colcon --log-base "${robot_validation_prefix}-log" build \
  --base-paths "$robot_source_root/ros2_canopen" "$robot_source_root/robot_canopen_config" \
               "$robot_source_root/robot_safety" "$robot_source_root/can_driver" "$robot_source_root/robot_hardware" \
  --packages-up-to robot_hardware --packages-ignore lely_core_libraries \
  --build-base "${robot_validation_prefix}-build" --install-base "${robot_validation_prefix}-install" \
  --cmake-args -DBUILD_TESTING=ON
source "${robot_validation_prefix}-install/setup.bash"
python3 "$robot_source_root/robot_hardware/scripts/audit_config.py" "${robot_validation_prefix}-install"
export ROS_LOG_DIR="${robot_validation_prefix}-roslogs"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=90
for robot_test_package in robot_safety can_driver canopen_core canopen_base_driver canopen_proxy_driver canopen_402_driver canopen_master_driver canopen_ros2_control robot_canopen_config robot_hardware; do
 ctest --test-dir "${robot_validation_prefix}-build/$robot_test_package" --output-on-failure --timeout 60
done
ROBOT_OFFLINE_REPORT="${robot_validation_prefix}-hybrid.log" python3 "$robot_source_root/robot_hardware/test/hybrid_smoke.py"
ROBOT_OFFLINE_REPORT="${robot_validation_prefix}-groups.log" python3 "$robot_source_root/robot_hardware/test/group_smoke.py"
bash "$robot_source_root/robot_hardware/test/relocation_smoke.sh" "${robot_validation_prefix}-install"
colcon test-result --test-result-base "${robot_validation_prefix}-build" --verbose
python3 "$robot_source_root/robot_hardware/scripts/offline_manifest.py" "$robot_source_root" "${robot_validation_prefix}-install" "${robot_validation_prefix}-report.json"
