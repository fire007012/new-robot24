#!/usr/bin/env bash
# Run from an ordinary local shell: integration tests need loopback DDS sockets.
set -eo pipefail
robot_source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
robot_prefix="${1:-/tmp/robot24-ros2}"
robot_mode="${2:-core}"
bash "$robot_source_root/tools/build_ros2.sh" "$robot_prefix" "$robot_mode"
source "${robot_prefix}-install/setup.bash"
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${robot_prefix}-roslogs"
for robot_package in robot_safety can_driver canopen_core canopen_base_driver canopen_proxy_driver canopen_402_driver canopen_master_driver canopen_ros2_control robot_canopen_config robot_hardware arm_control; do
  ctest --test-dir "${robot_prefix}-build/$robot_package" --output-on-failure --timeout 60
done
ROBOT_OFFLINE_REPORT="${robot_prefix}-control.log" python3 "$robot_source_root/robot_bringup/test/full_system_smoke.py"
ROBOT_OFFLINE_REPORT="${robot_prefix}-moveit.log" python3 "$robot_source_root/robot_bringup/test/moveit_smoke.py"

if [[ "$robot_mode" == full ]]; then
  python3 "$robot_source_root/car_control/test/servo_smoke.py"
  QT_QPA_PLATFORM=offscreen python3 "$robot_source_root/car_control/test/ui_smoke.py"
  python3 "$robot_source_root/vision_pkg/test/pipeline_smoke.py"
  python3 "$robot_source_root/tools/test/host_smoke.py"
  python3 "$robot_source_root/tools/test/sensor_smoke.py"
  timeout 45s python3 "$robot_source_root/eye_in_hand/test/calibration_smoke.py"
  # Fortress software rendering uses an X display; EGL can be configured for a
  # supported GPU separately. Do not count a skipped simulation as acceptance.
  python3 "$robot_source_root/robot_simulation/test/simulation_smoke.py"
fi
