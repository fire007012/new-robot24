#!/usr/bin/env bash
# Explicit package roots select the supported ROS2 dependency set.
set -eo pipefail
robot_source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
robot_prefix="${1:-/tmp/robot24-ros2}"
source /opt/ros/humble/setup.bash
robot_sdk_prefix="$robot_source_root/.deps/librealsense/opt/ros/humble"
if [[ -d "$robot_sdk_prefix" ]]; then
  export CMAKE_PREFIX_PATH="$robot_sdk_prefix:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$robot_sdk_prefix/lib:$robot_sdk_prefix/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi
robot_packages=(ros2_canopen robot_safety can_driver robot_canopen_config robot_hardware car_urdf car_moveit_config mobility_control arm_control car_control flipper_control robot_bringup)
robot_build_target=robot_bringup
if [[ "${2:-core}" == full ]]; then
  robot_packages+=(robot_simulation robot_full_system robot_camera_sdk vision_pkg eye_in_hand host_bridge senseair_co2 thermal_camera realsense-ros OrbbecSDK_ROS2 aruco_ros easy_handeye2)
  robot_build_target=robot_full_system
fi
robot_paths=()
for robot_package in "${robot_packages[@]}"; do
  robot_paths+=("$robot_source_root/$robot_package")
done
colcon --log-base "${robot_prefix}-log" build --base-paths "${robot_paths[@]}" \
  --packages-up-to "$robot_build_target" --packages-ignore lely_core_libraries \
  --build-base "${robot_prefix}-build" --install-base "${robot_prefix}-install" \
  --parallel-workers 3 --cmake-args -DBUILD_TESTING=ON -DINSTALL_UDEV_RULES=OFF
