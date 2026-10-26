#!/usr/bin/env bash
# Ubuntu 22.04 + ROS2 Humble; upstream driver SDKs are handled separately.
set -eo pipefail
robot_dependencies=(
  build-essential cmake git pkg-config autoconf automake libtool
  python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pytest
  ffmpeg python3-pyqt5 python3-yaml python3-numpy python3-opencv python3-matplotlib python3-transforms3d
  libyaml-cpp-dev libspdlog-dev libboost-system-dev libeigen3-dev libopencv-dev
  libgflags-dev nlohmann-json3-dev libgoogle-glog-dev libdw-dev
  libusb-1.0-0-dev libudev-dev libssl-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav can-utils
  ros-humble-ament-cmake ros-humble-ament-cmake-python
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest
  ros-humble-ament-lint-auto ros-humble-ament-lint-common
  ros-humble-rosidl-default-generators ros-humble-ros2-control ros-humble-ros2-controllers
  ros-humble-lely-core-libraries
  ros-humble-moveit ros-humble-moveit-servo ros-humble-ros-gz ros-humble-ign-ros2-control
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
  ros-humble-joy ros-humble-rviz2 ros-humble-cv-bridge ros-humble-image-transport
  ros-humble-image-publisher ros-humble-image-geometry ros-humble-camera-info-manager
  ros-humble-tf2-eigen ros-humble-tf2-sensor-msgs ros-humble-diagnostic-updater
  ros-humble-backward-ros ros-humble-rqt-gui-py ros-humble-ruckig
)
if [[ "${1:-}" == '--dry-run' ]]; then
  exec apt-get --simulate install "${robot_dependencies[@]}"
fi
if [[ $# -ne 0 ]]; then
  echo 'Usage: install_ros2_dependencies.sh [--dry-run]' >&2
  exit 2
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo 'This dependency list targets Ubuntu 22.04 / ROS2 Humble.' >&2
  exit 1
fi
robot_sudo=()
if [[ $EUID -ne 0 ]]; then
  robot_sudo=(sudo)
fi
"${robot_sudo[@]}" apt-get update
"${robot_sudo[@]}" apt-get install -y "${robot_dependencies[@]}"
printf '%s\n' 'Common dependencies installed. Pinned RealSense driver 4.58.1 uses SDK 2.58.4; prepare it with bash tools/prepare_camera_sdk.sh before a full build.'
