#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_SETUP_DEFAULT="${WS_ROOT}/devel/setup.bash"
ROS_SETUP_PATH="${ROS_SETUP:-${ROS_SETUP_DEFAULT}}"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "ERROR: gnome-terminal not found." >&2
  exit 1
fi

if [[ ! -f "${ROS_SETUP_PATH}" ]]; then
  echo "ERROR: ROS setup file not found: ${ROS_SETUP_PATH}" >&2
  echo "Set ROS_SETUP=/path/to/setup.bash or build the workspace first." >&2
  exit 1
fi

run_component() {
  local title="$1"
  local launch_file="$2"
  local extra_args="${3:-}"

  local cmd="source \"${ROS_SETUP_PATH}\" && roslaunch robot_bringup \"${launch_file}\""
  if [[ -n "${extra_args}" ]]; then
    cmd+=" ${extra_args}"
  fi
  cmd+="; exec bash"

  gnome-terminal --title="${title}" -- bash -lc "${cmd}" &
}

if (( $# > 0 )); then
  echo "usage: $0" >&2
  echo "optional env vars: HARDWARE_ARGS CONTROL_ARGS MOVEIT_ARGS VISION_ARGS ROS_SETUP" >&2
  exit 2
fi

run_component "Robot24 Hardware" "full_system_hardware.launch" "${HARDWARE_ARGS:-}"
sleep 0.5
run_component "Robot24 Control" "full_system_control.launch" "${CONTROL_ARGS:-}"
sleep 0.5
run_component "Robot24 MoveIt" "full_system_moveit.launch" "${MOVEIT_ARGS:-}"
sleep 0.5
run_component "Robot24 Vision" "full_system_vision.launch" "${VISION_ARGS:-}"

wait
