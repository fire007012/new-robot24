#!/usr/bin/env bash
# Called as root by robot-can@.service; no ROS environment is required.
set -euo pipefail

INTERFACE="${1:?Usage: configure_can.sh can0|can1}"
case "${INTERFACE}" in
  can0|can1) ;;
  *) echo "Unsupported CAN interface: ${INTERFACE}" >&2; exit 1 ;;
esac
[[ "$(cat "/sys/class/net/${INTERFACE}/type")" == 280 ]] || {
  echo "${INTERFACE} is not a CAN interface" >&2
  exit 1
}

ip link set dev "${INTERFACE}" down
ip link set dev "${INTERFACE}" type can bitrate 1000000
ip link set dev "${INTERFACE}" txqueuelen 1000
ip link set dev "${INTERFACE}" up
echo "${INTERFACE}: configured bitrate=1000000 txqueuelen=1000, interface up"
