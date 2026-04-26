#!/usr/bin/env bash
set -euo pipefail

INTERFACE="${1:-canable0}"
BITRATE="${CAN_BITRATE:-1000000}"
TX_QUEUE_LEN="${CAN_TXQUEUELEN:-10000}"

run_ip() {
  if [[ "$(id -u)" -eq 0 ]]; then
    ip "$@"
  else
    sudo ip "$@"
  fi
}

echo "[bringup_canable0] Configuring ${INTERFACE}"
run_ip link set "${INTERFACE}" down
run_ip link set "${INTERFACE}" type can bitrate "${BITRATE}"
run_ip link set "${INTERFACE}" txqueuelen "${TX_QUEUE_LEN}"
run_ip link set "${INTERFACE}" up
echo "[bringup_canable0] ${INTERFACE} is up (bitrate=${BITRATE}, txqueuelen=${TX_QUEUE_LEN})"
