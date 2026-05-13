#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SEC="${1:-30}"
DEADLINE=$((SECONDS + TIMEOUT_SEC))

REQUIRED_TOPICS=(
  "/gazebo/model_states"
  "/wheel_controller/cmd_vel"
  "/flipper_control/state"
  "/move_group/status"
  "/arm_position_controller/follow_joint_trajectory/goal"
  "/gripper_controller/command"
  "/joint_states"
)

while (( SECONDS < DEADLINE )); do
  TOPICS="$(rostopic list 2>/dev/null || true)"
  MISSING=()

  for t in "${REQUIRED_TOPICS[@]}"; do
    if ! grep -Fxq "$t" <<<"$TOPICS"; then
      MISSING+=("$t")
    fi
  done

  if (( ${#MISSING[@]} == 0 )); then
    echo "[wait_for_graph] PASS: all required topics/actions are available."
    exit 0
  fi

  sleep 1
done

echo "[wait_for_graph] FAIL: timeout after ${TIMEOUT_SEC}s. Missing entries:"
for t in "${MISSING[@]}"; do
  echo "  - $t"
done
exit 1
