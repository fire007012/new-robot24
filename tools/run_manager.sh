#!/usr/bin/env bash
# 保留终端启动方式，内置崩溃退避自恢复与熔断；不依赖外部中间目录。
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"

RUNTIME_DIR="${ROBOT24_RUNTIME_DIR:-$HOME/.local/share/robot24}"
INSTALL_DIR="${ROBOT24_INSTALL_DIR:-$WS_ROOT/install}"
MANAGER_PY="${ROBOT24_MANAGER_PY:-$WS_ROOT/src/robot_manager/manager.py}"
PREPARE_PY="$WS_ROOT/src/tools/prepare_runtime.py"

SUPERVISE=1
if [[ "${1:-}" == "--no-supervise" ]]; then
    SUPERVISE=0
    shift
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_HOME="$RUNTIME_DIR/ros-home"
export ROS_LOG_DIR="$RUNTIME_DIR/ros-logs"

mkdir -p "$RUNTIME_DIR" "$ROS_HOME" "$ROS_LOG_DIR"
chmod 700 "$RUNTIME_DIR" || true

/usr/bin/python3 -B "$PREPARE_PY" \
    --ws-root "$WS_ROOT" \
    --install "$INSTALL_DIR" \
    --runtime "$RUNTIME_DIR" >/dev/null

CONFIG_PATH="$RUNTIME_DIR/manager.json"
SUPERVISOR_LOG="$RUNTIME_DIR/supervisor.log"

if [[ "$SUPERVISE" -eq 0 ]]; then
    exec /usr/bin/python3 -B "$MANAGER_PY" --config "$CONFIG_PATH" "$@"
fi

STOP_REQUESTED=0
CHILD_PID=""

forward_stop() {
    STOP_REQUESTED=1
    if [[ -n "$CHILD_PID" ]] && kill -0 "$CHILD_PID" 2>/dev/null; then
        kill -TERM "$CHILD_PID" 2>/dev/null || true
    fi
}
trap forward_stop INT TERM

CRASH_TIMES=()
MAX_CRASHES_IN_WINDOW=5
WINDOW_SECONDS=120
BACKOFF=2

while true; do
    set +e
    /usr/bin/python3 -B "$MANAGER_PY" --config "$CONFIG_PATH" "$@" &
    CHILD_PID=$!
    wait "$CHILD_PID"
    rc=$?
    set -e
    CHILD_PID=""

    if [[ "$STOP_REQUESTED" -eq 1 || "$rc" -eq 0 ]]; then
        exit 0
    fi

    now="$(date +%s)"
    printf '[%s] manager exited abnormally (code=%s)\n' "$(date -Iseconds)" "$rc" >> "$SUPERVISOR_LOG"
    echo "[supervisor] manager 异常退出 (code=$rc)，正在检查自恢复条件..." >&2

    FILTERED=()
    for t in "${CRASH_TIMES[@]}"; do
        if (( now - t <= WINDOW_SECONDS )); then
            FILTERED+=("$t")
        fi
    done
    FILTERED+=("$now")
    CRASH_TIMES=("${FILTERED[@]}")

    if (( ${#CRASH_TIMES[@]} >= MAX_CRASHES_IN_WINDOW )); then
        msg="[supervisor] ${WINDOW_SECONDS}s 内连续异常退出 ${#CRASH_TIMES[@]} 次，触发熔断停止自拉起；请检查 $SUPERVISOR_LOG 与 $RUNTIME_DIR/state/state.json"
        printf '[%s] %s\n' "$(date -Iseconds)" "$msg" >> "$SUPERVISOR_LOG"
        echo "$msg" >&2
        exit "$rc"
    fi

    echo "[supervisor] ${BACKOFF}s 后自动重启 manager（重启后将自动进入状态对账/恢复模式）..." >&2
    sleep "$BACKOFF" &
    SLEEP_PID=$!
    wait "$SLEEP_PID" || true
    if [[ "$STOP_REQUESTED" -eq 1 ]]; then
        exit 0
    fi
    if (( BACKOFF < 16 )); then
        BACKOFF=$(( BACKOFF * 2 ))
    fi
done
