#!/usr/bin/env bash
# 一键自启动部署入口（未在当前非最终环境自动安装；目标车端确认后执行 --install）。
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
AUTOSTART_PY="$WS_ROOT/src/robot_manager/autostart.py"

if [[ $# -eq 0 ]]; then
    cat >&2 <<'EOF'
用法：
  bash src/tools/install_autostart.sh plan                    # 仅检查依赖并预览目标路径（不改系统）
  sudo bash src/tools/install_autostart.sh install           # 在目标下位机安装 systemd 开机自启服务
  sudo bash src/tools/install_autostart.sh install --with-can # 同时安装 CAN 口 udev/systemd 自动配置
  sudo bash src/tools/install_autostart.sh uninstall         # 卸载服务（先检查受管资源已停止）
EOF
    exit 2
fi

ACTION="$1"
shift
if [[ "$ACTION" == "install" || "$ACTION" == "uninstall" ]]; then
    HAS_PREFIX=0
    for arg in "$@"; do
        if [[ "$arg" == "--root-prefix" || "$arg" == --root-prefix=* ]]; then
            HAS_PREFIX=1
        fi
    done
    if [[ "$HAS_PREFIX" -eq 0 && "${EUID}" -ne 0 ]]; then
        exec sudo /usr/bin/python3 -B "$AUTOSTART_PY" "$ACTION" --ws-root "$WS_ROOT" "$@"
    fi
fi

exec /usr/bin/python3 -B "$AUTOSTART_PY" "$ACTION" --ws-root "$WS_ROOT" "$@"
