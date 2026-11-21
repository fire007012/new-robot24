#!/usr/bin/env bash
# Install once on the robot computer. Subsequent insertions need no terminal.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "${SCRIPT_DIR}")"
if [[ "${EUID}" -ne 0 ]]; then
  exec sudo bash "${BASH_SOURCE[0]}" "$@"
fi
if [[ "$(ps -p 1 -o comm=)" != systemd ]]; then
  echo 'Run this installer on the robot computer running systemd.' >&2
  exit 1
fi
for command in ip udevadm systemctl; do
  command -v "${command}" >/dev/null || {
    echo "Required command not found: ${command}" >&2
    exit 1
  }
done

install -D -m 0755 "${SCRIPT_DIR}/configure_can.sh" /usr/local/libexec/robot-can/configure_can.sh
install -D -m 0644 "${PACKAGE_DIR}/systemd/robot-can@.service" /etc/systemd/system/robot-can@.service
install -D -m 0644 "${PACKAGE_DIR}/udev/80-robot-can.rules" /etc/udev/rules.d/80-robot-can.rules
systemctl daemon-reload
udevadm control --reload-rules

# Configure idle interfaces already plugged in. Do not interrupt an UP bus.
for interface in can0 can1; do
  if [[ -f "/sys/class/net/${interface}/flags" ]]; then
    flags="$(cat "/sys/class/net/${interface}/flags")"
    if (( flags & 1 )); then
      echo "${interface} is already UP; keeping its current settings. Replug to apply automatically."
    else
      systemctl start "robot-can@${interface}.service"
    fi
  fi
done
echo 'Installed: can0/can1 will automatically use 1 Mbps and txqueuelen 1000 on insertion and boot.'
echo 'Inspect with: ip -details link show can0'
echo 'Logs: journalctl -u robot-can@can0.service -u robot-can@can1.service -b'
