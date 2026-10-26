#!/usr/bin/env bash
# No system installation: stage official deb contents in the workspace vendor root.
set -eo pipefail
robot_source_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
robot_sdk_cache="$(mktemp -d /tmp/robot24-camera-sdk-XXXXXX)"
trap 'rm -rf "$robot_sdk_cache"' EXIT
(cd "$robot_sdk_cache" && apt-get download ros-humble-librealsense2=2.58.4-1jammy.20260830.174155)
mkdir -p "$robot_source_root/.deps/librealsense"
for robot_deb in "$robot_sdk_cache"/*.deb; do
  dpkg-deb -x "$robot_deb" "$robot_source_root/.deps/librealsense"
done
