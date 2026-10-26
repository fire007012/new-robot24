#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  generate_dcf.sh [full|arm_only|flipper_only|all]

Profiles:
  full          Generate config/master.dcf from config/master.yaml
  arm_only      Generate config/master_arm_only.dcf from config/master_arm_only.yaml
  flipper_only  Generate config/master_flipper_4axis.dcf from config/master_flipper_4axis.yaml
  all           Generate full, arm_only, and flipper_only profiles
EOF
}

profile="${1:-all}"
if [[ "${profile}" == "-h" || "${profile}" == "--help" ]]; then
  usage
  exit 0
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg_dir="$(cd "${script_dir}/.." && pwd)"
cfg_dir="${pkg_dir}/config"

normalize_upload_paths_relative() {
  local dcf_path="$1"
  python3 - "$dcf_path" <<'PY'
import os
from pathlib import Path
import sys

dcf_path = Path(sys.argv[1]).resolve()
base_dir = dcf_path.parent
lines = dcf_path.read_text().splitlines()
rewritten = []

for line in lines:
    stripped = line.lstrip(" \t")
    indent = line[: len(line) - len(stripped)]
    prefix = "UploadFile="
    if stripped.startswith(prefix):
        value = stripped[len(prefix):].strip()
        path = Path(value)
        if path.is_absolute():
            portable = Path(os.path.relpath(path, base_dir))
        else:
            portable = path

        portable_str = portable.as_posix()
        if portable_str and not portable_str.startswith((".", "..")):
            portable_str = f"./{portable_str}"
        line = f"{indent}{prefix}{portable_str}"
    rewritten.append(line)

dcf_path.write_text("\n".join(rewritten) + ("\n" if rewritten else ""))
PY
}

generate_profile() {
  local profile_name="$1"
  local yaml_name="$2"
  local dcf_name="$3"
  shift 3
  local expected_bins=("$@")
  local tmp_dir
  tmp_dir="$(mktemp -d)"

  echo "[generate_dcf] profile=${profile_name}"
  echo "[generate_dcf] yaml=${cfg_dir}/${yaml_name}"

  (
    cd "${cfg_dir}"
    dcfgen -S -r -d "${tmp_dir}" "${yaml_name}"
  )

  cp "${tmp_dir}/master.dcf" "${cfg_dir}/${dcf_name}"
  normalize_upload_paths_relative "${cfg_dir}/${dcf_name}"
  if [[ -f "${tmp_dir}/master.bin" ]]; then
    cp "${tmp_dir}/master.bin" "${cfg_dir}/master.bin"
  fi

  local bin_name
  for bin_name in "${expected_bins[@]}"; do
    if [[ ! -f "${tmp_dir}/${bin_name}" ]]; then
      echo "[generate_dcf] missing expected output ${bin_name} for ${profile_name}" >&2
      exit 1
    fi
    cp "${tmp_dir}/${bin_name}" "${cfg_dir}/${bin_name}"
  done

  echo "[generate_dcf] wrote ${cfg_dir}/${dcf_name}"
  rm -rf "${tmp_dir}"
}

case "${profile}" in
  full)
    generate_profile full master.yaml master.dcf \
      left_front_arm_joint.bin right_front_arm_joint.bin \
      left_rear_arm_joint.bin right_rear_arm_joint.bin \
      shoulder_yaw_joint.bin shoulder_pitch_joint.bin
    ;;
  arm_only)
    generate_profile arm_only master_arm_only.yaml master_arm_only.dcf \
      shoulder_yaw_joint.bin shoulder_pitch_joint.bin
    ;;
  flipper_only)
    generate_profile flipper_only master_flipper_4axis.yaml master_flipper_4axis.dcf \
      left_front_arm_joint.bin right_front_arm_joint.bin \
      left_rear_arm_joint.bin right_rear_arm_joint.bin
    ;;
  all)
    generate_profile full master.yaml master.dcf \
      left_front_arm_joint.bin right_front_arm_joint.bin \
      left_rear_arm_joint.bin right_rear_arm_joint.bin \
      shoulder_yaw_joint.bin shoulder_pitch_joint.bin
    generate_profile arm_only master_arm_only.yaml master_arm_only.dcf \
      shoulder_yaw_joint.bin shoulder_pitch_joint.bin
    generate_profile flipper_only master_flipper_4axis.yaml master_flipper_4axis.dcf \
      left_front_arm_joint.bin right_front_arm_joint.bin \
      left_rear_arm_joint.bin right_rear_arm_joint.bin
    ;;
  *)
    usage >&2
    exit 1
    ;;
esac
