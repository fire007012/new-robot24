#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"${SCRIPT_DIR}/bringup_canable0.sh" canable0
"${SCRIPT_DIR}/bringup_canable1.sh" canable1
