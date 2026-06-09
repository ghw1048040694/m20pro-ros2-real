#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_ros2_ws}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this after su on 104. This script enables motion control." >&2
  exit 2
fi

set +u
source /opt/robot/scripts/setup_ros2.sh
set -u

cd "${WS_DIR}"
set +u
source install/setup.bash
set -u

exec ros2 run m20pro_bringup m20pro_real_full.sh move
