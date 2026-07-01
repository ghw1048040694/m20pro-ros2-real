#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
DURATION_S="${1:-180}"
PREFIX="${2:-m20_real}"

set +u
source /opt/robot/scripts/setup_ros2.sh
set -u

cd "${WS_DIR}"
set +u
source install/setup.bash
set -u

exec ros2 run m20pro_bringup m20pro_record_real.sh "${DURATION_S}" "${PREFIX}"
