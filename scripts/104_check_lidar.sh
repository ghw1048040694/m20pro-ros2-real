#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_ros2_ws}"
TIMEOUT_S="${1:-8}"

set +u
source /opt/robot/scripts/setup_ros2.sh
set -u

cd "${WS_DIR}"
if [[ -f install/setup.bash ]]; then
  set +u
  source install/setup.bash
  set -u
fi

echo "[104_check_lidar] key topics:"
ros2 topic list | grep -E '^/(LIDAR/POINTS|LIDAR/POINTS2|scan|ODOM|IMU)$' || true

echo "[104_check_lidar] waiting /LIDAR/POINTS for ${TIMEOUT_S}s"
if timeout "${TIMEOUT_S}" ros2 topic echo /LIDAR/POINTS --no-arr; then
  exit 0
fi

echo "[104_check_lidar] no /LIDAR/POINTS data within ${TIMEOUT_S}s" >&2
exit 3
