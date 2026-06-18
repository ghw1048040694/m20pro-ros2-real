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
M20PRO_LIDAR_WAIT_S="${TIMEOUT_S}" ros2 run m20pro_bringup m20pro_lidar_guard.sh check
