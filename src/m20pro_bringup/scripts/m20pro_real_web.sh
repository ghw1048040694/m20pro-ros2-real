#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_ros2_ws}"
PORT="${1:-8080}"

set +u
source /opt/robot/scripts/setup_ros2.sh
set -u
cd "${WS_DIR}"
set +u
source install/setup.bash
set -u

exec ros2 launch m20pro_bringup m20pro_web_dashboard.launch.py \
  port:="${PORT}" \
  enable_camera_proxy:=false
