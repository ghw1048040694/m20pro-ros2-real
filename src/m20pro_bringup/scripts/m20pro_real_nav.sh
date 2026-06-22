#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_ros2_ws}"
MODE="${1:-safe}"
CLOUD_TOPIC="${M20PRO_CLOUD_TOPIC:-/LIDAR/POINTS}"

set +u
source /opt/robot/scripts/setup_ros2.sh
set -u
cd "${WS_DIR}"
set +u
source install/setup.bash
set -u

COMMON_ARGS=(
  cloud_topic:="${CLOUD_TOPIC}"
  rviz:=false
  enable_web_dashboard:=false
  enable_camera_proxy:=false
  enable_inspection:=false
)

case "${MODE}" in
  safe|shadow)
    exec ros2 launch m20pro_bringup m20pro_real.launch.py \
      "${COMMON_ARGS[@]}" \
      enable_axis_command:=false
    ;;
  move)
    exec ros2 launch m20pro_bringup m20pro_real.launch.py \
      "${COMMON_ARGS[@]}" \
      enable_axis_command:=true
    ;;
  rviz)
    exec ros2 launch m20pro_bringup m20pro_real.launch.py \
      "${COMMON_ARGS[@]}" \
      rviz:=true \
      enable_axis_command:=false
    ;;
  *)
    echo "Usage: $0 [safe|shadow|move|rviz]" >&2
    exit 2
    ;;
esac
