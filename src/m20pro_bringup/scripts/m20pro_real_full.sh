#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_ros2_ws}"
MODE="${1:-shadow}"

if [[ "${EUID}" -ne 0 ]]; then
  cat >&2 <<'EOF'
m20pro_real_full.sh must be run after the known-good 104 root sequence:

  ssh user@10.21.31.104
  source /opt/robot/scripts/setup_ros2.sh
  su
  cd /home/user/m20pro_ros2_ws
  source install/setup.bash
  ros2 run m20pro_bringup m20pro_real_full.sh shadow

Use "move" only when the site is safe and motion control is allowed.
EOF
  exit 2
fi

if [[ -z "${ROS_DISTRO:-}" || ! -x "$(command -v ros2)" ]]; then
  set +u
  source /opt/robot/scripts/setup_ros2.sh
  set -u
fi

cd "${WS_DIR}"
set +u
source install/setup.bash
set -u

DEFAULT_FASTDDS="${WS_DIR}/install/m20pro_bringup/share/m20pro_bringup/config/m20pro_fastdds_udp.xml"
if [[ -f "${DEFAULT_FASTDDS}" && "${M20PRO_USE_FACTORY_FASTDDS:-0}" != "1" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${DEFAULT_FASTDDS}"
else
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/robot/fastdds.xml}"
fi

COMMON_ARGS=(
  mode:=real
  rviz:=false
  enable_web_dashboard:=true
  web_dashboard_data_dir:=/home/user/.m20pro_web
  web_dashboard_map_archive_dir:=/home/user/m20pro_maps
  enable_camera_proxy:=true
  camera_proxy_fps:=2.0
  camera_proxy_jpeg_quality:=45
  camera_proxy_max_width:=480
  cloud_topic:=/LIDAR/POINTS
)

case "${MODE}" in
  shadow|safe)
    exec ros2 launch m20pro_bringup m20pro.launch.py \
      "${COMMON_ARGS[@]}" \
      enable_axis_command:=false
    ;;
  move)
    exec ros2 launch m20pro_bringup m20pro.launch.py \
      "${COMMON_ARGS[@]}" \
      enable_axis_command:=true
    ;;
  *)
    echo "Usage: $0 [shadow|move]" >&2
    exit 2
    ;;
esac
