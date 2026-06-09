#!/usr/bin/env bash
set -euo pipefail

DURATION_S="${1:-90}"
PREFIX="${2:-m20_real}"
OUT_DIR="${M20PRO_BAG_DIR:-/home/user/bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_PATH="${OUT_DIR}/${PREFIX}_${STAMP}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  set +u
  source /opt/robot/scripts/setup_ros2.sh
  set -u
fi
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  UDP_PROFILE="/home/user/m20pro_ros2_ws/install/m20pro_bringup/share/m20pro_bringup/config/m20pro_fastdds_udp.xml"
  if [[ -f "${UDP_PROFILE}" && "${M20PRO_USE_FACTORY_FASTDDS:-0}" != "1" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${UDP_PROFILE}"
  else
    export FASTRTPS_DEFAULT_PROFILES_FILE="/opt/robot/fastdds.xml"
  fi
fi
mkdir -p "${OUT_DIR}"

if [[ "${EUID}" -ne 0 && "${M20PRO_ALLOW_USER_RECORD:-0}" != "1" ]]; then
  cat >&2 <<'EOF'
m20pro_record_real.sh should be run from the known-good root ROS environment:

  ssh user@10.21.31.104
  source /opt/robot/scripts/setup_ros2.sh
  su
  cd /home/user/m20pro_ros2_ws
  source install/setup.bash
  ros2 run m20pro_bringup m20pro_record_real.sh 90 factory_baseline

Do not restart multicast services from this script. If you intentionally want
to record as user, set M20PRO_ALLOW_USER_RECORD=1.
EOF
  exit 2
fi

echo "[m20pro_record_real] output: ${OUT_PATH}"
if ! timeout 5s ros2 topic list | grep -qx "/LIDAR/POINTS"; then
  echo "[m20pro_record_real] /LIDAR/POINTS is not visible." >&2
  echo "[m20pro_record_real] Use the exact source -> su sequence and do not touch multicast services during the test." >&2
  exit 3
fi

exec timeout "${DURATION_S}" ros2 bag record -o "${OUT_PATH}" \
  /LIDAR/POINTS \
  /LIDAR/POINTS2 \
  /LIDAR/IMU201 \
  /LIDAR/IMU202 \
  /LIDAR/STATUS \
  /IMU \
  /ODOM \
  /scan \
  /tf \
  /tf_static \
  /odom \
  /map \
  /local_costmap/costmap \
  /local_costmap/costmap_updates \
  /global_costmap/costmap \
  /global_costmap/costmap_updates \
  /plan \
  /cmd_vel \
  /m20pro/current_floor \
  /m20pro/stair_status \
  /m20pro/floor_goal \
  /m20pro_tcp_bridge/map_pose \
  /NAV_STATUS \
  /MOTION_STATE \
  /MOTION_STATUS \
  /MOTION_INFO \
  /GAIT \
  /LOCATION_STATUS \
  /LOCATION_STATUS/MATCHING_ERROR \
  /STEER \
  /HANDLE_STEER \
  /BATTERY_DATA \
  /FAULT_STATUS
