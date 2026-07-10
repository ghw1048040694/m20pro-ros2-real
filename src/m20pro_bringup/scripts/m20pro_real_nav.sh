#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
MODE="${1:-safe}"
RADAR_ENABLED="${M20PRO_ENABLE_RADAR_INSPECTION:-false}"
RADAR_BACKEND="${M20PRO_RADAR_BACKEND:-dry_run}"
RADAR_SCAN_MODE="${M20PRO_RADAR_SCAN_MODE:-measuring}"
RADAR_DEVICE_URL="${M20PRO_RADAR_DEVICE_URL:-http://192.168.107.72:8080}"
RADAR_OUTPUT_DIR="${M20PRO_RADAR_OUTPUT_DIR:-/home/user/m20pro_radar_results}"
RADAR_RELEASE_ON_ANALYSIS="${M20PRO_RADAR_RELEASE_ON_ANALYSIS:-true}"
RADAR_START_RETRY_TIMEOUT_S="${M20PRO_RADAR_START_RETRY_TIMEOUT_S:-120.0}"
RADAR_START_RETRY_INTERVAL_S="${M20PRO_RADAR_START_RETRY_INTERVAL_S:-5.0}"
RADAR_RESULT_RETRY_COUNT="${M20PRO_RADAR_RESULT_RETRY_COUNT:-5}"
RADAR_RESULT_RETRY_INTERVAL_S="${M20PRO_RADAR_RESULT_RETRY_INTERVAL_S:-2.0}"
RADAR_QUERY_ERROR_TIMEOUT_S="${M20PRO_RADAR_QUERY_ERROR_TIMEOUT_S:-120.0}"

set +u
source /opt/robot/scripts/setup_ros2.sh
set -u
cd "${WS_DIR}"
set +u
source install/setup.bash
set -u

COMMON_ARGS=(
  mode:=real
  rviz:=false
  enable_web_dashboard:=false
  enable_camera_proxy:=false
  enable_inspection:=false
  enable_radar_inspection:="${RADAR_ENABLED}"
  radar_backend:="${RADAR_BACKEND}"
  radar_scan_mode:="${RADAR_SCAN_MODE}"
  radar_device_url:="${RADAR_DEVICE_URL}"
  radar_output_dir:="${RADAR_OUTPUT_DIR}"
  radar_release_on_analysis:="${RADAR_RELEASE_ON_ANALYSIS}"
  radar_start_retry_timeout_s:="${RADAR_START_RETRY_TIMEOUT_S}"
  radar_start_retry_interval_s:="${RADAR_START_RETRY_INTERVAL_S}"
  radar_result_retry_count:="${RADAR_RESULT_RETRY_COUNT}"
  radar_result_retry_interval_s:="${RADAR_RESULT_RETRY_INTERVAL_S}"
  radar_query_error_timeout_s:="${RADAR_QUERY_ERROR_TIMEOUT_S}"
)

case "${MODE}" in
  safe|shadow)
    exec ros2 launch m20pro_bringup m20pro.launch.py \
      "${COMMON_ARGS[@]}" \
      enable_axis_command:=false
    ;;
  move)
    exec ros2 launch m20pro_bringup m20pro.launch.py \
      "${COMMON_ARGS[@]}" \
      enable_axis_command:=true
    ;;
  rviz)
    exec ros2 launch m20pro_bringup m20pro.launch.py \
      "${COMMON_ARGS[@]}" \
      rviz:=true \
      enable_axis_command:=false
    ;;
  *)
    echo "Usage: $0 [safe|shadow|move|rviz]" >&2
    exit 2
    ;;
esac
