#!/usr/bin/env bash
set -euo pipefail

# The production service runs as root while field builds run as user. Avoid
# root-owned __pycache__ files inside the symlink install tree blocking rebuilds.
export PYTHONDONTWRITEBYTECODE=1

WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
MODE="${1:-shadow}"
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

if [[ "${EUID}" -ne 0 ]]; then
  cat >&2 <<'EOF'
m20pro_real_full.sh must be run after the known-good 104 root sequence:

  ssh user@10.21.31.104
  source /opt/robot/scripts/setup_ros2.sh
  su
  cd /home/user/m20pro_real_ros2_ws
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
BRINGUP_PREFIX="$(ros2 pkg prefix m20pro_bringup)"

PROJECT_FASTDDS="${WS_DIR}/install/m20pro_bringup/share/m20pro_bringup/config/m20pro_fastdds_udp.xml"
FACTORY_FASTDDS="/opt/robot/fastdds.xml"

cleanup_stale_fastdds_shm() {
  if [[ "${M20PRO_CLEAN_STALE_FASTDDS_SHM:-1}" != "1" ]]; then
    return
  fi
  local stale=0
  local kept=0
  local file
  shopt -s nullglob
  for file in /dev/shm/fastrtps_*; do
    if fuser "${file}" >/dev/null 2>&1; then
      kept=$((kept + 1))
    else
      rm -f -- "${file}" && stale=$((stale + 1))
    fi
  done
  shopt -u nullglob
  if [[ "${stale}" -gt 0 || "${kept}" -gt 0 ]]; then
    echo "[m20pro_real_full] stale FastDDS SHM cleanup: removed=${stale} kept_open=${kept}" >&2
  fi
}

cleanup_stale_fastdds_shm

case "${M20PRO_FASTDDS_PROFILE:-factory}" in
  project_udp|udp)
    if [[ -f "${PROJECT_FASTDDS}" ]]; then
      export FASTRTPS_DEFAULT_PROFILES_FILE="${PROJECT_FASTDDS}"
    elif [[ -f "${FACTORY_FASTDDS}" ]]; then
      export FASTRTPS_DEFAULT_PROFILES_FILE="${FACTORY_FASTDDS}"
    fi
    ;;
  factory)
    if [[ -f "${FACTORY_FASTDDS}" ]]; then
      export FASTRTPS_DEFAULT_PROFILES_FILE="${FACTORY_FASTDDS}"
    fi
    ;;
  auto)
    if [[ "${M20PRO_USE_PROJECT_FASTDDS:-1}" == "1" && -f "${PROJECT_FASTDDS}" ]]; then
      export FASTRTPS_DEFAULT_PROFILES_FILE="${PROJECT_FASTDDS}"
    elif [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
      :
    elif [[ -f "${FACTORY_FASTDDS}" ]]; then
      export FASTRTPS_DEFAULT_PROFILES_FILE="${FACTORY_FASTDDS}"
    fi
    ;;
  inherit)
    ;;
  *)
    echo "[m20pro_real_full] invalid M20PRO_FASTDDS_PROFILE=${M20PRO_FASTDDS_PROFILE}; expected project_udp, factory, auto, or inherit" >&2
    exit 2
    ;;
esac

if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "[m20pro_real_full] FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}" >&2
fi

if ps -eo pid,args | awk '
  /ros2 launch m20pro_bringup m20pro.launch.py/ &&
  /mode:=real/ &&
  !/awk/ {print}
' >/tmp/m20pro_real_existing_stack.out && [[ -s /tmp/m20pro_real_existing_stack.out ]]; then
  cat >&2 <<'EOF'
[m20pro_real_full] another M20Pro real launch is already running.

Do not start multiple real stacks on 104. Stop the existing stack first:

  ./scripts/104_stop_real.sh
  # or
  systemctl stop m20pro-real.service
EOF
  cat /tmp/m20pro_real_existing_stack.out >&2 || true
  exit 70
fi

SCAN_TOPIC="${M20PRO_SCAN_TOPIC:-/scan}"
WEB_DASHBOARD_DATA_DIR="${M20PRO_WEB_DASHBOARD_DATA_DIR:-/home/user/.m20pro_web}"
WEB_DASHBOARD_MAP_ARCHIVE_DIR="${M20PRO_WEB_DASHBOARD_MAP_ARCHIVE_DIR:-/home/user/m20pro_maps}"
INSPECTION_MODEL_PATH="${M20PRO_INSPECTION_MODEL_PATH:-${WS_DIR}/install/m20pro_inspection/share/m20pro_inspection/models/best_rk3588_fp16.rknn}"
INSPECTION_CLASS_NAMES_PATH="${M20PRO_INSPECTION_CLASS_NAMES_PATH:-${WS_DIR}/install/m20pro_inspection/share/m20pro_inspection/models/labels_zh.txt}"
echo "[m20pro_real_full] perception=/scan from 106 edge scan" >&2

selected_map_yaml_for_launch() {
  python3 - "${WEB_DASHBOARD_DATA_DIR}" <<'PY'
import json
import os
import sys

data_dir = sys.argv[1]
settings_path = os.path.join(data_dir, "settings.json")
maps_path = os.path.join(data_dir, "maps.json")

try:
    with open(settings_path, "r", encoding="utf-8") as file:
        settings = json.load(file) or {}
        selected_map_id = str(settings.get("selected_map_id") or "").strip()
        working_map_id = str(settings.get("working_map_id") or "").strip()
except Exception:
    selected_map_id = ""
    working_map_id = ""

launch_map_ids = []
for value in (selected_map_id, working_map_id):
    if value and value not in launch_map_ids:
        launch_map_ids.append(value)
if not launch_map_ids:
    sys.exit(0)

try:
    with open(maps_path, "r", encoding="utf-8") as file:
        records = json.load(file) or []
except Exception:
    records = []

if not isinstance(records, list):
    records = []

for launch_map_id in launch_map_ids:
    for record in records:
        if str(record.get("id") or "").strip() != launch_map_id:
            continue
        yaml_path = os.path.expandvars(os.path.expanduser(str(record.get("yaml_path") or "").strip()))
        if yaml_path and os.path.exists(yaml_path):
            print(yaml_path)
            sys.exit(0)
        break
PY
}

SELECTED_MAP_YAML="$(selected_map_yaml_for_launch || true)"
if [[ -n "${SELECTED_MAP_YAML}" ]]; then
  echo "[m20pro_real_full] initial Nav2 map from web working map: ${SELECTED_MAP_YAML}" >&2
fi

COMMON_ARGS=(
  mode:=real
  rviz:=false
  enable_web_dashboard:=true
  enable_initialpose_relocalization:=true
  web_dashboard_data_dir:="${WEB_DASHBOARD_DATA_DIR}"
  web_dashboard_map_archive_dir:="${WEB_DASHBOARD_MAP_ARCHIVE_DIR}"
  enable_camera_proxy:=false
  camera_proxy_backend:=ffmpeg_mjpeg
  camera_proxy_fps:=10.0
  camera_proxy_jpeg_quality:=45
  camera_proxy_ffmpeg_mjpeg_qscale:=5
  camera_proxy_max_width:=480
  enable_inspection:="${M20PRO_ENABLE_INSPECTION:-false}"
  inspection_backend:="${M20PRO_INSPECTION_BACKEND:-rknn}"
  inspection_source_type:="${M20PRO_INSPECTION_SOURCE_TYPE:-rtsp}"
  inspection_rtsp_url:="${M20PRO_INSPECTION_RTSP_URL:-rtsp://10.21.31.103:8554/video1}"
  inspection_camera_name:="${M20PRO_INSPECTION_CAMERA_NAME:-front_wide}"
  inspection_model_path:="${INSPECTION_MODEL_PATH}"
  inspection_class_names_path:="${INSPECTION_CLASS_NAMES_PATH}"
  scan_topic:="${SCAN_TOPIC}"
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
if [[ -n "${SELECTED_MAP_YAML}" ]]; then
  COMMON_ARGS+=(map:="${SELECTED_MAP_YAML}")
fi

case "${MODE}" in
  shadow|safe)
    AXIS_ENABLED=false
    ;;
  move)
    AXIS_ENABLED=true
    ;;
  *)
    echo "Usage: $0 [shadow|move]" >&2
    exit 2
    ;;
esac

FIELD_PROFILE="${BRINGUP_PREFIX}/share/m20pro_bringup/config/m20pro_field_profile.yaml"
BASE_REAL_PARAMS="${BRINGUP_PREFIX}/share/m20pro_bringup/config/m20pro_real.yaml"
BASE_NAV2_PARAMS="${BRINGUP_PREFIX}/share/m20pro_bringup/config/nav2_params_real.yaml"
PROFILE_TOOL="${WS_DIR}/scripts/m20pro_field_profile.py"
RUNTIME_PARAMS=""
RUNTIME_NAV2_PARAMS=""
cleanup_runtime_params() {
  [[ -z "${RUNTIME_PARAMS}" ]] || rm -f -- "${RUNTIME_PARAMS}"
  [[ -z "${RUNTIME_NAV2_PARAMS}" ]] || rm -f -- "${RUNTIME_NAV2_PARAMS}"
}
trap cleanup_runtime_params EXIT

RUNTIME_PARAMS="$(mktemp "/tmp/m20pro_real_params_${MODE}.XXXXXX.yaml")"
RUNTIME_NAV2_PARAMS="$(mktemp "/tmp/m20pro_nav2_params_${MODE}.XXXXXX.yaml")"
"${PROFILE_TOOL}" render-real-yaml \
  --profile "${FIELD_PROFILE}" \
  --input "${BASE_REAL_PARAMS}" \
  --output "${RUNTIME_PARAMS}" \
  --axis-enabled "${AXIS_ENABLED}"
"${PROFILE_TOOL}" render-nav2-yaml \
  --profile "${FIELD_PROFILE}" \
  --input "${BASE_NAV2_PARAMS}" \
  --output "${RUNTIME_NAV2_PARAMS}"

ros2 launch m20pro_bringup m20pro.launch.py \
  "${COMMON_ARGS[@]}" \
  real_params_file:="${RUNTIME_PARAMS}" \
  real_nav2_params_file:="${RUNTIME_NAV2_PARAMS}" \
  enable_axis_command:="${AXIS_ENABLED}" \
  enable_stair_connector:="${AXIS_ENABLED}"
