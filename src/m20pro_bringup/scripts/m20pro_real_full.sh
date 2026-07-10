#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
MODE="${1:-shadow}"

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
)
if [[ -n "${SELECTED_MAP_YAML}" ]]; then
  COMMON_ARGS+=(map:="${SELECTED_MAP_YAML}")
fi

BASE_REAL_PARAMS="${WS_DIR}/install/m20pro_bringup/share/m20pro_bringup/config/m20pro_real.yaml"
make_runtime_params() {
  local axis_enabled="$1"
  local output
  output="$(mktemp "/tmp/m20pro_real_params_${MODE}.XXXXXX.yaml")"
  python3 - "${BASE_REAL_PARAMS}" "${output}" "${axis_enabled}" <<'PY'
import sys
import yaml

src, dst, axis_text = sys.argv[1:4]
with open(src, "r", encoding="utf-8") as file:
    data = yaml.safe_load(file) or {}

bridge = data.setdefault("m20pro_tcp_bridge", {}).setdefault("ros__parameters", {})
bridge["enable_axis_command"] = axis_text.lower() in ("1", "true", "yes", "on")
bridge["enable_initialpose_relocalization"] = True
bridge["enable_initialpose_3d_relocalization"] = False

with open(dst, "w", encoding="utf-8") as file:
    yaml.safe_dump(data, file, allow_unicode=True, sort_keys=False)
PY
  echo "${output}"
}

case "${MODE}" in
  shadow|safe)
    RUNTIME_PARAMS="$(make_runtime_params false)"
    exec ros2 launch m20pro_bringup m20pro.launch.py \
      "${COMMON_ARGS[@]}" \
      real_params_file:="${RUNTIME_PARAMS}" \
      enable_axis_command:=false
    ;;
  move)
    RUNTIME_PARAMS="$(make_runtime_params true)"
    exec ros2 launch m20pro_bringup m20pro.launch.py \
      "${COMMON_ARGS[@]}" \
      real_params_file:="${RUNTIME_PARAMS}" \
      enable_axis_command:=true
    ;;
  *)
    echo "Usage: $0 [shadow|move]" >&2
    exit 2
    ;;
esac
