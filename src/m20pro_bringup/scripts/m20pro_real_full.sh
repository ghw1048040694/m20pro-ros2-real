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
BRINGUP_LIBEXEC="${BRINGUP_PREFIX}/lib/m20pro_bringup"

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

relay_fastdds_profile_file() {
  case "${M20PRO_LIDAR_RELAY_FASTDDS_PROFILE:-factory}" in
    factory)
      if [[ -f "${FACTORY_FASTDDS}" ]]; then
        echo "${FACTORY_FASTDDS}"
      fi
      ;;
    project_udp|udp)
      if [[ -f "${PROJECT_FASTDDS}" ]]; then
        echo "${PROJECT_FASTDDS}"
      elif [[ -f "${FACTORY_FASTDDS}" ]]; then
        echo "${FACTORY_FASTDDS}"
      fi
      ;;
    inherit)
      echo "${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
      ;;
    *)
      echo "[m20pro_real_full] invalid M20PRO_LIDAR_RELAY_FASTDDS_PROFILE=${M20PRO_LIDAR_RELAY_FASTDDS_PROFILE}; expected factory, project_udp, or inherit" >&2
      exit 2
      ;;
  esac
}

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

if [[ "${M20PRO_RUN_RAW_LIDAR_GUARD:-0}" == "1" ]]; then
  set +e
  "${BRINGUP_LIBEXEC}/m20pro_lidar_guard.sh" startup
  status="$?"
  set -e
  if [[ "${status}" -ne 0 ]]; then
    if [[ "${M20PRO_LIDAR_GUARD_MODE:-warn}" == "strict" ]]; then
      if [[ "${status}" -eq 75 ]]; then
        echo "[m20pro_real_full] lidar samples are not ready; strict startup is intentionally skipped." >&2
      fi
      exit "${status}"
    fi
    echo "[m20pro_real_full] lidar guard returned ${status}; continuing so the workstation web frontend stays available." >&2
  fi
fi

LIDAR_RELAY_TOPIC="${M20PRO_LIDAR_RELAY_TOPIC:-/m20pro/lidar_points_relay}"
LIDAR2_INPUT_TOPIC="${M20PRO_LIDAR2_TOPIC:-/LIDAR/POINTS2}"
LIDAR2_RELAY_TOPIC="${M20PRO_LIDAR2_RELAY_TOPIC:-/m20pro/lidar_points2_relay}"
SCAN_SOURCE="${M20PRO_SCAN_SOURCE:-local_fusion}"
EDGE_SCAN_TOPIC="${M20PRO_EDGE_SCAN_TOPIC:-/scan}"
SCAN_TOPIC="${M20PRO_SCAN_TOPIC:-/scan}"
WEB_DASHBOARD_DATA_DIR="${M20PRO_WEB_DASHBOARD_DATA_DIR:-/home/user/.m20pro_web}"
WEB_DASHBOARD_MAP_ARCHIVE_DIR="${M20PRO_WEB_DASHBOARD_MAP_ARCHIVE_DIR:-/home/user/m20pro_maps}"
case "${SCAN_SOURCE}" in
  local_fusion)
    PERCEPTION_MODE="local_fusion"
    ENABLE_FUSION="true"
    ENABLE_LIDAR_POINTS_SUBSCRIPTIONS="true"
    WEB_CLOUD_TOPIC="${LIDAR_RELAY_TOPIC}"
    ;;
  edge_scan)
    PERCEPTION_MODE="edge_scan"
    ENABLE_FUSION="false"
    ENABLE_LIDAR_POINTS_SUBSCRIPTIONS="false"
    SCAN_TOPIC="${EDGE_SCAN_TOPIC}"
    WEB_CLOUD_TOPIC=""
    ;;
  *)
    echo "[m20pro_real_full] invalid M20PRO_SCAN_SOURCE=${SCAN_SOURCE}; expected local_fusion or edge_scan" >&2
    exit 2
    ;;
esac
echo "[m20pro_real_full] scan_source=${SCAN_SOURCE} scan_topic=${SCAN_TOPIC} perception_mode=${PERCEPTION_MODE}" >&2

LIDAR_RELAY_FASTDDS="$(relay_fastdds_profile_file)"
if [[ "${SCAN_SOURCE}" == "local_fusion" && -n "${LIDAR_RELAY_FASTDDS}" ]]; then
  echo "[m20pro_real_full] LIDAR relay FASTRTPS_DEFAULT_PROFILES_FILE=${LIDAR_RELAY_FASTDDS}" >&2
fi
if [[ "${SCAN_SOURCE}" == "local_fusion" ]]; then
  set +e
  FASTRTPS_DEFAULT_PROFILES_FILE="${LIDAR_RELAY_FASTDDS}" \
    "${BRINGUP_LIBEXEC}/m20pro_lidar_relay_guard.sh" start
  relay_status="$?"
  set -e
  if [[ "${relay_status}" -ne 0 ]]; then
    if [[ "${M20PRO_LIDAR_GUARD_MODE:-warn}" == "strict" ]]; then
      echo "[m20pro_real_full] lidar relay is not ready; strict startup is intentionally skipped." >&2
      exit "${relay_status}"
    fi
    echo "[m20pro_real_full] lidar relay returned ${relay_status}; continuing so the workstation web frontend stays available." >&2
  fi
else
  echo "[m20pro_real_full] edge_scan mode: stopping local lidar relays and skipping pointcloud_fusion" >&2
  FASTRTPS_DEFAULT_PROFILES_FILE="${LIDAR_RELAY_FASTDDS}" \
    "${BRINGUP_LIBEXEC}/m20pro_lidar_relay_guard.sh" stop || true
  M20PRO_LIDAR_RELAY_PID_FILE="${M20PRO_LIDAR2_RELAY_PID_FILE:-/tmp/m20pro_lidar_relay2.pid}" \
    M20PRO_LIDAR_TOPIC="${LIDAR2_INPUT_TOPIC}" \
    M20PRO_LIDAR_RELAY_TOPIC="${LIDAR2_RELAY_TOPIC}" \
    "${BRINGUP_LIBEXEC}/m20pro_lidar_relay_guard.sh" stop || true
fi

BACKUP_CLOUD_TOPIC=""
if [[ "${SCAN_SOURCE}" == "local_fusion" && "${M20PRO_ENABLE_LIDAR2_RELAY:-0}" == "1" && -n "${LIDAR2_INPUT_TOPIC}" && -n "${LIDAR2_RELAY_TOPIC}" ]]; then
  set +e
  FASTRTPS_DEFAULT_PROFILES_FILE="${LIDAR_RELAY_FASTDDS}" \
    M20PRO_LIDAR_TOPIC="${LIDAR2_INPUT_TOPIC}" \
    M20PRO_LIDAR_RELAY_TOPIC="${LIDAR2_RELAY_TOPIC}" \
    M20PRO_LIDAR_RELAY_STATUS_TOPIC="${M20PRO_LIDAR2_RELAY_STATUS_TOPIC:-/m20pro/lidar_relay2/status}" \
    M20PRO_LIDAR_RELAY_PID_FILE="${M20PRO_LIDAR2_RELAY_PID_FILE:-/tmp/m20pro_lidar_relay2.pid}" \
    M20PRO_LIDAR_RELAY_LOG_FILE="${M20PRO_LIDAR2_RELAY_LOG_FILE:-/tmp/m20pro_lidar_relay2.log}" \
    "${BRINGUP_LIBEXEC}/m20pro_lidar_relay_guard.sh" start
  relay2_status="$?"
  set -e
  if [[ "${relay2_status}" -eq 0 ]]; then
    BACKUP_CLOUD_TOPIC="${LIDAR2_RELAY_TOPIC}"
    echo "[m20pro_real_full] optional LIDAR2 relay started: ${LIDAR2_INPUT_TOPIC} -> ${LIDAR2_RELAY_TOPIC}" >&2
  else
    echo "[m20pro_real_full] optional LIDAR2 relay not ready (${LIDAR2_INPUT_TOPIC}); continuing with primary lidar only." >&2
    M20PRO_LIDAR_RELAY_PID_FILE="${M20PRO_LIDAR2_RELAY_PID_FILE:-/tmp/m20pro_lidar_relay2.pid}" \
      M20PRO_LIDAR_TOPIC="${LIDAR2_INPUT_TOPIC}" \
      M20PRO_LIDAR_RELAY_TOPIC="${LIDAR2_RELAY_TOPIC}" \
      "${BRINGUP_LIBEXEC}/m20pro_lidar_relay_guard.sh" stop || true
  fi
fi

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
        selected_map_id = str((json.load(file) or {}).get("selected_map_id") or "").strip()
except Exception:
    selected_map_id = ""

if not selected_map_id:
    sys.exit(0)

try:
    with open(maps_path, "r", encoding="utf-8") as file:
        records = json.load(file) or []
except Exception:
    records = []

if not isinstance(records, list):
    records = []

for record in records:
    if str(record.get("id") or "").strip() != selected_map_id:
        continue
    yaml_path = os.path.expandvars(os.path.expanduser(str(record.get("yaml_path") or "").strip()))
    if yaml_path and os.path.exists(yaml_path):
        print(yaml_path)
    break
PY
}

SELECTED_MAP_YAML="$(selected_map_yaml_for_launch || true)"
if [[ -n "${SELECTED_MAP_YAML}" ]]; then
  echo "[m20pro_real_full] initial Nav2 map from selected web map: ${SELECTED_MAP_YAML}" >&2
fi

COMMON_ARGS=(
  mode:=real
  rviz:=false
  enable_web_dashboard:=true
  enable_initialpose_relocalization:=true
  web_dashboard_data_dir:="${WEB_DASHBOARD_DATA_DIR}"
  web_dashboard_map_archive_dir:="${WEB_DASHBOARD_MAP_ARCHIVE_DIR}"
  enable_camera_proxy:=true
  camera_proxy_backend:=ffmpeg_mjpeg
  camera_proxy_fps:=10.0
  camera_proxy_jpeg_quality:=45
  camera_proxy_ffmpeg_mjpeg_qscale:=5
  camera_proxy_max_width:=480
  scan_topic:="${SCAN_TOPIC}"
  perception_mode:="${PERCEPTION_MODE}"
  fusion:="${ENABLE_FUSION}"
  enable_lidar_points_subscriptions:="${ENABLE_LIDAR_POINTS_SUBSCRIPTIONS}"
)
if [[ -n "${SELECTED_MAP_YAML}" ]]; then
  COMMON_ARGS+=(map:="${SELECTED_MAP_YAML}")
fi
if [[ -n "${WEB_CLOUD_TOPIC}" ]]; then
  COMMON_ARGS+=(cloud_topic:="${WEB_CLOUD_TOPIC}")
fi
if [[ -n "${BACKUP_CLOUD_TOPIC}" ]]; then
  COMMON_ARGS+=(backup_cloud_topic:="${BACKUP_CLOUD_TOPIC}")
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
