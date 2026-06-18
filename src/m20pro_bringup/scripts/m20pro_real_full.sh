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
BRINGUP_PREFIX="$(ros2 pkg prefix m20pro_bringup)"
BRINGUP_LIBEXEC="${BRINGUP_PREFIX}/lib/m20pro_bringup"

PROJECT_FASTDDS="${WS_DIR}/install/m20pro_bringup/share/m20pro_bringup/config/m20pro_fastdds_udp.xml"
if [[ -f "${PROJECT_FASTDDS}" && "${M20PRO_USE_PROJECT_FASTDDS:-0}" == "1" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${PROJECT_FASTDDS}"
else
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/opt/robot/fastdds.xml}"
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
set +e
M20PRO_LIDAR_RELAY_WAIT_S="${M20PRO_LIDAR_RELAY_WAIT_S:-45}" \
  "${BRINGUP_LIBEXEC}/m20pro_lidar_relay_guard.sh" start-wait
relay_status="$?"
set -e
if [[ "${relay_status}" -ne 0 ]]; then
  if [[ "${M20PRO_LIDAR_GUARD_MODE:-warn}" == "strict" ]]; then
    echo "[m20pro_real_full] lidar relay is not ready; strict startup is intentionally skipped." >&2
    exit "${relay_status}"
  fi
  echo "[m20pro_real_full] lidar relay returned ${relay_status}; continuing so the workstation web frontend stays available." >&2
fi

COMMON_ARGS=(
  mode:=real
  rviz:=false
  enable_web_dashboard:=true
  enable_initialpose_relocalization:=true
  web_dashboard_data_dir:=/home/user/.m20pro_web
  web_dashboard_map_archive_dir:=/home/user/m20pro_maps
  enable_camera_proxy:=true
  camera_proxy_fps:=2.0
  camera_proxy_jpeg_quality:=45
  camera_proxy_max_width:=480
  cloud_topic:="${LIDAR_RELAY_TOPIC}"
)

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
