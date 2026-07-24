#!/usr/bin/env bash
set -euo pipefail

DURATION_S="${1:-90}"
PREFIX="${2:-m20_real}"
OUT_DIR="${M20PRO_BAG_DIR:-/home/user/bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_PATH="${OUT_DIR}/${PREFIX}_${STAMP}"

if [[ -r /etc/default/m20pro-real ]]; then
  set +u
  source /etc/default/m20pro-real
  set -u
fi
if [[ -z "${ROS_DISTRO:-}" ]]; then
  set +u
  source /opt/robot/scripts/setup_ros2.sh
  set -u
fi
if [[ -f "${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}/install/setup.bash" ]]; then
  set +u
  source "${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}/install/setup.bash"
  set -u
fi
UDP_PROFILE="/home/user/m20pro_real_ros2_ws/install/m20pro_bringup/share/m20pro_bringup/config/m20pro_fastdds_udp.xml"
if [[ -f "${UDP_PROFILE}" && "${M20PRO_FASTDDS_PROFILE:-factory}" != "factory" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${UDP_PROFILE}"
elif [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  if [[ -f /opt/robot/fastdds.xml ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="/opt/robot/fastdds.xml"
  fi
fi
mkdir -p "${OUT_DIR}"
export ROS2CLI_DISABLE_DAEMON=1
ros2 daemon stop >/dev/null 2>&1 || true

if [[ "${EUID}" -ne 0 && "${M20PRO_ALLOW_USER_RECORD:-0}" != "1" ]]; then
  cat >&2 <<'EOF'
m20pro_record_real.sh should be run from the known-good root ROS environment:

  ssh user@10.21.31.104
  source /opt/robot/scripts/setup_ros2.sh
  su
  cd /home/user/m20pro_real_ros2_ws
  source install/setup.bash
  ros2 run m20pro_bringup m20pro_record_real.sh 90 factory_baseline

Do not restart multicast services from this script. If you intentionally want
to record as user, set M20PRO_ALLOW_USER_RECORD=1.
EOF
  exit 2
fi

echo "[m20pro_record_real] output: ${OUT_PATH}"

guard_topic_has_sample() {
  local topic="$1"
  local wait_s="${2:-8}"
  local list_out="${TMPDIR:-/tmp}/m20pro_record_topics.out"
  local echo_out="${TMPDIR:-/tmp}/m20pro_record_echo.out"
  local echo_err="${TMPDIR:-/tmp}/m20pro_record_echo.err"

  if ! timeout 8s ros2 topic list >"${list_out}" 2>/dev/null; then
    echo "[m20pro_record_real] ros2 topic list timed out; not recording an empty bag" >&2
    return 1
  fi
  if ! grep -qx "${topic}" "${list_out}"; then
    echo "[m20pro_record_real] ${topic} is not visible; not recording an empty bag" >&2
    return 1
  fi
  : >"${echo_out}"
  : >"${echo_err}"
  if timeout "${wait_s}" bash -c '
    ros2 topic echo "$1" --no-arr 2>"$2" |
      awk "{ print; if (\$0 == \"---\") exit 0 }" >"$3"
  ' _ "${topic}" "${echo_err}" "${echo_out}" && [[ -s "${echo_out}" ]]; then
    echo "[m20pro_record_real] ${topic} sample OK"
    return 0
  fi
  echo "[m20pro_record_real] ${topic} is visible but no sample arrived within ${wait_s}s; not recording an empty bag" >&2
  tr '\n' ' ' <"${echo_err}" | sed 's/[[:space:]][[:space:]]*/ /g' | cut -c1-220 >&2 || true
  echo >&2
  return 1
}

GUARD_TOPIC="${M20PRO_RECORD_GUARD_TOPIC:-/m20pro/recording_scan}"
GUARD_WAIT_S="${M20PRO_RECORD_GUARD_WAIT_S:-8}"
if [[ "${M20PRO_RECORD_SKIP_CLI_GUARD:-0}" != "1" ]]; then
  guard_topic_has_sample "${GUARD_TOPIC}" "${GUARD_WAIT_S}"
else
  echo "[m20pro_record_real] CLI guard skipped; caller supplied an in-process scan freshness check"
fi

TOPICS=(
  /m20pro/recording_scan
  /scan
  /tf
  /tf_static
  /odom
  /map
  /local_costmap/costmap
  /local_costmap/costmap_updates
  /global_costmap/costmap
  /global_costmap/costmap_updates
  /plan
  /local_plan
  /cmd_vel_nav
  /cmd_vel_teleop
  /cmd_vel
  /m20pro/cmd_vel_mux/status
  /m20pro/current_floor
  /m20pro/stair_status
  /m20pro/stair_executor/start
  /m20pro/stair_executor/status
  /m20pro/floor_switch_request
  /m20pro/floor_switch_result
  /m20pro/set_current_floor
  /m20pro/gait_command
  /m20pro/floor_goal
  /m20pro/active_waypoint
  /m20pro/charge_command
  /m20pro_tcp_bridge/map_pose
  /m20pro_tcp_bridge/localization_ok
  /m20pro_tcp_bridge/navigation_status
  /m20pro_tcp_bridge/relocalization_result
  /m20pro_tcp_bridge/gait_result
  /m20pro_tcp_bridge/charge_result
  /NAV_STATUS
  /MOTION_STATE
  /MOTION_STATUS
  /MOTION_INFO
  /GAIT
  /STEER
  /HANDLE_STEER
  /BATTERY_DATA
  /FAULT_STATUS
)

echo "[m20pro_record_real] guard_topic=${GUARD_TOPIC} duration=${DURATION_S}s"
exec timeout --signal=INT "${DURATION_S}" ros2 bag record -o "${OUT_PATH}" "${TOPICS[@]}"
