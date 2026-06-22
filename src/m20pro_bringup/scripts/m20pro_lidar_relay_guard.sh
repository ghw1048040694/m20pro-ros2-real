#!/usr/bin/env bash
set -euo pipefail

INPUT_TOPIC="${M20PRO_LIDAR_TOPIC:-/LIDAR/POINTS}"
RELAY_TOPIC="${M20PRO_LIDAR_RELAY_TOPIC:-/m20pro/lidar_points_relay}"
STATUS_TOPIC="${M20PRO_LIDAR_RELAY_STATUS_TOPIC:-/m20pro/lidar_relay/status}"
WAIT_S="${M20PRO_LIDAR_RELAY_WAIT_S:-45}"
PID_FILE="${M20PRO_LIDAR_RELAY_PID_FILE:-/tmp/m20pro_lidar_relay.pid}"
LOG_FILE="${M20PRO_LIDAR_RELAY_LOG_FILE:-/tmp/m20pro_lidar_relay.log}"
MAX_OUTPUT_POINTS="${M20PRO_LIDAR_RELAY_MAX_OUTPUT_POINTS:-12000}"
MIN_PUBLISH_INTERVAL_S="${M20PRO_LIDAR_RELAY_MIN_PUBLISH_INTERVAL_S:-0.1}"

if [[ -z "${ROS_DISTRO:-}" || ! -x "$(command -v ros2)" ]]; then
  set +u
  source /opt/robot/scripts/setup_ros2.sh
  set -u
fi

relay_exec() {
  local prefix
  prefix="$(ros2 pkg prefix m20pro_navigation)"
  echo "${prefix}/lib/m20pro_navigation/lidar_relay"
}

existing_relay_pid() {
  local exec_path="$1"
  ps -eo pid=,args= | awk -v exe="${exec_path}" -v input_arg="input_topic:=${INPUT_TOPIC}" -v output_arg="output_topic:=${RELAY_TOPIC}" '
    $0 ~ exe && $0 !~ /awk/ {
      has_input = 0
      has_output = 0
      for (i = 1; i <= NF; i++) {
        if ($i == input_arg) {
          has_input = 1
        }
        if ($i == output_arg) {
          has_output = 1
        }
      }
      if (has_input && has_output) {
        print $1
        exit
      }
    }
  '
}

relay_profile_for_pid() {
  local pid="$1"
  tr '\0' '\n' <"/proc/${pid}/environ" 2>/dev/null \
    | awk -F= '$1 == "FASTRTPS_DEFAULT_PROFILES_FILE" {print $2; exit}' || true
}

stop_pid() {
  local pid="$1"
  if [[ -z "${pid}" ]] || ! ps -p "${pid}" >/dev/null 2>&1; then
    return
  fi
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 30); do
    if ! ps -p "${pid}" >/dev/null 2>&1; then
      return
    fi
    sleep 0.1
  done
  kill "${pid}" 2>/dev/null || true
}

start_relay() {
  local exec_path
  exec_path="$(relay_exec)"
  if [[ ! -x "${exec_path}" ]]; then
    echo "[m20pro_lidar_relay_guard] relay executable not found: ${exec_path}" >&2
    exit 2
  fi

  existing_pid="$(existing_relay_pid "${exec_path}")"
  if [[ -n "${existing_pid}" ]]; then
    existing_profile="$(relay_profile_for_pid "${existing_pid}")"
    current_profile="${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
    if [[ "${M20PRO_LIDAR_RELAY_RESTART_ON_PROFILE_MISMATCH:-1}" == "1" \
      && -n "${current_profile}" \
      && "${existing_profile}" != "${current_profile}" ]]; then
      echo "[m20pro_lidar_relay_guard] restarting relay pid=${existing_pid} for DDS profile change: ${existing_profile:-unset} -> ${current_profile}"
      stop_pid "${existing_pid}"
    else
      echo "${existing_pid}" >"${PID_FILE}"
      echo "[m20pro_lidar_relay_guard] relay already running pid=${existing_pid}"
      return
    fi
  fi

  existing_pid="$(existing_relay_pid "${exec_path}")"
  if [[ -n "${existing_pid}" ]]; then
    echo "${existing_pid}" >"${PID_FILE}"
    echo "[m20pro_lidar_relay_guard] relay already running pid=${existing_pid}"
    return
  fi

  if [[ -f "${PID_FILE}" ]]; then
    old_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
    if [[ -n "${old_pid}" ]] && ps -p "${old_pid}" >/dev/null 2>&1; then
      echo "[m20pro_lidar_relay_guard] relay already running pid=${old_pid}"
      return
    fi
  fi

  echo "[m20pro_lidar_relay_guard] starting relay ${INPUT_TOPIC} -> ${RELAY_TOPIC}"
  : >"${LOG_FILE}"
  nohup "${exec_path}" \
    --ros-args \
    -p input_topic:="${INPUT_TOPIC}" \
    -p output_topic:="${RELAY_TOPIC}" \
    -p status_topic:="${STATUS_TOPIC}" \
    -p max_output_points:="${MAX_OUTPUT_POINTS}" \
    -p min_publish_interval_s:="${MIN_PUBLISH_INTERVAL_S}" \
    >"${LOG_FILE}" 2>&1 &
  echo "$!" >"${PID_FILE}"
}

wait_relay_sample() {
  echo "[m20pro_lidar_relay_guard] waiting relay sample topic=${RELAY_TOPIC} wait=${WAIT_S}s"
  deadline=$((SECONDS + WAIT_S))
  while (( SECONDS < deadline )); do
    if grep -q 'LIDAR relay sample OK:' "${LOG_FILE}" 2>/dev/null; then
      sample_line="$(grep 'LIDAR relay sample OK:' "${LOG_FILE}" | tail -n 1)"
      echo "[m20pro_lidar_relay_guard] relay sample OK via relay log: ${sample_line}"
      return 0
    fi

    if [[ -f "${PID_FILE}" ]]; then
      pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
      if [[ -n "${pid}" ]] && ! ps -p "${pid}" >/dev/null 2>&1; then
        echo "[m20pro_lidar_relay_guard] relay process exited before sample pid=${pid}" >&2
        break
      fi
    fi
    sleep 0.5
  done

  echo "[m20pro_lidar_relay_guard] no relay sample within ${WAIT_S}s" >&2
  echo "[m20pro_lidar_relay_guard] relay log tail:" >&2
  tail -n 40 "${LOG_FILE}" >&2 || true
  return 75
}

stop_relay() {
  local exec_path
  exec_path="$(relay_exec)"
  mapfile -t pids < <(
    ps -eo pid=,args= | awk -v exe="${exec_path}" -v input_arg="input_topic:=${INPUT_TOPIC}" -v output_arg="output_topic:=${RELAY_TOPIC}" '
      $0 ~ exe && $0 !~ /awk/ {
        has_input = 0
        has_output = 0
        for (i = 1; i <= NF; i++) {
          if ($i == input_arg) {
            has_input = 1
          }
          if ($i == output_arg) {
            has_output = 1
          }
        }
        if (has_input && has_output) {
          print $1
        }
      }
    '
  )
  if [[ "${#pids[@]}" -gt 0 ]]; then
    echo "[m20pro_lidar_relay_guard] stopping relay ${INPUT_TOPIC} -> ${RELAY_TOPIC} pids=${pids[*]}"
    kill -INT "${pids[@]}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      alive=()
      for pid in "${pids[@]}"; do
        if ps -p "${pid}" >/dev/null 2>&1; then
          alive+=("${pid}")
        fi
      done
      if [[ "${#alive[@]}" -eq 0 ]]; then
        break
      fi
      sleep 0.1
    done
    for pid in "${pids[@]}"; do
      if ps -p "${pid}" >/dev/null 2>&1; then
        kill "${pid}" 2>/dev/null || true
      fi
    done
  fi
  rm -f "${PID_FILE}"
}

case "${1:-start-wait}" in
  start-wait)
    start_relay
    wait_relay_sample
    ;;
  stop)
    stop_relay
    ;;
  wait)
    wait_relay_sample
    ;;
  *)
    echo "Usage: $0 [start-wait|wait|stop]" >&2
    exit 2
    ;;
esac
