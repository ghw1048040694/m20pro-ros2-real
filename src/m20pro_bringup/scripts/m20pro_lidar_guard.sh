#!/usr/bin/env bash
set -euo pipefail

PURPOSE="${1:-startup}"
TOPIC="${M20PRO_LIDAR_TOPIC:-/LIDAR/POINTS}"

case "${PURPOSE}" in
  startup)
    DEFAULT_WAIT_S="${M20PRO_LIDAR_STARTUP_WAIT_S:-45}"
    DEFAULT_EXIT_CODE=75
    ;;
  record|check)
    DEFAULT_WAIT_S="${M20PRO_LIDAR_CHECK_WAIT_S:-12}"
    DEFAULT_EXIT_CODE=3
    ;;
  *)
    echo "Usage: $0 [startup|record|check]" >&2
    exit 2
    ;;
esac

WAIT_S="${M20PRO_LIDAR_WAIT_S:-${DEFAULT_WAIT_S}}"
NO_DATA_EXIT_CODE="${M20PRO_LIDAR_NO_DATA_EXIT:-${DEFAULT_EXIT_CODE}}"
TMP_DIR="${TMPDIR:-/tmp}"
LIST_OUT="${TMP_DIR}/m20pro_lidar_guard_topics.out"
ECHO_OUT="${TMP_DIR}/m20pro_lidar_guard_echo.out"
ECHO_ERR="${TMP_DIR}/m20pro_lidar_guard_echo.err"

if [[ "${M20PRO_SKIP_LIDAR_GUARD:-0}" == "1" ]]; then
  echo "[m20pro_lidar_guard] skipped by M20PRO_SKIP_LIDAR_GUARD=1" >&2
  exit 0
fi

if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f /opt/robot/fastdds.xml ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/robot/fastdds.xml
fi

echo "[m20pro_lidar_guard] purpose=${PURPOSE} topic=${TOPIC} wait=${WAIT_S}s"
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "[m20pro_lidar_guard] FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi

if ! timeout 8s ros2 topic list >"${LIST_OUT}" 2>/dev/null; then
  echo "[m20pro_lidar_guard] ros2 topic list did not return within 8s" >&2
  exit "${NO_DATA_EXIT_CODE}"
fi

echo "[m20pro_lidar_guard] visible lidar topics:"
grep -E '^/(LIDAR/POINTS|LIDAR/POINTS2|scan|ODOM|IMU)$' "${LIST_OUT}" || true

if ! grep -qx "${TOPIC}" "${LIST_OUT}"; then
  cat >&2 <<EOF
[m20pro_lidar_guard] ${TOPIC} is not visible.

Do not clear /dev/shm/fastrtps_*.
Do not restart factory multicast/lidar services from this project script.
Stop only the M20Pro real stack, then use the known-good root sequence to
verify whether ${TOPIC} can produce real PointCloud2 samples.
EOF
  exit "${NO_DATA_EXIT_CODE}"
fi

: >"${ECHO_OUT}"
: >"${ECHO_ERR}"
if timeout "${WAIT_S}" bash -c '
  ros2 topic echo "$1" --no-arr 2>"$2" |
    awk "{ print; if (\$0 == \"---\") exit 0 }" >"$3"
' _ "${TOPIC}" "${ECHO_ERR}" "${ECHO_OUT}" && [[ -s "${ECHO_OUT}" ]]; then
  frame_id="$(awk -F"'" '/frame_id:/ {print $2; exit}' "${ECHO_OUT}" || true)"
  width="$(awk '/width:/ {print $2; exit}' "${ECHO_OUT}" || true)"
  height="$(awk '/height:/ {print $2; exit}' "${ECHO_OUT}" || true)"
  echo "[m20pro_lidar_guard] ${TOPIC} sample OK frame_id=${frame_id:-unknown} width=${width:-unknown} height=${height:-unknown}"
  exit 0
fi

err_tail="$(tr '\n' ' ' <"${ECHO_ERR}" | sed 's/[[:space:]][[:space:]]*/ /g' | cut -c1-220)"
cat >&2 <<EOF
[m20pro_lidar_guard] ${TOPIC} is visible but no PointCloud2 sample arrived within ${WAIT_S}s.

This is the dangerous topic-name-only state. Starting more real stacks can add
DDS participants and make 104 less stable, so startup is stopped here.

Recovery rule:
  1. Stop only our stack: ./scripts/104_stop_real.sh or systemctl stop m20pro-real.service
  2. Do not clear /dev/shm/fastrtps_*
  3. Do not restart multicast-relay.service or rsdriver.service unless the human explicitly asks
  4. If the topic-name-only state remains, reboot 104, then verify samples before full real startup

Last echo error: ${err_tail:-none}
EOF
exit "${NO_DATA_EXIT_CODE}"
