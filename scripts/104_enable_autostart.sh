#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-move}"
WS_DIR="${M20PRO_WS:-/home/user/m20pro_ros2_ws}"
UNIT_SRC="${WS_DIR}/systemd/m20pro-real.service"
UNIT_DST="/etc/systemd/system/m20pro-real.service"
ENV_DST="/etc/default/m20pro-real"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root on 104 after su." >&2
  echo "Usage: ./scripts/104_enable_autostart.sh [move|shadow]" >&2
  exit 2
fi

case "${MODE}" in
  move|shadow) ;;
  *)
    echo "Usage: ./scripts/104_enable_autostart.sh [move|shadow]" >&2
    exit 2
    ;;
esac

if [[ ! -f "${UNIT_SRC}" ]]; then
  echo "systemd unit not found: ${UNIT_SRC}" >&2
  exit 1
fi

install -m 0644 "${UNIT_SRC}" "${UNIT_DST}"
cat >"${ENV_DST}" <<EOF
M20PRO_REAL_MODE=${MODE}
M20PRO_WS=${WS_DIR}
M20PRO_LIDAR_STARTUP_WAIT_S=${M20PRO_LIDAR_STARTUP_WAIT_S:-45}
M20PRO_LIDAR_GUARD_MODE=${M20PRO_LIDAR_GUARD_MODE:-warn}
M20PRO_LIDAR_RELAY_TOPIC=${M20PRO_LIDAR_RELAY_TOPIC:-/m20pro/lidar_points_relay}
M20PRO_LIDAR_RELAY_WAIT_S=${M20PRO_LIDAR_RELAY_WAIT_S:-45}
M20PRO_LIDAR_RELAY_MAX_OUTPUT_POINTS=${M20PRO_LIDAR_RELAY_MAX_OUTPUT_POINTS:-6000}
M20PRO_LIDAR_RELAY_MIN_PUBLISH_INTERVAL_S=${M20PRO_LIDAR_RELAY_MIN_PUBLISH_INTERVAL_S:-0.2}
M20PRO_LIDAR_RELAY_CLOUD_RELIABILITY=${M20PRO_LIDAR_RELAY_CLOUD_RELIABILITY:-auto}
M20PRO_ENABLE_LIDAR2_RELAY=${M20PRO_ENABLE_LIDAR2_RELAY:-0}
M20PRO_LIDAR2_TOPIC=${M20PRO_LIDAR2_TOPIC:-/LIDAR/POINTS2}
M20PRO_LIDAR2_RELAY_TOPIC=${M20PRO_LIDAR2_RELAY_TOPIC:-/m20pro/lidar_points2_relay}
M20PRO_LIDAR2_RELAY_WAIT_S=${M20PRO_LIDAR2_RELAY_WAIT_S:-8}
M20PRO_FASTDDS_PROFILE=${M20PRO_FASTDDS_PROFILE:-project_udp}
M20PRO_LIDAR_RELAY_FASTDDS_PROFILE=${M20PRO_LIDAR_RELAY_FASTDDS_PROFILE:-project_udp}
EOF

systemctl daemon-reload
systemctl enable m20pro-real.service

echo "[104_enable_autostart] installed and enabled m20pro-real.service"
echo "[104_enable_autostart] mode=${MODE}"
echo "[104_enable_autostart] start now: systemctl start m20pro-real.service"
echo "[104_enable_autostart] status: ./scripts/104_autostart_status.sh"
