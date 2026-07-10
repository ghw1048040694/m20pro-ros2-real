#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-move}"
WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
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
M20PRO_FASTDDS_PROFILE=${M20PRO_FASTDDS_PROFILE:-project_udp}
M20PRO_CLEAN_STALE_FASTDDS_SHM=${M20PRO_CLEAN_STALE_FASTDDS_SHM:-1}
M20PRO_SCAN_TOPIC=${M20PRO_SCAN_TOPIC:-/scan}
M20PRO_ENABLE_INSPECTION=${M20PRO_ENABLE_INSPECTION:-false}
M20PRO_INSPECTION_BACKEND=${M20PRO_INSPECTION_BACKEND:-auto}
M20PRO_INSPECTION_SOURCE_TYPE=${M20PRO_INSPECTION_SOURCE_TYPE:-rtsp}
M20PRO_INSPECTION_RTSP_URL=${M20PRO_INSPECTION_RTSP_URL:-rtsp://10.21.31.103:8554/video1}
M20PRO_INSPECTION_CAMERA_NAME=${M20PRO_INSPECTION_CAMERA_NAME:-front_wide}
M20PRO_INSPECTION_MODEL_PATH=${M20PRO_INSPECTION_MODEL_PATH:-${WS_DIR}/install/m20pro_inspection/share/m20pro_inspection/models/best.pt}
M20PRO_INSPECTION_CLASS_NAMES_PATH=${M20PRO_INSPECTION_CLASS_NAMES_PATH:-${WS_DIR}/install/m20pro_inspection/share/m20pro_inspection/models/labels_zh.txt}
EOF

systemctl daemon-reload
systemctl enable m20pro-real.service

echo "[104_enable_autostart] installed and enabled m20pro-real.service"
echo "[104_enable_autostart] mode=${MODE}"
echo "[104_enable_autostart] start now: systemctl start m20pro-real.service"
echo "[104_enable_autostart] status: ./scripts/104_autostart_status.sh"
