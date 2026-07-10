#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
MEDIAMTX_BIN_SRC="${M20PRO_MEDIAMTX_BIN_SRC:-/tmp/mediamtx}"
BIN_DST="/usr/local/lib/m20pro/mediamtx"
CONFIG_SRC="${WS_DIR}/systemd/m20pro-camera-webrtc-104.yml"
CONFIG_DST="/etc/m20pro-camera-webrtc-104.yml"
UNIT_SRC="${WS_DIR}/systemd/m20pro-camera-webrtc-104.service"
UNIT_DST="/etc/systemd/system/m20pro-camera-webrtc-104.service"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root on 104" >&2
  exit 2
fi
for path in "${MEDIAMTX_BIN_SRC}" "${CONFIG_SRC}" "${UNIT_SRC}"; do
  if [[ ! -e "${path}" ]]; then
    echo "missing required file: ${path}" >&2
    exit 1
  fi
done

install -d -m 0755 "$(dirname "${BIN_DST}")"
install -m 0755 "${MEDIAMTX_BIN_SRC}" "${BIN_DST}"
install -m 0644 "${CONFIG_SRC}" "${CONFIG_DST}"
install -m 0644 "${UNIT_SRC}" "${UNIT_DST}"
systemctl daemon-reload
systemctl enable --now m20pro-camera-webrtc-104.service

echo "[104_enable_camera_webrtc] installed and started m20pro-camera-webrtc-104.service"
