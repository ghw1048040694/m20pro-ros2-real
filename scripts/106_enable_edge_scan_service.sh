#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
ENV_SRC="${WS_DIR}/tools/edge_scan_feasibility/service/m20pro-edge-scan-106.env.edge_scan"
UNIT_SRC="${WS_DIR}/tools/edge_scan_feasibility/service/m20pro-edge-scan-106.service.example"
ENV_DST="/etc/m20pro-edge-scan-106.env"
UNIT_DST="/etc/systemd/system/m20pro-edge-scan-106.service"
BIN_DIR="/usr/local/lib/m20pro"
BUILD_DIR="/tmp/m20pro_edge_scan_build"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this as root on 106 after: source /opt/robot/scripts/setup_ros2.sh && su" >&2
  exit 2
fi

if [[ ! -f "${ENV_SRC}" ]]; then
  echo "missing env template: ${ENV_SRC}" >&2
  exit 1
fi
if [[ ! -f "${UNIT_SRC}" ]]; then
  echo "missing service template: ${UNIT_SRC}" >&2
  exit 1
fi

install -m 0644 "${ENV_SRC}" "${ENV_DST}"
"${WS_DIR}/tools/edge_scan_feasibility/build_on_106.sh" "${BUILD_DIR}"
install -d -m 0755 "${BIN_DIR}"
install -m 0755 "${BUILD_DIR}/m20pro_edge_scan" "${BIN_DIR}/m20pro_edge_scan"
install -m 0644 "${UNIT_SRC}" "${UNIT_DST}"
systemctl daemon-reload
systemctl enable --now m20pro-edge-scan-106.service

echo "[106_enable_edge_scan_service] installed ${UNIT_DST}"
echo "[106_enable_edge_scan_service] installed ${ENV_DST}"
echo "[106_enable_edge_scan_service] installed ${BIN_DIR}/m20pro_edge_scan"
echo "[106_enable_edge_scan_service] enabled and started production edge scan"
