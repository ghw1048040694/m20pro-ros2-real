#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
PROFILE="${WS_DIR}/src/m20pro_bringup/config/m20pro_field_profile.yaml"
PROFILE_TOOL="${WS_DIR}/scripts/m20pro_field_profile.py"
UNIT_SRC="${WS_DIR}/tools/edge_scan_feasibility/service/m20pro-edge-scan-106.service.example"
ENV_DST="/etc/m20pro-edge-scan-106.env"
UNIT_DST="/etc/systemd/system/m20pro-edge-scan-106.service"
BIN_DIR="/usr/local/lib/m20pro"
BUILD_DIR="/tmp/m20pro_edge_scan_build"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this as root on 106 after: source /opt/robot/scripts/setup_ros2.sh && su" >&2
  exit 2
fi

if [[ ! -f "${PROFILE}" || ! -x "${PROFILE_TOOL}" ]]; then
  echo "missing canonical field profile or renderer under ${WS_DIR}" >&2
  exit 1
fi
if [[ ! -f "${UNIT_SRC}" ]]; then
  echo "missing service template: ${UNIT_SRC}" >&2
  exit 1
fi

ENV_TMP="$(mktemp /tmp/m20pro-edge-scan-106.env.XXXXXX)"
trap 'rm -f "${ENV_TMP}" "${ENV_DST}.new"; rm -rf "${BUILD_DIR}"' EXIT
"${PROFILE_TOOL}" render-edge-env >"${ENV_TMP}"
install -m 0644 "${ENV_TMP}" "${ENV_DST}.new"
mv -f "${ENV_DST}.new" "${ENV_DST}"
"${WS_DIR}/tools/edge_scan_feasibility/build_on_106.sh" "${BUILD_DIR}"
install -d -m 0755 "${BIN_DIR}"
install -m 0755 "${BUILD_DIR}/m20pro_edge_scan" "${BIN_DIR}/m20pro_edge_scan"
install -m 0644 "${UNIT_SRC}" "${UNIT_DST}"
systemctl daemon-reload
systemctl enable m20pro-edge-scan-106.service
systemctl restart m20pro-edge-scan-106.service
trap - EXIT
rm -f "${ENV_TMP}"
rm -rf "${BUILD_DIR}"

echo "[106_enable_edge_scan_service] installed ${UNIT_DST}"
echo "[106_enable_edge_scan_service] installed ${ENV_DST}"
echo "[106_enable_edge_scan_service] installed ${BIN_DIR}/m20pro_edge_scan"
echo "[106_enable_edge_scan_service] enabled and started production edge scan"
