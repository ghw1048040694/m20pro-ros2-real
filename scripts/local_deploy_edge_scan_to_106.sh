#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-user@10.21.31.106}"
REMOTE_WS="${2:-/home/user/m20pro_real_ros2_ws}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
STAGE="${REMOTE_WS}.edge_stage.${STAMP}"

cleanup_stage() {
  ssh "${HOST}" "rm -rf '${STAGE}'" >/dev/null 2>&1 || true
}
trap cleanup_stage EXIT

if ! command -v rsync >/dev/null 2>&1; then
  echo "rsync is required on the upper computer." >&2
  exit 2
fi

echo "[local_deploy_edge_scan_to_106] remote=${HOST}:${REMOTE_WS}"

ssh "${HOST}" "rm -rf '${STAGE}'; mkdir -p '${STAGE}/scripts' '${STAGE}/tools/edge_scan_feasibility'"

rsync -az --delete --no-owner --no-group \
  --exclude='__pycache__/' \
  "${ROOT_DIR}/tools/edge_scan_feasibility/" \
  "${HOST}:${STAGE}/tools/edge_scan_feasibility/"
rsync -az --no-owner --no-group \
  "${ROOT_DIR}/scripts/106_enable_edge_scan_service.sh" \
  "${HOST}:${STAGE}/scripts/106_enable_edge_scan_service.sh"

ssh -tt "${HOST}" "bash -lc '
set -e
sudo -v
sudo -n install -d -m 0755 "${REMOTE_WS}/scripts" \
  "${REMOTE_WS}/tools/edge_scan_feasibility"
sudo -n rsync -a --delete \
  "${STAGE}/tools/edge_scan_feasibility/" \
  "${REMOTE_WS}/tools/edge_scan_feasibility/"
sudo -n install -m 0755 \
  "${STAGE}/scripts/106_enable_edge_scan_service.sh" \
  "${REMOTE_WS}/scripts/106_enable_edge_scan_service.sh"
sudo -n chown -R \"\$(id -u):\$(id -g)\" \
  "${REMOTE_WS}/scripts/106_enable_edge_scan_service.sh" \
  "${REMOTE_WS}/tools/edge_scan_feasibility"
sudo -n env M20PRO_WS="${REMOTE_WS}" \
  bash "${REMOTE_WS}/scripts/106_enable_edge_scan_service.sh"
sudo -n systemctl restart m20pro-edge-scan-106.service
sudo -n systemctl is-enabled --quiet m20pro-edge-scan-106.service
sudo -n systemctl is-active --quiet m20pro-edge-scan-106.service
'"

trap - EXIT
cleanup_stage
echo "[local_deploy_edge_scan_to_106] edge scan is installed, enabled, and active"
