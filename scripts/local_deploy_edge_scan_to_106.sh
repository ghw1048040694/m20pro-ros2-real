#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-user@10.21.31.106}"
REMOTE_WS="${2:-/home/user/m20pro_real_ros2_ws}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v rsync >/dev/null 2>&1; then
  echo "rsync is required on the upper computer." >&2
  exit 2
fi

echo "[local_deploy_edge_scan_to_106] remote=${HOST}:${REMOTE_WS}"

ssh "${HOST}" "mkdir -p '${REMOTE_WS}/scripts' '${REMOTE_WS}/tools/edge_scan_feasibility'"

rsync -az --delete \
  --exclude='__pycache__/' \
  "${ROOT_DIR}/tools/edge_scan_feasibility/" \
  "${HOST}:${REMOTE_WS}/tools/edge_scan_feasibility/"
rsync -az \
  "${ROOT_DIR}/scripts/106_enable_edge_scan_service.sh" \
  "${HOST}:${REMOTE_WS}/scripts/106_enable_edge_scan_service.sh"

ssh -tt "${HOST}" "bash -lc '
set -e
chmod +x "${REMOTE_WS}/scripts/106_enable_edge_scan_service.sh" \
  "${REMOTE_WS}/tools/edge_scan_feasibility/build_on_106.sh"
sudo -v
sudo -n env M20PRO_WS="${REMOTE_WS}" \
  bash "${REMOTE_WS}/scripts/106_enable_edge_scan_service.sh"
sudo -n systemctl is-enabled --quiet m20pro-edge-scan-106.service
sudo -n systemctl is-active --quiet m20pro-edge-scan-106.service
'"

echo "[local_deploy_edge_scan_to_106] edge scan is installed, enabled, and active"
