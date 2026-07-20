#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-user@10.21.31.104}"
REMOTE_WS="${2:-/home/user/m20pro_real_ros2_ws}"
EDGE_HOST="${M20PRO_EDGE_HOST:-user@10.21.31.106}"
EDGE_WS="${M20PRO_EDGE_WS:-/home/user/m20pro_real_ros2_ws}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
STAGE="${REMOTE_WS}.deploy.${STAMP}"
BACKUP_ROOT="${M20PRO_DEPLOY_BACKUP_DIR:-/home/user/m20pro_deploy_backups}"
BACKUP="${BACKUP_ROOT}/$(basename "${REMOTE_WS}")_${STAMP}"
REVISION="$(git -C "${ROOT_DIR}" rev-parse HEAD 2>/dev/null || echo unknown)"
WEB_HOST="${HOST#*@}"
WEB_URL="${M20PRO_WEB_URL:-http://${WEB_HOST}:8080}"

echo "[local_deploy_to_test_robot] local=${ROOT_DIR}"
echo "[local_deploy_to_test_robot] remote=${HOST}:${REMOTE_WS}"
echo "[local_deploy_to_test_robot] edge=${EDGE_HOST}:${EDGE_WS}"

for command_name in ssh rsync curl python3; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "${command_name} is required on the upper computer." >&2
    exit 2
  fi
done

"${ROOT_DIR}/scripts/m20pro_field_profile.py" check
PROFILE_HASH="$("${ROOT_DIR}/scripts/m20pro_field_profile.py" show-json | \
  python3 -c 'import json, sys; print(json.load(sys.stdin)["profile_hash"])')"
echo "[local_deploy_to_test_robot] field_profile_hash=${PROFILE_HASH}"

edge_was_active=1
edge_was_enabled=1
if [[ "${M20PRO_DEPLOY_SKIP_EDGE:-0}" != "1" ]]; then
  edge_previous_state="$(ssh "${EDGE_HOST}" '
active=0
enabled=0
systemctl is-active --quiet m20pro-edge-scan-106.service 2>/dev/null && active=1
systemctl is-enabled --quiet m20pro-edge-scan-106.service 2>/dev/null && enabled=1
printf "%s %s\n" "${active}" "${enabled}"
')"
  read -r edge_was_active edge_was_enabled <<<"${edge_previous_state}"
  "${ROOT_DIR}/scripts/local_deploy_edge_scan_to_106.sh" "${EDGE_HOST}" "${EDGE_WS}"
fi

EDGE_PROFILE_HASH="$(ssh "${EDGE_HOST}" \
  "sed -n 's/^FIELD_PROFILE_HASH=//p' /etc/m20pro-edge-scan-106.env")"
if [[ "${EDGE_PROFILE_HASH}" != "${PROFILE_HASH}" ]]; then
  echo "106 field profile mismatch: local=${PROFILE_HASH} 106=${EDGE_PROFILE_HASH}" >&2
  exit 16
fi

ssh "${HOST}" "mkdir -p '${STAGE}' '${BACKUP_ROOT}'"

rsync -az --delete \
  --exclude='.git/' \
  --exclude='build/' \
  --exclude='install/' \
  --exclude='log/' \
  --exclude='__pycache__/' \
  --exclude='bags/' \
  --exclude='.colcon/' \
  --exclude='*.bag' \
  --exclude='*.db3' \
  --exclude='*.mcap' \
  "${ROOT_DIR}/" "${HOST}:${STAGE}/"

if ! ssh -tt "${HOST}" "bash -lc '
exec bash \"${STAGE}/scripts/104_install_staged_workspace.sh\" \
  \"${STAGE}\" \"${REMOTE_WS}\" \"${BACKUP}\" \"${REVISION}\"
'"; then
  if [[ "${M20PRO_DEPLOY_SKIP_EDGE:-0}" != "1" ]] && \
      { [[ "${edge_was_active}" -ne 1 ]] || [[ "${edge_was_enabled}" -ne 1 ]]; }; then
    ssh -tt "${EDGE_HOST}" "bash -lc '
set -e
sudo -v
if [[ ${edge_was_active} -ne 1 ]]; then
  sudo -n systemctl stop m20pro-edge-scan-106.service
fi
if [[ ${edge_was_enabled} -ne 1 ]]; then
  sudo -n systemctl disable m20pro-edge-scan-106.service
fi
'" || true
  fi
  exit 14
fi

# The 106 publisher is a bare DDS application. Restart it after the new 104
# subscribers exist so discovery is deterministic, unless this deployment was
# explicitly scoped to 104 only.
if [[ "${M20PRO_DEPLOY_SKIP_EDGE:-0}" != "1" ]]; then
  ssh -tt "${EDGE_HOST}" "bash -lc '
  set -e
  sudo -v
  sudo -n systemctl restart m20pro-edge-scan-106.service
  sudo -n systemctl is-active --quiet m20pro-edge-scan-106.service
  '"
fi

edge_ready=0
for _ in $(seq 1 90); do
  payload="$(curl --connect-timeout 1 --max-time 3 -fsS "${WEB_URL}/api/state" 2>/dev/null || true)"
  if [[ -n "${payload}" ]] && PAYLOAD="${payload}" python3 -c '
import json
import os
import sys

state = json.loads(os.environ["PAYLOAD"])
perception = state.get("perception_status") or {}
scan = perception.get("scan") or {}
ready = (
    perception.get("ready") is True
    and perception.get("mode") == "edge_scan"
    and scan.get("frame_id") == "m20pro_base_link"
    and int(scan.get("finite_ranges") or 0) >= 20
)
sys.exit(0 if ready else 1)
'; then
    edge_ready=1
    break
  fi
  sleep 1
done

if [[ "${edge_ready}" -ne 1 ]]; then
  echo "104 web is up, but 106 edge scan did not pass API acceptance at ${WEB_URL}." >&2
  echo "Previous 104 workspace is retained at ${BACKUP}." >&2
  exit 15
fi

REMOTE_PROFILE_HASH="$(ssh "${HOST}" \
  "'${REMOTE_WS}/scripts/m20pro_field_profile.py' show-json | python3 -c 'import json, sys; print(json.load(sys.stdin)[\"profile_hash\"])'")"
if [[ "${REMOTE_PROFILE_HASH}" != "${PROFILE_HASH}" ]]; then
  echo "104 field profile mismatch: local=${PROFILE_HASH} 104=${REMOTE_PROFILE_HASH}" >&2
  exit 17
fi

echo "[local_deploy_to_test_robot] done revision=${REVISION}"
echo "[local_deploy_to_test_robot] API=${WEB_URL} edge_scan=ready"
echo "[local_deploy_to_test_robot] field_profile_hash=${PROFILE_HASH} matched_on=104,106"

if [[ "${M20PRO_DEPLOY_KEEP_BACKUP:-0}" != "1" ]]; then
  ssh "${HOST}" "rm -rf '${BACKUP}' '${BACKUP}.systemd'"
  echo "[local_deploy_to_test_robot] removed successful-deployment backup"
else
  echo "[local_deploy_to_test_robot] kept backup=${BACKUP}"
fi
