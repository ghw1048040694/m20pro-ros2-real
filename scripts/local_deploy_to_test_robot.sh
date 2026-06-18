#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-user@10.21.31.104}"
REMOTE_WS="${2:-/home/user/m20pro_ros2_ws}"
BRANCH="${M20PRO_DEPLOY_BRANCH:-main}"
REMOTE_NAME="${M20PRO_DEPLOY_REMOTE:-gitlab}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[local_deploy_to_test_robot] local=${ROOT_DIR}"
echo "[local_deploy_to_test_robot] remote=${HOST}:${REMOTE_WS}"

if ! command -v rsync >/dev/null 2>&1; then
  echo "rsync is required on the local machine." >&2
  exit 2
fi

ssh "${HOST}" "mkdir -p '${REMOTE_WS}'"

ssh "${HOST}" "bash -lc '
if [ -d \"${REMOTE_WS}/build\" ] || [ -d \"${REMOTE_WS}/install\" ] || [ -d \"${REMOTE_WS}/log\" ]; then
  chown -R user:user \"${REMOTE_WS}\" 2>/dev/null || true
  rm -rf \"${REMOTE_WS}/build\" \"${REMOTE_WS}/install\" \"${REMOTE_WS}/log\" 2>/dev/null || {
    echo \"remote build/install/log contain root-owned files; clean them from a root shell on 104, then rerun deploy\" >&2
    exit 13
  }
fi
'"

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
  "${ROOT_DIR}/" "${HOST}:${REMOTE_WS}/"

ssh -tt "${HOST}" "bash -lc '
set -e
source /opt/robot/scripts/setup_ros2.sh
cd \"${REMOTE_WS}\"
if [ -f install/setup.bash ]; then rm -rf build install log; fi
colcon build --symlink-install
chmod +x scripts/*.sh src/m20pro_bringup/scripts/*.sh 2>/dev/null || true
echo \"[local_deploy_to_test_robot] deployed branch hint: ${REMOTE_NAME}/${BRANCH}\"
'"

echo "[local_deploy_to_test_robot] done"
