#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_ros2_ws}"
REMOTE="${M20PRO_REMOTE:-gitlab}"
BRANCH="${M20PRO_BRANCH:-main}"
REMOTE_URL="${M20PRO_REMOTE_URL:-git@git.fabu.ai:genghaowei/m20pro-ros2-real.git}"
PACKAGES="${M20PRO_BUILD_PACKAGES:-}"
RESTART_SERVICE="${M20PRO_RESTART_SERVICE:-0}"

if [[ "${EUID}" -ne 0 ]]; then
  cat >&2 <<'EOF'
Run this on 104 after su:

  ssh user@10.21.31.104
  source /opt/robot/scripts/setup_ros2.sh
  su
  cd /home/user/m20pro_ros2_ws
  ./scripts/104_update_from_gitlab.sh
EOF
  exit 2
fi

set +u
source /opt/robot/scripts/setup_ros2.sh
set -u

SSH_KEY="${M20PRO_GIT_SSH_KEY:-/home/user/.ssh/id_ed25519_m20pro_test_gitlab}"
if [[ -f "${SSH_KEY}" ]]; then
  export GIT_SSH_COMMAND="${GIT_SSH_COMMAND:-ssh -i ${SSH_KEY} -o IdentitiesOnly=yes -o StrictHostKeyChecking=accept-new}"
else
  export GIT_SSH_COMMAND="${GIT_SSH_COMMAND:-ssh -o StrictHostKeyChecking=accept-new}"
fi

if [[ ! -d "${WS_DIR}" ]]; then
  mkdir -p "${WS_DIR}"
fi

cd "${WS_DIR}"
git config --global --add safe.directory "${WS_DIR}" >/dev/null 2>&1 || true

if [[ ! -d .git ]]; then
  echo "[104_update_from_gitlab] no git repo at ${WS_DIR}; cloning ${REMOTE_URL}"
  tmp="${WS_DIR}.clone_tmp.$$"
  backup="${WS_DIR}.before_git.$(date +%Y%m%d_%H%M%S)"
  rm -rf "${tmp}"
  git clone --branch "${BRANCH}" "${REMOTE_URL}" "${tmp}"
  if find "${WS_DIR}" -mindepth 1 -maxdepth 1 | grep -q .; then
    echo "[104_update_from_gitlab] backing up existing non-git workspace to ${backup}"
    mv "${WS_DIR}" "${backup}"
    mkdir -p "${WS_DIR}"
  fi
  shopt -s dotglob nullglob
  mv "${tmp}"/* "${WS_DIR}/"
  rmdir "${tmp}"
  cd "${WS_DIR}"
fi

if ! git remote get-url "${REMOTE}" >/dev/null 2>&1; then
  git remote add "${REMOTE}" "${REMOTE_URL}"
fi

current_url="$(git remote get-url "${REMOTE}")"
if [[ "${current_url}" != "${REMOTE_URL}" ]]; then
  git remote set-url "${REMOTE}" "${REMOTE_URL}"
fi

if [[ -n "$(git status --porcelain)" ]]; then
  echo "[104_update_from_gitlab] local worktree is dirty; refusing to overwrite." >&2
  git status --short >&2
  exit 3
fi

echo "[104_update_from_gitlab] fetching ${REMOTE}/${BRANCH}"
git fetch "${REMOTE}" "${BRANCH}"
git checkout "${BRANCH}"
git reset --hard "${REMOTE}/${BRANCH}"

if [[ -n "${PACKAGES}" ]]; then
  echo "[104_update_from_gitlab] building selected packages: ${PACKAGES}"
  colcon build --packages-select ${PACKAGES} --symlink-install
else
  echo "[104_update_from_gitlab] building full workspace"
  colcon build --symlink-install
fi

chmod +x scripts/*.sh src/m20pro_bringup/scripts/*.sh 2>/dev/null || true

if [[ "${RESTART_SERVICE}" == "1" ]]; then
  echo "[104_update_from_gitlab] restarting m20pro-real.service"
  systemctl restart m20pro-real.service
fi

echo "[104_update_from_gitlab] done at $(date '+%F %T')"
git log -1 --oneline --decorate
