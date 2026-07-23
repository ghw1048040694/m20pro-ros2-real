#!/usr/bin/env bash
set -euo pipefail

# Use this on a robot when company GitLab is unreachable from the robot network.
# Gitee is the deployment mirror: it contains only the verified main branch and
# allows the robot's read-only SSH deploy key to read it.

export M20PRO_REMOTE="${M20PRO_REMOTE:-mirror}"
export M20PRO_BRANCH="${M20PRO_BRANCH:-main}"
export M20PRO_REMOTE_URL="${M20PRO_REMOTE_URL:-git@gitee.com:gggghw/m20pro-ros2-real.git}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/104_update_from_gitlab.sh"
