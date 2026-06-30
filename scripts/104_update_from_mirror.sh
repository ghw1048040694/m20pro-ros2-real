#!/usr/bin/env bash
set -euo pipefail

# Use this on the test robot when company GitLab is unreachable from the robot
# network. The mirror repo must contain the same main branch and must allow the
# robot's SSH deploy key to read it.

export M20PRO_REMOTE="${M20PRO_REMOTE:-mirror}"
export M20PRO_BRANCH="${M20PRO_BRANCH:-main}"
export M20PRO_REMOTE_URL="${M20PRO_REMOTE_URL:-git@github.com:ghw1048040694/m20pro-ros2-real.git}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/104_update_from_gitlab.sh"
