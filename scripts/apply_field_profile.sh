#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${M20PRO_104_SSH_TARGET:-user@10.21.31.104}"
WEB_HOST="${HOST#*@}"
WEB_URL="${M20PRO_WEB_URL:-http://${WEB_HOST}:8080}"

"${ROOT_DIR}/scripts/m20pro_field_profile.py" check

case "${1:---apply}" in
  --check)
    exit 0
    ;;
  --apply)
    ;;
  *)
    echo "Usage: ./scripts/apply_field_profile.sh [--check|--apply]" >&2
    exit 2
    ;;
esac

state="$(curl --connect-timeout 2 --max-time 5 -fsS "${WEB_URL}/api/state")" || {
  echo "cannot confirm that 104 is idle at ${WEB_URL}; profile was not applied" >&2
  exit 3
}
STATE="${state}" python3 -c '
import json
import os
import sys

state = json.loads(os.environ["STATE"])
active = state.get("active_task")
if isinstance(active, dict) and active.get("status") in ("running", "starting", "stopping"):
    print("an active task is running; field profile was not applied", file=sys.stderr)
    sys.exit(4)
'

exec "${ROOT_DIR}/scripts/local_deploy_to_test_robot.sh" "${HOST}"
