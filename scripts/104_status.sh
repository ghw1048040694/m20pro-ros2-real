#!/usr/bin/env bash
set -euo pipefail

echo "[104_status] disk:"
df -h /home/user 2>/dev/null || df -h .

echo
echo "[104_status] web/real processes:"
ps -eo pid,args | awk '/web_dashboard|m20pro_real_full|m20pro.launch.py/ && !/awk/ {print}'

echo
echo "[104_status] listening on 8080:"
ss -ltnp 2>/dev/null | grep ':8080' || true

echo
echo "[104_status] lidar sample check:"
if command -v ros2 >/dev/null 2>&1; then
  M20PRO_LIDAR_WAIT_S="${M20PRO_LIDAR_STATUS_WAIT_S:-6}" ros2 run m20pro_bringup m20pro_lidar_guard.sh check || true
else
  echo "ros2 not found; run after the fixed 104 source -> su environment"
fi

echo
echo "[104_status] quick health:"
curl -fsS http://127.0.0.1:8080/healthz 2>/dev/null || true
echo

echo
echo "[104_status] active web task:"
tasks_json="$(curl -fsS http://127.0.0.1:8080/api/tasks 2>/dev/null || true)"
TASKS_JSON="${tasks_json}" python3 - <<'PY' || true
import json
import os
import sys

try:
    text = os.environ.get("TASKS_JSON", "")
    if not text:
        sys.exit(0)
    payload = json.loads(text)
except Exception:
    sys.exit(0)
active = payload.get("active_task")
if not active:
    print("none")
else:
    print(json.dumps(active, ensure_ascii=False, separators=(",", ":")))
PY

echo
echo "[104_status] latest tcp bridge motion mode:"
search_dirs=()
if [[ -d "${HOME}/.ros/log" && -r "${HOME}/.ros/log" ]]; then
  search_dirs+=("${HOME}/.ros/log")
fi
if [[ -d /root/.ros/log && -r /root/.ros/log ]]; then
  search_dirs+=("/root/.ros/log")
fi
latest_log="$(
  {
    if [[ "${#search_dirs[@]}" -gt 0 ]]; then
      find "${search_dirs[@]}" -type f -name '*.log' -printf '%T@ %p\n' 2>/dev/null
    fi
  } \
    | sort -nr \
    | awk '{print $2}' \
    | while read -r file; do
        if grep -qE 'axis command enabled|shadow mode; axis command disabled' "$file" 2>/dev/null; then
          echo "$file"
          break
        fi
      done
)" || true
if [[ -n "${latest_log}" ]]; then
  grep -E 'axis command enabled|shadow mode; axis command disabled' "${latest_log}" | tail -1
else
  echo "unknown; start real first, or run this script after su to read root launch logs"
fi
