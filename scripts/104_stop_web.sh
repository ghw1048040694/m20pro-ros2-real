#!/usr/bin/env bash
set -euo pipefail

kill_pattern() {
  local pattern="$1"
  mapfile -t pids < <(ps -eo pid=,args= | awk -v pat="${pattern}" '$0 ~ pat && $0 !~ /awk/ {print $1}')
  if [[ "${#pids[@]}" -eq 0 ]]; then
    return 0
  fi
  echo "[104_stop_web] stopping ${pattern}: ${pids[*]}"
  kill "${pids[@]}" 2>/dev/null || true
}

kill_pattern 'm20pro_web_dashboard.launch.py'
kill_pattern 'ros2 run m20pro_cloud_bridge web_dashboard'
kill_pattern '/m20pro_cloud_bridge/web_dashboard'

sleep 1

echo "[104_stop_web] remaining web processes:"
ps -eo pid,args | awk '/web_dashboard/ && !/awk/ {print}' || true

echo "[104_stop_web] port 8080:"
ss -ltnp 2>/dev/null | grep ':8080' || true
