#!/usr/bin/env bash
set -euo pipefail

kill_pattern() {
  local pattern="$1"
  mapfile -t pids < <(ps -eo pid=,args= | awk -v pat="${pattern}" '$0 ~ pat && $0 !~ /awk/ {print $1}')
  if [[ "${#pids[@]}" -eq 0 ]]; then
    return 0
  fi
  echo "[104_stop_real] stopping ${pattern}: ${pids[*]}"
  kill "${pids[@]}" 2>/dev/null || true
}

kill_pattern 'ros2 launch m20pro_bringup m20pro.launch.py'
kill_pattern 'm20pro_real_full.sh'
kill_pattern 'm20pro_navigation/lidar_relay'
kill_pattern 'm20pro_navigation lidar_relay'

sleep 1
rm -f /tmp/m20pro_lidar_relay.pid /tmp/m20pro_lidar_relay2.pid

echo "[104_stop_real] remaining real launch processes:"
ps -eo pid,args | awk '/m20pro_real_full|m20pro.launch.py|lidar_relay/ && !/awk/ {print}' || true
