#!/usr/bin/env bash
set -euo pipefail

echo "[m20pro_runtime_snapshot] time=$(date '+%F %T')"
echo "[m20pro_runtime_snapshot] user=$(id -un) uid=$(id -u)"
echo "[m20pro_runtime_snapshot] FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}"
echo "[m20pro_runtime_snapshot] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "[m20pro_runtime_snapshot] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"

if command -v df >/dev/null 2>&1; then
  df -h /dev/shm 2>/dev/null | sed 's/^/[m20pro_runtime_snapshot] /' || true
fi

if [[ -d /dev/shm ]]; then
  echo "[m20pro_runtime_snapshot] largest /dev/shm entries:"
  du -sh /dev/shm/* 2>/dev/null | sort -h | tail -n 12 | sed 's/^/[m20pro_runtime_snapshot] /' || true
fi

if command -v ros2 >/dev/null 2>&1; then
  echo "[m20pro_runtime_snapshot] selected topics:"
  timeout 5s ros2 topic list 2>/dev/null \
    | grep -E '^/(LIDAR/POINTS2?|m20pro/lidar_points2?_relay|scan|map|odom|ODOM|tf|local_costmap/costmap|global_costmap/costmap)$' \
    | sort \
    | sed 's/^/[m20pro_runtime_snapshot] /' || true
fi
