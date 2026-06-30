#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_ros2_ws}"
MODE="${M20PRO_REAL_MODE:-move}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "m20pro-real.service must run as root." >&2
  exit 2
fi

case "${MODE}" in
  move)
    exec "${WS_DIR}/scripts/104_start_real_move.sh"
    ;;
  shadow)
    exec "${WS_DIR}/scripts/104_start_real_shadow.sh"
    ;;
  *)
    echo "invalid M20PRO_REAL_MODE=${MODE}; expected move or shadow" >&2
    exit 2
    ;;
esac
