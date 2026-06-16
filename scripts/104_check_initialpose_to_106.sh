#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_ros2_ws}"
FACTORY_HOST="${M20PRO_FACTORY_HOST:-10.21.31.106}"
FACTORY_USER="${M20PRO_FACTORY_USER:-user}"
X="${1:-0.0}"
Y="${2:-0.0}"
YAW="${3:-0.0}"

if [[ "${EUID}" -ne 0 ]]; then
  cat >&2 <<'EOF'
Run this after su on 104:

  ssh user@10.21.31.104
  source /opt/robot/scripts/setup_ros2.sh
  su
  cd /home/user/m20pro_ros2_ws
  source install/setup.bash
  ./scripts/104_check_initialpose_to_106.sh <x> <y> <yaw>

EOF
  exit 2
fi

set +u
source /opt/robot/scripts/setup_ros2.sh
set -u

cd "${WS_DIR}"
set +u
source install/setup.bash
set -u

tmp="/tmp/m20pro_initialpose_106_$$.log"
rm -f "${tmp}"

echo "[initialpose_check] listening on ${FACTORY_HOST} for one /initialpose message..."
ssh -o BatchMode=yes -o ConnectTimeout=5 "${FACTORY_USER}@${FACTORY_HOST}" \
  "bash -lc 'source /opt/robot/scripts/setup_ros2.sh >/dev/null 2>&1 || source /opt/ros/foxy/setup.bash; timeout 6 ros2 topic echo /initialpose --once --no-arr'" \
  >"${tmp}" 2>&1 &
listener_pid=$!

sleep 1
echo "[initialpose_check] publishing /initialpose from 104: x=${X}, y=${Y}, yaw=${YAW}"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: map},
  pose: {
    pose: {
      position: {x: ${X}, y: ${Y}, z: 0.0},
      orientation: {z: $(python3 - <<PY
import math
print(math.sin(float("${YAW}") * 0.5))
PY
), w: $(python3 - <<PY
import math
print(math.cos(float("${YAW}") * 0.5))
PY
)}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]
  }
}" >/dev/null

if wait "${listener_pid}"; then
  echo "[initialpose_check] OK: 106 received /initialpose"
  sed -n '1,80p' "${tmp}"
  rm -f "${tmp}"
  exit 0
fi

echo "[initialpose_check] FAIL: 106 did not receive /initialpose within 6 seconds"
sed -n '1,120p' "${tmp}" || true
rm -f "${tmp}"
exit 1
