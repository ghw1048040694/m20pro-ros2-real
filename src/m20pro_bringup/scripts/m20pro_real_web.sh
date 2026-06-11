#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_ros2_ws}"
PORT="${1:-8080}"

cat >&2 <<'EOF'
[m20pro_real_web] 仅用于开发预览网页界面。
[m20pro_real_web] 真机测试请使用 m20pro_real_full.sh shadow/move 全量启动。
[m20pro_real_web] 单独网页不会拉起 tcp_bridge/Nav2/点云融合。
EOF

set +u
source /opt/robot/scripts/setup_ros2.sh
set -u
cd "${WS_DIR}"
set +u
source install/setup.bash
set -u

exec ros2 launch m20pro_bringup m20pro_web_dashboard.launch.py \
  port:="${PORT}" \
  enable_camera_proxy:=false
