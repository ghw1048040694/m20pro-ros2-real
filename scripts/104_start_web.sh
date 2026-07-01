#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
PORT="${1:-8080}"

cat >&2 <<'EOF'
[104_start_web] 仅用于开发预览网页界面。
[104_start_web] 真机测试不要用它；请使用 104_start_real_shadow.sh 或 104_start_real_move.sh 全量启动。
[104_start_web] 单独网页不会拉起 tcp_bridge/Nav2/点云融合，不能作为重定位、标点、下发任务的现场流程。
EOF

set +u
source /opt/robot/scripts/setup_ros2.sh
set -u

cd "${WS_DIR}"
set +u
source install/setup.bash
set -u

exec ros2 launch m20pro_bringup m20pro_web_dashboard.launch.py \
  host:=0.0.0.0 \
  port:="${PORT}" \
  enable_camera_proxy:=false
