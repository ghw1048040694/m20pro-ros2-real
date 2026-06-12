#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_ros2_ws}"
MODE="${1:-move}"
TIMEOUT_S="${2:-45}"
WEB_URL="${M20PRO_WEB_URL:-http://127.0.0.1:8080}"

if [[ "${MODE}" != "move" && "${MODE}" != "shadow" ]]; then
  echo "Usage: $0 [move|shadow] [timeout_s]" >&2
  exit 2
fi

if [[ "${EUID}" -ne 0 ]]; then
  cat >&2 <<'EOF'
Run this after su on 104:

  ssh user@10.21.31.104
  source /opt/robot/scripts/setup_ros2.sh
  su
  cd /home/user/m20pro_ros2_ws
  source install/setup.bash
  ./scripts/104_preflight_check.sh move
EOF
  exit 2
fi

set +u
source /opt/robot/scripts/setup_ros2.sh
set -u

cd "${WS_DIR}"
if [[ -f install/setup.bash ]]; then
  set +u
  source install/setup.bash
  set -u
fi

deadline=$((SECONDS + TIMEOUT_S))
failures=()
warnings=()

pass() {
  printf '[OK] %s\n' "$1"
}

warn() {
  warnings+=("$1")
  printf '[WARN] %s\n' "$1"
}

fail() {
  failures+=("$1")
  printf '[FAIL] %s\n' "$1"
}

wait_for_cmd() {
  local label="$1"
  shift
  while (( SECONDS < deadline )); do
    if "$@" >/tmp/m20pro_preflight_cmd.out 2>/tmp/m20pro_preflight_cmd.err; then
      pass "${label}"
      return 0
    fi
    sleep 1
  done
  fail "${label}: $(tr '\n' ' ' </tmp/m20pro_preflight_cmd.err | sed 's/[[:space:]][[:space:]]*/ /g' | cut -c1-160)"
  return 1
}

topic_exists() {
  ros2 topic list | grep -qx "$1"
}

node_exists() {
  ros2 node list | sed 's#^/##' | grep -qx "${1#/}"
}

topic_once() {
  local timeout_s="$1"
  local topic="$2"
  : >/tmp/m20pro_preflight_topic.out
  : >/tmp/m20pro_preflight_topic.err
  timeout "${timeout_s}" bash -c '
    ros2 topic echo "$1" --no-arr 2>"$2" \
      | awk "{ print; if (\$0 == \"---\") exit 0 }" >"$3"
  ' _ "${topic}" /tmp/m20pro_preflight_topic.err /tmp/m20pro_preflight_topic.out || true
  [[ -s /tmp/m20pro_preflight_topic.out ]]
}

contains_nonfinite() {
  python3 - "$1" <<'PY'
import math
import re
import sys
from pathlib import Path

text = Path(sys.argv[1]).read_text(errors="ignore")
for match in re.finditer(r"(?i)(?<![A-Za-z0-9_])[-+]?(?:\.inf|inf|infinity|nan)(?![A-Za-z0-9_])", text):
    raise SystemExit(0)
raise SystemExit(1)
PY
}

lifecycle_active() {
  local node="$1"
  local output
  output="$(ros2 lifecycle get "$node" 2>/tmp/m20pro_preflight_lifecycle.err || true)"
  grep -q 'active' <<<"${output}"
}

echo "M20Pro preflight check"
echo "mode=${MODE} timeout=${TIMEOUT_S}s web=${WEB_URL}"
echo

wait_for_cmd "ROS graph available" timeout 5 ros2 node list || true

required_nodes=(
  m20pro_tcp_bridge
  m20pro_pointcloud_fusion
  m20pro_web_dashboard
  map_server
  controller_server
  planner_server
  bt_navigator
  m20pro_floor_manager
)
for node in "${required_nodes[@]}"; do
  if node_exists "$node"; then
    pass "node /${node}"
  else
    fail "node /${node} not found; start full real first"
  fi
done

required_topics=(
  /LIDAR/POINTS
  /scan
  /ODOM
  /m20pro_tcp_bridge/map_pose
  /m20pro_tcp_bridge/localization_ok
  /m20pro_tcp_bridge/navigation_status
  /map
  /local_costmap/costmap
  /global_costmap/costmap
)
for topic in "${required_topics[@]}"; do
  if topic_exists "$topic"; then
    pass "topic ${topic}"
  else
    fail "topic ${topic} not found"
  fi
done

if topic_once 12 /LIDAR/POINTS; then
  pass "/LIDAR/POINTS has data"
else
  fail "/LIDAR/POINTS has no data within 12s"
fi

if topic_once 8 /scan; then
  pass "/scan has data"
else
  fail "/scan has no data within 8s"
fi

if topic_once 8 /m20pro_tcp_bridge/map_pose; then
  if contains_nonfinite /tmp/m20pro_preflight_topic.out; then
    fail "/m20pro_tcp_bridge/map_pose contains nan/inf"
  else
    pass "/m20pro_tcp_bridge/map_pose is finite"
  fi
else
  fail "/m20pro_tcp_bridge/map_pose has no data within 8s"
fi

if topic_once 8 /ODOM; then
  if contains_nonfinite /tmp/m20pro_preflight_topic.out; then
    fail "/ODOM contains nan/inf; relocalize before task"
  else
    pass "/ODOM is finite"
  fi
else
  fail "/ODOM has no data within 8s"
fi

if topic_once 8 /m20pro_tcp_bridge/localization_ok; then
  if grep -Eq 'data:[[:space:]]*true' /tmp/m20pro_preflight_topic.out; then
    pass "localization_ok=true"
  else
    fail "localization_ok is not true; relocalize before task"
  fi
else
  fail "/m20pro_tcp_bridge/localization_ok has no data within 8s"
fi

if topic_once 8 /m20pro_tcp_bridge/navigation_status; then
  status="$(tr '\n' ' ' </tmp/m20pro_preflight_topic.out | sed 's/[[:space:]][[:space:]]*/ /g')"
  pass "navigation_status received: ${status:0:120}"
else
  warn "/m20pro_tcp_bridge/navigation_status has no data within 8s"
fi

for lifecycle_node in /map_server /controller_server /planner_server /bt_navigator; do
  if lifecycle_active "$lifecycle_node"; then
    pass "${lifecycle_node} active"
  else
    fail "${lifecycle_node} is not active"
  fi
done

if curl -fsS "${WEB_URL}/healthz" >/tmp/m20pro_preflight_web.out 2>/tmp/m20pro_preflight_web.err; then
  pass "web healthz OK"
else
  fail "web healthz failed: ${WEB_URL}/healthz"
fi

if curl -fsS "${WEB_URL}/api/state" >/tmp/m20pro_preflight_state.json 2>/tmp/m20pro_preflight_web.err; then
  python3 - <<'PY' || fail "web /api/state parse failed"
import json
from pathlib import Path

payload = json.loads(Path("/tmp/m20pro_preflight_state.json").read_text())
pose = payload.get("pose")
if not pose:
    raise SystemExit("no pose")
loc = payload.get("localization_ok")
print("[OK] web state pose x={:.3f} y={:.3f} yaw={:.3f} localization_ok={}".format(
    float(pose.get("x", 0.0)),
    float(pose.get("y", 0.0)),
    float(pose.get("yaw", 0.0)),
    loc,
))
PY
else
  fail "web /api/state failed"
fi

launch_args="$(ps -eo args | grep 'ros2 launch m20pro_bringup m20pro.launch.py' | grep -v grep | tail -1 || true)"
mode_line=""
search_dirs=()
if [[ -d "${HOME}/.ros/log" && -r "${HOME}/.ros/log" ]]; then
  search_dirs+=("${HOME}/.ros/log")
fi
if [[ -d /root/.ros/log && -r /root/.ros/log ]]; then
  search_dirs+=("/root/.ros/log")
fi
if [[ "${#search_dirs[@]}" -gt 0 ]]; then
  latest_log="$(
    find "${search_dirs[@]}" -type f -name '*.log' -printf '%T@ %p\n' 2>/dev/null \
      | sort -nr \
      | awk '{print $2}' \
      | while read -r file; do
          if grep -qE 'axis command enabled|shadow mode; axis command disabled' "$file" 2>/dev/null; then
            echo "$file"
            break
          fi
        done
  )" || true
  if [[ -n "${latest_log:-}" ]]; then
    mode_line="$(grep -E 'axis command enabled|shadow mode; axis command disabled' "${latest_log}" | tail -1 || true)"
  fi
fi

if [[ "${MODE}" == "move" ]]; then
  if grep -q 'enable_axis_command:=true' <<<"${launch_args}" || grep -q 'axis command enabled' <<<"${mode_line}"; then
    pass "motion mode is move: axis command enabled"
  else
    fail "motion mode is not confirmed as move; restart with ./scripts/104_start_real_move.sh"
  fi
else
  if grep -q 'enable_axis_command:=false' <<<"${launch_args}" || grep -q 'shadow mode; axis command disabled' <<<"${mode_line}"; then
    pass "motion mode is shadow: axis command disabled"
  else
    warn "shadow mode not confirmed from latest logs"
  fi
fi

echo
if [[ "${#warnings[@]}" -gt 0 ]]; then
  echo "Warnings:"
  printf '  - %s\n' "${warnings[@]}"
fi
if [[ "${#failures[@]}" -gt 0 ]]; then
  echo "M20PRO PREFLIGHT FAIL"
  printf '  - %s\n' "${failures[@]}"
  exit 1
fi

echo "M20PRO PREFLIGHT OK"
