#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
MODE="${1:-move}"
TIMEOUT_S="${2:-45}"
WEB_URL="${M20PRO_WEB_URL:-http://127.0.0.1:8080}"
REMOTE_MODE="${M20PRO_PREFLIGHT_REMOTE:-auto}"
REMOTE_TARGET="${M20PRO_PREFLIGHT_SSH_TARGET:-${M20PRO_104_SSH_TARGET:-user@10.21.31.104}}"

sh_quote() {
  printf "'%s'" "$(printf "%s" "$1" | sed "s/'/'\\\\''/g")"
}

is_local_104() {
  ip -br addr 2>/dev/null | grep -Eq '(^|[[:space:]])10\.21\.31\.104/'
}

if [[ "${REMOTE_MODE}" != "0" && "${REMOTE_MODE}" != "false" ]] && ! is_local_104; then
  if command -v ssh >/dev/null 2>&1 \
    && [[ -r "$0" ]] \
    && timeout 5 ssh -o BatchMode=yes -o ConnectTimeout=3 "${REMOTE_TARGET}" "true" >/dev/null 2>&1; then
    remote_cmd="cd $(sh_quote "${WS_DIR}") && \
M20PRO_PREFLIGHT_REMOTE=0 \
M20PRO_WS=$(sh_quote "${WS_DIR}") \
M20PRO_WEB_URL=$(sh_quote "${WEB_URL}") \
bash -s $(sh_quote "${MODE}") $(sh_quote "${TIMEOUT_S}")"
    echo "[104_preflight_check] running current local script on ${REMOTE_TARGET}; set M20PRO_PREFLIGHT_REMOTE=0 to force local mode"
    exec ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_TARGET}" "${remote_cmd}" <"$0"
  elif [[ "${REMOTE_MODE}" == "1" || "${REMOTE_MODE}" == "true" ]]; then
    echo "[104_preflight_check] cannot reach ${REMOTE_TARGET} or cannot read local script" >&2
    exit 1
  else
    echo "[104_preflight_check] ${REMOTE_TARGET} not reachable by SSH; continuing in local mode" >&2
  fi
fi

if [[ "${MODE}" != "move" && "${MODE}" != "shadow" ]]; then
  echo "Usage: $0 [move|shadow] [timeout_s]" >&2
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
PROJECT_FASTDDS="${WS_DIR}/install/m20pro_bringup/share/m20pro_bringup/config/m20pro_fastdds_udp.xml"
if [[ -f "${PROJECT_FASTDDS}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${PROJECT_FASTDDS}"
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
  python3 - "${timeout_s}" "${topic}" \
    >/tmp/m20pro_preflight_topic.out \
    2>/tmp/m20pro_preflight_topic.err <<'PY' || true
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String


timeout_s = float(sys.argv[1])
topic = sys.argv[2]


def best_effort_qos(depth=10):
    profile = QoSProfile(depth=depth)
    profile.history = HistoryPolicy.KEEP_LAST
    profile.reliability = ReliabilityPolicy.BEST_EFFORT
    profile.durability = DurabilityPolicy.VOLATILE
    return profile


def reliable_qos(depth=10):
    profile = QoSProfile(depth=depth)
    profile.history = HistoryPolicy.KEEP_LAST
    profile.reliability = ReliabilityPolicy.RELIABLE
    profile.durability = DurabilityPolicy.VOLATILE
    return profile


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


TOPICS = {
    "/m20pro/lidar_points_relay": (PointCloud2, reliable_qos()),
    "/scan": (LaserScan, best_effort_qos()),
    "/m20pro_tcp_bridge/map_pose": (PoseStamped, reliable_qos()),
    "/ODOM": (Odometry, reliable_qos()),
    "/m20pro_tcp_bridge/localization_ok": (Bool, reliable_qos()),
    "/m20pro_tcp_bridge/navigation_status": (String, reliable_qos()),
}
msg_type, qos = TOPICS.get(topic, (String, reliable_qos()))


class Probe(Node):
    def __init__(self):
        super().__init__("m20pro_preflight_topic_probe")
        self.sample = None
        self.create_subscription(msg_type, topic, self._on_msg, qos)

    def _on_msg(self, msg):
        if self.sample is None:
            self.sample = msg


def describe(msg):
    if isinstance(msg, PointCloud2):
        return "points: %d\nframe_id: %s" % (int(msg.width) * max(1, int(msg.height)), msg.header.frame_id)
    if isinstance(msg, LaserScan):
        finite = sum(1 for value in msg.ranges if math.isfinite(value))
        return "ranges: %d\nfinite_ranges: %d\nframe_id: %s" % (len(msg.ranges), finite, msg.header.frame_id)
    if isinstance(msg, PoseStamped):
        p = msg.pose.position
        yaw = yaw_from_quat(msg.pose.orientation)
        return "x: %r\ny: %r\nz: %r\nyaw: %r\nframe_id: %s" % (p.x, p.y, p.z, yaw, msg.header.frame_id)
    if isinstance(msg, Odometry):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        return "x: %r\ny: %r\nz: %r\nyaw: %r\nframe_id: %s" % (p.x, p.y, p.z, yaw, msg.header.frame_id)
    if isinstance(msg, Bool):
        return "data: %s" % ("true" if msg.data else "false")
    if isinstance(msg, String):
        return "data: %s" % msg.data
    return str(msg)


rclpy.init()
node = Probe()
deadline = time.time() + timeout_s
while time.time() < deadline and node.sample is None:
    rclpy.spin_once(node, timeout_sec=0.2)
if node.sample is not None:
    print(describe(node.sample), flush=True)
    result = 0
else:
    result = 1
node.destroy_node()
rclpy.shutdown()
raise SystemExit(result)
PY
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
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "ros_cli_fastdds=${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi
echo "基础自检用于确认全量系统、网页、感知链路和原厂状态链路。"
echo "当前默认按自动场地判断处理：未重定位时 Nav2/costmap 可延后确认；电量只显示给操作员参考，不作为软件自检或任务启动条件。"
echo

wait_for_cmd "ROS graph available" timeout 5 ros2 node list || true

required_nodes=(
  m20pro_tcp_bridge
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
  /m20pro_tcp_bridge/navigation_status
  /map
)
navigation_topics=(
  /scan
  /ODOM
  /m20pro_tcp_bridge/map_pose
  /m20pro_tcp_bridge/localization_ok
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
for topic in "${navigation_topics[@]}"; do
  if topic_exists "$topic"; then
    pass "navigation topic ${topic}"
  else
    warn "navigation topic ${topic} not found; relocalize before judging navigation"
  fi
done

if topic_once 8 /m20pro/lidar_points_relay; then
  pass "/m20pro/lidar_points_relay has data"
else
  warn "/m20pro/lidar_points_relay has no data within 8s; if web scan is fresh, suspect DDS/profile mismatch before blaming the lidar"
fi

if topic_once 8 /scan; then
  pass "/scan has data"
else
  fail "/scan has no data within 8s; scan/lidar must be present even at the workstation"
fi

if topic_once 8 /m20pro_tcp_bridge/map_pose; then
  if contains_nonfinite /tmp/m20pro_preflight_topic.out; then
    warn "/m20pro_tcp_bridge/map_pose contains nan/inf; relocalize before task"
  else
    pass "/m20pro_tcp_bridge/map_pose is finite"
  fi
else
  warn "/m20pro_tcp_bridge/map_pose has no data within 8s; relocalize before task"
fi

if topic_once 8 /ODOM; then
  if contains_nonfinite /tmp/m20pro_preflight_topic.out; then
    warn "/ODOM contains nan/inf; relocalize before task"
  else
    pass "/ODOM is finite"
  fi
else
  warn "/ODOM has no data within 8s; relocalize before task"
fi

if topic_once 8 /m20pro_tcp_bridge/localization_ok; then
  if grep -Eq 'data:[[:space:]]*true' /tmp/m20pro_preflight_topic.out; then
    pass "localization_ok=true"
  else
    warn "localization_ok is not true; relocalize before task"
  fi
else
  warn "/m20pro_tcp_bridge/localization_ok has no data within 8s; relocalize before task"
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
    warn "${lifecycle_node} is not active; if localization_ok=false this is deferred by the startup gate, otherwise do not start moving"
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
pose = payload.get("pose") or {}
loc = payload.get("localization_ok")
topics = payload.get("topics") or {}
scan_topic = topics.get("scan") or {}
scan = payload.get("scan") or {}
scan_age = scan_topic.get("age_sec")
finite_ranges = int(scan.get("finite_ranges") or 0)
if scan_topic.get("available") and scan_age is not None and float(scan_age) <= 8.0 and finite_ranges > 0:
    print("[OK] web perception chain scan fresh: finite_ranges={} age={:.2f}s".format(
        finite_ranges,
        float(scan_age),
    ))
else:
    print("[FAIL] web scan is not fresh; scan/lidar must be present even at the workstation")
    raise SystemExit(2)
if not pose:
    print("[INFO] 工位/未重定位状态：web state has no valid pose yet; Nav2/costmap 可延后，点云和 /scan 仍必须正常")
    raise SystemExit(0)
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
