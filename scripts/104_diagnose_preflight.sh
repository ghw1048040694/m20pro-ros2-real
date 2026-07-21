#!/usr/bin/env bash
set -uo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
WEB_URL="${M20PRO_WEB_URL:-http://127.0.0.1:8080}"
SINCE="${M20PRO_DIAG_SINCE:-15 min ago}"
PREFLIGHT_TIMEOUT_S="${M20PRO_DIAG_PREFLIGHT_TIMEOUT_S:-45}"
REMOTE_MODE="${M20PRO_DIAG_REMOTE:-auto}"
REMOTE_TARGET="${M20PRO_DIAG_SSH_TARGET:-${M20PRO_104_SSH_TARGET:-user@10.21.31.104}}"

TMP_DIR="$(mktemp -d /tmp/m20pro_diag.XXXXXX)"
trap 'rm -rf "${TMP_DIR}"' EXIT

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
M20PRO_DIAG_REMOTE=0 \
M20PRO_WS=$(sh_quote "${WS_DIR}") \
M20PRO_WEB_URL=$(sh_quote "${WEB_URL}") \
M20PRO_DIAG_SINCE=$(sh_quote "${SINCE}") \
M20PRO_DIAG_PREFLIGHT_TIMEOUT_S=$(sh_quote "${PREFLIGHT_TIMEOUT_S}") \
bash -s"
    echo "[104_diagnose_preflight] running current local script on ${REMOTE_TARGET}; set M20PRO_DIAG_REMOTE=0 to force local mode"
    exec ssh -o BatchMode=yes -o ConnectTimeout=5 "${REMOTE_TARGET}" "${remote_cmd}" <"$0"
  elif [[ "${REMOTE_MODE}" == "1" || "${REMOTE_MODE}" == "true" ]]; then
    echo "[104_diagnose_preflight] cannot reach ${REMOTE_TARGET} or cannot read local script" >&2
    exit 1
  else
    echo "[104_diagnose_preflight] ${REMOTE_TARGET} not reachable by SSH; continuing in local mode" >&2
  fi
fi

section() {
  printf '\n==== %s ====\n' "$1"
}

run() {
  printf '+ %s\n' "$*"
  "$@" 2>&1 || true
}

parse_state() {
  python3 - "$1" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
if not path.exists() or path.stat().st_size == 0:
    print("state: no JSON captured")
    raise SystemExit(0)

state = json.loads(path.read_text(encoding="utf-8"))
topics = state.get("topics") or {}
print("floor=%s localization_ok=%s navigation_status=%s" % (
    state.get("floor"),
    state.get("localization_ok"),
    state.get("navigation_status"),
))
print("navigation_status_parsed=%s" % (state.get("navigation_status_parsed"),))
for key in (
    "lidar_points",
    "scan",
    "odom",
    "pose",
    "map",
    "local_costmap",
    "global_costmap",
    "battery",
):
    info = topics.get(key) or {}
    age = info.get("age_sec")
    age_text = "-" if age is None else ("%.3fs" % float(age))
    print("%-16s available=%s age=%s" % (key, info.get("available"), age_text))
lidar = state.get("lidar_points") or {}
scan = state.get("scan") or {}
print("lidar width=%s frame=%s source=%s" % (
    lidar.get("width"),
    lidar.get("frame_id"),
    lidar.get("source"),
))
print("scan ranges=%s finite=%s frame=%s" % (
    scan.get("ranges"),
    scan.get("finite_ranges"),
    scan.get("frame_id"),
))
preflight = state.get("preflight") or {}
if preflight:
    print("cached_preflight ok=%s warnings=%s nav_warnings=%s summary=%s" % (
        preflight.get("ok"),
        preflight.get("warnings"),
        preflight.get("navigation_warnings"),
        preflight.get("summary"),
    ))
PY
}

parse_preflight() {
  python3 - "$1" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
if not path.exists() or path.stat().st_size == 0:
    print("preflight: no JSON captured")
    raise SystemExit(0)

payload = json.loads(path.read_text(encoding="utf-8"))
preflight = payload.get("preflight") or payload
print("ok=%s navigation_ready=%s relocalization_ready=%s" % (
    preflight.get("ok"),
    preflight.get("navigation_ready"),
    preflight.get("relocalization_ready"),
))
print("failures=%s warnings=%s navigation_warnings=%s" % (
    preflight.get("failures"),
    preflight.get("warnings"),
    preflight.get("navigation_warnings"),
))
print("site=%s site_mode=%s workstation_mode=%s" % (
    preflight.get("site"),
    preflight.get("site_mode"),
    preflight.get("workstation_mode"),
))
print("summary=%s" % preflight.get("summary"))
for item in preflight.get("items") or []:
    if item.get("status") != "ok" or item.get("key") in (
        "lidar_points",
        "scan",
        "local_costmap",
        "global_costmap",
        "nav2_lifecycle_deferred",
    ):
        print("[%s] %-24s %s" % (
            item.get("status"),
            item.get("key"),
            item.get("message"),
        ))
PY
}

section "M20Pro preflight diagnosis"
echo "ws=${WS_DIR}"
echo "web=${WEB_URL}"
echo "logs_since=${SINCE}"
echo "note: this script is read-only; it does not publish cmd_vel, gait, or usage-mode commands."

section "Service"
run systemctl show -p ActiveState -p SubState -p NRestarts -p ExecMainPID m20pro-real.service
run systemctl is-enabled m20pro-real.service
run df -h /dev/shm

section "Network and git readiness"
run ip -br addr
run ip route
if command -v nmcli >/dev/null 2>&1; then
  run nmcli -t -f DEVICE,TYPE,STATE,CONNECTION dev status
fi
if command -v resolvectl >/dev/null 2>&1; then
  run resolvectl dns
elif command -v systemd-resolve >/dev/null 2>&1; then
  run systemd-resolve --status
fi
if ip route | grep -q '^default '; then
  echo "OK: default route is present"
else
  echo "WARNING: no default route; this robot cannot pull git over the internet until Wi-Fi/NAT/gateway is configured"
fi
if timeout 5 getent hosts github.com >/dev/null 2>&1; then
  echo "OK: github.com resolves"
else
  echo "WARNING: github.com does not resolve; DNS or upstream internet is not ready"
fi
if [[ -L "${WS_DIR}" ]]; then
  echo "workspace_symlink=$(readlink -f "${WS_DIR}" 2>/dev/null || true)"
fi
if [[ -d "${WS_DIR}/.git" ]]; then
  (
    cd "${WS_DIR}" || exit 0
    echo "git_repo=yes"
    git log -1 --oneline --decorate 2>/dev/null || true
    git status --short 2>/dev/null || true
    git remote -v 2>/dev/null || true
  )
else
  echo "git_repo=no"
  echo "WARNING: ${WS_DIR} is not a git worktree; use local rsync deploy or convert/clone before expecting git pull to work here"
fi

section "Web health"
run curl -fsS --max-time 5 "${WEB_URL}/healthz"

section "Web state"
STATE_JSON="${TMP_DIR}/state.json"
if curl -fsS --max-time 8 "${WEB_URL}/api/state" -o "${STATE_JSON}" 2>"${TMP_DIR}/state.err"; then
  parse_state "${STATE_JSON}"
else
  cat "${TMP_DIR}/state.err"
fi

section "Blocking web preflight"
PREFLIGHT_JSON="${TMP_DIR}/preflight.json"
if curl -fsS --max-time "${PREFLIGHT_TIMEOUT_S}" \
  -H 'Content-Type: application/json' \
  -d '{"mode":"move","site":"auto","wait":true}' \
  "${WEB_URL}/api/preflight/run" \
  -o "${PREFLIGHT_JSON}" 2>"${TMP_DIR}/preflight.err"; then
  parse_preflight "${PREFLIGHT_JSON}"
else
  cat "${TMP_DIR}/preflight.err"
fi

section "Selected ROS graph"
M20PRO_WS="${WS_DIR}" bash <<'BASH' || true
set +euo pipefail
WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
if [[ -f /opt/robot/scripts/setup_ros2.sh ]]; then
  # shellcheck disable=SC1091
  source /opt/robot/scripts/setup_ros2.sh >/dev/null 2>&1 || true
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true
fi
if [[ -d "${WS_DIR}" ]]; then
  cd "${WS_DIR}" || true
  if [[ -f install/setup.bash ]]; then
    # shellcheck disable=SC1091
    source install/setup.bash >/dev/null 2>&1 || true
  fi
fi
PROJECT_FASTDDS="${WS_DIR}/install/m20pro_bringup/share/m20pro_bringup/config/m20pro_fastdds_udp.xml"
if [[ -f "${PROJECT_FASTDDS}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${PROJECT_FASTDDS}"
  echo "[ros cli] using project UDP FastDDS profile: ${PROJECT_FASTDDS}"
  echo "[ros cli] this avoids root/user SHM split when observing relay pointcloud from a developer shell"
elif [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "[ros cli] using inherited FastDDS profile: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
else
  echo "[ros cli] no project UDP FastDDS profile found; ROS CLI pointcloud checks may be weaker"
fi
string_topic_sample() {
  local topic="$1"
  python3 - "${topic}" <<'PY' 2>/dev/null
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Probe(Node):
    def __init__(self, topic):
        super().__init__("m20pro_diag_string_probe")
        self.sample = None
        self.create_subscription(String, topic, self._on_msg, 10)

    def _on_msg(self, msg):
        if self.sample is None:
            self.sample = msg.data


rclpy.init()
node = Probe(sys.argv[1])
deadline = time.time() + 5.0
while time.time() < deadline and node.sample is None:
    rclpy.spin_once(node, timeout_sec=0.2)
if node.sample is not None:
    print(node.sample)
    result = 0
else:
    result = 1
node.destroy_node()
rclpy.shutdown()
raise SystemExit(result)
PY
}
if command -v ros2 >/dev/null 2>&1; then
  echo "[topics]"
  ros2 topic list 2>/dev/null \
    | grep -E '^/(scan|ODOM|map|tf|local_costmap/costmap|global_costmap/costmap|m20pro/cmd_vel_mux/status|m20pro_tcp_bridge/(navigation_status|localization_ok|map_pose|usage_mode_result))$' \
    | sort || true
  echo
  echo "[edge scan topic info]"
  timeout 5 ros2 topic info /scan 2>/dev/null || echo "/scan not present"
  echo
  echo "[nodes]"
  ros2 node list 2>/dev/null \
    | grep -E '^/(m20pro_nav2_startup_gate|m20pro_tcp_bridge|m20pro_command_mux|m20pro_web_dashboard|controller_server|planner_server|bt_navigator|map_server|m20pro_floor_manager)$' \
    | sort || true
else
  echo "ros2 command not available in this shell"
fi
BASH

section "Usage-mode safety"
if [[ -f "${WS_DIR}/src/m20pro_bringup/config/m20pro_real.yaml" ]]; then
  grep -n 'enable_usage_mode_command' "${WS_DIR}/src/m20pro_bringup/config/m20pro_real.yaml" || true
fi
if [[ -f "${WS_DIR}/src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py" ]]; then
  if grep -Eq 'api/usage_mode|data-usage-mode|setUsageMode' "${WS_DIR}/src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py"; then
    echo "WARNING: usage-mode web control text found"
  else
    echo "OK: no web usage-mode control route/button found"
  fi
fi

section "Recent relevant logs"
journalctl -u m20pro-real.service --since "${SINCE}" --no-pager 2>/dev/null \
  | grep -Ei 'camera proxy: failed to open RTSP stream|rtsp .*404 Not Found' \
  | tail -n 6 \
  | sed 's/^/[camera proxy] /' || true
journalctl -u m20pro-real.service --since "${SINCE}" --no-pager 2>/dev/null \
  | grep -Ei 'relay sample OK|PointCloud fusion ready|Nav2 startup gate|Nav2 prerequisites|Nav2 lifecycle|M20PRO REAL OK|M20PRO REAL WAITING|local_costmap.*(Configuring|Subscribed|Activating|start)|global_costmap.*(Configuring|Subscribed|Activating|start)|usage mode|OOA|ControlUsageMode|Traceback|ERROR|failed|timeout|WARN' \
  | grep -Evi 'rtsp .*404 Not Found|camera proxy: failed to open RTSP stream' \
  | tail -n 220 || true

section "Done"
echo "If web preflight is green but plain ROS CLI cannot see lidar samples, rerun with the project UDP FastDDS profile or this script; factory SHM can split root service data from a user shell."
