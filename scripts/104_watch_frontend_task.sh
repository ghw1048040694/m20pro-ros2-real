#!/usr/bin/env bash
set -euo pipefail

if hostname -I 2>/dev/null | tr ' ' '\n' | grep -qx '10.21.31.104'; then
  DEFAULT_HOST="local"
  DEFAULT_WEB_URL="http://127.0.0.1:8080"
else
  DEFAULT_HOST="user@10.21.31.104"
  DEFAULT_WEB_URL="http://10.21.31.104:8080"
fi

HOST="${M20PRO_104_HOST:-${DEFAULT_HOST}}"
WEB_URL="${M20PRO_WEB_URL:-${DEFAULT_WEB_URL}}"
DURATION_S="${1:-300}"
LABEL="${2:-frontend_task}"
OUT_DIR="${M20PRO_TASK_WATCH_DIR:-./task_watch_logs}"

STAMP="$(date +%Y%m%d_%H%M%S)"
SAFE_LABEL="$(printf '%s' "${LABEL}" | tr -c 'A-Za-z0-9_.-' '_')"
RUN_DIR="${OUT_DIR}/${STAMP}_${SAFE_LABEL}"
mkdir -p "${RUN_DIR}"

STATE_JSONL="${RUN_DIR}/state.jsonl"
TASKS_JSONL="${RUN_DIR}/tasks.jsonl"
SUMMARY_TSV="${RUN_DIR}/summary.tsv"
JOURNAL_LOG="${RUN_DIR}/m20pro-real.journal.log"
META_TXT="${RUN_DIR}/meta.txt"
ANALYSIS_TXT="${RUN_DIR}/analysis.txt"
ANALYZER="${M20PRO_TASK_WATCH_ANALYZER:-$(dirname "${BASH_SOURCE[0]}")/104_analyze_frontend_task_watch.py}"

cat >"${META_TXT}" <<EOF
started_at=$(date '+%Y-%m-%d %H:%M:%S %Z')
host=${HOST}
web_url=${WEB_URL}
duration_s=${DURATION_S}
label=${LABEL}
run_dir=${RUN_DIR}
note=This watcher is read-only. It does not start tasks, publish goals, or send motion commands.
EOF

echo "[104_watch_frontend_task] read-only watcher"
echo "[104_watch_frontend_task] web=${WEB_URL}"
echo "[104_watch_frontend_task] journal=${HOST}"
echo "[104_watch_frontend_task] duration=${DURATION_S}s"
echo "[104_watch_frontend_task] output=${RUN_DIR}"
echo "[104_watch_frontend_task] start the task from the real web frontend now, if ready."

touch "${STATE_JSONL}" "${TASKS_JSONL}" "${SUMMARY_TSV}" "${JOURNAL_LOG}"
printf 'time\tlocalization\tpose\treadiness\tnav_ready\tlocal_costmap\tglobal_costmap\tbattery_level\tscan_finite\tlidar_points\tlidar_source\truntime_guard\truntime_guard_age\tactive_task\ttask_name\ttask_map\twaypoint_id\twaypoint\tphase\tnav\tdistance\tfirst_waypoint_distance\tnav_remaining\trecoveries\tnav_goal_seq\tgoal_attempt\tfloor_goal_published\tfloor_goal_publishes\tnav_match\tnav_match_reason\tgoal_sends\tresends\trobot_x\trobot_y\trobot_yaw\tgoal_x\tgoal_y\tgoal_yaw\tnav_pose_x\tnav_pose_y\tnav_pose_yaw\tpath_goal_error\tpath_points\tpath_raw_points\tgoal_sent_path_version\tplan_path_version\tplan_verified\trobot_goal_error\tnav_goal_error\trobot_nav_error\tstall_age\tlast_event\tlast_result\tmessage\n' >"${SUMMARY_TSV}"

if [ "${HOST}" = "local" ]; then
  journalctl -u m20pro-real.service -f --since now --no-pager >"${JOURNAL_LOG}" 2>&1 &
else
  ssh "${HOST}" "journalctl -u m20pro-real.service -f --since now --no-pager" >"${JOURNAL_LOG}" 2>&1 &
fi
JOURNAL_PID=$!

cleanup() {
  if kill -0 "${JOURNAL_PID}" >/dev/null 2>&1; then
    kill "${JOURNAL_PID}" >/dev/null 2>&1 || true
    wait "${JOURNAL_PID}" >/dev/null 2>&1 || true
  fi
  echo "finished_at=$(date '+%Y-%m-%d %H:%M:%S %Z')" >>"${META_TXT}"
  echo "[104_watch_frontend_task] saved ${RUN_DIR}"
}
trap cleanup EXIT INT TERM

python3 - "${WEB_URL}" "${DURATION_S}" "${STATE_JSONL}" "${TASKS_JSONL}" "${SUMMARY_TSV}" <<'PY'
import json
import math
import sys
import time
import urllib.error
import urllib.request

web_url = sys.argv[1].rstrip("/")
duration_s = float(sys.argv[2])
state_path = sys.argv[3]
tasks_path = sys.argv[4]
summary_path = sys.argv[5]

deadline = time.time() + max(1.0, duration_s)
last_line = None

def fetch_json(path):
    with urllib.request.urlopen(f"{web_url}{path}", timeout=5) as response:
        return json.load(response)

def append_jsonl(path, payload):
    with open(path, "a", encoding="utf-8") as file:
        file.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")

def text(value):
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)

def number(mapping, key):
    if not isinstance(mapping, dict):
        return None
    value = mapping.get(key)
    try:
        return float(value)
    except (TypeError, ValueError):
        return None

def planar_error(a, b):
    ax = number(a, "x")
    ay = number(a, "y")
    bx = number(b, "x")
    by = number(b, "y")
    if ax is None or ay is None or bx is None or by is None:
        return None
    return math.hypot(ax - bx, ay - by)

def feedback_pose(feedback):
    if not isinstance(feedback, dict):
        return {}
    return {
        "x": feedback.get("pose_x"),
        "y": feedback.get("pose_y"),
        "yaw": feedback.get("pose_yaw"),
    }

def first_value(*values):
    for value in values:
        if value is not None and value != "":
            return value
    return None

while time.time() < deadline:
    now = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
    try:
        state = fetch_json("/api/state")
        tasks = fetch_json("/api/tasks")
        append_jsonl(state_path, {"time": now, "state": state})
        append_jsonl(tasks_path, {"time": now, "tasks": tasks})

        readiness = state.get("task_readiness") or {}
        nav_ready = readiness.get("navigation_readiness") or {}
        nav_checks = nav_ready.get("checks") or {}
        local_costmap = nav_checks.get("local_costmap") or {}
        global_costmap = nav_checks.get("global_costmap") or {}
        active = state.get("active_task") or {}
        waypoint = ((state.get("active_waypoint") or {}).get("parsed") or {})
        wp = waypoint.get("waypoint") or {}
        battery = state.get("battery") or {}
        primary_battery = battery.get("primary") or {}
        scan = state.get("scan") or {}
        lidar = state.get("lidar_points") or {}
        nav_feedback = waypoint.get("nav_feedback") or active.get("last_nav_feedback") or {}
        nav_goal_match = waypoint.get("nav_goal_match") or active.get("last_nav_goal_match") or {}
        ignored_match = active.get("last_ignored_nav_goal_match") or {}
        runtime_guard = waypoint.get("runtime_guard") or active.get("runtime_guard") or {}
        robot_pose = waypoint.get("robot_pose") or active.get("last_robot_pose") or state.get("pose") or {}
        goal_pose = waypoint.get("goal_pose") or wp.get("pose") or {}
        nav_pose = feedback_pose(nav_feedback)
        timeline = active.get("timeline") if isinstance(active.get("timeline"), list) else []
        active_last_event = timeline[-1] if timeline else {}
        last_result = state.get("last_task_result") or {}
        last_result_event = last_result.get("last_event") or {}
        last_event = active_last_event or last_result_event
        line = {
            "time": now,
            "localization": state.get("localization_ok"),
            "pose": bool(state.get("pose")),
            "readiness": readiness.get("code"),
            "nav_ready": nav_ready.get("ready"),
            "local_costmap": local_costmap.get("ok"),
            "global_costmap": global_costmap.get("ok"),
            "battery_level": primary_battery.get("level"),
            "scan_finite": scan.get("finite_ranges"),
            "lidar_points": (int(lidar.get("width", 0) or 0) * max(1, int(lidar.get("height", 1) or 1))) if isinstance(lidar, dict) else None,
            "lidar_source": lidar.get("source") if isinstance(lidar, dict) else None,
            "runtime_guard": runtime_guard.get("code"),
            "runtime_guard_age": waypoint.get("runtime_guard_lost_age_s") if waypoint.get("runtime_guard_lost_age_s") is not None else active.get("runtime_guard_lost_age_s"),
            "active_task": active.get("task_id"),
            "task_name": active.get("task_name"),
            "task_map": active.get("map_id"),
            "waypoint_id": wp.get("id") or active.get("last_goal_annotation_id"),
            "waypoint": wp.get("label") or active.get("last_goal_label"),
            "phase": waypoint.get("phase") or active.get("phase"),
            "nav": waypoint.get("nav_goal_status") or active.get("last_nav_goal_status"),
            "distance": waypoint.get("distance_m") if waypoint.get("distance_m") is not None else active.get("last_distance_m"),
            "first_waypoint_distance": readiness.get("first_waypoint_distance_m"),
            "nav_remaining": nav_feedback.get("distance_remaining"),
            "recoveries": nav_feedback.get("recoveries"),
            "nav_goal_seq": first_value(
                nav_feedback.get("goal_seq"),
                nav_goal_match.get("nav_goal_seq"),
                ignored_match.get("nav_goal_seq"),
            ),
            "goal_attempt": active.get("last_goal_attempt_id"),
            "floor_goal_published": first_value(
                waypoint.get("last_floor_goal_published_at"),
                active.get("last_floor_goal_published_at"),
            ),
            "floor_goal_publishes": first_value(
                waypoint.get("floor_goal_publish_count"),
                active.get("floor_goal_publish_count"),
            ),
            "nav_match": nav_goal_match.get("matches"),
            "nav_match_reason": first_value(
                nav_goal_match.get("reason"),
                ignored_match.get("reason"),
            ),
            "goal_sends": waypoint.get("goal_send_count") if waypoint.get("goal_send_count") is not None else active.get("waypoint_goal_send_count"),
            "resends": waypoint.get("resend_goal_count") if waypoint.get("resend_goal_count") is not None else active.get("resend_goal_count"),
            "robot_x": number(robot_pose, "x"),
            "robot_y": number(robot_pose, "y"),
            "robot_yaw": number(robot_pose, "yaw"),
            "goal_x": number(goal_pose, "x"),
            "goal_y": number(goal_pose, "y"),
            "goal_yaw": number(goal_pose, "yaw"),
            "nav_pose_x": number(nav_feedback, "pose_x"),
            "nav_pose_y": number(nav_feedback, "pose_y"),
            "nav_pose_yaw": number(nav_feedback, "pose_yaw"),
            "path_goal_error": waypoint.get("path_goal_error_m"),
            "path_points": waypoint.get("path_point_count"),
            "path_raw_points": waypoint.get("path_raw_point_count"),
            "goal_sent_path_version": first_value(waypoint.get("goal_sent_path_version"), active.get("goal_sent_path_version")),
            "plan_path_version": first_value(waypoint.get("plan_path_version"), active.get("plan_path_version")),
            "plan_verified": first_value(waypoint.get("plan_goal_verified"), active.get("plan_goal_verified")),
            "robot_goal_error": planar_error(robot_pose, goal_pose),
            "nav_goal_error": planar_error(nav_pose, goal_pose),
            "robot_nav_error": planar_error(robot_pose, nav_pose),
            "stall_age": waypoint.get("stall_age_s") if waypoint.get("stall_age_s") is not None else active.get("stall_age_s"),
            "last_event": last_event.get("event") or last_event.get("message"),
            "last_result": last_result.get("last_error") or last_result_event.get("message"),
            "message": waypoint.get("status_message") or active.get("status_message") or readiness.get("message"),
        }
        rendered = "\t".join(text(line[key]) for key in (
            "time", "localization", "pose", "readiness", "nav_ready", "local_costmap",
            "global_costmap", "battery_level", "scan_finite", "lidar_points", "lidar_source",
            "runtime_guard", "runtime_guard_age", "active_task", "task_name", "task_map", "waypoint_id", "waypoint", "phase", "nav", "distance",
            "first_waypoint_distance", "nav_remaining", "recoveries", "nav_goal_seq", "goal_attempt", "floor_goal_published",
            "floor_goal_publishes", "nav_match", "nav_match_reason", "goal_sends", "resends", "robot_x", "robot_y",
            "robot_yaw", "goal_x", "goal_y", "goal_yaw", "nav_pose_x", "nav_pose_y",
            "nav_pose_yaw", "path_goal_error", "path_points", "path_raw_points", "goal_sent_path_version",
            "plan_path_version", "plan_verified", "robot_goal_error", "nav_goal_error", "robot_nav_error", "stall_age",
            "last_event", "last_result", "message"
        ))
        with open(summary_path, "a", encoding="utf-8") as file:
            file.write(rendered + "\n")
        if rendered != last_line:
            print(rendered, flush=True)
            last_line = rendered
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
        rendered = f"{now}\tERROR\t{exc}"
        with open(summary_path, "a", encoding="utf-8") as file:
            file.write(rendered + "\n")
        print(rendered, flush=True)
    time.sleep(1.0)
PY

if [ -x "${ANALYZER}" ]; then
  "${ANALYZER}" "${RUN_DIR}" >"${ANALYSIS_TXT}" 2>&1 || {
    {
      echo "[104_watch_frontend_task] analyzer failed"
      echo "analyzer=${ANALYZER}"
    } >>"${ANALYSIS_TXT}"
  }
  echo "[104_watch_frontend_task] analysis=${ANALYSIS_TXT}"
elif command -v python3 >/dev/null 2>&1 && [ -f "${ANALYZER}" ]; then
  python3 "${ANALYZER}" "${RUN_DIR}" >"${ANALYSIS_TXT}" 2>&1 || {
    {
      echo "[104_watch_frontend_task] analyzer failed"
      echo "analyzer=${ANALYZER}"
    } >>"${ANALYSIS_TXT}"
  }
  echo "[104_watch_frontend_task] analysis=${ANALYSIS_TXT}"
else
  {
    echo "[104_watch_frontend_task] analyzer not found"
    echo "analyzer=${ANALYZER}"
  } >"${ANALYSIS_TXT}"
  echo "[104_watch_frontend_task] analysis=${ANALYSIS_TXT}"
fi
