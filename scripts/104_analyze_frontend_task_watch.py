#!/usr/bin/env python3
"""Summarize a read-only frontend task watcher run.

Input is a directory produced by scripts/104_watch_frontend_task.sh.  The
analysis is intentionally offline: it reads summary.tsv/state.jsonl/journal logs
and never talks to ROS or the robot.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


def parse_float(value: Any) -> Optional[float]:
    if value in (None, "", "-"):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def parse_int(value: Any) -> Optional[int]:
    number = parse_float(value)
    if number is None:
        return None
    return int(number)


def read_tsv(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open("r", encoding="utf-8", newline="") as file:
        return list(csv.DictReader(file, delimiter="\t"))


def read_meta(path: Path) -> Dict[str, str]:
    result: Dict[str, str] = {}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        key, sep, value = line.partition("=")
        if sep:
            result[key.strip()] = value.strip()
    return result


def iter_state_jsonl(path: Path) -> Iterable[Dict[str, Any]]:
    if not path.exists():
        return
    with path.open("r", encoding="utf-8") as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(payload, dict):
                yield payload


def iter_tasks_jsonl(path: Path) -> Iterable[Dict[str, Any]]:
    if not path.exists():
        return
    with path.open("r", encoding="utf-8") as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(payload, dict):
                yield payload


def unique_in_order(values: Iterable[Optional[str]]) -> List[str]:
    seen = set()
    result = []
    for value in values:
        value = str(value or "").strip()
        if not value or value == "-":
            continue
        if value in seen:
            continue
        seen.add(value)
        result.append(value)
    return result


def min_float(rows: List[Dict[str, str]], key: str) -> Optional[float]:
    values = [parse_float(row.get(key)) for row in rows]
    values = [value for value in values if value is not None]
    return min(values) if values else None


def max_float(rows: List[Dict[str, str]], key: str) -> Optional[float]:
    values = [parse_float(row.get(key)) for row in rows]
    values = [value for value in values if value is not None]
    return max(values) if values else None


def final_value(rows: List[Dict[str, str]], key: str) -> str:
    for row in reversed(rows):
        value = str(row.get(key) or "").strip()
        if value and value != "-":
            return value
    return "-"


def unique_count(rows: List[Dict[str, str]], key: str) -> int:
    return len(unique_in_order(row.get(key) for row in rows))


def waypoint_text(point: Dict[str, Any], index: int) -> str:
    pose = point.get("pose") if isinstance(point.get("pose"), dict) else {}
    label = str(point.get("label") or point.get("id") or f"point{index + 1}")
    floor = str(point.get("floor") or "-")
    parts = [f"{index + 1}.{floor} {label}"]
    try:
        parts.append("x=%.2f y=%.2f" % (float(pose.get("x")), float(pose.get("y"))))
    except (TypeError, ValueError):
        pass
    try:
        parts.append("yaw=%.2f" % float(pose.get("yaw")))
    except (TypeError, ValueError):
        pass
    try:
        parts.append("dwell=%.1fs" % float(point.get("dwell_s")))
    except (TypeError, ValueError):
        pass
    return " ".join(parts)


def fmt_optional_float(value: Any, suffix: str = "", digits: int = 2) -> str:
    number = parse_float(value)
    if number is None:
        return "-"
    return f"{number:.{digits}f}{suffix}"


def compact_runtime_snapshot(task: Dict[str, Any], result: Dict[str, Any]) -> Optional[str]:
    snapshot = result.get("runtime_snapshot") if isinstance(result.get("runtime_snapshot"), dict) else {}
    if not snapshot:
        return None
    path = snapshot.get("path") if isinstance(snapshot.get("path"), dict) else {}
    scan = snapshot.get("scan") if isinstance(snapshot.get("scan"), dict) else {}
    lidar = snapshot.get("lidar_points") if isinstance(snapshot.get("lidar_points"), dict) else {}
    lidar_relay = snapshot.get("lidar_relay_status") if isinstance(snapshot.get("lidar_relay_status"), dict) else {}
    camera_proxy = snapshot.get("camera_proxy") if isinstance(snapshot.get("camera_proxy"), dict) else {}
    cameras = camera_proxy.get("cameras") if isinstance(camera_proxy.get("cameras"), dict) else {}
    nav_feedback = snapshot.get("last_nav_feedback") if isinstance(snapshot.get("last_nav_feedback"), dict) else {}
    nav_match = snapshot.get("last_nav_goal_match") if isinstance(snapshot.get("last_nav_goal_match"), dict) else {}
    runtime_guard = snapshot.get("runtime_guard") if isinstance(snapshot.get("runtime_guard"), dict) else {}
    pose = snapshot.get("pose") if isinstance(snapshot.get("pose"), dict) else {}
    camera_parts = []
    for name in ("front", "rear"):
        camera = cameras.get(name) if isinstance(cameras.get(name), dict) else {}
        if not camera:
            continue
        label = "front" if name == "front" else "rear"
        if camera.get("has_frame"):
            state = "frame_age=%s" % fmt_optional_float(camera.get("last_frame_age_s"), "s", 1)
        elif camera.get("running"):
            state = "waiting"
        else:
            state = "not_pulled"
        if camera.get("last_error"):
            state += f":{camera.get('last_error')}"
        camera_parts.append(f"{label}:{state}")
    parts = [
        "  - {name} id={task_id} result={status}".format(
            name=str(task.get("name") or "-"),
            task_id=str(task.get("id") or "-"),
            status=str(result.get("status") or task.get("status") or "-"),
        ),
        "floor={floor}".format(floor=str(snapshot.get("floor") or "-")),
        "loc={loc}".format(loc=str(snapshot.get("localization_ok"))),
        "pose_age={age}".format(age=fmt_optional_float(snapshot.get("pose_age_sec"), "s", 1)),
        "pose=({x},{y})".format(
            x=fmt_optional_float(pose.get("x")),
            y=fmt_optional_float(pose.get("y")),
        ) if pose else "pose=-",
        "nav_status={status}".format(status=str(snapshot.get("navigation_status") or "-")),
        "path=v{version}/raw={raw}/shown={shown}/verified={verified}/err={err}".format(
            version=str(path.get("version") or "-"),
            raw=str(path.get("raw_point_count") or "-"),
            shown=str(path.get("point_count") or "-"),
            verified=str(snapshot.get("plan_goal_verified")),
            err=fmt_optional_float(snapshot.get("plan_goal_error_m"), "m"),
        ),
        "scan={finite}/age={age}".format(
            finite=str(scan.get("finite_ranges") or "-"),
            age=fmt_optional_float(scan.get("age_sec"), "s", 1),
        ),
        "lidar={points}/source={source}/age={age}".format(
            points=str(lidar.get("width") or "-"),
            source=str(lidar.get("source") or "-"),
            age=fmt_optional_float(lidar.get("age_sec"), "s", 1),
        ),
        "relay=out{points}/stride={stride}/method={method}/in={in_hz}Hz/out={out_hz}Hz/skip={skip}".format(
            points=str(lidar_relay.get("output_width") or "-"),
            stride=str(lidar_relay.get("output_stride") or "-"),
            method=str(lidar_relay.get("downsample_method") or "-"),
            in_hz=fmt_optional_float(lidar_relay.get("input_rate_hz"), "", 1),
            out_hz=fmt_optional_float(lidar_relay.get("publish_rate_hz"), "", 1),
            skip=fmt_optional_float(lidar_relay.get("skip_ratio"), "", 2),
        ),
        "goal={label}/{attempt}".format(
            label=str(snapshot.get("last_goal_label") or snapshot.get("last_goal_annotation_id") or "-"),
            attempt=str(snapshot.get("last_goal_attempt_id") or "-"),
        ),
        "floor_goal=published_at={published}/count={count}".format(
            published=str(snapshot.get("last_floor_goal_published_at") or "-"),
            count=str(snapshot.get("floor_goal_publish_count") or "-"),
        ),
        "nav_goal={status}/seq={seq}/match={match}".format(
            status=str(snapshot.get("last_nav_goal_status") or "-"),
            seq=str(nav_match.get("nav_goal_seq") or snapshot.get("last_nav_goal_seq") or "-"),
            match=str(nav_match.get("matches") if nav_match else "-"),
        ),
        "nav_remaining={remaining}".format(
            remaining=fmt_optional_float(nav_feedback.get("distance_remaining"), "m")
        ),
        "runtime_guard={guard}".format(guard=str(runtime_guard.get("code") or snapshot.get("runtime_guard") or "-")),
        "camera=" + (",".join(camera_parts) if camera_parts else "-"),
    ]
    return " ".join(parts)


def tasks_overview(path: Path) -> Dict[str, Any]:
    latest_payload: Dict[str, Any] = {}
    for payload in iter_tasks_jsonl(path):
        tasks_payload = payload.get("tasks")
        if isinstance(tasks_payload, dict):
            latest_payload = tasks_payload
        elif isinstance(tasks_payload, list):
            latest_payload = payload
    raw_tasks = latest_payload.get("tasks") if isinstance(latest_payload.get("tasks"), list) else []
    result: Dict[str, Any] = {
        "tasks": raw_tasks,
        "by_id": {},
        "lines": [],
        "result_lines": [],
        "snapshot_lines": [],
        "result_messages": [],
    }
    for task in raw_tasks:
        if not isinstance(task, dict):
            continue
        task_id = str(task.get("id") or "").strip()
        if task_id:
            result["by_id"][task_id] = task
        waypoints = task.get("waypoints") if isinstance(task.get("waypoints"), list) else []
        first = waypoints[0] if waypoints and isinstance(waypoints[0], dict) else {}
        readiness = task.get("readiness") if isinstance(task.get("readiness"), dict) else {}
        sequence = " -> ".join(
            waypoint_text(point, index)
            for index, point in enumerate(waypoints)
            if isinstance(point, dict)
        ) or "-"
        result["lines"].append(
            "  - {name} id={task_id} status={status} map={map_id} points={count} first={first} readiness={ready} first_distance={first_distance}: {message} order={order}".format(
                name=str(task.get("name") or "-"),
                task_id=task_id or "-",
                status=str(task.get("status") or "-"),
                map_id=str(task.get("map_id") or "-"),
                count=len(waypoints),
                first=waypoint_text(first, 0) if first else "-",
                ready=str(readiness.get("code") or "-"),
                first_distance=(
                    "%.2fm" % float(readiness.get("first_waypoint_distance_m"))
                    if parse_float(readiness.get("first_waypoint_distance_m")) is not None
                    else "-"
                ),
                message=str(readiness.get("message") or "-"),
                order=sequence,
            )
        )
        timeline = task.get("last_timeline") if isinstance(task.get("last_timeline"), list) else []
        last_event = timeline[-1] if timeline and isinstance(timeline[-1], dict) else {}
        last_result = task.get("last_result") if isinstance(task.get("last_result"), dict) else {}
        last_error = str(task.get("last_error") or "").strip()
        if last_error or last_event or last_result:
            message = str(last_result.get("message") or last_event.get("message") or "-")
            event = str(last_event.get("event") or last_result.get("status") or "-")
            extra_parts = []
            for key in (
                "reason",
                "annotation_id",
                "label",
                "distance_m",
                "last_distance_m",
                "path_goal_error_m",
                "path_version",
                "goal_sent_path_version",
                "last_nav_goal_status",
                "runtime_guard",
                "runtime_guard_lost_age_s",
            ):
                value = last_result.get(key, last_event.get(key))
                if value not in (None, "", "-"):
                    extra_parts.append(f"{key}={value}")
            line = (
                "  - {name} id={task_id} status={status} last_error={last_error} "
                "last_event={event}: {message}{extra}"
            ).format(
                name=str(task.get("name") or "-"),
                task_id=task_id or "-",
                status=str(task.get("status") or "-"),
                last_error=last_error or str(last_result.get("last_error") or "-"),
                event=event,
                message=message,
                extra=(" " + " ".join(extra_parts)) if extra_parts else "",
            )
            result["result_lines"].append(line)
            snapshot_line = compact_runtime_snapshot(task, last_result)
            if snapshot_line:
                result["snapshot_lines"].append(snapshot_line)
            result["result_messages"].extend(
                item
                for item in (last_error, str(last_result.get("last_error") or ""), event, message, " ".join(extra_parts))
                if item
            )
    return result


def summarize_waypoints(rows: List[Dict[str, str]]) -> List[str]:
    grouped: Dict[str, List[Dict[str, str]]] = defaultdict(list)
    order = []
    for row in rows:
        waypoint = str(row.get("waypoint") or "").strip()
        if not waypoint or waypoint == "-":
            continue
        if waypoint not in grouped:
            order.append(waypoint)
        grouped[waypoint].append(row)

    lines = []
    for waypoint in order:
        items = grouped[waypoint]
        phases = "/".join(unique_in_order(row.get("phase") for row in items)) or "-"
        navs = "/".join(unique_in_order(row.get("nav") for row in items)) or "-"
        min_robot_goal = min_float(items, "robot_goal_error")
        min_nav_goal = min_float(items, "nav_goal_error")
        min_robot_nav = min_float(items, "robot_nav_error")
        min_path_goal = min_float(items, "path_goal_error")
        min_first_distance = min_float(items, "first_waypoint_distance")
        final_robot_goal = final_value(items, "robot_goal_error")
        final_path_goal = final_value(items, "path_goal_error")
        max_path_points = max_float(items, "path_points")
        max_path_raw_points = max_float(items, "path_raw_points")
        plan_versions = unique_in_order(row.get("plan_path_version") for row in items)
        sent_path_versions = unique_in_order(row.get("goal_sent_path_version") for row in items)
        plan_verified = unique_in_order(row.get("plan_verified") for row in items)
        max_goal_sends = max_float(items, "goal_sends")
        max_resends = max_float(items, "resends")
        max_recoveries = max_float(items, "recoveries")
        max_stall = max_float(items, "stall_age")
        nav_goal_seqs = unique_in_order(row.get("nav_goal_seq") for row in items)
        goal_attempts = unique_in_order(row.get("goal_attempt") for row in items)
        nav_match_reasons = unique_in_order(row.get("nav_match_reason") for row in items)
        waypoint_ids = unique_in_order(row.get("waypoint_id") for row in items)
        parts = [
            f"{waypoint}: samples={len(items)}",
            f"phase={phases}",
            f"nav={navs}",
        ]
        if waypoint_ids:
            parts.append("ids=" + "/".join(waypoint_ids[:3]))
        if nav_goal_seqs:
            parts.append("nav_goal_seq=" + "/".join(nav_goal_seqs[:5]))
        if goal_attempts:
            parts.append(f"goal_attempts={len(goal_attempts)}")
        if nav_match_reasons:
            parts.append("nav_match_reason=" + "/".join(nav_match_reasons[:3]))
        if min_robot_goal is not None:
            parts.append(f"min_robot_goal={min_robot_goal:.2f}m")
        if min_nav_goal is not None:
            parts.append(f"min_nav_goal={min_nav_goal:.2f}m")
        if min_robot_nav is not None:
            parts.append(f"min_robot_nav={min_robot_nav:.2f}m")
        if min_path_goal is not None:
            parts.append(f"min_path_goal={min_path_goal:.2f}m")
        if min_first_distance is not None:
            parts.append(f"first_distance={min_first_distance:.2f}m")
        if final_robot_goal != "-":
            parts.append(f"final_robot_goal={final_robot_goal}m")
        if final_path_goal != "-":
            parts.append(f"final_path_goal={final_path_goal}m")
        if max_path_points is not None:
            parts.append(f"path_points={int(max_path_points)}")
        if max_path_raw_points is not None:
            parts.append(f"path_raw_points={int(max_path_raw_points)}")
        if sent_path_versions:
            parts.append("sent_path_version=" + "/".join(sent_path_versions[:5]))
        if plan_versions:
            parts.append("plan_path_version=" + "/".join(plan_versions[:5]))
        if plan_verified:
            parts.append("plan_verified=" + "/".join(plan_verified[:3]))
        if max_goal_sends is not None:
            parts.append(f"max_goal_sends={int(max_goal_sends)}")
        if max_resends is not None and max_resends > 0:
            parts.append(f"resends={int(max_resends)}")
        if max_recoveries is not None and max_recoveries > 0:
            parts.append(f"recoveries={int(max_recoveries)}")
        if max_stall is not None and max_stall > 0:
            parts.append(f"max_stall={max_stall:.0f}s")
        lines.append("  - " + ", ".join(parts))
    return lines


def journal_counts(path: Path) -> Dict[str, int]:
    if not path.exists():
        return {}
    text = path.read_text(encoding="utf-8", errors="replace")
    patterns = {
        "floor_goals": r"web task published floor goal",
        "nav_accepted": r"nav_goal_accepted",
        "nav_feedback": r"nav_goal_feedback",
        "nav_succeeded": r"nav_goal_succeeded",
        "nav_errors": r"error reason=nav_|nav_goal_failed",
        "duplicate_ignored": r"duplicate_floor_goal",
        "stale_success_ignored": r"忽略非当前任务点 Nav2 成功事件",
        "nav_status_ignored": r"nav_status_ignored|忽略与当前任务点不匹配的 Nav2",
        "goal_seq_status": r"goal_seq=",
    }
    return {key: len(re.findall(pattern, text)) for key, pattern in patterns.items()}


def state_overview(path: Path) -> Dict[str, Any]:
    latest: Dict[str, Any] = {}
    active_seen = False
    ready_seen = False
    pose_seen = False
    for payload in iter_state_jsonl(path):
        state = payload.get("state") if isinstance(payload.get("state"), dict) else payload
        if not isinstance(state, dict):
            continue
        latest = state
        if state.get("active_task"):
            active_seen = True
        readiness = state.get("task_readiness") or {}
        if isinstance(readiness, dict) and readiness.get("ready") is True:
            ready_seen = True
        if state.get("pose"):
            pose_seen = True
    readiness = latest.get("task_readiness") if isinstance(latest.get("task_readiness"), dict) else {}
    return {
        "active_seen": active_seen,
        "ready_seen": ready_seen,
        "pose_seen": pose_seen,
        "latest_floor": latest.get("floor"),
        "latest_localization_ok": latest.get("localization_ok"),
        "latest_readiness_code": readiness.get("code"),
        "latest_active_task": latest.get("active_task"),
    }


def build_findings(
    rows: List[Dict[str, str]],
    state_info: Dict[str, Any],
    counts: Dict[str, int],
    task_info: Optional[Dict[str, Any]] = None,
) -> List[str]:
    findings = []
    readiness_codes = Counter(row.get("readiness") or "-" for row in rows)
    active_rows = [row for row in rows if (row.get("active_task") or "-") != "-"]
    error_messages = unique_in_order(row.get("last_result") for row in rows)
    messages = unique_in_order(row.get("message") for row in rows)
    task_result_messages = unique_in_order((task_info or {}).get("result_messages") or [])
    all_messages = error_messages + messages + task_result_messages

    if not rows:
        findings.append("no summary rows found")
    else:
        if not active_rows:
            code = readiness_codes.most_common(1)[0][0] if readiness_codes else "-"
            findings.append(f"no active task observed; dominant readiness={code}")
        if state_info.get("pose_seen") is False:
            findings.append("no map pose observed in state.jsonl")
        if state_info.get("ready_seen") is False and not active_rows:
            findings.append("task readiness never became ready during watcher window")
        max_resends = max_float(rows, "resends")
        if max_resends is not None and max_resends > 0:
            findings.append(f"goal resend observed: max_resends={int(max_resends)}")
        max_stall = max_float(rows, "stall_age")
        if max_stall is not None and max_stall > 0:
            findings.append(f"stall/low-progress observed: max_stall={max_stall:.0f}s")
        max_recoveries = max_float(rows, "recoveries")
        if max_recoveries is not None and max_recoveries > 0:
            findings.append(f"Nav2 recoveries observed: max_recoveries={int(max_recoveries)}")
    if counts.get("nav_errors", 0) > 0:
        findings.append(f"journal contains Nav2 errors: {counts['nav_errors']}")
    if counts.get("duplicate_ignored", 0) > 0:
        findings.append(f"duplicate floor goals ignored: {counts['duplicate_ignored']}")
    if counts.get("nav_status_ignored", 0) > 0:
        findings.append(f"mismatched Nav2 status ignored: {counts['nav_status_ignored']}")
    if any("path_goal_mismatch" in value for value in all_messages):
        findings.append("task stopped because Nav2 planned path endpoint did not match the active waypoint")
    if any("plan_update_timeout" in value for value in all_messages):
        findings.append("task stopped because no fresh Nav2 plan was observed after goal acceptance")
        if any("waypoint_stalled" in value for value in all_messages):
            findings.append("task stopped because waypoint progress stalled")
    if any("runtime_guard_lost" in value for value in all_messages):
        findings.append("task stopped because battery/perception runtime guard stayed unhealthy")
    if any("perception_scan_unavailable" in value for value in all_messages):
        findings.append("scan was unavailable or too sparse for task execution")
    if any("perception_lidar_unavailable" in value for value in all_messages):
        findings.append("lidar pointcloud relay was unavailable for task execution")
    if any("battery_low" in value for value in all_messages):
        findings.append("battery level was below task execution threshold")
    if task_result_messages:
        findings.append("task result records: " + " | ".join(task_result_messages[:4]))
    if active_rows:
        max_first_distance = max_float(active_rows, "first_waypoint_distance")
        if max_first_distance is not None and max_first_distance > 8.0:
            findings.append(
                "task started with a far first waypoint: max_first_waypoint_distance=%.2fm; "
                "confirm relocalization, selected map and waypoint coordinates before allowing motion"
                % max_first_distance
            )
        max_path_goal = max_float(active_rows, "path_goal_error")
        min_path_goal = min_float(active_rows, "path_goal_error")
        if max_path_goal is not None and max_path_goal > 0.8:
            detail = f"max_path_goal_error={max_path_goal:.2f}m"
            if min_path_goal is not None:
                detail += f" min_path_goal_error={min_path_goal:.2f}m"
            findings.append("planned path endpoint is far from active waypoint: " + detail)
        attempt_count = unique_count(active_rows, "goal_attempt")
        nav_seq_count = unique_count(active_rows, "nav_goal_seq")
        if attempt_count > 1 and nav_seq_count == 0:
            findings.append(f"frontend resent goals but no Nav2 goal sequence was observed: attempts={attempt_count}")
        elif attempt_count > max(1, nav_seq_count + 1):
            findings.append(f"goal attempts exceed Nav2 goal sequences: attempts={attempt_count} nav_goal_seq={nav_seq_count}")
        mismatch_reasons = unique_in_order(
            row.get("nav_match_reason")
            for row in active_rows
            if str(row.get("nav_match") or "").lower() == "false" or (row.get("nav_match_reason") or "-") != "-"
        )
        if mismatch_reasons:
            findings.append("Nav2 goal mismatch reasons: " + " | ".join(mismatch_reasons[:3]))
        min_battery = min_float(active_rows, "battery_level")
        max_scan = max_float(active_rows, "scan_finite")
        max_lidar = max_float(active_rows, "lidar_points")
        runtime_guards = unique_in_order(row.get("runtime_guard") for row in active_rows)
        if min_battery is not None and min_battery < 20:
            findings.append(f"battery fell low during task: min_battery={min_battery:.0f}%")
        if max_scan is not None and max_scan < 20:
            findings.append(f"scan remained sparse during active task: max_scan_finite={max_scan:.0f}")
        if max_lidar is not None and max_lidar <= 0:
            findings.append("no lidar relay points were visible during active task")
        if runtime_guards:
            findings.append("runtime guard states observed: " + " | ".join(runtime_guards[:4]))
    if error_messages:
        findings.append("last_result: " + " | ".join(error_messages[:3]))
    if not findings and messages:
        findings.append("latest message: " + messages[-1])
    return findings or ["no obvious issue detected in summary"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", help="Directory created by 104_watch_frontend_task.sh")
    args = parser.parse_args()

    run_dir = Path(args.run_dir).expanduser()
    if not run_dir.exists():
        print(f"[FAIL] run directory does not exist: {run_dir}", file=sys.stderr)
        return 2
    summary_path = run_dir / "summary.tsv"
    rows = read_tsv(summary_path)
    meta = read_meta(run_dir / "meta.txt")
    state_info = state_overview(run_dir / "state.jsonl")
    task_info = tasks_overview(run_dir / "tasks.jsonl")
    counts = journal_counts(run_dir / "m20pro-real.journal.log")

    print(f"run_dir: {run_dir}")
    if meta:
        print(
            "meta: started={started} duration={duration}s label={label} web={web}".format(
                started=meta.get("started_at", "-"),
                duration=meta.get("duration_s", "-"),
                label=meta.get("label", "-"),
                web=meta.get("web_url", "-"),
            )
        )
    print(f"samples: {len(rows)}")
    print(
        "latest: floor={floor} localization={loc} readiness={ready} active_task={task}".format(
            floor=state_info.get("latest_floor"),
            loc=state_info.get("latest_localization_ok"),
            ready=state_info.get("latest_readiness_code"),
            task="yes" if state_info.get("latest_active_task") else "no",
        )
    )
    if counts:
        print(
            "journal: floor_goals={floor_goals} accepted={nav_accepted} feedback={nav_feedback} "
            "succeeded={nav_succeeded} nav_errors={nav_errors} ignored={nav_status_ignored} "
            "goal_seq_status={goal_seq_status}".format(**counts)
        )
    print("task definitions:")
    task_lines = task_info.get("lines") or []
    if task_lines:
        print("\n".join(task_lines))
    else:
        print("  - none recorded")
    print("task results:")
    result_lines = task_info.get("result_lines") or []
    if result_lines:
        print("\n".join(result_lines))
    else:
        print("  - none recorded")
    print("runtime snapshots:")
    snapshot_lines = task_info.get("snapshot_lines") or []
    if snapshot_lines:
        print("\n".join(snapshot_lines))
    else:
        print("  - none recorded")
    print("waypoints:")
    waypoint_lines = summarize_waypoints(rows)
    if waypoint_lines:
        print("\n".join(waypoint_lines))
    else:
        print("  - none observed")
    print("findings:")
    for finding in build_findings(rows, state_info, counts, task_info):
        print(f"  - {finding}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
