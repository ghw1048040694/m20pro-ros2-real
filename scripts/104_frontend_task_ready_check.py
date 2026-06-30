#!/usr/bin/env python3
"""Read-only readiness check before starting a real frontend task.

The script queries the web dashboard only.  It does not call /api/tasks/start,
publish ROS goals, or send motion commands.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import urllib.error
import urllib.request
from typing import Any, Dict, Iterable, List, Optional, Tuple


def fetch_json(base_url: str, path: str, timeout_s: float = 5.0) -> Dict[str, Any]:
    with urllib.request.urlopen(f"{base_url.rstrip('/')}{path}", timeout=timeout_s) as response:
        payload = json.load(response)
    if not isinstance(payload, dict):
        raise RuntimeError(f"unexpected JSON from {path}: {type(payload).__name__}")
    return payload


def number(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result


def text(value: Any) -> str:
    if value is None or value == "":
        return "-"
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def waypoint_line(point: Dict[str, Any], index: int) -> str:
    pose = point.get("pose") if isinstance(point.get("pose"), dict) else {}
    x = number(pose.get("x"))
    y = number(pose.get("y"))
    yaw = number(pose.get("yaw"))
    parts = [
        f"{index + 1}.",
        str(point.get("floor") or "-"),
        str(point.get("label") or point.get("id") or "-"),
    ]
    if x is not None and y is not None:
        parts.append(f"x={x:.2f} y={y:.2f}")
    if yaw is not None:
        parts.append(f"yaw={yaw:.2f}")
    dwell = number(point.get("dwell_s"))
    if dwell is not None:
        parts.append(f"dwell={dwell:.1f}s")
    return " ".join(parts)


def compact_order(points: Iterable[Dict[str, Any]]) -> str:
    rendered = [waypoint_line(point, index) for index, point in enumerate(points)]
    return " -> ".join(rendered) if rendered else "-"


def summarize_state(state: Dict[str, Any]) -> List[str]:
    battery = state.get("battery") if isinstance(state.get("battery"), dict) else {}
    primary = battery.get("primary") if isinstance(battery.get("primary"), dict) else {}
    scan = state.get("scan") if isinstance(state.get("scan"), dict) else {}
    lidar = state.get("lidar_points") if isinstance(state.get("lidar_points"), dict) else {}
    relay = state.get("lidar_relay_status") if isinstance(state.get("lidar_relay_status"), dict) else {}
    perception_status = state.get("perception_status") if isinstance(state.get("perception_status"), dict) else {}
    readiness = state.get("task_readiness") if isinstance(state.get("task_readiness"), dict) else {}
    localization_status = state.get("localization_status") if isinstance(state.get("localization_status"), dict) else {}
    selected_map_status = state.get("selected_map_status") if isinstance(state.get("selected_map_status"), dict) else {}
    lidar_points = int(lidar.get("width", 0) or 0) * max(1, int(lidar.get("height", 1) or 1))
    relay_output_points = int(relay.get("output_width", 0) or 0) * max(1, int(relay.get("output_height", 1) or 1))
    relay_input_rate = number(relay.get("input_rate_hz"))
    relay_publish_rate = number(relay.get("publish_rate_hz"))
    relay_skip_ratio = number(relay.get("skip_ratio"))
    lines = [
        f"web_ok={text(state.get('ok'))}",
        f"floor={text(state.get('floor'))}",
        f"selected_map={text(state.get('selected_map_id'))}",
        f"localization_ok={text(state.get('localization_ok'))}",
        f"pose_fresh={text(state.get('pose_fresh'))}",
        f"active_task={text(bool(state.get('active_task')))}",
        f"active_waypoint={text(bool(state.get('active_waypoint')))}",
        "localization_status={code}: confirmed={confirmed} task_ready={task_ready} tcp_2101={tcp_2101} message={message}".format(
            code=text(localization_status.get("code")),
            confirmed=text(localization_status.get("confirmed")),
            task_ready=text(localization_status.get("task_ready")),
            tcp_2101=text(localization_status.get("tcp_2101_accepted")),
            message=text(localization_status.get("message")),
        ),
        "selected_map_status={code}: ready={ready} message={message}".format(
            code=text(selected_map_status.get("code")),
            ready=text(selected_map_status.get("ready")),
            message=text(selected_map_status.get("message")),
        ),
        f"task_readiness={text(readiness.get('code'))}: {text(readiness.get('message'))}",
        "perception_status={code}: ready={ready} message={message}".format(
            code=text(perception_status.get("code")),
            ready=text(perception_status.get("ready")),
            message=text(perception_status.get("message")),
        ),
        f"battery_level={text(primary.get('level'))}%",
        f"scan_finite={text(scan.get('finite_ranges'))}",
        f"lidar_points={lidar_points} source={text(lidar.get('source'))}",
    ]
    if relay:
        lines.append(
            "lidar_relay output_points={points} stride={stride} in_hz={in_hz} out_hz={out_hz} skip={skip} max_points={max_points} min_interval={interval}".format(
                points=relay_output_points,
                stride=text(relay.get("output_stride")),
                in_hz="-" if relay_input_rate is None else f"{relay_input_rate:.1f}",
                out_hz="-" if relay_publish_rate is None else f"{relay_publish_rate:.1f}",
                skip="-" if relay_skip_ratio is None else f"{relay_skip_ratio:.0%}",
                max_points=text(relay.get("max_output_points")),
                interval=text(relay.get("min_publish_interval_s")),
            )
        )
        lines.append(
            "lidar_relay_input topic={topic} publishers={publishers} messages={messages} published={published} qos={qos} sub={sub}".format(
                topic=text(relay.get("input_topic")),
                publishers=text(relay.get("input_publisher_count")),
                messages=text(relay.get("messages")),
                published=text(relay.get("messages_published")),
                qos=text(relay.get("cloud_reliability")),
                sub=",".join(str(item) for item in relay.get("subscription_modes") or []) or "-",
            )
        )
        if relay.get("downsample_method"):
            lines.append(f"lidar_relay_method={text(relay.get('downsample_method'))}")
    return lines


def first_distance_text(readiness: Dict[str, Any]) -> str:
    distance = number(readiness.get("first_waypoint_distance_m"))
    if distance is None:
        return "-"
    warn = number(readiness.get("first_waypoint_distance_warn_m"))
    max_distance = number(readiness.get("first_waypoint_distance_max_m"))
    flags = []
    if max_distance is not None and max_distance > 0 and distance > max_distance:
        flags.append("over_max")
    elif warn is not None and warn > 0 and distance > warn:
        flags.append("far")
    suffix = f" ({','.join(flags)})" if flags else ""
    return f"{distance:.2f}m{suffix}"


def bad_waypoint_line(item: Dict[str, Any]) -> str:
    pose = item.get("pose") if isinstance(item.get("pose"), dict) else {}
    detail = item.get("detail") if isinstance(item.get("detail"), dict) else {}
    x = number(pose.get("x"))
    y = number(pose.get("y"))
    parts = [
        f"#{int(item.get('index', 0) or 0) + 1}",
        str(item.get("label") or item.get("annotation_id") or "-"),
    ]
    if x is not None and y is not None:
        parts.append(f"x={x:.2f} y={y:.2f}")
    if detail:
        parts.append(f"reason={text(detail.get('code'))}")
        if detail.get("grid"):
            parts.append(f"grid={text(detail.get('grid'))}")
        if detail.get("value") is not None:
            parts.append(f"value={text(detail.get('value'))}")
    return " ".join(parts)


def task_matches(task: Dict[str, Any], task_id: str, task_name: str) -> bool:
    if task_id and str(task.get("id") or "") != task_id:
        return False
    if task_name and task_name not in str(task.get("name") or ""):
        return False
    return True


def summarize_tasks(tasks_payload: Dict[str, Any], task_id: str = "", task_name: str = "") -> List[str]:
    tasks = tasks_payload.get("tasks") if isinstance(tasks_payload.get("tasks"), list) else []
    tasks = [
        task
        for task in tasks
        if isinstance(task, dict) and task_matches(task, task_id, task_name)
    ]
    if not tasks:
        hidden_count = int(tasks_payload.get("hidden_task_count") or 0)
        if hidden_count and not (task_id or task_name):
            return [f"tasks=none current_map; hidden_old_map_tasks={hidden_count}"]
        return ["tasks=none" if not (task_id or task_name) else "tasks=none matching filter"]
    lines = []
    for task in tasks:
        readiness = task.get("readiness") if isinstance(task.get("readiness"), dict) else {}
        waypoints = task.get("waypoints") if isinstance(task.get("waypoints"), list) else []
        ready = readiness.get("ready") is True
        first = waypoint_line(waypoints[0], 0) if waypoints and isinstance(waypoints[0], dict) else "-"
        lines.append(
            "task name={name} id={task_id} status={status} ready={ready} code={code} first_distance={first_distance} message={message}".format(
                name=text(task.get("name")),
                task_id=text(task.get("id")),
                status=text(task.get("status")),
                ready="true" if ready else "false",
                code=text(readiness.get("code")),
                first_distance=first_distance_text(readiness),
                message=text(readiness.get("message")),
            )
        )
        lines.append(f"  first={first}")
        lines.append(f"  order={compact_order(point for point in waypoints if isinstance(point, dict))}")
        bad_waypoints = readiness.get("bad_waypoints") if isinstance(readiness.get("bad_waypoints"), list) else []
        for item in bad_waypoints:
            if isinstance(item, dict):
                lines.append(f"  bad_waypoint={bad_waypoint_line(item)}")
    return lines


def advice_for_code(code: str, message: str) -> str:
    if code == "no_current_map_task":
        return "当前选中地图下没有可执行任务；先在当前地图重新标任务点并生成任务，不要直接复用旧地图任务。"
    if code == "ready":
        return "先启动 watcher，再从真实前端确认首点/顺序并点击开始。"
    if code in ("localization_not_confirmed", "pose_invalid_or_stale"):
        return "先在前端定位页完成重定位，确认机器人位置稳定后再检查任务。"
    if code in ("selected_map_mismatch", "map_metadata_mismatch", "map_unavailable", "selected_map_metadata_mismatch"):
        return "先切换到任务对应地图，并确认网页地图与 Nav2 当前地图一致。"
    if code in ("wrong_floor", "floor_unknown"):
        return "先确认当前楼层和任务首点楼层一致。"
    if code in ("current_pose_out_of_map", "target_out_of_map", "waypoint_out_of_map"):
        return "检查机器人当前位置、任务点坐标和地图范围；必要时重新标点。"
    if code in ("waypoint_on_occupied_cell", "waypoint_on_unknown_cell"):
        return "任务点落在障碍/未知栅格上；先在地图可通行区域重新标点，再重新生成任务。"
    if code == "first_waypoint_too_far":
        return "机器人到任务首点距离异常大；不要启动任务，先确认重定位、地图和首点是否属于同一现场。"
    if code in ("battery_missing", "battery_stale"):
        return "等待电池数据恢复；如果持续缺失，检查 /BATTERY_DATA 和 web 节点订阅。"
    if code == "battery_low":
        return "先充电，不要开始移动任务。"
    if code == "perception_scan_unavailable":
        return "先检查 /scan、pointcloud_fusion 和 TF；/scan 新鲜后再开始任务。"
    if code == "perception_lidar_unavailable":
        return "先检查 /m20pro/lidar_points_relay 和 DDS profile；104 必须能看到点云。"
    if code == "factory_lidar_points_publisher_missing":
        return "原厂 /LIDAR/POINTS 没有 DDS publisher；先恢复 rsdriver 到 ROS2 点云端点，再做地图/定位/任务。"
    if code == "lidar_relay_no_samples":
        return "relay 没有收到 /LIDAR/POINTS 样本；先查原厂点云 publisher、DDS profile 和 relay 日志。"
    if code == "lidar_relay_output_unavailable":
        return "relay 输入可能存在但前端没有新鲜 relay 点云；先查 /m20pro/lidar_points_relay。"
    if code == "navigation_not_ready":
        return "重定位后等待 Nav2 lifecycle 和 local/global costmap 恢复。"
    if code in ("task_running",):
        return "已有任务在执行，先停止/复位当前任务后再检查。"
    if code:
        return f"按 readiness 提示处理：{message or code}"
    return "先确认前端 /api/state 和 /api/tasks 是否正常返回。"


def state_level_advice(state: Dict[str, Any]) -> Optional[Tuple[str, str]]:
    battery = state.get("battery") if isinstance(state.get("battery"), dict) else {}
    primary = battery.get("primary") if isinstance(battery.get("primary"), dict) else {}
    level = number(primary.get("level"))
    if level is not None and level < 25:
        return ("battery_low", "电量低于任务启动阈值 25%，先充电；不要为了验证任务链路继续重定位或移动。")
    perception_status = state.get("perception_status") if isinstance(state.get("perception_status"), dict) else {}
    perception_code = str(perception_status.get("code") or "")
    if perception_code in (
        "factory_lidar_points_publisher_missing",
        "lidar_relay_no_samples",
        "lidar_relay_output_unavailable",
        "scan_unavailable",
    ):
        return (
            perception_code,
            str(perception_status.get("message") or advice_for_code(perception_code, "")),
        )
    selected_map_status = state.get("selected_map_status") if isinstance(state.get("selected_map_status"), dict) else {}
    if selected_map_status.get("ready") is False:
        return (
            str(selected_map_status.get("code") or "selected_map_metadata_mismatch"),
            str(selected_map_status.get("message") or "网页选择地图与 Nav2 当前加载地图不一致；先切换到正确地图并重定位。"),
        )
    localization_status = state.get("localization_status") if isinstance(state.get("localization_status"), dict) else {}
    if localization_status.get("confirmed") is not True:
        if localization_status.get("tcp_2101_required") is True:
            return (
                str(localization_status.get("code") or "localization_not_confirmed"),
                "先在前端定位页完成重定位；必须看到定位页显示重定位成功，再处理当前地图标点和任务。",
            )
        return (
            str(localization_status.get("code") or "localization_not_confirmed"),
            "先在前端定位页完成重定位，确认定位页显示重定位成功后，再处理当前地图标点和任务。",
        )
    if localization_status.get("task_ready") is not True:
        return (
            str(localization_status.get("task_readiness_code") or localization_status.get("code") or "localized_task_not_ready"),
            "定位已确认但任务页还不可启动；先按 task_readiness 处理 Nav2、地图、位姿或感知链路问题。",
        )
    return None


def matching_tasks(tasks_payload: Dict[str, Any], task_id: str = "", task_name: str = "") -> List[Dict[str, Any]]:
    tasks = tasks_payload.get("tasks") if isinstance(tasks_payload.get("tasks"), list) else []
    return [
        task
        for task in tasks
        if isinstance(task, dict) and task_matches(task, task_id, task_name)
    ]


def task_readiness(task: Dict[str, Any]) -> Dict[str, Any]:
    readiness = task.get("readiness")
    return readiness if isinstance(readiness, dict) else {}


def task_priority_tuple(task: Dict[str, Any], selected_map: str) -> Tuple[int, int, str]:
    readiness = task_readiness(task)
    code = str(readiness.get("code") or "")
    priority = {
        "ready": 0,
        "localization_not_confirmed": 1,
        "pose_invalid_or_stale": 1,
        "navigation_not_ready": 2,
        "perception_scan_unavailable": 3,
        "perception_lidar_unavailable": 3,
        "battery_low": 4,
        "selected_map_mismatch": 8,
        "map_metadata_mismatch": 8,
        "map_unavailable": 8,
    }
    return (
        priority.get(code, 5),
        0 if str(task.get("map_id") or "") == selected_map else 1,
        str(task.get("created_at") or ""),
    )


def recommended_task(
    state: Dict[str, Any],
    tasks_payload: Dict[str, Any],
    task_id: str,
    task_name: str,
) -> Optional[Dict[str, Any]]:
    tasks = matching_tasks(tasks_payload, task_id, task_name)
    if not tasks:
        return None
    selected_map = str(state.get("selected_map_id") or "")
    if not (task_id or task_name):
        tasks = [
            task
            for task in tasks
            if str(task.get("map_id") or "") == selected_map
        ]
        if not tasks:
            return None
    return min(tasks, key=lambda task: task_priority_tuple(task, selected_map))


def shell_quote(value: str) -> str:
    return "'" + str(value).replace("'", "'\"'\"'") + "'"


def safe_label(value: str) -> str:
    label = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value or "").strip())
    return label.strip("._") or "field_task_short_run"


def summarize_recommended_task(task: Optional[Dict[str, Any]]) -> List[str]:
    if not task:
        return ["recommended_task=none"]
    readiness = task_readiness(task)
    waypoints = task.get("waypoints") if isinstance(task.get("waypoints"), list) else []
    first = waypoint_line(waypoints[0], 0) if waypoints and isinstance(waypoints[0], dict) else "-"
    task_id = str(task.get("id") or "")
    task_name = str(task.get("name") or "")
    task_suffix = task_id[-8:] if task_id else ""
    label = safe_label("_".join(part for part in ("field_task", task_suffix) if part))
    lines = [
        "recommended_task name={name} id={task_id} status={status} ready={ready} code={code} first_distance={first_distance}".format(
            name=text(task_name),
            task_id=text(task_id),
            status=text(task.get("status")),
            ready="true" if readiness.get("ready") is True else "false",
            code=text(readiness.get("code")),
            first_distance=first_distance_text(readiness),
        ),
        f"recommended_first={first}",
        "watcher_command=./scripts/104_watch_frontend_task.sh 180 {label}".format(
            label=shell_quote(label),
        ),
    ]
    if task_id:
        lines.append(
            "ready_check_command=./scripts/104_frontend_task_ready_check.py --task-id {task_id}".format(
                task_id=shell_quote(task_id),
            )
        )
    return lines


def advice_readiness_source(
    state: Dict[str, Any],
    tasks_payload: Dict[str, Any],
    task_id: str,
    task_name: str,
) -> Tuple[Dict[str, Any], str]:
    state_readiness = state.get("task_readiness") if isinstance(state.get("task_readiness"), dict) else {}
    tasks = matching_tasks(tasks_payload, task_id, task_name)
    if task_id or task_name:
        if tasks:
            return task_readiness(tasks[0]), "selected_task"
        return state_readiness, "state"
    selected_map = str(state.get("selected_map_id") or "")
    same_map_tasks = [
        task
        for task in tasks
        if str(task.get("map_id") or "") == selected_map
    ]
    if same_map_tasks:
        best_task = min(same_map_tasks, key=lambda task: task_priority_tuple(task, selected_map))
        return task_readiness(best_task), "current_map_task"
    if tasks:
        return {
            "ready": False,
            "code": "no_current_map_task",
            "message": "当前选中地图下没有任务；旧任务属于其他地图，不能作为本次现场验证任务",
            "selected_map_id": selected_map,
            "other_task_count": len(tasks),
        }, "state"
    return state_readiness, "state"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://10.21.31.104:8080", help="M20Pro frontend base URL")
    parser.add_argument("--task-id", default="", help="Only print/check this task id")
    parser.add_argument("--task-name", default="", help="Only print/check tasks whose name contains this text")
    parser.add_argument("--json", action="store_true", help="Print raw state/tasks JSON as well")
    args = parser.parse_args()

    try:
        state = fetch_json(args.url, "/api/state")
        tasks_path = "/api/tasks?include_all=1" if (args.task_id or args.task_name) else "/api/tasks"
        tasks = fetch_json(args.url, tasks_path)
    except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError, RuntimeError) as exc:
        print(f"[FAIL] cannot query frontend: {exc}", file=sys.stderr)
        return 2
    print("[104_frontend_task_ready_check] read-only")
    print(f"url={args.url.rstrip('/')}")
    print("state:")
    for line in summarize_state(state):
        print(f"  {line}")
    print("tasks:")
    for line in summarize_tasks(tasks, task_id=args.task_id, task_name=args.task_name):
        print(f"  {line}")
    print("recommended:")
    recommended = recommended_task(state, tasks, args.task_id, args.task_name)
    for line in summarize_recommended_task(recommended):
        print(f"  {line}")
    task_readiness = state.get("task_readiness") if isinstance(state.get("task_readiness"), dict) else {}
    advice_source, advice_source_name = advice_readiness_source(state, tasks, args.task_id, args.task_name)
    advice_code = str(advice_source.get("code") or "")
    advice_message = str(advice_source.get("message") or "")
    state_advice = state_level_advice(state)
    print("next:")
    if state_advice:
        next_code, next_message = state_advice
        print(f"  {next_message}")
        print(f"  advice_source=state code={text(next_code)}")
    else:
        print(f"  {advice_for_code(advice_code, advice_message)}")
        print(f"  advice_source={advice_source_name} code={text(advice_code)}")
    if args.json:
        print("raw:")
        print(json.dumps({"state": state, "tasks": tasks}, ensure_ascii=False, indent=2))

    filtered_tasks = matching_tasks(tasks, args.task_id, args.task_name)
    tasks_ready = [
        task
        for task in filtered_tasks
        if isinstance(task.get("readiness"), dict)
        and task["readiness"].get("ready") is True
    ]
    if (args.task_id or args.task_name) and not filtered_tasks:
        print("[FAIL] no task matched the requested filter")
        return 2
    if task_readiness.get("ready") is True and tasks_ready:
        print("[OK] at least one matching task is ready to start from the frontend")
        return 0
    print("[WARN] no matching task is ready to start from the frontend")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
