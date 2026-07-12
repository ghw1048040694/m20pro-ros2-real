"""Pure mapping-session helpers for the web dashboard."""

from __future__ import annotations

import re
from typing import Any, Callable, Dict, List, Optional


IdFactory = Callable[[str], str]
NowText = Callable[[], str]


def sanitize_mapping_name(value: str, fallback: str) -> str:
    text = str(value or "").strip() or str(fallback or "map")
    text = re.sub(r"[^\w.\-\u4e00-\u9fff]+", "_", text)
    return text.strip("._") or str(fallback or "map")


def normalize_mapping_session_request(
    payload: Dict[str, Any],
    *,
    default_project_name: str,
    default_map_name: str,
) -> Dict[str, Any]:
    project_name = str(payload.get("project_name") or payload.get("name") or "").strip()
    building = str(payload.get("building") or "").strip()
    mode = str(payload.get("mode") or "multi").strip()
    floors_raw = payload.get("floors") or []
    if isinstance(floors_raw, str):
        floors_raw = [item.strip() for item in floors_raw.split(",") if item.strip()]
    floors = [str(item).strip() for item in floors_raw if str(item).strip()]
    active_floor = str(payload.get("active_floor") or (floors[0] if floors else "")).strip()
    fallback_name = default_map_name.format(active_floor=active_floor or "map")
    map_name = sanitize_mapping_name(str(payload.get("map_name") or ""), fallback_name)
    return {
        "project_name": project_name or str(default_project_name or "M20Pro 工地巡检"),
        "building": building,
        "mode": mode,
        "floors": floors,
        "active_floor": active_floor,
        "map_name": map_name,
    }


def find_mapping_project(projects: List[Dict[str, Any]], name: str, building: str) -> Optional[Dict[str, Any]]:
    for item in projects:
        if item.get("name") == name and item.get("building", "") == building:
            return item
    return None


def build_mapping_project_record(
    *,
    project_id: str,
    name: str,
    building: str,
    created_at: str,
) -> Dict[str, Any]:
    return {
        "id": str(project_id or ""),
        "name": str(name or ""),
        "building": str(building or ""),
        "created_at": str(created_at or ""),
    }


def build_mapping_session_record(
    *,
    session_id: str,
    project: Dict[str, Any],
    request: Dict[str, Any],
    created_at: str,
) -> Dict[str, Any]:
    floors = list(request.get("floors") or [])
    active_floor = str(request.get("active_floor") or "")
    base_map_name = str(request.get("map_name") or "")
    floor_steps = [
        {
            "floor": str(floor),
            "map_name": floor_map_name(base_map_name, active_floor, str(floor), len(floors) > 1),
            "status": "ready" if str(floor) == active_floor else "pending",
            "updated_at": str(created_at or ""),
        }
        for floor in floors
    ]
    return {
        "id": str(session_id or ""),
        "project_id": str(project.get("id") or ""),
        "project_name": str(request.get("project_name") or ""),
        "building": str(request.get("building") or ""),
        "mode": str(request.get("mode") or ""),
        "floors": floors,
        "active_floor": active_floor,
        "map_name": active_mapping_step(floor_steps, active_floor).get("map_name") or base_map_name,
        "floor_steps": floor_steps,
        "status": "created",
        "created_at": str(created_at or ""),
        "updated_at": str(created_at or ""),
    }


def floor_map_name(base_name: str, active_floor: str, floor: str, multi_floor: bool) -> str:
    base_name = sanitize_mapping_name(base_name, floor or "map")
    if not multi_floor:
        return base_name
    active_floor = str(active_floor or "").strip()
    floor = str(floor or "").strip()
    if active_floor and base_name.startswith(active_floor + "_"):
        return sanitize_mapping_name(floor + base_name[len(active_floor):], floor)
    if base_name.endswith("_" + active_floor) and active_floor:
        return sanitize_mapping_name(base_name[: -len(active_floor)] + floor, floor)
    return sanitize_mapping_name(f"{base_name}_{floor}", floor)


def active_mapping_step(steps: Any, active_floor: Any) -> Dict[str, Any]:
    floor = str(active_floor or "").strip()
    for item in steps if isinstance(steps, list) else []:
        if isinstance(item, dict) and str(item.get("floor") or "").strip() == floor:
            return item
    return {}


def mapping_floor_steps(session: Dict[str, Any]) -> List[Dict[str, Any]]:
    existing = session.get("floor_steps")
    if isinstance(existing, list) and existing:
        return [dict(item) for item in existing if isinstance(item, dict)]
    floors = [str(item).strip() for item in (session.get("floors") or []) if str(item).strip()]
    active_floor = str(session.get("active_floor") or "").strip()
    base_map_name = str(session.get("map_name") or "")
    return [
        {
            "floor": floor,
            "map_name": (
                sanitize_mapping_name(base_map_name, floor or "map")
                if floor == active_floor
                else floor_map_name(base_map_name, active_floor, floor, len(floors) > 1)
            ),
            "status": str(session.get("status") or "ready") if floor == active_floor else "pending",
            "updated_at": str(session.get("updated_at") or session.get("created_at") or ""),
        }
        for floor in floors
    ]


def select_mapping_floor(session: Dict[str, Any], floor: Any, *, updated_at: str) -> Dict[str, Any]:
    floor = str(floor or "").strip()
    steps = mapping_floor_steps(session)
    step = active_mapping_step(steps, floor)
    if not step:
        return {"ok": False, "code": "mapping_floor_missing", "message": "建图任务中没有该楼层"}
    if str(session.get("status") or "") == "mapping":
        return {"ok": False, "code": "mapping_floor_busy", "message": "当前楼层正在建图，完成或取消后才能切换步骤"}
    updated = dict(session)
    updated["floor_steps"] = steps
    updated["active_floor"] = floor
    updated["map_name"] = str(step.get("map_name") or "")
    updated["status"] = str(step.get("status") or "ready")
    updated["updated_at"] = str(updated_at or "")
    return {"ok": True, "session": updated, "step": step}


def prepare_mapping_session_create(
    payload: Dict[str, Any],
    *,
    projects: List[Dict[str, Any]],
    id_factory: IdFactory,
    now_text: NowText,
    default_project_name: str,
    default_map_name: str,
) -> Dict[str, Any]:
    request = normalize_mapping_session_request(
        payload,
        default_project_name=default_project_name,
        default_map_name=default_map_name,
    )
    project = find_mapping_project(projects, request["project_name"], request["building"])
    created_project = None
    if project is None:
        project = build_mapping_project_record(
            project_id=id_factory("project"),
            name=request["project_name"],
            building=request["building"],
            created_at=now_text(),
        )
        created_project = project
    session = build_mapping_session_record(
        session_id=id_factory("map_session"),
        project=project,
        request=request,
        created_at=now_text(),
    )
    return {
        "request": request,
        "project": project,
        "created_project": created_project,
        "session": session,
    }


def mapping_command_status(param_name: str, current_status: Any, result: Dict[str, Any]) -> str:
    if result.get("ok"):
        return {
            "factory_mapping_start_command": "mapping",
            "factory_mapping_finish_command": "saved",
            "factory_mapping_cancel_command": "cancelled",
        }.get(str(param_name or ""), str(current_status or "updated"))
    if result.get("manual_required"):
        return "waiting_manual"
    return str(current_status or "updated")


def apply_mapping_command_result(
    session: Dict[str, Any],
    *,
    param_name: str,
    result: Dict[str, Any],
    updated_at: str,
) -> Dict[str, Any]:
    updated = dict(session)
    updated["status"] = mapping_command_status(param_name, session.get("status"), result)
    updated["updated_at"] = str(updated_at or "")
    steps = mapping_floor_steps(session)
    step = active_mapping_step(steps, session.get("active_floor"))
    if step:
        step["status"] = updated["status"]
        step["updated_at"] = str(updated_at or "")
        updated["floor_steps"] = steps
    return updated


def mark_mapping_floor_imported(
    session: Dict[str, Any],
    *,
    floor: str,
    map_id: str,
    updated_at: str,
) -> Dict[str, Any]:
    updated = dict(session)
    steps = mapping_floor_steps(session)
    step = active_mapping_step(steps, floor)
    if step:
        step["status"] = "imported"
        step["map_id"] = str(map_id or "")
        step["updated_at"] = str(updated_at or "")
    updated["floor_steps"] = steps
    updated["active_floor"] = str(floor or "")
    updated["map_name"] = str(step.get("map_name") or updated.get("map_name") or "")
    updated["status"] = "imported"
    updated["updated_at"] = str(updated_at or "")
    return updated


def mapping_command_context(
    session: Dict[str, Any],
    *,
    factory_host: str,
    factory_user: str,
    factory_active_map: str,
    map_archive_dir: str,
) -> Dict[str, str]:
    step = active_mapping_step(mapping_floor_steps(session), session.get("active_floor"))
    return {
        "session_id": str(session.get("id", "")),
        "project_name": str(session.get("project_name", "")),
        "building": str(session.get("building", "")),
        "mode": str(session.get("mode", "")),
        "active_floor": str(session.get("active_floor", "")),
        "map_name": sanitize_mapping_name(
            str(step.get("map_name") or session.get("map_name") or ""),
            str(session.get("id", "map")),
        ),
        "floors": ",".join(str(item) for item in session.get("floors") or []),
        "factory_host": str(factory_host or ""),
        "factory_user": str(factory_user or ""),
        "factory_active_map": str(factory_active_map or ""),
        "map_archive_dir": str(map_archive_dir or ""),
    }
