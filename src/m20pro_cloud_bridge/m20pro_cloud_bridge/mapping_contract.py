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
    return {
        "id": str(session_id or ""),
        "project_id": str(project.get("id") or ""),
        "project_name": str(request.get("project_name") or ""),
        "building": str(request.get("building") or ""),
        "mode": str(request.get("mode") or ""),
        "floors": list(request.get("floors") or []),
        "active_floor": str(request.get("active_floor") or ""),
        "map_name": str(request.get("map_name") or ""),
        "status": "created",
        "created_at": str(created_at or ""),
        "updated_at": str(created_at or ""),
    }


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
    return updated


def mapping_command_context(
    session: Dict[str, Any],
    *,
    factory_host: str,
    factory_user: str,
    factory_active_map: str,
    map_archive_dir: str,
) -> Dict[str, str]:
    return {
        "session_id": str(session.get("id", "")),
        "project_name": str(session.get("project_name", "")),
        "building": str(session.get("building", "")),
        "mode": str(session.get("mode", "")),
        "active_floor": str(session.get("active_floor", "")),
        "map_name": sanitize_mapping_name(
            str(session.get("map_name") or ""),
            str(session.get("id", "map")),
        ),
        "floors": ",".join(str(item) for item in session.get("floors") or []),
        "factory_host": str(factory_host or ""),
        "factory_user": str(factory_user or ""),
        "factory_active_map": str(factory_active_map or ""),
        "map_archive_dir": str(map_archive_dir or ""),
    }
