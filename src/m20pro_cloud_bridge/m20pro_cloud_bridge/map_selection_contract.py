"""Pure selected-map versus live Nav2 map status helpers."""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, Optional

from .task_contract import map_metadata_mismatch_error, readiness_failure, readiness_success


NowText = Callable[[], str]


def default_now_text() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def _map_summary(map_payload: Dict[str, Any], *, include_map_id: bool) -> Dict[str, Any]:
    summary = {
        "available": bool(map_payload.get("available")),
        "width": map_payload.get("width"),
        "height": map_payload.get("height"),
        "resolution": map_payload.get("resolution"),
        "origin": map_payload.get("origin"),
    }
    if include_map_id:
        summary = {
            "map_id": map_payload.get("map_id"),
            "name": map_payload.get("name"),
            "floor": map_payload.get("floor"),
            **summary,
        }
    return summary


def selected_map_status_payload(
    *,
    selected_map_id: Optional[str],
    live_map: Dict[str, Any],
    selected_map: Dict[str, Any],
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    selected_id = str(selected_map_id or "").strip()
    if not selected_id:
        return readiness_failure(
            "selected_map_missing",
            "当前没有选中固定地图；实时 /map 不能保存任务点或生成任务",
            {"selected_map_id": None},
            now_text=now_text or default_now_text,
        )

    live_payload = dict(live_map or {})
    selected_payload = dict(selected_map or {})
    detail = {
        "selected_map_id": selected_id,
        "selected_map": _map_summary(selected_payload, include_map_id=True),
        "live_map": _map_summary(live_payload, include_map_id=False),
    }
    metadata_error = map_metadata_mismatch_error(live_payload, selected_payload)
    if metadata_error:
        return readiness_failure(
            "selected_map_metadata_mismatch",
            str(metadata_error.get("message") or "网页选择地图与 Nav2 当前加载地图不一致"),
            {**detail, "detail": metadata_error},
            now_text=now_text or default_now_text,
        )
    return readiness_success(
        "网页选择地图与 Nav2 当前 /map 一致",
        detail,
        now_text=now_text or default_now_text,
    )


def map_relocalization_required_payload(
    *,
    map_id: Optional[str],
    map_name: Optional[str],
    yaml_path: Optional[str],
    reason: str,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    reason_key = str(reason or "map_loaded").strip()
    if reason_key == "startup_sync":
        message = "启动后已把当前固定地图同步到 Nav2，必须重新按开发手册2101定位"
    else:
        message = "当前固定地图已选择并同步到 Nav2，必须重新按开发手册2101定位"
    return {
        "map_id": map_id,
        "map_name": map_name,
        "yaml_path": yaml_path,
        "loaded_at": (now_text or default_now_text)(),
        "reason": reason_key,
        "message": message,
    }


def selected_map_wait_timeout_payload(
    *,
    selected_map_id: Optional[str],
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    return readiness_failure(
        "selected_map_metadata_mismatch",
        "等待 Nav2 /map 更新超时",
        {"selected_map_id": selected_map_id},
        now_text=now_text or default_now_text,
    )


def apply_selected_map_choice_state(
    settings: Dict[str, Any],
    *,
    map_id: Optional[str],
    previous_map_id: Optional[str],
    record: Optional[Dict[str, Any]],
    nav2_load: Dict[str, Any],
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    updated = dict(settings)
    selected_id = str(map_id or "").strip() or None
    selection_changed = str(previous_map_id or "") != str(selected_id or "")
    updated["selected_map_id"] = selected_id
    relocalization_required = None
    if selected_id and (selection_changed or bool(nav2_load.get("loaded"))):
        relocalization_required = map_relocalization_required_payload(
            map_id=selected_id,
            map_name=record.get("name") if isinstance(record, dict) else None,
            yaml_path=nav2_load.get("yaml_path"),
            reason="manual_select",
            now_text=now_text or default_now_text,
        )
        updated["map_relocalization_required"] = relocalization_required
    elif not selected_id:
        updated.pop("map_relocalization_required", None)
    else:
        relocalization_required = updated.get("map_relocalization_required")
    return {
        "settings": updated,
        "selection_changed": selection_changed,
        "relocalization_required": relocalization_required,
        "clear_pose": bool(selected_id and (selection_changed or bool(nav2_load.get("loaded")))),
    }
