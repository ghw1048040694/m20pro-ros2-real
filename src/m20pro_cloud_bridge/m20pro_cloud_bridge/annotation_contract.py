"""Pure annotation helpers for the M20Pro web dashboard."""

from __future__ import annotations

import re
from typing import Any, Callable, Dict, List, Optional

from .localization_contract import pose_is_plausible
from .task_contract import pose_map_bounds_error, pose_map_occupancy_error, readiness_failure, readiness_success


NowText = Callable[[], str]

MANUAL_POINT_TYPES: Dict[str, Dict[str, Any]] = {
    "transition": {
        "label": "过渡点",
        "point_info": 0,
        "default_dwell_s": 0.0,
        "default_nav_mode": 0,
    },
    "task": {
        "label": "任务点",
        "point_info": 1,
        "default_dwell_s": 5.0,
        "default_nav_mode": 1,
    },
    "charge": {
        "label": "充电点",
        "point_info": 3,
        "default_dwell_s": 0.0,
        "default_nav_mode": 1,
    },
}

UI_TYPE_TO_MANUAL_POINT_TYPE = {
    "patrol": "task",
    "task": "task",
    "inspection": "task",
    "transition": "transition",
    "stair_entry": "transition",
    "stair_switch": "transition",
    "stair_exit": "transition",
    "charge": "charge",
    "charging": "charge",
}

DEFAULT_VENDOR_NAVIGATION = {
    "Value": 0,
    "MapID": 0,
    "Gait": 12,
    "Speed": 1,
    "Manner": 0,
    "ObsMode": 0,
    "NavMode": 1,
}


def _sanitize_name(value: str, fallback: str) -> str:
    text = str(value or "").strip() or fallback
    text = re.sub(r"[^\w.\-\u4e00-\u9fff]+", "_", text)
    return text.strip("._") or fallback


def string_list(value: Any) -> List[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    if isinstance(value, list):
        return [str(item).strip() for item in value if str(item).strip()]
    return []


def annotation_result_prefix(item: Dict[str, Any]) -> str:
    parts = [
        str(item.get("floor") or "").strip(),
        str(item.get("area") or "").strip(),
        str(item.get("room") or "").strip(),
        str(item.get("label") or item.get("id") or "").strip(),
    ]
    raw = "_".join(part for part in parts if part)
    return _sanitize_name(raw, str(item.get("id") or "inspection_result"))


def manual_point_type_from_payload(payload: Dict[str, Any]) -> str:
    value = str(payload.get("manual_point_type") or "").strip()
    if value in MANUAL_POINT_TYPES:
        return value
    legacy_type = str(payload.get("type") or "patrol").strip()
    return UI_TYPE_TO_MANUAL_POINT_TYPE.get(legacy_type, "task")


def resolve_annotation_dwell_s(
    payload: Dict[str, Any],
    *,
    default_task_dwell_s: float,
    default_transition_dwell_s: float,
    default_charge_dwell_s: float,
) -> float:
    raw = payload.get("dwell_s", payload.get("inspect_duration_s", payload.get("stay_duration_s")))
    if raw is not None and str(raw).strip() != "":
        try:
            return max(0.0, float(raw))
        except (TypeError, ValueError):
            return 0.0
    manual_type = manual_point_type_from_payload(payload)
    if manual_type == "transition":
        return max(0.0, float(default_transition_dwell_s))
    if manual_type == "charge":
        return max(0.0, float(default_charge_dwell_s))
    return max(0.0, float(default_task_dwell_s))


def vendor_navigation_from_payload(payload: Dict[str, Any]) -> Dict[str, int]:
    manual_type = manual_point_type_from_payload(payload)
    defaults = dict(DEFAULT_VENDOR_NAVIGATION)
    defaults["PointInfo"] = int(MANUAL_POINT_TYPES[manual_type]["point_info"])
    defaults["NavMode"] = int(MANUAL_POINT_TYPES[manual_type]["default_nav_mode"])
    raw = payload.get("vendor_navigation") or {}
    if not isinstance(raw, dict):
        raw = {}
    aliases = {
        "value": "Value",
        "map_id": "MapID",
        "point_info": "PointInfo",
        "gait": "Gait",
        "speed": "Speed",
        "manner": "Manner",
        "obs_mode": "ObsMode",
        "nav_mode": "NavMode",
    }
    for key, canonical in aliases.items():
        if key in payload:
            raw[canonical] = payload[key]
    for key in list(defaults.keys()) + ["PointInfo"]:
        if key not in raw:
            continue
        try:
            defaults[key] = int(raw[key])
        except (TypeError, ValueError):
            pass
    return defaults


def annotation_create_static_context(
    payload: Dict[str, Any],
    *,
    default_label_index: int,
) -> Dict[str, Any]:
    pose = payload.get("pose") or {}
    try:
        x = float(pose.get("x"))
        y = float(pose.get("y"))
        z = float(pose.get("z", 0.0))
        yaw = float(pose.get("yaw", 0.0))
    except (TypeError, ValueError):
        return {
            "ok": False,
            "message": "点位坐标无效，请先点击地图取点",
            "code": "annotation_pose_invalid",
        }

    floor = str(payload.get("floor") or "").strip()
    if not floor:
        return {
            "ok": False,
            "message": "点位楼层不能为空",
            "code": "annotation_floor_missing",
        }

    point_type = str(payload.get("type") or "patrol").strip()
    label = str(payload.get("label") or "").strip()
    if not label:
        label = f"{floor}_{point_type}_{max(1, int(default_label_index))}"
    map_id = str(payload.get("map_id") or "").strip() or None
    if map_id == "live_map":
        map_id = "live_map"

    return {
        "ok": True,
        "pose": {"x": x, "y": y, "z": z, "yaw": yaw},
        "floor": floor,
        "type": point_type,
        "label": label,
        "map_id": map_id,
    }


def build_annotation_record(
    payload: Dict[str, Any],
    context: Dict[str, Any],
    *,
    annotation_id: str,
    map_id: str,
    dwell_s: float,
    now_text_value: str,
) -> Dict[str, Any]:
    payload_with_context = dict(payload)
    payload_with_context["type"] = str(context.get("type") or payload.get("type") or "patrol").strip()
    item = {
        "id": annotation_id,
        "map_id": map_id,
        "type": payload_with_context["type"],
        "floor": str(context.get("floor") or "").strip(),
        "label": str(context.get("label") or "").strip(),
        "area": str(payload.get("area") or payload.get("region") or "").strip(),
        "room": str(payload.get("room") or payload.get("place") or "").strip(),
        "result_file_prefix": str(payload.get("result_file_prefix") or "").strip(),
        "pose": dict(context.get("pose") or {}),
        "dwell_s": max(0.0, float(dwell_s)),
        "manual_point_type": manual_point_type_from_payload(payload_with_context),
        "vendor_navigation": vendor_navigation_from_payload(payload_with_context),
        "camera": str(payload.get("camera") or "").strip(),
        "target_classes": string_list(payload.get("target_classes")),
        "notes": str(payload.get("notes") or "").strip(),
        "created_at": now_text_value,
    }
    return normalize_annotation_semantics(item)


def annotation_map_pose_error_payload(
    point_pose: Dict[str, Any],
    target_map_payload: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    bounds_error = pose_map_bounds_error(point_pose, target_map_payload, "保存点位")
    if bounds_error:
        return {
            "ok": False,
            "code": "annotation_out_of_map",
            "message": str(bounds_error["message"]),
            "detail": bounds_error,
        }

    occupancy_error = pose_map_occupancy_error(point_pose, target_map_payload, "保存点位")
    if occupancy_error:
        code = (
            "annotation_on_occupied_cell"
            if occupancy_error.get("code") == "pose_on_occupied_cell"
            else "annotation_on_unknown_cell"
        )
        return {
            "ok": False,
            "code": code,
            "message": str(occupancy_error["message"]),
            "detail": occupancy_error,
        }

    return None


def normalize_annotation_semantics(item: Dict[str, Any]) -> Dict[str, Any]:
    legacy_type = str(item.get("type") or "patrol").strip()
    manual_type = str(item.get("manual_point_type") or "").strip()
    if manual_type not in MANUAL_POINT_TYPES:
        manual_type = UI_TYPE_TO_MANUAL_POINT_TYPE.get(legacy_type, "task")
    item["manual_point_type"] = manual_type

    vendor = item.get("vendor_navigation")
    if not isinstance(vendor, dict):
        vendor = {}
    merged = dict(DEFAULT_VENDOR_NAVIGATION)
    merged["PointInfo"] = int(MANUAL_POINT_TYPES[manual_type]["point_info"])
    merged["NavMode"] = int(MANUAL_POINT_TYPES[manual_type]["default_nav_mode"])
    for key in merged:
        if key not in vendor:
            continue
        try:
            merged[key] = int(vendor[key])
        except (TypeError, ValueError):
            pass
    item["vendor_navigation"] = merged

    if "dwell_s" not in item and "inspect_duration_s" in item:
        item["dwell_s"] = item.get("inspect_duration_s")
    try:
        item["dwell_s"] = max(0.0, float(item.get("dwell_s", MANUAL_POINT_TYPES[manual_type]["default_dwell_s"])))
    except (TypeError, ValueError):
        item["dwell_s"] = float(MANUAL_POINT_TYPES[manual_type]["default_dwell_s"])
    item["inspect_duration_s"] = item["dwell_s"]

    item["label"] = str(item.get("label") or item.get("name") or item.get("id") or "").strip()
    item["area"] = str(item.get("area") or item.get("region") or "").strip()
    item["room"] = str(item.get("room") or item.get("place") or item.get("location") or "").strip()
    result_prefix = str(item.get("result_file_prefix") or "").strip()
    item["result_file_prefix"] = result_prefix or annotation_result_prefix(item)

    if "camera" not in item:
        item["camera"] = ""
    item["target_classes"] = string_list(item.get("target_classes"))
    return item


def annotation_dwell_s(annotation: Dict[str, Any]) -> float:
    normalize_annotation_semantics(annotation)
    try:
        return max(0.0, float(annotation.get("dwell_s", 0.0)))
    except (TypeError, ValueError):
        return 0.0


def annotation_semantics_payload(annotation: Dict[str, Any]) -> Dict[str, Any]:
    normalize_annotation_semantics(annotation)
    pose = annotation.get("pose") or {}
    vendor = dict(annotation.get("vendor_navigation") or {})
    try:
        vendor["PosX"] = float(pose.get("x", 0.0))
        vendor["PosY"] = float(pose.get("y", 0.0))
        vendor["PosZ"] = float(pose.get("z", 0.0))
        vendor["AngleYaw"] = float(pose.get("yaw", 0.0))
    except (TypeError, ValueError):
        pass
    return {
        "id": annotation.get("id"),
        "label": annotation.get("label"),
        "area": annotation.get("area"),
        "room": annotation.get("room"),
        "result_file_prefix": annotation.get("result_file_prefix"),
        "floor": annotation.get("floor"),
        "type": annotation.get("type"),
        "manual_point_type": annotation.get("manual_point_type"),
        "manual_point_type_label": MANUAL_POINT_TYPES[annotation["manual_point_type"]]["label"],
        "pose": dict(pose),
        "yaw": float(pose.get("yaw", 0.0) or 0.0),
        "dwell_s": annotation_dwell_s(annotation),
        "camera": annotation.get("camera"),
        "target_classes": list(annotation.get("target_classes") or []),
        "vendor_navigation": vendor,
    }


def annotation_list_filter_payload(
    annotations: List[Dict[str, Any]],
    *,
    map_id: Optional[str] = None,
) -> Dict[str, Any]:
    selected_map_id = str(map_id or "").strip()
    items = [dict(item) for item in annotations if isinstance(item, dict)]
    if selected_map_id:
        filtered = [item for item in items if str(item.get("map_id") or "") == selected_map_id]
    else:
        filtered = list(items)
    return {
        "ok": True,
        "annotations": filtered,
        "hidden_annotation_count": len(items) - len(filtered),
        "total_annotation_count": len(items),
    }


def annotation_create_readiness_payload(
    *,
    map_id: Optional[str],
    selected_map_id: Optional[str],
    selected_map_status: Optional[Dict[str, Any]],
    map_relocalization_required: Optional[Dict[str, Any]],
    pose: Dict[str, Any],
    localization_ok: Any,
    pose_age_sec: Optional[float],
    pose_timeout_s: float,
    require_live_pose: bool = True,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    bounded_timeout_s = max(0.5, float(pose_timeout_s))
    if not map_id or map_id == "live_map":
        return readiness_failure(
            "annotation_fixed_map_required",
            "实时 /map 只用于临时观察；请先选择固定地图，再保存任务点位",
            {"map_id": map_id, "selected_map_id": selected_map_id},
            now_text=now_text,
        )
    if not require_live_pose:
        return readiness_success(
            "点位保存条件已满足；手动地图标点不要求机器狗位于当前查看楼层",
            {
                "map_id": map_id,
                "selected_map_id": selected_map_id,
                "require_live_pose": False,
            },
            now_text=now_text,
        )
    if not selected_map_id:
        return readiness_failure(
            "annotation_selected_map_required",
            "请先在前端选择当前固定地图，再保存任务点位",
            {"map_id": map_id, "selected_map_id": selected_map_id},
            now_text=now_text,
        )
    if str(map_id) != str(selected_map_id):
        return readiness_failure(
            "annotation_map_mismatch",
            "点位地图与当前选中地图不一致；请重新选择当前固定地图后再标点",
            {"map_id": map_id, "selected_map_id": selected_map_id},
            now_text=now_text,
        )
    selected_status = dict(selected_map_status or {})
    if not selected_status.get("ready"):
        return readiness_failure(
            "annotation_map_metadata_mismatch",
            str(selected_status.get("message") or "网页选择地图与 Nav2 当前加载地图不一致"),
            {
                "map_id": map_id,
                "selected_map_id": selected_map_id,
                "selected_map_status": selected_status,
            },
            now_text=now_text,
        )
    if map_relocalization_required:
        return readiness_failure(
            "annotation_map_relocalization_required",
            "Nav2 已加载当前固定地图，请先按开发手册2101完成重定位，再保存点位",
            {
                "map_id": map_id,
                "selected_map_id": selected_map_id,
                "map_relocalization_required": dict(map_relocalization_required),
            },
            now_text=now_text,
        )
    if localization_ok is not True:
        return readiness_failure(
            "annotation_localization_not_confirmed",
            "先完成重定位，看到手册2101成功、定位已确认后再保存点位",
            {
                "map_id": map_id,
                "selected_map_id": selected_map_id,
                "localization_ok": localization_ok,
                "pose_age_sec": pose_age_sec,
                "pose_timeout_s": bounded_timeout_s,
            },
            now_text=now_text,
        )
    if not pose_is_plausible(pose) or pose_age_sec is None or pose_age_sec > bounded_timeout_s:
        return readiness_failure(
            "annotation_pose_invalid_or_stale",
            "定位已确认但地图位姿无效或过期，请等待 /m20pro_tcp_bridge/map_pose 刷新后再保存点位",
            {
                "map_id": map_id,
                "selected_map_id": selected_map_id,
                "pose_age_sec": pose_age_sec,
                "pose_timeout_s": bounded_timeout_s,
            },
            now_text=now_text,
        )
    return readiness_success(
        "点位保存条件已满足",
        {
            "map_id": map_id,
            "selected_map_id": selected_map_id,
            "pose_age_sec": pose_age_sec,
            "pose_timeout_s": bounded_timeout_s,
        },
        now_text=now_text,
    )
