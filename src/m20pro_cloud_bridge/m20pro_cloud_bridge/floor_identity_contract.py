"""Floor identity rules shared by mapping, maps, localization and tasks."""

from __future__ import annotations

import copy
import re
from typing import Any, Dict, Iterable, List


def configured_floor_ids(config: Dict[str, Any]) -> List[str]:
    floors = config.get("floors") if isinstance(config.get("floors"), dict) else {}
    return [str(floor).strip() for floor in floors if str(floor).strip()]


def normalize_floor_id(value: Any) -> str:
    """Normalize operator floor text while preserving a stable internal identity."""
    text = str(value or "").strip().upper().replace(" ", "")
    if not text:
        return ""
    text = re.sub(r"(?:楼|层)$", "", text)
    basement = re.fullmatch(r"(?:地下|负|B)(\d+)", text)
    if basement:
        return "B%d" % int(basement.group(1))
    signed = re.fullmatch(r"([+-]?)(\d+)", text)
    if signed:
        level = int(signed.group(2))
        return ("B%d" % level) if signed.group(1) == "-" else ("F%d" % level)
    above_ground = re.fullmatch(r"F\+?(\d+)", text)
    if above_ground:
        return "F%d" % int(above_ground.group(1))
    if re.fullmatch(r"[A-Z][A-Z0-9_-]{0,15}", text):
        return text
    return ""


def floor_level_from_id(value: Any) -> Any:
    floor_id = normalize_floor_id(value)
    above = re.fullmatch(r"F(\d+)", floor_id)
    if above:
        return int(above.group(1))
    below = re.fullmatch(r"B(\d+)", floor_id)
    if below:
        return -int(below.group(1))
    return None


def project_floor_ids(projects: Iterable[Dict[str, Any]]) -> List[str]:
    result: List[str] = []
    for project in projects:
        if not isinstance(project, dict):
            continue
        for value in _floor_list(project.get("floors")):
            floor_id = normalize_floor_id(value)
            if floor_id and floor_id not in result:
                result.append(floor_id)
    return result


def augment_floor_config(config: Dict[str, Any], projects: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    """Add explicitly registered project floors without inventing routes or map paths."""
    updated = copy.deepcopy(config) if isinstance(config, dict) else {}
    floors = updated.get("floors") if isinstance(updated.get("floors"), dict) else {}
    updated["floors"] = floors
    for floor_id in project_floor_ids(projects):
        if floor_id in floors:
            continue
        floors[floor_id] = {
            "level": floor_level_from_id(floor_id),
            "map_yaml": "",
            "terrain_segments": {},
            "stairs": {},
            "registry_source": "project",
        }
    return updated


def resolve_operational_floor(
    reported_floor: Any,
    selected_map: Dict[str, Any],
    route_config_floors: Iterable[Any],
) -> Any:
    """Use a selected custom single-floor map without pretending it has cross-floor routes."""
    reported = str(reported_floor or "").strip() or None
    map_floor = str((selected_map or {}).get("floor") or "").strip()
    route_floors = {str(item or "").strip() for item in route_config_floors if str(item or "").strip()}
    if map_floor and map_floor not in route_floors:
        return map_floor
    return reported


def _floor_list(value: Any) -> List[str]:
    if isinstance(value, str):
        value = value.split(",")
    if not isinstance(value, Iterable) or isinstance(value, (bytes, dict)):
        return []
    return [str(item).strip() for item in value if str(item).strip()]


def validate_registered_floor(
    floor: Any,
    config: Dict[str, Any],
    *,
    subject: str = "楼层",
) -> Dict[str, Any]:
    floor_id = str(floor or "").strip()
    registered = configured_floor_ids(config)
    if not registered:
        return {
            "ok": False,
            "code": "floor_registry_unavailable",
            "message": "楼层注册表不可用，不能安全创建或切换地图数据",
            "registered_floors": [],
        }
    if not floor_id:
        return {
            "ok": False,
            "code": "floor_identity_missing",
            "message": f"{subject}不能为空",
            "registered_floors": registered,
        }
    if floor_id not in registered:
        return {
            "ok": False,
            "code": "floor_identity_unknown",
            "message": f"{subject} {floor_id} 未在当前项目楼层注册表中配置",
            "floor": floor_id,
            "registered_floors": registered,
        }
    return {"ok": True, "floor": floor_id, "registered_floors": registered}


def validate_mapping_session_identity(
    payload: Dict[str, Any],
    config: Dict[str, Any],
    *,
    allow_floor_registration: bool = False,
) -> Dict[str, Any]:
    registered = configured_floor_ids(config)
    if not registered:
        return validate_registered_floor("", config, subject="建图楼层")
    raw_floors = _floor_list(payload.get("floors"))
    floors = [normalize_floor_id(item) for item in raw_floors]
    if not floors:
        return {
            "ok": False,
            "code": "mapping_floor_selection_missing",
            "message": "请填写本次建图所在的实际楼层",
            "registered_floors": registered,
        }
    invalid = [raw for raw, normalized in zip(raw_floors, floors) if not normalized]
    if invalid:
        return {
            "ok": False,
            "code": "mapping_floor_invalid",
            "message": "楼层格式无效：%s；请填写例如 7、F7 或 B1" % ", ".join(invalid),
            "invalid_floors": invalid,
        }
    if len(set(floors)) != len(floors):
        return {
            "ok": False,
            "code": "mapping_floor_duplicate",
            "message": "建图楼层不能重复",
            "floors": floors,
        }
    unknown = [floor for floor in floors if floor not in registered]
    if unknown and not allow_floor_registration:
        return {
            "ok": False,
            "code": "mapping_floor_unknown",
            "message": "建图任务包含未注册楼层：%s" % ", ".join(unknown),
            "unknown_floors": unknown,
            "registered_floors": registered,
        }
    mode = str(payload.get("mode") or "multi").strip()
    if mode not in ("single", "multi"):
        return {
            "ok": False,
            "code": "mapping_mode_invalid",
            "message": "建图模式必须是单楼层或多楼层",
            "mode": mode,
        }
    if mode == "single" and len(floors) != 1:
        return {
            "ok": False,
            "code": "single_mapping_floor_count",
            "message": "单楼层建图只能选择一个楼层",
            "floors": floors,
        }
    if mode == "multi" and len(floors) < 2:
        return {
            "ok": False,
            "code": "multi_mapping_floor_count",
            "message": "多楼层建图至少选择两个楼层",
            "floors": floors,
        }
    active_floor = normalize_floor_id(payload.get("active_floor") or floors[0])
    if active_floor not in floors:
        return {
            "ok": False,
            "code": "mapping_active_floor_not_selected",
            "message": "当前建图楼层必须属于已选择的建图楼层",
            "active_floor": active_floor,
            "floors": floors,
        }
    return {
        "ok": True,
        "floors": floors,
        "active_floor": active_floor,
        "mode": mode,
        "registered_floors": registered,
    }


def validate_floor_matches_map(
    floor: Any,
    map_record: Dict[str, Any],
    config: Dict[str, Any],
    *,
    subject: str,
) -> Dict[str, Any]:
    requested = validate_registered_floor(floor, config, subject=subject)
    if not requested.get("ok"):
        return requested
    map_floor = str(map_record.get("floor") or "").strip()
    registered_map = validate_registered_floor(map_floor, config, subject="地图楼层")
    if not registered_map.get("ok"):
        return registered_map
    if requested["floor"] != map_floor:
        return {
            "ok": False,
            "code": "floor_map_identity_mismatch",
            "message": "%s %s 与地图绑定楼层 %s 不一致" % (subject, requested["floor"], map_floor),
            "floor": requested["floor"],
            "map_floor": map_floor,
            "map_id": map_record.get("id"),
        }
    return {"ok": True, "floor": map_floor, "map_id": map_record.get("id")}
