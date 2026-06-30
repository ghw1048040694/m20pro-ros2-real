"""Pure map-derived asset helpers for the web dashboard."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Callable, Dict, Optional


PathResolver = Callable[[str], str]


def builtin_map_derived_payload(yaml_path: Path, pcd_path: str) -> Dict[str, Any]:
    derived_dir = Path(yaml_path).parent / "derived"
    zones_path = derived_dir / "stair_zones.json"
    if zones_path.exists():
        return {
            "status": "ready",
            "message": "项目内置地图已有楼梯语义区",
            "stair_zones": str(zones_path.relative_to(Path(yaml_path).parent)),
            "pcd_path": str(pcd_path or ""),
        }
    return {
        "status": "pending",
        "message": "项目内置地图可生成楼梯语义区；为避免启动时改动仓库文件，请在导入归档地图时自动生成",
        "pcd_path": str(pcd_path or ""),
    }


def map_derived_payload(record: Dict[str, Any]) -> Dict[str, Any]:
    derived = record.get("derived") if isinstance(record.get("derived"), dict) else {}
    return dict(derived)


def stair_zones_relative_path(record: Dict[str, Any]) -> str:
    return str(map_derived_payload(record).get("stair_zones") or "").strip()


def should_generate_builtin_stair_zones(
    record: Dict[str, Any],
    *,
    enable_stair_zone_postprocess: bool,
) -> bool:
    derived = map_derived_payload(record)
    return bool(
        record.get("source") == "project_builtin"
        and derived.get("status") == "pending"
        and enable_stair_zone_postprocess
    )


def stair_zones_unavailable_payload(record: Optional[Dict[str, Any]], message: str) -> Dict[str, Any]:
    payload: Dict[str, Any] = {
        "ok": True,
        "available": False,
        "message": str(message or ""),
        "zones": [],
    }
    if record is not None:
        payload["map_id"] = record.get("id")
        payload["floor"] = record.get("floor")
    return payload


def resolve_map_asset_path(
    record: Dict[str, Any],
    relative_path: str,
    *,
    path_resolver: Optional[PathResolver] = None,
) -> Optional[Path]:
    value = str(relative_path or "").strip()
    if not value:
        return None
    path = Path(os.path.expandvars(os.path.expanduser(value)))
    if path.is_absolute():
        return path

    resolver = path_resolver or (lambda item: item)
    derived = map_derived_payload(record)
    base_dir = str(derived.get("base_dir") or "").strip()
    if base_dir:
        return Path(resolver(base_dir)) / path

    directory = str(record.get("directory") or "").strip()
    if not directory:
        return None
    return Path(resolver(directory)) / path


def read_json_object(path: Path) -> Dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as file:
        payload = json.load(file)
    if not isinstance(payload, dict):
        raise RuntimeError("JSON payload is not an object")
    return payload


def stair_zones_available_payload(
    record: Dict[str, Any],
    derived: Dict[str, Any],
    zones_payload: Dict[str, Any],
) -> Dict[str, Any]:
    payload = dict(zones_payload)
    payload["ok"] = True
    payload["available"] = True
    payload["map"] = {
        "id": record.get("id"),
        "name": record.get("name"),
        "floor": record.get("floor"),
        "derived_status": derived.get("status"),
    }
    return payload
