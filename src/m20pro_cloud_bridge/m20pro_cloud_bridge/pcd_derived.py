import json
import math
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


DEFAULT_DERIVED_DIR = "derived"
STAIR_ZONES_FILE = "stair_zones.json"


def process_imported_map(
    map_dir: Path,
    yaml_path: Path,
    floor: str,
    map_id: str,
    floor_config_path: Optional[Path] = None,
    pcd_path_override: Optional[Path] = None,
    cell_size: float = 0.25,
    stair_point_max: int = 20000,
) -> Dict[str, Any]:
    """Generate the optional stair-zone metadata exposed by the Web API.

    The frontend 3D map and stair pointcloud viewers were removed from the
    single-floor workflow, so this postprocess now avoids loading factory PCDs
    and only materializes configured stair semantics.
    """

    _ = (yaml_path, cell_size, stair_point_max)
    map_dir = Path(map_dir)
    derived_dir = map_dir / DEFAULT_DERIVED_DIR
    derived_dir.mkdir(parents=True, exist_ok=True)

    source_pcd = Path(pcd_path_override) if pcd_path_override else None
    if source_pcd is not None and not source_pcd.exists():
        source_pcd = None

    zones = _configured_stair_zones(floor_config_path, floor)
    stair_zones_path = derived_dir / STAIR_ZONES_FILE
    _write_json(stair_zones_path, _stair_zones_payload(map_id, floor, zones, source_pcd))
    return {
        "status": "ready",
        "message": "楼梯语义区已生成：%d 个区域；未生成 3D 地形或局部点云" % len(zones),
        "stair_zones": _relative_to(stair_zones_path, map_dir),
        "generated_at": _now_text(),
        "zone_count": len(zones),
        "source_pcd": str(source_pcd) if source_pcd else None,
    }


def _configured_stair_zones(floor_config_path: Optional[Path], floor: str) -> List[Dict[str, Any]]:
    if not floor_config_path:
        return []
    path = Path(floor_config_path)
    if not path.exists():
        return []
    try:
        with path.open("r", encoding="utf-8") as file:
            config = yaml.safe_load(file) or {}
    except Exception:
        return []
    floors = config.get("floors") or {}
    floor_data = floors.get(floor) or {}
    stairs = floor_data.get("stairs") or {}
    zones: List[Dict[str, Any]] = []
    for stair_name, stair in stairs.items():
        if not isinstance(stair, dict):
            continue
        target_floor = str(stair.get("target_floor") or stair.get("exit_floor") or "").strip()
        if not target_floor:
            continue
        transition = stair.get("transition") if isinstance(stair.get("transition"), dict) else {}
        entry = _pose_dict(
            transition.get("entry")
            or transition.get("approach")
            or stair.get("entry")
            or stair.get("approach")
        )
        source_platform = _pose_dict(
            transition.get("source_platform")
            or transition.get("platform_entry")
            or transition.get("platform_switch")
            or transition.get("traverse_to")
            or stair.get("source_platform")
            or stair.get("platform_entry")
            or stair.get("platform_switch")
            or stair.get("traverse_to")
        )
        target_platform = _pose_dict(
            transition.get("target_platform")
            or transition.get("target_exit")
            or transition.get("exit")
            or stair.get("target_platform")
            or stair.get("target_exit")
            or stair.get("exit")
        )
        post_exit = _pose_dict(
            transition.get("post_exit")
            or transition.get("flat_transition")
            or transition.get("flat_entry")
            or stair.get("post_exit")
            or stair.get("flat_transition")
            or stair.get("flat_entry")
        )
        points = [pose for pose in (entry, source_platform, target_platform, post_exit) if pose]
        if not points:
            continue
        margin = _safe_float(transition.get("entry_margin_m"), 0.8)
        xs = [float(pose["x"]) for pose in points]
        ys = [float(pose["y"]) for pose in points]
        min_x, max_x = min(xs) - margin, max(xs) + margin
        min_y, max_y = min(ys) - margin, max(ys) + margin
        center = {"x": round((min_x + max_x) * 0.5, 4), "y": round((min_y + max_y) * 0.5, 4)}
        zone = {
            "id": "%s_%s" % (floor, _slug(str(stair_name))),
            "name": "%s %s" % (floor, str(stair_name)),
            "route_name": str(stair_name),
            "floor": floor,
            "source_floor": floor,
            "target_floor": target_floor,
            "direction": str(stair.get("direction") or "").strip() or _infer_direction(floor, target_floor),
            "source": "configured_route",
            "trigger_gait": True,
            "confidence": 1.0,
            "entry": entry,
            "source_platform": source_platform,
            "target_platform": target_platform,
            "post_exit": post_exit,
            "center": center,
            "polygon": [
                {"x": round(min_x, 4), "y": round(min_y, 4)},
                {"x": round(max_x, 4), "y": round(min_y, 4)},
                {"x": round(max_x, 4), "y": round(max_y, 4)},
                {"x": round(min_x, 4), "y": round(max_y, 4)},
            ],
        }
        zones.append(zone)
    return zones


def _stair_zones_payload(
    map_id: str,
    floor: str,
    zones: List[Dict[str, Any]],
    pcd_path: Optional[Path],
) -> Dict[str, Any]:
    return {
        "ok": True,
        "available": True,
        "type": "stair_zones",
        "version": 1,
        "map_id": map_id,
        "floor": floor,
        "generated_at": _now_text(),
        "source_pcd": str(pcd_path) if pcd_path else None,
        "zones": zones,
    }


def _pose_dict(value: Any) -> Dict[str, float]:
    if not isinstance(value, dict):
        return {}
    try:
        return {
            "x": round(float(value.get("x", 0.0)), 4),
            "y": round(float(value.get("y", 0.0)), 4),
            "z": round(float(value.get("z", 0.0)), 4),
            "yaw": round(float(value.get("yaw", 0.0)), 6),
        }
    except (TypeError, ValueError):
        return {}


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    tmp = Path(path).with_suffix(Path(path).suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=False, separators=(",", ":"))
    os.replace(tmp, path)


def _relative_to(path: Path, root: Path) -> str:
    try:
        return str(Path(path).relative_to(Path(root)))
    except ValueError:
        return str(path)


def _safe_float(value: Any, default: float) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _infer_direction(source_floor: str, target_floor: str) -> str:
    source_level = _floor_level(source_floor)
    target_level = _floor_level(target_floor)
    if source_level is not None and target_level is not None:
        return "up" if target_level > source_level else "down"
    return "up"


def _floor_level(floor: str) -> Optional[float]:
    text = str(floor or "").strip().upper()
    if text.startswith("B") and text[1:].isdigit():
        return -float(text[1:])
    if text.startswith("F") and text[1:].isdigit():
        return float(text[1:])
    try:
        return float(text)
    except ValueError:
        return None


def _slug(value: str) -> str:
    text = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in value.strip())
    return text.strip("._") or "item"


def _now_text() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
