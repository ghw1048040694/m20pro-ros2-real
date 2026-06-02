import os
from pathlib import Path
from typing import Any, Dict

import yaml
from ament_index_python.packages import get_package_share_directory


def resolve_path(value: str, base_dir: str = "") -> str:
    path = os.path.expandvars(os.path.expanduser(str(value).strip()))
    if path.startswith("package://"):
        package_and_path = path[len("package://") :]
        package_name, _, relative_path = package_and_path.partition("/")
        if not package_name or not relative_path:
            raise RuntimeError("invalid package path: %s" % value)
        return os.path.join(get_package_share_directory(package_name), relative_path)
    if os.path.isabs(path):
        return path
    if base_dir:
        return str((Path(base_dir) / path).resolve())
    return str(Path(path).resolve())


def load_yaml(path: str) -> Dict[str, Any]:
    resolved = resolve_path(path)
    with open(resolved, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}
    if not isinstance(data, dict):
        raise RuntimeError("%s must be a YAML mapping" % resolved)
    return data


def load_manifest(path: str) -> Dict[str, Any]:
    manifest = load_yaml(path)
    floors = manifest.get("floors") or {}
    if not isinstance(floors, dict) or not floors:
        raise RuntimeError("map manifest has no floors")
    return manifest


def floor_z_ranges(manifest: Dict[str, Any]) -> list:
    ranges = []
    for floor_id, floor in sorted((manifest.get("floors") or {}).items()):
        if not isinstance(floor, dict):
            continue
        ranges.append(
            "%s:%s:%s:%s"
            % (
                floor_id,
                float(floor.get("z_min", -1.0)),
                float(floor.get("z_max", 1.5)),
                float(floor.get("z_offset", 0.0)),
            )
        )
    return ranges


def default_floor(manifest: Dict[str, Any], fallback: str = "") -> str:
    configured = str((manifest.get("map_set") or {}).get("default_floor") or "").strip()
    if configured:
        return configured
    floors = sorted((manifest.get("floors") or {}).keys())
    return fallback or (floors[0] if floors else "")
