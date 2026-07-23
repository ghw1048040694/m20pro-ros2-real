"""Pure map archive and occupancy-grid helpers for the web dashboard."""

from __future__ import annotations

import math
import os
import re
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import yaml

from .map_identity_contract import occupancy_grid_content_digest


def resolve_map_yaml_image_path(yaml_path: Path, image_value: str) -> Path:
    image_path = Path(os.path.expandvars(os.path.expanduser(str(image_value or "").strip())))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    return image_path


def find_local_map_image(yaml_path: Path, image_value: str) -> Optional[Path]:
    names: List[str] = []
    basename = Path(str(image_value or "")).name
    if basename:
        names.append(basename)
    for name in ("occ_grid.pgm", "map.pgm", "jueying.pgm", "occ_grid.png", "map.png", "jueying.png"):
        if name not in names:
            names.append(name)
    for name in names:
        candidate = yaml_path.parent / name
        if candidate.exists() and candidate.is_file():
            return candidate
    for pattern in ("*.pgm", "*.png", "*.jpg", "*.jpeg"):
        candidates = sorted(path for path in yaml_path.parent.glob(pattern) if path.is_file())
        if candidates:
            return candidates[0]
    return None


def same_path(left: Path, right: Path) -> bool:
    try:
        return left.resolve() == right.resolve()
    except Exception:
        return str(left) == str(right)


def write_map_yaml_image_reference(yaml_path: Path, image_reference: str) -> None:
    text = yaml_path.read_text(encoding="utf-8")
    replacement = f"image: {image_reference}"
    if re.search(r"(?m)^image\s*:", text):
        text = re.sub(r"(?m)^image\s*:.*$", replacement, text, count=1)
    else:
        text = replacement + "\n" + text
    yaml_path.write_text(text, encoding="utf-8")


def ensure_map_yaml_uses_local_image(yaml_path: Path) -> Dict[str, Any]:
    if not yaml_path.exists():
        return {"ok": False, "code": "map_yaml_missing", "message": f"地图 yaml 不存在: {yaml_path}"}
    try:
        with yaml_path.open("r", encoding="utf-8") as file:
            info = yaml.safe_load(file) or {}
    except Exception as exc:
        return {"ok": False, "code": "map_yaml_invalid", "message": str(exc), "yaml_path": str(yaml_path)}

    image_value = str(info.get("image") or "").strip()
    current_path = resolve_map_yaml_image_path(yaml_path, image_value) if image_value else None
    current_exists = bool(current_path is not None and current_path.exists())
    local_image = find_local_map_image(yaml_path, image_value)

    if local_image is not None:
        relative_image = os.path.relpath(local_image, yaml_path.parent)
        current_is_relative = bool(image_value and not Path(image_value).is_absolute())
        current_is_local = bool(current_path is not None and same_path(current_path, local_image))
        if (not image_value) or (not current_exists) or (not current_is_relative) or (not current_is_local):
            write_map_yaml_image_reference(yaml_path, relative_image)
            return {
                "ok": True,
                "repaired": True,
                "image": relative_image,
                "previous_image": image_value,
                "yaml_path": str(yaml_path),
                "message": "地图 yaml image 已修正为归档目录内相对路径",
            }
        return {
            "ok": True,
            "repaired": False,
            "image": image_value,
            "yaml_path": str(yaml_path),
            "message": "地图 yaml image 已指向归档目录内文件",
        }

    if current_exists:
        return {
            "ok": True,
            "repaired": False,
            "image": image_value,
            "yaml_path": str(yaml_path),
            "message": "地图 yaml image 指向的文件存在",
        }
    return {
        "ok": False,
        "code": "map_image_missing",
        "message": f"地图 yaml image 指向的文件不存在，且归档目录内没有可用栅格图: {image_value}",
        "yaml_path": str(yaml_path),
        "image": image_value,
    }


def find_map_yaml(directory: Path) -> Optional[Path]:
    for name in ("occ_grid.yaml", "map.yaml", "jueying.yaml"):
        candidate = directory / name
        if candidate.exists():
            return candidate
    for candidate in sorted(directory.rglob("*.yaml")):
        return candidate
    return None


def load_builtin_maps_from_manifest(
    manifest_path: Optional[Path],
    *,
    resolve_path: Callable[[str], str],
    derived_payload: Callable[[Path, str], Dict[str, Any]],
) -> Dict[str, Any]:
    if manifest_path is None or not manifest_path.exists():
        return {"maps": [], "default_floor": None, "default_map_id": None, "warnings": []}
    try:
        with manifest_path.open("r", encoding="utf-8") as file:
            manifest = yaml.safe_load(file) or {}
    except Exception as exc:
        return {
            "maps": [],
            "default_floor": None,
            "default_map_id": None,
            "warnings": [f"failed to read map manifest {manifest_path}: {exc}"],
        }

    map_set = manifest.get("map_set") or {}
    if not isinstance(map_set, dict):
        map_set = {}
    source_note = str(map_set.get("source_note") or "").strip()
    # `default_map_id` is an asset selection, not a floor-system default.
    explicit_default_map_id = str(map_set.get("default_map_id") or "").strip() or None
    default_floor = str(map_set.get("default_floor") or "").strip() or None
    global_pcd = str(map_set.get("global_pcd") or "").strip()
    raw_maps = manifest.get("maps")
    if isinstance(raw_maps, list):
        maps_by_id = {}
        for entry in raw_maps:
            if isinstance(entry, dict):
                map_id = str(entry.get("id") or entry.get("floor") or "").strip()
                if map_id:
                    maps_by_id[map_id] = entry
        map_entries = list(maps_by_id.items())
    else:
        floors = manifest.get("floors") or {}
        map_entries = list(floors.items()) if isinstance(floors, dict) else []
    if not map_entries:
        return {"maps": [], "default_floor": default_floor, "default_map_id": None, "warnings": []}

    maps: List[Dict[str, Any]] = []
    warnings: List[str] = []
    for key, info in map_entries:
        if not isinstance(info, dict):
            continue
        floor = str(info.get("floor") or key).strip()
        yaml_value = str(info.get("map_yaml") or "").strip()
        if not yaml_value:
            continue
        try:
            yaml_path = Path(resolve_path(yaml_value))
        except Exception as exc:
            warnings.append(f"failed to resolve builtin map {floor}: {exc}")
            continue
        pcd_value = str(info.get("pcd_map") or global_pcd or "").strip()
        pcd_path = ""
        if pcd_value:
            try:
                pcd_path = resolve_path(pcd_value)
            except Exception:
                pcd_path = pcd_value
        maps.append(
            {
                "id": str(info.get("id") or f"builtin_{floor}"),
                "name": str(info.get("label") or floor),
                "floor": str(floor),
                "level": info.get("level"),
                "directory": str(yaml_path.parent),
                "yaml_path": str(yaml_path),
                "source": "project_builtin",
                "readonly": True,
                "pcd_path": pcd_path,
                "derived": derived_payload(yaml_path, pcd_path),
                "source_note": source_note,
                "created_at": "项目内置地图",
            }
        )
    maps.sort(key=lambda item: (int(item.get("level") or 0), str(item.get("floor") or "")))
    return {
        "maps": maps,
        "default_floor": default_floor,
        "default_map_id": (
            explicit_default_map_id
            if explicit_default_map_id and any(item.get("id") == explicit_default_map_id for item in maps)
            else default_builtin_map_id(maps, default_floor)
        ),
        "warnings": warnings,
    }


def all_map_records(builtin_maps: List[Dict[str, Any]], archived_maps: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    archived_ids = {item.get("id") for item in archived_maps}
    return [
        dict(item)
        for item in builtin_maps
        if item.get("id") not in archived_ids
    ] + [dict(item) for item in archived_maps]


def find_map_record(
    builtin_maps: List[Dict[str, Any]],
    archived_maps: List[Dict[str, Any]],
    map_id: Optional[str],
) -> Optional[Dict[str, Any]]:
    if not map_id:
        return None
    target = str(map_id)
    for item in archived_maps:
        if item.get("id") == target:
            return item
    for item in builtin_maps:
        if item.get("id") == target:
            return item
    return None


def apply_map_delete_state(
    *,
    archived_maps: List[Dict[str, Any]],
    annotations: List[Dict[str, Any]],
    tasks: List[Dict[str, Any]],
    sessions: List[Dict[str, Any]],
    settings: Dict[str, Any],
    map_id: str,
    protected_map_ids: List[str],
    updated_at: str,
) -> Dict[str, Any]:
    target_id = str(map_id or "").strip()
    target = next((dict(item) for item in archived_maps if str(item.get("id") or "") == target_id), None)
    if target is None:
        return {"ok": False, "code": "map_not_deletable", "message": "地图不存在或属于项目内置地图"}
    protected = {str(item or "").strip() for item in protected_map_ids if str(item or "").strip()}
    if target_id in protected:
        return {"ok": False, "code": "map_in_use", "message": "当前生效或工作地图不能删除，请先切换到其他地图"}

    deleted_annotation_ids = {
        str(item.get("id") or "")
        for item in annotations
        if str(item.get("map_id") or "") == target_id
    }
    deleted_task_ids = {
        str(item.get("id") or "")
        for item in tasks
        if str(item.get("map_id") or "") == target_id
        or any(str(annotation_id) in deleted_annotation_ids for annotation_id in (item.get("annotation_ids") or []))
    }

    remaining_maps: List[Dict[str, Any]] = []
    for item in archived_maps:
        item_id = str(item.get("id") or "")
        if item_id == target_id:
            continue
        updated = dict(item)
        if str(updated.get("parent_map_id") or "") == target_id:
            updated.pop("parent_map_id", None)
            updated["deleted_parent_map_id"] = target_id
        remaining_maps.append(updated)

    remaining_annotations = [
        dict(item)
        for item in annotations
        if str(item.get("id") or "") not in deleted_annotation_ids
    ]
    remaining_tasks = [
        dict(item)
        for item in tasks
        if str(item.get("id") or "") not in deleted_task_ids
    ]

    updated_sessions: List[Dict[str, Any]] = []
    session_references = 0
    for session in sessions:
        updated_session = dict(session)
        steps = []
        active_floor_changed = False
        for raw_step in session.get("floor_steps") or []:
            if not isinstance(raw_step, dict):
                continue
            step = dict(raw_step)
            if str(step.get("map_id") or "") == target_id:
                step.pop("map_id", None)
                if str(step.get("status") or "") == "imported":
                    step["status"] = "saved"
                step["updated_at"] = str(updated_at or "")
                session_references += 1
                if str(step.get("floor") or "") == str(session.get("active_floor") or ""):
                    active_floor_changed = True
            steps.append(step)
        if isinstance(session.get("floor_steps"), list):
            updated_session["floor_steps"] = steps
        if active_floor_changed and str(updated_session.get("status") or "") == "imported":
            updated_session["status"] = "saved"
            updated_session["updated_at"] = str(updated_at or "")
        updated_sessions.append(updated_session)

    updated_settings = dict(settings)
    active_task = updated_settings.get("active_task")
    if isinstance(active_task, dict) and str(active_task.get("task_id") or "") in deleted_task_ids:
        updated_settings["active_task"] = None
    relocalization = updated_settings.get("map_relocalization_required")
    if isinstance(relocalization, dict) and target_id in {
        str(relocalization.get("map_id") or ""),
        str(relocalization.get("selected_map_id") or ""),
    }:
        updated_settings.pop("map_relocalization_required", None)

    return {
        "ok": True,
        "deleted_record": target,
        "maps": remaining_maps,
        "annotations": remaining_annotations,
        "tasks": remaining_tasks,
        "sessions": updated_sessions,
        "settings": updated_settings,
        "deleted_annotations": len(deleted_annotation_ids),
        "deleted_tasks": len(deleted_task_ids),
        "updated_sessions": session_references,
    }


def removable_map_archive_directory(
    archive_root: Path,
    record: Dict[str, Any],
    remaining_records: List[Dict[str, Any]],
) -> Dict[str, Any]:
    root = archive_root.expanduser().resolve()
    directory_value = str(record.get("directory") or "").strip()
    if directory_value:
        candidate = Path(os.path.expandvars(os.path.expanduser(directory_value))).resolve()
    else:
        yaml_value = str(record.get("yaml_path") or "").strip()
        candidate = Path(os.path.expandvars(os.path.expanduser(yaml_value))).resolve().parent if yaml_value else root
    if candidate == root or root not in candidate.parents:
        return {"delete": False, "path": str(candidate), "reason": "outside_map_archive"}
    for item in remaining_records:
        other_value = str(item.get("directory") or "").strip()
        if not other_value:
            yaml_value = str(item.get("yaml_path") or "").strip()
            other_value = str(Path(yaml_value).parent) if yaml_value else ""
        if not other_value:
            continue
        other = Path(os.path.expandvars(os.path.expanduser(other_value))).resolve()
        if other == candidate:
            return {"delete": False, "path": str(candidate), "reason": "shared_map_directory"}
    return {"delete": True, "path": str(candidate), "reason": "owned_map_archive"}


def default_builtin_map_id(builtin_maps: List[Dict[str, Any]], default_floor: Optional[str]) -> Optional[str]:
    floor = str(default_floor or "").strip()
    if not floor:
        return None
    for item in builtin_maps:
        if item.get("floor") == floor:
            return str(item.get("id") or "") or None
    return None


def default_map_id(
    builtin_maps: List[Dict[str, Any]],
    archived_maps: List[Dict[str, Any]],
    default_builtin_id: Optional[str],
) -> Optional[str]:
    if default_builtin_id and find_map_record(builtin_maps, archived_maps, default_builtin_id):
        return str(default_builtin_id)
    for item in builtin_maps:
        if item.get("id"):
            return str(item.get("id"))
    for item in archived_maps:
        if item.get("id"):
            return str(item.get("id"))
    return None


def build_imported_map_record(
    *,
    map_id: str,
    map_name: str,
    floor: str,
    mode: Any,
    project_id: Any,
    project_name: Any,
    building: Any,
    directory: Path,
    yaml_path: Path,
    source_path: str,
    created_at: str,
) -> Dict[str, Any]:
    return {
        "id": str(map_id or ""),
        "name": str(map_name or ""),
        "floor": str(floor or ""),
        "mode": mode,
        "project_id": project_id,
        "project_name": project_name,
        "building": building,
        "directory": str(directory),
        "yaml_path": str(yaml_path),
        "source": "106_active_map",
        "source_path": str(source_path or ""),
        "created_at": str(created_at or ""),
    }


def read_pgm(path: Path) -> Tuple[int, int, int, List[int]]:
    with path.open("rb") as file:

        def token() -> bytes:
            chars = bytearray()
            while True:
                value = file.read(1)
                if not value:
                    break
                if value == b"#":
                    file.readline()
                    continue
                if value.isspace():
                    if chars:
                        break
                    continue
                chars.extend(value)
            return bytes(chars)

        magic = token()
        if magic not in (b"P5", b"P2"):
            raise RuntimeError(f"unsupported PGM format: {magic!r}")
        width = int(token())
        height = int(token())
        max_value = int(token())
        if magic == b"P5":
            if max_value <= 255:
                raw = file.read(width * height)
                pixels = list(raw)
            else:
                raw = file.read(width * height * 2)
                pixels = [
                    int.from_bytes(raw[index:index + 2], "big")
                    for index in range(0, len(raw), 2)
                ]
        else:
            pixels = [int(token()) for _ in range(width * height)]
    if len(pixels) != width * height:
        raise RuntimeError(f"PGM pixel count mismatch: {path}")
    return width, height, max_value, pixels


def write_pgm(path: Path, width: int, height: int, max_value: int, pixels: List[int]) -> None:
    """Write a normalized binary PGM while preserving the map dimensions."""
    if len(pixels) != int(width) * int(height):
        raise ValueError("PGM pixel count does not match map dimensions")
    if not 1 <= int(max_value) <= 65535:
        raise ValueError("unsupported PGM max value")
    header = (f"P5\n{int(width)} {int(height)}\n{int(max_value)}\n").encode("ascii")
    bounded = [int(max(0, min(int(max_value), value))) for value in pixels]
    raw = (
        bytes(bounded)
        if int(max_value) <= 255
        else b"".join(value.to_bytes(2, "big") for value in bounded)
    )
    path.write_bytes(header + raw)


def apply_map_cell_edits(yaml_path: Path, cells: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Apply sparse ROS occupancy edits to the PGM referenced by a map YAML.

    Frontend editor coordinates are image coordinates (origin at the top-left),
    which match PGM row order.  The loaded ROS occupancy data remains flipped
    by ``load_map_file_payload`` and is not written directly by the browser.
    """
    if not yaml_path.exists():
        raise FileNotFoundError(str(yaml_path))
    with yaml_path.open("r", encoding="utf-8") as file:
        info = yaml.safe_load(file) or {}
    image_value = str(info.get("image") or "").strip()
    image_path = resolve_map_yaml_image_path(yaml_path, image_value) if image_value else None
    if image_path is None or not image_path.exists():
        image_path = find_local_map_image(yaml_path, image_value)
    if image_path is None or not image_path.exists():
        raise FileNotFoundError("map image not found")
    width, height, max_value, pixels = read_pgm(image_path)
    if len(cells) > 100000:
        raise ValueError("地图修饰单次最多保存 100000 个栅格")
    negate = int(info.get("negate", 0) or 0)
    occupied_thresh = float(info.get("occupied_thresh", 0.65) or 0.65)
    free_thresh = float(info.get("free_thresh", 0.196) or 0.196)
    unknown_ratio = max(0.0, min(1.0, (occupied_thresh + free_thresh) / 2.0))
    changed = 0
    seen = set()
    for item in cells:
        if not isinstance(item, dict):
            raise ValueError("地图修饰栅格格式无效")
        try:
            x = int(item["x"])
            y = int(item["y"])
            occupancy = int(item["value"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("地图修饰栅格必须包含整数 x/y/value") from exc
        if not (0 <= x < width and 0 <= y < height):
            raise ValueError("地图修饰栅格超出地图范围")
        if occupancy not in (-1, 0, 100):
            raise ValueError("地图修饰值只能是 -1、0 或 100")
        key = (x, y)
        if key in seen:
            continue
        seen.add(key)
        if occupancy == 100:
            ratio = 1.0 if negate else 0.0
        elif occupancy == 0:
            ratio = 0.0 if negate else 1.0
        else:
            ratio = unknown_ratio
        pixel = int(round(ratio * max_value))
        index = y * width + x
        if pixels[index] != pixel:
            pixels[index] = pixel
            changed += 1
    write_pgm(image_path, width, height, max_value, pixels)
    return {
        "ok": True,
        "width": width,
        "height": height,
        "changed_cells": changed,
        "image_path": str(image_path),
    }


def read_pgm_header(path: Path) -> Tuple[int, int, int]:
    with path.open("rb") as file:

        def token() -> bytes:
            chars = bytearray()
            while True:
                value = file.read(1)
                if not value:
                    break
                if value == b"#":
                    file.readline()
                    continue
                if value.isspace():
                    if chars:
                        break
                    continue
                chars.extend(value)
            return bytes(chars)

        magic = token()
        if magic not in (b"P5", b"P2"):
            raise RuntimeError(f"unsupported PGM format: {magic!r}")
        width = int(token())
        height = int(token())
        max_value = int(token())
    return width, height, max_value


def map_file_metadata_payload(record: Dict[str, Any], yaml_path: Path) -> Dict[str, Any]:
    if not yaml_path.exists():
        raise FileNotFoundError(str(yaml_path))
    image_repair = ensure_map_yaml_uses_local_image(yaml_path)
    if not image_repair.get("ok"):
        raise RuntimeError(str(image_repair["message"]))
    with yaml_path.open("r", encoding="utf-8") as file:
        info = yaml.safe_load(file) or {}
    image_value = str(info.get("image") or "").strip()
    if not image_value:
        raise RuntimeError("map yaml has no image field")
    image_path = resolve_map_yaml_image_path(yaml_path, image_value)
    if not image_path.exists():
        fallback = find_local_map_image(yaml_path, image_value)
        if fallback is not None and fallback.exists():
            image_path = fallback
    width, height, _max_value = read_pgm_header(image_path)
    resolution = float(info.get("resolution", 0.05))
    origin_raw = info.get("origin") or [0.0, 0.0, 0.0]
    yaw = float(origin_raw[2]) if len(origin_raw) > 2 else 0.0
    origin = {
        "x": float(origin_raw[0]) if len(origin_raw) > 0 else 0.0,
        "y": float(origin_raw[1]) if len(origin_raw) > 1 else 0.0,
        "z": 0.0,
        "yaw": yaw,
        "yaw_deg": math.degrees(yaw),
    }
    version = int(max(yaml_path.stat().st_mtime, image_path.stat().st_mtime) * 1000)
    return {
        "available": True,
        "source": "file",
        "map_source": record.get("source"),
        "map_id": record.get("id"),
        "name": record.get("name"),
        "floor": record.get("floor"),
        "version": version,
        "frame_id": "map",
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": origin,
    }


def map_file_fingerprint(yaml_path: Path) -> Optional[Tuple[float, int, str, Optional[float], Optional[int]]]:
    try:
        yaml_stat = yaml_path.stat()
    except OSError:
        return None
    image_mtime = None
    image_size = None
    try:
        with yaml_path.open("r", encoding="utf-8") as file:
            info = yaml.safe_load(file) or {}
        image_value = str(info.get("image") or "").strip()
        if image_value:
            image_path = resolve_map_yaml_image_path(yaml_path, image_value)
            if not image_path.exists():
                fallback = find_local_map_image(yaml_path, image_value)
                if fallback is not None and fallback.exists():
                    image_path = fallback
            if image_path.exists():
                image_stat = image_path.stat()
                image_mtime = float(image_stat.st_mtime)
                image_size = int(image_stat.st_size)
    except Exception:
        image_mtime = None
        image_size = None
    return (
        float(yaml_stat.st_mtime),
        int(yaml_stat.st_size),
        str(yaml_path),
        image_mtime,
        image_size,
    )


def load_map_file_payload(record: Dict[str, Any], yaml_path: Path) -> Dict[str, Any]:
    if not yaml_path.exists():
        raise FileNotFoundError(str(yaml_path))
    image_repair = ensure_map_yaml_uses_local_image(yaml_path)
    if not image_repair.get("ok"):
        raise RuntimeError(str(image_repair["message"]))
    with yaml_path.open("r", encoding="utf-8") as file:
        info = yaml.safe_load(file) or {}
    image_value = str(info.get("image") or "").strip()
    if not image_value:
        raise RuntimeError("map yaml has no image field")
    image_path = resolve_map_yaml_image_path(yaml_path, image_value)
    if not image_path.exists():
        fallback = find_local_map_image(yaml_path, image_value)
        if fallback is not None and fallback.exists():
            image_path = fallback
    width, height, max_value, pixels = read_pgm(image_path)
    resolution = float(info.get("resolution", 0.05))
    origin_raw = info.get("origin") or [0.0, 0.0, 0.0]
    yaw = float(origin_raw[2]) if len(origin_raw) > 2 else 0.0
    origin = {
        "x": float(origin_raw[0]) if len(origin_raw) > 0 else 0.0,
        "y": float(origin_raw[1]) if len(origin_raw) > 1 else 0.0,
        "z": 0.0,
        "yaw": yaw,
        "yaw_deg": math.degrees(yaw),
    }
    negate = int(info.get("negate", 0))
    occupied_thresh = float(info.get("occupied_thresh", 0.65))
    free_thresh = float(info.get("free_thresh", 0.196))
    data = [-1] * (width * height)
    max_value = max(1, max_value)
    for y in range(height):
        for x in range(width):
            pixel = pixels[y * width + x] / max_value
            occupied = pixel if negate else 1.0 - pixel
            if occupied > occupied_thresh:
                value = 100
            elif occupied < free_thresh:
                value = 0
            else:
                value = -1
            data[(height - 1 - y) * width + x] = value
    version = int(max(yaml_path.stat().st_mtime, image_path.stat().st_mtime) * 1000)
    return {
        "available": True,
        "source": "file",
        "map_source": record.get("source"),
        "map_id": record.get("id"),
        "name": record.get("name"),
        "floor": record.get("floor"),
        "version": version,
        "frame_id": "map",
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": origin,
        "data": data,
        "content_digest": occupancy_grid_content_digest(
            {
                "available": True,
                "width": width,
                "height": height,
                "resolution": resolution,
                "origin": origin,
                "data": data,
            }
        ),
    }
