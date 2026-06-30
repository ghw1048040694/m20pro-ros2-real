"""Pure map archive and occupancy-grid helpers for the web dashboard."""

from __future__ import annotations

import math
import os
import re
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import yaml


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
    default_floor = str(map_set.get("default_floor") or "").strip() or None
    global_pcd = str(map_set.get("global_pcd") or "").strip()
    floors = manifest.get("floors") or {}
    if not isinstance(floors, dict):
        return {"maps": [], "default_floor": default_floor, "default_map_id": None, "warnings": []}

    maps: List[Dict[str, Any]] = []
    warnings: List[str] = []
    for floor, info in floors.items():
        if not isinstance(info, dict):
            continue
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
                "id": f"builtin_{floor}",
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
        "default_map_id": default_builtin_map_id(maps, default_floor),
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
        if item.get("id") == "builtin_F20" or item.get("floor") == "F20":
            return str(item.get("id") or "") or None
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
    }
