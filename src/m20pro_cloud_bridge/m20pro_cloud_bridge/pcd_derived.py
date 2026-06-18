import io
import json
import math
import os
import struct
import time
from collections import deque
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np
import yaml


DEFAULT_DERIVED_DIR = "derived"
TERRAIN_MESH_FILE = "terrain_mesh.json"
HEIGHT_GRID_FILE = "height_grid.json"
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
    """Create lightweight web assets from the imported factory PCD map.

    The original PCD stays untouched. Derived files are safe to regenerate and
    small enough for the web dashboard to load on a handheld browser.
    """

    map_dir = Path(map_dir)
    yaml_path = Path(yaml_path)
    derived_dir = map_dir / DEFAULT_DERIVED_DIR
    derived_dir.mkdir(parents=True, exist_ok=True)

    pcd_path = Path(pcd_path_override) if pcd_path_override else find_pcd_file(map_dir)
    if pcd_path is not None and not pcd_path.exists():
        pcd_path = None
    if pcd_path is None:
        zones = _configured_stair_zones(floor_config_path, floor)
        stair_zones_path = derived_dir / STAIR_ZONES_FILE
        _write_json(stair_zones_path, _stair_zones_payload(map_id, floor, zones, None))
        return {
            "status": "missing_pcd",
            "message": "未找到 full_cloud.pcd/jueying.pcd；已仅生成楼梯语义区，2D 地图仍可用",
            "stair_zones": _relative_to(stair_zones_path, map_dir),
        }

    started = time.monotonic()
    points = load_pcd_xyz(pcd_path)
    if points.size == 0:
        return {
            "status": "failed",
            "message": "PCD 中没有可用 XYZ 点",
            "pcd_path": str(pcd_path),
        }
    points = points[np.isfinite(points).all(axis=1)].astype(np.float32, copy=False)
    original_points = int(len(points))
    map_info = _load_map_yaml(yaml_path)
    filtered_points = _filter_points_to_map(points, map_info, margin_m=1.0)
    if len(filtered_points) >= 100:
        points = filtered_points

    points = _clip_z_outliers(points)
    height_grid = _build_height_grid(points, map_info, cell_size)
    terrain_mesh = _build_terrain_mesh(height_grid, map_id, floor, pcd_path, original_points)

    config_zones = _configured_stair_zones(floor_config_path, floor)
    detected_zones = _height_grid_stair_candidates(height_grid, floor)
    zones = config_zones + detected_zones

    terrain_path = derived_dir / TERRAIN_MESH_FILE
    height_path = derived_dir / HEIGHT_GRID_FILE
    stair_zones_path = derived_dir / STAIR_ZONES_FILE
    _write_json(terrain_path, terrain_mesh)
    _write_json(height_path, height_grid)
    _write_stair_pointclouds(derived_dir, zones, points, stair_point_max)
    _write_json(stair_zones_path, _stair_zones_payload(map_id, floor, zones, pcd_path))

    elapsed = time.monotonic() - started
    return {
        "status": "ready",
        "message": "PCD 派生 3D 地形完成：%d 点 -> %dx%d 高度网格，用时 %.1fs"
        % (original_points, int(height_grid["cols"]), int(height_grid["rows"]), elapsed),
        "pcd_path": str(pcd_path),
        "terrain_mesh": _relative_to(terrain_path, map_dir),
        "height_grid": _relative_to(height_path, map_dir),
        "stair_zones": _relative_to(stair_zones_path, map_dir),
        "generated_at": _now_text(),
        "source_points": original_points,
        "used_points": int(len(points)),
    }


def find_pcd_file(map_dir: Path) -> Optional[Path]:
    preferred = [
        "full_cloud.pcd",
        "jueying.pcd",
        "map.pcd",
        "cloud.pcd",
        "full_cloud_cleaned.pcd",
    ]
    for name in preferred:
        candidate = map_dir / name
        if candidate.exists():
            return candidate
    candidates = [path for path in map_dir.rglob("*.pcd") if path.is_file()]
    if not candidates:
        return None
    candidates.sort(key=lambda item: item.stat().st_size, reverse=True)
    return candidates[0]


def load_pcd_xyz(path: Path) -> np.ndarray:
    with Path(path).open("rb") as file:
        header_lines: List[str] = []
        while True:
            line = file.readline()
            if not line:
                raise RuntimeError("PCD header is missing DATA line: %s" % path)
            decoded = line.decode("utf-8", errors="ignore").strip()
            header_lines.append(decoded)
            if decoded.startswith("DATA"):
                break
        payload = file.read()

    header = _parse_pcd_header(header_lines)
    fields = header.get("fields", [])
    sizes = header.get("sizes", [])
    types = header.get("types", [])
    counts = header.get("counts", [1] * len(fields))
    points = int(header.get("points", 0))
    data_mode = str(header.get("data", "")).lower()
    if points <= 0:
        return np.empty((0, 3), dtype=np.float32)

    offsets: Dict[str, int] = {}
    offset = 0
    for field, size, count in zip(fields, sizes, counts):
        offsets[str(field)] = offset
        offset += int(size) * int(count)
    point_step = offset
    for field in ("x", "y", "z"):
        if field not in offsets:
            raise RuntimeError("PCD is missing %s field: %s" % (field, path))

    if data_mode == "binary":
        return _load_binary_pcd_xyz(payload, points, point_step, offsets, fields, sizes, types)
    if data_mode == "ascii":
        array = np.loadtxt(io.BytesIO(payload), dtype=np.float32)
        if array.ndim == 1:
            array = array.reshape((1, -1))
        columns = [fields.index(field) for field in ("x", "y", "z")]
        return array[:, columns].astype(np.float32, copy=False)
    raise RuntimeError("unsupported PCD DATA mode %s in %s" % (data_mode, path))


def _parse_pcd_header(lines: List[str]) -> Dict[str, Any]:
    header: Dict[str, Any] = {}
    for line in lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0].upper()
        values = parts[1:]
        if key == "FIELDS":
            header["fields"] = values
        elif key == "SIZE":
            header["sizes"] = [int(value) for value in values]
        elif key == "TYPE":
            header["types"] = values
        elif key == "COUNT":
            header["counts"] = [int(value) for value in values]
        elif key == "POINTS":
            header["points"] = int(values[0])
        elif key == "DATA":
            header["data"] = values[0]
    if "counts" not in header and "fields" in header:
        header["counts"] = [1] * len(header["fields"])
    return header


def _load_binary_pcd_xyz(
    payload: bytes,
    points: int,
    point_step: int,
    offsets: Dict[str, int],
    fields: List[str],
    sizes: List[int],
    types: List[str],
) -> np.ndarray:
    field_info = {field: (sizes[idx], types[idx]) for idx, field in enumerate(fields)}
    aligned_float32 = (
        point_step % 4 == 0
        and all(offsets[field] % 4 == 0 for field in ("x", "y", "z"))
        and all(field_info[field] == (4, "F") for field in ("x", "y", "z"))
    )
    payload = payload[: points * point_step]
    if aligned_float32:
        values = np.frombuffer(payload, dtype="<f4")
        columns_per_point = point_step // 4
        rows = values.reshape((-1, columns_per_point))
        columns = [offsets[field] // 4 for field in ("x", "y", "z")]
        return rows[:, columns].astype(np.float32, copy=True)

    xyz = np.empty((points, 3), dtype=np.float32)
    for row in range(points):
        base = row * point_step
        xyz[row, 0] = struct.unpack_from("<f", payload, base + offsets["x"])[0]
        xyz[row, 1] = struct.unpack_from("<f", payload, base + offsets["y"])[0]
        xyz[row, 2] = struct.unpack_from("<f", payload, base + offsets["z"])[0]
    return xyz


def _load_map_yaml(yaml_path: Path) -> Dict[str, Any]:
    with Path(yaml_path).open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file) or {}
    origin_raw = data.get("origin") or [0.0, 0.0, 0.0]
    image_path = Path(str(data.get("image") or ""))
    if not image_path.is_absolute():
        image_path = Path(yaml_path).parent / image_path
    width, height = _read_pgm_size(image_path)
    return {
        "resolution": float(data.get("resolution", 0.05)),
        "origin": {
            "x": float(origin_raw[0]) if len(origin_raw) > 0 else 0.0,
            "y": float(origin_raw[1]) if len(origin_raw) > 1 else 0.0,
            "yaw": float(origin_raw[2]) if len(origin_raw) > 2 else 0.0,
        },
        "width": width,
        "height": height,
    }


def _read_pgm_size(path: Path) -> Tuple[int, int]:
    if not path.exists():
        return 0, 0
    with path.open("rb") as file:
        def token() -> bytes:
            chars = bytearray()
            while True:
                b = file.read(1)
                if not b:
                    break
                if b == b"#":
                    file.readline()
                    continue
                if b.isspace():
                    if chars:
                        break
                    continue
                chars.extend(b)
            return bytes(chars)

        magic = token()
        if magic not in (b"P5", b"P2"):
            return 0, 0
        width = int(token())
        height = int(token())
        return width, height


def _filter_points_to_map(points: np.ndarray, map_info: Dict[str, Any], margin_m: float) -> np.ndarray:
    width = int(map_info.get("width") or 0)
    height = int(map_info.get("height") or 0)
    resolution = float(map_info.get("resolution") or 0.0)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return points
    origin = map_info.get("origin") or {}
    x0 = float(origin.get("x", 0.0)) - margin_m
    y0 = float(origin.get("y", 0.0)) - margin_m
    x1 = x0 + width * resolution + 2.0 * margin_m
    y1 = y0 + height * resolution + 2.0 * margin_m
    mask = (
        (points[:, 0] >= x0)
        & (points[:, 0] <= x1)
        & (points[:, 1] >= y0)
        & (points[:, 1] <= y1)
    )
    return points[mask]


def _clip_z_outliers(points: np.ndarray) -> np.ndarray:
    if len(points) < 100:
        return points
    low, high = np.quantile(points[:, 2], [0.01, 0.99])
    if not math.isfinite(float(low)) or not math.isfinite(float(high)) or low >= high:
        return points
    return points[(points[:, 2] >= low) & (points[:, 2] <= high)]


def _build_height_grid(points: np.ndarray, map_info: Dict[str, Any], cell_size: float) -> Dict[str, Any]:
    cell_size = max(0.05, float(cell_size))
    if points.size == 0:
        raise RuntimeError("cannot build height grid from empty point set")

    origin = map_info.get("origin") or {}
    map_width = int(map_info.get("width") or 0)
    map_height = int(map_info.get("height") or 0)
    resolution = float(map_info.get("resolution") or 0.0)
    if map_width > 0 and map_height > 0 and resolution > 0.0:
        min_x = float(origin.get("x", 0.0))
        min_y = float(origin.get("y", 0.0))
        max_x = min_x + map_width * resolution
        max_y = min_y + map_height * resolution
    else:
        min_x, min_y = points[:, :2].min(axis=0)
        max_x, max_y = points[:, :2].max(axis=0)
    cols = max(1, int(math.ceil((max_x - min_x) / cell_size)))
    rows = max(1, int(math.ceil((max_y - min_y) / cell_size)))
    max_cells = 140000
    if rows * cols > max_cells:
        scale = math.sqrt((rows * cols) / max_cells)
        cell_size *= scale
        cols = max(1, int(math.ceil((max_x - min_x) / cell_size)))
        rows = max(1, int(math.ceil((max_y - min_y) / cell_size)))

    ix = np.floor((points[:, 0] - min_x) / cell_size).astype(np.int64)
    iy = np.floor((points[:, 1] - min_y) / cell_size).astype(np.int64)
    valid = (ix >= 0) & (iy >= 0) & (ix < cols) & (iy < rows)
    ix = ix[valid]
    iy = iy[valid]
    z = points[:, 2][valid]
    flat = iy * cols + ix

    count = np.bincount(flat, minlength=rows * cols).astype(np.int32)
    sum_z = np.bincount(flat, weights=z, minlength=rows * cols).astype(np.float64)
    min_z = np.full(rows * cols, np.inf, dtype=np.float32)
    max_z = np.full(rows * cols, -np.inf, dtype=np.float32)
    np.minimum.at(min_z, flat, z)
    np.maximum.at(max_z, flat, z)

    mean_z = np.full(rows * cols, np.nan, dtype=np.float32)
    nonzero = count > 0
    mean_z[nonzero] = (sum_z[nonzero] / count[nonzero]).astype(np.float32)
    min_z[~nonzero] = np.nan
    max_z[~nonzero] = np.nan
    z_range = max_z - min_z
    z_values = mean_z[nonzero]

    return {
        "type": "height_grid",
        "version": 1,
        "generated_at": _now_text(),
        "cell_size": round(float(cell_size), 4),
        "cols": int(cols),
        "rows": int(rows),
        "origin": {"x": round(float(min_x), 4), "y": round(float(min_y), 4), "z": 0.0},
        "bounds": {
            "min_x": round(float(min_x), 4),
            "max_x": round(float(min_x + cols * cell_size), 4),
            "min_y": round(float(min_y), 4),
            "max_y": round(float(min_y + rows * cell_size), 4),
            "min_z": round(float(np.nanmin(mean_z)), 4) if z_values.size else 0.0,
            "max_z": round(float(np.nanmax(mean_z)), 4) if z_values.size else 0.0,
        },
        "heights": _rounded_array_or_null(mean_z),
        "min_heights": _rounded_array_or_null(min_z),
        "max_heights": _rounded_array_or_null(max_z),
        "ranges": _rounded_array_or_null(z_range),
        "counts": count.astype(int).tolist(),
    }


def _build_terrain_mesh(
    height_grid: Dict[str, Any],
    map_id: str,
    floor: str,
    pcd_path: Path,
    original_points: int,
) -> Dict[str, Any]:
    return {
        "ok": True,
        "available": True,
        "type": "terrain_mesh",
        "version": 1,
        "map_id": map_id,
        "floor": floor,
        "generated_at": _now_text(),
        "source": {
            "pcd_path": str(pcd_path),
            "source_points": original_points,
        },
        "terrain": {
            "representation": "height_grid_mesh",
            "cell_size": height_grid["cell_size"],
            "cols": height_grid["cols"],
            "rows": height_grid["rows"],
            "origin": height_grid["origin"],
            "bounds": height_grid["bounds"],
            "heights": height_grid["heights"],
        },
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


def _height_grid_stair_candidates(height_grid: Dict[str, Any], floor: str) -> List[Dict[str, Any]]:
    rows = int(height_grid.get("rows") or 0)
    cols = int(height_grid.get("cols") or 0)
    if rows <= 0 or cols <= 0:
        return []
    ranges = np.asarray(
        [np.nan if value is None else float(value) for value in height_grid.get("ranges", [])],
        dtype=np.float32,
    ).reshape((rows, cols))
    counts = np.asarray(height_grid.get("counts", []), dtype=np.int32).reshape((rows, cols))
    mask = (ranges >= 0.28) & (counts >= 8)
    zones: List[Dict[str, Any]] = []
    visited = np.zeros(mask.shape, dtype=bool)
    cell_size = float(height_grid.get("cell_size") or 0.25)
    origin = height_grid.get("origin") or {}
    min_area_cells = max(6, int(math.ceil(1.0 / max(cell_size * cell_size, 1e-6))))
    for row in range(rows):
        for col in range(cols):
            if visited[row, col] or not mask[row, col]:
                continue
            cells = _collect_component(mask, visited, row, col)
            if len(cells) < min_area_cells:
                continue
            ys = [item[0] for item in cells]
            xs = [item[1] for item in cells]
            min_c, max_c = min(xs), max(xs) + 1
            min_r, max_r = min(ys), max(ys) + 1
            width = (max_c - min_c) * cell_size
            height = (max_r - min_r) * cell_size
            if width < 0.7 or height < 0.7:
                continue
            x0 = float(origin.get("x", 0.0)) + min_c * cell_size
            y0 = float(origin.get("y", 0.0)) + min_r * cell_size
            x1 = float(origin.get("x", 0.0)) + max_c * cell_size
            y1 = float(origin.get("y", 0.0)) + max_r * cell_size
            zone_id = "%s_pcd_stair_candidate_%d" % (floor, len(zones) + 1)
            zones.append(
                {
                    "id": zone_id,
                    "name": "PCD疑似楼梯区域%d" % (len(zones) + 1),
                    "floor": floor,
                    "source_floor": floor,
                    "source": "pcd_height_candidate",
                    "trigger_gait": False,
                    "confidence": 0.45,
                    "center": {"x": round((x0 + x1) * 0.5, 4), "y": round((y0 + y1) * 0.5, 4)},
                    "polygon": [
                        {"x": round(x0, 4), "y": round(y0, 4)},
                        {"x": round(x1, 4), "y": round(y0, 4)},
                        {"x": round(x1, 4), "y": round(y1, 4)},
                        {"x": round(x0, 4), "y": round(y1, 4)},
                    ],
                    "message": "仅展示，不自动触发步态；需要现场确认后再升级为正式楼梯区",
                }
            )
            if len(zones) >= 12:
                return zones
    return zones


def _collect_component(mask: np.ndarray, visited: np.ndarray, row: int, col: int) -> List[Tuple[int, int]]:
    queue: deque[Tuple[int, int]] = deque([(row, col)])
    visited[row, col] = True
    cells: List[Tuple[int, int]] = []
    while queue:
        item = queue.popleft()
        cells.append(item)
        r, c = item
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if nr < 0 or nc < 0 or nr >= mask.shape[0] or nc >= mask.shape[1]:
                continue
            if visited[nr, nc] or not mask[nr, nc]:
                continue
            visited[nr, nc] = True
            queue.append((nr, nc))
    return cells


def _write_stair_pointclouds(
    derived_dir: Path,
    zones: List[Dict[str, Any]],
    points: np.ndarray,
    max_points: int,
) -> None:
    stair_dir = derived_dir / "stairs"
    stair_dir.mkdir(parents=True, exist_ok=True)
    max_points = max(1000, int(max_points))
    for zone in zones:
        polygon = zone.get("polygon") or []
        if len(polygon) < 3:
            continue
        xs = [float(point["x"]) for point in polygon]
        ys = [float(point["y"]) for point in polygon]
        mask = (
            (points[:, 0] >= min(xs))
            & (points[:, 0] <= max(xs))
            & (points[:, 1] >= min(ys))
            & (points[:, 1] <= max(ys))
        )
        local = points[mask]
        if len(local) > max_points:
            step = max(1, int(math.ceil(len(local) / max_points)))
            local = local[::step]
        payload = {
            "ok": True,
            "available": True,
            "zone_id": zone.get("id"),
            "points": [
                [round(float(x), 3), round(float(y), 3), round(float(z), 3)]
                for x, y, z in local
            ],
        }
        path = stair_dir / ("%s.json" % _slug(str(zone.get("id") or "zone")))
        _write_json(path, payload)
        zone["pointcloud"] = _relative_to(path, derived_dir.parent)
        zone["point_count"] = int(len(local))


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


def _rounded_array_or_null(values: np.ndarray) -> List[Optional[float]]:
    result: List[Optional[float]] = []
    for value in values.reshape((-1,)):
        if not math.isfinite(float(value)):
            result.append(None)
        else:
            result.append(round(float(value), 3))
    return result


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
