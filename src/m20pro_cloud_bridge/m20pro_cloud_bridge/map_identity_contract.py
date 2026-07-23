"""Immutable occupancy-map identity helpers used by map transactions.

Metadata such as width and resolution is useful for a quick sanity check but
does not prove that Nav2 loaded the requested occupancy grid.  This module
provides one deterministic digest for the normalized OccupancyGrid payload so
the Web node can compare the archived map with the live ``/map`` sample.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
from typing import Any, Dict, Optional


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return default
    return number if math.isfinite(number) else default


def occupancy_grid_content_digest(payload: Dict[str, Any]) -> Optional[str]:
    """Return a digest of normalized map metadata and occupancy cells.

    The digest intentionally ignores runtime timestamps, map names and map IDs;
    those are identities of an asset, not its contents.  Occupancy values are
    encoded as signed 32-bit integers to keep the result independent of Python
    list/array implementations and ROS serialization details.
    """
    if not isinstance(payload, dict) or not payload.get("available"):
        return None
    try:
        width = int(payload["width"])
        height = int(payload["height"])
        resolution = _finite(payload["resolution"])
        origin_raw = payload.get("origin") if isinstance(payload.get("origin"), dict) else {}
        origin = {
            "x": _finite(origin_raw.get("x")),
            "y": _finite(origin_raw.get("y")),
            "z": _finite(origin_raw.get("z")),
            "yaw": _finite(origin_raw.get("yaw")),
        }
        cells = payload["data"]
        if width <= 0 or height <= 0 or not isinstance(cells, (list, tuple)):
            return None
        if len(cells) != width * height:
            return None
        normalized = [int(value) for value in cells]
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if any(value < -1 or value > 100 for value in normalized):
        return None
    header = json.dumps(
        {
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin": origin,
        },
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    digest = hashlib.sha256()
    digest.update(header)
    digest.update(b"\0")
    digest.update(b"".join(struct.pack("<i", value) for value in normalized))
    return digest.hexdigest()


def map_content_match(
    expected: Dict[str, Any],
    observed: Dict[str, Any],
    *,
    require_digest: bool = True,
) -> Dict[str, Any]:
    """Compare one archived map payload with a live OccupancyGrid payload."""
    expected_digest = str(
        expected.get("content_digest") or occupancy_grid_content_digest(expected) or ""
    )
    observed_digest = str(
        observed.get("content_digest") or occupancy_grid_content_digest(observed) or ""
    )
    if not expected_digest or not observed_digest:
        return {
            "ok": not require_digest,
            "code": "map_content_digest_missing",
            "message": "地图内容摘要缺失，不能确认 /map 与归档地图一致" if require_digest else "地图内容摘要未提供",
            "expected_digest": expected_digest or None,
            "observed_digest": observed_digest or None,
        }
    return {
        "ok": expected_digest == observed_digest,
        "code": "map_content_match" if expected_digest == observed_digest else "map_content_mismatch",
        "message": "地图内容摘要一致" if expected_digest == observed_digest else "Nav2 /map 内容摘要与目标地图不一致",
        "expected_digest": expected_digest,
        "observed_digest": observed_digest,
    }


def factory_identity_match(
    expected: Dict[str, Any],
    active: Dict[str, Any],
    *,
    expected_digest: Optional[str] = None,
) -> Dict[str, Any]:
    """Compare a captured 106 map package with the current ``active`` target.

    A canonical path is useful evidence, but a path alone cannot prove that a
    package was not replaced in place.  When a digest is available it is
    therefore mandatory; a path-only comparison is accepted only for legacy
    callers that have no content digest at all.
    """
    expected_path = str((expected or {}).get("resolved_path") or "").strip()
    active_path = str((active or {}).get("resolved_path") or "").strip()
    captured_digest = str(
        expected_digest
        or (expected or {}).get("content_digest")
        or ""
    ).strip().lower()
    active_digest = str((active or {}).get("content_digest") or "").strip().lower()
    path_match = bool(expected_path and active_path and expected_path == active_path)
    content_match = bool(captured_digest and active_digest and captured_digest == active_digest)
    ok = content_match if captured_digest else path_match
    return {
        "ok": bool(ok),
        "identity_mode": "path" if path_match and ok else ("content" if content_match else None),
        "expected_resolved_path": expected_path or None,
        "active_resolved_path": active_path or None,
        "expected_content_digest": captured_digest or None,
        "active_content_digest": active_digest or None,
        "path_match": path_match,
        "content_match": content_match,
        "code": "factory_identity_match" if ok else "factory_identity_mismatch",
        "message": (
            "106 active 已通过内容摘要确认目标地图"
            if ok and content_match
            else ("106 active 已解析到目标地图包" if ok else "106 active 路径和内容摘要均未确认目标地图")
        ),
    }
