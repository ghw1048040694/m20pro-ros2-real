"""Pure validation and serialization rules for native M20 charge navigation."""

from __future__ import annotations

import math
from typing import Any, Dict


def build_charge_command_items(payload: Any) -> Dict[str, Any]:
    """Build the Type=1003/Command=1 Items payload for PointInfo=3.

    The incoming point type is deliberately ignored.  This entry point is
    only reachable from a validated frontend charge annotation, so the
    native command must never silently become an ordinary task point.
    """

    if not isinstance(payload, dict):
        return {"ok": False, "request_id": "", "message": "charge command must be an object"}
    request_id = str(payload.get("request_id") or "").strip()
    if not request_id:
        return {"ok": False, "request_id": "", "message": "charge command request_id is required"}
    try:
        items = {
            "Value": 1,
            "MapID": int(payload.get("map_id", 0)),
            "PosX": float(payload["x"]),
            "PosY": float(payload["y"]),
            "PosZ": float(payload.get("z", 0.0)),
            "AngleYaw": float(payload["yaw"]),
            "PointInfo": 3,
            "Gait": int(payload.get("gait", 12)),
            "Speed": int(payload.get("speed", 1)),
            "Manner": 0,
            "ObsMode": 0,
            "NavMode": 1,
        }
    except (KeyError, TypeError, ValueError):
        return {"ok": False, "request_id": request_id, "message": "invalid charge target"}
    if not all(
        math.isfinite(float(items[key]))
        for key in ("PosX", "PosY", "PosZ", "AngleYaw")
    ):
        return {
            "ok": False,
            "request_id": request_id,
            "message": "charge target is not finite",
        }
    return {"ok": True, "request_id": request_id, "items": items}
