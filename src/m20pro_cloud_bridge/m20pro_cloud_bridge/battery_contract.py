"""Pure helpers for interpreting the vendor battery array.

The vendor message has a fixed-size array of battery slots.  An unplugged slot
is still published, but all of its telemetry is zero.  The web/API contract
must expose connected packs, not the number of array slots.
"""

from __future__ import annotations

import math
from typing import Any


def _nonzero_number(value: Any) -> bool:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number) and abs(number) > 1e-9


def _serial_present(value: Any) -> bool:
    if isinstance(value, (bytes, bytearray)):
        return any(int(item) != 0 for item in value)
    if isinstance(value, str):
        return bool(value.strip("\x00").strip())
    try:
        return any(int(item) != 0 for item in value)
    except (TypeError, ValueError):
        return bool(str(value).strip("\x00").strip())


def battery_pack_present(item: Any) -> bool:
    """Return whether one vendor battery slot contains usable telemetry."""

    if item is None:
        return False

    # These fields are zero for the unplugged slots observed on the M20 Pro.
    for field in (
        "voltage",
        "current",
        "remaining_capacity",
        "nominal_capacity",
        "cycles",
        "battery_level",
        "mos_state",
        "protected_state",
        "battery_quantity",
        "battery_ntc",
    ):
        if _nonzero_number(getattr(item, field, 0)):
            return True

    temperatures = getattr(item, "battery_temperature", []) or []
    if any(_nonzero_number(value) for value in temperatures):
        return True
    return _serial_present(getattr(item, "battery_serialnum", ""))
