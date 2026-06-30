"""Small pure helpers shared by the web dashboard runtime."""

from __future__ import annotations

import json
import re
import time
import uuid
from typing import Any, Dict, Optional


def parse_json_text(text: str) -> Any:
    try:
        return json.loads(text)
    except Exception:
        return None


def now_text() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def fmt_age_text(age: Optional[float]) -> str:
    if age is None:
        return "无时间"
    if age < 1.0:
        return "<1s"
    return f"{age:.0f}s前"


def payload_with_age(payload: Optional[Dict[str, Any]], now: Optional[float] = None) -> Optional[Dict[str, Any]]:
    if not payload:
        return None
    result = dict(payload)
    timestamp = result.get("timestamp")
    if timestamp is not None:
        result["age_sec"] = max(0.0, (time.time() if now is None else now) - float(timestamp))
    return result


def new_id(prefix: str) -> str:
    return f"{prefix}_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}"


def random_suffix(length: int = 6) -> str:
    return uuid.uuid4().hex[: max(1, int(length))]


def sanitize_name(value: str, fallback: str) -> str:
    text = str(value or "").strip() or fallback
    text = re.sub(r"[^\w.\-\u4e00-\u9fff]+", "_", text)
    return text.strip("._") or fallback


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def api_error_payload(message: str, extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    payload: Dict[str, Any] = {"ok": False, "message": message}
    if extra:
        payload.update(extra)
    return payload
