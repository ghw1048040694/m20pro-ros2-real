"""Pure startup selected-map sync status helpers."""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, Optional


NowText = Callable[[], str]


def default_now_text() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def _base(
    *,
    attempt: int,
    max_attempts: int,
    now_text: Optional[NowText],
) -> Dict[str, Any]:
    return {
        "attempt": int(attempt),
        "max_attempts": int(max_attempts),
        "updated_at": (now_text or default_now_text)(),
    }


def startup_map_sync_skipped_payload(
    *,
    reason: str,
    attempt: int,
    max_attempts: int,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    return {
        "ok": True,
        "skipped": True,
        "reason": str(reason or "skipped"),
        **_base(attempt=attempt, max_attempts=max_attempts, now_text=now_text),
    }


def startup_map_sync_missing_record_payload(
    *,
    selected_map_id: str,
    attempt: int,
    max_attempts: int,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    return {
        "ok": False,
        "code": "selected_map_missing",
        "message": "启动同步失败：当前选中地图不存在",
        "selected_map_id": str(selected_map_id or ""),
        **_base(attempt=attempt, max_attempts=max_attempts, now_text=now_text),
    }


def startup_map_sync_result_payload(
    *,
    selected_map_id: str,
    map_name: Optional[str],
    nav2_load_map: Dict[str, Any],
    attempt: int,
    max_attempts: int,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    return {
        "ok": bool((nav2_load_map or {}).get("ok")),
        "selected_map_id": str(selected_map_id or ""),
        "map_name": map_name,
        "nav2_load_map": dict(nav2_load_map or {}),
        **_base(attempt=attempt, max_attempts=max_attempts, now_text=now_text),
    }


def startup_map_sync_retry_decision(
    nav2_load_map: Dict[str, Any],
    *,
    attempt: int,
    max_attempts: int,
) -> Dict[str, Any]:
    payload = dict(nav2_load_map or {})
    code = str(payload.get("code") or "")
    retryable = (not payload.get("ok")) and code in {
        "load_map_service_unavailable",
        "load_map_timeout",
    }
    attempts_left = max(0, int(max_attempts) - int(attempt))
    return {
        "retryable": bool(retryable),
        "retry": bool(retryable and attempts_left > 0),
        "attempt": int(attempt),
        "max_attempts": int(max_attempts),
        "attempts_left": attempts_left,
        "next_attempt": int(attempt) + 1 if retryable and attempts_left > 0 else None,
        "code": code,
    }
