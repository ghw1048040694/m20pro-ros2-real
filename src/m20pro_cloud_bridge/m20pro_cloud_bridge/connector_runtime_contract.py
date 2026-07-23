"""Pure readiness gate for the unified connector runtime components."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional


def _finite(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _component_readiness(
    record: Any,
    *,
    component: str,
    label: str,
    now_unix_s: float,
    timeout_s: float,
) -> Dict[str, Any]:
    if not isinstance(record, dict):
        return {
            "ok": False,
            "code": f"{component}_status_missing",
            "message": f"未收到{label}状态，拒绝启动楼梯连接边",
        }
    parsed = record.get("parsed")
    if not isinstance(parsed, dict):
        return {
            "ok": False,
            "code": f"{component}_status_invalid",
            "message": f"{label}状态格式无效，拒绝启动楼梯连接边",
        }
    if str(parsed.get("component") or "").strip() != component:
        return {
            "ok": False,
            "code": f"{component}_identity_mismatch",
            "message": f"{label}状态身份不匹配，拒绝启动楼梯连接边",
        }
    stamp = _finite(record.get("last_update"))
    age_s = max(0.0, float(now_unix_s) - stamp) if stamp is not None else None
    freshness_limit = max(1.0, float(timeout_s))
    if age_s is None or age_s > freshness_limit:
        return {
            "ok": False,
            "code": f"{component}_status_stale",
            "message": f"{label}状态已过期，拒绝启动楼梯连接边",
            "age_s": age_s,
            "timeout_s": freshness_limit,
        }
    if not bool(parsed.get("enabled")) or not bool(parsed.get("ready")):
        return {
            "ok": False,
            "code": f"{component}_disabled",
            "message": f"{label}未启用或未就绪，拒绝启动楼梯连接边",
            "status_code": parsed.get("code"),
        }
    if bool(parsed.get("busy")):
        return {
            "ok": False,
            "code": f"{component}_busy",
            "message": f"{label}仍占用上一条连接边，拒绝并发启动",
            "active_request_id": parsed.get("request_id"),
        }
    return {
        "ok": True,
        "code": f"{component}_ready",
        "message": f"{label}已就绪",
        "age_s": age_s,
    }


def connector_runtime_readiness(
    *,
    executor_status: Any,
    orchestrator_status: Any,
    now_unix_s: float,
    timeout_s: float,
) -> Dict[str, Any]:
    """Require both halves of the one connector pipeline before dispatch."""
    checks = {
        "stair_executor": _component_readiness(
            executor_status,
            component="stair_executor",
            label="楼梯语义执行器",
            now_unix_s=now_unix_s,
            timeout_s=timeout_s,
        ),
        "stair_action_orchestrator": _component_readiness(
            orchestrator_status,
            component="stair_action_orchestrator",
            label="楼梯动作编排器",
            now_unix_s=now_unix_s,
            timeout_s=timeout_s,
        ),
    }
    for name in ("stair_executor", "stair_action_orchestrator"):
        if not checks[name].get("ok"):
            return {**checks[name], "checks": checks}
    return {
        "ok": True,
        "code": "connector_runtime_ready",
        "message": "楼梯执行器与动作编排器均已启用并就绪",
        "checks": checks,
    }


__all__ = ["connector_runtime_readiness"]
