"""Pure localization-status helpers for the M20Pro web dashboard."""

from __future__ import annotations

import math
import re
import time
from typing import Any, Callable, Dict, Optional


NowText = Callable[[], str]
_TCP_2101_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
_TCP_2101_POSE_RE = re.compile(r"\b(x|y|z|yaw)\s*=\s*(%s)\b" % _TCP_2101_NUMBER, re.IGNORECASE)


def default_now_text() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def _finite_number(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def pose_is_plausible(pose: Any, max_abs_position: float = 10000.0) -> bool:
    if not isinstance(pose, dict):
        return False
    required = ("x", "y", "z", "yaw")
    if any(key not in pose for key in required):
        return False
    if not all(_finite_number(pose.get(key)) for key in required):
        return False
    return all(abs(float(pose.get(key, 0.0))) <= max_abs_position for key in ("x", "y", "z"))


def relocalization_result_evidence(
    relocalization: Any,
    *,
    now_time: Optional[float] = None,
    recent_timeout_s: float = 300.0,
) -> Dict[str, Any]:
    result = dict(relocalization) if isinstance(relocalization, dict) else {}
    raw = str(result.get("raw") or "")
    last_update = None
    age_sec = None
    try:
        last_update = float(result.get("last_update"))
    except (TypeError, ValueError):
        last_update = None
    if now_time is not None and last_update is not None:
        age_sec = max(0.0, float(now_time) - last_update)
    recent = age_sec is not None and age_sec <= max(1.0, float(recent_timeout_s))
    return {
        "tcp_2101_result": raw,
        "tcp_2101_last_update": last_update,
        "tcp_2101_age_sec": age_sec,
        "tcp_2101_recent": recent,
        "tcp_2101_accepted": raw.startswith("success"),
        "tcp_2101_ambiguous": raw.startswith("pending_verification:"),
        "tcp_2101_failed": raw.startswith("failed:"),
    }


def parse_tcp_2101_success_pose(raw: Any) -> Optional[Dict[str, float]]:
    text = str(raw or "")
    if not text.startswith("success"):
        return None
    values: Dict[str, float] = {}
    for key, value in _TCP_2101_POSE_RE.findall(text):
        try:
            values[key.lower()] = float(value)
        except (TypeError, ValueError):
            return None
    if not all(key in values for key in ("x", "y", "z", "yaw")):
        return None
    pose = {
        "x": values["x"],
        "y": values["y"],
        "z": values["z"],
        "yaw": values["yaw"],
    }
    return pose if pose_is_plausible(pose) else None


def pose_tcp_2101_consistency_payload(
    pose: Any,
    tcp_2101_result: Any,
    *,
    pose_tolerance_m: float = 0.75,
) -> Dict[str, Any]:
    pose_payload = dict(pose) if isinstance(pose, dict) else {}
    pose_ok = pose_is_plausible(pose_payload)
    tcp_pose = parse_tcp_2101_success_pose(tcp_2101_result)
    tolerance_m = max(0.1, float(pose_tolerance_m))
    pose_error_m = None
    yaw_error_rad = None
    pose_near_2101 = None
    if pose_ok and tcp_pose is not None:
        try:
            pose_error_m = math.hypot(
                float(pose_payload.get("x", 0.0)) - float(tcp_pose.get("x", 0.0)),
                float(pose_payload.get("y", 0.0)) - float(tcp_pose.get("y", 0.0)),
            )
            yaw_error_rad = abs(
                wrap_angle(float(pose_payload.get("yaw", 0.0)) - float(tcp_pose.get("yaw", 0.0)))
            )
            pose_near_2101 = pose_error_m <= tolerance_m
        except (TypeError, ValueError):
            pose_near_2101 = False
    return {
        "tcp_2101_pose": tcp_pose,
        "pose_near_2101": pose_near_2101,
        "pose_error_m": pose_error_m,
        "yaw_error_rad": yaw_error_rad,
        "pose_tolerance_m": tolerance_m,
    }


def parse_initialpose_request(payload: Dict[str, Any]) -> Dict[str, Any]:
    try:
        pose = {
            "x": float(payload.get("x")),
            "y": float(payload.get("y")),
            "z": float(payload.get("z", 0.0)),
            "yaw": float(payload.get("yaw", 0.0)),
        }
    except (TypeError, ValueError):
        return {
            "ok": False,
            "code": "initialpose_pose_invalid",
            "message": "重定位坐标无效，请先在地图上拖箭头",
        }
    if not pose_is_plausible(pose):
        return {
            "ok": False,
            "code": "initialpose_pose_invalid",
            "message": "重定位坐标无效，请先在地图上拖箭头",
            "pose": pose,
        }
    frame_id = str(payload.get("frame_id") or "map").strip() or "map"
    floor = str(payload.get("floor") or "").strip()
    return {
        "ok": True,
        "pose": pose,
        "frame_id": frame_id,
        "floor": floor,
    }


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def map_relocalization_clearance_payload(
    *,
    map_relocalization_required: Optional[Dict[str, Any]],
    selected_map_id: Any,
    selected_map_status: Optional[Dict[str, Any]],
    localization_ok: Any,
    factory_localization_ok: Any,
    pose: Any,
    pose_age_sec: Optional[float],
    pose_timeout_s: float,
    relocalization_result: Any,
    lock_loaded_time: Optional[float] = None,
    now_time: Optional[float] = None,
    tcp_2101_recent_timeout_s: float = 300.0,
    pose_tolerance_m: float = 0.75,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    required = dict(map_relocalization_required or {})
    timeout_s = max(0.5, float(pose_timeout_s))
    tolerance_m = max(0.1, float(pose_tolerance_m))
    factory_ok = localization_ok if factory_localization_ok is None else factory_localization_ok
    pose_payload = dict(pose) if isinstance(pose, dict) else {}
    pose_ok = pose_is_plausible(pose_payload)
    pose_fresh = bool(pose_ok and pose_age_sec is not None and float(pose_age_sec) <= timeout_s)
    tcp = relocalization_result_evidence(
        relocalization_result,
        now_time=now_time,
        recent_timeout_s=tcp_2101_recent_timeout_s,
    )
    pose_tcp = pose_tcp_2101_consistency_payload(
        pose_payload,
        tcp.get("tcp_2101_result"),
        pose_tolerance_m=tolerance_m,
    )
    tcp_pose = pose_tcp.get("tcp_2101_pose")
    pose_near_2101 = pose_tcp.get("pose_near_2101")

    tcp_last_update = tcp.get("tcp_2101_last_update")
    tcp_after_lock = bool(
        tcp_last_update is not None
        and (lock_loaded_time is None or float(tcp_last_update) >= float(lock_loaded_time) - 1.0)
    )
    selected_id = str(selected_map_id or "").strip()
    lock_map_id = str(required.get("map_id") or "").strip()
    selected_status = dict(selected_map_status or {}) if isinstance(selected_map_status, dict) else {}
    base = {
        "clear": False,
        "map_relocalization_required": required or None,
        "selected_map_id": selected_id or None,
        "lock_map_id": lock_map_id or None,
        "selected_map_ready": selected_status.get("ready") if selected_status else None,
        "selected_map_status_code": selected_status.get("code") if selected_status else None,
        "tcp_2101_pose": tcp_pose,
        "tcp_2101_after_lock": tcp_after_lock,
        "factory_localization_ok": factory_ok,
        "pose_ok": pose_ok,
        "pose_fresh": pose_fresh,
        "pose_age_sec": pose_age_sec,
        "pose_timeout_s": timeout_s,
        "pose_near_2101": pose_near_2101,
        "pose_error_m": pose_tcp.get("pose_error_m"),
        "yaw_error_rad": pose_tcp.get("yaw_error_rad"),
        "pose_tolerance_m": tolerance_m,
        "lock_loaded_time": lock_loaded_time,
        "updated_at": (now_text or default_now_text)(),
        **tcp,
    }
    if not required:
        return {**base, "code": "no_map_relocalization_lock", "message": "没有固定地图重定位要求需要清除"}
    if lock_map_id and selected_id and lock_map_id != selected_id:
        return {
            **base,
            "code": "map_relocalization_lock_map_mismatch",
            "message": "固定地图重定位要求属于另一张地图，不能用当前证据清除",
        }
    if selected_status and selected_status.get("ready") is not True:
        return {
            **base,
            "code": str(selected_status.get("code") or "selected_map_not_ready"),
            "message": str(selected_status.get("message") or "网页选择地图与 Nav2 当前 /map 尚未一致，不能清除重定位要求"),
        }
    startup_sync_lock = str(required.get("reason") or "") == "startup_sync"
    if startup_sync_lock and factory_ok is True and pose_fresh:
        if tcp.get("tcp_2101_accepted") and tcp.get("tcp_2101_recent") and pose_near_2101 is False:
            return {
                **base,
                "code": "pose_not_near_tcp_2101",
                "message": "地图位姿与最近 2101 成功回执坐标不一致，不能清除重定位要求",
            }
        return {
            **base,
            "clear": True,
            "code": "startup_map_relocalization_lock_clearable",
            "message": "启动同步产生的固定地图重定位要求可清除：原厂定位确认、地图位姿新鲜",
        }
    if not tcp.get("tcp_2101_accepted"):
        return {**base, "code": "tcp_2101_not_success", "message": "未收到开发手册 2101/1 成功回执，不能清除重定位要求"}
    if not tcp.get("tcp_2101_recent"):
        return {**base, "code": "tcp_2101_not_recent", "message": "2101 成功回执不是最近结果，不能清除重定位要求"}
    if not tcp_after_lock:
        return {**base, "code": "tcp_2101_before_map_lock", "message": "2101 成功回执早于本次固定地图加载，不能清除重定位要求"}
    if factory_ok is not True:
        return {**base, "code": "factory_localization_not_confirmed", "message": "原厂定位尚未确认，不能清除重定位要求"}
    if not pose_fresh:
        return {**base, "code": "pose_missing_or_stale", "message": "地图位姿缺失、无效或已过期，不能清除重定位要求"}
    if pose_near_2101 is not True:
        return {
            **base,
            "code": "pose_not_near_tcp_2101",
            "message": "地图位姿与最近 2101 成功回执坐标不一致，不能清除重定位要求",
        }
    return {
        **base,
        "clear": True,
        "code": "map_relocalization_lock_clearable",
        "message": "固定地图重定位要求可清除：2101 成功、原厂定位确认、地图位姿新鲜",
    }


def relocalization_sample_evidence(
    *,
    request_started_at: float,
    requested_pose: Dict[str, Any],
    relocalization: Dict[str, Any],
    pose: Dict[str, Any],
    localization_ok: Any,
    scan: Dict[str, Any],
    local_costmap: Dict[str, Any],
    global_costmap: Dict[str, Any],
    pose_tolerance_m: float,
) -> Dict[str, Any]:
    result_age_ok = float(relocalization.get("last_update", 0.0) or 0.0) >= request_started_at
    result_text = str(relocalization.get("raw") or "") if result_age_ok else ""
    reply_accepted = result_text.startswith("success")
    reply_ambiguous = result_text.startswith("pending_verification:")
    pose_update = float(pose.get("last_update", pose.get("stamp", 0.0)) or 0.0)
    pose_ok = pose_update >= request_started_at and pose_is_plausible(pose)
    pose_error_m = None
    yaw_error_rad = None
    pose_near_request = False
    if pose_ok:
        try:
            pose_error_m = math.hypot(
                float(pose.get("x", 0.0)) - float(requested_pose.get("x", 0.0)),
                float(pose.get("y", 0.0)) - float(requested_pose.get("y", 0.0)),
            )
            yaw_error_rad = abs(
                wrap_angle(float(pose.get("yaw", 0.0)) - float(requested_pose.get("yaw", 0.0)))
            )
            pose_near_request = pose_error_m <= max(0.1, float(pose_tolerance_m))
        except (TypeError, ValueError):
            pose_near_request = False

    scan_ok = (
        float(scan.get("last_update", 0.0) or 0.0) >= request_started_at
        and int(scan.get("finite_ranges", 0) or 0) > 0
    )
    local_costmap_ok = (
        float(local_costmap.get("last_update", 0.0) or 0.0) >= request_started_at
        and bool(local_costmap.get("width"))
        and bool(local_costmap.get("height"))
    )
    global_costmap_ok = (
        float(global_costmap.get("last_update", 0.0) or 0.0) >= request_started_at
        and bool(global_costmap.get("width"))
        and bool(global_costmap.get("height"))
    )
    return {
        "tcp_2101_accepted": reply_accepted if result_age_ok else False,
        "tcp_2101_ambiguous": reply_ambiguous if result_age_ok else False,
        "tcp_2101_result": result_text,
        "tcp_2101_fresh": result_age_ok,
        "localization_ok": localization_ok is True,
        "pose_ok": pose_ok,
        "pose_near_request": pose_near_request,
        "scan_ok": scan_ok,
        "local_costmap_ok": local_costmap_ok,
        "global_costmap_ok": global_costmap_ok,
        "pose_error_m": pose_error_m,
        "yaw_error_rad": yaw_error_rad,
        "ready_to_finish_wait": bool(
            result_age_ok
            and (reply_accepted or reply_ambiguous)
            and localization_ok is True
            and pose_ok
            and pose_near_request
        ),
    }


def manual_relocalization_verification_payload(
    *,
    tcp_2101_accepted: bool,
    tcp_2101_result: str,
    tcp_2101_ambiguous: bool,
    localization_ok: bool,
    pose_ok: bool,
    pose_near_request: bool,
    scan_ok: bool,
    local_costmap_ok: bool,
    global_costmap_ok: bool,
    pose_error_m: Optional[float] = None,
    yaw_error_rad: Optional[float] = None,
    pose_tolerance_m: Optional[float] = None,
    navigation_readiness: Optional[Dict[str, Any]] = None,
    timeout_s: Optional[float] = None,
) -> Dict[str, Any]:
    """Build the frontend relocalization verdict from the vendor manual contract.

    The M20 developer manual defines relocalization as TCP Type=2101,
    Command=1. Publishing /initialpose is only our ROS-side trigger for that
    manual interface; it is not success evidence by itself.
    """

    result_text = str(tcp_2101_result or "")
    manual_reference = "山猫M20系列软件开发手册V0.0.9 1.4.1 Type=2101 Command=1"
    manual_ok = bool(tcp_2101_accepted)
    ambiguous_verified = bool(
        tcp_2101_ambiguous and localization_ok and pose_ok and pose_near_request
    )
    factory_pose_accepted = bool(
        (manual_ok or ambiguous_verified) and localization_ok and pose_ok and pose_near_request
    )
    navigation_ready = bool(
        factory_pose_accepted and scan_ok and local_costmap_ok and global_costmap_ok
    )
    checks = {
        "initialpose_published": "ok",
        "manual_tcp_2101": "ok" if manual_ok else ("warn" if tcp_2101_ambiguous else "fail"),
        "localization": "ok" if localization_ok else "warn",
        "map_pose": "ok" if pose_ok else "warn",
        "pose_near_request": "ok" if pose_near_request else "warn",
        "scan": "ok" if scan_ok else "warn",
        "local_costmap": "ok" if local_costmap_ok else "warn",
        "global_costmap": "ok" if global_costmap_ok else "warn",
    }
    if navigation_ready and ambiguous_verified:
        message = "2101/1 的 0xFFFF 回执与原厂执行结果不一致，但目标位姿证据已确认，导航链路已恢复"
    elif navigation_ready:
        message = "开发手册 2101/1 重定位已成功，原厂定位和导航链路已恢复"
    elif factory_pose_accepted and ambiguous_verified:
        message = "2101/1 回执异常，但原厂定位位姿已更新到请求位置；导航链路尚未全部恢复"
    elif factory_pose_accepted:
        message = "开发手册 2101/1 已成功，原厂定位位姿已更新，但导航链路尚未全部恢复"
    elif manual_ok:
        message = "开发手册 2101/1 已返回成功，但还未看到原厂定位位姿更新到请求位置"
    elif result_text.startswith("failed:"):
        message = "开发手册 2101/1 返回失败，重定位未确认：%s" % result_text
    else:
        message = "未收到开发手册 2101/1 成功回执，重定位未确认"

    return {
        "request_accepted": factory_pose_accepted,
        "initialpose_published": True,
        "tcp_2101_required": True,
        "tcp_2101_accepted": manual_ok,
        "tcp_2101_ambiguous": bool(tcp_2101_ambiguous),
        "tcp_2101_verified_by_pose": ambiguous_verified,
        "tcp_2101_diagnostic_only": False,
        "manual_reference": manual_reference,
        "factory_pose_accepted": factory_pose_accepted,
        "navigation_ready": navigation_ready,
        "message": message,
        "result": result_text or "未收到 /m20pro_tcp_bridge/relocalization_result；按开发手册 2101/1 视为未确认",
        "pose_error_m": pose_error_m,
        "yaw_error_rad": yaw_error_rad,
        "pose_tolerance_m": pose_tolerance_m,
        "checks": checks,
        "navigation_readiness": navigation_readiness,
        "timeout_s": timeout_s,
    }


def localization_status_payload(
    *,
    localization_ok: Any,
    pose: Any,
    pose_age_sec: Optional[float],
    pose_timeout_s: float,
    navigation_status: Any = None,
    navigation_readiness: Optional[Dict[str, Any]] = None,
    factory_localization_ok: Any = None,
    relocalization_result: Any = None,
    map_relocalization_required: Optional[Dict[str, Any]] = None,
    now_time: Optional[float] = None,
    tcp_2101_recent_timeout_s: float = 300.0,
    tcp_2101_pose_tolerance_m: float = 0.75,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    timeout_s = max(0.5, float(pose_timeout_s))
    pose_ok = pose_is_plausible(pose)
    age_ok = pose_age_sec is not None and float(pose_age_sec) <= timeout_s
    factory_ok = localization_ok if factory_localization_ok is None else factory_localization_ok
    tcp = relocalization_result_evidence(
        relocalization_result,
        now_time=now_time,
        recent_timeout_s=tcp_2101_recent_timeout_s,
    )
    pose_tcp = pose_tcp_2101_consistency_payload(
        pose,
        tcp.get("tcp_2101_result"),
        pose_tolerance_m=tcp_2101_pose_tolerance_m,
    )
    base = {
        "localization_ok": localization_ok,
        "factory_localization_ok": factory_ok,
        "pose_ok": pose_ok,
        "pose_fresh": bool(pose_ok and age_ok),
        "pose_age_sec": pose_age_sec,
        "pose_timeout_s": timeout_s,
        "navigation_status": navigation_status,
        "map_relocalization_required": map_relocalization_required,
        "navigation_ready": None,
        "updated_at": (now_text or default_now_text)(),
        **pose_tcp,
        **tcp,
    }
    if navigation_readiness is not None:
        base["navigation_ready"] = bool(navigation_readiness.get("ready"))
        base["navigation_readiness_message"] = navigation_readiness.get("message")

    if map_relocalization_required:
        if (
            tcp.get("tcp_2101_accepted")
            and tcp.get("tcp_2101_recent")
            and factory_ok is True
            and pose_ok
            and age_ok
        ):
            message = (
                "重定位失败：已收到 2101 回执，原厂定位和地图位姿也在更新；"
                "但当前固定地图的重定位要求还没有清除"
            )
        elif tcp.get("tcp_2101_accepted") and tcp.get("tcp_2101_recent"):
            message = "重定位失败：已收到 2101 回执，但原厂定位或地图位姿还没有确认"
        elif tcp.get("tcp_2101_accepted"):
            message = "重定位失败：只有旧的 2101 回执，不是本轮新确认；当前固定地图仍要求重定位"
        else:
            message = "重定位失败：%s" % str(
                map_relocalization_required.get("message")
                or "当前固定地图仍要求按开发手册2101完成重定位"
            )
        return {
            **base,
            "confirmed": False,
            "code": "map_relocalization_required",
            "message": message,
        }
    if factory_ok is not True:
        return {
            **base,
            "confirmed": False,
            "code": "localization_not_confirmed",
            "message": "重定位失败：原厂定位未确认",
        }
    if not pose_ok:
        return {
            **base,
            "confirmed": False,
            "code": "pose_missing_or_invalid",
            "message": "重定位失败：原厂定位为 true，但地图位姿缺失或无效",
        }
    if not age_ok:
        return {
            **base,
            "confirmed": False,
            "code": "pose_stale",
            "message": "重定位失败：地图位姿已过期",
        }
    source_note = "定位已确认"
    if localization_ok is not True:
        source_note = "定位状态源不一致：/localization_ok 未确认，但原厂导航状态确认定位"
    return {
        **base,
        "confirmed": True,
        "code": "localized_confirmed",
        "message": "重定位成功：%s" % source_note,
    }


def relocalization_response_payload(
    verification: Dict[str, Any],
    *,
    now_text: Optional[NowText] = None,
) -> Dict[str, Any]:
    confirmed = bool(verification.get("factory_pose_accepted"))
    navigation_ready = bool(verification.get("navigation_ready"))

    if confirmed:
        message = "重定位成功"
        code = "localized_confirmed"
    else:
        message = "重定位失败：%s" % str(verification.get("message") or "已发布 /initialpose，但尚未确认定位生效")
        code = "localization_not_confirmed"

    return {
        "confirmed": confirmed,
        "navigation_ready": navigation_ready,
        "tcp_2101_required": verification.get("tcp_2101_required"),
        "tcp_2101_accepted": verification.get("tcp_2101_accepted"),
        "manual_reference": verification.get("manual_reference"),
        "code": code,
        "message": message,
        "verification_message": verification.get("message"),
        "updated_at": (now_text or default_now_text)(),
    }


def initialpose_api_response_payload(
    *,
    localization_status: Dict[str, Any],
    verification: Dict[str, Any],
    topic: Any,
    publish_repeats: Any,
    frame_id: Any,
    floor: Any,
    pose: Dict[str, Any],
) -> Dict[str, Any]:
    """Build the complete /api/localization/initialpose response.

    The web node owns ROS publication. This contract owns the API semantics so
    "published", "factory confirmed" and "task ready" stay consistent.
    """

    normalized_pose = {
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "z": float(pose.get("z", 0.0)),
        "yaw": float(pose.get("yaw", 0.0)),
    }
    return {
        "ok": True,
        "confirmed": bool(localization_status.get("confirmed")),
        "navigation_ready": bool(localization_status.get("navigation_ready")),
        "code": localization_status.get("code"),
        "message": localization_status.get("message", "已发布网页重定位请求"),
        "topic": str(topic),
        "publish_repeats": int(publish_repeats),
        "frame_id": str(frame_id or "map"),
        "floor": str(floor or ""),
        "pose": normalized_pose,
        "verification": verification,
        "localization_status": localization_status,
    }
