"""Pure Nav2 status parsing and active-goal matching helpers."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional


def parse_key_value_status(text: str) -> Dict[str, Any]:
    parsed: Dict[str, Any] = {}
    for token in str(text or "").replace(",", " ").split():
        key, sep, value = token.partition("=")
        if not sep or not key:
            continue
        normalized_key = key.strip().lower()
        raw_value = value.strip()
        if raw_value in ("", "None", "none", "null"):
            parsed[normalized_key] = None
            continue
        try:
            number = float(raw_value)
            parsed[normalized_key] = int(number) if number.is_integer() else number
        except ValueError:
            parsed[normalized_key] = raw_value
    return parsed


def classify_navigation_status(status_text: str) -> Dict[str, Any]:
    text = str(status_text or "").strip()
    if not text:
        return {"action": "ignore", "reason": "empty_status"}
    if text.startswith("nav_goal_accepted"):
        payload = parse_key_value_status(text)
        label = str(payload.get("label") or "")
        if label and label != "floor_goal":
            return {"action": "update_transition_status", "goal_status": "accepted", "label": label}
        return {"action": "update_goal_status", "goal_status": "accepted"}
    if text.startswith("nav_goal_feedback"):
        payload = parse_key_value_status(text)
        label = str(payload.get("label") or "")
        if label and label != "floor_goal":
            return {"action": "update_transition_feedback", "label": label}
        return {"action": "update_feedback"}
    if text.startswith("nav_goal_succeeded"):
        payload = parse_key_value_status(text)
        if payload.get("label") == "floor_goal":
            return {"action": "complete_waypoint"}
        label = str(payload.get("label") or "")
        if label:
            return {"action": "update_transition_status", "goal_status": "succeeded", "label": label}
        return {"action": "update_goal_status", "goal_status": "succeeded"}
    if text.startswith("ignored reason=duplicate_floor_goal"):
        return {"action": "update_goal_status", "goal_status": "accepted"}
    if text.startswith("nav_goal_cancelled") or text.startswith("replacing_active_floor_goal"):
        return {"action": "fail", "goal_status": "interrupted"}
    if text.startswith("blocked "):
        return {"action": "fail", "goal_status": "blocked"}
    if text.startswith("error "):
        return {"action": "fail", "goal_status": "error"}
    return {"action": "update_message"}


def friendly_nav_status(status_text: str) -> str:
    text = str(status_text or "")
    payload = parse_key_value_status(text)
    label = str(payload.get("label") or "")
    transition_labels = {
        "stair_entry": "正在前往楼梯入口",
        "stair_traverse": "正在通过楼梯平台",
        "stair_exit": "正在离开楼梯区域",
    }
    if label in transition_labels:
        if text.startswith("nav_goal_accepted"):
            return "%s，Nav2 已接收" % transition_labels[label]
        if text.startswith("nav_goal_feedback"):
            distance_remaining = payload.get("distance_remaining")
            if isinstance(distance_remaining, (int, float)):
                return "%s，剩余 %.2f m" % (transition_labels[label], float(distance_remaining))
            return transition_labels[label]
        if text.startswith("nav_goal_succeeded"):
            return "%s完成" % transition_labels[label]
    if text.startswith("navigating_to_stair_entry"):
        return "跨楼层：正在前往楼梯入口"
    if text.startswith("started source_floor="):
        return "跨楼层：已切换楼梯步态，准备通过楼梯"
    if text.startswith("navigating_to_stair_platform"):
        return "跨楼层：正在通过楼梯平台"
    if text.startswith("switching_map_at_platform"):
        return "跨楼层：楼梯平台到位，正在切换楼层地图"
    if text.startswith("navigating_from_platform_to_flat"):
        return "跨楼层：目标楼层地图已切换，正在离开楼梯区域"
    if text.startswith("navigating_to_floor_goal"):
        return "跨楼层：已到目标楼层，正在前往任务点"
    if text.startswith("complete target_floor="):
        return "跨楼层：楼层切换完成"
    if text.startswith("nav_goal_accepted"):
        return "Nav2 已接收当前点位，正在导航"
    if text.startswith("nav_goal_feedback"):
        feedback = parse_key_value_status(text)
        distance_remaining = feedback.get("distance_remaining")
        navigation_time = feedback.get("navigation_time")
        recoveries = feedback.get("recoveries")
        parts = ["Nav2 正在执行当前点位"]
        if isinstance(distance_remaining, (int, float)):
            parts.append("剩余 %.2f m" % float(distance_remaining))
        if isinstance(navigation_time, (int, float)):
            parts.append("已导航 %.0f 秒" % float(navigation_time))
        if isinstance(recoveries, (int, float)) and float(recoveries) > 0:
            parts.append("恢复 %d 次" % int(recoveries))
        return "，".join(parts)
    if text.startswith("nav_goal_succeeded"):
        return "Nav2 已确认到达当前点位"
    if text.startswith("ignored reason=duplicate_floor_goal"):
        return "重复目标已忽略，继续执行当前导航"
    if text.startswith("nav_goal_cancelled") or text.startswith("replacing_active_floor_goal"):
        return "当前导航被取消/替换，任务已停止，请查看最近事件"
    if text.startswith("same_floor_goal"):
        return "同楼层目标已转交 Nav2"
    if text.startswith("flat_gait_requested"):
        return "已请求平地步态，等待 Nav2 接收目标"
    if text.startswith("stopped reason=before_start_task"):
        return "任务启动前导航会话已复位"
    if text.startswith("blocked reason=not_at_entry"):
        payload = parse_key_value_status(text)
        distance = payload.get("distance")
        tolerance = payload.get("tolerance")
        if isinstance(distance, (int, float)) and isinstance(tolerance, (int, float)):
            return "跨楼层被阻塞：距离楼梯入口 %.2f m，超过 %.2f m" % (
                float(distance),
                float(tolerance),
            )
        return "跨楼层被阻塞：机器人不在楼梯入口附近"
    if text.startswith("blocked "):
        return "跨楼层被阻塞，任务已停止"
    if "nav_goal_failed" in text:
        return "Nav2 当前点位执行失败，请查看状态码和现场障碍"
    if text.startswith("error "):
        return "导航链路报错，任务已停止"
    return text


def apply_nav_failure_state(
    active: Dict[str, Any],
    *,
    goal_status: str,
    status_text: str,
) -> Dict[str, Any]:
    updated = dict(active)
    updated["last_nav_goal_status"] = goal_status
    updated["last_nav_status"] = status_text
    updated["last_error"] = status_text
    updated["status_message"] = friendly_nav_status(status_text)
    return updated


def apply_nav_goal_status_state(
    active: Dict[str, Any],
    *,
    goal_status: str,
    status_text: str,
    match: Dict[str, Any],
    now_monotonic: float,
    now_text: str,
) -> Dict[str, Any]:
    updated = dict(active)
    updated["last_nav_goal_status"] = goal_status
    updated["last_nav_status"] = status_text
    updated["last_nav_status_at"] = now_text
    updated["last_nav_goal_match"] = match
    if goal_status == "accepted":
        updated["last_nav_accepted_monotonic"] = float(now_monotonic)
        updated["last_nav_accepted_at"] = now_text
    if match.get("nav_goal_seq") is not None:
        updated["last_nav_goal_seq"] = match.get("nav_goal_seq")
    updated["status_message"] = friendly_nav_status(status_text)
    return updated


def apply_nav_status_message_state(
    active: Dict[str, Any],
    *,
    status_text: str,
    now_text: str,
) -> Dict[str, Any]:
    updated = dict(active)
    updated["last_nav_status"] = status_text
    updated["last_nav_status_at"] = now_text
    updated["status_message"] = friendly_nav_status(status_text)
    return updated


def apply_nav_feedback_state(
    active: Dict[str, Any],
    *,
    status_text: str,
    feedback: Dict[str, Any],
    match: Dict[str, Any],
    now_monotonic: float,
    now_text: str,
) -> Dict[str, Any]:
    updated = dict(active)
    updated["last_nav_status"] = status_text
    updated["last_nav_status_at"] = now_text
    updated["last_nav_feedback"] = feedback
    updated["last_nav_feedback_at"] = now_text
    updated["last_nav_feedback_monotonic"] = float(now_monotonic)
    updated["last_nav_goal_match"] = match
    if match.get("nav_goal_seq") is not None and updated.get("last_nav_goal_seq") is None:
        updated["last_nav_goal_seq"] = match.get("nav_goal_seq")
    distance_remaining = feedback.get("distance_remaining")
    if isinstance(distance_remaining, (int, float)):
        updated["last_nav_distance_remaining_m"] = float(distance_remaining)
    status_message = friendly_nav_status(status_text)
    if status_message:
        updated["status_message"] = status_message
    recoveries = feedback.get("recoveries")
    updated["has_nav_feedback"] = True
    if recoveries is not None:
        updated["last_nav_feedback_recoveries"] = recoveries
    return updated


def nav_goal_status_event_payload(
    active: Dict[str, Any],
    *,
    goal_status: str,
    status_text: str,
    status_payload: Dict[str, Any],
    match: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "event": "nav_%s" % str(goal_status or "status"),
        "message": str(active.get("status_message") or friendly_nav_status(status_text) or status_text),
        "extra": {
            "nav_status": status_text,
            "nav_status_payload": status_payload,
            "nav_goal_match": match,
        },
    }


def nav_status_message_event_payload(active: Dict[str, Any], *, status_text: str) -> Dict[str, Any]:
    return {
        "event": "nav_status",
        "message": str(active.get("status_message") or friendly_nav_status(status_text) or status_text),
        "extra": {
            "nav_status": status_text,
        },
    }


def nav_feedback_event_payload(
    active: Dict[str, Any],
    *,
    status_text: str,
    feedback: Dict[str, Any],
    match: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "event": "nav_feedback",
        "message": str(friendly_nav_status(status_text) or "Nav2 正在执行当前点位"),
        "extra": {
            "nav_status": status_text,
            "nav_feedback": feedback,
            "nav_goal_match": match,
        },
    }


def should_record_nav_feedback_event(
    active_before: Dict[str, Any],
    feedback: Dict[str, Any],
) -> bool:
    if not active_before.get("has_nav_feedback"):
        return True
    recoveries = feedback.get("recoveries")
    return recoveries is not None and recoveries != active_before.get("last_nav_feedback_recoveries")


def nav_feedback_dispatch_payload(status_text: str) -> Dict[str, Any]:
    feedback = parse_key_value_status(status_text)
    if str(feedback.get("label") or "") != "floor_goal":
        return {
            "action": "update_message",
            "feedback": feedback,
            "reason": "not_floor_goal_feedback",
        }
    return {
        "action": "update_feedback",
        "feedback": feedback,
    }


def apply_transition_nav_status_state(
    active: Dict[str, Any],
    *,
    goal_status: Optional[str],
    status_text: str,
    status_payload: Dict[str, Any],
    now_text: str,
) -> Dict[str, Any]:
    updated = dict(active)
    updated["last_transition_nav_status"] = status_text
    updated["last_transition_nav_status_at"] = now_text
    updated["last_transition_nav_payload"] = dict(status_payload)
    if goal_status:
        updated["last_transition_nav_goal_status"] = str(goal_status)
    label = status_payload.get("label")
    if label:
        updated["last_transition_nav_label"] = label
    updated["status_message"] = friendly_nav_status(status_text)
    return updated


def transition_nav_status_event_payload(active: Dict[str, Any], *, status_text: str, status_payload: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "event": "cross_floor_nav_status",
        "message": str(active.get("status_message") or friendly_nav_status(status_text) or status_text),
        "extra": {
            "nav_status": status_text,
            "nav_status_payload": dict(status_payload),
        },
    }


def apply_ignored_nav_status_state(
    active: Dict[str, Any],
    *,
    status_text: str,
    match: Dict[str, Any],
) -> Dict[str, Any]:
    updated = dict(active)
    updated["last_ignored_nav_status"] = status_text
    updated["last_ignored_nav_goal_match"] = match
    return updated


def ignored_nav_status_event_payload(
    active: Dict[str, Any],
    *,
    message: str,
    status_text: str,
    match: Dict[str, Any],
) -> Dict[str, Any]:
    timeline_extra = {
        "nav_status": status_text,
        "nav_goal_match": match,
    }
    operator_payload = {
        "task_id": active.get("task_id"),
        "annotation_id": match.get("annotation_id"),
        "label": match.get("label"),
        "nav_status": status_text,
        "nav_goal_match": match,
    }
    return {
        "timeline_event": "nav_status_ignored",
        "timeline_message": message,
        "timeline_extra": timeline_extra,
        "operator_event": message,
        "operator_payload": operator_payload,
    }


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def nav_status_matches_active_goal(
    active: Dict[str, Any],
    annotation: Optional[Dict[str, Any]],
    status_payload: Dict[str, Any],
) -> Dict[str, Any]:
    if annotation is None:
        return {"matches": False, "reason": "no_active_annotation"}
    if str(status_payload.get("label") or "") not in ("", "floor_goal"):
        return {
            "matches": False,
            "reason": "label_mismatch",
            "status_label": status_payload.get("label"),
        }

    pose = annotation.get("pose") if isinstance(annotation.get("pose"), dict) else {}
    match: Dict[str, Any] = {
        "matches": True,
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
        "last_goal_annotation_id": active.get("last_goal_annotation_id"),
        "last_goal_attempt_id": active.get("last_goal_attempt_id"),
    }
    if active.get("last_goal_annotation_id") != annotation.get("id"):
        match.update({"matches": False, "reason": "annotation_mismatch"})
        return match

    expected_seq = active.get("last_nav_goal_seq")
    if expected_seq is not None:
        match["expected_nav_goal_seq"] = expected_seq
    if status_payload.get("goal_seq") is not None:
        match["nav_goal_seq"] = status_payload.get("goal_seq")
        if expected_seq is not None:
            try:
                if int(status_payload.get("goal_seq")) != int(expected_seq):
                    match.update(
                        {
                            "matches": False,
                            "reason": "goal_seq_mismatch",
                            "expected": int(expected_seq),
                            "observed": int(status_payload.get("goal_seq")),
                        }
                    )
                    return match
            except (TypeError, ValueError):
                match.update({"matches": False, "reason": "goal_seq_invalid"})
                return match

    for key, payload_key in (("x", "goal_x"), ("y", "goal_y"), ("z", "goal_z"), ("yaw", "goal_yaw")):
        if payload_key not in status_payload:
            continue
        try:
            expected = float(pose.get(key, 0.0))
            observed = float(status_payload.get(payload_key))
        except (TypeError, ValueError):
            match.update({"matches": False, "reason": f"{payload_key}_invalid"})
            return match
        if key == "yaw":
            error = abs(_wrap_angle(expected - observed))
            tolerance = 0.25
        else:
            error = abs(expected - observed)
            tolerance = 0.15 if key in ("x", "y") else 0.25
        match[f"{payload_key}_error"] = error
        if error > tolerance:
            match.update(
                {
                    "matches": False,
                    "reason": f"{payload_key}_mismatch",
                    "expected": expected,
                    "observed": observed,
                    "tolerance": tolerance,
                }
            )
            return match
    return match


def nav_success_completion_decision(
    active: Dict[str, Any],
    annotation: Optional[Dict[str, Any]],
    status_text: str,
) -> Dict[str, Any]:
    status_payload = parse_key_value_status(status_text)
    match = nav_status_matches_active_goal(active, annotation, status_payload)
    nav_status = str(active.get("last_nav_goal_status") or "")
    if annotation is None:
        return {
            "action": "ignore",
            "reason": "no_active_annotation",
            "status_payload": status_payload,
            "match": match,
            "event_extra": {
                "last_goal_annotation_id": active.get("last_goal_annotation_id"),
                "last_nav_goal_status": nav_status,
                "nav_status": status_text,
                "nav_status_payload": status_payload,
                "nav_goal_match": match,
            },
        }
    event_extra = {
        "annotation_id": annotation.get("id"),
        "label": annotation.get("label"),
        "last_goal_annotation_id": active.get("last_goal_annotation_id"),
        "last_nav_goal_status": nav_status,
        "nav_status": status_text,
        "nav_status_payload": status_payload,
        "nav_goal_match": match,
    }
    if active.get("last_goal_annotation_id") != annotation.get("id"):
        return {
            "action": "ignore",
            "reason": "annotation_not_last_goal",
            "status_payload": status_payload,
            "match": match,
            "event_extra": event_extra,
        }
    if nav_status not in ("sent", "accepted"):
        return {
            "action": "ignore",
            "reason": "nav_goal_not_active",
            "status_payload": status_payload,
            "match": match,
            "event_extra": event_extra,
        }
    if not match.get("matches"):
        return {
            "action": "ignore",
            "reason": str(match.get("reason") or "nav_goal_mismatch"),
            "status_payload": status_payload,
            "match": match,
            "event_extra": event_extra,
        }
    return {
        "action": "complete",
        "reason": "current_goal_succeeded",
        "status_payload": status_payload,
        "match": match,
        "event_extra": event_extra,
    }
