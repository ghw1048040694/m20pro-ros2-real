"""Pure contract for adapting stair-executor actions to existing interfaces.

The stair reducer deliberately emits semantic actions instead of ROS side
effects.  This module is the only translation boundary for those actions.  It
can produce safe interface intents for Nav2/floor-switch/stop, while gait and
connector motion remain semantic intents until a field-certified motion
adapter exists.  No action in this module publishes velocity or factory
commands.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


SAFE_ACTIONS = {
    "request_terrain_guard",
    "release_terrain_guard",
    "dispatch_entry_goal",
    "dispatch_exit_goal",
    "request_floor_switch",
    "stop",
}
SEMANTIC_ACTIONS = {
    "set_gait",
    "start_connector_motion",
    "resume_flat_navigation",
}
KNOWN_ACTIONS = SAFE_ACTIONS | SEMANTIC_ACTIONS


def _text(value: Any) -> str:
    return str(value or "").strip()


def _finite(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _positive_epoch(value: Any) -> Optional[int]:
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if result > 0 else None


def _identity(envelope: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "request_id": _text(envelope.get("request_id")),
        "route_id": _text(envelope.get("route_id")),
        "plan_id": _text(envelope.get("plan_id")),
        "map_epoch": envelope.get("map_epoch"),
    }


def _identity_matches(actual: Dict[str, Any], expected: Dict[str, Any]) -> bool:
    for key in ("request_id", "route_id", "plan_id"):
        expected_value = _text(expected.get(key))
        if expected_value and _text(actual.get(key)) != expected_value:
            return False
    if expected.get("map_epoch") is not None:
        try:
            return int(actual.get("map_epoch")) == int(expected.get("map_epoch"))
        except (TypeError, ValueError):
            return False
    return True


def _pose(value: Any) -> Optional[Dict[str, float]]:
    if not isinstance(value, dict):
        return None
    values = {key: _finite(value.get(key)) for key in ("x", "y", "yaw")}
    if any(item is None for item in values.values()):
        return None
    z = _finite(value.get("z"))
    if z is None:
        z = 0.0
    return {"x": values["x"], "y": values["y"], "z": z, "yaw": values["yaw"]}


def _error(code: str, message: str, **extra: Any) -> Dict[str, Any]:
    return {"ok": False, "code": code, "message": message, **extra, "commands": []}


def _goal_command(
    action: Dict[str, Any],
    *,
    label: str,
    floor: str,
    map_id: str,
) -> Dict[str, Any]:
    pose = _pose(action.get("pose"))
    if not pose:
        return _error(
            "stair_action_pose_invalid",
            "%s 动作缺少有限的 x/y/yaw 坐标" % label,
            action=action,
        )
    if not floor or not map_id:
        return _error(
            "stair_action_map_identity_missing",
            "%s 动作缺少楼层或地图身份" % label,
            action=action,
        )
    return {
        "ok": True,
        "kind": "publish_floor_goal",
        "topic": "/m20pro/floor_goal",
        "label": label,
        "floor": floor,
        "map_id": map_id,
        "pose": pose,
    }


def _validate_actions(actions: Any) -> Optional[Dict[str, Any]]:
    if not isinstance(actions, list):
        return _error("stair_action_list_invalid", "楼梯执行动作不是数组")
    for action in actions:
        if not isinstance(action, dict) or not _text(action.get("kind")):
            return _error("stair_action_invalid", "楼梯执行动作不是带 kind 的对象")
        if _text(action.get("kind")) not in KNOWN_ACTIONS:
            return _error(
                "stair_action_unknown",
                "楼梯执行器产生了未注册动作",
                kind=_text(action.get("kind")),
            )
    return None


def _terrain_guard_command(
    action: Dict[str, Any],
    *,
    identity: Dict[str, Any],
    enabled: bool,
) -> Dict[str, Any]:
    payload: Dict[str, Any] = {
        "enabled": bool(enabled),
        "request_id": identity["request_id"],
        "route_id": identity["route_id"],
        "plan_id": identity["plan_id"],
        "map_epoch": identity["map_epoch"],
    }
    if not enabled:
        return {
            "ok": True,
            "kind": "publish_terrain_guard_request",
            "topic": "/m20pro/terrain_guard/request",
            "payload": payload,
        }
    if (
        _text(action.get("request_id")) != identity["request_id"]
        or _text(action.get("route_id")) != identity["route_id"]
    ):
        return _error(
            "stair_action_terrain_identity_mismatch",
            "terrain_guard 请求与楼梯执行身份不一致",
        )
    profile_id = _text(action.get("profile_id"))
    corridor_version = _text(action.get("corridor_version"))
    corridor = action.get("corridor")
    if not profile_id or not corridor_version or not isinstance(corridor, dict):
        return _error(
            "stair_action_terrain_profile_missing",
            "terrain_guard 请求缺少 profile、走廊版本或走廊几何",
        )
    payload.update(
        {
            "profile_id": profile_id,
            "corridor_version": corridor_version,
            "direction": _text(action.get("direction")) or "forward",
            "corridor": dict(corridor),
        }
    )
    return {
        "ok": True,
        "kind": "publish_terrain_guard_request",
        "topic": "/m20pro/terrain_guard/request",
        "payload": payload,
    }


def translate_action_envelope(
    envelope: Any,
    *,
    expected_identity: Optional[Dict[str, Any]] = None,
    last_sequence: int = 0,
) -> Dict[str, Any]:
    """Translate one action envelope without performing any side effect.

    Returned ``commands`` are a closed set consumed by the ROS adapter.  A
    semantic motion action produces an ``intent`` command only; it never turns
    into ``cmd_vel``, a gait vendor request, or a factory control call here.
    """
    if not isinstance(envelope, dict):
        return _error("stair_action_envelope_invalid", "楼梯执行动作信封不是对象")
    if _text(envelope.get("source")) != "m20pro_stair_executor":
        return _error("stair_action_source_invalid", "楼梯执行动作来源不是 stair_executor")
    identity = _identity(envelope)
    if (
        not identity["request_id"]
        or not identity["route_id"]
        or not identity["plan_id"]
        or _positive_epoch(identity["map_epoch"]) is None
    ):
        return _error(
            "stair_action_identity_missing",
            "楼梯执行动作缺少 request_id、route_id、plan_id 或有效 map_epoch",
        )
    if expected_identity and not _identity_matches(identity, expected_identity):
        return _error(
            "stair_action_identity_mismatch",
            "忽略不属于当前楼梯执行的动作",
            identity=identity,
            expected_identity=dict(expected_identity),
        )
    try:
        sequence = int(envelope.get("sequence"))
    except (TypeError, ValueError):
        return _error("stair_action_sequence_invalid", "楼梯执行动作缺少有效 sequence")
    if sequence <= 0:
        return _error("stair_action_sequence_invalid", "楼梯执行动作 sequence 必须为正整数")
    if sequence <= int(last_sequence):
        return {
            "ok": True,
            "code": "stair_action_stale_ignored",
            "message": "忽略重复或迟到的楼梯执行动作",
            "identity": identity,
            "sequence": sequence,
            "commands": [],
            "ignored": True,
        }
    invalid = _validate_actions(envelope.get("actions"))
    if invalid:
        return invalid

    source_floor = _text(envelope.get("source_floor"))
    target_floor = _text(envelope.get("target_floor"))
    source_map_id = _text(envelope.get("source_map_id"))
    target_map_id = _text(envelope.get("target_map_id"))
    commands: List[Dict[str, Any]] = []
    semantic_intents: List[Dict[str, Any]] = []
    for action in envelope.get("actions") or []:
        kind = _text(action.get("kind"))
        if kind in {"request_terrain_guard", "release_terrain_guard"}:
            command = _terrain_guard_command(
                action,
                identity=identity,
                enabled=kind == "request_terrain_guard",
            )
            if not command.get("ok"):
                return command
            commands.append(command)
            continue
        if kind == "dispatch_entry_goal":
            command = _goal_command(
                action,
                label="stair_entry",
                floor=source_floor,
                map_id=source_map_id or _text(action.get("map_id")),
            )
            if not command.get("ok"):
                return command
            commands.append(command)
            continue
        if kind == "dispatch_exit_goal":
            command = _goal_command(
                action,
                label="stair_exit",
                floor=target_floor,
                map_id=target_map_id or _text(action.get("map_id")),
            )
            if not command.get("ok"):
                return command
            commands.append(command)
            continue
        if kind == "request_floor_switch":
            request_id = _text(action.get("request_id"))
            if request_id != identity["request_id"]:
                return _error(
                    "stair_action_switch_identity_mismatch",
                    "切层动作 request_id 与楼梯执行身份不一致",
                )
            switch_source = _text(action.get("source_floor"))
            switch_target = _text(action.get("target_floor"))
            switch_map = _text(action.get("target_map_id"))
            if (
                not switch_source
                or not switch_target
                or not switch_map
                or (source_floor and switch_source != source_floor)
                or (target_floor and switch_target != target_floor)
                or (target_map_id and switch_map != target_map_id)
            ):
                return _error(
                    "stair_action_switch_identity_mismatch",
                    "切层动作的源/目标楼层或地图与楼梯执行身份不一致",
                )
            commands.append(
                {
                    "kind": "publish_floor_switch_request",
                    "topic": "/m20pro/floor_switch_request",
                    "payload": {
                        "request_id": identity["request_id"],
                        "route_id": identity["route_id"],
                        "plan_id": identity["plan_id"] or None,
                        "map_epoch": identity["map_epoch"],
                        "source_floor": switch_source,
                        "target_floor": switch_target,
                        "target_map_id": switch_map,
                    },
                }
            )
            continue
        if kind == "stop":
            reason = _text(action.get("reason")) or "stair_executor_stop"
            commands.append(
                {
                    "kind": "publish_stop_task",
                    "topic": "/m20pro/stop_task",
                    "reason": reason,
                }
            )
            continue
        # These are deliberately intent-only. Preserve reducer order so a
        # future certified adapter cannot reorder gait/release/resume stages.
        intent = {"kind": kind, **{key: value for key, value in action.items() if key != "kind"}}
        semantic_intents.append(intent)
        commands.append(
            {
                "kind": "publish_semantic_intent",
                "topic": "/m20pro/stair_executor/intent",
                "intents": [intent],
                "dispatchable": False,
            }
        )
    return {
        "ok": True,
        "code": "stair_actions_translated",
        "message": "楼梯语义动作已映射到现有接口；运动动作仍保持意图态",
        "identity": identity,
        "sequence": sequence,
        "commands": commands,
        "semantic_only": bool(semantic_intents),
    }


def event_for_stair_status(
    status_text: Any,
    *,
    identity: Dict[str, Any],
    expected_nav_label: Optional[str],
    expected_stage: Optional[str],
    expected_goal_seq: Optional[int] = None,
) -> Optional[Dict[str, Any]]:
    """Convert only matching Nav2 success/failure statuses into reducer events."""
    text = _text(status_text)
    if not text or not identity.get("request_id"):
        return None
    fields = {}
    for token in text.replace(",", " ").split():
        key, separator, value = token.partition("=")
        if separator and key:
            fields[key.strip()] = value.strip()
    label = _text(fields.get("label"))
    if expected_nav_label and label != expected_nav_label:
        return None
    raw_goal_seq = fields.get("goal_seq")
    try:
        goal_seq = int(raw_goal_seq) if raw_goal_seq is not None else None
    except (TypeError, ValueError):
        goal_seq = None
    if expected_goal_seq is not None and goal_seq != int(expected_goal_seq):
        return None
    if text.startswith("nav_goal_succeeded"):
        if expected_goal_seq is None or goal_seq is None:
            return None
        event_type = (
            "entry_reached"
            if expected_stage == "stair_entry"
            else "exit_reached"
            if expected_stage == "stair_exit"
            else ""
        )
        if event_type:
            return {
                "type": event_type,
                "request_id": identity["request_id"],
                "route_id": identity.get("route_id"),
                "plan_id": identity.get("plan_id"),
                "map_epoch": identity.get("map_epoch"),
                "goal_seq": goal_seq,
            }
    if text.startswith("nav_goal_failed") or text.startswith("error "):
        return {
            "type": "stop_requested",
            "request_id": identity["request_id"],
            "route_id": identity.get("route_id"),
            "plan_id": identity.get("plan_id"),
            "map_epoch": identity.get("map_epoch"),
            "reason": "stair_nav_status_failed",
        }
    return None


def event_for_floor_switch_result(
    result: Any,
    *,
    identity: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    """Bind a floor-switch result to the active connector request."""
    if not isinstance(result, dict):
        return None
    if any(
        _text(result.get(key)) != _text(identity.get(key))
        for key in ("request_id", "route_id", "plan_id")
    ):
        return None
    if (
        _positive_epoch(result.get("map_epoch")) is None
        or _positive_epoch(result.get("map_epoch"))
        != _positive_epoch(identity.get("map_epoch"))
    ):
        return None
    event = {
        "type": "floor_switch_result",
        "request_id": identity["request_id"],
        "route_id": identity.get("route_id"),
        "plan_id": identity.get("plan_id"),
        "map_epoch": identity.get("map_epoch"),
        "ok": bool(result.get("ok")),
    }
    for key in ("target_floor", "target_map_id", "post_exit_pose", "code", "message"):
        if key in result:
            event[key] = result[key]
    return event


__all__ = [
    "event_for_floor_switch_result",
    "event_for_stair_status",
    "translate_action_envelope",
]
