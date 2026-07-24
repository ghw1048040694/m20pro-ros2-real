#!/usr/bin/env python3
"""Static guardrails for the unified task-plan integration."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WEB_SOURCE = (
    ROOT
    / "src"
    / "m20pro_cloud_bridge"
    / "m20pro_cloud_bridge"
    / "web_dashboard_node.py"
).read_text(encoding="utf-8")


def test_task_creation_builds_unified_plan() -> None:
    assert "def _build_unified_navigation_plan(" in WEB_SOURCE
    assert "unified_plan = self._build_unified_navigation_plan(" in WEB_SOURCE
    assert "task[\"navigation_plan\"] = navigation_plan_record(unified_plan)" in WEB_SOURCE


def test_connector_plan_carries_terrain_identity() -> None:
    contract = (
        ROOT
        / "src"
        / "m20pro_cloud_bridge"
        / "m20pro_cloud_bridge"
        / "unified_navigation_contract.py"
    ).read_text(encoding="utf-8")
    assert "connector_terrain_guard_profile" in contract
    assert '"terrain_guard": connector_terrain_guard_profile(route)' in contract
    assert '"terrain_guard": connector_terrain_guard_profile(' in contract


def test_terrain_guard_remains_shadow_observation_only() -> None:
    assert '"terrain_guard_status_topic"' in WEB_SOURCE
    assert "def _on_terrain_guard_status(" in WEB_SOURCE
    assert "terrain_guard_status=terrain_status" not in WEB_SOURCE
    assert "terrain_guard_timeout_s=" not in WEB_SOURCE


def test_cross_floor_dispatch_uses_connector_gate() -> None:
    assert "_resolve_active_connector_transition" in WEB_SOURCE
    assert "_publish_stair_connector_start" in WEB_SOURCE
    assert "connector_route_activation_decision(" in WEB_SOURCE
    assert "connector_runtime_readiness(" in WEB_SOURCE
    assert '"stair_executor_start_topic"' in WEB_SOURCE
    assert "mark_connector_started_state" in WEB_SOURCE
    assert '"navigation_task_plan_stale"' in WEB_SOURCE
    assert "current_record != stored_plan" in WEB_SOURCE
    assert "connector_owns_navigation_status(active)" in WEB_SOURCE
    assert "_stop_task_if_connector_unresponsive" in WEB_SOURCE
    assert 'self._settings["floor_switch_map_epoch"] = reserved_epoch' in WEB_SOURCE
    assert '"map_epoch": int(map_epoch)' in WEB_SOURCE


def test_stair_executor_is_the_single_connector_runtime_owner() -> None:
    source = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "stair_executor_contract.py"
    ).read_text(encoding="utf-8")
    assert "def create_connector_execution(" in source
    assert "def step_connector_execution(" in source
    assert "cmd_vel_pub" not in source
    assert "from geometry_msgs" not in source
    assert "request_floor_switch" in source
    assert "def connector_motion_decision(" in source
    assert '"source_platform"' in source
    assert '"start_connector_motion"' in source
    node = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "stair_executor_node.py"
    ).read_text(encoding="utf-8")
    assert 'self.declare_parameter("enabled", False)' in node
    assert 'self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")' in node
    assert 'self.declare_parameter("gait_command_topic", "/m20pro/gait_command")' in node
    assert 'self.declare_parameter("localization_ok_topic", "/m20pro_tcp_bridge/localization_ok")' in node
    assert "connector_motion_decision(" in node
    assert "Twist(" in node
    assert "def _on_tick(" in node
    assert '"component": "stair_executor"' in node
    assert '"ready": self._enabled' in node
    setup = (ROOT / "src" / "m20pro_navigation" / "setup.py").read_text(encoding="utf-8")
    assert '"stair_executor = m20pro_navigation.stair_executor_node:main"' in setup
    assert "stair_action_orchestrator" not in setup
    assert not (
        ROOT / "src" / "m20pro_navigation" / "m20pro_navigation" / "stair_action_orchestrator_node.py"
    ).exists()


def test_real_move_mode_starts_connector_with_the_existing_axis_chain() -> None:
    real_launch = (
        ROOT / "src" / "m20pro_bringup" / "launch" / "m20pro_real.launch.py"
    ).read_text(encoding="utf-8")
    top_launch = (
        ROOT / "src" / "m20pro_bringup" / "launch" / "m20pro.launch.py"
    ).read_text(encoding="utf-8")
    start = (
        ROOT / "src" / "m20pro_bringup" / "scripts" / "m20pro_real_full.sh"
    ).read_text(encoding="utf-8")
    assert 'LaunchConfiguration("enable_stair_connector")' in real_launch
    assert 'executable="stair_executor"' in real_launch
    assert 'executable="stair_action_orchestrator"' not in real_launch
    assert "**stair_executor_parameters" in real_launch
    assert '"enable_stair_connector": enable_stair_connector' in top_launch
    assert 'enable_stair_connector:="${AXIS_ENABLED}"' in start


def test_floor_goal_early_errors_keep_protocol_label() -> None:
    floor_manager = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "floor_manager.py"
    ).read_text(encoding="utf-8")
    for reason in (
        "no_current_floor_for_goal",
        "unknown_goal_floor",
        "ordinary_map_floor_mismatch",
        "stair_execution_retired",
    ):
        marker = 'reason=%s' % reason
        start = floor_manager.index(marker)
        assert "label=floor_goal" in floor_manager[start : start + 180]


def test_compatibility_fields_are_plan_projections() -> None:
    marker = 'task["navigation_plan"] = navigation_plan_record(unified_plan)'
    start = WEB_SOURCE.index(marker)
    block = WEB_SOURCE[start : start + 700]
    assert 'task["floor_sequence"] = list(unified_plan.get("floor_sequence") or [])' in block
    assert 'task["route_plans"] = list(unified_plan.get("transition_paths") or [])' in block
    assert 'task["multi_floor"] = not bool(unified_plan.get("single_floor"))' in block


def test_task_start_revalidates_or_migrates_plan() -> None:
    assert "def _task_navigation_plan_state(" in WEB_SOURCE
    assert "task_plan_state = self._task_navigation_plan_state(task, known)" in WEB_SOURCE
    assert '"navigation_plan": record' in WEB_SOURCE
    assert '"task_plan": task_plan_state' in WEB_SOURCE


def test_runtime_plan_failure_stops_task() -> None:
    marker = 'if pre_dispatch.get("action") == "fail":'
    start = WEB_SOURCE.index(marker)
    block = WEB_SOURCE[start : start + 1200]
    assert 'plan_code.startswith("navigation_plan_")' in block
    assert "self._fail_active_task(" in block


if __name__ == "__main__":
    test_task_creation_builds_unified_plan()
    test_connector_plan_carries_terrain_identity()
    test_terrain_guard_remains_shadow_observation_only()
    test_stair_executor_is_the_single_connector_runtime_owner()
    test_cross_floor_dispatch_uses_connector_gate()
    test_real_move_mode_starts_connector_with_the_existing_axis_chain()
    test_floor_goal_early_errors_keep_protocol_label()
    test_compatibility_fields_are_plan_projections()
    print("unified navigation wiring tests passed")
