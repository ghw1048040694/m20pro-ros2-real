#!/usr/bin/env python3
"""Static regression contract for coordinated cross-floor runtime wiring."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CLOUD = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge"
NAVIGATION = ROOT / "src/m20pro_navigation/m20pro_navigation"


def main() -> None:
    web = (CLOUD / "web_dashboard_node.py").read_text(encoding="utf-8")
    floor_manager = (NAVIGATION / "floor_manager.py").read_text(encoding="utf-8")
    tcp_bridge = (NAVIGATION / "tcp_bridge_node.py").read_text(encoding="utf-8")

    # Profile metadata is configuration/diagnostics, not a runtime license.
    assert "requires a validated canonical field profile" not in floor_manager
    assert "requires a validated canonical field profile" not in tcp_bridge
    assert 'm20pro_field_profile.py" check' not in (
        ROOT / "scripts" / "local_deploy_to_test_robot.sh"
    ).read_text(encoding="utf-8")

    for topic in (
        "/m20pro/floor_route_config",
        "/m20pro/floor_switch_request",
        "/m20pro/floor_switch_result",
        "/m20pro/set_current_floor",
    ):
        assert topic in floor_manager
        assert topic in web

    for retired_topic in ("/m20pro/stair_perception_mode", "/m20pro/stair_clearance"):
        assert retired_topic not in floor_manager
        assert retired_topic not in web

    assert "DurabilityPolicy.TRANSIENT_LOCAL" in web
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in floor_manager
    assert "self._publish_runtime_floor_config()" in web
    assert "resolve_floor_switch_request(" in web
    assert 'reason="cross_floor_transition"' in web
    assert 'reason="cross_floor_rollback"' in web
    assert "rollback_factory_map" in web
    assert "selected_map_not_observed" in web
    assert "self._floor_switch_task_is_active(task_id)" in web
    assert '"state_uncertain":' in web

    finish_start = floor_manager.index("def _finish_pending_stair_transition")
    finish_end = floor_manager.index("def _request_coordinated_floor_switch", finish_start)
    finish_body = floor_manager[finish_start:finish_end]
    assert "self._request_coordinated_floor_switch(transition)" in finish_body
    assert "self.switch_floor(" not in finish_body
    assert "self.current_floor = target_floor" in floor_manager
    assert 'if bool(result.get("state_uncertain")):' in floor_manager
    assert 'self.current_floor = ""' in floor_manager
    assert "self.pending_floor_switch = None" in floor_manager
    assert "floor_switch_timeout_s" in floor_manager, "coordinated switch must have a bounded wait"
    goal_start = floor_manager.index("def _on_floor_goal")
    goal_end = floor_manager.index("def _on_rviz_floor_goal", goal_start)
    goal_body = floor_manager[goal_start:goal_end]
    assert "stair_execution_retired" in goal_body
    assert "_resolve_next_stair_route" not in goal_body
    assert "stair_clearance" not in floor_manager
    assert "stair_perception" not in floor_manager
    assert "stair_execution_retired" in floor_manager
    assert 'label in ("stair_traverse", "stair_exit")' in floor_manager
    assert "self._cancel_active_nav_goal(reason)" in floor_manager

    print("cross-floor runtime contract tests passed")


if __name__ == "__main__":
    main()
