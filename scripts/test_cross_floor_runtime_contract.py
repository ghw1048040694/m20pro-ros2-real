#!/usr/bin/env python3
"""Static regression contract for coordinated cross-floor runtime wiring."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CLOUD = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge"
NAVIGATION = ROOT / "src/m20pro_navigation/m20pro_navigation"


def main() -> None:
    web = (CLOUD / "web_dashboard_node.py").read_text(encoding="utf-8")
    floor_manager = (NAVIGATION / "floor_manager.py").read_text(encoding="utf-8")

    for topic in (
        "/m20pro/floor_route_config",
        "/m20pro/floor_switch_request",
        "/m20pro/floor_switch_result",
        "/m20pro/set_current_floor",
        "/m20pro/stair_perception_mode",
        "/m20pro/stair_clearance",
    ):
        assert topic in floor_manager
        if topic in (
            "/m20pro/floor_route_config",
            "/m20pro/floor_switch_request",
            "/m20pro/floor_switch_result",
            "/m20pro/set_current_floor",
        ):
            assert topic in web

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
    assert "_start_stair_clearance_session(self.pending_stair_transition)" in floor_manager
    assert "_prepare_stair_exit_clearance()" in floor_manager
    assert "_deactivate_stair_perception()" in floor_manager
    assert '"profile_hash": session.get("profile_hash")' in floor_manager
    assert "profile_hash=self.field_profile_hash" in floor_manager
    assert 'session["phase"] = "platform"' in floor_manager
    assert 'label in ("stair_traverse", "stair_exit")' in floor_manager
    assert "self._cancel_active_nav_goal(reason)" in floor_manager

    print("cross-floor runtime contract tests passed")


if __name__ == "__main__":
    main()
