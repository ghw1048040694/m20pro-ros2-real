#!/usr/bin/env python3
"""Static regression contract for the minimal cross-floor runtime."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CLOUD = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge"
NAVIGATION = ROOT / "src/m20pro_navigation/m20pro_navigation"


def main() -> None:
    web = (CLOUD / "web_dashboard_node.py").read_text(encoding="utf-8")
    transaction = (CLOUD / "floor_switch_transaction_contract.py").read_text(
        encoding="utf-8"
    )
    executor = (NAVIGATION / "stair_executor_node.py").read_text(encoding="utf-8")
    floor_manager = (NAVIGATION / "floor_manager.py").read_text(encoding="utf-8")

    for topic in (
        "/m20pro/floor_switch_request",
        "/m20pro/floor_switch_result",
        "/m20pro/set_current_floor",
    ):
        assert topic in web
    assert "resolve_floor_switch_request(" in web
    assert "begin_transaction(" in web
    assert "completion_decision(" in web
    assert "_persist_floor_switch_transaction" in web
    assert "_activate_cross_floor_target_map" in web
    assert "ThreadPoolExecutor(max_workers=2)" in web
    assert "verify_observed=False" in web
    assert "require_lifecycle=False" in web
    assert "stability_window_s=0.0" in web
    assert "floor_message.data = target_floor" in web

    for retired in (
        "commit_decision(",
        "_rollback_floor_switch_transaction",
        "recover_uncertain_transaction",
        "mark_uncertain_transaction",
        'parsed.path == "/api/floor_switch/recover"',
        "source_map_digest",
        "target_map_digest",
    ):
        assert retired not in web
    for retired in (
        "UNCERTAIN",
        "ROLLING_BACK",
        "source_map_digest",
        "target_map_digest",
        "content_digest",
    ):
        assert retired not in transaction

    assert '"SWITCHING_MAP"' in transaction
    assert '"RELOCALIZING"' in transaction
    assert '"COMMITTED"' in transaction
    assert '"FAILED"' in transaction
    assert "factory_pose_accepted" in transaction
    assert "nav2_load_map" in transaction
    assert "factory_apply_map" in transaction

    assert 'self.declare_parameter("current_floor_topic", "/m20pro/current_floor")' in executor
    assert "self._current_floor == target_floor" in executor
    assert "floor_context_sync_timeout" in executor
    assert "self._publish_floor_goal(action, stage=\"exit\")" in executor
    assert 'self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")' in executor

    # floor_manager remains the same-floor Nav2 gateway.  The connector owns
    # the map transition and only sends entry/exit goals after floor context
    # has changed, so direct cross-floor goals remain rejected.
    assert "stair_execution_retired" in floor_manager
    assert 'label in ("stair_traverse", "stair_exit")' in floor_manager

    print("cross-floor runtime contract tests passed")


if __name__ == "__main__":
    main()
