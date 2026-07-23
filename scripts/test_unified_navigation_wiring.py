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
    test_compatibility_fields_are_plan_projections()
    print("unified navigation wiring tests passed")
