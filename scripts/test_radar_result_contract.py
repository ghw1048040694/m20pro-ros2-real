#!/usr/bin/env python3
"""Offline tests for generic radar result search."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/m20pro_cloud_bridge"))

from m20pro_cloud_bridge.radar_result_contract import (  # noqa: E402
    radar_job_matches_query,
    radar_job_search_values,
)


def assert_true(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def sample_job() -> dict:
    return {
        "task_id": "task_F20",
        "taskId": "u360_001",
        "scan_mode": "measuring",
        "scan_label": "实测实量",
        "waypoint": {"floor": "F20", "room": "2008", "label": "客厅扫描点"},
        "summary": {
            "statusText": "finished",
            "metrics": [
                {"measurementItem": "地面平整度", "displayValue": "2.5 mm", "qualified": True},
                {"name": "顶板水平度极差", "value": 4.2, "unit": "mm", "conclusion": "合格"},
            ],
        },
    }


def test_dynamic_metric_fields_are_searchable() -> None:
    job = sample_job()
    assert_true(radar_job_matches_query(job, "平整度 2.5"), "known metric name and value")
    assert_true(radar_job_matches_query(job, "顶板 极差 合格"), "provider-specific metric fields")
    assert_true(radar_job_matches_query(job, "F20 2008"), "waypoint location")
    assert_true(not radar_job_matches_query(job, "墙面"), "unmatched query")


def test_query_is_case_insensitive_and_bounded_to_public_result() -> None:
    job = sample_job()
    job["raw_result"] = {"secretRawOnly": "not-indexed"}
    assert_true(radar_job_matches_query(job, "MEASURING"), "case-insensitive mode")
    assert_true(not radar_job_matches_query(job, "secretRawOnly"), "raw device payload is not indexed")
    assert_true(bool(radar_job_search_values(job)), "search values returned")


def main() -> int:
    for test in (
        test_dynamic_metric_fields_are_searchable,
        test_query_is_case_insensitive_and_bounded_to_public_result,
    ):
        test()
        print(f"[OK] {test.__name__}")
    print("[OK] radar result contract tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
