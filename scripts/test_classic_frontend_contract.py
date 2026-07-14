#!/usr/bin/env python3
"""Static contract for the maintained classic frontend shell."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STATIC = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/static"


def main() -> None:
    html = (STATIC / "dashboard.html").read_text(encoding="utf-8")
    css = (STATIC / "dashboard.css").read_text(encoding="utf-8")
    script = (STATIC / "dashboard.js").read_text(encoding="utf-8")
    backend = (
        ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py"
    ).read_text(encoding="utf-8")

    for tab in ("mapping", "marks", "tasks", "detect"):
        assert f'data-tab="{tab}"' in html
        assert f'id="tab-{tab}"' in html
    for removed in (
        'data-tab="live"',
        'data-tab="localize"',
        'data-tab="maps"',
        'data-tab="preflight"',
        'id="tab-live"',
        'id="tab-localize"',
        'id="tab-maps"',
        'id="tab-preflight"',
        "作业前状态",
        'class="mono"',
    ):
        assert removed not in html

    for element_id in (
        "mapStatusBtn",
        "mapStatusPopover",
        "localizationStatusBtn",
        "localizationStatusPopover",
        "recordingStatusBtn",
        "recordingStatusPopover",
        "preflightTopStatus",
        "taskExecutionFlow",
        "yoloStatus",
        "detections",
        "radarInspection",
        "radarResultList",
        "operationFeedbackDialog",
    ):
        assert f'id="{element_id}"' in html

    assert "status-popover" in css
    assert "task-execution-flow" in css
    assert "detection-results" in css
    assert "renderTaskExecutionFlow" in script
    assert "renderYoloWorkspace" in script
    assert "localizationPopoverOpen" in script
    assert 'setStatusPopover("", false)' in script

    assert "DASHBOARD_LITE_DIR" not in backend
    assert 'parsed.path in ("/lite", "/lite/")' in backend
    assert 'extra_headers={"Location": "/"}' in backend
    assert 'self.declare_parameter("auto_preflight_enabled", True)' in backend
    assert 'self.declare_parameter("auto_preflight_interval_s", 300.0)' in backend
    assert "def _tick_auto_preflight" in backend
    assert '"source": "automatic"' in backend

    print("classic frontend contract tests passed")


if __name__ == "__main__":
    main()
