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
        'id="mapFloorBadge"',
        'id="mapMode"',
        'id="manualPointType"',
        'id="markXY"',
        'id="markYaw"',
        'id="markManner"',
        'id="markObsMode"',
        'id="markNavMode"',
        'id="mappingActiveFloor"',
        'id="mappingActiveFloorRow"',
        "作业前状态",
        "尚未建立建图任务",
        "当前地图还没有任务；",
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
        "markPoseSummary",
    ):
        assert f'id="{element_id}"' in html

    assert "status-popover" in css
    assert "floor-badge" not in css
    assert ".pill" not in css
    assert ".status-chip > span:not(.dot)" in css
    assert "button.status-chip {\n      font: inherit;" not in css
    assert "button.status-chip {\n      appearance: none;" in css
    assert "task-execution-flow" in css
    assert "detection-results" in css
    assert "renderTaskExecutionFlow" in script
    assert "renderYoloWorkspace" in script
    assert "localizationPopoverOpen" in script
    assert "mapModeLabel" not in script
    assert "renderTaskNextStep" not in script
    assert 'setStatusPopover("", false)' in script
    point_type_order = [
        '<option value="patrol">任务点</option>',
        '<option value="transition">过渡点</option>',
        '<option value="charge">充电点</option>',
        '<option value="stair_entry">爬楼梯点</option>',
        '<option value="stair_exit">出楼梯点</option>',
        '<option value="stair_switch">楼层切换点</option>',
    ]
    assert all(item in html for item in point_type_order)
    assert [html.index(item) for item in point_type_order] == sorted(html.index(item) for item in point_type_order)
    assert '<option value="14">爬楼梯（14）</option>' in html
    assert "renderMarkPoseSummary" in script
    assert "syncMappingActiveFloorOptions" not in script
    assert 'active_floor: $("mappingActiveFloor")' not in script
    assert 'id="createSessionBtn"' not in html
    assert 'id="checkMappingEnvBtn"' not in html
    assert 'id="startMappingBtn"' in html
    assert 'id="finishMappingBtn"' in html
    assert 'async function ensureMappingSession()' in script
    assert '"/api/mapping/check_environment"' in script
    assert 'await ensureMappingSession();' in script
    assert "state.latestMappingSession = payload.latest_mapping_session || null;" in script
    assert "const session = state.mappingSession;" in script
    assert "workspace && workspace.latest_mapping_session" not in script
    assert "(session.floor_steps || [])" in script
    assert "mapping-actions" in css
    assert "grid-template-columns: repeat(2, minmax(0, 1fr));" in css
    assert '<button class="danger" id="resetTaskSessionBtn"' in html
    assert "清理导航会话" in html
    assert "复位导航状态" not in html

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
