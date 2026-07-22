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

    assert "<title>M20 Pro ROS 2 跨楼层巡检导航系统</title>" in html
    assert "<strong>M20 Pro</strong><span>跨楼层巡检导航系统</span>" in html

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
        'id="taskPoseTracker"',
        'id="activeTaskSummary"',
    ):
        assert removed not in html

    for element_id in (
        "mapStatusBtn",
        "mapStatusPopover",
        "localizationStatusBtn",
        "localizationStatusPopover",
        "preflightStatusBtn",
        "preflightStatusPopover",
        "closePreflightStatusBtn",
        "preflightSummary",
        "preflightCounts",
        "preflightItems",
        "runPreflightBtn",
        "refreshPreflightBtn",
        "radarStatusBtn",
        "radarStatusPopover",
        "closeRadarStatusBtn",
        "radarStatusSummary",
        "radarManualMode",
        "radarManualScanBtn",
        "refreshRadarStatusBtn",
        "recordingStatusBtn",
        "recordingStatusPopover",
        "taskStatusBtn",
        "taskStatusPopover",
        "closeTaskStatusBtn",
        "preflightTopStatus",
        "taskExecutionFlow",
        "yoloStatus",
        "yoloEnabledToggle",
        "yoloToggleLabel",
        "frontYoloOverlay",
        "detections",
        "radarResultList",
        "operationFeedbackDialog",
        "markPoseSummary",
        "mapEditorBtn",
        "mapEditorToolbar",
        "mapEditorSaveBtn",
        "deleteMapBtn",
        "floorRouteName",
        "floorRouteEntry",
        "floorRouteSourcePlatform",
        "floorRouteTargetPlatform",
        "floorRoutePostExit",
        "floorRoutePreview",
        "saveFloorRouteBtn",
        "reloadFloorRoutesBtn",
        "floorRouteList",
    ):
        assert f'id="{element_id}"' in html

    assert "status-popover" in css
    assert "position: fixed;" in css
    assert "right: 12px;" not in css
    assert 'id="mapStatusBtn" class="map-toolbar-map-button status-action"' in html
    assert '<button id="mapStatusBtn" class="status-chip' not in html
    assert "floor-badge" not in css
    assert ".pill" not in css
    assert ".status-chip > span:not(.dot)" in css
    assert "button.status-chip {\n      font: inherit;" not in css
    assert "button.status-chip {\n      appearance: none;" in css
    assert "task-execution-flow" in css
    assert "detection-results" in css
    assert "renderTaskExecutionFlow" in script
    assert 'for (const key of ["pose", "scan", "path", "local_path", "active_waypoint"])' in script
    assert "function drawScanOverlay" in script
    assert "const usingDraft = localizationDraftActive();" in script
    assert 'return !!state.localizeDraft;' in script
    assert "must not depend on which popover/tab currently owns pointer focus" in script
    assert "function isLocalizationMapInteraction" in script
    assert 'target.closest(".canvas-box")' in script
    assert "if (isLocalizationMapInteraction(target)) return;" in script
    assert "if (!robotFloor) return true;" in script
    assert "confirmed different floor should suppress its red scan overlay" in script
    assert "renderYoloWorkspace" in script
    assert "function drawYoloOverlay" in script
    assert "function setYoloAnnotatedStream" not in script
    assert '"/api/inspection/toggle"' in script
    assert '"/api/inspection/state"' in script
    assert '"/api/inspection/state"' in backend
    assert "const shouldPoll = !document.hidden && cameraViewers.front.active;" in script
    assert "resultBox.hidden = !enabled;" in script
    assert '"/camera/yolo.mjpg"' not in backend
    assert '"/camera/yolo.jpg"' not in backend
    assert 'elif parsed.path == "/api/inspection/toggle":' in backend
    assert "annotated_image" not in backend
    assert "subscribe_annotated_image" not in backend
    assert "localizationPopoverOpen" in script
    assert 'task: "taskStatusPopover"' in script
    assert '$("taskStatusBtn").addEventListener' in script
    assert 'preflight: "preflightStatusPopover"' in script
    assert '$("preflightStatusBtn").addEventListener' in script
    assert 'radar: "radarStatusPopover"' in script
    assert '$("radarStatusBtn").addEventListener' in script
    assert '"/api/radar/status"' in script
    assert '"/api/radar/manual_start"' in script
    assert 'renderRadarInspection(payload.latest || null)' in script
    assert "function positionStatusPopover" in script
    assert "positionOpenStatusPopover" in script
    assert 'document.addEventListener("pointerdown"' in script
    assert 'function preflightTopLabel' in script
    assert "escapeHtml(item.label || item.key)" in script
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
    assert "map-editor-toolbar" in css
    assert "grid-template-columns: repeat(2, minmax(0, 1fr));" in css
    assert '<button class="danger" id="resetTaskSessionBtn"' in html
    assert "清理导航会话" in html
    assert "复位导航状态" not in html
    assert '"/api/maps/edit"' in script
    assert '`/api/maps?id=${encodeURIComponent(mapId)}&cascade=true`' in script
    assert "function setMapEditorActive" in script
    assert "function paintMapEditor" in script
    assert "button.textContent = item.label || item.id;" in script
    assert "button.textContent = `+ ${item.label || item.id}`" not in script
    assert "20260722-floor-chip-1" in html
    assert html.count("20260722-floor-chip-1") == 2
    assert 'id="floor"' not in html
    assert '$("floor")' not in script
    for stale_default in (
        'value="testfield"',
        'value="M20Pro 工地巡检"',
        'value="主楼"',
        'value="日常巡检任务"',
    ):
        assert stale_default not in html
    for element_id in ("recordingPrefix", "projectName", "buildingName", "taskName"):
        element_start = html.index(f'id="{element_id}"')
        element_end = html.index(">", element_start)
        assert 'autocomplete="off"' in html[element_start:element_end]
    assert 'prefix: $("recordingPrefix").value.trim()' in script
    assert 'prefix: $("recordingPrefix").value.trim() || "testfield"' not in script
    assert '$("recordingPrefix").value = "";' in script
    assert '$("taskName").value = "";' in script
    for element_id in (
        "batteryStatusBtn",
        "batteryStatusPopover",
        "oneKeyChargeBtn",
        "teleopStatusBtn",
        "teleopStatusPopover",
        "acquireTeleopBtn",
        "releaseTeleopBtn",
        "teleopPostureBtn",
        "teleopSoftStopBtn",
        "teleopZeroBtn",
        "teleopConfirmDialog",
        "confirmTeleopConfirmBtn",
        "cancelTeleopConfirmBtn",
        "recordingList",
        "refreshRecordingsBtn",
    ):
        assert f'id="{element_id}"' in html
    for endpoint in (
        "/api/teleop/state",
        "/api/teleop/acquire",
        "/api/teleop/command",
        "/api/teleop/release",
        "/api/teleop/emergency_stop",
        "/api/teleop/motion",
        "/api/charge/one_key",
    ):
        assert endpoint in script or endpoint in backend
    assert 'id="teleopEmergencyStopBtn"' not in html
    assert "停止全部运动" not in html
    assert "targetPostureAction" in script
    assert "function startTeleopHeartbeat" in script
    assert "function requestTeleopConfirmation" in script
    assert "确认窗口不会暂停遥控心跳" in script
    assert "state.teleop.releasing = true" in script
    assert "!state.teleop.releasing" in script
    assert "window.addEventListener(\"blur\"" in script
    assert "document.addEventListener(\"visibilitychange\"" in script
    assert "teleop-pad" in css
    assert "function currentMotionState" in script
    assert "if (!motion.fresh) return null;" in script
    assert 'return [1, 6, 8].includes(motion.state) ? "lie" : "stand";' in script
    assert '$("teleopPostureBtn").disabled = !owns || !postureAction;' in script
    assert '>姿态未知</button>' in html
    assert 'self.declare_parameter("motion_state_topic", "/m20pro_tcp_bridge/motion_state")' in backend
    assert 'id="locFloor"' not in html
    assert '$("locFloor")' not in script
    assert "mappingMapNameTouched" in script
    assert "const activeSession = [state.mappingSession, state.latestMappingSession]" in script
    assert "mapping_start_precondition(session)" in backend
    assert "clamp(460px, 34vw, 620px)" in css
    assert "function mappingSessionMatchesDraft" in script
    assert "const currentSession = state.mappingSession;" in script
    assert '"/api/floor_routes"' in script
    assert '"/api/floor_routes/delete"' in script
    assert 'elif parsed.path == "/api/floor_routes":' in backend
    assert 'elif parsed.path == "/api/floor_routes/delete":' in backend
    assert "def _run_floor_switch_transaction" in backend
    assert "floor_switch_request_topic" in backend

    assert "DASHBOARD_LITE_DIR" not in backend
    assert 'parsed.path in ("/lite", "/lite/")' in backend
    assert 'extra_headers={"Location": "/"}' in backend
    assert 'self.declare_parameter("auto_preflight_enabled", True)' in backend
    assert '"scan": scan or None' in backend
    assert 'self.declare_parameter("auto_preflight_interval_s", 300.0)' in backend
    assert "def _tick_auto_preflight" in backend
    assert '"source": "automatic"' in backend
    assert "def _edit_map" in backend
    assert 'elif parsed.path == "/api/maps/edit":' in backend
    assert 'elif parsed.path == "/api/maps":' in backend
    assert "node._delete_map(map_id, cascade=cascade)" in backend
    assert 'plan["settings"]["hidden_builtin_map_ids"]' in backend
    assert '"source": "web_map_editor"' in backend
    assert 'elif parsed.path == "/api/radar/manual_start":' in backend
    assert "def _radar_manual_start" in backend
    for endpoint in (
        "/api/recording/list",
        "/api/recording/download",
        "/api/recording/rename",
        "/api/recording?id=",
    ):
        assert endpoint in script or endpoint in backend
    assert "function renderRecordingList" in script
    assert "function loadRecordingList" in script
    assert "data-recording-download" in script
    assert "data-recording-rename" in script
    assert "data-recording-delete" in script
    assert "def _recording_list_payload" in backend
    assert "def _rename_recording" in backend
    assert "def _delete_recording" in backend
    assert "def _send_recording_download" in backend
    assert 'tarfile.open(fileobj=handler.wfile, mode="w|gz")' in backend

    print("classic frontend contract tests passed")


if __name__ == "__main__":
    main()
