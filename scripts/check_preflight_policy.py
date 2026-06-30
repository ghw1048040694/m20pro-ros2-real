#!/usr/bin/env python3
"""Offline guardrails for M20Pro preflight and assist-mode safety.

This is intentionally lightweight: the web dashboard is a ROS node, so importing
it in CI or on a laptop without ROS dependencies is brittle.  These checks guard
the concrete invariants that have repeatedly broken field testing.
"""

from __future__ import annotations

import re
import sys
import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py"
DASHBOARD = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/static/dashboard.html"
DASHBOARD_CSS = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/static/dashboard.css"
DASHBOARD_JS = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/static/dashboard.js"
ANNOTATION_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/annotation_contract.py"
TASK_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/task_contract.py"
LOCALIZATION_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/localization_contract.py"
ACTIVE_TASK_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/active_task_contract.py"
TASK_SNAPSHOT_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/task_snapshot_contract.py"
NAV_STATUS_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/nav_status_contract.py"
NAVIGATION_READINESS_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/navigation_readiness_contract.py"
TASK_PLAN_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/task_plan_contract.py"
TASK_PROGRESS_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/task_progress_contract.py"
PERCEPTION_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/perception_contract.py"
PREFLIGHT_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/preflight_contract.py"
MAP_DERIVED_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/map_derived_contract.py"
MAP_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/map_contract.py"
MAP_SELECTION_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/map_selection_contract.py"
MAPPING_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/mapping_contract.py"
STARTUP_MAP_SYNC_CONTRACT = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/startup_map_sync_contract.py"
SINGLE_FLOOR_ARCH_DOC = ROOT / "docs/single_floor_navigation_architecture.md"
REAL_LAUNCH = ROOT / "src/m20pro_bringup/launch/m20pro_real.launch.py"
WRAPPER_LAUNCH = ROOT / "src/m20pro_bringup/launch/m20pro.launch.py"
WEB_DASHBOARD_LAUNCH = ROOT / "src/m20pro_bringup/launch/m20pro_web_dashboard.launch.py"
NAV2_PARAMS = ROOT / "src/m20pro_bringup/config/nav2_params_real.yaml"
REAL_CONFIG = ROOT / "src/m20pro_bringup/config/m20pro_real.yaml"
SIM_CONFIG = ROOT / "src/m20pro_bringup/config/m20pro.yaml"
TCP_BRIDGE = ROOT / "src/m20pro_navigation/m20pro_navigation/tcp_bridge_node.py"
SETUP = ROOT / "src/m20pro_navigation/setup.py"
CLOUD_BRIDGE_SETUP = ROOT / "src/m20pro_cloud_bridge/setup.py"
CLOUD_BRIDGE_MANIFEST = ROOT / "src/m20pro_cloud_bridge/MANIFEST.in"
CLOUD_BRIDGE_PACKAGE = ROOT / "src/m20pro_cloud_bridge/package.xml"
PCD_DERIVED = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/pcd_derived.py"
NAV_PACKAGE = ROOT / "src/m20pro_navigation/package.xml"
POINTCLOUD_FUSION = ROOT / "src/m20pro_navigation/m20pro_navigation/pointcloud_fusion.py"
LIDAR_RELAY_NODE = ROOT / "src/m20pro_navigation/m20pro_navigation/lidar_relay_node.py"
FLOOR_MANAGER = ROOT / "src/m20pro_navigation/m20pro_navigation/floor_manager.py"
REAL_FULL = ROOT / "src/m20pro_bringup/scripts/m20pro_real_full.sh"
DIAGNOSE = ROOT / "scripts/104_diagnose_preflight.sh"
TERMINAL_PREFLIGHT = ROOT / "scripts/104_preflight_check.sh"
RELAY_GUARD = ROOT / "src/m20pro_bringup/scripts/m20pro_lidar_relay_guard.sh"
AUTOSTART = ROOT / "scripts/104_enable_autostart.sh"
AUTOSTART_ENTRYPOINT = ROOT / "scripts/104_autostart_entrypoint.sh"
AUTOSTART_UNIT = ROOT / "systemd/m20pro-real.service"
AUTOSTART_DEFAULT = ROOT / "systemd/m20pro-real.default"
TASK_WATCHER = ROOT / "scripts/104_watch_frontend_task.sh"
TASK_WATCH_ANALYZER = ROOT / "scripts/104_analyze_frontend_task_watch.py"
FRONTEND_TASK_SMOKE = ROOT / "scripts/104_frontend_task_smoke.py"
FRONTEND_TASK_READY_CHECK = ROOT / "scripts/104_frontend_task_ready_check.py"
GOAL_MODE_BATTERY_GATE = ROOT / "scripts/104_goal_mode_battery_gate.py"
ANNOTATION_CONTRACT_TEST = ROOT / "scripts/test_annotation_contract.py"
TASK_CONTRACT_TEST = ROOT / "scripts/test_task_contract.py"
LOCALIZATION_CONTRACT_TEST = ROOT / "scripts/test_localization_contract.py"
ACTIVE_TASK_CONTRACT_TEST = ROOT / "scripts/test_active_task_contract.py"
TASK_SNAPSHOT_CONTRACT_TEST = ROOT / "scripts/test_task_snapshot_contract.py"
NAV_STATUS_CONTRACT_TEST = ROOT / "scripts/test_nav_status_contract.py"
NAVIGATION_READINESS_CONTRACT_TEST = ROOT / "scripts/test_navigation_readiness_contract.py"
TASK_PLAN_CONTRACT_TEST = ROOT / "scripts/test_task_plan_contract.py"
TASK_PROGRESS_CONTRACT_TEST = ROOT / "scripts/test_task_progress_contract.py"
PERCEPTION_CONTRACT_TEST = ROOT / "scripts/test_perception_contract.py"
PREFLIGHT_CONTRACT_TEST = ROOT / "scripts/test_preflight_contract.py"
MAP_DERIVED_CONTRACT_TEST = ROOT / "scripts/test_map_derived_contract.py"
MAP_CONTRACT_TEST = ROOT / "scripts/test_map_contract.py"
MAP_SELECTION_CONTRACT_TEST = ROOT / "scripts/test_map_selection_contract.py"
MAPPING_CONTRACT_TEST = ROOT / "scripts/test_mapping_contract.py"
STARTUP_MAP_SYNC_CONTRACT_TEST = ROOT / "scripts/test_startup_map_sync_contract.py"
PCD_DERIVED_TEST = ROOT / "scripts/test_pcd_derived.py"


def fail(message: str) -> None:
    print(f"[FAIL] {message}")
    raise SystemExit(1)


def ok(message: str) -> None:
    print(f"[OK] {message}")


def read(path: Path) -> str:
    if not path.exists():
        fail(f"missing file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def require(text: str, pattern: str, message: str, *, flags: int = 0) -> None:
    if re.search(pattern, text, flags) is None:
        fail(message)
    ok(message)


def forbid(text: str, pattern: str, message: str, *, flags: int = 0) -> None:
    if re.search(pattern, text, flags) is not None:
        fail(message)
    ok(message)


def require_imports(text: str, module: str, names: list, message: str) -> None:
    try:
        tree = ast.parse(text)
    except SyntaxError as exc:
        fail(f"{message}: cannot parse Python source: {exc}")
    imported = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        if node.module != module:
            continue
        imported.update(alias.name for alias in node.names)
    missing = [name for name in names if name not in imported]
    if missing:
        fail(f"{message}: missing {', '.join(missing)}")
    ok(message)


def main() -> int:
    web = read(WEB)
    dashboard_html = read(DASHBOARD)
    dashboard_css = read(DASHBOARD_CSS)
    dashboard_js = read(DASHBOARD_JS)
    dashboard = "\n".join((dashboard_html, dashboard_css, dashboard_js))
    annotation_contract = read(ANNOTATION_CONTRACT)
    task_contract = read(TASK_CONTRACT)
    localization_contract = read(LOCALIZATION_CONTRACT)
    active_task_contract = read(ACTIVE_TASK_CONTRACT)
    task_snapshot_contract = read(TASK_SNAPSHOT_CONTRACT)
    nav_status_contract = read(NAV_STATUS_CONTRACT)
    navigation_readiness_contract = read(NAVIGATION_READINESS_CONTRACT)
    task_plan_contract = read(TASK_PLAN_CONTRACT)
    task_progress_contract = read(TASK_PROGRESS_CONTRACT)
    perception_contract = read(PERCEPTION_CONTRACT)
    preflight_contract = read(PREFLIGHT_CONTRACT)
    map_derived_contract = read(MAP_DERIVED_CONTRACT)
    map_contract = read(MAP_CONTRACT)
    map_selection_contract = read(MAP_SELECTION_CONTRACT)
    mapping_contract = read(MAPPING_CONTRACT)
    startup_map_sync_contract = read(STARTUP_MAP_SYNC_CONTRACT)
    single_floor_arch_doc = read(SINGLE_FLOOR_ARCH_DOC)
    launch = read(REAL_LAUNCH)
    wrapper_launch = read(WRAPPER_LAUNCH)
    web_dashboard_launch = read(WEB_DASHBOARD_LAUNCH)
    nav2 = read(NAV2_PARAMS)
    real_config = read(REAL_CONFIG)
    sim_config = read(SIM_CONFIG)
    tcp = read(TCP_BRIDGE)
    setup = read(SETUP)
    cloud_bridge_setup = read(CLOUD_BRIDGE_SETUP)
    cloud_bridge_manifest = read(CLOUD_BRIDGE_MANIFEST)
    cloud_bridge_package = read(CLOUD_BRIDGE_PACKAGE)
    pcd_derived = read(PCD_DERIVED)
    nav_package = read(NAV_PACKAGE)
    fusion = read(POINTCLOUD_FUSION)
    lidar_relay_node = read(LIDAR_RELAY_NODE)
    real_full = read(REAL_FULL)
    diagnose = read(DIAGNOSE)
    terminal_preflight = read(TERMINAL_PREFLIGHT)
    relay_guard = read(RELAY_GUARD)
    autostart = read(AUTOSTART)
    autostart_entrypoint = read(AUTOSTART_ENTRYPOINT)
    autostart_unit = read(AUTOSTART_UNIT)
    autostart_default = read(AUTOSTART_DEFAULT)
    task_watcher = read(TASK_WATCHER)
    task_watch_analyzer = read(TASK_WATCH_ANALYZER)
    frontend_task_smoke = read(FRONTEND_TASK_SMOKE)
    frontend_task_ready_check = read(FRONTEND_TASK_READY_CHECK)
    goal_mode_battery_gate = read(GOAL_MODE_BATTERY_GATE)
    annotation_contract_test = read(ANNOTATION_CONTRACT_TEST)
    task_contract_test = read(TASK_CONTRACT_TEST)
    localization_contract_test = read(LOCALIZATION_CONTRACT_TEST)
    active_task_contract_test = read(ACTIVE_TASK_CONTRACT_TEST)
    task_snapshot_contract_test = read(TASK_SNAPSHOT_CONTRACT_TEST)
    nav_status_contract_test = read(NAV_STATUS_CONTRACT_TEST)
    navigation_readiness_contract_test = read(NAVIGATION_READINESS_CONTRACT_TEST)
    task_plan_contract_test = read(TASK_PLAN_CONTRACT_TEST)
    task_progress_contract_test = read(TASK_PROGRESS_CONTRACT_TEST)
    perception_contract_test = read(PERCEPTION_CONTRACT_TEST)
    preflight_contract_test = read(PREFLIGHT_CONTRACT_TEST)
    map_derived_contract_test = read(MAP_DERIVED_CONTRACT_TEST)
    map_contract_test = read(MAP_CONTRACT_TEST)
    map_selection_contract_test = read(MAP_SELECTION_CONTRACT_TEST)
    mapping_contract_test = read(MAPPING_CONTRACT_TEST)
    startup_map_sync_contract_test = read(STARTUP_MAP_SYNC_CONTRACT_TEST)
    pcd_derived_test = read(PCD_DERIVED_TEST)

    require(
        web,
        r'declare_parameter\("preflight_settle_wait_s",\s*6\.0\)',
        "web preflight keeps a short baseline settle wait",
    )
    require(
        web,
        r'declare_parameter\("task_start_require_battery_ok",\s*True\)[\s\S]{0,600}declare_parameter\("task_start_min_battery_level",\s*25\)',
        "web task start requires a fresh battery level above the task threshold",
    )
    require(
        web,
        r'declare_parameter\("task_start_require_scan_ok",\s*True\)[\s\S]{0,800}declare_parameter\("task_start_require_lidar_points_ok",\s*True\)',
        "web task start requires fresh scan and lidar pointcloud evidence",
    )
    require(
        web,
        r'declare_parameter\("task_start_warn_first_waypoint_distance_m",\s*8\.0\)[\s\S]{0,180}declare_parameter\("task_start_max_first_waypoint_distance_m",\s*25\.0\)',
        "web task start records warning and hard limits for first-waypoint distance",
    )
    require(
        task_contract,
        r'def task_start_runtime_readiness_payload[\s\S]{0,4200}first_waypoint_distance_m[\s\S]{0,1200}first_waypoint_too_far',
        "task contract blocks obviously far first waypoints after relocalization",
    )
    require(
        localization_contract,
        r'def localization_status_payload[\s\S]{0,1800}factory_localization_ok[\s\S]{0,1000}task_ready[\s\S]{0,2600}"confirmed"',
        "localization contract owns confirmed-vs-task-ready status",
    )
    require(
        tcp,
        r'declare_parameter\("vendor_position_scale",\s*1\.0\)',
        "TCP bridge treats vendor map pose coordinates as meters by default",
    )
    require(
        real_config,
        r'vendor_position_scale:\s*1\.0',
        "real config keeps vendor map pose coordinates in meters",
    )
    require(
        sim_config,
        r'vendor_position_scale:\s*1\.0',
        "sim config keeps vendor map pose coordinates in meters",
    )
    forbid(
        real_config + "\n" + sim_config + "\n" + tcp,
        r'vendor_position_scale:\s*0\.001|declare_parameter\("vendor_position_scale",\s*0\.001\)',
        "vendor map pose scale must not shrink meter coordinates by 1000x",
    )
    require(
        localization_contract,
        r'def pose_tcp_2101_consistency_payload[\s\S]{0,1800}pose_near_2101[\s\S]{0,900}pose_error_m',
        "localization contract checks live map pose against the latest 2101 success pose",
    )
    require(
        localization_contract,
        r'def localization_status_payload[\s\S]{0,3200}pose_tcp_2101_consistency_payload[\s\S]{0,3600}pose_not_near_tcp_2101',
        "localization status rejects live map pose and 2101 pose mismatch",
    )
    require(
        localization_contract,
        r'def parse_initialpose_request[\s\S]{0,900}initialpose_pose_invalid[\s\S]{0,900}frame_id[\s\S]{0,700}floor',
        "localization contract owns initialpose request parsing",
    )
    require(
        localization_contract,
        r'def relocalization_sample_evidence[\s\S]{0,2600}tcp_2101_accepted[\s\S]{0,1000}pose_near_request[\s\S]{0,1000}scan_ok[\s\S]{0,1600}ready_to_finish_wait',
        "localization contract owns per-sample relocalization evidence checks",
    )
    require(
        localization_contract,
        r'def relocalization_response_payload[\s\S]{0,1800}factory_pose_accepted[\s\S]{0,1600}localized_task_not_ready',
        "localization contract separates initialpose publish from confirmed task readiness",
    )
    require(
        localization_contract,
        r'def initialpose_api_response_payload[\s\S]{0,2200}"confirmed"[\s\S]{0,900}"task_ready"[\s\S]{0,900}"verification"',
        "localization contract owns the complete initialpose API response payload",
    )
    require(
        localization_contract,
        r'def manual_relocalization_verification_payload[\s\S]{0,2600}Type=2101[\s\S]{0,1800}"tcp_2101_required": True[\s\S]{0,500}"tcp_2101_diagnostic_only": False',
        "localization contract treats developer-manual TCP 2101/1 as required relocalization evidence",
    )
    require(
        localization_contract,
        r'def map_relocalization_clearance_payload[\s\S]{0,5200}startup_map_relocalization_lock_clearable[\s\S]{0,2400}tcp_2101_accepted[\s\S]{0,2600}factory_localization_not_confirmed[\s\S]{0,1200}pose_missing_or_stale[\s\S]{0,900}map_relocalization_lock_clearable',
        "localization contract can clear stale fixed-map relocalization locks from current 2101/factory/pose evidence",
    )
    require(
        web,
        r'snapshot\["localization_status"\]\s*=\s*localization_status_payload\(',
        "web state snapshot exposes localization_status from the localization contract",
    )
    require(
        web,
        r'localization_status_payload\([\s\S]{0,900}relocalization_result=snapshot\.get\("relocalization_result"\)[\s\S]{0,500}map_relocalization_required=snapshot\.get\("map_relocalization_required"\)',
        "web state snapshot passes recent 2101 evidence and fixed-map relocalization lock into localization_status",
    )
    require(
        web,
        r'def _snapshot[\s\S]{0,4200}_should_check_navigation_readiness\(runtime_state=snapshot\)[\s\S]{0,900}_current_task_readiness_payload\([\s\S]{0,500}runtime_state=snapshot',
        "web state snapshot computes localization and task readiness from the same runtime snapshot",
    )
    require(
        web,
        r'def _snapshot[\s\S]{0,3600}_map_relocalization_clearance_payload\([\s\S]{0,900}_clear_map_relocalization_required\(clearance\)[\s\S]{0,700}snapshot\["map_relocalization_required"\]\s*=\s*None',
        "web state snapshot auto-clears stale fixed-map relocalization locks before task readiness is computed",
    )
    require(
        web,
        r'def _snapshot[\s\S]{0,3400}_current_task_readiness_payload\([\s\S]{0,400}runtime_state=snapshot[\s\S]{0,200}now=now',
        "web state snapshot reuses the same timestamp for localization status and task readiness",
    )
    forbid(
        tcp,
        r'def _publish_navigation_status[\s\S]{0,1800}self\.loc_pub\.publish',
        "TCP bridge navigation status polling does not override localization_ok",
    )
    require(
        web,
        r'def _wait_for_relocalization_verification[\s\S]{0,3600}manual_relocalization_verification_payload\([\s\S]{0,1200}tcp_2101_accepted=bool\(evidence\.get\("tcp_2101_accepted"\)\)',
        "web initialpose API verifies relocalization through the developer-manual TCP 2101/1 contract",
    )
    require(
        web,
        r'def _wait_for_relocalization_verification[\s\S]{0,2200}relocalization_sample_evidence\([\s\S]{0,1400}ready_to_finish_wait[\s\S]{0,2200}manual_relocalization_verification_payload',
        "web initialpose API delegates per-sample relocalization evidence checks",
    )
    require(
        web,
        r'def _publish_initialpose[\s\S]{0,900}parse_initialpose_request\(payload\)[\s\S]{0,900}PoseWithCovarianceStamped',
        "web initialpose API delegates request parsing before ROS publish",
    )
    require(
        web,
        r'def _publish_initialpose[\s\S]{0,700}parse_initialpose_request\(payload\)[\s\S]{0,400}request\["message"\][\s\S]{0,250}request\.items\(\)',
        "web initialpose API consumes required localization-contract parse errors",
    )
    forbid(
        web,
        r'def _publish_initialpose[\s\S]{0,900}request\.get\("message"\)[\s\S]{0,180}重定位坐标无效',
        "web initialpose API no longer hardcodes localization parse fallback messages",
    )
    require(
        web,
        r'status\s*=\s*relocalization_response_payload\([\s\S]{0,1400}initialpose_api_response_payload\(',
        "web initialpose API builds localization status before delegating response assembly",
    )
    require(
        web,
        r'initialpose_api_response_payload\([\s\S]{0,900}localization_status=status[\s\S]{0,900}verification=verification[\s\S]{0,900}task_readiness=task_readiness',
        "web initialpose API delegates response assembly to the localization contract",
    )
    require(
        dashboard_js,
        r'function renderLocalizationStatus[\s\S]{0,2600}重定位成功[\s\S]{0,900}重定位失败[\s\S]{0,1200}已收到回执[\s\S]{0,1200}距2101[\s\S]{0,1200}固定地图[\s\S]{0,400}任务页',
        "frontend renders one final relocalization verdict with 2101 reply only as evidence",
    )
    forbid(
        dashboard_js,
        r'const finalSuccess\s*=\s*confirmed\s*&&\s*taskReady|payload\.confirmed\s*&&\s*payload\.task_ready\s*\?\s*"重定位成功"',
        "frontend relocalization success must not depend on task-page readiness",
    )
    require(
        dashboard_js,
        r'function localizationConfirmedForDisplay[\s\S]{0,900}confirmed === true[\s\S]{0,900}pose_near_2101[\s\S]{0,900}map_relocalization_required',
        "frontend has a localization-confirmed predicate separate from task readiness",
    )
    require(
        dashboard_js,
        r'if \(payload\.confirmed\)[\s\S]{0,180}state\.localizeDraft = null[\s\S]{0,400}fetchJson\("/api/state"\)',
        "frontend clears the red relocalization draft after confirmed relocalization",
    )
    require(
        dashboard_js,
        r'function drawLocalizeDraft\(\)[\s\S]{0,240}localizationConfirmedForDisplay\(\)[\s\S]{0,240}drawArrow',
        "frontend hides the red relocalization draft after confirmed localization",
    )
    require(
        dashboard_js,
        r'const usingDraft = activeTabName\(\) === "localize" && state\.localizeDraft && !localizationConfirmedForDisplay\(\)',
        "frontend uses red scan preview only before confirmed localization",
    )
    require(
        dashboard_js,
        r'pose_invalid_or_stale[\s\S]{0,160}localizationConfirmedForDisplay\(\)[\s\S]{0,260}等待地图位姿刷新',
        "frontend does not tell operators to relocalize when localization is confirmed but pose freshness is waiting",
    )
    require(
        dashboard_js,
        r'function renderLocalizationStatus[\s\S]{0,900}relocalization_result[\s\S]{0,900}tcp_2101_result[\s\S]{0,1400}task_readiness[\s\S]{0,500}map_relocalization_required[\s\S]{0,900}factory_localization_ok[\s\S]{0,500}pose_fresh',
        "frontend fills localization evidence from legacy /api/state fields while backend restart is pending",
    )
    forbid(
        dashboard_js,
        r'function renderLocalizationStatus[\s\S]{0,2600}手册2101成功',
        "frontend localization status must not call a raw 2101 reply relocalization success",
    )
    require(
        dashboard_js,
        r'2101原始回执[\s\S]{0,500}定位结论[\s\S]{0,500}任务页',
        "frontend labels raw relocalization messages as 2101 replies instead of final success",
    )
    require(
        localization_contract_test,
        r'test_parse_initialpose_request[\s\S]{0,1600}blank z invalid message[\s\S]{0,1600}frame is trimmed[\s\S]{0,900}infinite invalid message',
        "offline localization contract tests cover initialpose request parsing",
    )
    require(
        localization_contract_test,
        r'test_relocalization_sample_evidence[\s\S]{0,1800}fresh success 2101 is accepted[\s\S]{0,1800}yaw wraps across pi boundary[\s\S]{0,1800}stale 2101 result ignored',
        "offline localization contract tests cover per-sample relocalization evidence",
    )
    require(
        localization_contract_test,
        r'no_2101\s*=\s*relocalization_sample_evidence[\s\S]{0,1000}blank 2101 is not accepted[\s\S]{0,300}wait does not finish before 2101 success',
        "offline localization contract tests keep relocalization verification waiting for 2101 success",
    )
    require(
        localization_contract_test,
        r'test_localization_status_rejects_scaled_tcp_pose_mismatch[\s\S]{0,1200}pose_not_near_tcp_2101[\s\S]{0,900}meter-scale pose matches 2101',
        "offline localization contract tests reject 1000x scaled live pose mismatches",
    )
    require(
        localization_contract_test,
        r'test_map_relocalization_clearance_uses_strong_current_evidence[\s\S]{0,2200}fresh 2101 plus factory pose clears manual map lock[\s\S]{0,3200}pose_not_near_tcp_2101[\s\S]{0,3200}startup sync lock clears from current factory pose evidence[\s\S]{0,1600}manual map switch still requires 2101 success',
        "offline localization contract tests cover fixed-map relocalization lock auto-clear boundaries",
    )
    require(
        localization_contract_test,
        r'test_localization_status_explains_success_reply_but_map_lock[\s\S]{0,1800}partial success is reported as final failure[\s\S]{0,900}map lock blocker is explained',
        "offline localization contract tests explain 2101 success when fixed-map relocalization lock remains",
    )
    require(
        localization_contract_test,
        r'test_relocalization_response_separates_sent_from_confirmed[\s\S]{0,1600}published initialpose is not treated as confirmed[\s\S]{0,1600}localized_task_not_ready',
        "offline localization contract tests cover sent-vs-confirmed and task blockers",
    )
    require(
        frontend_task_smoke,
        r'taskBlockedText[\s\S]{0,1200}task-blocked localization success must use ok class',
        "frontend smoke keeps localization success green when only task readiness is blocked",
    )
    require(
        localization_contract_test,
        r'test_initialpose_api_response_payload[\s\S]{0,1600}confirmed comes from localization status[\s\S]{0,1600}task readiness remains separate',
        "offline localization contract tests cover initialpose API response assembly",
    )
    require(
        localization_contract_test,
        r'test_manual_relocalization_requires_tcp_2101_success[\s\S]{0,1800}tcp_2101_required[\s\S]{0,1800}tcp_2101_diagnostic_only',
        "offline localization contract tests cover required manual TCP 2101 evidence",
    )
    require(
        web,
        r'def _task_runtime_guard_payload[\s\S]{0,1200}task_runtime_min_battery_level[\s\S]{0,1200}task_runtime_require_perception_ok[\s\S]{0,900}require_scan=False[\s\S]{0,300}require_lidar=False[\s\S]{0,900}runtime_guard_readiness_payload\(',
        "web task runtime guard samples battery/perception then delegates the final decision to task contract",
    )
    forbid(
        web,
        r'def _task_runtime_guard_payload[\s\S]{0,1800}任务运行感知检查已关闭|def _task_runtime_guard_payload[\s\S]{0,1800}readiness_success\(',
        "web runtime guard no longer hardcodes disabled-perception readiness payload",
    )
    require(
        web,
        r'def _stop_task_if_runtime_guard_lost[\s\S]{0,1800}runtime_guard_lost_decision[\s\S]{0,2600}_fail_active_task_from_payload',
        "web task runtime guard delegates sustained critical-link loss decisions before stopping tasks",
    )
    require(
        web,
        r'def _stop_task_if_runtime_guard_lost[\s\S]{0,2200}apply_runtime_guard_wait_state',
        "web delegates runtime guard waiting state updates to the task contract",
    )
    require(
        web,
        r'def _stop_task_if_runtime_guard_lost[\s\S]{0,1800}apply_runtime_guard_clear_state',
        "web delegates runtime guard recovered-clear state updates to the task contract",
    )
    require(
        dashboard_html,
        r'<link rel="stylesheet" href="/static/dashboard\.css\?v=[^"]+" />[\s\S]*<script src="/static/dashboard\.js\?v=[^"]+"></script>',
        "frontend HTML loads versioned split CSS and JS assets",
    )
    forbid(
        dashboard_html,
        r'<style>|</style>|<script>\s|</script>\s*[^<]*const ',
        "frontend HTML does not inline CSS or JavaScript",
    )
    require(
        web,
        r'DASHBOARD_STATIC_FILES[\s\S]{0,400}/static/dashboard\.css[\s\S]{0,400}/static/dashboard\.js[\s\S]{0,1200}_load_dashboard_static',
        "web dashboard serves only the known split frontend static assets",
    )
    require(
        cloud_bridge_setup,
        r'static/dashboard\.html[\s\S]{0,180}static/dashboard\.css[\s\S]{0,180}static/dashboard\.js',
        "cloud bridge package data includes split frontend static assets",
    )
    require(
        cloud_bridge_manifest,
        r'recursive-include m20pro_cloud_bridge/static \*\.html \*\.css \*\.js',
        "cloud bridge manifest includes split frontend static assets",
    )
    readme = read(ROOT / "README.md")
    require(
        readme,
        r'docs/single_floor_navigation_architecture\.md',
        "README points operators and new developers at the single-floor architecture document",
    )
    require(
        readme,
        r'src/m20pro_cloud_bridge/m20pro_cloud_bridge/static/',
        "README points developers at the split frontend static files",
    )
    require(
        readme,
        r'src/m20pro_cloud_bridge/m20pro_cloud_bridge/\*_contract\.py',
        "README points developers at the task contract modules",
    )
    require(
        single_floor_arch_doc,
        r'当前阶段只把单层楼导航做扎实[\s\S]{0,1600}真实前端任务里跑通之前，不认为单层导航完成',
        "single-floor architecture doc keeps single-floor proof as the current completion gate",
    )
    require(
        single_floor_arch_doc,
        r'task_contract\.py[\s\S]{0,900}active_task_contract\.py[\s\S]{0,900}nav_status_contract\.py[\s\S]{0,900}task_plan_contract\.py[\s\S]{0,900}task_progress_contract\.py[\s\S]{0,900}task_snapshot_contract\.py',
        "single-floor architecture doc names the main task contract modules",
    )
    require(
        single_floor_arch_doc,
        r'任务执行证据链[\s\S]{0,900}floor_goal_published[\s\S]{0,900}last_floor_goal_published_at[\s\S]{0,900}floor_goal_publish_count[\s\S]{0,900}task_execution_evidence[\s\S]{0,900}floor_goal_published` / `floor_goal_publishes',
        "single-floor architecture doc explains floor-goal publish and frontend snapshot evidence",
    )
    require(
        single_floor_arch_doc,
        r'如果任务页显示导航中，但没有 `floor_goal_published`[\s\S]{0,900}Nav2 没有 accepted/feedback[\s\S]{0,900}path_goal_error_m[\s\S]{0,900}plan_goal_verified=false',
        "single-floor architecture doc explains how to locate task execution breakpoints",
    )
    require(
        single_floor_arch_doc,
        r'A：复杂环境重定位[\s\S]{0,1200}不应该直接改 active task 状态机[\s\S]{0,1200}B：跨楼层逻辑[\s\S]{0,1200}不应该绕过单层任务 contract',
        "single-floor architecture doc defines intern A/B ownership boundaries",
    )
    require(
        single_floor_arch_doc,
        r'能删除的入口不保留[\s\S]{0,600}现场脚本只用于采集证据或启动标准链路，不用于替代核心逻辑',
        "single-floor architecture doc records subtraction over patch-script development",
    )
    require(
        single_floor_arch_doc,
        r'从前端点击开始[\s\S]{0,900}到点后进入 dwell[\s\S]{0,900}dwell 后切到下一个点[\s\S]{0,900}last_result',
        "single-floor architecture doc records the missing real motion evidence",
    )
    require(
        single_floor_arch_doc,
        r'当前阻断状态[\s\S]{0,900}104_goal_mode_battery_gate\.py --url http://10\.21\.31\.104:8080[\s\S]{0,900}低于 `25%`[\s\S]{0,1000}factory_lidar_points_publisher_missing',
        "single-floor architecture doc records the current battery and perception blockers",
    )
    require(
        single_floor_arch_doc,
        r'充电后上车顺序[\s\S]{0,1000}只重启 `m20pro-real\.service`[\s\S]{0,1200}/LIDAR/POINTS -> lidar_relay -> /scan -> perception_status[\s\S]{0,1200}开发手册 2101[\s\S]{0,1200}ready-check 和 watcher',
        "single-floor architecture doc records the post-charge field sequence",
    )
    require(
        dashboard,
        r'\{\s*mode:\s*"move",\s*site:\s*"auto",\s*wait:\s*false\s*\}',
        "frontend starts preflight in auto site mode asynchronously",
    )
    require(
        web,
        r'"m20pro_nav2_startup_gate"',
        "web preflight requires the Nav2 startup gate node",
    )
    require(
        preflight_contract,
        r'def preflight_lifecycle_deferred_item[\s\S]{0,500}"nav2_lifecycle_deferred"[\s\S]{0,300}"info"',
        "unlocalized/workstation Nav2 lifecycle is informational",
    )
    require(
        preflight_contract,
        r'def preflight_costmap_items[\s\S]{0,900}status = "ok" if fresh and size_ok else "info"[\s\S]{0,500}未重定位前 Nav2/costmap[\s\S]{0,900}"local_costmap"',
        "unlocalized/workstation local costmap is deferred info",
    )
    require(
        preflight_contract,
        r'def preflight_costmap_items[\s\S]{0,900}status = "ok" if fresh and size_ok else "info"[\s\S]{0,500}未重定位前 Nav2/costmap[\s\S]{0,1200}"global_costmap"',
        "unlocalized/workstation global costmap is deferred info",
    )
    require(
        preflight_contract,
        r'def preflight_result_payload[\s\S]{0,2200}navigation_failures[\s\S]{0,900}item\.get\("status"\) in \("fail", "warn"\)',
        "preflight contract counts only fail/warn navigation items, not info",
    )
    require(
        preflight_contract,
        r'"navigation_ready":\s*not navigation_failures',
        "preflight contract derives navigation_ready from navigation_failures",
    )
    require(
        preflight_contract,
        r'perception_chain[\s\S]{0,900}relocalization_ready[\s\S]{0,1400}地图/点云/scan 可用',
        "preflight contract owns perception-chain failure and relocalization-ready summary rules",
    )
    require(
        preflight_contract,
        r'def preflight_context[\s\S]{0,700}mode not in \("move", "shadow"\)[\s\S]{0,700}site in \("workstation", "bench", "desk", "office", "charging"\)[\s\S]{0,900}"location=1" in nav_status_text\.lower\(\)[\s\S]{0,900}"defer_nav2_startup_checks": workstation_mode or unlocalized',
        "preflight contract owns mode/site/workstation/deferred preflight context rules",
    )
    require(
        preflight_contract,
        r'def preflight_node_item[\s\S]{0,700}"nodes"[\s\S]{0,500}核心节点[\s\S]{0,500}全部在线[\s\S]{0,500}缺少：',
        "preflight contract owns core-node item status and message rules",
    )
    require(
        preflight_contract,
        r'def preflight_base_topics_item[\s\S]{0,500}"topics"[\s\S]{0,500}基础话题[\s\S]{0,500}缺少：[\s\S]{0,300}"base"',
        "preflight contract owns base-topic item status and message rules",
    )
    require(
        preflight_contract,
        r'def preflight_navigation_topics_item[\s\S]{0,500}"navigation_topics"[\s\S]{0,500}导航话题[\s\S]{0,500}重定位后应出现：[\s\S]{0,300}"navigation"',
        "preflight contract owns navigation-topic item status and message rules",
    )
    require(
        preflight_contract,
        r'def preflight_odom_item[\s\S]{0,600}"odom"[\s\S]{0,500}原厂里程计[\s\S]{0,700}未收到有效 /ODOM',
        "preflight contract owns odom item status and message rules",
    )
    require(
        preflight_contract,
        r'def preflight_navigation_status_item[\s\S]{0,500}"navigation_status"[\s\S]{0,500}原厂导航状态[\s\S]{0,500}暂未收到 navigation_status',
        "preflight contract owns navigation-status item status and message rules",
    )
    require(
        preflight_contract,
        r'def preflight_lifecycle_deferred_item[\s\S]{0,500}"nav2_lifecycle_deferred"[\s\S]{0,500}Nav2 生命周期[\s\S]{0,700}Nav2 可由启动门延后激活',
        "preflight contract owns deferred Nav2 lifecycle item rules",
    )
    require(
        preflight_contract,
        r'def preflight_lifecycle_item[\s\S]{0,500}lifecycle:\{name\}[\s\S]{0,500}\{name\} 生命周期[\s\S]{0,500}"status": "ok" if \(lifecycle or \{\}\)\.get\("active"\) else "warn"',
        "preflight contract owns lifecycle item status rules",
    )
    require(
        preflight_contract,
        r'def preflight_perception_items[\s\S]{0,900}perception_ok[\s\S]{0,900}"lidar_points"[\s\S]{0,600}原始点云[\s\S]{0,900}"scan"[\s\S]{0,600}二维激光',
        "preflight contract owns lidar/scan item status and perception readiness rules",
    )
    require(
        preflight_contract,
        r'def preflight_perception_items[\s\S]{0,900}未直接缓存原始点云[\s\S]{0,900}未收到 /LIDAR/POINTS[\s\S]{0,900}未收到 /scan；未定位',
        "preflight contract owns lidar/scan fallback messages",
    )
    require(
        preflight_contract,
        r'def preflight_costmap_items[\s\S]{0,900}deferred[\s\S]{0,700}"local_costmap"[\s\S]{0,500}局部代价地图[\s\S]{0,900}"global_costmap"[\s\S]{0,500}全局代价地图',
        "preflight contract owns local/global costmap item status rules",
    )
    require(
        preflight_contract,
        r'def preflight_costmap_items[\s\S]{0,900}未重定位前 Nav2/costmap[\s\S]{0,1200}已定位但未收到 local_costmap[\s\S]{0,700}已定位但未收到 global_costmap',
        "preflight contract owns deferred and localized-missing costmap messages",
    )
    require(
        preflight_contract,
        r'def preflight_battery_item[\s\S]{0,900}"battery"[\s\S]{0,900}"status": "ok" if level >= required_level else "fail"[\s\S]{0,600}最低要求',
        "preflight contract owns battery item status and message rules",
    )
    require(
        preflight_contract,
        r'def preflight_map_item[\s\S]{0,700}"map"[\s\S]{0,700}"status": "ok" if map_available else "fail"[\s\S]{0,500}未收到 /map',
        "preflight contract owns map item status and message rules",
    )
    require(
        preflight_contract,
        r'def preflight_map_pose_item[\s\S]{0,400}pose_ok[\s\S]{0,700}未收到有效 /m20pro_tcp_bridge/map_pose[\s\S]{0,500}"map_pose"[\s\S]{0,300}"地图位姿"[\s\S]{0,300}"status": "ok" if pose_ok else "warn"',
        "preflight contract owns map-pose item status and message rules",
    )
    require(
        preflight_contract,
        r'def preflight_localization_item[\s\S]{0,900}"localization"[\s\S]{0,900}"status": "ok" if confirmed else "warn"[\s\S]{0,700}localization_ok=true',
        "preflight contract owns localization item status and message rules",
    )
    require(
        preflight_contract,
        r'def preflight_motion_mode_item[\s\S]{0,900}requested_mode[\s\S]{0,900}"motion_mode"[\s\S]{0,900}"status": "ok" if detected_mode == "move" else "fail"',
        "preflight contract owns motion-mode item status and message rules",
    )
    require_imports(
        web,
        "preflight_contract",
        [
            "preflight_base_topics_item",
            "preflight_battery_item",
            "preflight_context",
            "preflight_costmap_items",
            "preflight_lifecycle_deferred_item",
            "preflight_lifecycle_item",
            "preflight_localization_item",
            "preflight_map_item",
            "preflight_map_pose_item",
            "preflight_motion_mode_item",
            "preflight_navigation_status_item",
            "preflight_navigation_topics_item",
            "preflight_node_item",
            "preflight_odom_item",
            "preflight_perception_items",
            "preflight_result_payload",
        ],
        "web imports the pure preflight summary contract",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,6200}context = preflight_context\([\s\S]{0,500}localization_ok=current_state\.get\("localization_ok"\)[\s\S]{0,500}navigation_status=current_state\.get\("navigation_status"\)',
        "web preflight delegates mode/site/workstation context to the preflight contract",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}explicit_workstation|def _run_preflight_locked[\s\S]{0,13000}auto_site|def _run_preflight_locked[\s\S]{0,13000}site in \("workstation", "bench", "desk", "office", "charging"\)|def _run_preflight_locked[\s\S]{0,13000}"location=1" in nav_status_text\.lower\(\)',
        "web preflight no longer owns mode/site/workstation context rules inline",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,9800}node_names = set\(self\.get_node_names\(\)\)[\s\S]{0,900}items\.append\(preflight_node_item\(list\(node_names\), required_nodes\)\)',
        "web preflight delegates core-node item rules to the preflight contract",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}add\([\s\S]{0,160}"nodes"|def _run_preflight_locked[\s\S]{0,13000}核心节点|def _run_preflight_locked[\s\S]{0,13000}全部在线',
        "web preflight no longer assembles core-node item messages inline",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,10600}items\.append\(preflight_base_topics_item\(list\(topic_names\), base_topics\)\)[\s\S]{0,260}items\.append\(preflight_navigation_topics_item\(list\(topic_names\), navigation_topics\)\)',
        "web preflight delegates base/navigation topic item rules to the preflight contract",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}add\([\s\S]{0,160}"topics"|def _run_preflight_locked[\s\S]{0,13000}add\([\s\S]{0,160}"navigation_topics"|def _run_preflight_locked[\s\S]{0,13000}基础话题|def _run_preflight_locked[\s\S]{0,13000}导航话题|def _run_preflight_locked[\s\S]{0,13000}重定位后应出现：|def _run_preflight_locked[\s\S]{0,13000}缺少：',
        "web preflight no longer assembles base/navigation topic item messages inline",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}def add\(',
        "web preflight no longer keeps a generic inline item assembler",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,12000}scan_ok, scan_age, scan = fresh\("scan"\)[\s\S]{0,900}preflight_perception_items\([\s\S]{0,500}lidar_ok=lidar_ok[\s\S]{0,300}scan_ok=scan_ok[\s\S]{0,500}items\.extend\(perception\["items"\]\)[\s\S]{0,160}perception_ok = bool\(perception\["perception_ok"\]\)',
        "web preflight delegates lidar/scan item and perception readiness rules to the preflight contract",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}add\([\s\S]{0,160}"lidar_points"|def _run_preflight_locked[\s\S]{0,13000}add\([\s\S]{0,160}"scan"|def _run_preflight_locked[\s\S]{0,13000}未直接缓存原始点云|def _run_preflight_locked[\s\S]{0,13000}未收到 /LIDAR/POINTS|def _run_preflight_locked[\s\S]{0,13000}未收到 /scan；未定位',
        "web preflight no longer assembles lidar/scan item messages inline",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,12200}odom_ok, odom_age, odom = fresh\("odom"\)[\s\S]{0,700}preflight_odom_item\([\s\S]{0,260}odom_ok=odom_ok[\s\S]{0,260}odom_finite=odom_finite',
        "web preflight delegates odom item rules to the preflight contract",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}add\([\s\S]{0,160}"odom"|def _run_preflight_locked[\s\S]{0,13000}原厂里程计|def _run_preflight_locked[\s\S]{0,13000}未收到有效 /ODOM',
        "web preflight no longer assembles odom item messages inline",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,12600}local_ok, local_age, local_costmap = fresh\("local_costmap"\)[\s\S]{0,260}global_ok, global_age, global_costmap = fresh\("global_costmap"\)[\s\S]{0,520}preflight_costmap_items\([\s\S]{0,420}local_ok=local_ok[\s\S]{0,260}global_ok=global_ok[\s\S]{0,360}deferred=bool\(context\["defer_nav2_startup_checks"\]\)',
        "web preflight delegates local/global costmap item rules to the preflight contract",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}add\([\s\S]{0,160}"local_costmap"|def _run_preflight_locked[\s\S]{0,13000}add\([\s\S]{0,160}"global_costmap"|def _run_preflight_locked[\s\S]{0,13000}局部代价地图|def _run_preflight_locked[\s\S]{0,13000}全局代价地图|def _run_preflight_locked[\s\S]{0,13000}未重定位前 Nav2/costmap|def _run_preflight_locked[\s\S]{0,13000}已定位但未收到 local_costmap|def _run_preflight_locked[\s\S]{0,13000}已定位但未收到 global_costmap',
        "web preflight no longer assembles local/global costmap item messages inline",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,12400}loc_ok = bool\(context\["localized"\]\)[\s\S]{0,420}items\.append\(preflight_localization_item\(loc_ok\)\)',
        "web preflight delegates localization item rules to the preflight contract",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,12600}nav_status_text = str\(context\["navigation_status_text"\]\)[\s\S]{0,420}items\.append\(preflight_navigation_status_item\(nav_status_text\)\)',
        "web preflight delegates navigation-status item rules to the preflight contract",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}add\([\s\S]{0,160}"navigation_status"|def _run_preflight_locked[\s\S]{0,13000}原厂导航状态|def _run_preflight_locked[\s\S]{0,13000}暂未收到 navigation_status',
        "web preflight no longer assembles navigation-status item messages inline",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}add\([\s\S]{0,160}"localization"|def _run_preflight_locked[\s\S]{0,13000}localization_ok=true|def _run_preflight_locked[\s\S]{0,13000}定位未确认是预期状态',
        "web preflight no longer assembles localization item messages inline",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,11800}map_payload = current_state\.get\("map"\)[\s\S]{0,260}items\.append\(preflight_map_item\(map_payload if isinstance\(map_payload, dict\) else \{\}\)\)',
        "web preflight delegates map item rules to the preflight contract",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}add\("map"|def _run_preflight_locked[\s\S]{0,13000}已加载 /map|def _run_preflight_locked[\s\S]{0,13000}未收到 /map',
        "web preflight no longer assembles map item messages inline",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,12400}pose = current_state\.get\("pose"\)[\s\S]{0,520}items\.append\(\s*preflight_map_pose_item\([\s\S]{0,260}pose_ok=pose_has_stamp[\s\S]{0,260}age_text=fmt_age_text\(pose_age\)',
        "web preflight delegates map-pose item rules to the preflight contract",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}add\([\s\S]{0,160}"map_pose"|def _run_preflight_locked[\s\S]{0,13000}地图位姿|def _run_preflight_locked[\s\S]{0,13000}未收到有效 /m20pro_tcp_bridge/map_pose',
        "web preflight no longer assembles map-pose item messages inline",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,12200}battery = current_state\.get\("battery"\)[\s\S]{0,300}items\.append\(preflight_battery_item\(battery if isinstance\(battery, dict\) else \{\}, min_level=min_level\)\)',
        "web preflight delegates battery item rules to the preflight contract",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}primary = battery\.get\("primary"\)|def _run_preflight_locked[\s\S]{0,13000}最低要求|def _run_preflight_locked[\s\S]{0,13000}未收到电池数据',
        "web preflight no longer assembles battery item messages inline",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,12500}motion = self\._detect_motion_mode\(\)[\s\S]{0,220}items\.append\(preflight_motion_mode_item\(requested_mode=mode, motion=motion\)\)',
        "web preflight delegates motion-mode item rules to the preflight contract",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}if context\["defer_nav2_startup_checks"\]:\s*items\.append\(preflight_lifecycle_deferred_item\(\)\)[\s\S]{0,500}items\.append\(preflight_lifecycle_item\(node_name, lifecycle\)\)',
        "web preflight delegates Nav2 lifecycle item rules to the preflight contract",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}nav2_lifecycle_deferred|def _run_preflight_locked[\s\S]{0,13000}Nav2 生命周期|def _run_preflight_locked[\s\S]{0,13000}Nav2 可由启动门延后激活|def _run_preflight_locked[\s\S]{0,13000}生命周期"',
        "web preflight no longer assembles Nav2 lifecycle item messages inline",
    )
    forbid(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}motion\.get\("message"\)\s*or|def _run_preflight_locked[\s\S]{0,13000}未确认 move 模式|def _run_preflight_locked[\s\S]{0,13000}未确认运动模式',
        "web preflight no longer falls back from motion-mode messages",
    )
    require(
        web,
        r'def _run_preflight_locked[\s\S]{0,13000}preflight_result_payload\(',
        "web preflight delegates final summary/readiness result to preflight contract",
    )
    require(
        preflight_contract_test,
        r'test_ready_field_navigation[\s\S]{0,1400}test_workstation_navigation_deferred[\s\S]{0,1400}test_perception_chain_failure_is_added[\s\S]{0,1600}test_relocalization_can_still_be_ready_with_noncritical_failure[\s\S]{0,1600}test_motion_mode_item[\s\S]{0,1600}test_node_and_topic_items[\s\S]{0,1600}test_odom_navigation_status_and_lifecycle_items[\s\S]{0,1600}test_perception_items[\s\S]{0,1600}test_costmap_items[\s\S]{0,1600}test_battery_item[\s\S]{0,1600}test_map_item[\s\S]{0,1600}test_localization_item[\s\S]{0,1600}test_map_pose_item',
        "offline preflight contract tests cover field, workstation, perception, relocalization-ready, motion-mode, nodes/topics, odom/navigation-status/lifecycle, lidar/scan, costmap, battery, map, localization and map-pose items",
    )

    forbid(dashboard, r'api/usage_mode|data-usage-mode|setUsageMode', "web has no usage-mode control route/button")
    require(real_config, r'enable_usage_mode_command:\s*false', "real config keeps usage-mode command disabled")
    require(sim_config, r'enable_usage_mode_command:\s*false', "sim config keeps usage-mode command disabled")
    require(tcp, r'declare_parameter\("enable_usage_mode_command",\s*False\)', "TCP bridge default disables usage-mode command")

    require(setup, r'nav2_startup_gate\s*=\s*m20pro_navigation\.nav2_startup_gate:main', "Nav2 startup gate is installed")
    require(launch, r'"autostart":\s*"False"', "real Nav2 navigation lifecycle autostart is disabled")
    require(launch, r'executable="nav2_startup_gate"', "real launch starts Nav2 startup gate")
    require(nav2, r'always_send_full_costmap:\s*true', "Nav2 costmaps publish full costmaps for dashboard freshness")
    require(fusion, r'backup_cloud_ranges', "pointcloud fusion stores backup cloud ranges separately")
    require(
        fusion,
        r'np\.minimum\(self\.cloud_ranges,\s*self\.backup_cloud_ranges\)',
        "pointcloud fusion merges primary and backup lidar ranges",
    )
    require(real_full, r'M20PRO_LIDAR2_TOPIC:-/LIDAR/POINTS2', "real full startup knows the optional second lidar input")
    require(
        real_full,
        r'M20PRO_ENABLE_LIDAR2_RELAY:-0',
        "real full startup keeps optional second lidar relay disabled by default",
    )
    require(
        autostart,
        r'M20PRO_ENABLE_LIDAR2_RELAY=\$\{M20PRO_ENABLE_LIDAR2_RELAY:-0\}',
        "autostart default keeps optional second lidar relay disabled",
    )
    require(
        autostart_default,
        r'M20PRO_ENABLE_LIDAR2_RELAY=0',
        "systemd default keeps optional second lidar relay disabled",
    )
    require(
        real_full,
        r'cleanup_stale_fastdds_shm[\s\S]{0,1000}/dev/shm/fastrtps_\*[\s\S]{0,900}kept_open',
        "real full startup removes stale unused FastDDS SHM segments before creating new participants",
    )
    require(
        real_full,
        r'M20PRO_LIDAR_RELAY_FASTDDS_PROFILE:-project_udp',
        "real full startup defaults lidar relay to the bounded project FastDDS profile",
    )
    require(
        autostart_default,
        r'M20PRO_LIDAR_RELAY_FASTDDS_PROFILE=project_udp',
        "systemd default keeps lidar relay on the bounded project FastDDS profile",
    )
    require(
        autostart,
        r'M20PRO_LIDAR_RELAY_FASTDDS_PROFILE=\$\{M20PRO_LIDAR_RELAY_FASTDDS_PROFILE:-project_udp\}',
        "autostart installer writes the same bounded project FastDDS relay default",
    )
    require(
        autostart,
        r'Usage: \./scripts/104_enable_autostart\.sh \[move\|shadow\][\s\S]{0,260}move\|shadow\) ;;',
        "autostart installer accepts only explicit move/shadow modes",
    )
    forbid(
        autostart,
        r'\bsafe\b',
        "autostart installer does not keep the undocumented safe mode alias",
    )
    require(
        autostart_entrypoint,
        r'case "\$\{MODE\}" in[\s\S]{0,80}move\)[\s\S]{0,160}104_start_real_move\.sh[\s\S]{0,120}shadow\)[\s\S]{0,160}104_start_real_shadow\.sh[\s\S]{0,180}expected move or shadow',
        "autostart entrypoint dispatches only explicit move/shadow modes",
    )
    forbid(
        autostart_entrypoint,
        r'\bsafe\b',
        "autostart entrypoint does not keep the undocumented safe mode alias",
    )
    require(
        read(ROOT / "src/m20pro_bringup/config/m20pro_fastdds_udp.xml"),
        r'<transport_id>m20pro_shm_transport</transport_id>[\s\S]{0,120}<type>SHM</type>[\s\S]{0,120}<segment_size>67108864</segment_size>',
        "project FastDDS profile uses bounded 64MB SHM instead of factory 500MB segments",
    )
    require(real_full, r'COMMON_ARGS\+=\(backup_cloud_topic:="\$\{BACKUP_CLOUD_TOPIC\}"\)', "real full startup conditionally passes backup cloud topic")
    require(wrapper_launch, r'DeclareLaunchArgument\("backup_cloud_topic"', "wrapper launch declares backup cloud topic")
    require(wrapper_launch, r'"backup_cloud_topic":\s*backup_cloud_topic', "wrapper launch forwards backup cloud topic")
    require(launch, r'DeclareLaunchArgument\("backup_cloud_topic"', "real launch declares backup cloud topic")
    require(launch, r'"backup_cloud_topic":\s*backup_cloud_topic', "real launch configures pointcloud fusion backup topic")
    require(
        diagnose,
        r"M20PRO_DIAG_REMOTE=0[\s\S]{0,400}bash -s",
        "diagnose script executes the current local script on 104 when run from an upper computer",
    )
    require(
        diagnose,
        r"FASTRTPS_DEFAULT_PROFILES_FILE=\"\$\{PROJECT_FASTDDS\}\"[\s\S]{0,240}project UDP FastDDS",
        "diagnose script uses project UDP FastDDS for ROS CLI pointcloud observation",
    )
    forbid(
        diagnose,
        r"ros2 topic echo",
        "diagnose script avoids ros2 topic echo for samples because Foxy CLI can be misleading here",
    )
    require(
        terminal_preflight,
        r"M20PRO_PREFLIGHT_REMOTE=0[\s\S]{0,400}bash -s",
        "terminal preflight executes the current local script on 104 when run from an upper computer",
    )
    require(
        terminal_preflight,
        r"FASTRTPS_DEFAULT_PROFILES_FILE=\"\$\{PROJECT_FASTDDS\}\"",
        "terminal preflight uses project UDP FastDDS for ROS CLI observation",
    )
    forbid(
        terminal_preflight,
        r"ros2 topic echo",
        "terminal preflight avoids ros2 topic echo for samples because Foxy CLI can be misleading here",
    )
    require(
        terminal_preflight,
        r'fail "\/scan has no data within 8s; scan\/lidar must be present even at the workstation"',
        "terminal preflight treats missing /scan as a hard failure even at the workstation",
    )
    require(
        read(ROOT / "src/m20pro_navigation/m20pro_navigation/floor_manager.py"),
        r'declare_parameter\("flat_gait_label",\s*"flat"\)',
        "floor manager default does not automatically request untested assist gait",
    )
    require(
        launch,
        r'"flat_gait_label":\s*"flat"',
        "real launch does not automatically request untested assist gait",
    )
    require(
        relay_guard,
        r'M20PRO_LIDAR_RELAY_MAX_OUTPUT_POINTS:-6000',
        "lidar relay guard defaults to downsampled pointcloud output",
    )
    require(
        relay_guard,
        r'M20PRO_LIDAR_RELAY_MIN_PUBLISH_INTERVAL_S:-0\.2',
        "lidar relay guard limits pointcloud publish rate",
    )
    require(
        relay_guard,
        r'M20PRO_LIDAR_RELAY_CLOUD_RELIABILITY:-auto[\s\S]{0,2000}cloud_reliability:=',
        "lidar relay defaults to auto input QoS compatibility",
    )
    require(
        lidar_relay_node,
        r'declare_parameter\("cloud_reliability",\s*"auto"\)[\s\S]{0,4200}def _subscription_reliability_modes[\s\S]{0,260}return \("best_effort",\s*"reliable"\)',
        "lidar relay auto mode subscribes with both best-effort and reliable QoS",
    )
    require(
        relay_guard,
        r'relay_args_match_for_pid[\s\S]{0,900}cloud_reliability:=\$\{CLOUD_RELIABILITY\}[\s\S]{0,900}max_output_points:=\$\{MAX_OUTPUT_POINTS\}[\s\S]{0,900}min_publish_interval_s:=\$\{MIN_PUBLISH_INTERVAL_S\}',
        "lidar relay guard restarts old relay processes when downsample parameters change",
    )
    require(
        relay_guard,
        r'case "\$\{1:-start\}"[\s\S]{0,260}\n  start\)[\s\S]{0,120}start_relay[\s\S]{0,260}start-wait\)',
        "lidar relay guard has a non-blocking start mode and keeps start-wait only for diagnostics",
    )
    require(
        real_full,
        r'm20pro_lidar_relay_guard\.sh" start',
        "real full startup starts the lidar relay without blocking web startup on a pointcloud sample",
    )
    forbid(
        real_full,
        r'm20pro_lidar_relay_guard\.sh" start-wait',
        "real full startup does not wait for lidar relay samples before launching the web frontend",
    )
    require(
        readme,
        r'全量 real 启动[\s\S]{0,260}默认不等待 `/LIDAR/POINTS` 样本才启动 Nav2 和网页[\s\S]{0,360}/LIDAR/POINTS -> lidar_relay -> /scan',
        "README documents that real startup keeps the web frontend available while perception readiness is checked before tasks",
    )
    require(
        readme,
        r'自启动[\s\S]{0,260}默认不因 `/LIDAR/POINTS` 暂时无样本而阻塞网页[\s\S]{0,260}感知链路故障',
        "README documents that autostart does not block the web frontend on temporary lidar sample loss",
    )
    forbid(
        readme,
        r'必须在 104 上实际收到 `/LIDAR/POINTS` 的 `PointCloud2` 样本后才继续拉起 Nav2 和网页|自启动同样会先等 `/LIDAR/POINTS` 样本',
        "README must not describe the removed lidar-sample startup block",
    )
    require(
        nav_package,
        r'<exec_depend>python3-numpy</exec_depend>',
        "navigation package declares numpy runtime dependency for vectorized pointcloud work",
    )
    forbid(
        cloud_bridge_package,
        r'<exec_depend>python3-numpy</exec_depend>',
        "cloud bridge no longer depends on numpy after removing frontend 3D PCD generation",
    )
    require(
        pcd_derived,
        r'def process_imported_map[\s\S]{0,2400}STAIR_ZONES_FILE[\s\S]{0,1200}"stair_zones"',
        "PCD postprocess only materializes stair-zone semantics",
    )
    forbid(
        pcd_derived,
        r'import numpy|terrain_mesh|height_grid|_write_stair_pointclouds|_height_grid_stair_candidates|load_pcd_xyz|_build_height_grid',
        "PCD postprocess no longer generates frontend 3D terrain or local stair pointclouds",
    )
    require(
        pcd_derived_test,
        r'test_process_imported_map_generates_only_stair_zones[\s\S]{0,2000}terrain_mesh\.json[\s\S]{0,1000}height_grid\.json[\s\S]{0,1000}"pointcloud" not in zone',
        "offline pcd-derived test covers stair-zone-only generation",
    )
    require_imports(
        web,
        "map_derived_contract",
        [
            "builtin_map_derived_payload",
            "read_json_object",
            "resolve_map_asset_path",
            "should_generate_builtin_stair_zones",
            "stair_zones_available_payload",
            "stair_zones_unavailable_payload",
        ],
        "web imports pure map-derived contract helpers",
    )
    require(
        map_derived_contract,
        r'def builtin_map_derived_payload[\s\S]{0,900}项目内置地图已有楼梯语义区[\s\S]{0,900}项目内置地图可生成楼梯语义区',
        "map-derived contract owns builtin derived status payloads",
    )
    require(
        web,
        r'def _load_builtin_maps[\s\S]{0,900}load_builtin_maps_from_manifest\([\s\S]{0,600}derived_payload=builtin_map_derived_payload',
        "web builtin map loader passes derived payload construction into the map contract",
    )
    forbid(
        web,
        r'def _builtin_map_derived|项目内置地图已有楼梯语义区|项目内置地图可生成楼梯语义区',
        "web no longer owns builtin map-derived payload rules",
    )
    require(
        map_derived_contract,
        r'def resolve_map_asset_path[\s\S]{0,1200}base_dir[\s\S]{0,700}directory',
        "map-derived contract owns derived asset path resolution",
    )
    require(
        web,
        r'def _stair_zones_payload[\s\S]{0,1100}resolve_map_asset_path\(record, zones_rel, path_resolver=self\._resolve_path\)',
        "web stair-zones loader delegates asset path resolution",
    )
    forbid(
        web,
        r'def _resolve_map_asset_path|def _read_json_file',
        "web no longer owns derived asset path or JSON-object helpers",
    )
    require(
        map_derived_contract,
        r'def stair_zones_unavailable_payload[\s\S]{0,900}"available": False[\s\S]{0,500}"zones": \[\]',
        "map-derived contract owns stair-zone availability payloads",
    )
    require(
        map_derived_contract,
        r'def stair_zones_available_payload[\s\S]{0,700}"available"\] = True[\s\S]{0,700}"derived_status"',
        "map-derived contract owns available stair-zone map metadata",
    )
    require(
        web,
        r'def _stair_zones_payload[\s\S]{0,1600}stair_zones_unavailable_payload\(record, "当前地图没有楼梯语义区"\)[\s\S]{0,1600}stair_zones_available_payload\(record, derived, payload\)',
        "web stair-zones loader delegates availability payloads",
    )
    require(
        map_derived_contract_test,
        r'test_builtin_map_derived_payload_pending_and_ready[\s\S]{0,1800}test_resolve_map_asset_path[\s\S]{0,1800}test_stair_zones_payloads',
        "offline map-derived contract tests cover builtin derived status, asset paths and stair-zone payloads",
    )
    require(
        web,
        r'declare_parameter\("enable_stair_zone_postprocess",\s*True\)',
        "web dashboard uses explicit stair-zone postprocess parameter",
    )
    forbid(
        web,
        r'enable_map_pcd_postprocess|pcd_terrain_cell_size',
        "web dashboard removes stale PCD terrain postprocess parameters",
    )
    require(
        launch,
        r'enable_stair_zone_postprocess',
        "real launch forwards stair-zone postprocess parameter",
    )
    require(
        wrapper_launch,
        r'enable_stair_zone_postprocess',
        "wrapper launch forwards stair-zone postprocess parameter",
    )
    forbid(
        "\n".join((launch, wrapper_launch)),
        r'enable_map_pcd_postprocess|pcd_terrain_cell_size',
        "real/wrapper launch remove stale PCD terrain postprocess parameters",
    )
    require(
        lidar_relay_node,
        r'import numpy as np[\s\S]{0,8000}def _sample_data_bytes',
        "lidar relay downsampling uses numpy stride slicing instead of Python per-point copying",
    )
    require(
        lidar_relay_node,
        r'np\.frombuffer\(bounded,\s*dtype=np\.uint8\)\.reshape\(\(-1,\s*point_step\)\)',
        "lidar relay builds a vectorized byte-row view for downsampling",
    )
    require(
        lidar_relay_node,
        r'return rows\[::stride\]\.copy\(\)\.tobytes\(\)',
        "lidar relay samples point rows with numpy stride slicing",
    )
    require(
        lidar_relay_node,
        r'last_downsample_method[\s\S]{0,1200}numpy_stride[\s\S]{0,1200}python_loop',
        "lidar relay reports whether vectorized downsampling or fallback was used",
    )
    require(
        web,
        r'_last_scan_overlay_update = 0\.0[\s\S]{0,120}_last_scan_overlay_points: List\[Dict\[str, float\]\] = \[\]',
        "web scan overlay keeps a throttled frontend update interval",
    )
    require(
        web,
        r'declare_parameter\("scan_overlay_update_min_interval_s",\s*0\.1\)',
        "web scan overlay exposes the frontend update interval as a parameter",
    )
    require(
        web,
        r'def _on_scan[\s\S]{0,700}now - self\._last_scan_overlay_update >= min_interval_s[\s\S]{0,1800}self\._last_scan_overlay_points = points',
        "web scan overlay avoids rebuilding frontend point arrays on every scan frame",
    )
    require(
        dashboard,
        r'id="frontVideoBtn"[\s\S]{0,180}data-video-camera="front"[\s\S]{0,220}id="frontVideo"[\s\S]{0,140}data-src="/camera/front\.mjpg"[\s\S]{0,500}id="rearVideoBtn"[\s\S]{0,180}data-video-camera="rear"[\s\S]{0,220}id="rearVideo"[\s\S]{0,140}data-src="/camera/rear\.mjpg"',
        "frontend keeps on-demand front/rear camera images",
    )
    forbid(
        dashboard,
        r'id="frontVideo"[^>]+\ssrc="/camera/front\.(?:mjpg|jpg)"|id="rearVideo"[^>]+\ssrc="/camera/rear\.(?:mjpg|jpg)"',
        "frontend camera images do not auto-load camera streams",
    )
    require(
        web,
        r'def acquire_client[\s\S]{0,260}self\._client_count \+= 1[\s\S]{0,220}def release_client[\s\S]{0,320}self\._client_count = max\(0, self\._client_count - 1\)',
        "camera proxy tracks active MJPEG viewers",
    )
    require(
        web,
        r'def _serve_mjpeg[\s\S]{0,700}worker\.acquire_client\(\)[\s\S]{0,2200}finally:\s+worker\.release_client\(\)',
        "camera proxy releases MJPEG viewers when browser connections close",
    )
    require(
        web,
        r'"running": thread_alive and \(client_count > 0 or snapshot_lease_active\)[\s\S]{0,260}"snapshot_lease_active": snapshot_lease_active',
        "camera proxy reports running for attached viewers or low-latency snapshot leases",
    )
    require(
        web,
        r'def _run[\s\S]{0,220}if not self\._wait_for_client\(\):[\s\S]{0,2600}cap\.release\(\)[\s\S]{0,160}cap = None',
        "camera proxy releases RTSP capture after the last viewer disconnects",
    )
    require(
        web,
        r'declare_parameter\("camera_proxy_backend",\s*"ffmpeg_mjpeg"\)',
        "camera proxy defaults to the FFmpeg MJPEG backend",
    )
    require(
        web,
        r'declare_parameter\("camera_proxy_fps",\s*10\.0\)[\s\S]{0,160}declare_parameter\("camera_proxy_jpeg_quality",\s*45\)[\s\S]{0,160}declare_parameter\("camera_proxy_ffmpeg_mjpeg_qscale",\s*5\)',
        "camera proxy defaults to the tested 10fps/480/q45 FFmpeg profile",
    )
    require(
        web,
        r'def _run\(self\)[\s\S]{0,180}self\.backend == "ffmpeg_mjpeg"[\s\S]{0,120}_run_ffmpeg_mjpeg\(\)[\s\S]{0,120}_run_opencv\(\)',
        "camera proxy dispatches between FFmpeg MJPEG and OpenCV fallback",
    )
    require(
        web,
        r'def _ffmpeg_mjpeg_command[\s\S]{0,1200}"-analyzeduration"[\s\S]{0,120}"0"[\s\S]{0,220}"-probesize"[\s\S]{0,120}"32"[\s\S]{0,900}"-f"[\s\S]{0,120}"mjpeg"[\s\S]{0,220}"-flush_packets"[\s\S]{0,120}"1"[\s\S]{0,120}"pipe:1"',
        "camera proxy can stream FFmpeg MJPEG directly to the browser proxy",
    )
    require(
        web,
        r'def _read_ffmpeg_mjpeg_frames[\s\S]{0,1200}os\.read\(proc\.stdout\.fileno\(\),\s*8192\)[\s\S]{0,400}_publish_jpeg_frames_from_buffer',
        "camera proxy reads FFmpeg MJPEG frames without OpenCV decoding",
    )
    require(
        web,
        r'"ffmpeg_available":\s*shutil\.which\("ffmpeg"\)[\s\S]{0,180}"backend":\s*self\._camera_proxy_backend\(\)[\s\S]{0,240}"ffmpeg_mjpeg_qscale":',
        "camera proxy state reports FFmpeg availability, selected backend and qscale",
    )
    require(
        web,
        r'def _camera_proxy_backend\(self\)[\s\S]{0,260}backend in \("ffmpeg", "ffmpeg_mjpeg"\)[\s\S]{0,160}return "ffmpeg_mjpeg"[\s\S]{0,120}return "opencv"',
        "camera proxy falls back to OpenCV only when FFmpeg is unavailable or not selected",
    )
    require(
        web,
        r'def _run_opencv[\s\S]{0,2200}if cap is not None and not self\._has_clients\(\):[\s\S]{0,180}cap\.release\(\)[\s\S]{0,100}cap = None',
        "OpenCV fallback releases RTSP capture after the last viewer disconnects",
    )
    require(
        web,
        r'def _run_ffmpeg_mjpeg[\s\S]{0,500}if not self\._wait_for_client\(\):[\s\S]{0,120}break[\s\S]{0,900}finally:[\s\S]{0,180}_terminate_process\(proc\)',
        "FFmpeg backend starts only for active viewers and terminates after disconnect",
    )
    require(
        web,
        r'import socket',
        "camera proxy imports socket controls for low-latency MJPEG",
    )
    require(
        web,
        r'declare_parameter\("camera_proxy_low_latency",\s*True\)[\s\S]{0,260}declare_parameter\("camera_proxy_socket_send_buffer_bytes",\s*65536\)[\s\S]{0,180}declare_parameter\("camera_proxy_snapshot_keepalive_s",\s*1\.5\)',
        "camera proxy exposes low-latency socket and snapshot keepalive settings",
    )
    require(
        web,
        r'def _configure_mjpeg_socket[\s\S]{0,700}TCP_NODELAY[\s\S]{0,900}SO_SNDBUF',
        "camera proxy disables Nagle and limits MJPEG send buffering",
    )
    require(
        web,
        r'def _serve_mjpeg[\s\S]{0,3000}_configure_mjpeg_socket\(handler\)[\s\S]{0,3000}X-Accel-Buffering[\s\S]{0,3000}handler\.wfile\.flush\(\)',
        "camera proxy flushes each MJPEG frame with buffering disabled",
    )
    require(
        dashboard_js,
        r'async function pumpVideoFrames[\s\S]{0,1200}fetch\(`\$\{source\}\$\{separator\}ts=\$\{Date\.now\(\)\}`[\s\S]{0,700}response\.body\.getReader\(\)[\s\S]{0,1800}content-length[\s\S]{0,1400}queueLatestVideoFrame\(img,\s*viewer,\s*token,\s*payload\)',
        "frontend reads low-latency MJPEG with fetch stream parsing and latest-frame-only display",
    )
    require(
        dashboard_js,
        r'function queueLatestVideoFrame[\s\S]{0,260}viewer\.latestPayload = payload[\s\S]{0,360}requestAnimationFrame[\s\S]{0,500}displayVideoFrame\(img,\s*viewer,\s*latestPayload\)',
        "frontend drops queued stale camera frames before browser rendering",
    )
    require(
        web,
        r'Cache-Control",\s*"no-store, no-cache, must-revalidate, max-age=0"[\s\S]{0,140}Pragma",\s*"no-cache"[\s\S]{0,140}Expires",\s*"0"',
        "web static responses use strict no-cache headers for frontend video fixes",
    )
    require(
        web,
        r'def _serve_jpeg_snapshot[\s\S]{0,900}extend_snapshot_lease\(max\(keepalive_s,\s*frame_timeout \+ 0\.5\)\)[\s\S]{0,1200}wait_for_frame\(last_sequence,\s*frame_timeout\)[\s\S]{0,1300}Content-Type",\s*"image/jpeg"[\s\S]{0,700}handler\.wfile\.flush\(\)',
        "camera proxy exposes low-latency JPEG snapshot endpoints",
    )
    require(
        web,
        r'parsed\.path in \("/camera/front\.jpg", "/camera/rear\.jpg"\)[\s\S]{0,220}_serve_jpeg_snapshot',
        "web routes front/rear low-latency JPEG snapshot endpoints",
    )
    launch_bundle = "\n".join((launch, wrapper_launch, web_dashboard_launch))
    require(
        launch_bundle,
        r'DeclareLaunchArgument\("camera_proxy_backend",\s*default_value="ffmpeg_mjpeg"\)',
        "launch files default the camera proxy to FFmpeg MJPEG",
    )
    require(
        launch_bundle,
        r'DeclareLaunchArgument\("camera_proxy_ffmpeg_mjpeg_qscale",\s*default_value="5"\)',
        "launch files declare the FFmpeg MJPEG qscale",
    )
    require(
        launch_bundle,
        r'"camera_proxy_backend":\s*camera_proxy_backend',
        "launch files forward the camera proxy backend",
    )
    require(
        launch_bundle,
        r'"camera_proxy_ffmpeg_mjpeg_qscale":\s*(?:ParameterValue\(\s*)?camera_proxy_ffmpeg_mjpeg_qscale',
        "launch files forward the FFmpeg MJPEG qscale",
    )
    require(
        real_full,
        r'camera_proxy_backend:=ffmpeg_mjpeg[\s\S]{0,160}camera_proxy_fps:=10\.0[\s\S]{0,160}camera_proxy_jpeg_quality:=45[\s\S]{0,160}camera_proxy_ffmpeg_mjpeg_qscale:=5[\s\S]{0,160}camera_proxy_max_width:=480',
        "real full startup requests the tested FFmpeg MJPEG camera profile",
    )
    forbid(
        dashboard,
        r'id="cameraStatus"|data-camera-toggle-all|function renderCameraStatus|function setCameraEnabled',
        "frontend removes camera toggle/status controls",
    )
    require(
        web,
        r'def _task_result_snapshot_unlocked[\s\S]{0,500}runtime_snapshot = self\._task_runtime_snapshot_unlocked\(active\)[\s\S]{0,500}build_task_result_snapshot',
        "web task result snapshot persists runtime evidence for post-failure diagnosis",
    )
    require(
        task_snapshot_contract,
        r'def build_task_result_snapshot[\s\S]{0,1800}"runtime_snapshot": runtime_snapshot',
        "task snapshot contract result payload includes the persisted runtime snapshot",
    )
    require(
        task_snapshot_contract,
        r'RESULT_EXTRA_KEYS\s*=\s*\([\s\S]{0,180}"reason"[\s\S]{0,180}"nav_status"',
        "task result snapshots promote raw Nav2 status from terminal extra diagnostics",
    )
    require(
        task_snapshot_contract_test,
        r'test_result_snapshot[\s\S]{0,1600}"nav_status": "error nav2 action unavailable"[\s\S]{0,900}raw Nav2 status promoted[\s\S]{0,900}raw Nav2 status retained in extra',
        "offline task snapshot tests cover raw Nav2 status retention in task results",
    )
    require(
        task_snapshot_contract,
        r'def build_task_runtime_snapshot',
        "task snapshot contract owns task runtime snapshots",
    )
    require(
        task_snapshot_contract,
        r'def last_task_result_payload[\s\S]{0,1200}last_result[\s\S]{0,900}last_error[\s\S]{0,900}last_event',
        "task snapshot contract owns last task result payload selection",
    )
    require(
        task_snapshot_contract,
        r'def apply_task_result_persistence[\s\S]{0,700}updated\["status"\]\s*=\s*str\(status\)[\s\S]{0,900}last_result[\s\S]{0,900}last_timeline[\s\S]{0,900}last_error[\s\S]{0,900}updated_at',
        "task snapshot contract owns persisted task result and terminal status updates",
    )
    require(
        task_snapshot_contract,
        r'def apply_task_result_to_tasks[\s\S]{0,1200}apply_task_result_persistence\([\s\S]{0,900}"tasks": updated_tasks[\s\S]{0,700}"changed": changed',
        "task snapshot contract owns list-level task result persistence",
    )
    require(
        task_snapshot_contract,
        r'def build_active_waypoint_payload[\s\S]{0,2600}path_goal_error_m[\s\S]{0,2600}nav_feedback_age_s[\s\S]{0,2200}"waypoint": waypoint',
        "task snapshot contract owns live active-waypoint payloads for frontend task diagnostics",
    )
    require(
        task_snapshot_contract,
        r'def build_idle_waypoint_payload[\s\S]{0,500}"phase": "idle"[\s\S]{0,500}"reason": str\(reason or "idle"\)',
        "task snapshot contract owns idle active-waypoint payloads",
    )
    require(
        task_snapshot_contract,
        r'RUNTIME_ACTIVE_KEYS[\s\S]{0,900}"last_nav_feedback"',
        "task snapshot contract runtime snapshot includes Nav2 feedback state",
    )
    require(
        task_snapshot_contract,
        r'RUNTIME_ACTIVE_KEYS[\s\S]{0,900}"plan_goal_verified"',
        "task snapshot contract runtime snapshot includes plan verification state",
    )
    require(
        web,
        r'lidar_relay_status_topic[\s\S]{0,1200}_on_lidar_relay_status',
        "web state subscribes to lidar relay diagnostics",
    )
    require(
        task_snapshot_contract,
        r'"lidar_relay_status":\s*\{[\s\S]{0,900}"downsample_method"[\s\S]{0,900}"input_rate_hz"[\s\S]{0,900}"publish_rate_hz"[\s\S]{0,900}"skip_ratio"',
        "web task snapshots expose lidar relay rate/downsample diagnostics",
    )
    require(
        autostart,
        r'M20PRO_LIDAR_RELAY_MAX_OUTPUT_POINTS=\$\{M20PRO_LIDAR_RELAY_MAX_OUTPUT_POINTS:-6000\}',
        "autostart default records relay pointcloud downsample limit",
    )
    require(
        autostart,
        r'M20PRO_LIDAR_RELAY_MIN_PUBLISH_INTERVAL_S=\$\{M20PRO_LIDAR_RELAY_MIN_PUBLISH_INTERVAL_S:-0\.2\}',
        "autostart default records relay pointcloud publish-rate limit",
    )
    require(
        autostart,
        r'M20PRO_LIDAR_RELAY_CLOUD_RELIABILITY=\$\{M20PRO_LIDAR_RELAY_CLOUD_RELIABILITY:-auto\}',
        "autostart default records relay auto input QoS compatibility",
    )
    require(
        autostart_unit,
        r'WorkingDirectory=/home/user/m20pro_real_ros2_ws[\s\S]{0,120}ExecStart=/home/user/m20pro_real_ros2_ws/scripts/104_autostart_entrypoint\.sh',
        "autostart unit runs from the real workspace instead of the legacy/sim workspace",
    )
    require(
        autostart_unit,
        r'TimeoutStopSec=12',
        "autostart unit uses a bounded stop timeout for faster frontend recovery after restart",
    )
    require(
        autostart_default,
        r'M20PRO_WS=/home/user/m20pro_real_ros2_ws',
        "autostart default points M20PRO_WS at the real workspace",
    )
    require(
        launch,
        r'"max_points_per_cloud":\s*6000',
        "real pointcloud fusion processes a bounded number of points per cloud",
    )
    require(
        launch,
        r'"publish_on_cloud_update":\s*False',
        "real pointcloud fusion publishes scan at timer rate instead of every cloud callback",
    )
    require(
        launch,
        r'"cloud_reliability":\s*"best_effort"',
        "real pointcloud consumers use best-effort sensor QoS for relay streams",
    )
    require(
        launch,
        r'"lidar_points_topic":\s*cloud_topic[\s\S]{0,140}"lidar_points_relay_subscribe_topic":\s*cloud_topic',
        "real web dashboard observes the downsampled relay pointcloud instead of raw lidar",
    )
    require(
        web,
        r'cloud_qos\s*=\s*QoSProfile\(depth=2\)[\s\S]{0,140}cloud_qos\.reliability\s*=\s*ReliabilityPolicy\.BEST_EFFORT',
        "web dashboard subscribes to lidar pointcloud as best-effort sensor data",
    )
    require(
        web,
        r'create_subscription\(PointCloud2,\s*self\._topic\("lidar_points_topic"\),\s*self\._on_lidar_points,\s*cloud_qos\)',
        "web dashboard applies cloud_qos to lidar pointcloud subscription",
    )
    require(
        web,
        r'source\s*=\s*"relay"\s*if\s*self\._lidar_points_topic\s*==\s*self\._lidar_points_relay_subscribe_topic\s*else\s*"raw"',
        "web dashboard labels relay pointcloud source accurately when the primary topic is the relay",
    )
    require(
        web,
        r'relay_qos\s*=\s*QoSProfile\(depth=2\)[\s\S]{0,160}relay_qos\.reliability\s*=\s*ReliabilityPolicy\.BEST_EFFORT',
        "web dashboard internal pointcloud relay publisher also uses best-effort QoS",
    )
    require(
        web,
        r'self\._data_lock\s*=\s*threading\.RLock\(\)',
        "web task state lock is reentrant to avoid dispatcher deadlock",
    )
    require(
        nav_status_contract,
        r'def apply_nav_feedback_state[\s\S]{0,1700}last_nav_feedback[\s\S]{0,1700}last_nav_feedback_monotonic',
        "nav status contract saves Nav2 feedback fields so the frontend can show live task progress",
    )
    require(
        web,
        r'def _update_active_task_from_nav_feedback[\s\S]{0,2200}apply_nav_feedback_state[\s\S]{0,1200}self\._save_json\("settings\.json",\s*self\._settings\)',
        "web persists Nav2 feedback state from the nav status contract",
    )
    floor_manager = read(ROOT / "src/m20pro_navigation/m20pro_navigation/floor_manager.py")
    require(
        floor_manager,
        r'def _goal_status_suffix[\s\S]{0,900}goal_seq=%d[\s\S]{0,900}goal_x=%.3f[\s\S]{0,900}goal_y=%.3f[\s\S]{0,900}goal_yaw=%.4f',
        "floor manager publishes Nav2 goal identity and target pose in status text",
    )
    require(
        nav_status_contract,
        r'def nav_status_matches_active_goal[\s\S]{0,5000}goal_seq_mismatch[\s\S]{0,5000}goal_x[\s\S]{0,5000}goal_y[\s\S]{0,5000}goal_yaw',
        "nav status contract validates Nav2 status against active waypoint id, sequence and pose",
    )
    require(
        nav_status_contract,
        r'def friendly_nav_status[\s\S]{0,1200}Nav2 正在执行当前点位[\s\S]{0,1200}导航链路报错，任务已停止',
        "nav status contract owns operator-facing Nav2 status text",
    )
    require(
        nav_status_contract,
        r'def apply_nav_failure_state[\s\S]{0,900}last_nav_goal_status[\s\S]{0,900}last_error[\s\S]{0,900}status_message',
        "nav status contract owns active-task Nav2 failure state updates",
    )
    require(
        nav_status_contract,
        r'def apply_nav_goal_status_state[\s\S]{0,1600}last_nav_accepted_monotonic[\s\S]{0,1200}last_nav_goal_seq[\s\S]{0,900}status_message',
        "nav status contract owns active-task Nav2 goal-status state updates",
    )
    require(
        nav_status_contract,
        r'def nav_goal_status_event_payload[\s\S]{0,900}nav_status_payload[\s\S]{0,900}nav_goal_match',
        "nav status contract owns Nav2 goal-status timeline payloads",
    )
    require(
        nav_status_contract,
        r'def apply_nav_status_message_state[\s\S]{0,800}last_nav_status_at[\s\S]{0,800}status_message',
        "nav status contract owns active-task Nav2 message state updates",
    )
    require(
        nav_status_contract,
        r'def nav_status_message_event_payload[\s\S]{0,700}nav_status',
        "nav status contract owns Nav2 message timeline payloads",
    )
    require(
        nav_status_contract,
        r'def apply_nav_feedback_state[\s\S]{0,2200}last_nav_feedback_monotonic[\s\S]{0,1600}last_nav_distance_remaining_m[\s\S]{0,1600}has_nav_feedback',
        "nav status contract owns active-task Nav2 feedback state updates",
    )
    require(
        nav_status_contract,
        r'def nav_feedback_event_payload[\s\S]{0,900}nav_feedback[\s\S]{0,900}nav_goal_match',
        "nav status contract owns Nav2 feedback timeline payloads",
    )
    require(
        nav_status_contract,
        r'def nav_feedback_dispatch_payload[\s\S]{0,700}update_message[\s\S]{0,700}update_feedback',
        "nav status contract owns Nav2 feedback dispatch rules",
    )
    require(
        nav_status_contract,
        r'def apply_ignored_nav_status_state[\s\S]{0,700}last_ignored_nav_status[\s\S]{0,700}last_ignored_nav_goal_match',
        "nav status contract owns ignored Nav2 status state updates",
    )
    require(
        nav_status_contract,
        r'def ignored_nav_status_event_payload[\s\S]{0,1000}nav_status_ignored[\s\S]{0,1000}operator_payload',
        "nav status contract owns ignored Nav2 status timeline and operator event payloads",
    )
    require(
        nav_status_contract,
        r'def nav_success_completion_decision[\s\S]{0,1800}annotation_not_last_goal[\s\S]{0,1400}nav_goal_not_active[\s\S]{0,1600}current_goal_succeeded',
        "nav status contract owns Nav2 success completion gating",
    )
    require(
        nav_status_contract,
        r'def classify_navigation_status[\s\S]{0,1800}nav_goal_accepted[\s\S]{0,1800}complete_waypoint[\s\S]{0,1800}nav_goal_cancelled[\s\S]{0,1800}update_message',
        "nav status contract owns Nav2 status classification rules",
    )
    forbid(
        web,
        r'def _nav_status_matches_active_goal',
        "web node must call nav-status contract active-goal matching directly instead of keeping a thin wrapper",
    )
    forbid(
        web,
        r'def _friendly_nav_status',
        "web node no longer owns operator-facing Nav2 status text",
    )
    require(
        web,
        r'match = nav_status_matches_active_goal\(active,\s*annotation,\s*status_payload\)',
        "web calls nav-status contract active-goal matching directly",
    )
    require(
        web,
        r'def _fail_active_task_from_nav_status[\s\S]{0,700}apply_nav_failure_state[\s\S]{0,700}fail_active_task_state\([\s\S]{0,900}failed\["result_status"\]',
        "web delegates Nav2 failure state to nav status and terminal failure contracts",
    )
    require(
        web,
        r'def _update_active_task_from_nav_status[\s\S]{0,1600}apply_nav_goal_status_state[\s\S]{0,900}nav_goal_status_event_payload[\s\S]{0,600}event_payload\["event"\][\s\S]{0,300}event_payload\["message"\]',
        "web delegates Nav2 goal-status active-task state and timeline payloads to nav status contract",
    )
    require(
        web,
        r'def _update_active_task_status_message[\s\S]{0,900}apply_nav_status_message_state[\s\S]{0,700}nav_status_message_event_payload[\s\S]{0,500}event_payload\["event"\][\s\S]{0,300}event_payload\["message"\]',
        "web delegates Nav2 message active-task state and timeline payloads to nav status contract",
    )
    require(
        web,
        r'def _update_active_task_from_nav_feedback[\s\S]{0,700}nav_feedback_dispatch_payload[\s\S]{0,1800}should_record_nav_feedback_event[\s\S]{0,1200}apply_nav_feedback_state[\s\S]{0,900}nav_feedback_event_payload[\s\S]{0,500}event_payload\["event"\][\s\S]{0,300}event_payload\["message"\]',
        "web delegates Nav2 feedback active-task state and timeline payloads to nav status contract",
    )
    forbid(
        web,
        r'def _update_active_task_from_nav_status[\s\S]{0,2200}or "nav_%s"|def _update_active_task_status_message[\s\S]{0,1200}or "nav_status"|def _update_active_task_from_nav_feedback[\s\S]{0,2600}or "nav_feedback"|def _update_active_task_from_nav_feedback[\s\S]{0,2600}or "Nav2 正在执行当前点位"',
        "web node no longer hardcodes Nav2 status/feedback timeline fallback payloads",
    )
    forbid(
        web,
        r'feedback\.get\("label"\)[\s\S]{0,80}floor_goal',
        "web node no longer owns Nav2 feedback floor-goal dispatch rules",
    )
    require(
        web,
        r'def _record_ignored_nav_status[\s\S]{0,700}apply_ignored_nav_status_state[\s\S]{0,900}ignored_nav_status_event_payload\(',
        "web task runner records ignored mismatched Nav2 statuses through nav status contract state and event payloads",
    )
    require(
        web,
        r'def _record_ignored_nav_status[\s\S]{0,1400}event_payload\["timeline_event"\][\s\S]{0,300}event_payload\["timeline_message"\][\s\S]{0,300}event_payload\["timeline_extra"\][\s\S]{0,900}event_payload\["operator_event"\][\s\S]{0,300}event_payload\["operator_payload"\]',
        "web ignored Nav2-status path consumes required nav-status contract event fields",
    )
    forbid(
        web,
        r'def _record_ignored_nav_status[\s\S]{0,1800}event_payload\.get\("(timeline_event|timeline_message|timeline_extra|operator_event|operator_payload)"',
        "web ignored Nav2-status path no longer falls back from contract event fields",
    )
    require(
        active_task_contract,
        r'def mark_goal_sent[\s\S]{0,1600}last_goal_attempt_id[\s\S]{0,800}last_goal_pose[\s\S]{0,800}goal_attempt_id',
        "active task contract records frontend goal attempt identity",
    )
    require(
        active_task_contract,
        r'def stale_goal_dispatch_payload[\s\S]{0,900}goal_dispatch_ignored[\s\S]{0,900}requested_annotation_id[\s\S]{0,900}current_annotation_id',
        "active task contract owns stale goal-dispatch diagnostics",
    )
    require(
        active_task_contract,
        r'def prepare_goal_send_state[\s\S]{0,900}active_annotation_missing_failure[\s\S]{0,1200}stale_goal_dispatch_payload[\s\S]{0,1200}mark_goal_sent[\s\S]{0,1200}"action": "send_goal"',
        "active task contract owns final goal-send state preparation and stale switch handling",
    )
    require(
        web,
        r'prepare_goal_send_state\([\s\S]{0,700}goal_attempt_id=new_id\("goal"\)',
        "web task runner passes a fresh frontend goal attempt identity to the active task contract",
    )
    require(
        dashboard,
        r'function taskStartConfirmText[\s\S]{0,900}首点[\s\S]{0,900}顺序[\s\S]{0,900}确认后机器狗会立即向首点导航',
        "frontend requires explicit task start confirmation with first waypoint and order",
    )
    require(
        dashboard,
        r'function taskWatcherCommand\(task\)[\s\S]{0,500}104_watch_frontend_task\.sh 180',
        "frontend can generate the recommended task watcher command",
    )
    require(
        dashboard,
        r'function taskReadyCheckCommand\(task\)[\s\S]{0,500}104_frontend_task_ready_check\.py --task-id',
        "frontend can generate the task-specific ready-check command",
    )
    forbid(
        dashboard,
        r'104_frontend_task_field_run|taskFieldRunCommand|fieldRunCommand|现场验证入口|复制验证',
        "frontend does not expose the removed field-run wrapper path",
    )
    require(
        dashboard,
        r'const evidenceCommandHtml = canCopyEvidence \? `[\s\S]{0,220}开跑前验收：\$\{readyCheckCommand\}[\s\S]{0,220}开跑前记录：\$\{watcherCommand\}',
        "frontend task cards show ready-check and watcher commands only for executable current-map tasks",
    )
    require(
        dashboard,
        r'function taskMapMismatchText[\s\S]{0,500}请在当前地图重新标点生成任务',
        "frontend can explain old-map task mismatch",
    )
    require(
        dashboard,
        r'旧地图任务已隐藏[\s\S]{0,260}默认接口不会返回[\s\S]{0,260}不能用于本次现场执行',
        "frontend task list hides old-map tasks instead of rendering disabled task cards",
    )
    require(
        dashboard_js,
        r'for \(const task of currentMapTasks\)',
        "frontend task list renders only current-map tasks",
    )
    require(
        dashboard,
        r'const canCopyEvidence = !mapMismatchText && readiness\.ready === true;',
        "frontend disables field evidence copy actions for old-map or unready tasks",
    )
    require(
        dashboard,
        r'function taskDisplayStatus\(taskStatus, readiness, mapMismatchText\)[\s\S]{0,220}if \(mapMismatchText\) return "旧地图";[\s\S]{0,220}if \(readiness && readiness\.ready === true\) return "可执行";[\s\S]{0,120}return "未就绪";',
        "frontend task cards do not show raw ready status for old-map or unready tasks",
    )
    require(
        dashboard,
        r'<span class="tag">\$\{displayStatus\}</span>',
        "frontend task card tag uses the derived display status",
    )
    forbid(
        dashboard,
        r'<span class="tag">\$\{taskStatus\}</span>',
        "frontend task card tag does not expose the raw backend task status",
    )
    require(
        dashboard,
        r'function updateCreateTaskButton\(\)[\s\S]{0,700}当前地图还没有任务点[\s\S]{0,500}先勾选当前地图点位',
        "frontend disables task creation until current-map task points are selected",
    )
    require(
        dashboard_js,
        r'function updateCreateTaskButton\(\)[\s\S]{0,500}selectedMapStatus\.ready === false[\s\S]{0,500}Nav2 当前加载地图不一致',
        "frontend disables task creation when selected map differs from Nav2 map",
    )
    require(
        dashboard_js,
        r'function markBlockedReason\(payload = state\.latest\)[\s\S]{0,700}重定位成功[\s\S]{0,500}/m20pro_tcp_bridge/map_pose',
        "frontend blocks mark saving until manual relocalization and fresh map pose are confirmed",
    )
    require(
        dashboard_js,
        r'function markBlockedReason\(payload = state\.latest\)[\s\S]{0,500}selectedMapStatus[\s\S]{0,500}Nav2 当前加载地图不一致',
        "frontend blocks mark saving when selected map differs from Nav2 map",
    )
    require(
        dashboard_js,
        r'function updateMarkControls\(payload = state\.latest\)[\s\S]{0,900}saveMarkBtn[\s\S]{0,900}useRobotPoseBtn',
        "frontend disables mark controls through one shared readiness check",
    )
    require(
        dashboard,
        r'id="taskNextStepSummary"',
        "frontend task page gives a single next-step summary for relocalization and current-map task creation",
    )
    require(
        dashboard_js,
        r'function renderTaskNextStep\(\)[\s\S]{0,1800}定位页[\s\S]{0,260}重定位成功[\s\S]{0,500}当前地图没有任务点',
        "frontend next-step summary points to manual relocalization before task creation",
    )
    require(
        dashboard,
        r'const evidenceButtonHtml = canCopyEvidence \? `[\s\S]{0,240}data-copy-command="\$\{readyCheckCommand\}"',
        "frontend ready-check copy button is gated by task evidence readiness",
    )
    require(
        dashboard,
        r'const evidenceButtonHtml = canCopyEvidence \? `[\s\S]{0,420}data-copy-command="\$\{watcherCommand\}"',
        "frontend watcher copy button is gated by task evidence readiness",
    )
    require(
        dashboard,
        r'data-copy-command="\$\{readyCheckCommand\}"[\s\S]{0,300}data-copy-command="\$\{watcherCommand\}"',
        "frontend task cards provide copy buttons for field evidence commands",
    )
    require(
        dashboard,
        r'id="copyFieldSnapshotBtn"[\s\S]{0,260}复制现场快照',
        "frontend task page provides a field snapshot copy button",
    )
    require(
        dashboard,
        r'function taskExecutionEvidence\(activeTask,\s*activeWaypoint\)[\s\S]{0,2400}floor_goal_published_at[\s\S]{0,1800}nav_goal_status[\s\S]{0,1800}path_goal_error_m',
        "frontend field snapshot has structured task execution evidence helper",
    )
    require(
        dashboard,
        r'function buildFieldSnapshot\(\)[\s\S]{0,4600}captured_at[\s\S]{0,4600}perception[\s\S]{0,4600}task_execution_evidence[\s\S]{0,1600}recommended_task[\s\S]{0,1600}task_pose_tracker_text',
        "frontend field snapshot captures task, perception, execution evidence and pose tracker state",
    )
    require(
        dashboard,
        r'const currentMapTasks = state\.tasks\.filter\(task => taskBelongsToSelectedMap\(task\)\);[\s\S]{0,360}const recommendedTask = currentMapTasks\.find',
        "frontend field snapshot recommends only tasks that belong to the selected map",
    )
    require(
        dashboard,
        r'function renderActiveTaskSummary\(activeTask,\s*waypoint\)[\s\S]{0,240}if \(!activeTask && !waypoint\) \{[\s\S]{0,120}box\.textContent = "无任务";',
        "frontend active task summary only represents live active task state",
    )
    require(
        dashboard_js,
        r'function renderActiveTaskSummary\(activeTask,\s*waypoint\)[\s\S]{0,3000}last_floor_goal_published_at[\s\S]{0,160}floor_goal已发[\s\S]{0,260}floor_goal_publish_count[\s\S]{0,260}/floor_goal',
        "frontend active task summary renders floor-goal publish evidence",
    )
    require(
        dashboard_js,
        r'\} else if \(Date\.now\(\) > state\.activeTaskLogUntil\) \{[\s\S]{0,180}renderActiveTaskSummary\(null, null\);[\s\S]{0,120}\$\("activeTask"\)\.textContent = "无任务";',
        "frontend active task raw panel does not render historical last_result as current execution",
    )
    forbid(
        dashboard_js,
        r'\$\("activeTask"\)\.textContent = s\.last_task_result',
        "frontend active task raw panel must not display last_task_result in the current-execution panel",
    )
    require(
        dashboard_js,
        r'function updateTaskControlButtons\(payload = state\.latest\)[\s\S]{0,700}stopBtn\.disabled = !hasActiveTask[\s\S]{0,500}当前没有前端任务在执行',
        "frontend disables the stop-task button when no active frontend task exists",
    )
    require(
        dashboard_js,
        r'resetBtn\.title = "显式复位导航会话；会停止前端任务、清理导航会话并清代价地图"',
        "frontend labels navigation reset as an explicit reset action",
    )
    require(
        dashboard_js,
        r'确认复位导航状态[\s\S]{0,220}清理导航会话并清代价地图',
        "frontend requires confirmation before explicit navigation reset",
    )
    forbid(
        dashboard,
        r'function renderActiveTaskSummary\(activeTask,\s*waypoint[\s\S]{0,900}lastResult',
        "frontend active task summary must not render historical last_result as current execution",
    )
    require(
        dashboard,
        r'fieldSnapshot\(\)[\s\S]{0,220}buildFieldSnapshot',
        "frontend debug API exposes the field snapshot for smoke tests",
    )
    require(
        dashboard,
        r'taskStartConfirmText[\s\S]{0,900}先验收[\s\S]{0,300}taskReadyCheckCommand\(task\)[\s\S]{0,500}再开记录[\s\S]{0,300}taskWatcherCommand\(task\)',
        "frontend task start confirmation reminds the operator to run ready-check and watcher",
    )
    require(
        task_contract,
        r'def validate_task_start_expectations[\s\S]{0,2600}expected_annotation_ids[\s\S]{0,2600}expected_first_annotation_id[\s\S]{0,2600}expected_first_pose',
        "task contract validates frontend task start expectations before motion",
    )
    forbid(
        web,
        r'def _validate_task_start_expectations',
        "web node must call task-contract start expectation validation directly instead of keeping a thin wrapper",
    )
    require(
        web,
        r'expectation_error = validate_task_start_expectations\(\s*payload,\s*task,\s*first_annotation,\s*task_map_id',
        "web calls task-contract start expectation validation directly before motion",
    )
    require(
        active_task_contract,
        r'def idle_stop_task_response[\s\S]{0,500}"reset_navigation": False',
        "active task contract owns no-active-task stop no-reset semantics",
    )
    require(
        web,
        r'def _stop_task\(self, payload[\s\S]{0,650}normalize_stop_task_request\(payload\)[\s\S]{0,650}if not stopped_task_id and not stop_request\["is_reset"\][\s\S]{0,160}return idle_stop_task_response\(\)',
        "web stop API delegates no-active-task stop response instead of resetting navigation",
    )
    require(
        task_contract,
        r'def task_status_allows_start[\s\S]{0,260}ready[\s\S]{0,260}stopped[\s\S]{0,260}completed[\s\S]{0,260}error',
        "task contract owns task startable status rules",
    )
    forbid(
        web,
        r'def _task_status_allows_start|def _task_readiness_success|def _task_readiness_failure|def _readiness_error_payload|def _task_waypoint_payload|def _readiness_waypoint_payload|def _pose_age_sec|def _pose_distance_m',
        "web node no longer keeps thin task-contract wrapper helpers",
    )
    forbid(
        web,
        r'def _validate_task_start_readiness',
        "web node must not keep a second task-start readiness implementation beside _task_start_readiness_payload",
    )
    forbid(
        web,
        r'task_status_allows_start\(',
        "web delegates startable-status checks through task_start_static_context instead of calling status rules inline",
    )
    forbid(
        web,
        r'def _is_finite_pose_dict|def _is_plausible_pose_dict',
        "web node no longer owns duplicate pose-dict validation helpers",
    )
    forbid(
        web,
        r'contractis_|contract[a-zA-Z0-9_]*pose',
        "web node must not contain typoed contract helper names that py_compile cannot catch",
    )
    require(
        web,
        r'from \.task_contract import \([\s\S]{0,400}is_finite_pose_dict[\s\S]{0,400}is_plausible_pose_dict',
        "web imports pose-dict validation from task contract",
    )
    require(
        task_contract,
        r'def validate_task_annotation_order[\s\S]{0,700}manual_point_type[\s\S]{0,700}charge[\s\S]{0,700}充电点必须放在任务最后',
        "task contract owns charge-point ordering rule",
    )
    require(
        annotation_contract,
        r'MANUAL_POINT_TYPES[\s\S]{0,1200}UI_TYPE_TO_MANUAL_POINT_TYPE[\s\S]{0,900}DEFAULT_VENDOR_NAVIGATION',
        "annotation contract owns manual point type and vendor navigation constants",
    )
    require(
        annotation_contract,
        r'def normalize_annotation_semantics[\s\S]{0,2600}manual_point_type[\s\S]{0,2600}vendor_navigation[\s\S]{0,1600}inspect_duration_s',
        "annotation contract owns annotation semantics normalization",
    )
    require(
        annotation_contract,
        r'def annotation_semantics_payload[\s\S]{0,1800}manual_point_type_label[\s\S]{0,1800}vendor_navigation',
        "annotation contract owns runtime annotation semantics payloads",
    )
    forbid(
        web,
        r'MANUAL_POINT_TYPES|UI_TYPE_TO_MANUAL_POINT_TYPE|DEFAULT_VENDOR_NAVIGATION|def _manual_point_type_from_payload|def _vendor_navigation_from_payload|def _annotation_semantics_payload|def _annotation_dwell_s|def _normalize_annotation_semantics|def _string_list',
        "web node no longer owns annotation semantics constants or thin wrapper helpers",
    )
    require(
        web,
        r'def _validate_task_annotation_order[\s\S]{0,500}normalize_annotation_semantics\(annotation\)[\s\S]{0,500}return validate_task_annotation_order\(normalized\)',
        "web delegates annotation normalization before task ordering",
    )
    require(
        task_contract_test,
        r'test_start_expectation_failures[\s\S]{0,1800}expected_annotation_ids[\s\S]{0,1800}expected_first_pose[\s\S]{0,1800}test_charge_must_be_last',
        "offline task contract tests cover start expectation and charge ordering rules",
    )
    require(
        active_task_contract,
        r'def goal_dispatch_decision[\s\S]{0,1800}accepted[\s\S]{0,1800}succeeded[\s\S]{0,1800}resend_interval_s[\s\S]{0,1800}send_goal',
        "active task contract owns goal dispatch/resend decision rules",
    )
    require(
        active_task_contract,
        r'def goal_dispatch_decision[\s\S]{0,2600}operator_event[\s\S]{0,600}补发当前任务点[\s\S]{0,900}operator_payload',
        "active task contract owns resend-goal operator event payloads",
    )
    require(
        active_task_contract,
        r'def create_active_task_state[\s\S]{0,2200}"status": "running"[\s\S]{0,1200}"status_message": "任务已创建，准备下发第一个点位"[\s\S]{0,900}task_started',
        "active task contract owns active task creation state and timeline payload",
    )
    require(
        active_task_contract,
        r'def create_active_task_state[\s\S]{0,3000}operator_event[\s\S]{0,500}启动前端任务[\s\S]{0,600}operator_payload',
        "active task contract owns task-start operator event payload",
    )
    require(
        active_task_contract,
        r'GOAL_SENT_RESET_KEYS[\s\S]{0,1600}last_nav_feedback[\s\S]{0,1600}stall_warned[\s\S]{0,1600}plan_goal_verified',
        "active task contract owns stale per-goal reset keys",
    )
    require(
        active_task_contract,
        r'NEXT_WAYPOINT_RESET_KEYS[\s\S]{0,600}last_goal_attempt_id[\s\S]{0,600}goal_sent_path_version',
        "active task contract owns stale next-waypoint reset keys",
    )
    require(
        active_task_contract,
        r'def mark_goal_sent[\s\S]{0,2200}total_goal_send_count[\s\S]{0,2200}waypoint_goal_send_count[\s\S]{0,2200}resend_goal_count',
        "active task contract owns goal-sent counters and active state update",
    )
    require(
        active_task_contract,
        r'def mark_goal_sent[\s\S]{0,3200}waypoint_goal_sent[\s\S]{0,1000}event_extra',
        "active task contract owns waypoint-goal-sent timeline payloads",
    )
    require(
        active_task_contract,
        r'def mark_floor_goal_published_state[\s\S]{0,1600}last_floor_goal_published_at[\s\S]{0,1600}floor_goal_publish_count[\s\S]{0,1600}floor_goal_published',
        "active task contract owns floor-goal publish evidence state",
    )
    require(
        active_task_contract,
        r'def advance_active_task_state[\s\S]{0,2200}task_completed[\s\S]{0,800}result_status[\s\S]{0,800}waypoint_advanced',
        "active task contract owns waypoint advance/completion terminal state update",
    )
    require(
        active_task_contract,
        r'def dwell_tick_decision[\s\S]{0,900}"action": "wait"[\s\S]{0,900}"action": "advance"',
        "active task contract owns dwell tick decisions",
    )
    require(
        active_task_contract,
        r'def begin_waypoint_dwell_state[\s\S]{0,1800}waypoint_dwell_started',
        "active task contract owns dwell start state update",
    )
    require(
        active_task_contract,
        r'def begin_waypoint_dwell_state[\s\S]{0,2200}operator_event[\s\S]{0,600}到达点位并开始停留[\s\S]{0,700}operator_payload',
        "active task contract owns dwell-start operator event payloads",
    )
    require(
        active_task_contract,
        r'def remaining_dwell_s[\s\S]{0,700}dwell_until[\s\S]{0,700}def active_waypoint_elapsed_s[\s\S]{0,700}waypoint_started_monotonic',
        "active task contract owns active task time-derived diagnostics",
    )
    require(
        active_task_contract,
        r'def append_active_task_timeline_event_state[\s\S]{0,1800}timeline\[-limit:\][\s\S]{0,500}last_timeline_event',
        "active task contract owns active task timeline event state updates",
    )
    require(
        active_task_contract,
        r'def active_annotation_missing_failure[\s\S]{0,1400}active_waypoint_missing[\s\S]{0,1400}已停止任务',
        "active task contract owns missing active waypoint failure payload",
    )
    require(
        active_task_contract,
        r'def active_annotation_resolution[\s\S]{0,1200}annotation_ids[\s\S]{0,900}annotation_id[\s\S]{0,900}def active_annotation_from_list',
        "active task contract owns active waypoint id resolution from active task state",
    )
    require(
        active_task_contract,
        r'def active_task_failure_payload[\s\S]{0,900}default_message: Optional\[str\] = None[\s\S]{0,900}pop\("message"[\s\S]{0,500}pop\("action"',
        "active task contract owns failure-payload normalization",
    )
    require(
        active_task_contract_test,
        r'test_active_task_failure_payload[\s\S]{0,1600}active_task_failure_payload\(\{"task_id": "task_3"\}\)[\s\S]{0,700}任务执行失败，已停止任务[\s\S]{0,900}active_task_failure_payload\(active_annotation_missing_failure',
        "offline active-task tests cover implicit and missing-waypoint failure messages",
    )
    require(
        web,
        r'def _fail_active_task_from_payload[\s\S]{0,900}active_task_failure_payload\([\s\S]{0,900}payload\["message"\]',
        "web consumes normalized failure messages from the active-task contract",
    )
    forbid(
        web,
        r'def _fail_active_task_from_payload[\s\S]{0,1200}payload\.get\("message"\)[\s\S]{0,300}任务执行失败，已停止任务',
        "web failure helper no longer falls back from normalized contract messages",
    )
    require(
        active_task_contract,
        r'def mark_active_task_stopped_state[\s\S]{0,500}任务已手动停止/复位[\s\S]{0,500}status_message',
        "active task contract owns manual stop state updates",
    )
    require(
        active_task_contract,
        r'def normalize_stop_task_request[\s\S]{0,600}web_manual_stop[\s\S]{0,500}web_manual_reset',
        "active task contract owns stop-task request normalization",
    )
    require(
        active_task_contract,
        r'def stop_task_state[\s\S]{0,800}mark_active_task_stopped_state[\s\S]{0,800}task_stopped[\s\S]{0,800}result_status',
        "active task contract owns manual stop event/result state",
    )
    require(
        active_task_contract,
        r'def idle_stop_task_response[\s\S]{0,500}reset_navigation": False[\s\S]{0,500}当前没有前端任务在执行，无需停止',
        "active task contract owns no-active-task stop response semantics",
    )
    require(
        active_task_contract,
        r'def task_terminal_event_payload[\s\S]{0,1200}前端任务完成[\s\S]{0,900}停止前端任务[\s\S]{0,900}前端任务停止[\s\S]{0,900}前端任务因导航状态停止',
        "active task contract owns terminal task event payloads",
    )
    require(
        active_task_contract,
        r'def stop_task_state[\s\S]{0,1100}operator_event[\s\S]{0,600}operator_payload',
        "active task contract owns manual-stop operator event payloads",
    )
    require(
        active_task_contract,
        r'def stop_task_operator_event_payload[\s\S]{0,600}task_terminal_event_payload\(event="stopped"',
        "active task contract owns explicit-reset operator event payloads",
    )
    require(
        active_task_contract,
        r'def mark_active_task_failed_state[\s\S]{0,500}last_error[\s\S]{0,500}status_message',
        "active task contract owns active task failure state updates",
    )
    require(
        active_task_contract,
        r'def fail_active_task_state[\s\S]{0,1200}task_failed[\s\S]{0,900}result_status[\s\S]{0,500}operator_event[\s\S]{0,500}operator_payload',
        "active task contract owns terminal failure state and operator event payloads",
    )
    require(
        active_task_contract,
        r'def advance_active_task_state[\s\S]{0,2600}task_completed[\s\S]{0,900}operator_event[\s\S]{0,500}operator_payload',
        "active task contract owns completed-task operator event payloads",
    )
    require(
        active_task_contract,
        r'def mark_active_task_waiting_state[\s\S]{0,900}last_wait_code[\s\S]{0,900}last_wait_at[\s\S]{0,900}should_record_event',
        "active task contract owns generic waiting state updates",
    )
    forbid(
        web,
        r'def _remaining_dwell_s|def _active_waypoint_elapsed_s',
        "web node no longer owns active task time-derived diagnostics",
    )
    forbid(
        web,
        r'extra = dict\([^)]*\)[\s\S]{0,180}extra\.pop\("message"[\s\S]{0,180}extra\.pop\("action"',
        "web node no longer normalizes active-task failure payloads inline",
    )
    require(
        web,
        r'def _start_task[\s\S]{0,9000}create_active_task_state\([\s\S]{0,1200}created\["active"\][\s\S]{0,1200}created\["event"\][\s\S]{0,1400}created\["operator_event"\]',
        "web delegates active task creation state, timeline and operator payloads to active task contract",
    )
    require(
        web,
        r'def _start_task[\s\S]{0,9000}created\["event"\][\s\S]{0,400}created\["message"\][\s\S]{0,1400}created\["operator_event"\]',
        "web consumes required active-task creation message from the contract",
    )
    forbid(
        web,
        r'def _start_task[\s\S]{0,9000}created\.get\("message"\)|def _start_task[\s\S]{0,9000}active\.get\("status_message"\) or ""',
        "web active-task creation path no longer falls back from contract message",
    )
    forbid(
        web,
        r'_append_active_task_timeline_event\([\s\S]{0,200}"task_started"|_append_event\("启动前端任务"',
        "web node no longer hardcodes task-start timeline/operator event payloads",
    )
    require(
        web,
        r'def _stop_task[\s\S]{0,500}normalize_stop_task_request[\s\S]{0,1200}idle_stop_task_response\(\)[\s\S]{0,1200}stop_task_state',
        "web delegates stop request normalization, idle stop and active-task stop state to active task contract",
    )
    require(
        web,
        r'def _fail_active_task[\s\S]{0,1000}fail_active_task_state\(',
        "web delegates terminal failure active-task state to active task contract",
    )
    require(
        web,
        r'def _fail_active_task_from_nav_status[\s\S]{0,1200}terminal_event="nav_failed"[\s\S]{0,1800}failed\["event"\][\s\S]{0,350}failed\["message"\][\s\S]{0,900}failed\["result_status"\][\s\S]{0,1400}failed\["operator_event"\][\s\S]{0,350}failed\["operator_payload"\]',
        "web consumes nav-failure timeline, result and operator payloads from active task contract",
    )
    require(
        web,
        r'def _stop_task[\s\S]{0,1800}stop_task_state[\s\S]{0,1000}stopped\["event"\][\s\S]{0,350}stopped\["message"\][\s\S]{0,900}stopped\["result_status"\][\s\S]{0,1300}stopped\["operator_event"\][\s\S]{0,450}stopped\["operator_payload"\][\s\S]{0,700}stop_task_operator_event_payload',
        "web consumes stopped/reset timeline, result and operator payloads from active task contract",
    )
    require(
        web,
        r'def _fail_active_task[\s\S]{0,1200}fail_active_task_state[\s\S]{0,900}failed\["event"\][\s\S]{0,350}failed\["message"\][\s\S]{0,900}failed\["result_status"\][\s\S]{0,1300}failed\["operator_event"\][\s\S]{0,350}failed\["operator_payload"\]',
        "web consumes failed-task timeline, result and operator payloads from active task contract",
    )
    require(
        web,
        r'def _advance_active_task[\s\S]{0,1100}advance_active_task_state[\s\S]{0,800}result\["event"\][\s\S]{0,350}result\["message"\][\s\S]{0,900}result\["result_status"\][\s\S]{0,1300}result\["operator_event"\][\s\S]{0,350}result\["operator_payload"\]',
        "web consumes active-task advance/completion timeline, result and operator payloads from active task contract",
    )
    forbid(
        web,
        r'def _fail_active_task_from_nav_status[\s\S]{0,3500}failed\.get\("(event|message|event_extra|result_status|operator_event|operator_payload)"|def _stop_task[\s\S]{0,3200}stopped\.get\("(event|message|event_extra|result_status|operator_event|operator_payload)"|def _fail_active_task\([\s\S]{0,3000}failed\.get\("(event|message|event_extra|result_status|operator_event|operator_payload)"|def _advance_active_task[\s\S]{0,3000}result\.get\("(event|message|event_extra|result_status|operator_event|operator_payload)"',
        "web terminal/advance paths require active task contract payload fields instead of fallback getters",
    )
    forbid(
        web,
        r'task_terminal_event_payload\(',
        "web node no longer assembles terminal task event payloads directly",
    )
    require(
        web,
        r'def _dispatch_active_goal[\s\S]{0,1500}goal_dispatch_decision[\s\S]{0,3000}prepare_goal_send_state',
        "web delegates active goal dispatch state to active task contract",
    )
    require(
        web,
        r'def _dispatch_active_goal[\s\S]{0,2200}operator_payload = decision\.get\("operator_payload"\)[\s\S]{0,500}decision\["operator_event"\]',
        "web delegates resend-goal operator event payload assembly to active task contract",
    )
    require(
        web,
        r'def _dispatch_active_goal[\s\S]{0,3800}prepared = prepare_goal_send_state\([\s\S]{0,1400}prepared\["event"\][\s\S]{0,500}prepared\["message"\]',
        "web delegates waypoint-goal-sent timeline payload assembly to active task contract",
    )
    require(
        web,
        r'def _dispatch_active_goal[\s\S]{0,5200}_publish_floor_goal[\s\S]{0,1400}mark_floor_goal_published_state[\s\S]{0,900}result\["event"\][\s\S]{0,500}result\["message"\]',
        "web records floor-goal publish evidence after publishing the task goal",
    )
    require(
        web,
        r'def _dispatch_active_goal[\s\S]{0,3600}action == "record_stale"[\s\S]{0,600}prepared\["event"\][\s\S]{0,500}prepared\["message"\]',
        "web records stale goal-dispatch diagnostics instead of silently returning after waypoint switches",
    )
    forbid(
        web,
        r'def _dispatch_active_goal[\s\S]{0,6500}current_annotation\.get\("id"\) != annotation\.get\("id"\)|def _dispatch_active_goal[\s\S]{0,6500}stale_goal_dispatch_payload\(|def _dispatch_active_goal[\s\S]{0,6500}mark_goal_sent\(',
        "web dispatch path no longer owns final goal-send state preparation inline",
    )
    forbid(
        web,
        r'def _dispatch_active_goal[\s\S]{0,6500}or "补发当前任务点"|def _dispatch_active_goal[\s\S]{0,6500}or "goal_dispatch_ignored"|def _dispatch_active_goal[\s\S]{0,6500}or "任务点已切换，忽略过期目标下发"|def _dispatch_active_goal[\s\S]{0,6500}or "waypoint_goal_sent"|def _dispatch_active_goal[\s\S]{0,6500}or "floor_goal_published"',
        "web dispatch path no longer hardcodes task dispatch timeline/operator fallback payloads",
    )
    require(
        web,
        r'def _dispatch_active_goal[\s\S]{0,3200}goal = waypoint_goal_payload\(annotation\)[\s\S]{0,600}_fail_active_task_from_payload',
        "web uses the shared active-task failure payload path for bad waypoint goals",
    )
    require(
        web,
        r'def _dispatch_active_goal[\s\S]{0,3200}extra=waypoint_goal_failure_extra\(annotation\)',
        "web delegates bad waypoint goal failure extras to active task contract",
    )
    forbid(
        web,
        r'当前任务点位已删除或索引越界，已停止任务(?!；请重新生成任务)',
        "web no longer hardcodes the missing-waypoint fallback message outside the active-task contract",
    )
    require(
        web,
        r'def _dispatch_active_goal[\s\S]{0,900}annotation is None:[\s\S]{0,700}active_annotation_missing_failure[\s\S]{0,700}_fail_active_task',
        "web fails active tasks instead of silently returning when dispatch first finds a missing waypoint",
    )
    require(
        web,
        r'def _dispatch_active_goal[\s\S]{0,4800}if action == "fail":[\s\S]{0,600}missing_failure = prepared\["failure"\][\s\S]{0,1800}if missing_failure is not None:[\s\S]{0,700}_fail_active_task',
        "web fails active tasks instead of silently returning when dispatch recheck finds a missing waypoint",
    )
    require(
        web,
        r'def _update_active_task_from_nav_status[\s\S]{0,900}missing_failure = None[\s\S]{0,900}active_annotation_missing_failure\(active\)[\s\S]{0,1600}if missing_failure is not None:[\s\S]{0,700}_fail_active_task',
        "web fails active tasks when Nav2 status arrives after the active waypoint disappeared",
    )
    require(
        web,
        r'def _update_active_task_from_nav_feedback[\s\S]{0,900}missing_failure = None[\s\S]{0,900}active_annotation_missing_failure\(active\)[\s\S]{0,2600}if missing_failure is not None:[\s\S]{0,700}_fail_active_task',
        "web fails active tasks when Nav2 feedback arrives after the active waypoint disappeared",
    )
    require(
        web,
        r'def _advance_active_task[\s\S]{0,1200}advance_active_task_state[\s\S]{0,1200}result\["result_status"\]',
        "web delegates active task advance and completion result state to active task contract",
    )
    require(
        web,
        r'def _tick_active_task[\s\S]{0,700}dwell_tick_decision[\s\S]{0,700}_advance_active_task',
        "web delegates dwell tick decisions to active task contract",
    )
    require(
        web,
        r'def _tick_active_task[\s\S]{0,900}annotation is None:[\s\S]{0,500}active_annotation_missing_failure[\s\S]{0,500}_fail_active_task',
        "web fails active tasks instead of silently returning when the active waypoint is missing",
    )
    require(
        web,
        r'def _active_annotation\(self,\s*active:[\s\S]{0,500}return active_annotation_from_list\(active,\s*self\._annotations\)',
        "web delegates active waypoint lookup to the active task contract",
    )
    require(
        web,
        r'def _mark_active_task_waiting[\s\S]{0,900}mark_active_task_waiting_state[\s\S]{0,900}should_record_event[\s\S]{0,400}result\["event"\][\s\S]{0,350}result\["message"\][\s\S]{0,350}result\["event_extra"\]',
        "web delegates generic active-task waiting state updates to active task contract",
    )
    require(
        web,
        r'def _append_active_task_timeline_event[\s\S]{0,700}append_active_task_timeline_event_state',
        "web delegates active-task timeline event updates to active task contract",
    )
    require(
        web,
        r'def _begin_waypoint_dwell_or_advance[\s\S]{0,1000}begin_waypoint_dwell_state',
        "web delegates dwell start state to active task contract",
    )
    require(
        web,
        r'def _begin_waypoint_dwell_or_advance[\s\S]{0,1200}begin_waypoint_dwell_state[\s\S]{0,900}result\["event"\][\s\S]{0,350}result\["message"\][\s\S]{0,900}result\["operator_event"\][\s\S]{0,400}result\["operator_payload"\]',
        "web delegates dwell-start timeline and operator event payload assembly to active task contract",
    )
    forbid(
        web,
        r'def _begin_waypoint_dwell_or_advance[\s\S]{0,2500}result\.get\("(event|message|event_extra|operator_event|operator_payload)"|def _mark_active_task_waiting[\s\S]{0,1800}result\.get\("(event|message|event_extra)"',
        "web dwell/waiting paths require active task contract payload fields instead of fallback getters",
    )
    require(
        active_task_contract_test,
        r'test_dispatch_decision[\s\S]{0,1800}test_mark_goal_sent_new_and_resend[\s\S]{0,3000}test_prepare_goal_send_state[\s\S]{0,3000}test_create_active_task_state[\s\S]{0,3000}test_mark_active_task_terminal_states[\s\S]{0,1800}test_fail_active_task_state[\s\S]{0,1800}test_stop_task_state[\s\S]{0,1800}test_task_terminal_event_payload[\s\S]{0,1800}test_dwell_state[\s\S]{0,1800}test_active_waypoint_elapsed_s[\s\S]{0,1800}test_append_active_task_timeline_event_state[\s\S]{0,1800}test_active_annotation_missing_failure[\s\S]{0,1800}test_active_annotation_resolution[\s\S]{0,1800}test_active_annotation_from_list[\s\S]{0,1800}test_active_task_failure_payload[\s\S]{0,1800}test_mark_active_task_waiting_state[\s\S]{0,1800}test_advance_active_task_state',
        "offline active task contract tests cover creation, dispatch, resend, terminal states, stop requests, terminal event payloads, dwell, elapsed-time, timeline, active waypoint lookup, waiting, missing waypoint, failure payloads, reset and completion rules",
    )
    require(
        active_task_contract_test,
        r'def test_create_active_task_state[\s\S]{0,1000}task_started[\s\S]{0,600}operator_event[\s\S]{0,300}启动前端任务[\s\S]{0,400}operator_payload',
        "offline active task contract tests cover task-start timeline and operator payloads",
    )
    require(
        active_task_contract_test,
        r'def test_mark_goal_sent_new_and_resend[\s\S]{0,1800}waypoint_goal_sent[\s\S]{0,1200}goal sent event path version',
        "offline active task contract tests cover waypoint-goal-sent timeline payloads",
    )
    require(
        active_task_contract_test,
        r'def test_dispatch_decision[\s\S]{0,1600}补发当前任务点[\s\S]{0,900}legacy resend payload alias',
        "offline active task contract tests cover resend-goal operator event payloads",
    )
    require(
        active_task_contract_test,
        r'def test_dwell_state[\s\S]{0,1400}到达点位并开始停留[\s\S]{0,900}dwell operator seconds',
        "offline active task contract tests cover dwell-start operator event payloads",
    )
    require(
        task_snapshot_contract_test,
        r'test_runtime_snapshot[\s\S]{0,1800}last_goal_attempt_id[\s\S]{0,1800}last_nav_feedback[\s\S]{0,1800}test_active_waypoint_payload[\s\S]{0,2400}path_goal_error_m[\s\S]{0,1800}test_idle_waypoint_payload[\s\S]{0,1200}test_result_snapshot',
        "offline task snapshot contract tests cover runtime, live/idle active-waypoint and result diagnostics",
    )
    require(
        task_snapshot_contract_test,
        r'test_last_task_result_payload[\s\S]{0,1400}latest task id[\s\S]{0,1400}error-only task selected',
        "offline task snapshot contract tests cover last task result payload selection",
    )
    require(
        task_snapshot_contract_test,
        r'test_apply_task_result_persistence[\s\S]{0,1400}status stored with result[\s\S]{0,1400}completed status stored[\s\S]{0,1400}completed task clears last error',
        "offline task snapshot contract tests cover persisted task result and terminal status updates",
    )
    require(
        task_snapshot_contract_test,
        r'test_apply_task_result_to_tasks[\s\S]{0,1600}task result list update finds task[\s\S]{0,1200}missing task result update fails[\s\S]{0,1200}non-list tasks fail',
        "offline task snapshot contract tests cover list-level task result persistence",
    )
    require(
        nav_status_contract_test,
        r'test_parse_key_value_status[\s\S]{0,1800}test_matching_goal[\s\S]{0,1800}test_mismatch_reasons[\s\S]{0,2200}test_nav_success_completion_decision[\s\S]{0,2400}test_friendly_nav_status[\s\S]{0,2600}test_apply_nav_failure_state[\s\S]{0,2600}test_apply_nav_goal_status_state[\s\S]{0,3200}test_apply_nav_feedback_state[\s\S]{0,3600}test_apply_ignored_nav_status_state',
        "offline nav status contract tests cover parsing, goal matching, success completion gating, mismatch reasons, operator-facing text, active-task state updates and timeline payloads",
    )
    require(
        nav_status_contract_test,
        r'test_apply_nav_goal_status_state[\s\S]{0,2200}nav_goal_status_event_payload[\s\S]{0,2200}test_apply_nav_status_message_state[\s\S]{0,1600}nav_status_message_event_payload[\s\S]{0,2600}test_apply_nav_feedback_state[\s\S]{0,1800}nav_feedback_dispatch_payload[\s\S]{0,3000}nav_feedback_event_payload',
        "offline nav status contract tests cover Nav2 dispatch and timeline payload helpers",
    )
    require(
        nav_status_contract_test,
        r'test_apply_ignored_nav_status_state[\s\S]{0,1800}ignored_nav_status_event_payload[\s\S]{0,1400}operator_payload',
        "offline nav status contract tests cover ignored-status event payloads",
    )
    require(
        navigation_readiness_contract,
        r'def navigation_readiness_payload[\s\S]{0,2600}Nav2、/scan 和代价地图已就绪[\s\S]{0,2200}等待复位后的 /scan 和 local/global costmap 新数据[\s\S]{0,2200}Nav2 lifecycle 尚未全部 active',
        "navigation readiness contract owns scan/costmap/lifecycle readiness messages",
    )
    require(
        navigation_readiness_contract,
        r'def should_check_navigation_readiness[\s\S]{0,900}require_nav_ready[\s\S]{0,900}require_localization_ok[\s\S]{0,900}pose_is_plausible',
        "navigation readiness contract owns the precondition for checking Nav2 readiness",
    )
    require(
        navigation_readiness_contract,
        r'def navigation_readiness_disabled_payload[\s\S]{0,700}任务启动前不要求 Nav2 readiness[\s\S]{0,400}required',
        "navigation readiness contract owns disabled Nav2 readiness payload semantics",
    )
    require(
        navigation_readiness_contract,
        r'def navigation_readiness_wait_timeout_payload[\s\S]{0,900}navigation_not_ready_after_reset[\s\S]{0,900}任务启动复位后 Nav2/代价地图未在[\s\S]{0,900}navigation_readiness',
        "navigation readiness contract owns post-reset Nav2 readiness timeout payload semantics",
    )
    require(
        navigation_readiness_contract_test,
        r'test_ready[\s\S]{0,1600}test_scan_missing[\s\S]{0,1600}test_costmap_missing[\s\S]{0,1600}test_lifecycle_inactive[\s\S]{0,1600}test_waits_for_post_reset_data[\s\S]{0,2600}test_should_check_navigation_readiness[\s\S]{0,2600}test_navigation_readiness_disabled_payload[\s\S]{0,2600}test_navigation_readiness_wait_timeout_payload',
        "offline navigation readiness contract tests cover ready, scan, costmap, lifecycle, post-reset data, should-check states, disabled state and post-reset timeout payloads",
    )
    require(
        web,
        r'from \.navigation_readiness_contract import \([\s\S]{0,500}navigation_readiness_disabled_payload[\s\S]{0,500}navigation_readiness_payload[\s\S]{0,500}navigation_readiness_wait_timeout_payload[\s\S]{0,500}should_check_navigation_readiness',
        "web imports pure navigation readiness contract helpers",
    )
    require(
        web,
        r'def _navigation_readiness_payload[\s\S]{0,1600}navigation_readiness_payload\(',
        "web delegates navigation readiness final decision to navigation readiness contract",
    )
    require(
        web,
        r'def _cached_navigation_readiness_payload[\s\S]{0,1200}_navigation_readiness_payload\(check_lifecycle=False\)',
        "web cached task/navigation readiness avoids high-frequency lifecycle service clients",
    )
    require(
        web,
        r'def _wait_for_navigation_ready_after_reset[\s\S]{0,1800}_navigation_readiness_payload\([\s\S]{0,220}check_lifecycle=False',
        "web post-reset navigation wait avoids repeated lifecycle service clients",
    )
    require(
        web,
        r'def _should_check_navigation_readiness[\s\S]{0,1000}should_check_navigation_readiness\(',
        "web delegates should-check navigation readiness precondition to navigation readiness contract",
    )
    require(
        web,
        r'def _wait_for_navigation_ready_after_reset[\s\S]{0,500}navigation_readiness_disabled_payload\([\s\S]{0,1800}navigation_readiness_wait_timeout_payload\(',
        "web delegates post-reset Nav2 readiness wait result payloads to navigation readiness contract",
    )
    forbid(
        web,
        r'def _wait_for_navigation_ready_after_reset[\s\S]{0,2400}任务启动前不要求 Nav2 readiness|def _wait_for_navigation_ready_after_reset[\s\S]{0,2400}navigation_not_ready_after_reset|def _wait_for_navigation_ready_after_reset[\s\S]{0,2400}任务启动复位后 Nav2/代价地图未在',
        "web no longer hardcodes post-reset Nav2 readiness wait payload strings",
    )
    forbid(
        web,
        r'def _parse_navigation_status|def _parse_key_value_status',
        "web node no longer owns navigation status parsing helpers",
    )
    require(
        web,
        r'def _on_navigation_status[\s\S]{0,360}parse_key_value_status\(msg\.data\)',
        "web delegates raw navigation status parsing to nav status contract",
    )
    require(
        web,
        r'from \.nav_status_contract import \([\s\S]{0,500}classify_navigation_status',
        "web imports Nav2 status classifier from nav status contract",
    )
    require(
        web,
        r'def _handle_navigation_status_for_task[\s\S]{0,500}decision = classify_navigation_status\(status_text\)',
        "web delegates Nav2 status classification to nav status contract",
    )
    forbid(
        web,
        r'def _handle_navigation_status_for_task[\s\S]{0,1400}status_text\.startswith\("nav_goal_',
        "web task handler must not re-own Nav2 status prefix classification",
    )
    forbid(
        web,
        r'def _handle_navigation_status_for_task[\s\S]{0,1400}status_text\.startswith\("error "\)',
        "web task handler must not re-own Nav2 error prefix classification",
    )
    require(
        task_plan_contract_test,
        r'test_plan_verified[\s\S]{0,1800}test_apply_plan_goal_verified_state[\s\S]{0,1800}test_plan_goal_verified_event_payload[\s\S]{0,2200}test_plan_mismatch[\s\S]{0,1800}test_plan_timeout_and_wait',
        "offline task plan contract tests cover verified state, verified event payload, mismatch and timeout decisions",
    )
    require(
        task_progress_contract_test,
        r'test_progress_initializes_reference[\s\S]{0,2400}test_tick_gate_decisions[\s\S]{0,3000}test_distance_decision[\s\S]{0,3000}test_pre_dispatch_decision[\s\S]{0,3600}test_progress_detects_stall_and_recovery[\s\S]{0,2400}test_stall_decision[\s\S]{0,2400}test_apply_stall_warning_state[\s\S]{0,2400}test_stall_warning_event_payload[\s\S]{0,2400}test_localization_lost_timeout_decision[\s\S]{0,2400}test_apply_localization_lost_start_state[\s\S]{0,2600}test_localization_lost_start_event_payload[\s\S]{0,2400}test_goal_accept_timeout_decision[\s\S]{0,2400}test_near_goal_wait_decision[\s\S]{0,3000}test_apply_near_goal_wait_state[\s\S]{0,2400}test_prepare_near_goal_wait_update[\s\S]{0,2600}test_timeout_decisions[\s\S]{0,2200}test_timeout_failure_extra',
        "offline task progress contract tests cover tick gate, progress, stall state, localization loss state and timeout decisions",
    )
    require(
        task_progress_contract,
        r'def active_task_tick_gate_decision[\s\S]{0,1200}no_pose[\s\S]{0,1200}localization_lost[\s\S]{0,1200}pose_stale[\s\S]{0,1200}wrong_floor',
        "task progress contract owns active task tick gate decisions",
    )
    require(
        task_progress_contract,
        r'def update_active_task_progress_state[\s\S]{0,5000}last_progress_monotonic[\s\S]{0,5000}stall_started_monotonic',
        "task progress contract owns progress/stall state calculations",
    )
    require(
        task_progress_contract,
        r'def active_task_distance_decision[\s\S]{0,1800}pose_invalid[\s\S]{0,1800}active_waypoint_pose_invalid[\s\S]{0,900}distance_ready',
        "task progress contract owns active task distance and invalid-pose decisions",
    )
    require(
        task_progress_contract,
        r'def active_task_pre_dispatch_decision[\s\S]{0,1800}active_task_tick_gate_decision[\s\S]{0,1400}active_task_distance_decision[\s\S]{0,900}pre_dispatch_ready',
        "task progress contract owns the pre-dispatch ordering of tick gate and distance decisions",
    )
    require(
        task_progress_contract,
        r'def task_stall_decision[\s\S]{0,3600}waypoint_stalled[\s\S]{0,3600}waypoint_stall_warning',
        "task progress contract owns stall warning/stop decisions",
    )
    require(
        task_progress_contract,
        r'def apply_stall_warning_state[\s\S]{0,700}stall_warned[\s\S]{0,500}status_message',
        "task progress contract owns stall warning state updates",
    )
    require(
        task_progress_contract,
        r'def stall_warning_event_payload[\s\S]{0,1400}waypoint_stall_warning[\s\S]{0,1200}operator_event[\s\S]{0,700}任务点位进展过慢',
        "task progress contract owns stall warning timeline and operator event payloads",
    )
    require(
        task_progress_contract,
        r'def stall_failure_extra[\s\S]{0,500}annotation_id[\s\S]{0,500}label',
        "task progress contract owns stall failure extras",
    )
    require(
        task_progress_contract,
        r'def localization_lost_timeout_decision[\s\S]{0,2200}start_timer[\s\S]{0,2200}定位/位姿丢失超过',
        "task progress contract owns localization-lost timeout decisions",
    )
    require(
        task_progress_contract,
        r'def apply_localization_lost_start_state[\s\S]{0,900}localization_lost_started_monotonic[\s\S]{0,700}status_message',
        "task progress contract owns localization-lost start state updates",
    )
    require(
        task_progress_contract,
        r'def localization_lost_start_event_payload[\s\S]{0,900}localization_lost_waiting[\s\S]{0,900}started_monotonic',
        "task progress contract owns localization-lost waiting timeline payloads",
    )
    require(
        task_progress_contract,
        r'def localization_lost_failure_extra[\s\S]{0,500}localization_lost_age_s',
        "task progress contract owns localization-lost failure extras",
    )
    require(
        task_progress_contract,
        r'def goal_accept_timeout_decision[\s\S]{0,2400}goal_accept_timeout[\s\S]{0,1800}/m20pro/floor_goal',
        "task progress contract owns goal-accept timeout decisions",
    )
    require(
        task_progress_contract,
        r'def near_goal_wait_decision[\s\S]{0,2600}current_goal_not_sent[\s\S]{0,2600}nav_goal_not_active[\s\S]{0,2600}near_goal_waiting_nav2',
        "task progress contract owns near-goal wait-vs-dispatch decisions",
    )
    require(
        task_progress_contract,
        r'def apply_near_goal_wait_state[\s\S]{0,900}near_goal_started_monotonic[\s\S]{0,600}near_goal_started_at',
        "task progress contract owns near-goal waiting state updates",
    )
    require(
        task_progress_contract,
        r'def prepare_near_goal_wait_update[\s\S]{0,700}task_not_running[\s\S]{0,700}task_changed[\s\S]{0,900}apply_near_goal_wait_state[\s\S]{0,900}"action": "update"',
        "task progress contract owns near-goal wait update applicability checks",
    )
    require(
        task_progress_contract,
        r'def waypoint_timeout_decision[\s\S]{0,2200}waypoint_timeout[\s\S]{0,2200}def near_goal_timeout_decision[\s\S]{0,2600}near_goal_no_nav2_result',
        "task progress contract owns waypoint and near-goal timeout decisions",
    )
    require(
        task_progress_contract,
        r'def timeout_failure_extra[\s\S]{0,900}annotation_id[\s\S]{0,700}distance_m[\s\S]{0,700}timeout_s',
        "task progress contract owns waypoint and near-goal timeout failure extras",
    )
    require(
        active_task_contract,
        r'def waypoint_goal_failure_extra[\s\S]{0,700}annotation_id[\s\S]{0,700}pose',
        "active task contract owns bad waypoint goal failure extras",
    )
    require(
        web,
        r'def _tick_active_task[\s\S]{0,2200}active_task_pre_dispatch_decision\(',
        "web delegates active task pre-dispatch decisions to task progress contract",
    )
    require(
        web,
        r'def _tick_active_task[\s\S]{0,2600}pre_dispatch\.get\("action"\) == "wait_and_monitor_localization"[\s\S]{0,1200}pre_dispatch\.get\("action"\) == "fail"[\s\S]{0,900}distance = float\(pre_dispatch\.get\("distance_m"\)\)',
        "web consumes the pre-dispatch contract result for wait/fail/pass without reordering checks",
    )
    require(
        web,
        r'def _tick_active_task[\s\S]{0,1800}pre_dispatch\["code"\][\s\S]{0,500}pre_dispatch\["message"\][\s\S]{0,500}pre_dispatch\["reason"\][\s\S]{0,900}pre_dispatch\["code"\][\s\S]{0,500}pre_dispatch\["message"\]',
        "web task tick consumes required pre-dispatch contract fields instead of fallback messages",
    )
    forbid(
        web,
        r'def _tick_active_task[\s\S]{0,2600}任务执行条件暂未满足|def _tick_active_task[\s\S]{0,2600}当前任务点坐标无效，已停止任务',
        "web task tick no longer hardcodes pre-dispatch waiting/failure fallback messages",
    )
    forbid(
        web,
        r'def _dispatch_active_goal[\s\S]{0,3200}当前任务点坐标无效，已停止任务',
        "web dispatch path uses waypoint-goal contract messages instead of hardcoded bad-pose fallback",
    )
    require(
        web,
        r'def _update_active_task_progress[\s\S]{0,900}update_active_task_progress_state',
        "web delegates active task progress state calculations to task progress contract",
    )
    require(
        web,
        r'def _stop_task_if_stalled[\s\S]{0,900}task_stall_decision',
        "web delegates stall decisions to task progress contract",
    )
    require(
        web,
        r'def _stop_task_if_stalled[\s\S]{0,1500}apply_stall_warning_state',
        "web delegates stall warning state updates to task progress contract",
    )
    require(
        web,
        r'def _stop_task_if_stalled[\s\S]{0,1800}stall_warning_event_payload\([\s\S]{0,1200}_append_active_task_timeline_event[\s\S]{0,1200}_append_event',
        "web delegates stall warning event payload assembly to task progress contract",
    )
    require(
        web,
        r'def _stop_task_if_stalled[\s\S]{0,1100}extra=stall_failure_extra\(annotation\)',
        "web delegates stall failure extras to task progress contract",
    )
    require(
        web,
        r'def _stop_task_if_stalled[\s\S]{0,2200}event_payload\["timeline_event"\][\s\S]{0,400}event_payload\["timeline_message"\][\s\S]{0,1200}operator_event_payload\["operator_event"\]',
        "web consumes required stall event contract fields instead of fallback messages",
    )
    forbid(
        web,
        r'def _stop_task_if_stalled[\s\S]{0,2600}当前点位进展过慢，已停止任务|def _stop_task_if_stalled[\s\S]{0,2600}or "waypoint_stall_warning"|def _stop_task_if_stalled[\s\S]{0,2600}or "任务点位进展过慢"',
        "web stall path no longer hardcodes failure or event fallback messages",
    )
    require(
        web,
        r'def _stop_task_if_localization_lost[\s\S]{0,1200}localization_lost_timeout_decision',
        "web delegates localization-lost timeout decisions to task progress contract",
    )
    require(
        web,
        r'def _stop_task_if_localization_lost[\s\S]{0,1200}apply_localization_lost_start_state',
        "web delegates localization-lost start state updates to task progress contract",
    )
    require(
        web,
        r'def _stop_task_if_localization_lost[\s\S]{0,1600}localization_lost_start_event_payload\([\s\S]{0,900}_append_active_task_timeline_event',
        "web delegates localization-lost waiting event payload assembly to task progress contract",
    )
    require(
        web,
        r'def _stop_task_if_localization_lost[\s\S]{0,1900}event_payload\["event"\][\s\S]{0,500}event_payload\["message"\]',
        "web consumes required localization-lost event contract fields instead of fallback messages",
    )
    require(
        web,
        r'def _stop_task_if_localization_lost[\s\S]{0,2400}_fail_active_task_from_payload',
        "web uses the shared active-task failure payload path for localization loss",
    )
    require(
        web,
        r'def _stop_task_if_localization_lost[\s\S]{0,2600}extra=localization_lost_failure_extra\(decision\)',
        "web delegates localization-lost failure extras to task progress contract",
    )
    forbid(
        web,
        r'def _stop_task_if_localization_lost[\s\S]{0,3000}任务执行中定位/位姿丢失，已停止任务|def _stop_task_if_localization_lost[\s\S]{0,3000}or "localization_lost_waiting"',
        "web localization-loss path no longer hardcodes failure or event fallback messages",
    )
    require(
        web,
        r'def _stop_task_if_goal_accept_timed_out[\s\S]{0,1000}goal_accept_timeout_decision',
        "web delegates goal-accept timeout decisions to task progress contract",
    )
    forbid(
        web,
        r'def _stop_task_if_goal_accept_timed_out[\s\S]{0,1200}当前点位下发后 Nav2 仍未接收，已停止任务',
        "web goal-accept timeout path no longer hardcodes failure fallback messages",
    )
    require(
        web,
        r'def _tick_active_task[\s\S]{0,5200}near_goal_wait_decision\(',
        "web delegates near-goal wait-vs-dispatch decisions to task progress contract",
    )
    require(
        web,
        r'def _tick_active_task[\s\S]{0,5600}prepare_near_goal_wait_update',
        "web delegates near-goal waiting state updates to task progress contract",
    )
    forbid(
        web,
        r'def _tick_active_task[\s\S]{0,6200}current\.get\("status"\) == "running" and current\.get\("task_id"\) == active\.get\("task_id"\)|def _tick_active_task[\s\S]{0,6200}apply_near_goal_wait_state\(',
        "web near-goal waiting path no longer owns update applicability checks inline",
    )
    require(
        web,
        r'def _tick_active_task[\s\S]{0,6000}near_goal_decision\["reason"\][\s\S]{0,500}near_goal_decision\["message"\]',
        "web consumes required near-goal waiting contract fields instead of fallback messages",
    )
    forbid(
        web,
        r'def _tick_active_task[\s\S]{0,6200}near_goal_decision\.get\("reason"\)|def _tick_active_task[\s\S]{0,6200}near_goal_decision\.get\("message"\)|def _tick_active_task[\s\S]{0,6200}已接近目标点，等待 Nav2 返回到达确认',
        "web near-goal waiting path no longer falls back from contract fields",
    )
    require(
        web,
        r'def _stop_task_if_waypoint_timed_out[\s\S]{0,900}waypoint_timeout_decision[\s\S]{0,1800}def _stop_task_if_near_goal_timed_out[\s\S]{0,900}near_goal_timeout_decision',
        "web delegates waypoint and near-goal timeout decisions to task progress contract",
    )
    require(
        web,
        r'def _stop_task_if_waypoint_timed_out[\s\S]{0,1200}extra=timeout_failure_extra\(annotation, decision\)[\s\S]{0,1800}def _stop_task_if_near_goal_timed_out[\s\S]{0,1200}extra=timeout_failure_extra\(annotation, decision\)',
        "web delegates waypoint and near-goal timeout failure extras to task progress contract",
    )
    forbid(
        web,
        r'def _stop_task_if_waypoint_timed_out[\s\S]{0,1400}当前点位执行超时，已停止任务|def _stop_task_if_near_goal_timed_out[\s\S]{0,1400}机器人已接近目标但 Nav2 未返回到达，已停止任务',
        "web waypoint timeout paths no longer hardcode failure fallback messages",
    )
    require(
        task_contract,
        r'def pose_map_bounds_error[\s\S]{0,1800}不在当前地图范围内',
        "task contract owns waypoint/map bounds checks before motion",
    )
    require(
        task_contract,
        r'def pose_map_occupancy_error[\s\S]{0,1800}map_value[\s\S]{0,1800}pose_on_occupied_cell[\s\S]{0,1800}pose_on_unknown_cell',
        "task contract owns waypoint occupancy-grid checks before motion",
    )
    require(
        task_contract,
        r'def map_metadata_mismatch_error[\s\S]{0,2200}width[\s\S]{0,1200}resolution[\s\S]{0,1200}origin_x[\s\S]{0,1200}origin_y[\s\S]{0,1800}网页选择地图与 Nav2 当前加载地图不一致',
        "task contract owns live/selected map metadata mismatch checks",
    )
    require(
        task_contract,
        r'def battery_readiness_payload[\s\S]{0,2600}battery_missing[\s\S]{0,2600}battery_stale[\s\S]{0,2600}battery_low',
        "task contract owns battery readiness decisions",
    )
    require(
        task_contract,
        r'def perception_readiness_payload[\s\S]{0,3200}perception_scan_unavailable[\s\S]{0,3200}perception_lidar_unavailable',
        "task contract owns scan/lidar readiness decisions",
    )
    require(
        task_contract,
        r'def runtime_guard_readiness_payload[\s\S]{0,1800}battery_readiness[\s\S]{0,1800}perception_readiness[\s\S]{0,1800}任务运行关键链路可用',
        "task contract owns runtime guard battery/perception composition",
    )
    require(
        task_contract,
        r'def task_runtime_readiness_payload[\s\S]{0,1200}map_relocalization_readiness[\s\S]{0,1200}pose_readiness[\s\S]{0,1600}battery_readiness[\s\S]{0,1600}perception_readiness[\s\S]{0,1200}success_message',
        "task contract owns task-start pose/battery/perception readiness composition",
    )
    require(
        task_contract,
        r'def current_task_readiness_payload[\s\S]{0,1200}task_running[\s\S]{0,1600}navigation_not_ready[\s\S]{0,1200}定位、位姿和导航链路已就绪',
        "task contract owns current task-page readiness composition",
    )
    require(
        task_contract,
        r'def task_readiness_pre_runtime_payload[\s\S]{0,1600}task_running[\s\S]{0,1800}task_static_context_invalid[\s\S]{0,1800}selected_map_mismatch',
        "task contract owns task-list pre-runtime startability composition",
    )
    require(
        task_contract,
        r'def runtime_guard_lost_decision[\s\S]{0,1600}"action": "clear"[\s\S]{0,1800}runtime_guard_lost[\s\S]{0,1800}"action": "fail"',
        "task contract owns runtime guard lost wait/stop decisions",
    )
    require(
        task_contract,
        r'def runtime_guard_failure_extra[\s\S]{0,500}runtime_guard[\s\S]{0,500}runtime_guard_lost_age_s',
        "task contract owns runtime guard failure extras",
    )
    require(
        task_contract,
        r'def runtime_guard_waiting_event_payload[\s\S]{0,1000}runtime_guard_waiting[\s\S]{0,1000}age_s[\s\S]{0,700}timeout_s',
        "task contract owns runtime guard waiting timeline event payload",
    )
    require(
        task_contract,
        r'def apply_runtime_guard_wait_state[\s\S]{0,1800}runtime_guard_lost_started_monotonic[\s\S]{0,1200}last_wait_code[\s\S]{0,1200}should_record_event',
        "task contract owns runtime guard waiting state updates",
    )
    require(
        task_contract,
        r'def apply_runtime_guard_clear_state[\s\S]{0,800}clear_keys[\s\S]{0,700}"changed"',
        "task contract owns runtime guard recovered-clear state updates",
    )
    require(
        web,
        r'from \.perception_contract import perception_status_payload',
        "web imports the pure perception status contract",
    )
    require(
        web,
        r'from \.annotation_contract import \([\s\S]{0,320}annotation_map_pose_error_payload[\s\S]{0,320}annotation_create_readiness_payload[\s\S]{0,320}annotation_semantics_payload[\s\S]{0,320}normalize_annotation_semantics',
        "web imports the pure annotation readiness and semantics contract",
    )
    require(
        web,
        r'from \.map_selection_contract import \([\s\S]{0,260}apply_selected_map_choice_state[\s\S]{0,260}map_relocalization_required_payload[\s\S]{0,260}selected_map_status_payload[\s\S]{0,260}selected_map_wait_timeout_payload',
        "web imports the pure selected-map status contract",
    )
    require(
        web,
        r'from \.startup_map_sync_contract import \([\s\S]{0,320}startup_map_sync_missing_record_payload[\s\S]{0,320}startup_map_sync_retry_decision[\s\S]{0,320}startup_map_sync_result_payload[\s\S]{0,320}startup_map_sync_skipped_payload',
        "web imports the pure startup map sync status contract",
    )
    require(
        web,
        r'def _snapshot\(self\)[\s\S]{0,1400}snapshot\["perception_status"\]\s*=\s*perception_status_payload\(snapshot,\s*now=now,\s*now_text=now_text\)',
        "web state exposes a structured perception status diagnostic",
    )
    require(
        perception_contract,
        r'def perception_status_payload[\s\S]{0,2600}factory_lidar_points_publisher_missing[\s\S]{0,2400}lidar_relay_no_samples[\s\S]{0,1800}scan_unavailable[\s\S]{0,1400}perception_ready',
        "perception contract distinguishes factory lidar publisher loss, relay loss, scan loss and ready state",
    )
    forbid(
        web,
        r'def _perception_status_payload|def _safe_float|def _safe_int',
        "web node no longer owns perception status or generic safe-cast helpers",
    )
    require(
        perception_contract_test,
        r'test_factory_lidar_publisher_missing[\s\S]{0,1200}factory_lidar_points_publisher_missing[\s\S]{0,1800}test_scan_unavailable[\s\S]{0,1600}test_perception_ready',
        "offline perception contract tests cover hard lidar, relay, scan and ready states",
    )
    require(
        task_contract,
        r'def task_pose_readiness_payload[\s\S]{0,2600}first_waypoint_distance_m[\s\S]{0,2600}localization_not_confirmed[\s\S]{0,2200}pose_invalid_or_stale',
        "task contract owns pose/localization/first-waypoint readiness decisions",
    )
    require(
        task_contract,
        r'def task_start_runtime_readiness_payload[\s\S]{0,1600}floor_unknown[\s\S]{0,1600}wrong_floor[\s\S]{0,1800}current_pose_out_of_map[\s\S]{0,1800}target_out_of_map[\s\S]{0,1800}map_metadata_mismatch[\s\S]{0,1800}first_waypoint_too_far[\s\S]{0,1800}navigation_not_ready',
        "task contract owns final task-start runtime readiness composition",
    )
    require(
        task_contract,
        r'def map_relocalization_task_readiness_payload[\s\S]{0,1200}map_relocalization_required[\s\S]{0,900}开发手册2101',
        "task contract owns map-relocalization-required task readiness decisions",
    )
    require(
        task_contract,
        r'def validate_task_annotations_for_map[\s\S]{0,5200}waypoint_map_mismatch[\s\S]{0,5200}waypoint_floor_mixed[\s\S]{0,5200}waypoint_on_occupied_cell[\s\S]{0,1400}waypoint_on_unknown_cell',
        "task contract owns task waypoint list validation",
    )
    require(
        task_contract,
        r'def validate_task_create_map_selection[\s\S]{0,1400}selected_map_missing[\s\S]{0,1400}live_map_task_disabled[\s\S]{0,1400}task_create_map_mismatch',
        "task contract owns current-map-only task creation rules",
    )
    require(
        task_contract,
        r'def task_create_map_metadata_mismatch_payload[\s\S]{0,1400}task_create_map_metadata_mismatch[\s\S]{0,1000}readiness_error_payload',
        "task contract owns task creation selected-map/Nav2-map mismatch payloads",
    )
    require(
        task_contract,
        r'def task_create_static_context[\s\S]{0,1800}task_create_no_waypoint[\s\S]{0,1800}task_create_missing_waypoint[\s\S]{0,1800}validate_task_create_map_selection[\s\S]{0,1200}def build_task_create_record',
        "task contract owns static task creation context and record construction",
    )
    require(
        task_contract,
        r'def task_start_static_context[\s\S]{0,2600}task_status_allows_start[\s\S]{0,2600}"readiness"[\s\S]{0,2600}mark_task_invalid[\s\S]{0,2200}first_annotation',
        "task contract owns static task-start context and readiness validation before runtime checks",
    )
    require(
        task_contract,
        r'def task_readiness_pre_runtime_payload[\s\S]{0,1800}static_context[\s\S]{0,1800}task_validation[\s\S]{0,1800}selected_map_mismatch[\s\S]{0,1400}"proceed": True',
        "task contract owns task-card readiness pre-runtime decisions",
    )
    require(
        task_contract,
        r'def apply_task_start_pre_runtime_failure_state[\s\S]{0,1200}mark_task_invalid[\s\S]{0,1200}task_validation[\s\S]{0,1200}"invalid"[\s\S]{0,900}last_error',
        "task contract owns task-start pre-runtime invalid task state updates",
    )
    require(
        task_contract,
        r'def apply_deleted_annotation_to_tasks[\s\S]{0,1600}annotation_ids[\s\S]{0,1200}"invalid"[\s\S]{0,900}"affected_tasks"',
        "task contract owns task list state updates after deleting an annotation",
    )
    require(
        task_contract,
        r'def apply_task_name_update[\s\S]{0,1800}active_task[\s\S]{0,1200}settings_changed[\s\S]{0,900}def apply_task_delete[\s\S]{0,1800}task_running[\s\S]{0,1200}deleted_task_id',
        "task contract owns task rename/delete state updates",
    )
    require(
        task_contract,
        r'def stop_stale_running_tasks[\s\S]{0,1200}"running"[\s\S]{0,900}"stopped"[\s\S]{0,900}"stopped_task_ids"',
        "task contract owns stale running task cleanup for task list state",
    )
    require(
        task_contract,
        r'def task_list_filter_payload[\s\S]{0,1200}selected_map_id[\s\S]{0,1200}hidden_task_count[\s\S]{0,900}total_task_count',
        "task contract owns current-map task-list filtering and hidden task counts",
    )
    require(
        task_contract,
        r'def normalize_startup_task_runtime_state[\s\S]{0,1200}"active_task"[\s\S]{0,1600}stop_stale_running_tasks\([\s\S]{0,900}"cleared_active_task"',
        "task contract owns startup active-task clearing and running-task normalization",
    )
    require(
        annotation_contract,
        r'def annotation_create_readiness_payload[\s\S]{0,1600}annotation_fixed_map_required[\s\S]{0,1200}annotation_map_metadata_mismatch[\s\S]{0,1200}annotation_map_relocalization_required[\s\S]{0,1200}annotation_localization_not_confirmed[\s\S]{0,1200}annotation_pose_invalid_or_stale',
        "annotation contract owns fixed-map, selected-map, relocalization and pose readiness for point saving",
    )
    require(
        annotation_contract,
        r'def annotation_create_static_context[\s\S]{0,1200}annotation_pose_invalid[\s\S]{0,1200}annotation_floor_missing[\s\S]{0,1200}def build_annotation_record[\s\S]{0,1800}normalize_annotation_semantics\(item\)',
        "annotation contract owns static point payload parsing and annotation record construction",
    )
    require(
        annotation_contract,
        r'def annotation_map_pose_error_payload[\s\S]{0,900}pose_map_bounds_error[\s\S]{0,900}annotation_out_of_map[\s\S]{0,900}pose_map_occupancy_error[\s\S]{0,1200}annotation_on_occupied_cell[\s\S]{0,900}annotation_on_unknown_cell',
        "annotation contract owns fixed-map bounds and occupancy errors for point saving",
    )
    require(
        annotation_contract,
        r'def annotation_list_filter_payload[\s\S]{0,1200}hidden_annotation_count[\s\S]{0,900}total_annotation_count',
        "annotation contract owns annotation-list filtering and hidden old-map counts",
    )
    require(
        annotation_contract_test,
        r'test_requires_fixed_map[\s\S]{0,900}test_blocks_map_mismatch[\s\S]{0,900}test_blocks_selected_map_metadata_mismatch[\s\S]{0,900}test_blocks_required_relocalization[\s\S]{0,900}test_blocks_unconfirmed_or_stale_pose[\s\S]{0,900}test_ready[\s\S]{0,1600}test_annotation_create_static_context[\s\S]{0,1600}test_build_annotation_record[\s\S]{0,1600}test_annotation_map_pose_error_payload[\s\S]{0,1600}test_annotation_semantics_normalization[\s\S]{0,1600}test_payload_helpers[\s\S]{0,1600}test_annotation_semantics_payload[\s\S]{0,1200}test_annotation_list_filter_payload',
        "offline annotation contract tests cover point-saving readiness, static point creation, semantics normalization, payload helpers and list filtering",
    )
    require(
        web,
        r'def _task_battery_readiness_payload[\s\S]{0,700}return battery_readiness_payload\(',
        "web delegates battery readiness decisions to task contract",
    )
    require(
        web,
        r'def _task_perception_readiness_payload[\s\S]{0,900}return perception_readiness_payload\(',
        "web delegates perception readiness decisions to task contract",
    )
    require(
        web,
        r'def _task_runtime_readiness_payload[\s\S]{0,1800}pose_readiness = task_pose_readiness_payload\(',
        "web delegates pose/localization/first-waypoint readiness decisions to task contract",
    )
    require(
        web,
        r'def _task_runtime_readiness_payload[\s\S]{0,4200}return task_runtime_readiness_payload\(',
        "web delegates task-start pose/battery/perception readiness composition to task contract",
    )
    require(
        web,
        r'def _current_task_readiness_payload[\s\S]{0,2200}return contract_current_task_readiness_payload\(',
        "web delegates current task-page readiness composition to task contract",
    )
    require(
        web,
        r'def _task_start_readiness_payload[\s\S]{0,2600}return task_start_runtime_readiness_payload\(',
        "web delegates final task-start runtime readiness composition to task contract",
    )
    require(
        web,
        r'def _task_readiness_for_task[\s\S]{0,1800}static_context\s*=\s*task_start_static_context\([\s\S]{0,1200}task_readiness_pre_runtime_payload\(',
        "web task list readiness delegates pre-runtime startability composition to the task contract",
    )
    require(
        web,
        r'def _validate_task_annotations_for_map[\s\S]{0,900}contract_validate_task_annotations_for_map\(',
        "web delegates task waypoint list validation to task contract",
    )
    require(
        web,
        r'def _tasks_payload\(self,\s*query:[\s\S]{0,3200}task_list_filter_payload\([\s\S]{0,1200}"hidden_task_count": task_list\["hidden_task_count"\][\s\S]{0,800}"total_task_count": task_list\["total_task_count"\]',
        "web tasks API delegates current-map task filtering and hidden old-map counts to task contract",
    )
    require(
        web,
        r'def _tasks_payload\(self,\s*query:[\s\S]{0,1400}stop_stale_running_tasks\(',
        "web tasks API delegates stale running task cleanup to the task contract",
    )
    require(
        web,
        r'def _normalize_runtime_state_on_startup[\s\S]{0,1800}normalize_startup_task_runtime_state\(',
        "web startup runtime normalization delegates task runtime state cleanup to the task contract",
    )
    require(
        web,
        r'def _snapshot\(self\)[\s\S]{0,1800}map_status_runtime = \{"map": dict\(self\._state\.get\("map"\) or \{\}\)\}[\s\S]{0,1000}snapshot\["selected_map_status"\] = self\._selected_map_status_payload\(map_status_runtime\)',
        "web state exposes selected-map versus Nav2-map status",
    )
    require(
        web,
        r'def _selected_map_status_payload[\s\S]{0,1200}return selected_map_status_payload\(',
        "web delegates selected-map status decisions to the map-selection contract",
    )
    require(
        map_selection_contract,
        r'def selected_map_status_payload[\s\S]{0,1600}selected_map_missing[\s\S]{0,1400}map_metadata_mismatch_error\(live_payload,\s*selected_payload\)[\s\S]{0,1200}selected_map_metadata_mismatch[\s\S]{0,800}readiness_success[\s\S]{0,1000}def map_relocalization_required_payload',
        "map-selection contract owns selected-map missing, mismatch, ready and relocalization-required states",
    )
    require(
        map_selection_contract,
        r'def apply_selected_map_choice_state[\s\S]{0,1600}selected_map_id[\s\S]{0,1400}map_relocalization_required[\s\S]{0,1200}clear_pose',
        "map-selection contract owns selected-map choice state updates",
    )
    require(
        map_selection_contract,
        r'def selected_map_wait_timeout_payload[\s\S]{0,800}selected_map_metadata_mismatch[\s\S]{0,600}等待 Nav2 /map 更新超时',
        "map-selection contract owns selected-map wait timeout payloads",
    )
    require(
        map_selection_contract_test,
        r'test_selected_map_missing[\s\S]{0,900}test_selected_map_ready[\s\S]{0,900}test_selected_map_metadata_mismatch[\s\S]{0,900}test_live_map_unavailable[\s\S]{0,900}test_map_relocalization_required_payload[\s\S]{0,1400}test_selected_map_wait_timeout_payload[\s\S]{0,900}test_apply_selected_map_choice_state',
        "offline map-selection contract tests cover missing, ready, metadata mismatch, live-map unavailable, relocalization-required, wait timeout and selected-map choice states",
    )
    require(
        startup_map_sync_contract,
        r'def startup_map_sync_skipped_payload[\s\S]{0,900}def startup_map_sync_missing_record_payload[\s\S]{0,900}def startup_map_sync_result_payload[\s\S]{0,900}def startup_map_sync_retry_decision[\s\S]{0,900}load_map_service_unavailable[\s\S]{0,900}load_map_timeout',
        "startup map sync contract owns skipped, missing-record, LoadMap result payloads and delayed Nav2 retry decisions",
    )
    require(
        startup_map_sync_contract_test,
        r'test_skipped_payload[\s\S]{0,900}test_missing_record_payload[\s\S]{0,900}test_result_payload[\s\S]{0,900}test_retry_decision_for_delayed_nav2_load_map_service',
        "offline startup map sync contract tests cover skipped, missing-record, LoadMap result payloads and delayed Nav2 retries",
    )
    require(
        web,
        r'def _tasks_payload\(self,\s*query:[\s\S]{0,2800}"selected_map_status": self\._selected_map_status_payload',
        "web tasks API exposes selected-map status for task-page gating",
    )
    require(
        web,
        r'elif parsed\.path == "/api/tasks":[\s\S]{0,180}node\._tasks_payload\(query\)',
        "web tasks API passes query flags only for explicit history/audit requests",
    )
    require(
        web,
        r'def _create_annotation[\s\S]{0,2200}annotation_map_pose_error_payload\(point_pose,\s*target_map_payload\)[\s\S]{0,700}map_pose_error\["message"\][\s\S]{0,300}map_pose_error\["code"\]',
        "web delegates saved-annotation fixed-map bounds and occupancy validation to annotation contract",
    )
    forbid(
        web,
        r'def _create_annotation[\s\S]{0,2600}pose_map_bounds_error\(point_pose,\s*target_map_payload,\s*"保存点位"\)|def _create_annotation[\s\S]{0,2600}pose_map_occupancy_error\(point_pose,\s*target_map_payload,\s*"保存点位"\)',
        "web no longer combines annotation map bounds/occupancy checks inline",
    )
    require(
        web,
        r'def _create_annotation[\s\S]{0,900}annotation_create_static_context\([\s\S]{0,2600}build_annotation_record\(',
        "web annotation creation delegates static point payload parsing and record construction to annotation contract",
    )
    require(
        web,
        r'def _create_annotation[\s\S]{0,900}context\["message"\][\s\S]{0,260}context\["code"\][\s\S]{0,1600}annotation_readiness\["message"\]',
        "web annotation creation consumes required annotation-contract messages",
    )
    forbid(
        web,
        r'def _create_annotation[\s\S]{0,2200}点位数据无效|def _create_annotation[\s\S]{0,2600}当前不能保存点位|def _create_annotation[\s\S]{0,3000}点位不在当前地图范围内|def _create_annotation[\s\S]{0,3400}点位不在可通行栅格上',
        "web annotation creation no longer hardcodes annotation-contract fallback messages",
    )
    require(
        web,
        r'def _create_annotation[\s\S]{0,1800}_annotation_create_readiness_payload\(map_id,\s*selected_map_id\)[\s\S]{0,500}readiness_error_payload\(annotation_readiness\)',
        "web annotation API rejects point saving before localization readiness is confirmed",
    )
    require(
        web,
        r'def _annotation_create_readiness_payload[\s\S]{0,1200}selected_map_status = self\._selected_map_status_payload[\s\S]{0,1400}return annotation_create_readiness_payload\(',
        "web annotation readiness delegates point-saving readiness rules to the annotation contract",
    )
    require(
        web,
        r'def _annotations_payload\(self,\s*query:[\s\S]{0,600}return annotation_list_filter_payload\(annotations,\s*map_id=map_id\)',
        "web annotations API delegates map filtering to the annotation contract",
    )
    require(
        web,
        r'def _create_task[\s\S]{0,1200}task_create_static_context\([\s\S]{0,2600}build_task_create_record\(',
        "web task creation delegates static task creation context and record construction to task contract",
    )
    require(
        web,
        r'def _create_task[\s\S]{0,1200}error_payload\["message"\][\s\S]{0,1800}readiness\["message"\]',
        "web task creation consumes required task-contract messages instead of fallback messages",
    )
    forbid(
        web,
        r'def _create_task[\s\S]{0,2600}任务静态条件无效|def _create_task[\s\S]{0,2600}任务点位无效',
        "web task creation no longer hardcodes task-contract fallback messages",
    )
    require(
        web,
        r'def _task_start_pre_runtime_context[\s\S]{0,1400}task_start_static_context\([\s\S]{0,1400}task_readiness_pre_runtime_payload\([\s\S]{0,1800}apply_task_start_pre_runtime_failure_state',
        "web task start pre-runtime helper delegates static validation and invalid-task updates to the task contract",
    )
    require(
        web,
        r'def _task_start_pre_runtime_context[\s\S]{0,2600}readiness\["message"\]',
        "web task start pre-runtime helper consumes required task-contract messages",
    )
    forbid(
        web,
        r'def _task_start_pre_runtime_context[\s\S]{0,3200}任务启动条件无效',
        "web task start pre-runtime helper no longer hardcodes fallback messages",
    )
    require(
        web,
        r'def _task_start_pre_runtime_context[\s\S]{0,2600}validate_task_start_expectations\(\s*payload,\s*task,\s*first_annotation,\s*task_map_id',
        "web task start pre-runtime helper validates frontend start expectations through the task contract",
    )
    require(
        web,
        r'def _delete_annotation[\s\S]{0,1800}apply_deleted_annotation_to_tasks\(',
        "web annotation deletion delegates affected-task state updates to the task contract",
    )
    require(
        web,
        r'def _update_task[\s\S]{0,1200}apply_task_name_update\([\s\S]{0,900}settings_changed[\s\S]{0,900}def _delete_task[\s\S]{0,1200}apply_task_delete\(',
        "web task update/delete delegates task state changes to the task contract",
    )
    require(
        web,
        r'def _update_task[\s\S]{0,1400}update\["message"\][\s\S]{0,300}update\["code"\][\s\S]{0,1000}def _delete_task[\s\S]{0,900}delete\["message"\][\s\S]{0,300}delete\["code"\]',
        "web task update/delete consumes required task-contract messages",
    )
    forbid(
        web,
        r'def _update_task[\s\S]{0,1600}return self\._error\("任务不存在"\)|def _delete_task[\s\S]{0,1200}delete\.get\("message"\)[\s\S]{0,180}任务不存在',
        "web task update/delete no longer hardcodes task-contract fallback messages",
    )
    forbid(
        web,
        r'def _mark_task_status|_mark_task_status\(',
        "web does not keep a separate task-status update path beside task result persistence",
    )
    require(
        web,
        r'def _create_task[\s\S]{0,2600}_selected_map_status_payload[\s\S]{0,900}task_create_map_metadata_mismatch_payload\(',
        "web task creation rejects selected-map/Nav2-map mismatch before storing tasks",
    )
    forbid(
        web,
        r'def _create_task[\s\S]{0,3200}task_create_map_metadata_mismatch"[\s\S]{0,900}readiness_failure\(',
        "web task creation no longer assembles selected-map/Nav2-map mismatch readiness inline",
    )
    require(
        web,
        r'from nav2_msgs\.srv import ClearEntireCostmap,\s*LoadMap',
        "web dashboard imports Nav2 LoadMap for selected fixed maps",
    )
    require(
        web,
        r'declare_parameter\("map_server_load_map_service",\s*"/map_server/load_map"\)[\s\S]{0,300}declare_parameter\("map_select_load_nav2_map",\s*True\)',
        "web dashboard declares selected-map Nav2 LoadMap controls",
    )
    require(
        web,
        r'declare_parameter\("startup_sync_selected_map_to_nav2",\s*True\)[\s\S]{0,500}declare_parameter\("startup_sync_selected_map_max_attempts",\s*12\)',
        "web dashboard syncs the saved selected fixed map back into Nav2 after startup with a long enough retry window",
    )
    require(
        web,
        r'def _sync_selected_map_to_nav2_on_startup[\s\S]{0,500}threading\.Thread\(target=self\._run_startup_selected_map_sync[\s\S]{0,900}def _run_startup_selected_map_sync_once[\s\S]{0,2600}_load_selected_map_into_nav2\(record\)',
        "web startup selected-map sync runs LoadMap from a background thread instead of blocking the ROS timer callback",
    )
    require(
        web,
        r'def _run_startup_selected_map_sync_once[\s\S]{0,2600}map_relocalization_required_payload\([\s\S]{0,400}reason="startup_sync"',
        "web startup selected-map sync delegates relocalization-required state to the map-selection contract",
    )
    require(
        web,
        r'def _run_startup_selected_map_sync_once[\s\S]{0,2600}startup_map_sync_skipped_payload[\s\S]{0,1800}startup_map_sync_missing_record_payload[\s\S]{0,2200}startup_map_sync_result_payload',
        "web delegates startup selected-map sync status payloads to the startup map sync contract",
    )
    require(
        web,
        r'def _run_startup_selected_map_sync_once[\s\S]{0,4600}startup_map_sync_retry_decision\([\s\S]{0,900}retry\.get\("retry"\)[\s\S]{0,900}finish_timer\(\)',
        "web startup selected-map sync keeps retrying delayed Nav2 load_map availability instead of giving up early",
    )
    require(
        web,
        r'def _run_startup_selected_map_sync_once[\s\S]{0,3900}else:[\s\S]{0,180}启动同步固定地图失败[\s\S]{0,220}warning\(str\(load_result\["message"\]\)\)',
        "web startup selected-map sync consumes required LoadMap failure messages",
    )
    forbid(
        web,
        r'def _run_startup_selected_map_sync_once[\s\S]{0,4300}load_result\.get\("message"\)\s*or\s*"startup selected-map sync failed"|def _run_startup_selected_map_sync_once[\s\S]{0,4300}"startup selected-map sync failed"',
        "web startup selected-map sync no longer falls back from LoadMap failure messages",
    )
    require(
        web,
        r'def _snapshot\(self\)[\s\S]{0,1800}snapshot\["startup_map_sync"\]\s*=\s*self\._settings\.get\("startup_map_sync"\)',
        "web state exposes startup selected-map sync evidence",
    )
    require(
        web,
        r'create_client\(\s*LoadMap,[\s\S]{0,300}get_parameter\("map_server_load_map_service"\)',
        "web dashboard creates a Nav2 LoadMap client",
    )
    require(
        web,
        r'def _select_map[\s\S]{0,900}active\.get\("status"\) == "running"[\s\S]{0,1800}_load_selected_map_into_nav2\(record\)[\s\S]{0,1400}apply_selected_map_choice_state',
        "web map selection loads Nav2 map only after rejecting active-task changes and then delegates selected-map state updates",
    )
    require(
        web,
        r'def _select_map[\s\S]{0,1800}if not nav2_load\.get\("ok"\):[\s\S]{0,220}str\(nav2_load\["message"\]\)',
        "web map selection consumes required Nav2 load failure messages",
    )
    forbid(
        web,
        r'def _select_map[\s\S]{0,2200}nav2_load\.get\("message"\)\s*or\s*"Nav2 地图加载失败"|def _select_map[\s\S]{0,2200}"Nav2 地图加载失败"',
        "web map selection no longer falls back from Nav2 load failure messages",
    )
    require(
        web,
        r'request = LoadMap\.Request\(\)[\s\S]{0,200}request\.map_url = str\(yaml_path\)[\s\S]{0,300}self\.load_map_client\.call_async\(request\)',
        "web selected-map loader calls Nav2 LoadMap with the selected yaml path",
    )
    require(
        map_contract,
        r'def ensure_map_yaml_uses_local_image[\s\S]{0,2600}write_map_yaml_image_reference',
        "map contract repairs archived map yaml image references to local relative files",
    )
    require_imports(
        web,
        "map_contract",
        [
            "all_map_records",
            "build_imported_map_record",
            "default_map_id",
            "ensure_map_yaml_uses_local_image",
            "find_map_record",
            "find_map_yaml",
            "load_builtin_maps_from_manifest",
            "load_map_file_payload",
            "map_file_fingerprint",
            "map_file_metadata_payload",
        ],
        "web imports pure map archive contract helpers",
    )
    require(
        map_contract,
        r'def load_builtin_maps_from_manifest[\s\S]{0,2200}default_floor[\s\S]{0,2200}"project_builtin"[\s\S]{0,1600}default_builtin_map_id',
        "map contract owns builtin map manifest parsing and record construction",
    )
    require(
        map_contract,
        r'def all_map_records[\s\S]{0,900}archived_ids[\s\S]{0,900}def find_map_record[\s\S]{0,1200}for item in archived_maps[\s\S]{0,1200}def default_map_id[\s\S]{0,1200}builtin_F20',
        "map contract owns map merge, lookup and default-map selection",
    )
    require(
        web,
        r'def _load_builtin_maps[\s\S]{0,900}load_builtin_maps_from_manifest\([\s\S]{0,600}derived_payload=builtin_map_derived_payload',
        "web builtin-map loader delegates manifest parsing to map contract",
    )
    require(
        web,
        r'def _all_maps_unlocked[\s\S]{0,260}all_map_records\(self\._builtin_maps,\s*self\._maps\)[\s\S]{0,420}def _find_map_record_unlocked[\s\S]{0,260}find_map_record\(self\._builtin_maps,\s*self\._maps,\s*map_id\)[\s\S]{0,420}def _default_map_id_unlocked[\s\S]{0,260}default_map_id\(self\._builtin_maps,\s*self\._maps,\s*self\._default_builtin_map_id\)',
        "web map list, lookup and default selection delegate to map contract",
    )
    forbid(
        web,
        r'yaml\.safe_load|manifest\.get\("map_set"\)|manifest\.get\("floors"\)|"builtin_F20"|def _load_builtin_maps[\s\S]{0,2200}"project_builtin"|def _default_map_id_unlocked[\s\S]{0,900}for item in self\._builtin_maps',
        "web no longer owns builtin map manifest parsing or default-map rules",
    )
    require(
        web,
        r'def _import_active_map[\s\S]{0,3600}ensure_map_yaml_uses_local_image\(yaml_path\)[\s\S]{0,800}地图已拉取，但栅格图文件不可用',
        "web active-map import delegates local occupancy image validation to map contract",
    )
    require(
        web,
        r'def _load_selected_map_into_nav2[\s\S]{0,1100}ensure_map_yaml_uses_local_image\(yaml_path\)[\s\S]{0,700}"image_repair": image_repair',
        "web selected-map loader delegates yaml image repair before Nav2 LoadMap",
    )
    forbid(
        web,
        r'def _ensure_map_yaml_uses_local_image|def _resolve_map_yaml_image_path|def _find_local_map_image|def _write_map_yaml_image_reference|def _read_pgm|def _load_map_file_payload|def _find_map_yaml',
        "web node no longer owns map yaml/image/PGM parsing helpers",
    )
    require(
        map_contract,
        r'def build_imported_map_record[\s\S]{0,900}"source": "106_active_map"[\s\S]{0,500}"created_at"',
        "map contract owns imported active-map record construction",
    )
    require(
        web,
        r'def _import_active_map[\s\S]{0,4200}build_imported_map_record\([\s\S]{0,800}created_at=now_text\(\)',
        "web active-map import delegates record construction to map contract",
    )
    forbid(
        web,
        r'def _import_active_map[\s\S]{0,5200}"source": "106_active_map"|def _import_active_map[\s\S]{0,5200}"created_at": _now_text\(\)',
        "web active-map import no longer hardcodes imported map record fields",
    )
    require(
        web,
        r'def _load_selected_map_into_nav2[\s\S]{0,1100}if not image_repair\.get\("ok"\):[\s\S]{0,260}"code": str\(image_repair\["code"\]\)[\s\S]{0,120}"message": str\(image_repair\["message"\]\)',
        "web selected-map loader consumes required image repair failure fields",
    )
    forbid(
        web,
        r'def _load_selected_map_into_nav2[\s\S]{0,1600}image_repair\.get\("message"\)\s*or|def _load_selected_map_into_nav2[\s\S]{0,1600}"地图栅格图不可用"',
        "web selected-map loader no longer falls back from image repair messages",
    )
    require(
        map_contract,
        r'def load_map_file_payload[\s\S]{0,420}if not image_repair\.get\("ok"\):[\s\S]{0,160}RuntimeError\(str\(image_repair\["message"\]\)\)',
        "map contract snapshot loader consumes required image repair failure message",
    )
    require(
        map_contract,
        r'def read_pgm_header[\s\S]{0,900}return width, height, max_value[\s\S]{0,400}def map_file_metadata_payload[\s\S]{0,1800}"origin"[\s\S]{0,500}def map_file_fingerprint',
        "map contract provides lightweight map metadata and fingerprint helpers",
    )
    require(
        web,
        r'_map_file_cache_lock = threading\.Lock\(\)[\s\S]{0,200}_map_file_summary_cache',
        "web caches lightweight selected-map metadata for state snapshots",
    )
    require(
        web,
        r'def _map_file_summary[\s\S]{0,900}map_file_fingerprint\(yaml_path\)[\s\S]{0,900}self\._map_file_summary_cache\.get\(cache_key\)',
        "web reuses selected-map metadata summaries while map files are unchanged",
    )
    require(
        web,
        r'def _map_file_summary[\s\S]{0,1400}map_file_metadata_payload\(record, yaml_path\)[\s\S]{0,700}self\._map_file_summary_cache\[cache_key\]',
        "web fills selected-map status cache from lightweight metadata",
    )
    require(
        web,
        r'def _selected_map_status_payload[\s\S]{0,1000}selected_map = self\._map_file_summary\(selected_map_id\) if selected_map_id else \{\}',
        "web selected-map status uses lightweight map metadata instead of full occupancy data",
    )
    forbid(
        web,
        r'image_repair\.get\("message"\)\s*or|def _map_file_snapshot[\s\S]{0,700}"map image unavailable"',
        "web map snapshot no longer falls back from image repair messages",
    )
    require(
        map_contract_test,
        r'test_load_builtin_maps_from_manifest[\s\S]{0,2200}test_map_record_merge_find_and_default[\s\S]{0,2200}test_ensure_map_yaml_repairs_to_local_relative_image[\s\S]{0,1800}test_read_pgm_p2_and_p5[\s\S]{0,1800}test_map_file_metadata_payload_is_lightweight[\s\S]{0,2200}test_map_file_fingerprint_tracks_yaml_and_image[\s\S]{0,2200}test_load_map_file_payload_builds_nav_occupancy_grid[\s\S]{0,2200}test_build_imported_map_record',
        "offline map contract tests cover builtin manifest, map merge/defaults, yaml repair, PGM parsing, lightweight metadata, occupancy payload and imported record generation",
    )
    require_imports(
        web,
        "mapping_contract",
        ["apply_mapping_command_result", "mapping_command_context", "prepare_mapping_session_create"],
        "web imports pure mapping session contract helpers",
    )
    require(
        mapping_contract,
        r'def prepare_mapping_session_create[\s\S]{0,1500}build_mapping_project_record[\s\S]{0,900}build_mapping_session_record',
        "mapping contract owns mapping project/session record construction",
    )
    require(
        web,
        r'def _create_mapping_session[\s\S]{0,900}prepare_mapping_session_create\([\s\S]{0,1100}self\._sessions\.append\(session\)',
        "web mapping-session creation delegates record construction to mapping contract",
    )
    forbid(
        web,
        r'def _create_mapping_session[\s\S]{0,2200}"status": "created"|def _create_mapping_session[\s\S]{0,2200}"project_id": project\["id"\]',
        "web mapping-session creation no longer hardcodes project/session records",
    )
    require(
        mapping_contract,
        r'def mapping_command_status[\s\S]{0,900}"factory_mapping_start_command": "mapping"[\s\S]{0,400}"factory_mapping_finish_command": "saved"[\s\S]{0,400}"factory_mapping_cancel_command": "cancelled"',
        "mapping contract owns mapping command status transitions",
    )
    require(
        web,
        r'def _mapping_command[\s\S]{0,900}apply_mapping_command_result\([\s\S]{0,800}self\._save_json\("mapping_sessions\.json"',
        "web mapping command delegates session status updates to mapping contract",
    )
    forbid(
        web,
        r'def _mapping_command[\s\S]{0,1500}"factory_mapping_start_command": "mapping"|def _mapping_command[\s\S]{0,1500}"waiting_manual"',
        "web mapping command no longer hardcodes mapping status transitions",
    )
    require(
        mapping_contract,
        r'def mapping_command_context[\s\S]{0,1200}"factory_active_map"[\s\S]{0,400}"map_archive_dir"',
        "mapping contract owns factory mapping command context fields",
    )
    require(
        web,
        r'def _command_context[\s\S]{0,500}mapping_command_context\([\s\S]{0,500}factory_active_map=str\(self\.get_parameter\("factory_active_map"\)\.value\)',
        "web command context delegates field assembly to mapping contract",
    )
    forbid(
        web,
        r'def _command_context[\s\S]{0,1000}"session_id": str\(session\.get|def _command_context[\s\S]{0,1000}"floors": ","\.join',
        "web command context no longer assembles mapping command fields inline",
    )
    require(
        mapping_contract_test,
        r'test_normalize_mapping_session_request[\s\S]{0,1800}test_prepare_mapping_session_create_reuses_project[\s\S]{0,1800}test_mapping_command_status[\s\S]{0,1800}test_mapping_command_context',
        "offline mapping contract tests cover request normalization, records, status transitions and command context",
    )
    require(
        web,
        r'def _load_selected_map_into_nav2[\s\S]{0,3600}_clear_task_costmaps\("select_map_load_nav2"\)[\s\S]{0,300}_wait_for_selected_map_match\(selected_map\)',
        "web selected-map loader clears costmaps and waits for /map metadata match",
    )
    require(
        web,
        r'def _load_selected_map_into_nav2[\s\S]{0,4200}_wait_for_selected_map_match\(selected_map\)[\s\S]{0,600}"message": str\(match\["message"\]\)',
        "web selected-map loader consumes required selected-map match messages",
    )
    forbid(
        web,
        r'def _load_selected_map_into_nav2[\s\S]{0,4800}Nav2 地图已加载，但 /map 元数据尚未对齐|def _load_selected_map_into_nav2[\s\S]{0,4800}match\.get\("message"\)',
        "web selected-map loader no longer falls back from selected-map match messages",
    )
    require(
        web,
        r'def _wait_for_selected_map_match[\s\S]{0,1200}selected_map_status_payload\([\s\S]{0,700}if last_status\.get\("ready"\):[\s\S]{0,700}selected_map_wait_timeout_payload\(',
        "web selected-map match waits through the same selected-map status contract",
    )
    forbid(
        web,
        r'def _wait_for_selected_map_match[\s\S]{0,1800}等待 Nav2 /map 更新超时|def _wait_for_selected_map_match[\s\S]{0,1800}readiness_failure\(',
        "web selected-map match no longer assembles timeout readiness inline",
    )
    require(
        web,
        r'def _task_runtime_readiness_payload[\s\S]{0,1700}map_relocalization_task_readiness_payload\([\s\S]{0,2600}map_relocalization_readiness=map_relocalization_readiness',
        "web passes selected-map relocalization task readiness into the runtime contract",
    )
    require(
        web,
        r'def _start_task[\s\S]{0,900}readiness\["message"\][\s\S]{0,1800}post_reset_readiness\["message"\][\s\S]{0,1400}post_reset_task_ready\["message"\][\s\S]{0,1600}final_readiness\["message"\]',
        "web task start consumes required readiness contract messages through all start gates",
    )
    forbid(
        web,
        r'def _start_task[\s\S]{0,7000}任务链路未就绪|def _start_task[\s\S]{0,7000}任务启动复位后导航链路未恢复|def _start_task[\s\S]{0,7000}任务启动复位后位姿/任务条件失效，未下发目标|def _start_task[\s\S]{0,7000}任务下发前最终条件失效，未下发目标',
        "web task start no longer hardcodes readiness fallback messages",
    )
    require(
        dashboard,
        r'当前地图还没有选中的任务点[\s\S]{0,400}api\("POST", "/api/tasks"',
        "frontend blocks empty task creation before calling the API",
    )
    require(
        frontend_task_smoke,
        r'saveMarkButton[\s\S]{0,700}useRobotPoseButton[\s\S]{0,1800}save-mark button must be disabled until localization is confirmed[\s\S]{0,1000}final relocalization success',
        "frontend smoke verifies mark controls stay disabled before localization confirmation",
    )
    require(
        frontend_task_smoke,
        r'tasksPayloadIncludeAll[\s\S]{0,1800}default /api/tasks payload must not include all historical tasks[\s\S]{0,700}hidden old-map task count',
        "frontend smoke verifies default tasks API hides old-map tasks",
    )
    require(
        frontend_task_ready_check,
        r'tasks_path = "/api/tasks\?include_all=1" if \(args\.task_id or args\.task_name\) else "/api/tasks"',
        "frontend task ready check only uses all-task history when explicitly filtered",
    )
    require(
        frontend_task_ready_check,
        r'def state_level_advice[\s\S]{0,900}perception_status[\s\S]{0,700}factory_lidar_points_publisher_missing[\s\S]{0,1200}selected_map_status[\s\S]{0,700}selected_map_metadata_mismatch[\s\S]{0,900}localization_status',
        "frontend task ready check prioritizes hard perception faults and selected-map/Nav2-map mismatch before relocalization advice",
    )
    forbid(
        web,
        r'def _map_metadata_mismatch_error',
        "web node no longer owns live/selected map metadata mismatch rules",
    )
    require(
        web,
        r'task_start_runtime_readiness_payload\([\s\S]{0,900}live_map=live_map[\s\S]{0,900}target_map_payload=target_map_payload',
        "web delegates live/selected map metadata mismatch checks through task-start runtime contract",
    )
    forbid(
        web,
        r'blocked_cells|unknown_cells|waypoint_floor_mixed|waypoint_on_occupied_cell|waypoint_on_unknown_cell',
        "web node no longer owns task waypoint list validation branches",
    )
    require(
        task_contract_test,
        r'test_map_relocalization_task_readiness_payload[\s\S]{0,1200}map_relocalization_required[\s\S]{0,800}startup_sync',
        "offline task contract tests cover map relocalization task readiness",
    )
    require(
        task_contract_test,
        r'test_pose_map_bounds_error[\s\S]{0,1200}test_pose_map_occupancy_error[\s\S]{0,1600}pose_on_unknown_cell',
        "offline task contract tests cover map bounds and occupancy rules",
    )
    require(
        task_contract_test,
        r'test_validate_task_annotations_for_map[\s\S]{0,4200}waypoint_map_mismatch[\s\S]{0,4200}waypoint_floor_mixed[\s\S]{0,4200}waypoint_on_occupied_cell[\s\S]{0,1000}waypoint_on_unknown_cell',
        "offline task contract tests cover waypoint list validation rules",
    )
    require(
        task_contract_test,
        r'test_validate_task_create_map_selection[\s\S]{0,1200}selected_map_missing[\s\S]{0,1200}live_map_task_disabled[\s\S]{0,1200}task_create_map_mismatch',
        "offline task contract tests cover current-map-only task creation rules",
    )
    require(
        task_contract_test,
        r'test_task_create_map_metadata_mismatch_payload[\s\S]{0,1800}task_create_map_metadata_mismatch[\s\S]{0,1200}error extra wraps readiness',
        "offline task contract tests cover task creation selected-map/Nav2-map mismatch payloads",
    )
    require(
        task_contract_test,
        r'test_task_create_static_context[\s\S]{0,1600}task_create_no_waypoint[\s\S]{0,1600}task_create_missing_waypoint[\s\S]{0,1600}waypoint_order_invalid[\s\S]{0,1600}task_create_map_mismatch[\s\S]{0,1200}test_build_task_create_record',
        "offline task contract tests cover static task creation context and record construction",
    )
    require(
        task_contract_test,
        r'test_task_start_static_context[\s\S]{0,1600}task_missing[\s\S]{0,1600}mark_task_invalid[\s\S]{0,1600}waypoint_order_invalid',
        "offline task contract tests cover static task-start context and readiness validation",
    )
    require(
        task_contract_test,
        r'test_apply_deleted_annotation_to_tasks[\s\S]{0,1600}invalid[\s\S]{0,1600}error task remains error[\s\S]{0,1200}missing annotation changes nothing',
        "offline task contract tests cover affected-task state updates after deleting an annotation",
    )
    require(
        task_contract_test,
        r'test_apply_task_name_update[\s\S]{0,1600}rename success code[\s\S]{0,1600}missing rename message[\s\S]{0,1200}test_apply_task_delete[\s\S]{0,1600}running active task cannot be deleted[\s\S]{0,1600}historical active task can be deleted[\s\S]{0,1200}missing task delete fails',
        "offline task contract tests cover task rename/delete state updates",
    )
    require(
        task_contract_test,
        r'test_stop_stale_running_tasks[\s\S]{0,1600}task_stale[\s\S]{0,1600}all running tasks stale without active task[\s\S]{0,1200}no running tasks changes nothing',
        "offline task contract tests cover stale running task cleanup",
    )
    require(
        task_contract_test,
        r'test_task_list_filter_payload[\s\S]{0,1000}all tasks visible[\s\S]{0,1000}current map task visible[\s\S]{0,1000}no selected map shows no current-map tasks',
        "offline task contract tests cover current-map task-list filtering and hidden counts",
    )
    require(
        task_contract_test,
        r'test_normalize_startup_task_runtime_state[\s\S]{0,1600}active task cleared[\s\S]{0,1600}running task ids stopped[\s\S]{0,1600}no runtime residue changes nothing',
        "offline task contract tests cover startup active-task clearing and running-task normalization",
    )
    require(
        task_contract_test,
        r'test_map_metadata_mismatch_error[\s\S]{0,1600}origin_y[\s\S]{0,800}Nav2 当前 /map 不可用',
        "offline task contract tests cover live/selected map metadata mismatch rules",
    )
    require(
        task_contract_test,
        r'test_task_start_runtime_readiness_payload[\s\S]{0,2600}floor_unknown[\s\S]{0,1800}wrong_floor[\s\S]{0,1800}current_pose_out_of_map[\s\S]{0,1800}target_out_of_map[\s\S]{0,1800}map_metadata_mismatch[\s\S]{0,1800}first_waypoint_too_far[\s\S]{0,1800}navigation_not_ready',
        "offline task contract tests cover final task-start runtime readiness composition",
    )
    require(
        task_contract_test,
        r'test_task_runtime_readiness_payload[\s\S]{0,1200}map_relocalization_required[\s\S]{0,1200}pose_invalid_or_stale[\s\S]{0,1200}battery_low[\s\S]{0,1200}perception_lidar_unavailable[\s\S]{0,1200}runtime readiness passes',
        "offline task contract tests cover task-start pose/battery/perception readiness composition",
    )
    require(
        task_contract_test,
        r'test_current_task_readiness_payload[\s\S]{0,1400}task_running[\s\S]{0,1200}battery_low[\s\S]{0,1200}navigation_not_ready[\s\S]{0,1200}current readiness passes',
        "offline task contract tests cover current task-page readiness composition",
    )
    require(
        task_contract_test,
        r'test_task_readiness_pre_runtime_payload[\s\S]{0,1200}task_running[\s\S]{0,2200}task_missing[\s\S]{0,2200}waypoint_pose_invalid[\s\S]{0,2200}selected_map_mismatch[\s\S]{0,2200}pre-runtime passes',
        "offline task contract tests cover task-list pre-runtime startability composition",
    )
    require(
        task_contract_test,
        r'test_apply_task_start_pre_runtime_failure_state[\s\S]{0,1200}missing waypoint invalidates task[\s\S]{0,1200}task validation failure invalidates task[\s\S]{0,1200}non-invalidating failure does not edit tasks',
        "offline task contract tests cover task-start pre-runtime invalid task state updates",
    )
    require(
        task_contract_test,
        r'test_battery_readiness_payload[\s\S]{0,2200}battery_missing[\s\S]{0,2200}battery_stale[\s\S]{0,2200}battery_low',
        "offline task contract tests cover battery readiness rules",
    )
    require(
        task_contract_test,
        r'test_perception_readiness_payload[\s\S]{0,2600}perception_scan_unavailable[\s\S]{0,2600}perception_lidar_unavailable',
        "offline task contract tests cover scan/lidar readiness rules",
    )
    require(
        task_contract_test,
        r'test_runtime_guard_readiness_payload[\s\S]{0,1800}battery_low[\s\S]{0,1800}perception_scan_unavailable',
        "offline task contract tests cover runtime guard battery/perception composition",
    )
    require(
        task_contract_test,
        r'test_perception_readiness_payload[\s\S]{0,2600}任务感知检查已关闭[\s\S]{0,600}disabled perception message',
        "offline task contract tests cover disabled perception readiness payload",
    )
    require(
        task_contract_test,
        r'test_runtime_guard_lost_decision[\s\S]{0,2400}runtime_guard_lost[\s\S]{0,1800}runtime_guard_failure_extra[\s\S]{0,2200}test_apply_runtime_guard_wait_state[\s\S]{0,2600}should_record_event[\s\S]{0,1800}test_apply_runtime_guard_clear_state',
        "offline task contract tests cover runtime guard wait/clear state, failure extras and lost-link timeout decisions",
    )
    require(
        task_contract_test,
        r'test_runtime_guard_waiting_event_payload[\s\S]{0,1200}runtime_guard_waiting[\s\S]{0,1200}age_s[\s\S]{0,700}timeout_s',
        "offline task contract tests cover runtime guard waiting timeline payload",
    )
    require(
        task_contract_test,
        r'test_pose_helpers_and_readiness[\s\S]{0,2600}localization_not_confirmed[\s\S]{0,2600}pose_invalid_or_stale',
        "offline task contract tests cover pose/localization/first-waypoint readiness rules",
    )
    require(
        dashboard,
        r'waypoint_on_occupied_cell[\s\S]{0,260}waypoint_on_unknown_cell[\s\S]{0,260}检查点位',
        "frontend labels occupied/unknown waypoint failures as point checks",
    )
    require(
        dashboard,
        r'function taskStatusAllowsStart\(status\)[\s\S]{0,260}normalized === "error"',
        "frontend allows failed tasks to be retried after readiness becomes healthy",
    )
    require(
        web,
        r'def _start_task[\s\S]{0,900}context\s*=\s*self\._task_start_pre_runtime_context\(payload\)[\s\S]{0,2600}post_reset_task_ready\s*=\s*self\._task_start_readiness_payload[\s\S]{0,1400}context\s*=\s*self\._task_start_pre_runtime_context\(payload\)[\s\S]{0,1400}final_readiness\s*=\s*self\._task_start_readiness_payload',
        "backend rechecks shared pre-runtime context and task pose/readiness after navigation reset before dispatching the first goal",
    )
    require(
        active_task_contract,
        r'GOAL_SENT_RESET_KEYS[\s\S]{0,900}has_nav_feedback[\s\S]{0,900}last_nav_status[\s\S]{0,900}last_nav_feedback_monotonic[\s\S]{0,900}last_nav_goal_seq[\s\S]{0,900}last_ignored_nav_goal_match',
        "active task contract clears stale Nav2 feedback/status when switching waypoints",
    )
    require(
        task_progress_contract,
        r'current_goal_sent\s*=\s*active\.get\("last_goal_annotation_id"\)\s*==\s*annotation\.get\("id"\)[\s\S]{0,900}current_nav_status\s+not in \("sent",\s*"accepted"\)[\s\S]{0,900}near_goal_waiting_nav2',
        "near-goal waiting only happens after the current waypoint was sent to Nav2",
    )
    require(
        web,
        r'def _complete_active_waypoint_from_nav_result[\s\S]{0,900}nav_success_completion_decision[\s\S]{0,900}忽略非当前任务点 Nav2 成功事件',
        "Nav2 success cannot advance a task unless it matches the active sent waypoint",
    )
    require(
        task_watcher,
        r'nav_remaining[\s\S]{0,900}recoveries[\s\S]{0,900}robot_x[\s\S]{0,900}goal_x[\s\S]{0,900}nav_pose_x',
        "frontend task watcher records Nav2 remaining distance and recovery count",
    )
    require(
        task_watcher,
        r'nav_goal_seq[\s\S]{0,500}goal_attempt[\s\S]{0,500}nav_match[\s\S]{0,500}nav_match_reason',
        "frontend task watcher records Nav2 goal identity and match result",
    )
    require(
        task_watcher,
        r'goal_attempt\\tfloor_goal_published\\tfloor_goal_publishes\\tnav_match',
        "frontend task watcher TSV header records floor-goal publish evidence",
    )
    require(
        task_watcher,
        r'"floor_goal_published": first_value\([\s\S]{0,500}last_floor_goal_published_at[\s\S]{0,700}"floor_goal_publishes": first_value\([\s\S]{0,500}floor_goal_publish_count',
        "frontend task watcher reads floor-goal publish evidence",
    )
    require(
        task_watcher,
        r'"goal_attempt", "floor_goal_published"[\s\S]{0,200}"floor_goal_publishes", "nav_match"',
        "frontend task watcher writes floor-goal publish evidence columns",
    )
    require(
        task_watcher,
        r'task_name[\s\S]{0,500}task_map[\s\S]{0,500}waypoint_id',
        "frontend task watcher records task and waypoint identity columns",
    )
    require(
        task_watcher,
        r'robot_goal_error[\s\S]{0,500}nav_goal_error[\s\S]{0,500}robot_nav_error',
        "frontend task watcher records robot/goal/Nav2 pose error metrics",
    )
    require(
        task_watcher,
        r'path_goal_error[\s\S]{0,500}path_points',
        "frontend task watcher records planned path endpoint error",
    )
    require(
        task_watcher,
        r'first_waypoint_distance',
        "frontend task watcher records current distance to the first waypoint",
    )
    require(
        dashboard,
        r'function planarError[\s\S]{0,500}Math\.hypot',
        "frontend task summary shows robot/goal/Nav2 pose error metrics",
    )
    require(
        dashboard,
        r'robotGoalError[\s\S]{0,160}navGoalError[\s\S]{0,160}robotNavError',
        "frontend task summary computes robot/goal/Nav2 pose error metrics",
    )
    require(
        dashboard,
        r'狗差[\s\S]{0,200}Nav2差[\s\S]{0,200}位姿差',
        "frontend task summary labels robot/goal/Nav2 pose error metrics",
    )
    require(
        dashboard,
        r'id="taskPoseTracker"',
        "frontend task page includes the live pose tracker container",
    )
    require(
        dashboard,
        r'function renderPoseTracker[\s\S]{0,3200}地图位姿[\s\S]{0,3200}Nav2反馈[\s\S]{0,3200}当前目标[\s\S]{0,3200}误差',
        "frontend task page exposes a dedicated live pose tracker",
    )
    require(
        dashboard,
        r'renderPoseTracker\("livePoseTracker",\s*s\)[\s\S]{0,160}renderPoseTracker\("taskPoseTracker",\s*s\)',
        "frontend updates live and task pose trackers from /api/state",
    )
    require(
        dashboard,
        r'path_goal_error_m[\s\S]{0,500}路径差',
        "frontend task summary labels planned path endpoint error",
    )
    require(
        web,
        r'raw_poses = list\(msg\.poses\)[\s\S]{0,900}path_last_point[\s\S]{0,900}raw_poses\[-1\][\s\S]{0,900}poses\.append\(raw_poses\[-1\]\)',
        "web path endpoint diagnostics preserve the original Nav2 plan endpoint after display downsampling",
    )
    require(
        web,
        r'task_plan_match_required',
        "web task runner requires current Nav2 plan to match the active waypoint",
    )
    require(
        task_plan_contract,
        r'def task_plan_match_decision[\s\S]{0,3600}path_goal_mismatch',
        "task plan contract stops tasks when the current plan targets the wrong endpoint",
    )
    require(
        task_plan_contract,
        r'def task_plan_match_decision[\s\S]{0,5000}plan_update_timeout',
        "task plan contract stops tasks when no current plan is observed",
    )
    require(
        task_plan_contract,
        r'def apply_plan_goal_verified_state[\s\S]{0,900}plan_goal_verified[\s\S]{0,700}plan_path_version[\s\S]{0,700}status_message',
        "task plan contract owns plan-goal verified state updates",
    )
    require(
        task_plan_contract,
        r'def plan_goal_verified_event_payload[\s\S]{0,1200}plan_goal_verified[\s\S]{0,1200}path_last_point[\s\S]{0,800}path_goal_error_m',
        "task plan contract owns plan-goal verified timeline event payloads",
    )
    require(
        web,
        r'def _stop_task_if_plan_mismatched[\s\S]{0,1200}task_plan_match_decision',
        "web delegates task plan endpoint verification to task plan contract",
    )
    require(
        web,
        r'def _stop_task_if_plan_mismatched[\s\S]{0,1500}apply_plan_goal_verified_state',
        "web delegates plan-goal verified state updates to task plan contract",
    )
    require(
        web,
        r'def _stop_task_if_plan_mismatched[\s\S]{0,1700}plan_goal_verified_event_payload\([\s\S]{0,900}_append_active_task_timeline_event',
        "web delegates plan-goal verified event payload assembly to task plan contract",
    )
    require(
        web,
        r'def _stop_task_if_plan_mismatched[\s\S]{0,1800}event_payload\["event"\][\s\S]{0,500}event_payload\["message"\]',
        "web consumes required plan-goal verified event contract fields instead of fallback messages",
    )
    forbid(
        web,
        r'def _stop_task_if_plan_mismatched[\s\S]{0,2600}Nav2 规划路径与当前任务点不匹配，已停止任务|def _stop_task_if_plan_mismatched[\s\S]{0,2600}or "plan_goal_verified"',
        "web plan-match path no longer hardcodes failure or event fallback messages",
    )
    require(
        task_snapshot_contract,
        r'def build_active_waypoint_payload[\s\S]{0,3200}goal_sent_path_version[\s\S]{0,1300}plan_goal_verified[\s\S]{0,1300}last_floor_goal_published_at[\s\S]{0,900}floor_goal_publish_count',
        "task snapshot contract exposes plan-version verification for frontend and watcher diagnostics",
    )
    require(
        dashboard,
        r'路径已校验[\s\S]{0,500}下发路径版[\s\S]{0,500}校验路径版',
        "frontend task summary shows plan verification status and plan versions",
    )
    require(
        task_snapshot_contract,
        r'RESULT_ACTIVE_KEYS[\s\S]{0,1600}"last_goal_attempt_id"[\s\S]{0,1200}"last_floor_goal_published_at"[\s\S]{0,1200}"plan_goal_verified"[\s\S]{0,1200}"last_nav_feedback"[\s\S]{0,1200}"last_robot_pose"',
        "task snapshot contract preserves goal, plan, Nav2 and robot diagnostics",
    )
    require(
        web,
        r'def _task_result_snapshot_unlocked[\s\S]{0,900}build_task_result_snapshot',
        "web delegates task result snapshots to task snapshot contract",
    )
    require(
        web,
        r'def _task_runtime_snapshot_unlocked[\s\S]{0,900}build_task_runtime_snapshot',
        "web delegates task runtime snapshots to task snapshot contract",
    )
    require(
        web,
        r'def _last_task_result_unlocked[\s\S]{0,260}last_task_result_payload\(self\._tasks\)',
        "web delegates last task result payload selection to task snapshot contract",
    )
    require(
        web,
        r'def _publish_active_waypoint[\s\S]{0,900}build_active_waypoint_payload',
        "web delegates live active-waypoint payloads to task snapshot contract",
    )
    require(
        web,
        r'def _publish_idle_waypoint[\s\S]{0,500}build_idle_waypoint_payload',
        "web delegates idle active-waypoint payloads to task snapshot contract",
    )
    require(
        web,
        r'def _persist_task_result_unlocked[\s\S]{0,900}apply_task_result_to_tasks\([\s\S]{0,900}self\._tasks = list\(update\["tasks"\]\)',
        "web task result persistence delegates list-level state updates to task snapshot contract",
    )
    forbid(
        web,
        r'def _persist_task_result_unlocked[\s\S]{0,900}task\.update\(',
        "web task result persistence does not mutate a task dict directly",
    )
    require(
        dashboard,
        r'function taskLastResultText\(task\)[\s\S]{0,260}task\.last_result[\s\S]{0,1400}路径差',
        "frontend task cards show persisted task result diagnostics",
    )
    require(
        dashboard,
        r'const lastResultText = taskLastResultText\(task\)[\s\S]{0,900}\$\{lastResultText \?',
        "frontend task cards render persisted task result diagnostics",
    )
    forbid(
        dashboard,
        r'id="map2dBtn"|id="map3dBtn"|function drawTerrain|function loadTerrain|function setMapViewMode|/api/map_3d\?map_id=|/api/stair_zones\?map_id=|terrainMessage|state\.terrain',
        "frontend removes 3D map mode controls and terrain rendering",
    )
    forbid(
        web,
        r'parsed\.path == "/api/map_3d"|parsed\.path == "/api/stair_zones"|parsed\.path == "/api/stair_pointcloud"|def _map_3d_payload|def _stair_pointcloud_payload',
        "web dashboard removes frontend-only 3D/stair pointcloud HTTP APIs",
    )
    require(
        web,
        r'stair_zones_pub[\s\S]{0,500}stair_zones_topic',
        "web dashboard keeps internal stair-zones topic publisher for future cross-floor logic",
    )
    require(
        web,
        r'def _publish_selected_stair_zones[\s\S]{0,900}stair_zones_pub\.publish',
        "web dashboard keeps selected stair-zones publisher timer path",
    )
    require(
        floor_manager,
        r'stair_zones_topic[\s\S]{0,1200}self\._on_stair_zones',
        "floor manager keeps internal stair-zones subscriber",
    )
    require(
        task_watcher,
        r'last_event[\s\S]{0,500}last_result[\s\S]{0,500}message',
        "frontend task watcher records task timeline/result context",
    )
    require(
        task_watcher,
        r'path_raw_points[\s\S]{0,500}goal_sent_path_version[\s\S]{0,500}plan_path_version[\s\S]{0,500}plan_verified',
        "frontend task watcher records raw path count and plan-version verification columns",
    )
    require(
        task_watcher,
        r'battery_level[\s\S]{0,500}scan_finite[\s\S]{0,500}lidar_points[\s\S]{0,500}runtime_guard',
        "frontend task watcher records battery, scan, lidar and runtime guard columns",
    )
    require(
        task_watch_analyzer,
        r'runtime_guard_lost[\s\S]{0,1200}perception_scan_unavailable[\s\S]{0,1200}perception_lidar_unavailable',
        "frontend task watcher analyzer detects runtime guard and perception failures",
    )
    require(
        task_watcher,
        r'ANALYSIS_TXT="\$\{RUN_DIR\}/analysis\.txt"',
        "frontend task watcher automatically writes analysis.txt",
    )
    require(
        task_watcher,
        r'ANALYZER=.*104_analyze_frontend_task_watch\.py',
        "frontend task watcher runs the offline analyzer",
    )
    require(
        task_watcher,
        r'"\$\{ANALYZER\}" "\$\{RUN_DIR\}" >"\$\{ANALYSIS_TXT\}"',
        "frontend task watcher directly executes analyzer when executable",
    )
    require(
        task_watcher,
        r'python3 "\$\{ANALYZER\}" "\$\{RUN_DIR\}" >"\$\{ANALYSIS_TXT\}"',
        "frontend task watcher can run analyzer through python3",
    )
    require(
        task_watcher,
        r'analysis=\$\{ANALYSIS_TXT\}',
        "frontend task watcher prints analysis artifact path",
    )
    require(
        task_watch_analyzer,
        r'summary_path\s*=\s*run_dir / "summary\.tsv"[\s\S]{0,120}read_tsv\(summary_path\)',
        "frontend task watcher analyzer reads summary, state and journal artifacts",
    )
    require(
        task_watch_analyzer,
        r'state_overview\(run_dir / "state\.jsonl"\)[\s\S]{0,120}journal_counts\(run_dir / "m20pro-real\.journal\.log"\)',
        "frontend task watcher analyzer reads state and journal artifacts",
    )
    require(
        task_watch_analyzer,
        r'tasks_overview\(run_dir / "tasks\.jsonl"\)',
        "frontend task watcher analyzer reads task definitions from tasks.jsonl",
    )
    require(
        task_watch_analyzer,
        r'"result_lines": \[\]',
        "frontend task watcher analyzer stores persisted task result lines",
    )
    require(
        task_watch_analyzer,
        r'"snapshot_lines": \[\]',
        "frontend task watcher analyzer stores persisted runtime snapshot lines",
    )
    require(
        task_watch_analyzer,
        r'last_timeline',
        "frontend task watcher analyzer reads persisted task result timelines",
    )
    require(
        task_watch_analyzer,
        r'last_result[\s\S]{0,900}last_distance_m[\s\S]{0,900}path_goal_error_m',
        "frontend task watcher analyzer reads persisted task result snapshots",
    )
    require(
        task_watch_analyzer,
        r'def compact_runtime_snapshot[\s\S]{0,4200}last_nav_feedback[\s\S]{0,4200}plan_goal_verified',
        "frontend task watcher analyzer summarizes persisted runtime snapshots",
    )
    require(
        task_watch_analyzer,
        r'def compact_runtime_snapshot[\s\S]{0,5200}last_floor_goal_published_at[\s\S]{0,900}floor_goal_publish_count',
        "frontend task watcher analyzer summarizes floor-goal publish evidence",
    )
    require(
        task_watch_analyzer,
        r'lidar_relay\s*=\s*snapshot\.get\("lidar_relay_status"\)',
        "frontend task watcher analyzer summarizes lidar relay downsample diagnostics",
    )
    require(
        task_watch_analyzer,
        r'input_rate_hz[\s\S]{0,220}publish_rate_hz',
        "frontend task watcher analyzer reads lidar relay rate diagnostics",
    )
    require(
        task_watch_analyzer,
        r'relay=out\{points\}/stride=\{stride\}/method=\{method\}/in=\{in_hz\}Hz/out=\{out_hz\}Hz/skip=\{skip\}',
        "frontend task watcher analyzer prints compact lidar relay diagnostics",
    )
    require(
        task_watch_analyzer,
        r'last_error',
        "frontend task watcher analyzer reads persisted task errors",
    )
    require(
        task_watch_analyzer,
        r'print\("task definitions:"\)',
        "frontend task watcher analyzer prints task definitions section",
    )
    require(
        task_watch_analyzer,
        r'print\("task results:"\)[\s\S]{0,500}result_lines',
        "frontend task watcher analyzer prints persisted task results section",
    )
    require(
        task_watch_analyzer,
        r'print\("runtime snapshots:"\)[\s\S]{0,500}snapshot_lines',
        "frontend task watcher analyzer prints persisted runtime snapshot section",
    )
    require(
        task_watch_analyzer,
        r'waypoint_text[\s\S]{0,900}order=',
        "frontend task watcher analyzer reports first waypoint and waypoint order",
    )
    require(
        task_watch_analyzer,
        r'min_robot_goal[\s\S]{0,180}robot_goal_error[\s\S]{0,180}min_nav_goal[\s\S]{0,180}nav_goal_error',
        "frontend task watcher analyzer summarizes pose error metrics",
    )
    require(
        task_watch_analyzer,
        r'min_robot_nav[\s\S]{0,180}robot_nav_error',
        "frontend task watcher analyzer reads robot/Nav2 pose error metric",
    )
    require(
        task_watch_analyzer,
        r'min_path_goal[\s\S]{0,500}planned path endpoint is far from active waypoint',
        "frontend task watcher analyzer reports planned path endpoint mismatch",
    )
    require(
        task_watch_analyzer,
        r'path_goal_mismatch[\s\S]{0,500}planned path endpoint did not match the active waypoint',
        "frontend task watcher analyzer reports hard path-goal mismatch failures",
    )
    require(
        task_watch_analyzer,
        r'task_result_messages',
        "frontend task watcher analyzer includes persisted task results in hard-failure findings",
    )
    require(
        task_watch_analyzer,
        r'plan_update_timeout[\s\S]{0,500}no fresh Nav2 plan was observed',
        "frontend task watcher analyzer reports missing current-plan failures",
    )
    require(
        task_watch_analyzer,
        r'nav_goal_seq',
        "frontend task watcher analyzer summarizes Nav2 goal sequence ids",
    )
    require(
        task_watch_analyzer,
        r'goal_attempts',
        "frontend task watcher analyzer summarizes frontend goal attempts",
    )
    require(
        task_watch_analyzer,
        r'Nav2 goal mismatch reasons',
        "frontend task watcher analyzer reports Nav2 goal identity and mismatch context",
    )
    require(
        task_watch_analyzer,
        r'nav_status_ignored[\s\S]{0,500}mismatched Nav2 status ignored',
        "frontend task watcher analyzer reports ignored mismatched Nav2 statuses",
    )
    require(
        task_watch_analyzer,
        r'no active task observed[\s\S]{0,500}goal resend observed[\s\S]{0,500}Nav2 recoveries observed',
        "frontend task watcher analyzer reports actionable findings",
    )
    require(
        task_watch_analyzer,
        r'print\("findings:"\)',
        "frontend task watcher analyzer prints findings section",
    )
    require(
        frontend_task_smoke,
        r'Read-only browser smoke test[\s\S]{0,900}does not click task start',
        "frontend task smoke is documented as read-only",
    )
    forbid(
        frontend_task_smoke,
        r'/api/tasks/start|querySelector\([^)]*data-start-task[^)]*\)\.click|startButtons\[[^\]]+\]\.click',
        "frontend task smoke does not start tasks or click motion controls",
    )
    require(
        frontend_task_smoke,
        r'taskStartConfirmText is missing',
        "frontend task smoke checks task confirmation and waypoint order UI",
    )
    require(
        frontend_task_smoke,
        r'taskStartRequest is missing',
        "frontend task smoke checks task start request builder",
    )
    require(
        frontend_task_smoke,
        r'task cards must show first waypoint and order',
        "frontend task smoke checks waypoint order UI",
    )
    require(
        frontend_task_smoke,
        r'typeof loadTasks === \'function\'[\s\S]{0,120}await loadTasks\(\)',
        "frontend task smoke explicitly loads the real task list before assertions",
    )
    require(
        frontend_task_smoke,
        r'current-map task cards must show the recommended watcher command',
        "frontend task smoke checks watcher command appears only on current-map executable task cards",
    )
    require(
        frontend_task_smoke,
        r'current-map task cards must show the task-specific ready-check command',
        "frontend task smoke checks ready-check command appears only on current-map executable task cards",
    )
    require(
        frontend_task_smoke,
        r'enabledCopyCommandButtons[\s\S]{0,1800}104_frontend_task_ready_check\.py --task-id[\s\S]{0,800}104_watch_frontend_task\.sh 180',
        "frontend task smoke checks evidence command copy buttons",
    )
    require(
        frontend_task_smoke,
        r'currentMapTaskNotice',
        "frontend task smoke checks old-map task evidence actions are disabled",
    )
    require(
        frontend_task_smoke,
        r'enabledCopyCommandButtons',
        "frontend task smoke inspects only enabled evidence copy buttons",
    )
    require(
        frontend_task_smoke,
        r'old-map tasks must be hidden from the main task list[\s\S]{0,500}task list must report hidden old-map tasks',
        "frontend task smoke fails if old-map tasks are rendered in the main task list",
    )
    require(
        frontend_task_smoke,
        r'old-map tasks must not expose enabled watcher copy buttons[\s\S]{0,500}old-map task cards must not display field evidence commands',
        "frontend task smoke fails if old-map tasks expose or display field evidence commands",
    )
    require(
        frontend_task_smoke,
        r'\\nready\\n[\s\S]{0,240}old-map task cards must not display raw ready status',
        "frontend task smoke fails if old-map tasks display raw ready status",
    )
    require(
        frontend_task_smoke,
        r'createTaskDisabled[\s\S]{0,1200}current map has no task points',
        "frontend task smoke checks create-task button is disabled without current-map points",
    )
    require(
        frontend_task_smoke,
        r'taskNextStep[\s\S]{0,1000}final relocalization success',
        "frontend task smoke checks unlocalized next-step guidance",
    )
    require(
        frontend_task_smoke,
        r'stopTaskButton[\s\S]{0,1200}stop current task button must be disabled when no active task exists[\s\S]{0,800}当前没有前端任务',
        "frontend task smoke checks stop-task button is disabled with no active task",
    )
    require(
        frontend_task_smoke,
        r'activeTaskRaw[\s\S]{0,1200}active task raw panel must show only no-task state[\s\S]{0,500}last_task_result',
        "frontend task smoke checks the current-execution raw panel does not show historical task results",
    )
    require(
        frontend_task_smoke,
        r'resetTaskSessionButton[\s\S]{0,1200}显式复位导航会话',
        "frontend task smoke checks reset is labeled as an explicit navigation reset",
    )
    require(
        frontend_task_smoke,
        r'fieldSnapshot[\s\S]{0,1200}required_snapshot_keys[\s\S]{0,1200}perception[\s\S]{0,1200}task_execution_evidence[\s\S]{0,1200}recommended_task',
        "frontend task smoke checks the field snapshot payload",
    )
    require(
        frontend_task_smoke,
        r'def evaluate_active_summary\(client: CdpClient\)',
        "frontend task smoke synthesizes an active task summary with path and pose diagnostics",
    )
    require(
        frontend_task_smoke,
        r'renderActiveTaskSummary\(activeTask,\s*waypoint\)',
        "frontend task smoke calls the real active task summary renderer",
    )
    require(
        frontend_task_smoke,
        r'inactiveWithHistoryText[\s\S]{0,900}inactive task summary must not render historical task results as current execution',
        "frontend task smoke rejects historical task results in the live active summary",
    )
    require(
        frontend_task_smoke,
        r'summary_payload = evaluate_active_summary\(client\)[\s\S]{0,160}assert_active_summary\(summary_payload\)',
        "frontend task smoke verifies the synthetic active summary output",
    )
    require(
        frontend_task_smoke,
        r'def evaluate_manual_relocalization_status\(client: CdpClient\)[\s\S]{0,1600}renderLocalizationStatus\([\s\S]{0,1000}tcp_2101_accepted:\s*false[\s\S]{0,1200}tcp_2101_accepted:\s*true',
        "frontend task smoke synthesizes manual TCP 2101 relocalization status rendering",
    )
    require(
        frontend_task_smoke,
        r'def evaluate_manual_relocalization_status\(client: CdpClient\)[\s\S]{0,3600}relocalization_result:\s*\{[\s\S]{0,300}success: x=-10\.771[\s\S]{0,900}task_readiness:\s*\{[\s\S]{0,600}map_relocalization_required',
        "frontend task smoke covers legacy backend payload with top-level 2101 success but map relocalization lock",
    )
    require(
        frontend_task_smoke,
        r'def assert_manual_relocalization_status[\s\S]{0,2200}重定位失败[\s\S]{0,1200}2101回执[\s\S]{0,1200}重定位成功[\s\S]{0,1800}legacy_required[\s\S]{0,700}重定位锁未清除',
        "frontend task smoke asserts manual 2101 success/failure text and task readiness",
    )
    require(
        frontend_task_smoke,
        r'relocalization_payload = evaluate_manual_relocalization_status\(client\)[\s\S]{0,180}assert_manual_relocalization_status\(relocalization_payload\)[\s\S]{0,180}syntheticManualRelocalizationStatus',
        "frontend task smoke runs the manual 2101 relocalization rendering assertion",
    )
    require(
        frontend_task_smoke,
        r'path_goal_error_m:\s*0\.12',
        "frontend task smoke injects synthetic path endpoint error",
    )
    require(
        frontend_task_smoke,
        r'plan_goal_verified:\s*true',
        "frontend task smoke injects synthetic plan verification",
    )
    require(
        frontend_task_smoke,
        r'goal_sent_path_version:\s*3',
        "frontend task smoke injects synthetic path verification values",
    )
    require(
        frontend_task_smoke,
        r'狗差[\s\S]{0,600}Nav2差[\s\S]{0,600}位姿差[\s\S]{0,600}反馈差[\s\S]{0,600}Nav2反馈 2s前[\s\S]{0,600}路径差 0\.12m[\s\S]{0,600}路径已校验[\s\S]{0,600}floor_goal已发[\s\S]{0,600}/floor_goal 1次',
        "frontend task smoke asserts live summary diagnostic labels are rendered",
    )
    require(
        frontend_task_smoke,
        r'tracker_required[\s\S]{0,700}地图位姿[\s\S]{0,700}Nav2反馈[\s\S]{0,700}当前目标[\s\S]{0,700}路径-目标 0\.12m',
        "frontend task smoke defines required pose tracker diagnostics",
    )
    require(
        frontend_task_smoke,
        r'pose tracker missing fields',
        "frontend task smoke asserts the dedicated pose tracker renders task diagnostics",
    )
    require(
        frontend_task_smoke,
        r'hasMap3dButton[\s\S]{0,3000}frontend 2D/3D map mode buttons should be removed[\s\S]{0,2200}canvasSample',
        "frontend task smoke verifies removed map-mode controls and 2D canvas output",
    )
    require(
        frontend_task_smoke,
        r'hasFrontVideo[\s\S]{0,500}hasRearVideo[\s\S]{0,1000}front/rear camera images should remain visible[\s\S]{0,1200}camera images should not auto-load camera streams[\s\S]{0,700}MJPEG stream endpoints',
        "frontend task smoke verifies on-demand camera images remain visible",
    )
    require(
        frontend_task_smoke,
        r'cameraToggleButtons[\s\S]{0,700}front/rear camera toggle buttons should be removed[\s\S]{0,700}all-camera on/off controls should be removed[\s\S]{0,700}camera status diagnostics should be removed',
        "frontend task smoke verifies removed camera controls",
    )
    require(
        frontend_task_ready_check,
        r'Read-only readiness check[\s\S]{0,900}does not call /api/tasks/start',
        "frontend task ready check is documented as read-only",
    )
    forbid(
        frontend_task_ready_check,
        r'fetch_json\(args\.url,\s*"/api/tasks/start"|urlopen\([^)]*/api/tasks/start|create_publisher|\.publish\(|data-start-task',
        "frontend task ready check does not start tasks or reference motion commands",
    )
    require(
        frontend_task_ready_check,
        r'battery_level[\s\S]{0,800}scan_finite[\s\S]{0,800}lidar_points',
        "frontend task ready check reports battery, scan and lidar readiness",
    )
    require(
        frontend_task_ready_check,
        r'perception_status=\{code\}: ready=\{ready\} message=\{message\}',
        "frontend task ready check reports structured perception status",
    )
    require(
        frontend_task_ready_check,
        r'localization_status=\{code\}: confirmed=\{confirmed\} task_ready=\{task_ready\} tcp_2101=\{tcp_2101\}',
        "frontend task ready check reports manual relocalization confirmation status",
    )
    require(
        frontend_task_ready_check,
        r'lidar_relay output_points[\s\S]{0,500}in_hz[\s\S]{0,500}out_hz[\s\S]{0,500}skip',
        "frontend task ready check reports lidar relay rate/downsample diagnostics",
    )
    require(
        frontend_task_ready_check,
        r'lidar_relay_method=',
        "frontend task ready check reports lidar relay downsample method",
    )
    require(
        frontend_task_ready_check,
        r'lidar_relay_input topic=\{topic\} publishers=\{publishers\} messages=\{messages\} published=\{published\}',
        "frontend task ready check reports relay input publisher count",
    )
    forbid(
        frontend_task_ready_check,
        r'summarize_camera_proxy|summarize_map_3d|/api/map_3d|/api/stair_zones|camera_proxy',
        "frontend task ready check omits removed camera and 3D-map diagnostics",
    )
    require(
        frontend_task_ready_check,
        r'def advice_readiness_source[\s\S]{0,2200}same_map_tasks[\s\S]{0,1200}no_current_map_task[\s\S]{0,1200}旧任务属于其他地图',
        "frontend task ready check does not use old-map tasks as default advice",
    )
    require(
        frontend_task_ready_check,
        r'advice_source=.*code=',
        "frontend task ready check prints the advice source",
    )
    require(
        frontend_task_ready_check,
        r'waypoint_on_occupied_cell[\s\S]{0,260}waypoint_on_unknown_cell[\s\S]{0,260}重新标点',
        "frontend task ready check advises remaking waypoints on occupied or unknown cells",
    )
    require(
        frontend_task_ready_check,
        r'def bad_waypoint_line[\s\S]{0,1600}reason=',
        "frontend task ready check prints bad waypoint details from readiness payloads",
    )
    require(
        frontend_task_ready_check,
        r'bad_waypoints[\s\S]{0,900}bad_waypoint=',
        "frontend task ready check includes bad waypoint lines in task summaries",
    )
    require(
        frontend_task_ready_check,
        r'first=[\s\S]{0,500}order=',
        "frontend task ready check reports first waypoint and full order",
    )
    require(
        frontend_task_ready_check,
        r'first_distance=',
        "frontend task ready check reports current distance to the first waypoint",
    )
    require(
        frontend_task_ready_check,
        r'def recommended_task[\s\S]{0,1200}if not \(task_id or task_name\):[\s\S]{0,800}task\.get\("map_id"\)[\s\S]{0,900}return min\(tasks, key=lambda task: task_priority_tuple\(task, selected_map\)\)',
        "frontend task ready check recommends only current-map tasks unless explicitly filtered",
    )
    require(
        frontend_task_ready_check,
        r'watcher_command=\.\/scripts\/104_watch_frontend_task\.sh 180',
        "frontend task ready check prints a copyable watcher command",
    )
    require(
        frontend_task_ready_check,
        r'ready_check_command=\.\/scripts\/104_frontend_task_ready_check\.py --task-id',
        "frontend task ready check prints a copyable task-specific ready check command",
    )
    require(
        frontend_task_ready_check,
        r'def advice_for_code[\s\S]{0,2400}no_current_map_task[\s\S]{0,2400}localization_not_confirmed[\s\S]{0,2400}perception_lidar_unavailable[\s\S]{0,2400}navigation_not_ready',
        "frontend task ready check prints actionable next-step advice",
    )
    forbid(
        "\n".join(path.name for path in (ROOT / "scripts").glob("*")),
        r'104_frontend_task_field_run\.sh',
        "removed field-run wrapper script stays deleted",
    )
    script_names = "\n".join(path.name for path in (ROOT / "scripts").glob("*"))
    forbid(
        script_names,
        r'104_check_initialpose_to_106\.sh|104_watch_localization_status\.py',
        "removed initialpose-to-106 and localization watcher scripts stay deleted",
    )
    require(
        frontend_task_ready_check,
        r'def state_level_advice[\s\S]{0,600}battery_low[\s\S]{0,600}先充电[\s\S]{0,1400}factory_lidar_points_publisher_missing[\s\S]{0,1600}selected_map_status[\s\S]{0,1200}localization_not_confirmed[\s\S]{0,900}重定位成功',
        "frontend task ready check prioritizes low battery, hard perception faults, map mismatch, then manual relocalization advice",
    )
    require(
        dashboard_js,
        r'perceptionStatus\.ready === false[\s\S]{0,700}factory_lidar_points_publisher_missing[\s\S]{0,900}selectedMapStatus\.ready === false',
        "frontend task next-step summary surfaces hard perception faults before map/relocation guidance",
    )
    require(
        dashboard_js,
        r'感知链路:\s*s\.perception_status',
        "frontend navigation status panel includes structured perception status",
    )
    require(
        frontend_task_ready_check,
        r'if state_advice:[\s\S]{0,220}advice_source=state code=\{text\(next_code\)\}',
        "frontend task ready check labels state-level blocker codes consistently with the next action",
    )
    require(
        frontend_task_ready_check,
        r'print\("next:"\)[\s\S]{0,300}if state_advice:[\s\S]{0,500}else:[\s\S]{0,240}advice_for_code',
        "frontend task ready check includes a next section in output",
    )
    require(
        goal_mode_battery_gate,
        r'Read-only battery gate[\s\S]{0,900}/api/state[\s\S]{0,900}min-level',
        "goal-mode battery gate is documented as read-only and threshold-based",
    )
    forbid(
        goal_mode_battery_gate,
        r'/api/tasks/start|/m20pro/floor_goal|/api/localization/initialpose|create_publisher|\.publish\(|cmd_vel',
        "goal-mode battery gate does not start tasks, relocalize or send motion",
    )
    require(
        read(ROOT / "scripts/README.md"),
        r'104_goal_mode_battery_gate\.py[\s\S]{0,500}低于 25%[\s\S]{0,500}停止目标模式',
        "scripts README documents the goal-mode battery gate",
    )
    require(
        read(ROOT / "scripts/README.md"),
        r'开发手册 TCP `2101/1` 回执为准[\s\S]{0,160}不要再用“106 是否收到 `/initialpose`”作为成功判断',
        "scripts README makes manual TCP 2101 the relocalization diagnostic source of truth",
    )
    require(
        read(ROOT / "README.md"),
        r'网页重定位以定位页最终结论为准[\s\S]{0,260}重定位成功',
        "README makes manual TCP 2101 the web relocalization success criterion",
    )
    forbid(
        read(ROOT / "README.md") + "\n" + read(ROOT / "scripts/README.md"),
        r'优先复刻 106 RViz|默认不让 `m20pro_tcp_bridge` 抢占 `/initialpose`',
        "README files do not describe initialpose arrival as the relocalization success path",
    )
    forbid(
        read(ROOT / "scripts/README.md") + "\n" + read(ROOT / "README.md"),
        r'104_check_initialpose_to_106|104_watch_localization_status',
        "README files no longer advertise removed localization helper scripts",
    )

    print("[OK] preflight policy checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
