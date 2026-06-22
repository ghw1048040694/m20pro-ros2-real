#!/usr/bin/env python3
"""Offline guardrails for M20Pro preflight and assist-mode safety.

This is intentionally lightweight: the web dashboard is a ROS node, so importing
it in CI or on a laptop without ROS dependencies is brittle.  These checks guard
the concrete invariants that have repeatedly broken field testing.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py"
REAL_LAUNCH = ROOT / "src/m20pro_bringup/launch/m20pro_real.launch.py"
WEB_LAUNCH = ROOT / "src/m20pro_bringup/launch/m20pro_web_dashboard.launch.py"
NAV2_PARAMS = ROOT / "src/m20pro_bringup/config/nav2_params_real.yaml"
REAL_CONFIG = ROOT / "src/m20pro_bringup/config/m20pro_real.yaml"
TCP_BRIDGE = ROOT / "src/m20pro_navigation/m20pro_navigation/tcp_bridge_node.py"
SETUP = ROOT / "src/m20pro_navigation/setup.py"
POINTCLOUD_FUSION = ROOT / "src/m20pro_navigation/m20pro_navigation/pointcloud_fusion.py"
REAL_FULL = ROOT / "src/m20pro_bringup/scripts/m20pro_real_full.sh"
DIAGNOSE = ROOT / "scripts/104_diagnose_preflight.sh"
TERMINAL_PREFLIGHT = ROOT / "scripts/104_preflight_check.sh"
RELAY_GUARD = ROOT / "src/m20pro_bringup/scripts/m20pro_lidar_relay_guard.sh"
AUTOSTART = ROOT / "scripts/104_enable_autostart.sh"


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


def main() -> int:
    web = read(WEB)
    launch = read(REAL_LAUNCH)
    web_launch = read(WEB_LAUNCH)
    nav2 = read(NAV2_PARAMS)
    real_config = read(REAL_CONFIG)
    tcp = read(TCP_BRIDGE)
    setup = read(SETUP)
    fusion = read(POINTCLOUD_FUSION)
    real_full = read(REAL_FULL)
    diagnose = read(DIAGNOSE)
    terminal_preflight = read(TERMINAL_PREFLIGHT)
    relay_guard = read(RELAY_GUARD)
    autostart = read(AUTOSTART)

    require(
        web,
        r'declare_parameter\("preflight_settle_wait_s",\s*6\.0\)',
        "web preflight keeps a short baseline settle wait",
    )
    require(
        web,
        r'\{\s*mode:\s*"move",\s*site:\s*"auto",\s*wait:\s*false\s*\}',
        "frontend starts preflight in auto site mode asynchronously",
    )
    require(
        web,
        r'"m20pro_nav2_startup_gate"',
        "web preflight requires the Nav2 startup gate node",
    )
    require(
        web,
        r'base_required_nodes\s*=\s*\[[\s\S]{0,280}"m20pro_nav2_startup_gate"[\s\S]{0,160}\]',
        "web preflight separates base nodes from Nav2 nodes",
    )
    require(
        web,
        r'navigation_required_nodes\s*=\s*\[[\s\S]{0,180}"bt_navigator"[\s\S]{0,120}"waypoint_follower"',
        "web preflight treats Nav2 nodes as navigation readiness",
    )
    forbid(
        web,
        r'base_required_nodes\s*=\s*\[[\s\S]{0,260}"bt_navigator"',
        "bt_navigator must not block base preflight",
    )
    require(
        web,
        r'"nav2_lifecycle_deferred"[\s\S]{0,300}"info"',
        "unlocalized/workstation Nav2 lifecycle is informational",
    )
    require(
        web,
        r'factory_initialpose_ssh_identity_file",\s*"/home/user/\.ssh/id_ed25519"',
        "web 106 initialpose uses the user SSH key even when service runs as root",
    )
    require(
        web,
        r'\["scp",\s*"-r",\s*\*ssh_options,\s*remote,\s*str\(dest\)\]',
        "web map import scp uses the same root-compatible SSH options",
    )
    require(
        web,
        r'_factory_ssh_file_options\(8\)[\s\S]{0,700}sudo -n -l',
        "web mapping environment check uses root-compatible SSH and non-mutating sudo permission probes",
    )
    forbid(
        web,
        r'sudo -n (?:/usr/local/bin/)?drmap stop_mapping -h',
        "web mapping environment check must not execute stop_mapping as a help probe",
    )
    for label, text in (("real launch", launch), ("web launch", web_launch), ("web defaults", web)):
        require(
            text,
            r'factory_mapping_start_command[\s\S]{0,420}-i /home/user/\.ssh/id_ed25519[\s\S]{0,260}UserKnownHostsFile=/home/user/\.ssh/known_hosts[\s\S]{0,260}/usr/local/bin/drmap mapping',
            f"{label} mapping start command uses root-compatible SSH and absolute drmap",
        )
        require(
            text,
            r'factory_mapping_finish_command[\s\S]{0,360}-i /home/user/\.ssh/id_ed25519[\s\S]{0,240}UserKnownHostsFile=/home/user/\.ssh/known_hosts[\s\S]{0,220}/usr/local/bin/drmap stop_mapping',
            f"{label} mapping finish command uses root-compatible SSH and absolute drmap",
        )
    require(
        web,
        r'def _rewrite_imported_map_image_path',
        "web map import rewrites 106 absolute image paths for 104",
    )
    require(
        web,
        r'pub\.get_subscription_count\(\)',
        "106 initialpose publish waits for a subscriber before sending",
    )
    require(
        launch,
        r'factory_initialpose_ssh_identity_file',
        "real launch forwards the 106 initialpose SSH identity file",
    )
    require(
        web,
        r'"local_costmap"[\s\S]{0,260}"info"[\s\S]{0,260}未重定位前 Nav2/costmap',
        "unlocalized/workstation local costmap is deferred info",
    )
    require(
        web,
        r'"global_costmap"[\s\S]{0,260}"info"[\s\S]{0,260}未重定位前 Nav2/costmap',
        "unlocalized/workstation global costmap is deferred info",
    )
    require(
        web,
        r'navigation_failures\s*=\s*\[[\s\S]{0,220}item\["status"\]\s+in\s+\("fail",\s*"warn"\)',
        "navigation readiness counts only fail/warn, not info",
    )
    require(
        web,
        r'"navigation_ready":\s*not navigation_failures',
        "navigation_ready is derived from navigation_failures",
    )

    forbid(web, r'api/usage_mode|data-usage-mode|setUsageMode', "web has no usage-mode control route/button")
    require(real_config, r'enable_usage_mode_command:\s*false', "real config keeps usage-mode command disabled")
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
    require(real_full, r'COMMON_ARGS\+=\(backup_cloud_topic:="\$\{BACKUP_CLOUD_TOPIC\}"\)', "real full startup conditionally passes backup cloud topic")
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
        r'M20PRO_LIDAR_RELAY_MAX_OUTPUT_POINTS:-12000',
        "lidar relay guard defaults to downsampled pointcloud output",
    )
    require(
        relay_guard,
        r'M20PRO_LIDAR_RELAY_MIN_PUBLISH_INTERVAL_S:-0\.1',
        "lidar relay guard limits pointcloud publish rate",
    )
    require(
        autostart,
        r'M20PRO_LIDAR_RELAY_MAX_OUTPUT_POINTS=\$\{M20PRO_LIDAR_RELAY_MAX_OUTPUT_POINTS:-12000\}',
        "autostart default records relay pointcloud downsample limit",
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

    print("[OK] preflight policy checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
