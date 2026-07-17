#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    config_path = ROOT / "src" / "m20pro_bringup" / "config" / "nav2_params_real.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    real_config_path = ROOT / "src" / "m20pro_bringup" / "config" / "m20pro_real.yaml"
    real_config = yaml.safe_load(real_config_path.read_text(encoding="utf-8"))
    bridge_config = real_config["m20pro_tcp_bridge"]["ros__parameters"]
    assert float(bridge_config["pose_jump_accept_after_s"]) == 0.0
    assert float(bridge_config["pose_stationary_drift_reject_m"]) > 0.0
    assert float(bridge_config["pose_motion_command_hold_s"]) > 0.0
    controller = config["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]

    assert controller["controller_frequency"] <= 10.0
    assert controller["progress_checker"]["movement_time_allowance"] >= 12.0
    assert controller["goal_checker"]["stateful"] is False
    assert "ObstacleFootprint" in follow["critics"]
    assert "BaseObstacle" not in follow["critics"]
    assert follow["sim_time"] >= 1.5
    assert follow["publish_local_plan"] is True
    assert follow["stateful"] is False
    assert local["always_send_full_costmap"] is True
    assert global_costmap["always_send_full_costmap"] is False
    for costmap in (local, global_costmap):
        assert "footprint" in costmap
        assert "robot_radius" not in costmap
        assert costmap["inflation_layer"]["inflation_radius"] >= 0.60

    tree_path = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "behavior_trees"
        / "m20pro_navigate_to_pose_foxy.xml"
    )
    root = ET.parse(tree_path).getroot()
    assert root.find(".//ReactiveFallback[@name='FollowPathFallback']") is not None
    assert root.find(".//BackUp") is not None
    navigate_recovery = root.find(".//RecoveryNode[@name='NavigateRecovery']")
    follow_recovery = root.find(".//RecoveryNode[@name='FollowPath']")
    assert navigate_recovery is not None
    assert follow_recovery is not None
    assert int(navigate_recovery.attrib["number_of_retries"]) >= 4
    assert int(follow_recovery.attrib["number_of_retries"]) >= 8
    controller_actions = root.find(".//RoundRobin[@name='ControllerRecoveryActions']")
    navigation_actions = root.find(".//RoundRobin[@name='NavigationRecoveryActions']")
    assert controller_actions is not None
    assert navigation_actions is not None
    controller_children = list(controller_actions)
    navigation_children = list(navigation_actions)
    assert controller_children[0].tag == "Wait"
    assert navigation_children[0].tag == "Wait"
    assert int(controller_children[0].attrib["wait_duration"]) >= 5
    assert int(navigation_children[0].attrib["wait_duration"]) >= 5
    timed_bt_tags = {
        "ComputePathToPose",
        "FollowPath",
        "ClearEntireCostmap",
        "BackUp",
        "Spin",
        "Wait",
    }
    timed_nodes = [node for node in root.iter() if node.tag in timed_bt_tags]
    assert timed_nodes
    for node in timed_nodes:
        assert int(node.attrib.get("server_timeout", "0")) >= 500

    dashboard = (
        ROOT
        / "src"
        / "m20pro_cloud_bridge"
        / "m20pro_cloud_bridge"
        / "web_dashboard_node.py"
    ).read_text(encoding="utf-8")
    tcp_bridge = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "tcp_bridge_node.py"
    ).read_text(encoding="utf-8")
    assert "accept_after_s=0.0" in tcp_bridge
    assert "allow_stable_recovery=location_ok" not in tcp_bridge
    assert "OccupancyGridUpdate" in dashboard
    assert '"local_costmap_updates_topic"' in dashboard
    assert '"global_costmap_updates_topic"' in dashboard
    assert '"local_plan_topic"' in dashboard
    assert '"odom_topic", "/odom"' in dashboard
    assert "local_costmap_odom_alignment_payload" in dashboard

    recorder = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "scripts"
        / "m20pro_record_real.sh"
    ).read_text(encoding="utf-8")
    assert "/local_plan" in recorder

    real_launch = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "launch"
        / "m20pro_real.launch.py"
    ).read_text(encoding="utf-8")
    nav_launch = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "launch"
        / "nav2_navigation_real_foxy.launch.py"
    ).read_text(encoding="utf-8")
    startup_gate = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "nav2_startup_gate.py"
    ).read_text(encoding="utf-8")
    system_check = (
        ROOT
        / "src"
        / "m20pro_navigation"
        / "m20pro_navigation"
        / "system_check_node.py"
    ).read_text(encoding="utf-8")
    assert "nav2_navigation_real_foxy.launch.py" in real_launch
    assert 'executable="waypoint_follower"' not in nav_launch
    assert '"waypoint_follower"' not in startup_gate
    assert '"/waypoint_follower"' not in system_check
    assert '"/waypoint_follower"' not in dashboard
    assert 'default_value="/m20pro/recording_scan"' in real_launch
    assert '"scan_topic": web_scan_topic' in real_launch

    fastdds_profile = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "config"
        / "m20pro_fastdds_udp.xml"
    ).read_text(encoding="utf-8")
    assert "<type>UDPv4</type>" in fastdds_profile
    assert "<type>SHM</type>" not in fastdds_profile
    assert "m20pro_shm_transport" not in fastdds_profile

    real_start = (
        ROOT
        / "src"
        / "m20pro_bringup"
        / "scripts"
        / "m20pro_real_full.sh"
    ).read_text(encoding="utf-8")
    assert "export PYTHONDONTWRITEBYTECODE=1" in real_start

    edge_env_path = (
        ROOT
        / "tools"
        / "edge_scan_feasibility"
        / "service"
        / "m20pro-edge-scan-106.env.edge_scan"
    )
    edge_env = {}
    for raw_line in edge_env_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        edge_env[key] = value
    assert float(edge_env["HEIGHT_MIN"]) <= -0.25
    assert float(edge_env["HEIGHT_MAX"]) >= 0.60
    assert int(edge_env["MAX_POINTS"]) == 0
    assert 0.5 <= float(edge_env["BIN_HOLD_S"]) <= 1.0

    print("real Nav2 configuration contract tests passed")


if __name__ == "__main__":
    main()
