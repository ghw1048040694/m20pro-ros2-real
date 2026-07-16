#!/usr/bin/env python3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    deploy = (ROOT / "scripts" / "local_deploy_to_test_robot.sh").read_text(
        encoding="utf-8"
    )
    edge = (ROOT / "scripts" / "local_deploy_edge_scan_to_106.sh").read_text(
        encoding="utf-8"
    )
    install = (ROOT / "scripts" / "104_install_staged_workspace.sh").read_text(
        encoding="utf-8"
    )

    assert "local_deploy_edge_scan_to_106.sh" in deploy
    assert deploy.index("local_deploy_edge_scan_to_106.sh") < deploy.index("rsync -az --delete")
    assert "systemctl restart m20pro-edge-scan-106.service" in deploy
    assert 'if [[ "${M20PRO_DEPLOY_SKIP_EDGE:-0}" != "1" ]]; then' in deploy
    assert 'perception.get("mode") == "edge_scan"' in deploy
    assert 'scan.get("frame_id") == "m20pro_base_link"' in deploy
    assert 'int(scan.get("finite_ranges") or 0) >= 20' in deploy
    for excluded in (".git/", "build/", "install/", "log/", "bags/", "*.db3"):
        assert excluded in deploy

    assert "106_enable_edge_scan_service.sh" in edge
    assert "tools/edge_scan_feasibility" in edge
    assert 'STAGE="${REMOTE_WS}.edge_stage.${STAMP}"' in edge
    assert "--no-owner --no-group" in edge
    assert '"${STAGE}/tools/edge_scan_feasibility/"' in edge
    assert 'sudo -n rsync -a --delete' in edge
    assert 'sudo -n chown -R' in edge
    assert 'systemctl restart m20pro-edge-scan-106.service' in edge
    assert "systemctl is-enabled --quiet m20pro-edge-scan-106.service" in edge
    assert "systemctl is-active --quiet m20pro-edge-scan-106.service" in edge

    move_index = install.index('mv "${STAGE}" "${TARGET}"')
    build_index = install.index("colcon build --symlink-install")
    cutover_index = install.index("cutover_started=1", install.index("systemctl stop"))
    assert cutover_index < move_index
    assert move_index < build_index
    assert "A symlink install embeds absolute build paths" in install
    assert 'source /opt/robot/scripts/setup_ros2.sh' in install
    assert '104_enable_autostart.sh" move' in install
    assert "rollback" in install
    assert "systemctl reset-failed m20pro-real.service" in install

    assert "edge_previous_state" in deploy
    assert "systemctl stop m20pro-edge-scan-106.service" in deploy
    assert "systemctl disable m20pro-edge-scan-106.service" in deploy
    assert 'M20PRO_DEPLOY_KEEP_BACKUP:-0' in deploy
    assert "${BACKUP}.systemd'" in deploy
    assert "removed successful-deployment backup" in deploy

    print("test-robot deployment contract tests passed")


if __name__ == "__main__":
    main()
