#!/usr/bin/env python3
"""Static contract for on-demand RKNN inspection and Web control."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    node = (
        ROOT
        / "src/m20pro_inspection/m20pro_inspection/yolov8_inspection_node.py"
    ).read_text(encoding="utf-8")
    inspection_launch = (
        ROOT / "src/m20pro_inspection/launch/m20pro_inspection.launch.py"
    ).read_text(encoding="utf-8")
    real_launch = (
        ROOT / "src/m20pro_bringup/launch/m20pro_real.launch.py"
    ).read_text(encoding="utf-8")
    config = (
        ROOT / "src/m20pro_inspection/config/yolov8_inspection.yaml"
    ).read_text(encoding="utf-8")

    assert 'self.declare_parameter("enabled", False)' in node
    assert '"~/set_enabled"' in node
    assert "def _activate_inspection" in node
    assert "def _deactivate_inspection" in node
    assert "self._release_backend()" in node
    assert "if not self.runtime_lock.acquire(blocking=False):" in node
    assert "def _tick_locked" in node
    assert "thread.join(timeout=2.0)" in node
    assert 'self.active_backend = "disabled"' in node
    assert '"enabled": self.enabled' in node
    assert '"ready": ready' in node
    assert 'DeclareLaunchArgument("enabled", default_value="false")' in inspection_launch
    assert "respawn=True" in inspection_launch
    assert '"enabled": enabled' in inspection_launch
    assert '"enabled": enable_inspection' in real_launch
    inspection_include = real_launch.index("PythonLaunchDescriptionSource(inspection_launch)")
    radar_include = real_launch.index("PythonLaunchDescriptionSource(radar_launch)")
    assert "condition=IfCondition(enable_inspection)" not in real_launch[inspection_include:radar_include]
    assert "enabled: false" in config
    assert "publish_rate_hz: 3.0" in config
    assert "publish_annotated_image: false" in config

    print("inspection control contract tests passed")


if __name__ == "__main__":
    main()
