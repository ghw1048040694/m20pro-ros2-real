from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node

from .map_manifest import load_manifest, load_yaml, resolve_path


class ConfigAuditNode(Node):
    """Validate map/floor configuration once and print actionable results."""

    def __init__(self) -> None:
        super().__init__("m20pro_config_audit")
        self.declare_parameter("map_manifest", "")
        self.declare_parameter("floor_config", "")
        self.declare_parameter("fail_on_error", False)
        self.timer = self.create_timer(0.2, self._run_once)
        self.finished = False

    def _run_once(self) -> None:
        if self.finished:
            return
        self.finished = True
        self.timer.cancel()
        errors: List[str] = []
        warnings: List[str] = []

        manifest_path = str(self.get_parameter("map_manifest").value).strip()
        floor_config_path = str(self.get_parameter("floor_config").value).strip()

        manifest: Dict[str, Any] = {}
        floor_config: Dict[str, Any] = {}
        try:
            manifest = load_manifest(manifest_path)
        except Exception as exc:
            errors.append("map_manifest invalid: %s" % exc)

        try:
            floor_config = load_yaml(floor_config_path)
        except Exception as exc:
            errors.append("floor_config invalid: %s" % exc)

        if manifest and floor_config:
            self._check_manifest_assets(manifest, errors, warnings)
            self._check_floor_consistency(manifest, floor_config, errors, warnings)

        for warning in warnings:
            self.get_logger().warning(warning)
        for error in errors:
            self.get_logger().error(error)

        if errors:
            if bool(self.get_parameter("fail_on_error").value):
                raise RuntimeError("configuration audit failed with %d errors" % len(errors))
            self.get_logger().warning(
                "configuration audit finished with %d errors and %d warnings"
                % (len(errors), len(warnings))
            )
            return

        self.get_logger().info(
            "configuration audit OK: %d floors, %d warnings"
            % (len((manifest.get("floors") or {})), len(warnings))
        )

    def _check_manifest_assets(
        self,
        manifest: Dict[str, Any],
        errors: List[str],
        warnings: List[str],
    ) -> None:
        manifest_dir = str(Path(resolve_path(str(self.get_parameter("map_manifest").value))).parent)
        map_set = manifest.get("map_set") or {}
        global_pcd = str(map_set.get("global_pcd") or "").strip()
        if global_pcd and not Path(resolve_path(global_pcd, manifest_dir)).exists():
            errors.append("global_pcd does not exist: %s" % global_pcd)

        for floor_id, floor in (manifest.get("floors") or {}).items():
            if not isinstance(floor, dict):
                errors.append("manifest floor %s is not a mapping" % floor_id)
                continue
            map_yaml = str(floor.get("map_yaml") or "").strip()
            if not map_yaml:
                errors.append("manifest floor %s has empty map_yaml" % floor_id)
            elif not Path(resolve_path(map_yaml, manifest_dir)).exists():
                errors.append("manifest floor %s map_yaml does not exist: %s" % (floor_id, map_yaml))

            pcd_map = str(floor.get("pcd_map") or global_pcd).strip()
            if pcd_map and not Path(resolve_path(pcd_map, manifest_dir)).exists():
                warnings.append("manifest floor %s pcd_map does not exist: %s" % (floor_id, pcd_map))

    def _check_floor_consistency(
        self,
        manifest: Dict[str, Any],
        floor_config: Dict[str, Any],
        errors: List[str],
        warnings: List[str],
    ) -> None:
        manifest_floors = set((manifest.get("floors") or {}).keys())
        route_floors = set((floor_config.get("floors") or {}).keys())
        for floor_id in sorted(manifest_floors - route_floors):
            warnings.append("manifest floor %s has no route config" % floor_id)
        for floor_id in sorted(route_floors - manifest_floors):
            warnings.append("route floor %s has no map manifest entry" % floor_id)

        for floor_id in sorted(route_floors & manifest_floors):
            route_map = str((floor_config["floors"][floor_id] or {}).get("map_yaml") or "").strip()
            manifest_map = str((manifest["floors"][floor_id] or {}).get("map_yaml") or "").strip()
            if route_map and manifest_map and route_map != manifest_map:
                warnings.append(
                    "floor %s map differs between manifest and route config" % floor_id
                )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ConfigAuditNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
