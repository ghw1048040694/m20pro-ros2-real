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

        manifest_maps = manifest.get("maps") if isinstance(manifest.get("maps"), list) else manifest.get("floors") or {}
        self.get_logger().info(
            "configuration audit OK: %d map assets, %d warnings"
            % (len(manifest_maps), len(warnings))
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

        raw_maps = manifest.get("maps")
        entries = (
            [(item.get("id") or item.get("floor"), item) for item in raw_maps if isinstance(item, dict)]
            if isinstance(raw_maps, list)
            else list((manifest.get("floors") or {}).items())
        )
        for floor_id, floor in entries:
            if not isinstance(floor, dict):
                errors.append("manifest map %s is not a mapping" % floor_id)
                continue
            map_yaml = str(floor.get("map_yaml") or "").strip()
            if not map_yaml:
                errors.append("manifest map %s has empty map_yaml" % floor_id)
            elif not Path(resolve_path(map_yaml, manifest_dir)).exists():
                errors.append("manifest map %s map_yaml does not exist: %s" % (floor_id, map_yaml))

            pcd_map = str(floor.get("pcd_map") or global_pcd).strip()
            if pcd_map and not Path(resolve_path(pcd_map, manifest_dir)).exists():
                warnings.append("manifest map %s pcd_map does not exist: %s" % (floor_id, pcd_map))

    def _check_floor_consistency(
        self,
        manifest: Dict[str, Any],
        floor_config: Dict[str, Any],
        errors: List[str],
        warnings: List[str],
    ) -> None:
        raw_maps = manifest.get("maps")
        manifest_floors = {
            str(item.get("floor") or item.get("id") or "")
            for item in raw_maps
            if isinstance(item, dict) and (item.get("floor") or item.get("id"))
        } if isinstance(raw_maps, list) else set((manifest.get("floors") or {}).keys())
        route_floors = set((floor_config.get("floors") or {}).keys())
        # A map without a route profile is valid ordinary single-map data.
        for floor_id in sorted(route_floors - manifest_floors):
            warnings.append("route floor %s has no map asset entry" % floor_id)

        for floor_id in sorted(route_floors & manifest_floors):
            route_map = str((floor_config["floors"][floor_id] or {}).get("map_yaml") or "").strip()
            if isinstance(raw_maps, list):
                manifest_item = next(
                    (
                        item for item in raw_maps
                        if isinstance(item, dict)
                        and str(item.get("floor") or item.get("id") or "") == floor_id
                    ),
                    {},
                )
            else:
                manifest_item = (manifest.get("floors") or {}).get(floor_id) or {}
            manifest_map = str(manifest_item.get("map_yaml") or "").strip()
            if route_map and manifest_map and route_map != manifest_map:
                warnings.append(
                    "floor %s map differs between manifest and route config" % floor_id
                )
            self._check_waypoint_semantics(floor_id, floor_config["floors"][floor_id], warnings)

    def _check_waypoint_semantics(
        self,
        floor_id: str,
        floor: Dict[str, Any],
        warnings: List[str],
    ) -> None:
        waypoints = floor.get("waypoints") or []
        if not isinstance(waypoints, list):
            warnings.append("floor %s waypoints is not a list" % floor_id)
            return
        for index, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, dict):
                warnings.append("floor %s waypoint %d is not a mapping" % (floor_id, index))
                continue
            waypoint_id = str(waypoint.get("id") or "<unnamed>")
            pose = waypoint.get("pose") or {}
            if not isinstance(pose, dict):
                warnings.append("floor %s waypoint %s pose is not a mapping" % (floor_id, waypoint_id))
                continue
            for key in ("x", "y", "yaw"):
                if key not in pose:
                    warnings.append("floor %s waypoint %s missing pose.%s" % (floor_id, waypoint_id, key))
            if not str(waypoint.get("label") or waypoint.get("name") or "").strip():
                warnings.append(
                    "floor %s waypoint %s has no label/name for inspection reports"
                    % (floor_id, waypoint_id)
                )
            if str(waypoint.get("point_type") or waypoint.get("manual_point_type") or "").strip() == "task":
                if not str(waypoint.get("area") or waypoint.get("region") or "").strip():
                    warnings.append(
                        "floor %s task waypoint %s has no area/region"
                        % (floor_id, waypoint_id)
                    )
                if not str(waypoint.get("room") or waypoint.get("place") or "").strip():
                    warnings.append(
                        "floor %s task waypoint %s has no room/place"
                        % (floor_id, waypoint_id)
                    )
            point_type = str(waypoint.get("point_type") or waypoint.get("manual_point_type") or "").strip()
            if point_type not in ("transition", "task", "charge"):
                warnings.append(
                    "floor %s waypoint %s has no manual point_type transition/task/charge"
                    % (floor_id, waypoint_id)
                )
            if point_type == "task" and "dwell_s" not in waypoint and "inspect_duration_s" not in waypoint:
                warnings.append(
                    "floor %s task waypoint %s has no dwell_s/inspect_duration_s"
                    % (floor_id, waypoint_id)
                )
            vendor = waypoint.get("vendor_navigation") or {}
            if isinstance(vendor, dict):
                for key in ("PointInfo", "Gait", "Speed", "Manner", "ObsMode", "NavMode"):
                    if key not in vendor:
                        warnings.append(
                            "floor %s waypoint %s missing vendor_navigation.%s"
                            % (floor_id, waypoint_id, key)
                        )
            else:
                warnings.append(
                    "floor %s waypoint %s vendor_navigation is not a mapping"
                    % (floor_id, waypoint_id)
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
