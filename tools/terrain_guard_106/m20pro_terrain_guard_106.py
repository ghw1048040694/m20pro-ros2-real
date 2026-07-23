#!/usr/bin/env python3
"""106-local, read-only ROS 2 adapter for the stair terrain contract.

The node consumes the vendor PointCloud2 locally and publishes only a small
JSON status message.  It has no cmd_vel publisher, no gait publisher, and no
104 network client.  The default output is shadow-only: even a traversable
classification never authorizes motion.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


sys.path.insert(0, str(Path(__file__).resolve().parent))
from terrain_guard_contract import inspect_cloud, normalize_corridor  # noqa: E402


def _json_message(payload: Dict[str, Any]) -> String:
    message = String()
    message.data = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
    return message


class TerrainGuard106(Node):
    def __init__(self) -> None:
        super().__init__("m20pro_terrain_guard_106")
        self.declare_parameter("cloud_topic", "/LIDAR/POINTS")
        self.declare_parameter("request_topic", "/m20pro/terrain_guard/request")
        self.declare_parameter("status_topic", "/m20pro/terrain_guard/status")
        self.declare_parameter("expected_frame", "m20pro_base_link")
        self.declare_parameter("cloud_timeout_s", 0.75)
        self.declare_parameter("publish_period_s", 0.20)

        cloud_topic = str(self.get_parameter("cloud_topic").value)
        request_topic = str(self.get_parameter("request_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self._expected_frame = str(self.get_parameter("expected_frame").value).strip()
        self._cloud_timeout_s = float(self.get_parameter("cloud_timeout_s").value)
        self._request: Optional[Dict[str, Any]] = None
        self._request_error: Optional[str] = None
        self._points: list[tuple[float, float, float]] = []
        self._last_cloud_monotonic: Optional[float] = None
        self._last_cloud_frame = ""
        self._last_cloud_error: Optional[str] = None
        self._sequence = 0
        self._last_result: Dict[str, Any] = {
            "state": "unknown",
            "reason": "terrain_guard_disabled",
            "confidence": 0.0,
            "traversable": False,
            "permit_motion": False,
        }

        self._status_publisher = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, request_topic, self._on_request, 10)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data)
        period = max(0.05, self.get_parameter("publish_period_s").value)
        self.create_timer(float(period), self._on_timer)
        self.get_logger().info(
            "shadow terrain guard ready: cloud=%s request=%s status=%s",
            cloud_topic,
            request_topic,
            status_topic,
        )

    def _on_request(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self._request = None
            self._request_error = "terrain_request_json_invalid"
            self._publish_status()
            return
        if not isinstance(value, dict):
            self._request = None
            self._request_error = "terrain_request_invalid"
            self._publish_status()
            return
        enabled = value.get("enabled", value.get("active", True))
        if enabled is False:
            self._request = None
            self._request_error = None
            self._last_result = {
                "state": "unknown",
                "reason": "terrain_guard_disabled",
                "confidence": 0.0,
                "traversable": False,
                "permit_motion": False,
            }
            self._publish_status()
            return
        normalized = normalize_corridor(value)
        if not normalized.get("ok", True):
            self._request = None
            self._request_error = str(normalized.get("code") or "terrain_request_invalid")
            self._publish_status()
            return
        self._request = value
        self._request_error = None
        self._sequence = 0
        self._last_result = {
            "state": "unknown",
            "reason": "awaiting_pointcloud",
            "confidence": 0.0,
            "traversable": False,
            "permit_motion": False,
        }
        self._publish_status()

    def _on_cloud(self, message: PointCloud2) -> None:
        self._last_cloud_frame = str(message.header.frame_id or "").strip()
        self._last_cloud_monotonic = time.monotonic()
        self._last_cloud_error = None
        if self._expected_frame and self._last_cloud_frame != self._expected_frame:
            self._points = []
            self._last_cloud_error = "cloud_frame_mismatch"
            self._publish_status()
            return
        try:
            self._points = [
                (float(x), float(y), float(z))
                for x, y, z in point_cloud2.read_points(
                    message,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
                if all(math.isfinite(float(value)) for value in (x, y, z))
            ]
        except (AttributeError, IndexError, TypeError, ValueError):
            self._points = []
            self._last_cloud_error = "pointcloud_fields_invalid"
        self._evaluate()

    def _on_timer(self) -> None:
        if self._request is not None:
            self._evaluate()
        else:
            self._publish_status()

    def _evaluate(self) -> None:
        if self._request is None:
            self._publish_status()
            return
        if self._request_error:
            self._last_result = {
                "state": "unknown",
                "reason": self._request_error,
                "confidence": 0.0,
                "traversable": False,
                "permit_motion": False,
            }
        elif self._last_cloud_error:
            self._last_result = {
                "state": "unknown",
                "reason": self._last_cloud_error,
                "confidence": 0.0,
                "traversable": False,
                "permit_motion": False,
            }
        else:
            age = (
                time.monotonic() - self._last_cloud_monotonic
                if self._last_cloud_monotonic is not None
                else None
            )
            self._last_result = inspect_cloud(
                self._points,
                request=self._request,
                cloud_age_s=age,
                cloud_timeout_s=self._cloud_timeout_s,
            )
        self._publish_status()

    def _publish_status(self) -> None:
        self._sequence += 1
        payload = dict(self._last_result)
        payload.update(
            {
                "route_id": str((self._request or {}).get("route_id") or ""),
                "corridor_version": str((self._request or {}).get("corridor_version") or ""),
                "sequence": self._sequence,
                "stamp_unix_s": time.time(),
                "cloud_frame": self._last_cloud_frame,
                "cloud_age_s": (
                    time.monotonic() - self._last_cloud_monotonic
                    if self._last_cloud_monotonic is not None
                    else None
                ),
                # Explicitly reported so a future executor cannot mistake the
                # classifier result for a control lease.
                "certified_motion": False,
                "source": "106_local_pointcloud",
            }
        )
        self._status_publisher.publish(_json_message(payload))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TerrainGuard106()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
