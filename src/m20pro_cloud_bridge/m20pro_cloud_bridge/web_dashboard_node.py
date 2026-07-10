import json
import math
import os
import select
import shlex
import shutil
import socket
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FsPath
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from .active_task_contract import (
    advance_active_task_state,
    active_annotation_from_list,
    active_annotation_missing_failure,
    active_task_failure_payload,
    append_active_task_timeline_event_state,
    begin_waypoint_dwell_state,
    create_active_task_state,
    dwell_tick_decision,
    fail_active_task_state,
    goal_dispatch_decision,
    idle_stop_task_response,
    mark_floor_goal_published_state,
    mark_active_task_waiting_state,
    normalize_stop_task_request,
    prepare_goal_send_state,
    stop_task_state,
    stop_task_operator_event_payload,
    waypoint_goal_failure_extra,
    waypoint_goal_payload,
)
from .annotation_contract import (
    annotation_list_filter_payload,
    annotation_map_pose_error_payload,
    annotation_create_static_context,
    annotation_create_readiness_payload,
    annotation_dwell_s,
    annotation_semantics_payload,
    build_annotation_record,
    normalize_annotation_semantics,
    resolve_annotation_dwell_s,
)
from .localization_contract import (
    initialpose_api_response_payload,
    localization_status_payload,
    map_relocalization_clearance_payload,
    manual_relocalization_verification_payload,
    parse_initialpose_request,
    relocalization_sample_evidence,
    relocalization_response_payload,
)
from .map_derived_contract import (
    builtin_map_derived_payload,
    map_derived_payload,
    read_json_object,
    resolve_map_asset_path,
    should_generate_builtin_stair_zones,
    stair_zones_available_payload,
    stair_zones_relative_path,
    stair_zones_unavailable_payload,
)
from .map_contract import (
    all_map_records,
    build_imported_map_record,
    default_map_id,
    ensure_map_yaml_uses_local_image,
    find_map_record,
    find_map_yaml,
    load_builtin_maps_from_manifest,
    load_map_file_payload,
    map_file_fingerprint,
    map_file_metadata_payload,
)
from .map_selection_contract import (
    apply_selected_map_choice_state,
    effective_map_id_for_display,
    map_relocalization_required_payload,
    matching_fixed_map_id_for_live_map,
    selected_map_status_payload,
    selected_map_wait_timeout_payload,
)
from .mapping_contract import (
    apply_mapping_command_result,
    mapping_command_context,
    prepare_mapping_session_create,
)
from .nav_status_contract import (
    apply_ignored_nav_status_state,
    apply_nav_failure_state,
    apply_nav_feedback_state,
    apply_nav_goal_status_state,
    apply_nav_status_message_state,
    apply_transition_nav_status_state,
    classify_navigation_status,
    ignored_nav_status_event_payload,
    nav_feedback_dispatch_payload,
    nav_feedback_event_payload,
    nav_goal_status_event_payload,
    nav_status_message_event_payload,
    nav_status_matches_active_goal,
    nav_success_completion_decision,
    parse_key_value_status,
    should_record_nav_feedback_event,
    transition_nav_status_event_payload,
)
from .navigation_readiness_contract import (
    navigation_readiness_payload,
)
from .pcd_derived import process_imported_map
from .perception_contract import perception_status_payload
from .preflight_contract import (
    preflight_base_topics_item,
    preflight_context,
    preflight_costmap_items,
    preflight_localization_item,
    preflight_lifecycle_deferred_item,
    preflight_lifecycle_item,
    preflight_map_item,
    preflight_map_pose_item,
    preflight_motion_mode_item,
    preflight_navigation_status_item,
    preflight_navigation_topics_item,
    preflight_node_item,
    preflight_odom_item,
    preflight_perception_items,
    preflight_result_payload,
)
from .ros_message_contract import (
    pose_to_dict,
    stamp_to_float,
    wrap_angle,
    yaw_to_orientation,
)
from .startup_map_sync_contract import (
    startup_map_sync_missing_record_payload,
    startup_map_sync_retry_decision,
    startup_map_sync_result_payload,
    startup_map_sync_skipped_payload,
)
from .task_contract import (
    apply_deleted_annotation_to_tasks,
    apply_task_start_pre_runtime_failure_state,
    apply_task_delete,
    apply_task_name_update,
    build_task_create_record,
    is_finite_pose_dict,
    is_plausible_pose_dict,
    map_metadata_mismatch_error,
    normalize_startup_task_runtime_state,
    validate_task_annotations_for_map as contract_validate_task_annotations_for_map,
    stop_stale_running_tasks,
    task_create_map_metadata_mismatch_payload,
    task_list_filter_payload,
    task_create_static_context,
    task_start_static_context,
    task_waypoint_payload,
    validation_error_payload,
    validate_task_annotation_order,
    validate_task_start_expectations,
)
from .task_progress_contract import (
    active_task_pre_dispatch_decision,
    apply_localization_lost_start_state,
    localization_lost_failure_extra,
    localization_lost_start_event_payload,
    localization_lost_timeout_decision,
    near_goal_wait_decision,
    prepare_near_goal_wait_update,
    update_active_task_progress_state,
)
from .active_waypoint_contract import (
    build_active_waypoint_payload,
    build_idle_waypoint_payload,
    pose_age_sec,
)
from .web_runtime_contract import (
    api_error_payload,
    as_bool,
    fmt_age_text,
    new_id,
    now_text,
    parse_json_text,
    payload_with_age,
    random_suffix,
    sanitize_name,
)

try:
    from lifecycle_msgs.srv import GetState
except ImportError:  # pragma: no cover - ROS lifecycle package should exist on robot.
    GetState = None

try:
    from nav2_msgs.srv import ClearEntireCostmap, LoadMap
except ImportError:  # pragma: no cover - Nav2 package should exist on robot.
    ClearEntireCostmap = None
    LoadMap = None

try:
    from drdds.msg import BatteryData
except ImportError:  # Only available on the robot's factory ROS environment.
    BatteryData = None

try:
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:  # Foxy does not expose this internal exception module.
    RCLError = Exception

cv2 = None
_CV2_IMPORT_ERROR: Optional[str] = None
_CV2_IMPORT_ATTEMPTED = False
_CV2_IMPORT_LOCK = threading.Lock()


def get_cv2() -> Any:
    global cv2, _CV2_IMPORT_ATTEMPTED, _CV2_IMPORT_ERROR
    with _CV2_IMPORT_LOCK:
        if not _CV2_IMPORT_ATTEMPTED:
            _CV2_IMPORT_ATTEMPTED = True
            try:
                import cv2 as imported_cv2

                cv2 = imported_cv2
                _CV2_IMPORT_ERROR = None
            except Exception as exc:  # pragma: no cover - runtime dependency
                cv2 = None
                _CV2_IMPORT_ERROR = str(exc) or exc.__class__.__name__
        return cv2


DASHBOARD_STATIC_DIR = FsPath(__file__).resolve().parent / "static"
DASHBOARD_HTML_PATH = DASHBOARD_STATIC_DIR / "dashboard.html"
DASHBOARD_STATIC_FILES = {
    "/static/dashboard.css": (DASHBOARD_STATIC_DIR / "dashboard.css", "text/css; charset=utf-8"),
    "/static/dashboard.js": (DASHBOARD_STATIC_DIR / "dashboard.js", "application/javascript; charset=utf-8"),
}


def _load_dashboard_html() -> bytes:
    return DASHBOARD_HTML_PATH.read_bytes()


def _load_dashboard_static(path: str) -> Optional[Tuple[bytes, str]]:
    item = DASHBOARD_STATIC_FILES.get(path)
    if item is None:
        return None
    file_path, content_type = item
    return file_path.read_bytes(), content_type


class _ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


class _CameraProxyWorker:
    _opencv_env_lock = threading.Lock()

    def __init__(self, node: "WebDashboardNode", camera_name: str, url: str) -> None:
        self.node = node
        self.camera_name = camera_name
        self.url = url
        self.backend = node._camera_proxy_backend()
        self._condition = threading.Condition()
        self._thread: Optional[threading.Thread] = None
        self._stopped = False
        self._latest_jpeg: Optional[bytes] = None
        self._latest_stamp = 0.0
        self._sequence = 0
        self._client_count = 0
        self._snapshot_lease_until = 0.0
        self._last_error: Optional[str] = None
        self._last_error_stamp = 0.0
        self._last_error_log_time = 0.0

    def start(self) -> None:
        with self._condition:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stopped = False
            thread = threading.Thread(
                target=self._run,
                name=f"m20pro_camera_proxy_{self.camera_name}",
                daemon=True,
            )
            self._thread = thread
        thread.start()

    def stop(self) -> None:
        with self._condition:
            self._stopped = True
            self._snapshot_lease_until = 0.0
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def acquire_client(self) -> None:
        self.start()
        with self._condition:
            self._client_count += 1
            self._condition.notify_all()

    def release_client(self) -> None:
        with self._condition:
            self._client_count = max(0, self._client_count - 1)
            if self._client_count == 0 and not self._snapshot_lease_active_locked():
                self._latest_jpeg = None
                self._latest_stamp = 0.0
            self._condition.notify_all()

    def extend_snapshot_lease(self, keepalive_s: float) -> None:
        self.start()
        keepalive_s = max(0.0, keepalive_s)
        with self._condition:
            self._snapshot_lease_until = max(
                self._snapshot_lease_until,
                time.monotonic() + keepalive_s,
            )
            self._condition.notify_all()

    def current_frame_state(self) -> Tuple[int, float]:
        with self._condition:
            return self._sequence, self._latest_stamp

    def wait_for_frame(
        self,
        last_sequence: int,
        timeout_s: float,
    ) -> Tuple[int, Optional[bytes], float, Optional[str]]:
        deadline = time.monotonic() + max(0.1, timeout_s)
        with self._condition:
            while not self._stopped and (self._latest_jpeg is None or self._sequence <= last_sequence):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(timeout=remaining)
            return self._sequence, self._latest_jpeg, self._latest_stamp, self._last_error

    def status(self) -> Dict[str, Any]:
        with self._condition:
            latest_stamp = self._latest_stamp
            last_error_stamp = self._last_error_stamp
            thread_alive = self._thread is not None and self._thread.is_alive()
            client_count = self._client_count
            snapshot_lease_active = self._snapshot_lease_active_locked()
            return {
                "camera": self.camera_name,
                "url": self.url,
                "backend": self.backend,
                "running": thread_alive and (client_count > 0 or snapshot_lease_active),
                "clients": client_count,
                "snapshot_lease_active": snapshot_lease_active,
                "sequence": self._sequence,
                "has_frame": self._latest_jpeg is not None,
                "last_frame_age_s": None if latest_stamp <= 0.0 else max(0.0, time.time() - latest_stamp),
                "last_error": self._last_error,
                "last_error_age_s": None if last_error_stamp <= 0.0 else max(0.0, time.time() - last_error_stamp),
            }

    def _run(self) -> None:
        if self.backend == "ffmpeg_mjpeg":
            self._run_ffmpeg_mjpeg()
        else:
            self._run_opencv()

    def _run_opencv(self) -> None:
        cap = None
        reconnect_s = max(0.2, float(self.node.get_parameter("camera_proxy_reconnect_s").value))
        while not self._is_stopped():
            if not self._wait_for_client():
                break
            try:
                cv2_module = get_cv2()
                if cv2_module is None:
                    detail = _CV2_IMPORT_ERROR or "python3-opencv is not installed"
                    self._set_error(f"OpenCV unavailable: {detail}")
                    time.sleep(reconnect_s)
                    continue
                if cap is None or not cap.isOpened():
                    cap = self._open_capture(cv2_module)
                    if not cap.isOpened():
                        self._set_error("failed to open RTSP stream")
                        cap.release()
                        cap = None
                        time.sleep(reconnect_s)
                        continue
                    self._set_error(None)

                ok, frame = cap.read()
                if not ok or frame is None:
                    self._set_error("failed to read RTSP frame")
                    cap.release()
                    cap = None
                    time.sleep(reconnect_s)
                    continue

                if not self._should_publish_frame():
                    continue
                payload = self._encode_frame(cv2_module, frame)
                if payload is not None:
                    with self._condition:
                        if self._client_count > 0 or self._snapshot_lease_active_locked():
                            self._latest_jpeg = payload
                            self._latest_stamp = time.time()
                            self._sequence += 1
                            self._last_error = None
                            self._condition.notify_all()
            except Exception as exc:
                self._set_error(str(exc))
                if cap is not None:
                    cap.release()
                    cap = None
                time.sleep(reconnect_s)
            if cap is not None and not self._has_clients():
                cap.release()
                cap = None
                self._clear_idle_frame()
        if cap is not None:
            cap.release()

    def _run_ffmpeg_mjpeg(self) -> None:
        reconnect_s = max(0.2, float(self.node.get_parameter("camera_proxy_reconnect_s").value))
        while not self._is_stopped():
            if not self._wait_for_client():
                break
            proc: Optional[subprocess.Popen] = None
            try:
                cmd = self._ffmpeg_mjpeg_command()
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                )
                self._set_error(None)
                self._read_ffmpeg_mjpeg_frames(proc)
                code = proc.poll()
                if code not in (None, 0) and not self._is_stopped():
                    self._set_error(f"ffmpeg exited with code {code}")
            except Exception as exc:
                self._set_error(str(exc))
                time.sleep(reconnect_s)
            finally:
                if proc is not None:
                    self._terminate_process(proc)
                self._clear_idle_frame()

    def _ffmpeg_mjpeg_command(self) -> List[str]:
        fps = max(1.0, float(self.node.get_parameter("camera_proxy_fps").value))
        max_width = int(self.node.get_parameter("camera_proxy_max_width").value)
        qscale = max(2, min(31, int(self.node.get_parameter("camera_proxy_ffmpeg_mjpeg_qscale").value)))
        transport = str(self.node.get_parameter("camera_proxy_transport").value).lower()
        if transport not in ("tcp", "udp"):
            transport = "tcp"
        filters = [f"fps={fps:g}"]
        if max_width > 0:
            filters.append(f"scale={max_width}:-2")
        return [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostdin",
            "-rtsp_transport",
            transport,
            "-analyzeduration",
            "0",
            "-probesize",
            "32",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-max_delay",
            "0",
            "-i",
            self.url,
            "-an",
            "-vf",
            ",".join(filters),
            "-q:v",
            str(qscale),
            "-f",
            "mjpeg",
            "-flush_packets",
            "1",
            "pipe:1",
        ]

    def _read_ffmpeg_mjpeg_frames(self, proc: subprocess.Popen) -> None:
        if proc.stdout is None:
            raise RuntimeError("ffmpeg stdout unavailable")
        frame_timeout = max(0.5, float(self.node.get_parameter("camera_proxy_frame_timeout_s").value))
        buffer = bytearray()
        last_data = time.monotonic()
        while not self._is_stopped() and self._has_clients():
            if proc.poll() is not None:
                break
            readable, _, _ = select.select([proc.stdout], [], [], 0.25)
            if not readable:
                if time.monotonic() - last_data > frame_timeout:
                    self._set_error("ffmpeg camera frame timeout")
                    break
                continue
            chunk = os.read(proc.stdout.fileno(), 8192)
            if not chunk:
                break
            last_data = time.monotonic()
            buffer.extend(chunk)
            self._publish_jpeg_frames_from_buffer(buffer)

    def _publish_jpeg_frames_from_buffer(self, buffer: bytearray) -> None:
        while True:
            start = buffer.find(b"\xff\xd8")
            if start < 0:
                if len(buffer) > 1:
                    del buffer[:-1]
                return
            if start > 0:
                del buffer[:start]
            end = buffer.find(b"\xff\xd9", 2)
            if end < 0:
                if len(buffer) > 2_000_000:
                    del buffer[:-2]
                return
            payload = bytes(buffer[:end + 2])
            del buffer[:end + 2]
            with self._condition:
                if self._client_count <= 0 and not self._snapshot_lease_active_locked():
                    return
                self._latest_jpeg = payload
                self._latest_stamp = time.time()
                self._sequence += 1
                self._last_error = None
                self._condition.notify_all()

    def _terminate_process(self, proc: subprocess.Popen) -> None:
        if proc.poll() is not None:
            return
        try:
            proc.terminate()
            proc.wait(timeout=1.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    def _open_capture(self, cv2_module: Any) -> Any:
        if str(self.node.get_parameter("camera_proxy_transport").value).lower() == "tcp":
            options = str(self.node.get_parameter("camera_proxy_ffmpeg_options").value)
            with self._opencv_env_lock:
                os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = options
                if hasattr(cv2_module, "CAP_FFMPEG"):
                    cap = cv2_module.VideoCapture(self.url, cv2_module.CAP_FFMPEG)
                else:
                    cap = cv2_module.VideoCapture(self.url)
        else:
            cap = cv2_module.VideoCapture(self.url)

        self._set_capture_property(cv2_module, cap, "CAP_PROP_BUFFERSIZE", 1)
        self._set_capture_property(
            cv2_module,
            cap,
            "CAP_PROP_OPEN_TIMEOUT_MSEC",
            int(float(self.node.get_parameter("camera_proxy_open_timeout_s").value) * 1000.0),
        )
        self._set_capture_property(
            cv2_module,
            cap,
            "CAP_PROP_READ_TIMEOUT_MSEC",
            int(float(self.node.get_parameter("camera_proxy_read_timeout_s").value) * 1000.0),
        )
        return cap

    def _set_capture_property(self, cv2_module: Any, cap: Any, name: str, value: float) -> None:
        prop = getattr(cv2_module, name, None)
        if prop is None:
            return
        try:
            cap.set(prop, value)
        except Exception:
            pass

    def _should_publish_frame(self) -> bool:
        fps = max(1.0, float(self.node.get_parameter("camera_proxy_fps").value))
        with self._condition:
            last_stamp = self._latest_stamp
        if last_stamp <= 0.0:
            return True
        return (time.time() - last_stamp) >= (1.0 / fps)

    def _encode_frame(self, cv2_module: Any, frame: Any) -> Optional[bytes]:
        max_width = int(self.node.get_parameter("camera_proxy_max_width").value)
        if max_width > 0 and hasattr(frame, "shape") and frame.shape[1] > max_width:
            scale = max_width / float(frame.shape[1])
            height = max(1, int(frame.shape[0] * scale))
            frame = cv2_module.resize(frame, (max_width, height), interpolation=cv2_module.INTER_AREA)

        quality = max(30, min(95, int(self.node.get_parameter("camera_proxy_jpeg_quality").value)))
        ok, encoded = cv2_module.imencode(".jpg", frame, [int(cv2_module.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            return None
        return encoded.tobytes()

    def _set_error(self, error: Optional[str]) -> None:
        with self._condition:
            self._last_error = error
            self._last_error_stamp = time.time() if error else 0.0
            self._condition.notify_all()
        if error:
            now = time.monotonic()
            if now - self._last_error_log_time >= 5.0:
                self._last_error_log_time = now
                self.node.get_logger().warning(f"{self.camera_name} camera proxy: {error}")

    def _is_stopped(self) -> bool:
        with self._condition:
            return self._stopped

    def _has_clients(self) -> bool:
        with self._condition:
            return self._client_count > 0 or self._snapshot_lease_active_locked()

    def _wait_for_client(self) -> bool:
        with self._condition:
            while not self._stopped and self._client_count <= 0 and not self._snapshot_lease_active_locked():
                self._condition.wait(timeout=1.0)
            return not self._stopped

    def _snapshot_lease_active_locked(self) -> bool:
        return self._snapshot_lease_until > time.monotonic()

    def _clear_idle_frame(self) -> None:
        with self._condition:
            if self._client_count <= 0 and not self._snapshot_lease_active_locked():
                self._latest_jpeg = None
                self._latest_stamp = 0.0


class WebDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("m20pro_web_dashboard")
        self._declare_parameters()

        self._lock = threading.Lock()
        # Task callbacks often reuse small helpers that also inspect saved web data.
        # Keep this lock reentrant so a status update cannot deadlock the dispatcher.
        self._data_lock = threading.RLock()
        self.data_dir = FsPath(
            os.path.expandvars(os.path.expanduser(str(self.get_parameter("data_dir").value)))
        )
        self.map_archive_dir = FsPath(
            os.path.expandvars(os.path.expanduser(str(self.get_parameter("map_archive_dir").value)))
        )
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.map_archive_dir.mkdir(parents=True, exist_ok=True)

        self._projects = self._load_json("projects.json", [])
        self._maps = self._load_json("maps.json", [])
        builtin_maps = self._load_builtin_maps()
        self._default_builtin_floor: Optional[str] = builtin_maps["default_floor"]
        self._default_builtin_map_id: Optional[str] = builtin_maps["default_map_id"]
        self._builtin_maps = list(builtin_maps["maps"])
        self._annotations = self._load_json("annotations.json", [])
        self._tasks = self._load_json("tasks.json", [])
        self._sessions = self._load_json("mapping_sessions.json", [])
        self._settings = self._load_json(
            "settings.json",
            {"selected_map_id": None, "working_map_id": None, "active_task": None},
        )
        self._normalize_runtime_state_on_startup()
        self._mapping_processes: Dict[str, Dict[str, Any]] = {}
        self._camera_workers: Dict[str, _CameraProxyWorker] = {}
        self._last_preflight: Optional[Dict[str, Any]] = None
        self._preflight_lock = threading.Lock()
        self._preflight_run_lock = threading.Lock()
        self._preflight_running: Optional[Dict[str, Any]] = None
        self._map_file_cache_lock = threading.Lock()
        self._map_file_cache: Dict[str, Dict[str, Any]] = {}
        self._map_file_summary_cache: Dict[str, Dict[str, Any]] = {}
        self._last_scan_overlay_update = 0.0
        self._last_scan_overlay_points: List[Dict[str, float]] = []
        self._startup_map_sync_timer = None
        self._startup_map_sync_attempts = 0
        self._startup_map_sync_lock = threading.Lock()
        self._startup_map_sync_inflight = False

        self.floor_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("floor_goal_topic").value),
            10,
        )
        self.stop_task_pub = self.create_publisher(
            String,
            str(self.get_parameter("stop_task_topic").value),
            10,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self.active_waypoint_pub = self.create_publisher(
            String,
            str(self.get_parameter("active_waypoint_topic").value),
            10,
        )
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initialpose_topic").value),
            10,
        )
        self.stair_zones_pub = self.create_publisher(
            String,
            str(self.get_parameter("stair_zones_topic").value),
            10,
        )
        self.clear_costmap_clients = []
        if ClearEntireCostmap is not None:
            self.clear_costmap_clients = [
                self.create_client(ClearEntireCostmap, str(service_name))
                for service_name in self.get_parameter("task_clear_costmap_services").value
            ]
        self.load_map_client = None
        if LoadMap is not None:
            self.load_map_client = self.create_client(
                LoadMap,
                str(self.get_parameter("map_server_load_map_service").value),
            )

        self._state: Dict[str, Any] = {
            "floor": None,
            "stair_status": None,
            "gait_command": None,
            "gait_result": None,
            "usage_mode_result": None,
            "localization_ok": None,
            "navigation_status": None,
            "navigation_status_parsed": None,
            "battery": None,
            "pose": None,
            "path": {"version": 0, "points": []},
            "pose_history": [],
            "map": None,
            "map_version": 0,
            "dynamic_obstacles": [],
            "detections": None,
            "inspection_status": None,
            "active_waypoint": None,
            "relocalization_result": None,
            "map_relocalization_required": None,
            "events": [],
            "topics": {},
        }

        self._create_subscriptions()
        self.create_timer(1.0, self._tick_active_task)
        self.create_timer(2.0, self._publish_selected_stair_zones)
        self._server = self._start_http_server()
        if bool(self.get_parameter("startup_sync_selected_map_to_nav2").value):
            delay_s = max(0.1, float(self.get_parameter("startup_sync_selected_map_delay_s").value))
            self._startup_map_sync_timer = self.create_timer(delay_s, self._sync_selected_map_to_nav2_on_startup)

    def _declare_parameters(self) -> None:
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("data_dir", "~/.m20pro_web")
        self.declare_parameter("map_archive_dir", "~/m20pro_maps")
        self.declare_parameter("map_manifest", "")
        self.declare_parameter("map_server_load_map_service", "/map_server/load_map")
        self.declare_parameter("map_select_load_nav2_map", True)
        self.declare_parameter("map_select_load_timeout_s", 8.0)
        self.declare_parameter("map_select_wait_match_timeout_s", 5.0)
        self.declare_parameter("map_select_wait_match_poll_s", 0.1)
        self.declare_parameter("startup_sync_selected_map_to_nav2", True)
        self.declare_parameter("startup_sync_selected_map_delay_s", 1.5)
        self.declare_parameter("startup_sync_selected_map_max_attempts", 12)
        self.declare_parameter("factory_host", "10.21.31.106")
        self.declare_parameter("factory_user", "user")
        self.declare_parameter("factory_active_map", "/var/opt/robot/data/maps/active")
        self.declare_parameter(
            "factory_mapping_start_command",
            'ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} '
            '"nohup sudo -n drmap mapping -b -s -n {map_name} > /tmp/m20pro_drmap_mapping_{session_id}.log 2>&1 &"',
        )
        self.declare_parameter(
            "factory_mapping_finish_command",
            'ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} '
            '"sudo -n drmap stop_mapping"',
        )
        self.declare_parameter(
            "factory_mapping_cancel_command",
            'ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} '
            '"sudo -n drmap stop_mapping"',
        )
        self.declare_parameter("mapping_command_timeout_s", 120.0)
        self.declare_parameter("map_import_timeout_s", 180.0)
        self.declare_parameter("enable_stair_zone_postprocess", True)
        self.declare_parameter("stair_zones_topic", "/m20pro/stair_zones")
        self.declare_parameter("goal_reached_tolerance_m", 0.3)
        self.declare_parameter("task_goal_resend_interval_s", 5.0)
        self.declare_parameter("task_goal_accept_timeout_s", 12.0)
        self.declare_parameter("task_waypoint_timeout_s", 180.0)
        self.declare_parameter("task_progress_min_pose_movement_m", 0.08)
        self.declare_parameter("task_progress_min_distance_delta_m", 0.12)
        self.declare_parameter("task_timeline_max_events", 80)
        self.declare_parameter("task_start_settle_s", 0.5)
        self.declare_parameter("task_start_pose_timeout_s", 3.0)
        self.declare_parameter("task_runtime_localization_lost_stop_s", 2.0)
        self.declare_parameter("task_runtime_scan_lost_stop_s", 2.0)
        self.declare_parameter("task_runtime_scan_timeout_s", 1.0)
        self.declare_parameter("task_runtime_scan_min_finite_ranges", 20)
        self.declare_parameter("relocalization_nav_timeout_s", 3.0)
        self.declare_parameter("task_stop_zero_cmd_samples", 10)
        self.declare_parameter(
            "task_clear_costmap_services",
            [
                "/global_costmap/clear_entirely_global_costmap",
                "/local_costmap/clear_entirely_local_costmap",
            ],
        )
        self.declare_parameter("default_task_dwell_s", 5.0)
        self.declare_parameter("default_transition_dwell_s", 0.0)
        self.declare_parameter("default_charge_dwell_s", 0.0)
        self.declare_parameter("floor_goal_topic", "/m20pro/floor_goal")
        self.declare_parameter("stop_task_topic", "/m20pro/stop_task")
        self.declare_parameter("active_waypoint_topic", "/m20pro/active_waypoint")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("initialpose_covariance_xy", 0.25)
        self.declare_parameter("initialpose_covariance_yaw", 0.0685)
        self.declare_parameter("initialpose_publish_repeats", 1)
        self.declare_parameter("initialpose_publish_interval_s", 0.15)
        self.declare_parameter("relocalization_verify_timeout_s", 8.0)
        self.declare_parameter("relocalization_pose_tolerance_m", 2.0)
        self.declare_parameter("robot_pose_display_yaw_offset_rad", 0.0)
        self.declare_parameter("current_floor_topic", "/m20pro/current_floor")
        self.declare_parameter("stair_status_topic", "/m20pro/stair_status")
        self.declare_parameter("gait_command_topic", "/m20pro/gait_command")
        self.declare_parameter("gait_result_topic", "/m20pro_tcp_bridge/gait_result")
        self.declare_parameter("usage_mode_result_topic", "/m20pro_tcp_bridge/usage_mode_result")
        self.declare_parameter("localization_ok_topic", "/m20pro_tcp_bridge/localization_ok")
        self.declare_parameter("navigation_status_topic", "/m20pro_tcp_bridge/navigation_status")
        self.declare_parameter("battery_topic", "/BATTERY_DATA")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_overlay_max_points", 720)
        self.declare_parameter("scan_overlay_update_min_interval_s", 0.1)
        self.declare_parameter("scan_overlay_min_range_m", 0.05)
        self.declare_parameter("scan_overlay_max_range_m", 30.0)
        self.declare_parameter("scan_overlay_offset_x_m", 0.0)
        self.declare_parameter("scan_overlay_offset_y_m", 0.0)
        self.declare_parameter("scan_overlay_offset_yaw_rad", 0.0)
        self.declare_parameter("odom_topic", "/ODOM")
        self.declare_parameter("pose_topic", "/m20pro_tcp_bridge/map_pose")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("local_costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("global_costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("dynamic_obstacle_topic", "/dynamic_obstacle_markers")
        self.declare_parameter("relocalization_result_topic", "/m20pro_tcp_bridge/relocalization_result")
        self.declare_parameter("detections_topic", "/m20pro_yolov8_inspection/detections")
        self.declare_parameter("events_topic", "/m20pro_yolov8_inspection/events")
        self.declare_parameter("inspection_status_topic", "/m20pro_yolov8_inspection/status")
        self.declare_parameter("annotated_image_topic", "/m20pro_yolov8_inspection/annotated_image")
        self.declare_parameter("subscribe_annotated_image", False)
        self.declare_parameter("enable_camera_proxy", False)
        self.declare_parameter("front_camera_url", "rtsp://10.21.31.103:8554/video1")
        self.declare_parameter("rear_camera_url", "rtsp://10.21.31.103:8554/video2")
        self.declare_parameter("camera_proxy_backend", "ffmpeg_mjpeg")
        self.declare_parameter("camera_proxy_fps", 10.0)
        self.declare_parameter("camera_proxy_jpeg_quality", 45)
        self.declare_parameter("camera_proxy_ffmpeg_mjpeg_qscale", 5)
        self.declare_parameter("camera_proxy_max_width", 480)
        self.declare_parameter("camera_proxy_transport", "tcp")
        self.declare_parameter("camera_proxy_low_latency", True)
        self.declare_parameter("camera_proxy_socket_send_buffer_bytes", 65536)
        self.declare_parameter("camera_proxy_snapshot_keepalive_s", 1.5)
        self.declare_parameter(
            "camera_proxy_ffmpeg_options",
            "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;500000|stimeout;3000000",
        )
        self.declare_parameter("camera_proxy_open_timeout_s", 3.0)
        self.declare_parameter("camera_proxy_read_timeout_s", 3.0)
        self.declare_parameter("camera_proxy_reconnect_s", 5.0)
        self.declare_parameter("camera_proxy_frame_timeout_s", 2.0)
        self.declare_parameter("max_path_points", 800)
        self.declare_parameter("max_events", 30)
        self.declare_parameter("api_slow_log_threshold_s", 1.0)
        self.declare_parameter("preflight_topic_timeout_s", 5.0)
        self.declare_parameter("preflight_settle_wait_s", 6.0)

    def _topic(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _json_path(self, name: str) -> FsPath:
        return self.data_dir / name

    def _load_json(self, name: str, default: Any) -> Any:
        path = self._json_path(name)
        if not path.exists():
            return default
        try:
            with path.open("r", encoding="utf-8") as file:
                return json.load(file)
        except Exception as exc:
            self.get_logger().warning(f"failed to read {path}: {exc}")
            return default

    def _save_json(self, name: str, value: Any) -> None:
        path = self._json_path(name)
        tmp = path.with_suffix(path.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as file:
            json.dump(value, file, ensure_ascii=False, indent=2)
        os.replace(tmp, path)

    def _load_builtin_maps(self) -> Dict[str, Any]:
        manifest_path = self._map_manifest_path()
        result = load_builtin_maps_from_manifest(
            manifest_path,
            resolve_path=self._resolve_path,
            derived_payload=builtin_map_derived_payload,
        )
        for warning in result.get("warnings") or []:
            self.get_logger().warning(str(warning))
        return result

    def _map_manifest_path(self) -> Optional[FsPath]:
        value = str(self.get_parameter("map_manifest").value or "").strip()
        if value:
            return FsPath(os.path.expandvars(os.path.expanduser(self._resolve_path(value))))
        try:
            return FsPath(get_package_share_directory("m20pro_bringup")) / "config" / "map_manifest.yaml"
        except PackageNotFoundError:
            return None

    def _resolve_path(self, value: str) -> str:
        path = os.path.expandvars(os.path.expanduser(str(value).strip()))
        if path.startswith("package://"):
            package_and_path = path[len("package://") :]
            package_name, _, relative_path = package_and_path.partition("/")
            if not package_name or not relative_path:
                raise ValueError(f"invalid package path: {value}")
            return os.path.join(get_package_share_directory(package_name), relative_path)
        return path

    def _all_maps_unlocked(self) -> List[Dict[str, Any]]:
        return all_map_records(self._builtin_maps, self._maps)

    def _find_map_record_unlocked(self, map_id: Optional[str]) -> Optional[Dict[str, Any]]:
        return find_map_record(self._builtin_maps, self._maps, map_id)

    def _default_map_id_unlocked(self) -> Optional[str]:
        return default_map_id(self._builtin_maps, self._maps, self._default_builtin_map_id)

    def _normalize_runtime_state_on_startup(self) -> None:
        changed = False
        selected_map_id = str(self._settings.get("selected_map_id") or "").strip()
        working_map_id = str(self._settings.get("working_map_id") or "").strip()
        if selected_map_id and not self._find_map_record_unlocked(selected_map_id):
            self.get_logger().warning(f"selected map {selected_map_id} no longer exists; keeping live display mode")
            self._settings["selected_map_id"] = None
            selected_map_id = ""
            changed = True
        if selected_map_id:
            if working_map_id != selected_map_id:
                self._settings["working_map_id"] = selected_map_id
                working_map_id = selected_map_id
                changed = True
        elif working_map_id and not self._find_map_record_unlocked(working_map_id):
            self.get_logger().warning(f"working map {working_map_id} no longer exists; clearing startup working map")
            self._settings["working_map_id"] = None
            working_map_id = ""
            changed = True
        if not selected_map_id and not working_map_id:
            default_map_id = self._default_map_id_unlocked()
            if default_map_id:
                self._settings["working_map_id"] = default_map_id
                changed = True
        for item in self._annotations:
            before = json.dumps(item, ensure_ascii=False, sort_keys=True)
            normalize_annotation_semantics(item)
            after = json.dumps(item, ensure_ascii=False, sort_keys=True)
            if before != after:
                changed = True
        task_runtime = normalize_startup_task_runtime_state(
            self._settings,
            self._tasks,
            now_text_value=now_text(),
        )
        if task_runtime.get("changed"):
            self._settings = dict(task_runtime["settings"])
            self._tasks = list(task_runtime["tasks"])
            changed = True
        if changed:
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
            self._save_json("annotations.json", self._annotations)

    def _sync_selected_map_to_nav2_on_startup(self) -> None:
        with self._startup_map_sync_lock:
            if self._startup_map_sync_inflight:
                return
            self._startup_map_sync_inflight = True
        thread = threading.Thread(target=self._run_startup_selected_map_sync, daemon=True)
        thread.start()

    def _run_startup_selected_map_sync(self) -> None:
        try:
            self._run_startup_selected_map_sync_once()
        finally:
            with self._startup_map_sync_lock:
                self._startup_map_sync_inflight = False

    def _run_startup_selected_map_sync_once(self) -> None:
        self._startup_map_sync_attempts += 1
        attempt = self._startup_map_sync_attempts
        max_attempts = max(1, int(self.get_parameter("startup_sync_selected_map_max_attempts").value))

        def finish_timer() -> None:
            timer = self._startup_map_sync_timer
            if timer is None:
                return
            try:
                timer.cancel()
                self.destroy_timer(timer)
            except Exception as exc:
                self.get_logger().warning(f"failed to stop startup selected-map sync timer: {exc}")
            self._startup_map_sync_timer = None

        def store_startup_sync(result: Dict[str, Any]) -> None:
            with self._data_lock:
                self._settings["startup_map_sync"] = result
                self._save_json("settings.json", self._settings)

        with self._data_lock:
            selected_map_id = str(self._working_map_id_unlocked() or "").strip()
            if not selected_map_id:
                store_startup_sync(
                    startup_map_sync_skipped_payload(
                        reason="working_map_missing",
                        attempt=attempt,
                        max_attempts=max_attempts,
                        now_text=now_text,
                    )
                )
                finish_timer()
                return
            record = self._find_map_record_unlocked(selected_map_id)
        if record is None:
            result = startup_map_sync_missing_record_payload(
                selected_map_id=selected_map_id,
                attempt=attempt,
                max_attempts=max_attempts,
                now_text=now_text,
            )
            self.get_logger().warning(result["message"])
            store_startup_sync(result)
            self._append_event("启动同步固定地图失败", result)
            finish_timer()
            return
        load_result = self._load_selected_map_into_nav2(record)
        result = startup_map_sync_result_payload(
            selected_map_id=selected_map_id,
            map_name=record.get("name"),
            nav2_load_map=load_result,
            attempt=attempt,
            max_attempts=max_attempts,
            now_text=now_text,
        )
        with self._data_lock:
            self._settings["startup_map_sync"] = result
            if load_result.get("ok") and bool(load_result.get("loaded")):
                self._settings["map_relocalization_required"] = map_relocalization_required_payload(
                    map_id=selected_map_id,
                    map_name=record.get("name"),
                    yaml_path=load_result.get("yaml_path"),
                    reason="startup_sync",
                    now_text=now_text,
                )
            self._save_json("settings.json", self._settings)
        if load_result.get("ok") and bool(load_result.get("loaded")):
            with self._lock:
                self._state["localization_ok"] = False
                self._state["pose"] = None
                self._state["pose_history"] = []
                self._state["path"] = {"version": int(self._state.get("path", {}).get("version", 0) or 0) + 1, "points": []}
            self._append_event("启动同步固定地图", result)
            self.get_logger().info(
                "startup selected-map sync loaded Nav2 map: %s" % str(load_result.get("yaml_path") or "")
            )
            finish_timer()
        elif load_result.get("ok"):
            self._append_event("启动检查固定地图", result)
            finish_timer()
        else:
            self._append_event("启动同步固定地图失败", result)
            self.get_logger().warning(str(load_result["message"]))
            retry = startup_map_sync_retry_decision(
                load_result,
                attempt=attempt,
                max_attempts=max_attempts,
            )
            if retry.get("retry"):
                self.get_logger().info(
                    "startup selected-map sync waiting for Nav2 map service; "
                    "attempt %s/%s code=%s next=%s"
                    % (
                        str(retry.get("attempt")),
                        str(retry.get("max_attempts")),
                        str(retry.get("code")),
                        str(retry.get("next_attempt")),
                    )
                )
            else:
                finish_timer()

    def _append_event(self, text: str, parsed: Optional[Dict[str, Any]] = None) -> None:
        max_events = int(self.get_parameter("max_events").value)
        event = {
            "last_update": time.time(),
            "raw": text,
            "parsed": parsed or {"source": "web_dashboard"},
        }
        with self._lock:
            events = list(self._state["events"])
            events.append(event)
            self._state["events"] = events[-max_events:]

    def _create_subscriptions(self) -> None:
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        scan_qos = QoSProfile(depth=5)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(String, self._topic("current_floor_topic"), self._on_current_floor, 10)
        self.create_subscription(String, self._topic("stair_status_topic"), self._on_stair_status, 10)
        self.create_subscription(String, self._topic("gait_command_topic"), self._on_gait_command, 10)
        self.create_subscription(String, self._topic("gait_result_topic"), self._on_gait_result, 10)
        self.create_subscription(String, self._topic("usage_mode_result_topic"), self._on_usage_mode_result, 10)
        self.create_subscription(Bool, self._topic("localization_ok_topic"), self._on_localization_ok, 10)
        self.create_subscription(String, self._topic("navigation_status_topic"), self._on_navigation_status, 10)
        if BatteryData is not None:
            self.create_subscription(BatteryData, self._topic("battery_topic"), self._on_battery, 10)
        else:
            self.get_logger().warning("drdds.msg.BatteryData is unavailable; battery display is disabled")
        self.create_subscription(LaserScan, self._topic("scan_topic"), self._on_scan, scan_qos)
        self.create_subscription(Odometry, self._topic("odom_topic"), self._on_odom, 10)
        self.create_subscription(PoseStamped, self._topic("pose_topic"), self._on_pose, 20)
        self.create_subscription(RosPath, self._topic("plan_topic"), self._on_path, 5)
        self.create_subscription(String, self._topic("active_waypoint_topic"), self._on_active_waypoint, 10)
        self.create_subscription(OccupancyGrid, self._topic("map_topic"), self._on_map, map_qos)
        self.create_subscription(OccupancyGrid, self._topic("local_costmap_topic"), self._on_local_costmap, 2)
        self.create_subscription(OccupancyGrid, self._topic("global_costmap_topic"), self._on_global_costmap, 2)
        self.create_subscription(MarkerArray, self._topic("dynamic_obstacle_topic"), self._on_markers, 10)
        self.create_subscription(
            String,
            self._topic("relocalization_result_topic"),
            self._on_relocalization_result,
            10,
        )
        self.create_subscription(String, self._topic("detections_topic"), self._on_detections, 10)
        self.create_subscription(String, self._topic("events_topic"), self._on_event, 10)
        self.create_subscription(String, self._topic("inspection_status_topic"), self._on_inspection_status, 10)

        if bool(self.get_parameter("subscribe_annotated_image").value):
            self.create_subscription(Image, self._topic("annotated_image_topic"), self._on_annotated_image, 2)

    def _mark_topic(self, topic_key: str) -> None:
        self._state["topics"][topic_key] = {
            "last_update": time.time(),
            "available": True,
        }

    def _on_current_floor(self, msg: String) -> None:
        with self._lock:
            self._state["floor"] = msg.data
            self._mark_topic("current_floor")

    def _on_stair_status(self, msg: String) -> None:
        with self._lock:
            self._state["stair_status"] = msg.data
            self._mark_topic("stair_status")
        self._handle_navigation_status_for_task(msg.data)

    def _on_gait_command(self, msg: String) -> None:
        with self._lock:
            self._state["gait_command"] = msg.data
            self._mark_topic("gait_command")

    def _on_gait_result(self, msg: String) -> None:
        with self._lock:
            self._state["gait_result"] = msg.data
            self._mark_topic("gait_result")

    def _on_usage_mode_result(self, msg: String) -> None:
        with self._lock:
            self._state["usage_mode_result"] = msg.data
            self._mark_topic("usage_mode_result")

    def _on_localization_ok(self, msg: Bool) -> None:
        with self._lock:
            self._state["localization_ok"] = bool(msg.data)
            self._mark_topic("localization_ok")

    def _on_navigation_status(self, msg: String) -> None:
        with self._lock:
            self._state["navigation_status"] = msg.data
            self._state["navigation_status_parsed"] = parse_key_value_status(msg.data)
            self._mark_topic("navigation_status")

    def _on_battery(self, msg: Any) -> None:
        batteries = []
        for index, item in enumerate(getattr(msg, "data", []) or []):
            temperatures = [
                float(value)
                for value in (getattr(item, "battery_temperature", []) or [])
                if math.isfinite(float(value))
            ]
            avg_temp = sum(temperatures) / len(temperatures) if temperatures else None
            serial_raw = getattr(item, "battery_serialnum", "")
            if isinstance(serial_raw, (bytes, bytearray)):
                serial = serial_raw.decode("utf-8", errors="ignore").strip("\x00").strip()
            elif isinstance(serial_raw, str):
                serial = serial_raw.strip("\x00").strip()
            else:
                try:
                    serial_values = list(serial_raw)
                except TypeError:
                    serial_values = None
                if serial_values is None:
                    serial = str(serial_raw).strip("\x00").strip()
                else:
                    chars = []
                    for value in serial_values:
                        try:
                            ivalue = int(value)
                        except (TypeError, ValueError):
                            continue
                        if ivalue == 0:
                            continue
                        chars.append(chr(ivalue))
                    serial = "".join(chars).strip()
            batteries.append(
                {
                    "index": index,
                    "level": int(getattr(item, "battery_level", 0)),
                    "voltage_v": float(getattr(item, "voltage", 0)) * 0.01,
                    "current_a": float(getattr(item, "current", 0)) * 0.01,
                    "remaining_mah": float(getattr(item, "remaining_capacity", 0)) * 10.0,
                    "nominal_mah": float(getattr(item, "nominal_capacity", 0)) * 10.0,
                    "cycles": int(getattr(item, "cycles", 0)),
                    "temperature_c": avg_temp,
                    "mos_state": int(getattr(item, "mos_state", 0)),
                    "protected_state": int(getattr(item, "protected_state", 0)),
                    "serial": serial,
                }
            )
        battery = {
            "last_update": time.time(),
            "count": len(batteries),
            "packs": batteries,
            "primary": batteries[0] if batteries else None,
        }
        with self._lock:
            self._state["battery"] = battery
            self._mark_topic("battery")

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.time()
        ranges_count = len(msg.ranges)
        finite_count = sum(1 for value in msg.ranges if math.isfinite(float(value)))
        points = self._last_scan_overlay_points
        min_interval_s = max(0.0, float(self.get_parameter("scan_overlay_update_min_interval_s").value))
        if now - self._last_scan_overlay_update >= min_interval_s:
            min_range = max(
                float(getattr(msg, "range_min", 0.0) or 0.0),
                float(self.get_parameter("scan_overlay_min_range_m").value),
            )
            max_range_param = float(self.get_parameter("scan_overlay_max_range_m").value)
            sensor_max = float(getattr(msg, "range_max", 0.0) or 0.0)
            max_range = max_range_param if max_range_param > 0.0 else sensor_max
            if sensor_max > 0.0:
                max_range = min(max_range, sensor_max)
            max_points = max(0, int(self.get_parameter("scan_overlay_max_points").value))
            step = 1
            if max_points > 0 and ranges_count > max_points:
                step = max(1, math.ceil(ranges_count / max_points))
            rebuilt_points: List[Dict[str, float]] = []
            for index in range(0, ranges_count, step):
                try:
                    distance = float(msg.ranges[index])
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(distance) or distance < min_range or distance > max_range:
                    continue
                angle = float(msg.angle_min) + index * float(msg.angle_increment)
                rebuilt_points.append(
                    {
                        "x": distance * math.cos(angle),
                        "y": distance * math.sin(angle),
                    }
                )
            points = rebuilt_points
            self._last_scan_overlay_points = points
            self._last_scan_overlay_update = now
        with self._lock:
            self._state["scan"] = {
                "last_update": now,
                "stamp": stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "ranges": ranges_count,
                "finite_ranges": finite_count,
                "angle_min": float(msg.angle_min),
                "angle_max": float(msg.angle_max),
                "range_min": float(getattr(msg, "range_min", 0.0) or 0.0),
                "range_max": float(getattr(msg, "range_max", 0.0) or 0.0),
                "overlay_points": len(points),
                "points": list(points),
            }
            self._mark_topic("scan")

    def _on_odom(self, msg: Odometry) -> None:
        pose = pose_to_dict(msg.pose.pose)
        with self._lock:
            self._state["odom"] = {
                "last_update": time.time(),
                "stamp": stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "pose": pose,
                "finite": is_finite_pose_dict(pose),
            }
            self._mark_topic("odom")

    def _on_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            pose = pose_to_dict(msg.pose)
            if not is_plausible_pose_dict(pose):
                self._mark_topic("pose_invalid")
                return
            raw_display_offset = float(self.get_parameter("robot_pose_display_yaw_offset_rad").value)
            display_offset = raw_display_offset if math.isfinite(raw_display_offset) else 0.0
            pose["display_yaw_offset_rad"] = display_offset
            pose["display_yaw_offset_deg"] = math.degrees(display_offset)
            if abs(display_offset) > 1e-12:
                display_yaw = wrap_angle(pose["yaw"] + display_offset)
                pose["display_yaw"] = display_yaw
                pose["display_yaw_deg"] = math.degrees(display_yaw)
            stamp = stamp_to_float(msg.header.stamp)
            if stamp is not None:
                pose["stamp"] = stamp
            pose["last_update"] = time.time()
            self._state["pose"] = pose
            self._mark_topic("pose")

    def _on_path(self, msg: RosPath) -> None:
        max_points = max(2, int(self.get_parameter("max_path_points").value))
        raw_poses = list(msg.poses)
        path_last_point = None
        if raw_poses:
            last_pose = raw_poses[-1].pose.position
            path_last_point = {
                "x": float(last_pose.x),
                "y": float(last_pose.y),
                "z": float(last_pose.z),
            }
        poses = raw_poses
        if len(raw_poses) > max_points:
            step = max(1, math.ceil((len(raw_poses) - 1) / max(1, max_points - 1)))
            poses = raw_poses[::step]
            if poses[-1] is not raw_poses[-1]:
                poses.append(raw_poses[-1])
        points = [
            {
                "x": float(item.pose.position.x),
                "y": float(item.pose.position.y),
                "z": float(item.pose.position.z),
            }
            for item in poses
        ]
        with self._lock:
            self._state["path"] = {
                "version": int(self._state["path"]["version"]) + 1,
                "frame_id": msg.header.frame_id,
                "points": points,
                "last_point": path_last_point,
                "point_count": len(points),
                "raw_point_count": len(raw_poses),
                "last_update": time.time(),
            }
            self._mark_topic("path")

    def _on_active_waypoint(self, msg: String) -> None:
        with self._lock:
            self._state["active_waypoint"] = {
                "last_update": time.time(),
                "raw": msg.data,
                "parsed": parse_json_text(msg.data),
            }
            self._mark_topic("active_waypoint")

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        origin = pose_to_dict(info.origin)
        map_payload = {
            "available": True,
            "version": int(time.time() * 1000),
            "last_update": time.time(),
            "frame_id": msg.header.frame_id,
            "stamp": stamp_to_float(msg.header.stamp),
            "width": int(info.width),
            "height": int(info.height),
            "resolution": float(info.resolution),
            "origin": origin,
            "data": list(msg.data),
        }
        with self._lock:
            self._state["map"] = map_payload
            self._state["map_version"] = map_payload["version"]
            self._mark_topic("map")

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        info = msg.info
        with self._lock:
            self._state["local_costmap"] = {
                "last_update": time.time(),
                "stamp": stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "width": int(info.width),
                "height": int(info.height),
                "resolution": float(info.resolution),
            }
            self._mark_topic("local_costmap")

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        info = msg.info
        with self._lock:
            self._state["global_costmap"] = {
                "last_update": time.time(),
                "stamp": stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "width": int(info.width),
                "height": int(info.height),
                "resolution": float(info.resolution),
            }
            self._mark_topic("global_costmap")

    def _on_markers(self, msg: MarkerArray) -> None:
        markers: List[Dict[str, Any]] = []
        for item in msg.markers:
            if item.action in (Marker.DELETE, Marker.DELETEALL):
                continue
            pose = item.pose
            markers.append(
                {
                    "ns": item.ns,
                    "id": int(item.id),
                    "type": int(item.type),
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "z": float(pose.position.z),
                    "scale_x": float(item.scale.x),
                    "scale_y": float(item.scale.y),
                    "scale_z": float(item.scale.z),
                }
            )
        with self._lock:
            self._state["dynamic_obstacles"] = markers
            self._mark_topic("dynamic_obstacles")

    def _on_detections(self, msg: String) -> None:
        with self._lock:
            self._state["detections"] = {
                "last_update": time.time(),
                "raw": msg.data,
                "parsed": parse_json_text(msg.data),
            }
            self._mark_topic("detections")

    def _on_inspection_status(self, msg: String) -> None:
        with self._lock:
            self._state["inspection_status"] = {
                "last_update": time.time(),
                "raw": msg.data,
                "parsed": parse_json_text(msg.data),
            }
            self._mark_topic("inspection_status")

    def _on_relocalization_result(self, msg: String) -> None:
        with self._lock:
            self._state["relocalization_result"] = {
                "last_update": time.time(),
                "raw": msg.data,
                "parsed": parse_json_text(msg.data),
            }
            self._mark_topic("relocalization_result")

    def _factory_localization_ok(self, state: Dict[str, Any]) -> bool:
        if state.get("localization_ok") is True:
            return True
        nav_status_parsed = (
            state.get("navigation_status_parsed")
            if isinstance(state.get("navigation_status_parsed"), dict)
            else {}
        )
        if not nav_status_parsed and state.get("navigation_status") is not None:
            nav_status_parsed = parse_key_value_status(str(state.get("navigation_status") or ""))
        location = nav_status_parsed.get("location")
        try:
            return float(location) == 0.0
        except (TypeError, ValueError):
            return False

    def _on_event(self, msg: String) -> None:
        max_events = int(self.get_parameter("max_events").value)
        event = {
            "last_update": time.time(),
            "raw": msg.data,
            "parsed": parse_json_text(msg.data),
        }
        with self._lock:
            events = list(self._state["events"])
            events.append(event)
            self._state["events"] = events[-max_events:]
            self._mark_topic("events")

    def _on_annotated_image(self, msg: Image) -> None:
        with self._lock:
            self._state["annotated_image"] = {
                "last_update": time.time(),
                "width": int(msg.width),
                "height": int(msg.height),
                "encoding": msg.encoding,
            }
            self._mark_topic("annotated_image")

    def _snapshot(self, include_debug: bool = True) -> Dict[str, Any]:
        now = time.time()
        with self._lock:
            snapshot = dict(self._state)
            snapshot["path"] = dict(self._state["path"])
            snapshot["pose_history"] = list(self._state.get("pose_history") or [])
            snapshot["dynamic_obstacles"] = list(self._state["dynamic_obstacles"])
            snapshot["events"] = list(self._state["events"])
            for key in ("scan", "odom", "local_costmap", "global_costmap", "active_waypoint"):
                if isinstance(self._state.get(key), dict):
                    snapshot[key] = dict(self._state[key])
            snapshot["topics"] = {
                key: dict(value)
                for key, value in self._state["topics"].items()
            }
            map_status_runtime = {"map": dict(self._state.get("map") or {})}
            snapshot.pop("map", None)
        snapshot["perception_status"] = perception_status_payload(
            snapshot,
            now=now,
            now_text=now_text,
        )
        with self._data_lock:
            snapshot["selected_map_id"] = self._settings.get("selected_map_id")
            snapshot["working_map_id"] = self._settings.get("working_map_id")
            snapshot["active_task"] = self._settings.get("active_task")
            snapshot["map_relocalization_required"] = self._settings.get("map_relocalization_required")
            snapshot["startup_map_sync"] = self._settings.get("startup_map_sync")
        snapshot["effective_map_id"] = self._effective_map_id(runtime_state=map_status_runtime)
        self._remember_working_map_id(snapshot["effective_map_id"], reason="state_effective_map")
        with self._data_lock:
            snapshot["working_map_id"] = self._settings.get("working_map_id")
        pose = snapshot.get("pose") if isinstance(snapshot.get("pose"), dict) else {}
        pose_age = pose_age_sec(pose, now)
        pose_timeout_s = max(0.5, float(self.get_parameter("task_start_pose_timeout_s").value))
        snapshot["selected_map_status"] = self._selected_map_status_payload(map_status_runtime)
        snapshot["factory_localization_ok"] = self._factory_localization_ok(snapshot)
        clearance = self._map_relocalization_clearance_payload(
            snapshot=snapshot,
            selected_map_status=snapshot["selected_map_status"],
            pose=pose,
            pose_age=pose_age,
            pose_timeout_s=pose_timeout_s,
            now=now,
        )
        snapshot["map_relocalization_clearance"] = clearance
        if clearance.get("clear"):
            self._clear_map_relocalization_required(clearance)
            snapshot["map_relocalization_required"] = None
            snapshot["localization_ok"] = True
            snapshot["factory_localization_ok"] = True
        elif snapshot.get("map_relocalization_required"):
            snapshot["factory_localization_ok"] = bool(clearance.get("factory_localization_ok"))
            snapshot["localization_ok"] = False
        with self._preflight_lock:
            snapshot["preflight"] = self._preflight_with_age_unlocked()
        nav_readiness = None
        snapshot["pose_age_sec"] = pose_age
        snapshot["pose_timeout_s"] = pose_timeout_s
        snapshot["pose_fresh"] = bool(
            is_plausible_pose_dict(pose)
            and pose_age is not None
            and pose_age <= pose_timeout_s
        )
        snapshot["localization_status"] = localization_status_payload(
            localization_ok=snapshot.get("localization_ok"),
            pose=pose,
            pose_age_sec=pose_age,
            pose_timeout_s=pose_timeout_s,
            navigation_status=snapshot.get("navigation_status"),
            factory_localization_ok=snapshot.get("factory_localization_ok"),
            relocalization_result=snapshot.get("relocalization_result"),
            map_relocalization_required=snapshot.get("map_relocalization_required"),
            now_time=now,
            now_text=now_text,
        )

        snapshot["ok"] = True
        snapshot["node_time"] = now
        snapshot["node_time_text"] = now_text()
        snapshot["scan_overlay_offset"] = {
            "x": float(self.get_parameter("scan_overlay_offset_x_m").value),
            "y": float(self.get_parameter("scan_overlay_offset_y_m").value),
            "yaw": float(self.get_parameter("scan_overlay_offset_yaw_rad").value),
        }
        snapshot["camera_proxy"] = self._camera_proxy_status_payload()
        for value in snapshot["topics"].values():
            last_update = value.get("last_update")
            value["age_sec"] = None if last_update is None else max(0.0, now - float(last_update))
        snapshot["debug_included"] = bool(include_debug)
        if not include_debug:
            snapshot["events"] = []
            snapshot["topics"] = {}
        return snapshot

    def _map_relocalization_lock_loaded_time(self, lock_payload: Any) -> Optional[float]:
        if not isinstance(lock_payload, dict):
            return None
        loaded_at = str(lock_payload.get("loaded_at") or "").strip()
        if not loaded_at:
            return None
        for fmt in ("%Y-%m-%d %H:%M:%S",):
            try:
                return time.mktime(time.strptime(loaded_at, fmt))
            except ValueError:
                continue
        return None

    def _map_relocalization_clearance_payload(
        self,
        *,
        snapshot: Dict[str, Any],
        selected_map_status: Dict[str, Any],
        pose: Dict[str, Any],
        pose_age: Optional[float],
        pose_timeout_s: float,
        now: float,
    ) -> Dict[str, Any]:
        lock_payload = snapshot.get("map_relocalization_required")
        nav_status_parsed = (
            snapshot.get("navigation_status_parsed")
            if isinstance(snapshot.get("navigation_status_parsed"), dict)
            else {}
        )
        factory_localization_ok = bool(
            snapshot.get("localization_ok") is True
            or nav_status_parsed.get("location") == 0
            or str(nav_status_parsed.get("location") or "") == "0"
        )
        return map_relocalization_clearance_payload(
            map_relocalization_required=lock_payload,
            selected_map_id=snapshot.get("effective_map_id") or snapshot.get("selected_map_id"),
            selected_map_status=selected_map_status,
            localization_ok=snapshot.get("localization_ok"),
            factory_localization_ok=factory_localization_ok,
            pose=pose,
            pose_age_sec=pose_age,
            pose_timeout_s=pose_timeout_s,
            relocalization_result=snapshot.get("relocalization_result"),
            lock_loaded_time=self._map_relocalization_lock_loaded_time(lock_payload),
            now_time=now,
            pose_tolerance_m=max(0.1, float(self.get_parameter("relocalization_pose_tolerance_m").value)),
            now_text=now_text,
        )

    def _clear_map_relocalization_required(self, clearance: Dict[str, Any]) -> None:
        with self._data_lock:
            current = self._settings.get("map_relocalization_required")
            if not current:
                return
            self._settings.pop("map_relocalization_required", None)
            self._save_json("settings.json", self._settings)
        self._append_event("固定地图重定位要求已自动清除", clearance)

    def _selected_map_status_payload(
        self,
        runtime_state: Optional[Dict[str, Any]] = None,
        *,
        selected_map_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        if selected_map_id is None:
            with self._data_lock:
                selected_map_id = self._settings.get("selected_map_id")
        if runtime_state is None:
            with self._lock:
                runtime_state = {"map": dict(self._state.get("map") or {})}
        selected_map_id = str(selected_map_id or "").strip()
        if not selected_map_id:
            selected_map_id = str(self._effective_map_id(runtime_state=runtime_state) or "").strip()
        live_map = dict((runtime_state or {}).get("map") or {})
        selected_map = self._map_file_summary(selected_map_id) if selected_map_id else {}
        return selected_map_status_payload(
            selected_map_id=selected_map_id,
            live_map=live_map,
            selected_map=selected_map,
            now_text=now_text,
        )

    def _fixed_map_id_matching_live_map(self, runtime_state: Optional[Dict[str, Any]] = None) -> Optional[str]:
        if runtime_state is None:
            with self._lock:
                runtime_state = {"map": dict(self._state.get("map") or {})}
        live_map = dict((runtime_state or {}).get("map") or {})
        if not live_map.get("available"):
            return None
        with self._data_lock:
            records = [dict(item) for item in self._all_maps_unlocked()]
        candidates: List[Dict[str, Any]] = []
        for record in records:
            map_id = str(record.get("id") or "").strip()
            if not map_id:
                continue
            selected_map = self._map_file_summary(map_id)
            candidates.append({**record, "summary": selected_map})
        return matching_fixed_map_id_for_live_map(live_map, candidates)

    def _working_map_id_unlocked(self) -> Optional[str]:
        selected = str(self._settings.get("selected_map_id") or "").strip()
        if selected and self._find_map_record_unlocked(selected):
            return selected
        working = str(self._settings.get("working_map_id") or "").strip()
        if working and self._find_map_record_unlocked(working):
            return working
        return None

    def _remember_working_map_id(self, map_id: Optional[str], *, reason: str) -> None:
        target = str(map_id or "").strip()
        if not target or target == "live_map":
            return
        changed = False
        with self._data_lock:
            if self._find_map_record_unlocked(target) is None:
                return
            if str(self._settings.get("working_map_id") or "").strip() != target:
                self._settings["working_map_id"] = target
                self._save_json("settings.json", self._settings)
                changed = True
        if changed:
            self._append_event("更新工作地图", {"map_id": target, "reason": reason})

    def _effective_map_id(
        self,
        requested_map_id: Optional[str] = None,
        runtime_state: Optional[Dict[str, Any]] = None,
    ) -> Optional[str]:
        requested = str(requested_map_id or "").strip()
        if requested and requested != "live_map":
            return requested
        with self._data_lock:
            selected = str(self._settings.get("selected_map_id") or "").strip()
            working = str(self._settings.get("working_map_id") or "").strip()
        live_map = dict((runtime_state or {}).get("map") or {})
        if runtime_state is None:
            with self._lock:
                live_map = dict(self._state.get("map") or {})
        with self._data_lock:
            records = [dict(item) for item in self._all_maps_unlocked()]
        candidates = []
        for record in records:
            map_id = str(record.get("id") or "").strip()
            if not map_id:
                continue
            candidates.append({**record, "summary": self._map_file_summary(map_id)})
        return effective_map_id_for_display(
            selected_map_id=selected,
            live_map=live_map,
            candidates=candidates,
            working_map_id=working,
        )

    def _camera_proxy_status_payload(self) -> Dict[str, Any]:
        enabled = self._as_bool(self.get_parameter("enable_camera_proxy").value)
        cameras: Dict[str, Any] = {}
        for camera_name, parameter_name in (("front", "front_camera_url"), ("rear", "rear_camera_url")):
            url = str(self.get_parameter(parameter_name).value)
            worker = self._camera_workers.get(camera_name)
            if worker is not None and worker.url == url and worker.backend == self._camera_proxy_backend():
                cameras[camera_name] = worker.status()
            else:
                cameras[camera_name] = {
                    "camera": camera_name,
                    "url": url,
                    "backend": self._camera_proxy_backend(),
                    "running": False,
                    "clients": 0,
                    "snapshot_lease_active": False,
                    "sequence": 0,
                    "has_frame": False,
                    "last_frame_age_s": None,
                    "last_error": None if enabled else "camera proxy disabled",
                    "last_error_age_s": None,
                }
        return {
            "enabled": enabled,
            "opencv_available": get_cv2() is not None if enabled else None,
            "opencv_error": _CV2_IMPORT_ERROR,
            "ffmpeg_available": shutil.which("ffmpeg") is not None if enabled else None,
            "backend": self._camera_proxy_backend(),
            "transport": str(self.get_parameter("camera_proxy_transport").value),
            "fps": float(self.get_parameter("camera_proxy_fps").value),
            "ffmpeg_mjpeg_qscale": int(self.get_parameter("camera_proxy_ffmpeg_mjpeg_qscale").value),
            "max_width": int(self.get_parameter("camera_proxy_max_width").value),
            "low_latency": self._as_bool(self.get_parameter("camera_proxy_low_latency").value),
            "snapshot_keepalive_s": float(self.get_parameter("camera_proxy_snapshot_keepalive_s").value),
            "cameras": cameras,
        }

    def _camera_proxy_backend(self) -> str:
        backend = str(self.get_parameter("camera_proxy_backend").value or "").strip().lower()
        if backend in ("ffmpeg", "ffmpeg_mjpeg") and shutil.which("ffmpeg") is not None:
            return "ffmpeg_mjpeg"
        return "opencv"

    def _map_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            current_map = self._state.get("map")
            if not current_map:
                return {"available": False}
            return dict(current_map)

    def _map_cache_key(self, map_id: Optional[str], yaml_path: FsPath) -> str:
        return "%s:%s" % (str(map_id or ""), str(yaml_path))

    def _map_file_summary(self, map_id: Optional[str]) -> Dict[str, Any]:
        with self._data_lock:
            if not map_id:
                map_id = self._settings.get("selected_map_id")
            record = self._find_map_record_unlocked(map_id)
        if record is None:
            return {"available": False, "message": "map not selected"}
        yaml_path = FsPath(self._resolve_path(str(record.get("yaml_path") or "")))
        fingerprint = map_file_fingerprint(yaml_path)
        if fingerprint is None:
            return {"available": False, "map_id": map_id, "message": f"map yaml not found: {yaml_path}"}
        cache_key = self._map_cache_key(map_id, yaml_path)
        with self._map_file_cache_lock:
            cached = self._map_file_summary_cache.get(cache_key)
            if cached and cached.get("fingerprint") == fingerprint:
                return dict(cached["payload"])
        try:
            payload = map_file_metadata_payload(record, yaml_path)
        except Exception as exc:
            payload = {"available": False, "map_id": map_id, "message": str(exc)}
        with self._map_file_cache_lock:
            self._map_file_summary_cache[cache_key] = {
                "fingerprint": fingerprint,
                "payload": dict(payload),
            }
        return payload

    def _slow_log_threshold_s(self) -> float:
        return max(0.0, float(self.get_parameter("api_slow_log_threshold_s").value))

    def _log_slow_operation(self, label: str, elapsed_s: float, detail: str = "") -> None:
        threshold_s = self._slow_log_threshold_s()
        if threshold_s <= 0.0 or elapsed_s < threshold_s:
            return
        suffix = f" {detail}" if detail else ""
        self.get_logger().warning("slow web operation: %s took %.3fs%s" % (label, elapsed_s, suffix))

    def _projects_payload(self) -> Dict[str, Any]:
        with self._data_lock:
            return {"ok": True, "projects": list(self._projects)}

    def _create_project(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        name = str(payload.get("name") or payload.get("project_name") or "").strip()
        building = str(payload.get("building") or "").strip()
        if not name:
            return self._error("项目名称不能为空")
        project = {
            "id": new_id("project"),
            "name": name,
            "building": building,
            "created_at": now_text(),
        }
        with self._data_lock:
            self._projects.append(project)
            self._save_json("projects.json", self._projects)
        return {"ok": True, "project": project}

    def _maps_payload(self) -> Dict[str, Any]:
        effective_map_id = self._effective_map_id()
        self._remember_working_map_id(effective_map_id, reason="maps_effective_map")
        with self._data_lock:
            return {
                "ok": True,
                "maps": self._all_maps_unlocked(),
                "selected_map_id": self._settings.get("selected_map_id"),
                "working_map_id": self._settings.get("working_map_id"),
                "effective_map_id": effective_map_id,
            }

    def _preflight_payload(self) -> Dict[str, Any]:
        with self._preflight_lock:
            running = self._preflight_running_payload_unlocked()
            if running:
                return {"ok": True, "running": True, "preflight": running}
            return {"ok": True, "preflight": self._preflight_with_age_unlocked()}

    def _preflight_with_age_unlocked(self) -> Optional[Dict[str, Any]]:
        return payload_with_age(self._last_preflight)

    def _preflight_running_payload_unlocked(self) -> Optional[Dict[str, Any]]:
        return payload_with_age(self._preflight_running)

    def _run_preflight(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not self._as_bool(payload.get("wait", False)):
            return self._start_preflight_background(payload)
        if not self._preflight_run_lock.acquire(blocking=False):
            with self._preflight_lock:
                last = self._preflight_running_payload_unlocked() or self._preflight_with_age_unlocked()
            return {
                "ok": True,
                "running": True,
                "preflight": last,
                "message": "自检正在执行，请稍后刷新结果",
            }
        try:
            return self._run_preflight_locked(payload)
        except Exception as exc:
            self.get_logger().exception("preflight failed unexpectedly")
            now = time.time()
            result = {
                "ok": False,
                "navigation_ready": False,
                "mode": str(payload.get("mode") or "move").strip() or "move",
                "timestamp": now,
                "time_text": now_text(),
                "age_sec": 0.0,
                "items": [
                    {
                        "key": "preflight_exception",
                        "label": "自检程序",
                        "status": "fail",
                        "message": str(exc) or exc.__class__.__name__,
                        "group": "base",
                    }
                ],
                "failures": 1,
                "navigation_warnings": 0,
                "warnings": 0,
                "summary": "基础自检异常中断，请重启全量系统或查看服务日志",
            }
            with self._preflight_lock:
                self._last_preflight = result
            return {"ok": True, "preflight": result, "message": result["summary"]}
        finally:
            self._preflight_run_lock.release()

    def _start_preflight_background(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not self._preflight_run_lock.acquire(blocking=False):
            with self._preflight_lock:
                running = self._preflight_running_payload_unlocked() or self._preflight_with_age_unlocked()
            return {
                "ok": True,
                "running": True,
                "preflight": running,
                "message": "自检正在后台执行，请稍后刷新结果",
            }
        now = time.time()
        request_id = new_id("preflight")
        running = {
            "ok": True,
            "running": True,
            "navigation_ready": False,
            "relocalization_ready": False,
            "mode": str(payload.get("mode") or "move").strip() or "move",
            "site": str(payload.get("site") or "workstation").strip() or "workstation",
            "timestamp": now,
            "time_text": now_text(),
            "age_sec": 0.0,
            "items": [],
            "failures": 0,
            "navigation_warnings": 0,
            "warnings": 0,
            "summary": "基础自检后台执行中，请稍候",
            "request_id": request_id,
        }
        with self._preflight_lock:
            self._preflight_running = running
        thread = threading.Thread(
            target=self._run_preflight_background_worker,
            args=(dict(payload), request_id),
            daemon=True,
        )
        thread.start()
        return {
            "ok": True,
            "running": True,
            "preflight": dict(running),
            "message": running["summary"],
        }

    def _run_preflight_background_worker(self, payload: Dict[str, Any], request_id: str) -> None:
        try:
            response = self._run_preflight_locked(payload)
            result = dict(response.get("preflight") or response)
            result["running"] = False
            result["request_id"] = request_id
        except Exception as exc:
            self.get_logger().exception("background preflight failed unexpectedly")
            now = time.time()
            result = {
                "ok": False,
                "running": False,
                "navigation_ready": False,
                "relocalization_ready": False,
                "mode": str(payload.get("mode") or "move").strip() or "move",
                "timestamp": now,
                "time_text": now_text(),
                "age_sec": 0.0,
                "items": [
                    {
                        "key": "preflight_exception",
                        "label": "自检程序",
                        "status": "fail",
                        "message": str(exc) or exc.__class__.__name__,
                        "group": "base",
                    }
                ],
                "failures": 1,
                "navigation_warnings": 0,
                "warnings": 0,
                "summary": "基础自检异常中断，请重启全量系统或查看服务日志",
                "request_id": request_id,
            }
        finally:
            with self._preflight_lock:
                self._last_preflight = result
                self._preflight_running = None
            self._preflight_run_lock.release()

    def _run_preflight_locked(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        self._wait_for_preflight_baseline()
        now = time.time()
        timeout_s = max(2.0, min(8.0, float(self.get_parameter("preflight_topic_timeout_s").value)))
        items: List[Dict[str, Any]] = []

        node_names = set(self.get_node_names())
        required_nodes = [
            "m20pro_tcp_bridge",
            "m20pro_web_dashboard",
            "map_server",
            "controller_server",
            "planner_server",
            "bt_navigator",
            "m20pro_nav2_startup_gate",
            "m20pro_floor_manager",
        ]
        items.append(preflight_node_item(list(node_names), required_nodes))

        topic_names = {name for name, _types in self.get_topic_names_and_types()}
        base_topics = [
            self._topic("navigation_status_topic"),
            self._topic("map_topic"),
        ]
        navigation_topics = [
            self._topic("scan_topic"),
            self._topic("odom_topic"),
            self._topic("pose_topic"),
            self._topic("localization_ok_topic"),
            self._topic("local_costmap_topic"),
            self._topic("global_costmap_topic"),
        ]
        items.append(preflight_base_topics_item(list(topic_names), base_topics))
        items.append(preflight_navigation_topics_item(list(topic_names), navigation_topics))

        with self._lock:
            current_state = {
                key: self._state.get(key)
                for key in (
                    "scan",
                    "odom",
                    "pose",
                    "localization_ok",
                    "navigation_status",
                    "map",
                    "local_costmap",
                    "global_costmap",
                )
            }
        context = preflight_context(
            payload,
            localization_ok=current_state.get("localization_ok"),
            navigation_status=current_state.get("navigation_status"),
        )
        mode = str(context["mode"])
        site = str(context["site"])

        def fresh(key: str) -> Tuple[bool, Optional[float], Any]:
            value = current_state.get(key)
            if not isinstance(value, dict):
                return False, None, value
            last_update = value.get("last_update")
            if last_update is None:
                return False, None, value
            age = max(0.0, now - float(last_update))
            return age <= timeout_s, age, value

        scan_ok, scan_age, scan = fresh("scan")
        finite_ranges = int(scan.get("finite_ranges", 0)) if isinstance(scan, dict) else 0
        perception = preflight_perception_items(
            scan if isinstance(scan, dict) else {},
            scan_ok=scan_ok,
            scan_age_text=fmt_age_text(scan_age) if scan_age is not None else "",
            finite_ranges=finite_ranges,
        )
        items.extend(perception["items"])
        perception_ok = bool(perception["perception_ok"])

        odom_ok, odom_age, odom = fresh("odom")
        odom_finite = bool(isinstance(odom, dict) and odom.get("finite"))
        items.append(
            preflight_odom_item(
                odom if isinstance(odom, dict) else {},
                odom_ok=odom_ok,
                odom_finite=odom_finite,
                age_text=fmt_age_text(odom_age) if odom_age is not None else "",
            )
        )

        pose = current_state.get("pose")
        pose_has_stamp = isinstance(pose, dict) and is_plausible_pose_dict(pose)
        pose_age = None
        if isinstance(pose, dict) and pose.get("stamp"):
            pose_age = max(0.0, now - float(pose["stamp"]))
        items.append(
            preflight_map_pose_item(
                pose if isinstance(pose, dict) else {},
                pose_ok=pose_has_stamp,
                age_text=fmt_age_text(pose_age),
            )
        )

        loc_ok = bool(context["localized"])
        nav_status_text = str(context["navigation_status_text"])
        items.append(preflight_localization_item(loc_ok))
        items.append(preflight_navigation_status_item(nav_status_text))

        map_payload = current_state.get("map")
        map_ok = isinstance(map_payload, dict)
        items.append(preflight_map_item(map_payload if isinstance(map_payload, dict) else {}))

        local_ok, local_age, local_costmap = fresh("local_costmap")
        global_ok, global_age, global_costmap = fresh("global_costmap")
        items.extend(
            preflight_costmap_items(
                local_costmap if isinstance(local_costmap, dict) else {},
                global_costmap if isinstance(global_costmap, dict) else {},
                local_ok=local_ok,
                global_ok=global_ok,
                local_age_text=fmt_age_text(local_age) if local_age is not None else "",
                global_age_text=fmt_age_text(global_age) if global_age is not None else "",
                deferred=bool(context["defer_nav2_startup_checks"]),
            )
        )

        if context["defer_nav2_startup_checks"]:
            items.append(preflight_lifecycle_deferred_item())
        else:
            lifecycle_results = self._check_lifecycle_nodes(
                ["/map_server", "/controller_server", "/planner_server", "/bt_navigator"]
            )
            for node_name, lifecycle in lifecycle_results.items():
                items.append(preflight_lifecycle_item(node_name, lifecycle))

        motion = self._detect_motion_mode()
        items.append(preflight_motion_mode_item(requested_mode=mode, motion=motion))

        result = preflight_result_payload(
            items,
            mode=mode,
            site=site,
            workstation_mode=bool(context["workstation_mode"]),
            map_ok=map_ok,
            perception_ok=perception_ok,
            timestamp=now,
            now_text=now_text,
        )
        with self._preflight_lock:
            self._last_preflight = result
        self._append_event("作业前自检", {"ok": result["ok"], "failures": result["failures"]})
        return {"ok": True, "preflight": result, "message": result["summary"]}

    def _wait_for_preflight_baseline(self) -> None:
        deadline = time.time() + max(
            0.0,
            min(10.0, float(self.get_parameter("preflight_settle_wait_s").value)),
        )
        while time.time() < deadline:
            now = time.time()
            with self._lock:
                scan = dict(self._state.get("scan") or {})
                navigation_status = self._state.get("navigation_status")
                map_seen = isinstance(self._state.get("map"), dict)
            scan_ok = (
                int(scan.get("finite_ranges", 0) or 0) > 0
                and now - float(scan.get("last_update", 0.0) or 0.0) <= 2.0
            )
            if map_seen and scan_ok and navigation_status:
                return
            time.sleep(0.1)

    def _check_lifecycle_nodes(self, node_names: List[str]) -> Dict[str, Dict[str, Any]]:
        results: Dict[str, Dict[str, Any]] = {}
        if GetState is None:
            return {
                node_name: {"active": False, "message": "lifecycle_msgs 不可用"}
                for node_name in node_names
            }
        for node_name in node_names:
            service_name = f"{node_name}/get_state"
            result = {"active": False, "message": "未查询"}
            try:
                client = self.create_client(GetState, service_name)
                if not client.wait_for_service(timeout_sec=0.25):
                    result["message"] = f"{service_name} 不可用"
                    results[node_name] = result
                    self.destroy_client(client)
                    continue
                future = client.call_async(GetState.Request())
                deadline = time.monotonic() + 0.75
                while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                    time.sleep(0.02)
                if future.done() and future.result() is not None:
                    state = future.result().current_state
                    label = str(state.label)
                    result["active"] = label == "active"
                    result["message"] = label or f"id={state.id}"
                else:
                    result["message"] = "查询超时"
            except Exception as exc:
                result["message"] = str(exc)
            finally:
                try:
                    self.destroy_client(client)
                except Exception:
                    pass
            results[node_name] = result
        return results

    def _detect_motion_mode(self) -> Dict[str, str]:
        try:
            output = subprocess.run(
                ["ps", "-eo", "args"],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                timeout=1.0,
                check=False,
            ).stdout
        except Exception as exc:
            return {"mode": "unknown", "message": f"无法读取进程列表：{exc}"}
        launch_lines = [
            line
            for line in output.splitlines()
            if "m20pro_bringup" in line and ("m20pro.launch.py" in line or "m20pro_real_full.sh" in line)
        ]
        joined = "\n".join(launch_lines)
        if "enable_axis_command:=true" in joined or "m20pro_real_full.sh move" in joined:
            return {"mode": "move", "message": "已确认 move：运动控制已放开"}
        if "enable_axis_command:=false" in joined or "m20pro_real_full.sh shadow" in joined:
            return {"mode": "shadow", "message": "当前是 shadow：不会下发运动控制"}
        if launch_lines:
            return {"mode": "unknown", "message": "找到 real launch，但未确认 enable_axis_command"}
        return {"mode": "unknown", "message": "未找到全量 real 启动进程"}

    def _select_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        map_id = str(payload.get("map_id") or "").strip() or None
        record: Optional[Dict[str, Any]] = None
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") == "running":
                return self._error("任务执行中不能切换地图，请先停止当前任务")
            previous_map_id = self._settings.get("selected_map_id")
            if map_id:
                record = self._find_map_record_unlocked(map_id)
                if record is None:
                    return self._error("地图不存在")
        if map_id:
            nav2_load = self._load_selected_map_into_nav2(record)
            if not nav2_load.get("ok"):
                return self._error(
                    str(nav2_load["message"]),
                    {"code": "nav2_map_load_failed", "nav2_load_map": nav2_load},
                )
        else:
            nav2_load = {
                "ok": True,
                "skipped": True,
                "message": "已切换到实时 /map 观察；未调用 Nav2 load_map",
            }
        factory_apply = self._apply_selected_map_to_factory(record) if map_id and record else {
            "ok": True,
            "skipped": True,
            "message": "未选择固定地图；未调用 106 drmap apply",
        }
        if not factory_apply.get("ok"):
            return self._error(
                str(factory_apply.get("message") or "106 原厂地图切换失败"),
                {
                    "code": "factory_map_apply_failed",
                    "nav2_load_map": nav2_load,
                    "factory_apply_map": factory_apply,
                },
            )
        with self._data_lock:
            result = apply_selected_map_choice_state(
                self._settings,
                map_id=map_id,
                previous_map_id=previous_map_id,
                record=record,
                nav2_load=nav2_load,
                now_text=now_text,
            )
            self._settings = result["settings"]
            self._save_json("settings.json", self._settings)
            map_relocalization_required = result.get("relocalization_required")
            clear_pose = bool(result.get("clear_pose"))
        if clear_pose:
            with self._lock:
                self._state["localization_ok"] = False
                self._state["pose"] = None
                self._state["pose_history"] = []
                self._state["path"] = {"version": int(self._state.get("path", {}).get("version", 0) or 0) + 1, "points": []}
        effective_map_id = self._effective_map_id()
        self._remember_working_map_id(effective_map_id, reason="select_map_effective_map")
        with self._data_lock:
            working_map_id = self._settings.get("working_map_id")
        self._append_event(
            "切换地图显示",
            {
                "selected_map_id": map_id,
                "working_map_id": working_map_id,
                "effective_map_id": effective_map_id,
                "nav2_load_map": nav2_load,
                "factory_apply_map": factory_apply,
            },
        )
        return {
            "ok": True,
            "selected_map_id": map_id,
            "working_map_id": working_map_id,
            "effective_map_id": effective_map_id,
            "nav2_load_map": nav2_load,
            "factory_apply_map": factory_apply,
            "map_relocalization_required": map_relocalization_required,
        }

    def _load_selected_map_into_nav2(self, record: Dict[str, Any]) -> Dict[str, Any]:
        if not bool(self.get_parameter("map_select_load_nav2_map").value):
            return {"ok": True, "skipped": True, "message": "map_select_load_nav2_map=false"}
        if LoadMap is None or self.load_map_client is None:
            return {"ok": False, "code": "load_map_unavailable", "message": "nav2_msgs/LoadMap 不可用"}
        yaml_path = FsPath(self._resolve_path(str(record.get("yaml_path") or "")))
        if not yaml_path.exists():
            return {
                "ok": False,
                "code": "map_yaml_missing",
                "message": f"地图 yaml 不存在: {yaml_path}",
                "yaml_path": str(yaml_path),
            }
        image_repair = ensure_map_yaml_uses_local_image(yaml_path)
        if not image_repair.get("ok"):
            return {
                "ok": False,
                "code": str(image_repair["code"]),
                "message": str(image_repair["message"]),
                "yaml_path": str(yaml_path),
                "image_repair": image_repair,
            }
        selected_map = self._map_file_snapshot(str(record.get("id") or ""))
        with self._lock:
            live_map = dict(self._state.get("map") or {})
        if map_metadata_mismatch_error(live_map, selected_map) is None:
            return {
                "ok": True,
                "loaded": False,
                "already_loaded": True,
                "message": "Nav2 当前 /map 已经与前端选中地图一致",
                "yaml_path": str(yaml_path),
                "image_repair": image_repair,
            }
        timeout_s = max(0.5, float(self.get_parameter("map_select_load_timeout_s").value))
        if not self.load_map_client.wait_for_service(timeout_sec=timeout_s):
            return {
                "ok": False,
                "code": "load_map_service_unavailable",
                "message": f"{self.load_map_client.srv_name} 不可用",
                "yaml_path": str(yaml_path),
                "image_repair": image_repair,
            }
        request = LoadMap.Request()
        request.map_url = str(yaml_path)
        future = self.load_map_client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return {
                "ok": False,
                "code": "load_map_timeout",
                "message": f"Nav2 load_map 超时 {timeout_s:.1f}s",
                "yaml_path": str(yaml_path),
                "image_repair": image_repair,
            }
        try:
            response = future.result()
        except Exception as exc:
            return {
                "ok": False,
                "code": "load_map_call_failed",
                "message": str(exc),
                "yaml_path": str(yaml_path),
                "image_repair": image_repair,
            }
        result = int(getattr(response, "result", 255))
        if result != int(getattr(LoadMap.Response, "RESULT_SUCCESS", 0)):
            return {
                "ok": False,
                "code": "load_map_rejected",
                "message": f"Nav2 load_map 返回错误码 {result}",
                "result": result,
                "yaml_path": str(yaml_path),
                "image_repair": image_repair,
            }
        self._clear_task_costmaps("select_map_load_nav2")
        match = self._wait_for_selected_map_match(selected_map)
        return {
            "ok": True,
            "loaded": True,
            "message": str(match["message"]),
            "yaml_path": str(yaml_path),
            "result": result,
            "map_matched": bool(match.get("ready")),
            "selected_map_status": match,
            "image_repair": image_repair,
        }

    def _wait_for_selected_map_match(self, selected_map: Dict[str, Any]) -> Dict[str, Any]:
        timeout_s = max(0.1, float(self.get_parameter("map_select_wait_match_timeout_s").value))
        poll_s = max(0.02, float(self.get_parameter("map_select_wait_match_poll_s").value))
        deadline = time.time() + timeout_s
        last_status: Optional[Dict[str, Any]] = None
        selected_map_id = str(selected_map.get("map_id") or "").strip()
        while time.time() < deadline:
            with self._lock:
                live_map = dict(self._state.get("map") or {})
            last_status = selected_map_status_payload(
                selected_map_id=selected_map_id,
                live_map=live_map,
                selected_map=selected_map,
                now_text=now_text,
            )
            if last_status.get("ready"):
                return last_status
            time.sleep(poll_s)
        if last_status is not None:
            return last_status
        return selected_map_wait_timeout_payload(
            selected_map_id=selected_map.get("map_id"),
            now_text=now_text,
        )

    def _create_mapping_session(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        prepared = prepare_mapping_session_create(
            payload,
            projects=self._projects,
            id_factory=new_id,
            now_text=now_text,
            default_project_name="M20Pro 工地巡检",
            default_map_name="{active_floor}_" + stamp,
        )
        session = prepared["session"]
        with self._data_lock:
            if prepared.get("created_project"):
                self._projects.append(prepared["created_project"])
            self._sessions.append(session)
            self._save_json("projects.json", self._projects)
            self._save_json("mapping_sessions.json", self._sessions)
        self._append_event(
            "建立建图任务",
            {
                "session_id": session["id"],
                "mode": session.get("mode"),
                "floors": session.get("floors"),
                "map_name": session.get("map_name"),
            },
        )
        return {"ok": True, "session": session}

    def _mapping_command(self, param_name: str, session_id: Optional[str]) -> Dict[str, Any]:
        session = self._find_session(session_id)
        if session is None:
            return self._error("建图任务不存在，请先建立建图任务")
        context = self._command_context(session)
        result = self._run_configured_command(param_name, context)
        updated = apply_mapping_command_result(
            session,
            param_name=param_name,
            result=result,
            updated_at=now_text(),
        )
        session.clear()
        session.update(updated)
        with self._data_lock:
            self._save_json("mapping_sessions.json", self._sessions)
        self._append_event(
            "建图命令执行",
            {"session_id": session["id"], "command": param_name, "status": session["status"]},
        )
        result["session"] = session
        return result

    def _check_mapping_environment(self) -> Dict[str, Any]:
        factory_host = str(self.get_parameter("factory_host").value).strip()
        factory_user = str(self.get_parameter("factory_user").value).strip()
        active_map = str(self.get_parameter("factory_active_map").value).strip()
        timeout = min(20.0, float(self.get_parameter("mapping_command_timeout_s").value))
        if factory_host in ("", "localhost", "127.0.0.1"):
            prefix = ""
        else:
            prefix = f"ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} "

        remote_probe = (
            "set -e; "
            "echo host=$(hostname); "
            "echo user=$(whoami); "
            "echo drmap_probe=drmap_mapping_help; "
            f"echo active_map={active_map}; "
            f"active_path=$(readlink -f {active_map} || true); "
            f"echo active_resolved=${{active_path:-{active_map}}}; "
            f"test -d {active_map}; "
            "sudo -n drmap mapping -h >/dev/null; "
            "echo drmap_mapping_help=ok"
        )
        command = f"{prefix}{json.dumps(remote_probe)}" if prefix else remote_probe
        try:
            result = subprocess.run(
                command,
                shell=True,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=timeout,
                check=False,
            )
        except Exception as exc:
            return self._error(
                "106 建图环境检查失败",
                {
                    "factory_host": factory_host,
                    "factory_user": factory_user,
                    "factory_active_map": active_map,
                    "error": str(exc),
                },
            )

        ok = result.returncode == 0
        payload = {
            "ok": ok,
            "factory_host": factory_host,
            "factory_user": factory_user,
            "factory_active_map": active_map,
            "command": command,
            "returncode": result.returncode,
            "output": result.stdout or "",
        }
        if ok:
            payload["message"] = (
                "106 建图环境可用：SSH、drmap、active map、启动建图权限均通过。"
                "完成/保存建图会调用 drmap stop_mapping，但该命令没有安全的 dry-run 检查。"
            )
        else:
            payload["message"] = (
                "106 建图环境未通过。常见原因：104 到 106 未配置 SSH 免密，"
                "或 106 上 sudo drmap mapping 仍需要交互输入密码。"
            )
        return payload

    def _run_factory_shell(
        self,
        shell_command: str,
        *,
        factory_host: str,
        factory_user: str,
        timeout: float,
    ) -> Dict[str, Any]:
        if factory_host in ("", "localhost", "127.0.0.1"):
            command: Any = shell_command
            command_text = shell_command
            use_shell = True
        else:
            command = [
                "ssh",
                "-o",
                "BatchMode=yes",
                "-o",
                "ConnectTimeout=8",
                f"{factory_user}@{factory_host}",
                shell_command,
            ]
            command_text = " ".join(shlex.quote(str(item)) for item in command)
            use_shell = False
        try:
            result = subprocess.run(
                command,
                shell=use_shell,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=timeout,
                check=False,
            )
        except Exception as exc:
            return {"ok": False, "command": command_text, "output": "", "message": str(exc)}
        return {
            "ok": result.returncode == 0,
            "command": command_text,
            "returncode": result.returncode,
            "output": result.stdout or "",
        }

    def _resolve_factory_map_path(
        self,
        source: str,
        *,
        factory_host: str,
        factory_user: str,
        timeout: float,
    ) -> Dict[str, Any]:
        source = str(source or "").strip()
        if not source:
            return self._error("106 地图路径为空")
        if factory_host in ("", "localhost", "127.0.0.1"):
            path = FsPath(source).expanduser()
            resolved = path.resolve(strict=False)
            if not resolved.is_dir():
                return self._error("地图目录不存在", {"source": source, "resolved_source": str(resolved)})
            return {
                "ok": True,
                "source_path": str(resolved),
                "source_reason": "explicit_or_active",
                "command": f"readlink -f {source}",
                "output": str(resolved),
            }
        quoted_source = shlex.quote(source)
        probe = (
            f"target={quoted_source}; "
            'resolved=$(readlink -f "$target" 2>/dev/null || true); '
            'if [ -n "$resolved" ]; then target="$resolved"; fi; '
            'test -d "$target"; '
            'printf "%s\\n" "$target"'
        )
        result = self._run_factory_shell(
            probe,
            factory_host=factory_host,
            factory_user=factory_user,
            timeout=timeout,
        )
        if not result.get("ok"):
            return self._error(
                "106 地图目录不存在或无法访问",
                {"source": source, "command": result.get("command"), "output": result.get("output")},
            )
        resolved = (str(result.get("output") or "").splitlines() or [source])[-1].strip() or source
        return {
            "ok": True,
            "source_path": resolved,
            "source_reason": "explicit_or_active",
            "command": result.get("command"),
            "output": result.get("output"),
        }

    def _find_latest_factory_map_package(
        self,
        map_name: str,
        *,
        factory_host: str,
        factory_user: str,
        timeout: float,
    ) -> Dict[str, Any]:
        base = "/var/opt/robot/data/maps"
        name = sanitize_name(str(map_name or ""), "")
        if not name:
            return self._error("建图名称为空，无法按名称查找 106 地图包")
        if factory_host in ("", "localhost", "127.0.0.1"):
            base_path = FsPath(base)
            matches = [
                item
                for item in base_path.iterdir()
                if item.is_dir() and item.name.startswith(f"{name}-")
            ] if base_path.is_dir() else []
            if not matches:
                return self._error("没有找到同名 106 地图包", {"factory_map_name": name, "base": base})
            latest = max(matches, key=lambda item: item.stat().st_mtime)
            return {
                "ok": True,
                "source_path": str(latest),
                "source_reason": "latest_named_map_package",
                "factory_map_name": name,
                "command": f"find {base} -name {name}-*",
                "output": str(latest),
            }
        probe = (
            f"base={shlex.quote(base)}; "
            f"name={shlex.quote(name)}; "
            'find "$base" -maxdepth 1 -mindepth 1 -type d -name "$name-*" '
            '-printf "%T@ %p\\n" | sort -nr | head -n 1 | cut -d" " -f2-'
        )
        result = self._run_factory_shell(
            probe,
            factory_host=factory_host,
            factory_user=factory_user,
            timeout=timeout,
        )
        source_path = (str(result.get("output") or "").splitlines() or [""])[-1].strip()
        if not result.get("ok") or not source_path:
            return self._error(
                "没有找到同名 106 地图包",
                {
                    "factory_map_name": name,
                    "base": base,
                    "command": result.get("command"),
                    "output": result.get("output"),
                },
            )
        return {
            "ok": True,
            "source_path": source_path,
            "source_reason": "latest_named_map_package",
            "factory_map_name": name,
            "command": result.get("command"),
            "output": result.get("output"),
        }

    def _resolve_factory_import_source(
        self,
        payload: Dict[str, Any],
        session: Optional[Dict[str, Any]],
        *,
        factory_host: str,
        factory_user: str,
        timeout: float,
    ) -> Dict[str, Any]:
        explicit_source = str(payload.get("source") or "").strip()
        if explicit_source:
            return self._resolve_factory_map_path(
                explicit_source,
                factory_host=factory_host,
                factory_user=factory_user,
                timeout=timeout,
            )
        factory_map_name = str(
            payload.get("factory_map_name")
            or (session or {}).get("map_name")
            or payload.get("map_name")
            or ""
        ).strip()
        if factory_map_name:
            return self._find_latest_factory_map_package(
                factory_map_name,
                factory_host=factory_host,
                factory_user=factory_user,
                timeout=timeout,
            )
        return self._resolve_factory_map_path(
            str(self.get_parameter("factory_active_map").value),
            factory_host=factory_host,
            factory_user=factory_user,
            timeout=timeout,
        )

    def _apply_selected_map_to_factory(self, record: Dict[str, Any]) -> Dict[str, Any]:
        source_path = str(
            record.get("factory_apply_path")
            or record.get("source_path")
            or ""
        ).strip()
        base = "/var/opt/robot/data/maps/"
        if not source_path:
            return {"ok": True, "skipped": True, "message": "地图记录没有 106 原厂路径；未调用 drmap apply"}
        if source_path == "/var/opt/robot/data/maps/active":
            return {"ok": True, "skipped": True, "message": "地图记录只指向 active 软链接；未盲目 apply"}
        if not source_path.startswith(base):
            return {"ok": True, "skipped": True, "message": "地图不是 106 原厂地图包；未调用 drmap apply", "source_path": source_path}
        factory_host = str(self.get_parameter("factory_host").value).strip()
        factory_user = str(self.get_parameter("factory_user").value).strip()
        timeout = min(120.0, max(10.0, float(self.get_parameter("map_import_timeout_s").value)))
        command = f"sudo -n drmap apply {shlex.quote(source_path)}"
        result = self._run_factory_shell(
            command,
            factory_host=factory_host,
            factory_user=factory_user,
            timeout=timeout,
        )
        if not result.get("ok"):
            return {
                "ok": False,
                "message": "106 drmap apply 失败，地图未完整切换",
                "source_path": source_path,
                "command": result.get("command"),
                "output": result.get("output"),
                "returncode": result.get("returncode"),
            }
        return {
            "ok": True,
            "source_path": source_path,
            "command": result.get("command"),
            "output": result.get("output"),
            "message": "106 原厂 active 地图已切换",
        }

    def _cleanup_failed_map_import(self, dest: FsPath) -> Dict[str, Any]:
        try:
            if dest.exists():
                shutil.rmtree(dest)
            return {"ok": True, "removed": str(dest)}
        except Exception as exc:
            return {"ok": False, "path": str(dest), "error": str(exc)}

    def _import_active_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        session_id = payload.get("session_id")
        session = self._find_session(str(session_id).strip() if session_id else None)
        floor = str(payload.get("floor") or (session or {}).get("active_floor") or "").strip()
        if not floor:
            return self._error("请指定地图楼层")

        factory_host = str(payload.get("factory_host") or self.get_parameter("factory_host").value).strip()
        factory_user = str(payload.get("factory_user") or self.get_parameter("factory_user").value).strip()
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        default_name = str((session or {}).get("map_name") or "").strip() or f"{floor}_{stamp}"
        map_name = sanitize_name(str(payload.get("map_name") or ""), default_name)
        dest = self.map_archive_dir / map_name
        if dest.exists():
            dest = self.map_archive_dir / f"{map_name}_{random_suffix(6)}"
        timeout = float(self.get_parameter("map_import_timeout_s").value)
        source_result = self._resolve_factory_import_source(
            payload,
            session,
            factory_host=factory_host,
            factory_user=factory_user,
            timeout=min(30.0, timeout),
        )
        if not source_result.get("ok"):
            return source_result
        source = str(source_result.get("source_path") or "").strip()

        try:
            if factory_host in ("", "localhost", "127.0.0.1"):
                shutil.copytree(source, dest)
                command_text = f"copy {source} -> {dest}"
                command_output = ""
            else:
                remote = f"{factory_user}@{factory_host}:{source.rstrip('/')}"
                result = subprocess.run(
                    ["scp", "-r", remote, str(dest)],
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    timeout=timeout,
                    check=False,
                )
                command_text = " ".join(["scp", "-r", remote, str(dest)])
                command_output = result.stdout or ""
                if result.returncode != 0:
                    return self._error(
                        "从 106 拉取地图失败，请确认 104 到 106 的 SSH/scp 可用",
                        {"command": command_text, "output": command_output},
                    )
        except Exception as exc:
            return self._error("从 106 拉取地图失败", {"error": str(exc)})

        yaml_path = find_map_yaml(dest)
        if yaml_path is None:
            cleanup = self._cleanup_failed_map_import(dest)
            return self._error(
                "地图包已拉取，但没有生成可供前端/Nav2使用的栅格 yaml；请确认 106 建图已成功保存完成",
                {
                    "directory": str(dest),
                    "source_path": source,
                    "source_resolution": source_result,
                    "command": command_text,
                    "output": command_output,
                    "cleanup": cleanup,
                },
            )
        image_repair = ensure_map_yaml_uses_local_image(yaml_path)
        if not image_repair.get("ok"):
            cleanup = self._cleanup_failed_map_import(dest)
            return self._error(
                "地图已拉取，但栅格图文件不可用",
                {
                    "directory": str(dest),
                    "source_path": source,
                    "yaml_path": str(yaml_path),
                    "image_repair": image_repair,
                    "cleanup": cleanup,
                },
            )

        session_payload = session or {}
        map_record = build_imported_map_record(
            map_id=new_id("map"),
            map_name=map_name,
            floor=floor,
            mode=session_payload.get("mode"),
            project_id=session_payload.get("project_id"),
            project_name=session_payload.get("project_name"),
            building=session_payload.get("building"),
            directory=dest,
            yaml_path=yaml_path,
            source_path=source,
            created_at=now_text(),
        )
        map_record["factory_apply_path"] = source
        map_record["factory_source_reason"] = source_result.get("source_reason")
        if source_result.get("factory_map_name"):
            map_record["factory_map_name"] = source_result.get("factory_map_name")
        map_record["derived"] = self._generate_map_derived(map_record, dest, yaml_path, floor)
        with self._data_lock:
            self._maps.append(map_record)
            if session:
                session["status"] = "imported"
                session["updated_at"] = now_text()
            self._save_json("maps.json", self._maps)
            self._save_json("mapping_sessions.json", self._sessions)
            selected_map_id = self._settings.get("selected_map_id")
        self._append_event(
            "从 106 拉取地图完成",
            {"map_id": map_record["id"], "floor": floor, "source_path": source},
        )
        return {
            "ok": True,
            "map": map_record,
            "imported_map_id": map_record["id"],
            "selected_map_id": selected_map_id,
            "source_path": source,
            "source_resolution": source_result,
            "command": command_text,
            "output": command_output,
            "image_repair": image_repair,
        }

    def _generate_map_derived(
        self,
        map_record: Dict[str, Any],
        map_dir: FsPath,
        yaml_path: FsPath,
        floor: str,
    ) -> Dict[str, Any]:
        if not self._as_bool(self.get_parameter("enable_stair_zone_postprocess").value):
            return {
                "status": "disabled",
                "message": "楼梯语义区生成未启用",
            }
        try:
            floor_config = self._floor_config_path()
            result = process_imported_map(
                FsPath(map_dir),
                FsPath(yaml_path),
                floor,
                str(map_record.get("id") or ""),
                floor_config_path=floor_config,
            )
            self._append_event(
                "地图楼梯语义区生成完成",
                {
                    "map_id": map_record.get("id"),
                    "floor": floor,
                    "status": result.get("status"),
                    "message": result.get("message"),
                },
            )
            return result
        except Exception as exc:
            self.get_logger().warning("stair-zone postprocess failed: %s" % exc)
            return {
                "status": "failed",
                "message": str(exc) or exc.__class__.__name__,
            }

    def _floor_config_path(self) -> Optional[FsPath]:
        try:
            return FsPath(get_package_share_directory("m20pro_bringup")) / "config" / "inspection_waypoints.yaml"
        except PackageNotFoundError:
            return None

    def _ensure_builtin_map_derived(self, record: Dict[str, Any], derived: Dict[str, Any]) -> str:
        yaml_path = FsPath(self._resolve_path(str(record.get("yaml_path") or "")))
        pcd_value = str(record.get("pcd_path") or "")
        pcd_path = FsPath(self._resolve_path(pcd_value)) if pcd_value else None
        if not yaml_path.exists():
            return ""
        cache_dir = self.data_dir / "builtin_derived" / sanitize_name(str(record.get("id") or "builtin"), "builtin")
        cache_dir.mkdir(parents=True, exist_ok=True)
        result = process_imported_map(
            cache_dir,
            yaml_path,
            str(record.get("floor") or ""),
            str(record.get("id") or ""),
            floor_config_path=self._floor_config_path(),
            pcd_path_override=pcd_path,
        )
        result["base_dir"] = str(cache_dir)
        record["derived"] = result
        with self._data_lock:
            for builtin in self._builtin_maps:
                if builtin.get("id") == record.get("id"):
                    builtin["derived"] = result
                    break
        return str(result.get("stair_zones") or "")

    def _stair_zones_payload(self, map_id: Optional[str]) -> Dict[str, Any]:
        with self._data_lock:
            if not map_id:
                map_id = self._settings.get("selected_map_id")
            record = self._find_map_record_unlocked(map_id)
        if record is None:
            return stair_zones_unavailable_payload(None, "未选择固定地图")
        derived = map_derived_payload(record)
        zones_rel = stair_zones_relative_path(record)
        if not zones_rel:
            if should_generate_builtin_stair_zones(
                record,
                enable_stair_zone_postprocess=self._as_bool(
                    self.get_parameter("enable_stair_zone_postprocess").value
                ),
            ):
                self._ensure_builtin_map_derived(record, derived)
                derived = map_derived_payload(record)
                zones_rel = stair_zones_relative_path(record)
        if not zones_rel:
            return stair_zones_unavailable_payload(record, "当前地图没有楼梯语义区")
        zones_path = resolve_map_asset_path(record, zones_rel, path_resolver=self._resolve_path)
        if zones_path is None or not zones_path.exists():
            return stair_zones_unavailable_payload(record, "楼梯语义区文件不存在")
        try:
            payload = read_json_object(zones_path)
        except Exception as exc:
            return stair_zones_unavailable_payload(record, str(exc))
        return stair_zones_available_payload(record, derived, payload)

    def _publish_selected_stair_zones(self) -> None:
        try:
            map_id = self._effective_map_id()
            payload = self._stair_zones_payload(map_id)
            zones = payload.get("zones") or []
            if not payload.get("available") and not map_id:
                return
            msg = String()
            msg.data = json.dumps(
                {
                    "map_id": payload.get("map_id") or ((payload.get("map") or {}).get("id")),
                    "floor": payload.get("floor") or ((payload.get("map") or {}).get("floor")),
                    "zones": zones,
                    "available": bool(payload.get("available")),
                    "updated_at": now_text(),
                },
                ensure_ascii=False,
                separators=(",", ":"),
            )
            self.stair_zones_pub.publish(msg)
        except Exception as exc:
            self.get_logger().debug("failed to publish stair zones: %s" % exc)

    def _annotations_payload(self, query: Dict[str, List[str]]) -> Dict[str, Any]:
        requested_map_id = (query.get("map_id") or [None])[0]
        requested_text = str(requested_map_id or "").strip() if requested_map_id is not None else ""
        if requested_map_id is None:
            map_id = None
        else:
            map_id = self._effective_map_id(requested_text) or requested_text
        if (not requested_text or requested_text == "live_map") and map_id:
            self._remember_working_map_id(map_id, reason="annotations_effective_map")
        with self._data_lock:
            annotations = list(self._annotations)
        payload = annotation_list_filter_payload(annotations, map_id=map_id)
        payload["requested_map_id"] = requested_map_id
        payload["effective_map_id"] = map_id
        return payload

    def _create_annotation(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            runtime_state = {"map": dict(self._state.get("map") or {})}
        context = annotation_create_static_context(
            payload,
            default_label_index=len(self._annotations) + 1,
        )
        if not context.get("ok"):
            return self._error(str(context["message"]), {"code": context["code"]})
        map_id = context.get("map_id")
        if not map_id or map_id == "live_map":
            map_id = self._effective_map_id(map_id, runtime_state=runtime_state) or map_id
        if map_id and map_id != "live_map":
            self._remember_working_map_id(map_id, reason="create_annotation")
        with self._data_lock:
            if map_id and map_id != "live_map" and not self._find_map_record_unlocked(map_id):
                return self._error("地图不存在")
            if not map_id:
                map_id = self._effective_map_id(runtime_state=runtime_state)
            if not map_id and runtime_state.get("map"):
                map_id = "live_map"
            if not map_id:
                return self._error("没有可用地图，请等待实时 /map 或先选择固定地图")
            selected_map_id = self._effective_map_id(runtime_state=runtime_state)
        annotation_source = str(payload.get("source") or "map_click").strip()
        require_live_pose = annotation_source == "robot_pose"
        annotation_readiness = self._annotation_create_readiness_payload(
            map_id,
            selected_map_id,
            require_live_pose=require_live_pose,
        )
        if not annotation_readiness.get("ready"):
            return self._error(
                str(annotation_readiness["message"]),
                validation_error_payload(annotation_readiness),
            )
        if map_id and map_id != "live_map":
            target_map_payload = self._map_file_snapshot(map_id)
            point_pose = dict(context["pose"])
            map_pose_error = annotation_map_pose_error_payload(point_pose, target_map_payload)
            if map_pose_error:
                return self._error(
                    str(map_pose_error["message"]),
                    {"code": map_pose_error["code"], "detail": map_pose_error["detail"]},
                )
        item = build_annotation_record(
            payload,
            context,
            annotation_id=new_id("point"),
            map_id=map_id,
            dwell_s=self._resolve_dwell_s(payload),
            now_text_value=now_text(),
        )
        with self._data_lock:
            self._annotations.append(item)
            self._save_json("annotations.json", self._annotations)
        self._append_event(
            "保存地图点位",
            {
                "annotation_id": item["id"],
                "floor": item["floor"],
                "manual_point_type": item.get("manual_point_type"),
            },
        )
        return {"ok": True, "annotation": item}

    def _annotation_create_readiness_payload(
        self,
        map_id: Optional[str],
        selected_map_id: Optional[str],
        require_live_pose: bool = True,
    ) -> Dict[str, Any]:
        selected_map_status = self._selected_map_status_payload(selected_map_id=selected_map_id)
        with self._data_lock:
            map_relocalization_required = self._settings.get("map_relocalization_required")
        with self._lock:
            pose = dict(self._state.get("pose") or {})
            localization_ok = self._state.get("localization_ok")
        pose_age = pose_age_sec(pose, time.time())
        pose_timeout_s = max(0.5, float(self.get_parameter("task_start_pose_timeout_s").value))
        return annotation_create_readiness_payload(
            map_id=map_id,
            selected_map_id=selected_map_id,
            selected_map_status=selected_map_status,
            map_relocalization_required=map_relocalization_required,
            pose=pose,
            localization_ok=localization_ok,
            pose_age_sec=pose_age,
            pose_timeout_s=pose_timeout_s,
            require_live_pose=require_live_pose,
            now_text=now_text,
        )

    def _resolve_dwell_s(self, payload: Dict[str, Any]) -> float:
        return resolve_annotation_dwell_s(
            payload,
            default_task_dwell_s=float(self.get_parameter("default_task_dwell_s").value),
            default_transition_dwell_s=float(self.get_parameter("default_transition_dwell_s").value),
            default_charge_dwell_s=float(self.get_parameter("default_charge_dwell_s").value),
        )

    def _delete_annotation(self, annotation_id: str) -> Dict[str, Any]:
        if not annotation_id:
            return self._error("缺少点位 id")
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") == "running" and annotation_id in (active.get("annotation_ids") or []):
                return self._error("点位正在当前任务中执行，请先停止任务再删除")
            before = len(self._annotations)
            self._annotations = [item for item in self._annotations if item.get("id") != annotation_id]
            if len(self._annotations) == before:
                return self._error("点位不存在")
            task_update = apply_deleted_annotation_to_tasks(
                self._tasks,
                annotation_id,
                now_text_value=now_text(),
            )
            self._tasks = list(task_update["tasks"])
            affected_tasks = list(task_update["affected_tasks"])
            self._save_json("annotations.json", self._annotations)
            self._save_json("tasks.json", self._tasks)
        return {"ok": True, "deleted": annotation_id, "affected_tasks": affected_tasks}

    def _tasks_payload(self, query: Optional[Dict[str, List[str]]] = None) -> Dict[str, Any]:
        include_all = self._as_bool((query or {}).get("include_all", [False])[0])
        requested_map_id = str(((query or {}).get("map_id") or [""])[0] or "").strip()
        effective_map_id = self._effective_map_id(requested_map_id or None)
        if (not requested_map_id or requested_map_id == "live_map") and effective_map_id:
            self._remember_working_map_id(effective_map_id, reason="tasks_effective_map")
        with self._data_lock:
            active_task = self._settings.get("active_task")
            active_running = active_task if isinstance(active_task, dict) and active_task.get("status") == "running" else None
            active_task_id = active_running.get("task_id") if active_running else None
            selected_map_id = effective_map_id
            map_relocalization_required = self._settings.get("map_relocalization_required")
            stale_result = stop_stale_running_tasks(
                self._tasks,
                active_task_id=active_task_id,
                now_text_value=now_text(),
            )
            if stale_result.get("changed"):
                self._tasks = list(stale_result["tasks"])
                self._save_json("tasks.json", self._tasks)
            all_tasks = [dict(item) for item in self._tasks]
            known_annotations = {
                str(item.get("id")): dict(item)
                for item in self._annotations
                if item.get("id")
            }
        task_list = task_list_filter_payload(
            all_tasks,
            selected_map_id=selected_map_id,
            include_all=include_all,
            annotations_by_id=known_annotations,
        )
        tasks = list(task_list["tasks"])
        with self._preflight_lock:
            preflight = self._preflight_with_age_unlocked()
        for task in tasks:
            task["waypoints"] = [
                task_waypoint_payload(str(annotation_id), known_annotations.get(str(annotation_id)), index)
                for index, annotation_id in enumerate(task.get("annotation_ids") or [])
            ]
            task["map_ids"] = sorted(
                {
                    str(known_annotations.get(str(annotation_id), {}).get("map_id") or "")
                    for annotation_id in (task.get("annotation_ids") or [])
                    if str(known_annotations.get(str(annotation_id), {}).get("map_id") or "")
                }
            )
            task["multi_map"] = len(task["map_ids"]) > 1
            task["floors"] = sorted(
                {
                    str(known_annotations.get(str(annotation_id), {}).get("floor") or "")
                    for annotation_id in (task.get("annotation_ids") or [])
                    if str(known_annotations.get(str(annotation_id), {}).get("floor") or "")
                }
            )
            task["multi_floor"] = len(task["floors"]) > 1
            task.pop("readiness", None)
        return {
            "ok": True,
            "tasks": tasks,
            "selected_map_id": selected_map_id,
            "effective_map_id": effective_map_id,
            "requested_map_id": requested_map_id or None,
            "selected_map_status": self._selected_map_status_payload(selected_map_id=selected_map_id),
            "map_relocalization_required": map_relocalization_required,
            "include_all": task_list["include_all"],
            "hidden_task_count": task_list["hidden_task_count"],
            "total_task_count": task_list["total_task_count"],
            "active_task": active_task,
            "preflight": preflight,
            "last_preflight_ok": bool(preflight and preflight.get("ok")),
        }

    def _validate_task_annotations_for_map(
        self,
        annotations: List[Optional[Dict[str, Any]]],
        task_map_id: str,
        map_cache: Optional[Dict[str, Dict[str, Any]]] = None,
    ) -> Optional[Dict[str, Any]]:
        expected_map_id = str(task_map_id or "").strip() or "live_map"
        target_map_payload: Optional[Dict[str, Any]] = None
        if expected_map_id and expected_map_id != "live_map":
            target_map_payload = self._map_file_snapshot_cached(expected_map_id, map_cache)
        target_map_payloads: Dict[str, Dict[str, Any]] = {}
        for annotation in annotations:
            if not isinstance(annotation, dict):
                continue
            annotation_map_id = str(annotation.get("map_id") or "").strip()
            if not annotation_map_id or annotation_map_id == "live_map":
                continue
            if annotation_map_id in target_map_payloads:
                continue
            target_map_payloads[annotation_map_id] = self._map_file_snapshot_cached(
                annotation_map_id,
                map_cache,
            )
        return contract_validate_task_annotations_for_map(
            annotations,
            expected_map_id,
            target_map_payload=target_map_payload,
            target_map_payloads=target_map_payloads or None,
            allow_multi_floor=True,
            allow_multi_map=True,
            now_text=now_text,
        )

    def _navigation_readiness_payload(
        self,
        check_lifecycle: bool = True,
        min_update_time: Optional[float] = None,
    ) -> Dict[str, Any]:
        now = time.time()
        timeout_s = max(0.5, float(self.get_parameter("relocalization_nav_timeout_s").value))
        with self._lock:
            scan = dict(self._state.get("scan") or {})
            local_costmap = dict(self._state.get("local_costmap") or {})
            global_costmap = dict(self._state.get("global_costmap") or {})
        lifecycle = None
        if check_lifecycle:
            lifecycle = self._check_lifecycle_nodes(
                ["/map_server", "/controller_server", "/planner_server", "/bt_navigator", "/waypoint_follower"]
            )
        return navigation_readiness_payload(
            scan=scan,
            local_costmap=local_costmap,
            global_costmap=global_costmap,
            lifecycle=lifecycle,
            check_lifecycle=check_lifecycle,
            timeout_s=timeout_s,
            now=now,
            min_update_time=min_update_time,
            now_text=now_text,
        )

    def _map_file_snapshot_cached(
        self,
        map_id: Optional[str],
        map_cache: Optional[Dict[str, Dict[str, Any]]],
    ) -> Dict[str, Any]:
        key = str(map_id or "")
        if map_cache is None:
            return self._map_file_snapshot(map_id)
        if key not in map_cache:
            map_cache[key] = self._map_file_snapshot(map_id)
        return map_cache[key]

    def _update_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        task_id = str(payload.get("task_id") or payload.get("id") or "").strip()
        name = str(payload.get("name") or "").strip()
        if not task_id:
            return self._error("缺少任务 id")
        if not name:
            return self._error("任务名称不能为空")
        with self._data_lock:
            update = apply_task_name_update(
                self._tasks,
                self._settings,
                task_id=task_id,
                name=name,
                now_text_value=now_text(),
            )
            if not update.get("ok"):
                return self._error(str(update["message"]), {"code": update["code"]})
            self._tasks = list(update["tasks"])
            if update.get("settings_changed"):
                self._settings = dict(update["settings"])
                self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
            updated = dict(update["task"])
        self._append_event("修改任务名称", {"task_id": task_id, "name": name})
        return {"ok": True, "task": updated}

    def _delete_task(self, task_id: str) -> Dict[str, Any]:
        task_id = str(task_id or "").strip()
        if not task_id:
            return self._error("缺少任务 id")
        with self._data_lock:
            delete = apply_task_delete(self._tasks, self._settings, task_id=task_id)
            if not delete.get("ok"):
                return self._error(str(delete["message"]), {"code": delete["code"]})
            self._tasks = list(delete["tasks"])
            if delete.get("settings_changed"):
                self._settings = dict(delete["settings"])
                self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        self._append_event("删除任务", {"task_id": task_id})
        return {"ok": True, "deleted": task_id}

    def _reset_navigation_session(
        self,
        reason: str,
        clear_costmaps: bool = True,
        publish_idle: bool = True,
    ) -> None:
        msg = String()
        msg.data = reason
        for _ in range(3):
            self.stop_task_pub.publish(msg)
            time.sleep(0.02)
        zero_samples = max(3, int(self.get_parameter("task_stop_zero_cmd_samples").value))
        self._publish_zero_cmd(samples=zero_samples)
        if clear_costmaps:
            self._clear_task_costmaps(reason)
        if publish_idle:
            self._publish_idle_waypoint(reason)
        self._clear_navigation_display_state(reason)
        self._append_event("复位导航会话", {"reason": reason, "clear_costmaps": clear_costmaps})

    def _clear_navigation_display_state(self, reason: str) -> None:
        with self._lock:
            current_version = int(self._state.get("path", {}).get("version", 0) or 0)
            self._state["path"] = {
                "version": current_version + 1,
                "points": [],
                "point_count": 0,
                "raw_point_count": 0,
                "last_point": None,
                "cleared_reason": reason,
                "last_update": time.time(),
            }
            self._state["pose_history"] = []

    def _publish_zero_cmd(self, samples: int = 1) -> None:
        count = max(1, int(samples))
        for index in range(count):
            self.cmd_vel_pub.publish(Twist())
            if index + 1 < count:
                time.sleep(0.03)

    def _clear_task_costmaps(self, reason: str) -> None:
        if ClearEntireCostmap is None:
            return
        for client in self.clear_costmap_clients:
            try:
                if not client.wait_for_service(timeout_sec=0.05):
                    continue
                future = client.call_async(ClearEntireCostmap.Request())
                future.add_done_callback(
                    lambda done, service_name=client.srv_name: self._on_clear_task_costmap_done(
                        done, service_name, reason
                    )
                )
            except Exception as exc:
                self.get_logger().warning("task costmap clear request failed for %s: %s" % (client.srv_name, exc))

    def _on_clear_task_costmap_done(self, future: Any, service_name: str, reason: str) -> None:
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warning("task costmap clear failed for %s reason=%s: %s" % (service_name, reason, exc))

    def _publish_idle_waypoint(self, reason: str) -> None:
        payload = build_idle_waypoint_payload(reason=reason, now_text=now_text())
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.active_waypoint_pub.publish(msg)

    def _append_active_task_timeline_event(
        self,
        active: Dict[str, Any],
        event: str,
        message: str,
        extra: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        max_events = max(10, int(self.get_parameter("task_timeline_max_events").value))
        return append_active_task_timeline_event_state(
            active,
            event=event,
            message=message,
            now_text=now_text(),
            now_monotonic=time.monotonic(),
            max_events=max_events,
            extra=extra,
        )

    def _update_task_terminal_state_unlocked(
        self,
        task_id: Optional[str],
        active: Dict[str, Any],
        status: str,
        message: Optional[str] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> bool:
        target_id = str(task_id or active.get("task_id") or "").strip()
        if not target_id:
            return False
        for task in self._tasks:
            if str(task.get("id") or "") != target_id:
                continue
            task["status"] = status
            task["last_error"] = message if status == "error" else ""
            task["updated_at"] = now_text()
            task.pop("last_result", None)
            task.pop("last_timeline", None)
            return True
        return False

    def _handle_navigation_status_for_task(self, status_text: str) -> None:
        status_text = str(status_text or "").strip()
        decision = classify_navigation_status(status_text)
        action = decision.get("action")
        if action == "ignore":
            return
        if action == "update_goal_status":
            self._update_active_task_from_nav_status(str(decision.get("goal_status") or ""), status_text)
            return
        if action in ("update_transition_status", "update_transition_feedback"):
            self._update_active_task_from_transition_nav_status(
                str(decision.get("goal_status") or ""),
                status_text,
            )
            return
        if action == "update_feedback":
            self._update_active_task_from_nav_feedback(status_text)
            return
        if action == "complete_waypoint":
            self._complete_active_waypoint_from_nav_result(status_text)
            return
        if action == "fail":
            self._fail_active_task_from_nav_status(str(decision.get("goal_status") or "error"), status_text)
            return
        self._update_active_task_status_message(status_text, save=False)

    def _fail_active_task_from_nav_status(self, status: str, status_text: str) -> None:
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") != "running":
                return
            task_id = active.get("task_id")
            active = apply_nav_failure_state(active, goal_status=status, status_text=status_text)
            failed = fail_active_task_state(
                active,
                message=str(active.get("status_message") or status_text),
                event_extra={"nav_status": status_text},
                terminal_event="nav_failed",
                terminal_status_text=status_text,
            )
            active = failed["active"]
            self._append_active_task_timeline_event(
                active,
                str(failed["event"]),
                str(failed["message"]),
                dict(failed["event_extra"]),
            )
            self._settings["active_task"] = active
            self._update_task_terminal_state_unlocked(
                task_id,
                active,
                str(failed["result_status"]),
                str(failed["message"]),
                dict(failed["event_extra"]),
            )
            self._settings["active_task"] = None
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        self._reset_navigation_session("navigation_error", clear_costmaps=True)
        self._append_event(str(failed["operator_event"]), failed["operator_payload"])

    def _update_active_task_from_transition_nav_status(
        self,
        status: str,
        status_text: str,
        save: bool = True,
    ) -> None:
        status_payload = parse_key_value_status(status_text)
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") != "running":
                return
            active = apply_transition_nav_status_state(
                active,
                goal_status=status or None,
                status_text=status_text,
                status_payload=status_payload,
                now_text=now_text(),
            )
            event_payload = transition_nav_status_event_payload(
                active,
                status_text=status_text,
                status_payload=status_payload,
            )
            self._append_active_task_timeline_event(
                active,
                str(event_payload["event"]),
                str(event_payload["message"]),
                event_payload.get("extra") if isinstance(event_payload.get("extra"), dict) else {},
            )
            self._settings["active_task"] = active
            if save:
                self._save_json("settings.json", self._settings)

    def _update_active_task_from_nav_status(
        self,
        status: str,
        status_text: str,
        save: bool = True,
    ) -> None:
        status_payload = parse_key_value_status(status_text)
        missing_failure = None
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") != "running":
                return
            annotation = self._active_annotation(active)
            if annotation is None:
                missing_failure = active_annotation_missing_failure(active)
            else:
                match = nav_status_matches_active_goal(active, annotation, status_payload)
                if not match["matches"]:
                    self._record_ignored_nav_status(
                        active,
                        "忽略与当前任务点不匹配的 Nav2 状态",
                        status_text,
                        match,
                    )
                    return
                active = apply_nav_goal_status_state(
                    active,
                    goal_status=status,
                    status_text=status_text,
                    match=match,
                    now_monotonic=time.monotonic(),
                    now_text=now_text(),
                )
                event_payload = nav_goal_status_event_payload(
                    active,
                    goal_status=status,
                    status_text=status_text,
                    status_payload=status_payload,
                    match=match,
                )
                self._append_active_task_timeline_event(
                    active,
                    str(event_payload["event"]),
                    str(event_payload["message"]),
                    event_payload.get("extra") if isinstance(event_payload.get("extra"), dict) else {},
                )
                self._settings["active_task"] = active
                if save:
                    self._save_json("settings.json", self._settings)
        if missing_failure is not None:
            self._fail_active_task_from_payload(missing_failure)

    def _update_active_task_status_message(
        self,
        status_text: str,
        save: bool = True,
    ) -> None:
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") != "running":
                return
            active = apply_nav_status_message_state(active, status_text=status_text, now_text=now_text())
            event_payload = nav_status_message_event_payload(active, status_text=status_text)
            self._append_active_task_timeline_event(
                active,
                str(event_payload["event"]),
                str(event_payload["message"]),
                event_payload.get("extra") if isinstance(event_payload.get("extra"), dict) else {},
            )
            self._settings["active_task"] = active
            if save:
                self._save_json("settings.json", self._settings)

    def _update_active_task_from_nav_feedback(self, status_text: str) -> None:
        dispatch = nav_feedback_dispatch_payload(status_text)
        feedback = dispatch.get("feedback") if isinstance(dispatch.get("feedback"), dict) else {}
        if dispatch.get("action") != "update_feedback":
            self._update_active_task_status_message(status_text, save=False)
            return
        missing_failure = None
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") != "running":
                return
            annotation = self._active_annotation(active)
            if annotation is None:
                missing_failure = active_annotation_missing_failure(active)
            else:
                match = nav_status_matches_active_goal(active, annotation, feedback)
                if not match["matches"]:
                    self._record_ignored_nav_status(
                        active,
                        "忽略与当前任务点不匹配的 Nav2 反馈",
                        status_text,
                        match,
                    )
                    return
                should_record = should_record_nav_feedback_event(active, feedback)
                active = apply_nav_feedback_state(
                    active,
                    status_text=status_text,
                    feedback=feedback,
                    match=match,
                    now_monotonic=time.monotonic(),
                    now_text=now_text(),
                )
                if should_record:
                    event_payload = nav_feedback_event_payload(
                        active,
                        status_text=status_text,
                        feedback=feedback,
                        match=match,
                    )
                    self._append_active_task_timeline_event(
                        active,
                        str(event_payload["event"]),
                        str(event_payload["message"]),
                        event_payload.get("extra") if isinstance(event_payload.get("extra"), dict) else {},
                    )
                self._settings["active_task"] = active
                self._save_json("settings.json", self._settings)
        if missing_failure is not None:
            self._fail_active_task_from_payload(missing_failure)

    def _complete_active_waypoint_from_nav_result(self, status_text: str) -> None:
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
        if active.get("status") != "running":
            return
        annotation = self._active_annotation(active)
        if annotation is None:
            failure = active_annotation_missing_failure(active)
            self._fail_active_task_from_payload(failure, task_id=active.get("task_id"))
            return
        decision = nav_success_completion_decision(
            active,
            annotation,
            status_text,
            goal_tolerance_m=float(self.get_parameter("goal_reached_tolerance_m").value),
        )
        if decision.get("action") == "fail":
            extra = decision.get("event_extra") if isinstance(decision.get("event_extra"), dict) else {}
            extra = {**extra, "reason": decision.get("reason")}
            self._fail_active_task(
                active.get("task_id"),
                str(decision.get("message") or "Nav2 到达状态缺少有效距离确认，已停止任务"),
                extra,
            )
            return
        if decision.get("action") != "complete":
            self._append_event(
                "忽略非当前任务点 Nav2 成功事件",
                decision.get("event_extra") if isinstance(decision.get("event_extra"), dict) else {},
            )
            return
        self._update_active_task_from_nav_status("succeeded", status_text)
        self._begin_waypoint_dwell_or_advance(annotation, "nav2_goal_succeeded")

    def _record_ignored_nav_status(
        self,
        active: Dict[str, Any],
        message: str,
        status_text: str,
        match: Dict[str, Any],
    ) -> None:
        current = dict(self._settings.get("active_task") or {})
        if current.get("status") != "running" or current.get("task_id") != active.get("task_id"):
            return
        current = apply_ignored_nav_status_state(current, status_text=status_text, match=match)
        event_payload = ignored_nav_status_event_payload(
            current,
            message=message,
            status_text=status_text,
            match=match,
        )
        self._append_active_task_timeline_event(
            current,
            str(event_payload["timeline_event"]),
            str(event_payload["timeline_message"]),
            dict(event_payload["timeline_extra"]),
        )
        self._settings["active_task"] = current
        self._save_json("settings.json", self._settings)
        self._append_event(
            str(event_payload["operator_event"]),
            dict(event_payload["operator_payload"]),
        )

    def _create_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            runtime_state = {"map": dict(self._state.get("map") or {})}
        selected_map_id = self._effective_map_id(runtime_state=runtime_state)
        self._remember_working_map_id(selected_map_id, reason="create_task")
        with self._data_lock:
            known = {item["id"]: item for item in self._annotations}
            static_context = task_create_static_context(
                payload,
                known,
                selected_map_id=selected_map_id,
                now_text=now_text,
            )
            if not static_context.get("ok"):
                error_payload = dict(static_context.get("error") or {})
                return self._error(
                    str(error_payload["message"]),
                    {key: value for key, value in error_payload.items() if key != "message"},
                )
            task_map_id = str(static_context["task_map_id"])
            selected_map_status = self._selected_map_status_payload(
                runtime_state,
                selected_map_id=selected_map_id,
            )
            if not selected_map_status.get("ready"):
                mismatch = task_create_map_metadata_mismatch_payload(
                    task_map_id=task_map_id,
                    selected_map_id=selected_map_id,
                    selected_map_status=selected_map_status,
                    now_text=now_text,
                )
                return self._error(str(mismatch["message"]), mismatch["error_extra"])
            readiness = self._validate_task_annotations_for_map(
                list(static_context.get("annotations") or []),
                task_map_id,
            )
            if readiness:
                return self._error(
                    str(readiness["message"]),
                    validation_error_payload(readiness),
                )
            task = build_task_create_record(
                static_context,
                task_id=new_id("task"),
                now_text_value=now_text(),
            )
            self._tasks.append(task)
            self._save_json("tasks.json", self._tasks)
        return {"ok": True, "task": task}

    def _task_start_pre_runtime_context(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        task_id = str(payload.get("task_id") or "").strip()
        selected_map_id = self._effective_map_id() or "live_map"
        self._remember_working_map_id(selected_map_id, reason="start_task")
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            task = self._find_by_id(self._tasks, task_id)
            known = {item.get("id"): item for item in self._annotations if item.get("id")}
            static_context = task_start_static_context(
                task_id,
                task,
                known,
                selected_map_id=selected_map_id,
                now_text=now_text,
            )
            task_validation = None
            if static_context.get("ok"):
                task_validation = self._validate_task_annotations_for_map(
                    list(static_context.get("annotations") or []),
                    str(static_context["task_map_id"]),
                )
            if active.get("status") == "running":
                return self._error(
                    "当前任务正在执行中" if active.get("task_id") == task_id else "已有任务正在执行，请先停止当前任务",
                    {
                        "code": "task_running",
                        "task_id": task_id,
                        "active_task_id": active.get("task_id"),
                    },
                )
            if not static_context.get("ok") or task_validation:
                validation = dict(task_validation or static_context.get("validation") or {})
                if not validation:
                    error_payload = dict(static_context.get("error") or {})
                    validation = {
                        "code": str(error_payload.get("code") or "task_static_context_invalid"),
                        "message": str(error_payload.get("message") or "任务静态条件无效"),
                    }
                failure_state = apply_task_start_pre_runtime_failure_state(
                    self._tasks,
                    task_id=task_id,
                    static_context=static_context,
                    task_validation=task_validation,
                    validation=validation,
                    now_text_value=now_text(),
                )
                if failure_state.get("changed"):
                    self._tasks = list(failure_state["tasks"])
                    self._save_json("tasks.json", self._tasks)
                return self._error(
                    str(validation.get("message") or "任务静态条件无效"),
                    validation_error_payload(validation),
                )
            task_map_id = str(static_context.get("task_map_id") or "live_map")
            selected_map_id = str(selected_map_id or "live_map")
            annotation_map_ids = {
                str(item.get("map_id") or "").strip()
                for item in (static_context.get("annotations") or [])
                if isinstance(item, dict) and str(item.get("map_id") or "").strip()
            }
            if task_map_id != selected_map_id and selected_map_id not in annotation_map_ids:
                return self._error(
                    "当前地图与任务地图不一致，请先切换到任务对应地图",
                    {
                        "code": "selected_map_mismatch",
                        "task_id": task_id,
                        "task_map_id": task_map_id,
                        "selected_map_id": selected_map_id,
                    },
                )
            first_annotation = static_context.get("first_annotation")
            expectation_error = validate_task_start_expectations(
                payload,
                task,
                first_annotation,
                task_map_id,
            )
            if expectation_error:
                return expectation_error
            return {
                "ok": True,
                "task_id": task_id,
                "task": task,
                "task_map_id": task_map_id,
                "selected_map_id": selected_map_id,
                "first_annotation": first_annotation,
            }

    def _start_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        context = self._task_start_pre_runtime_context(payload)
        if not context.get("ok"):
            return context
        self._reset_navigation_session("before_start_task", clear_costmaps=True)
        settle_s = max(0.0, float(self.get_parameter("task_start_settle_s").value))
        if settle_s > 0.0:
            time.sleep(min(settle_s, 2.0))
        context = self._task_start_pre_runtime_context(payload)
        if not context.get("ok"):
            return context
        task = context.get("task")
        task_id = str(context.get("task_id") or "")
        task_map_id = str(context.get("task_map_id") or "live_map")
        with self._data_lock:
            created = create_active_task_state(
                task,
                task_map_id=task_map_id,
                now_text=now_text(),
            )
            active = created["active"]
            self._append_active_task_timeline_event(
                active,
                str(created["event"]),
                str(created["message"]),
                created.get("event_extra") if isinstance(created.get("event_extra"), dict) else {},
            )
            self._settings["active_task"] = active
            task["status"] = "running"
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        self._dispatch_active_goal(force=True)
        self._append_event(str(created["operator_event"]), created["operator_payload"])
        with self._data_lock:
            return {
                "ok": True,
                "active_task": self._settings.get("active_task"),
            }

    def _stop_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        stop_request = normalize_stop_task_request(payload)
        reason = str(stop_request["reason"])
        stopped_task_id = None
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            stopped_task_id = active.get("task_id")
            if stopped_task_id:
                stopped = stop_task_state(active, reason=reason)
                active = stopped["active"]
                self._append_active_task_timeline_event(
                    active,
                    str(stopped["event"]),
                    str(stopped["message"]),
                    dict(stopped["event_extra"]),
                )
                self._update_task_terminal_state_unlocked(
                    stopped_task_id,
                    active,
                    str(stopped["result_status"]),
                    str(stopped["message"]),
                    dict(stopped["event_extra"]),
                )
            self._settings["active_task"] = None
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        self._reset_navigation_session(reason, clear_costmaps=True)
        if stopped_task_id:
            operator_event = str(stopped["operator_event"])
            operator_payload = stopped["operator_payload"]
        else:
            idle = idle_stop_task_response(reason)
            operator = stop_task_operator_event_payload(task_id=stopped_task_id, reason=reason)
            operator_event = str(operator["operator_event"])
            operator_payload = operator["operator_payload"]
        self._append_event(operator_event, operator_payload)
        if not stopped_task_id:
            return idle
        return {
            "ok": True,
            "active_task": None,
            "stopped_task_id": stopped_task_id,
            "reset_navigation": True,
            "message": "已发送停止/复位指令" if stopped_task_id else "已显式复位导航状态",
        }

    def _publish_initialpose(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") == "running":
                return self._error("任务执行中不能重定位，请先停止当前任务")
        request_started_at = time.time()
        request = parse_initialpose_request(payload)
        if not request.get("ok"):
            return self._error(
                str(request["message"]),
                {key: value for key, value in request.items() if key not in ("ok", "message")},
            )
        pose = dict(request["pose"])
        x = float(pose["x"])
        y = float(pose["y"])
        z = float(pose["z"])
        yaw = float(pose["yaw"])
        frame_id = str(request.get("frame_id") or "map")
        floor = str(request.get("floor") or "")
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        xy_cov = max(0.0, float(self.get_parameter("initialpose_covariance_xy").value))
        yaw_cov = max(0.0, float(self.get_parameter("initialpose_covariance_yaw").value))
        msg.pose.covariance[0] = xy_cov
        msg.pose.covariance[7] = xy_cov
        msg.pose.covariance[35] = yaw_cov
        repeats = max(1, int(self.get_parameter("initialpose_publish_repeats").value))
        interval_s = max(0.0, float(self.get_parameter("initialpose_publish_interval_s").value))
        for _ in range(repeats):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(msg)
            if interval_s > 0.0:
                time.sleep(interval_s)
        verification = self._wait_for_relocalization_verification(
            request_started_at,
            {"x": x, "y": y, "z": z, "yaw": yaw},
        )
        if bool(verification.get("factory_pose_accepted")):
            with self._data_lock:
                self._settings.pop("map_relocalization_required", None)
                self._save_json("settings.json", self._settings)
        status = relocalization_response_payload(
            verification,
            now_text=now_text,
        )
        result = initialpose_api_response_payload(
            localization_status=status,
            verification=verification,
            topic=self.get_parameter("initialpose_topic").value,
            publish_repeats=repeats,
            frame_id=frame_id,
            floor=floor,
            pose={"x": x, "y": y, "z": z, "yaw": yaw},
        )
        self._append_event("网页发布重定位", result)
        return result

    def _wait_for_relocalization_verification(
        self,
        request_started_at: float,
        requested_pose: Dict[str, float],
    ) -> Dict[str, Any]:
        timeout_s = max(0.5, float(self.get_parameter("relocalization_verify_timeout_s").value))
        pose_tolerance_m = max(
            0.1,
            float(self.get_parameter("relocalization_pose_tolerance_m").value),
        )
        deadline = time.time() + timeout_s
        evidence: Dict[str, Any] = {
            "tcp_2101_accepted": False,
            "tcp_2101_result": "",
            "tcp_2101_fresh": False,
            "localization_ok": False,
            "pose_ok": False,
            "pose_near_request": False,
            "scan_ok": False,
            "local_costmap_ok": False,
            "global_costmap_ok": False,
            "pose_error_m": None,
            "yaw_error_rad": None,
        }

        while time.time() < deadline:
            with self._lock:
                relocalization = dict(self._state.get("relocalization_result") or {})
                pose = dict(self._state.get("pose") or {})
                localization = self._factory_localization_ok(self._state)
                scan = dict(self._state.get("scan") or {})
                local_costmap = dict(self._state.get("local_costmap") or {})
                global_costmap = dict(self._state.get("global_costmap") or {})

            evidence = relocalization_sample_evidence(
                request_started_at=request_started_at,
                requested_pose=requested_pose,
                relocalization=relocalization,
                pose=pose,
                localization_ok=localization,
                scan=scan,
                local_costmap=local_costmap,
                global_costmap=global_costmap,
                pose_tolerance_m=pose_tolerance_m,
            )

            if evidence.get("ready_to_finish_wait"):
                break
            time.sleep(0.2)

        navigation_readiness = self._navigation_readiness_payload(check_lifecycle=False)
        return manual_relocalization_verification_payload(
            tcp_2101_accepted=bool(evidence.get("tcp_2101_accepted")),
            tcp_2101_result=(
                str(evidence.get("tcp_2101_result") or "")
                if evidence.get("tcp_2101_fresh")
                else "未收到 /m20pro_tcp_bridge/relocalization_result"
            ),
            localization_ok=bool(evidence.get("localization_ok")),
            pose_ok=bool(evidence.get("pose_ok")),
            pose_near_request=bool(evidence.get("pose_near_request")),
            scan_ok=bool(evidence.get("scan_ok")),
            local_costmap_ok=bool(evidence.get("local_costmap_ok")),
            global_costmap_ok=bool(evidence.get("global_costmap_ok")),
            pose_error_m=evidence.get("pose_error_m"),
            yaw_error_rad=evidence.get("yaw_error_rad"),
            pose_tolerance_m=pose_tolerance_m,
            navigation_readiness=navigation_readiness,
            timeout_s=timeout_s,
        )

    def _tick_active_task(self) -> None:
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
        if active.get("status") != "running":
            return
        annotation = self._active_annotation(active)
        if annotation is None:
            failure = active_annotation_missing_failure(active)
            self._fail_active_task_from_payload(failure, task_id=active.get("task_id"))
            return
        if active.get("phase") == "dwelling":
            decision = dwell_tick_decision(active, now_time=time.time())
            if decision.get("action") == "wait":
                self._publish_active_waypoint(annotation, active, "dwelling")
                return
            if decision.get("action") == "advance":
                self._advance_active_task(annotation)
                return
        if self._active_waypoint_waiting_cross_floor(active, annotation):
            target_floor = str(annotation.get("floor") or "")
            self._mark_active_task_waiting(
                active,
                "cross_floor_transitioning",
                "跨楼层目标已下发，等待 floor_manager 切换到 %s" % target_floor,
            )
            with self._data_lock:
                active_snapshot = dict(self._settings.get("active_task") or active)
            self._publish_active_waypoint(annotation, active_snapshot, "cross_floor")
            if self._stop_task_if_cross_floor_unresponsive(active_snapshot, annotation):
                return
            if self._stop_task_if_cross_floor_transition_timed_out(active_snapshot, annotation):
                return
            return
        with self._lock:
            pose = dict(self._state.get("pose") or {})
            current_floor = self._state.get("floor")
            localization_ok = self._factory_localization_ok(self._state)
            navigation_status = self._state.get("navigation_status")
            scan = dict(self._state.get("scan") or {})
        scan_guard = self._active_task_scan_guard(scan)
        if not scan_guard.get("ok"):
            if self._handle_active_task_runtime_loss(
                active,
                reason=str(scan_guard.get("reason") or "scan_unavailable"),
                message=str(scan_guard.get("message") or "任务执行中 /scan 不可用"),
                timeout_s=float(self.get_parameter("task_runtime_scan_lost_stop_s").value),
                extra={
                    "scan_age_s": scan_guard.get("age_s"),
                    "finite_ranges": scan_guard.get("finite_ranges"),
                    "frame_id": scan_guard.get("frame_id"),
                },
            ):
                return
            with self._data_lock:
                active = dict(self._settings.get("active_task") or active)
            self._mark_active_task_waiting(
                active,
                str(scan_guard.get("reason") or "scan_unavailable"),
                str(scan_guard.get("message") or "任务执行中 /scan 不可用"),
            )
            return
        pose_age = pose_age_sec(pose, time.time())
        pose_timeout_s = max(0.5, float(self.get_parameter("task_start_pose_timeout_s").value))
        pre_dispatch = active_task_pre_dispatch_decision(
            active=active,
            pose=pose,
            annotation=annotation,
            current_floor=current_floor,
            localization_ok=localization_ok,
            pose_age=pose_age,
            pose_timeout_s=pose_timeout_s,
        )
        if pre_dispatch.get("action") == "pass_cross_floor":
            self._clear_active_task_runtime_loss(active)
            self._mark_active_task_waiting(
                active,
                str(pre_dispatch["code"]),
                str(pre_dispatch["message"]),
            )
            self._dispatch_active_goal(force=False)
            return
        if pre_dispatch.get("action") == "wait_and_monitor_localization":
            if self._handle_active_task_runtime_loss(
                active,
                reason=str(pre_dispatch.get("reason") or pre_dispatch.get("code") or "localization_lost"),
                message=str(pre_dispatch.get("message") or "任务执行中定位/位姿暂时丢失"),
                timeout_s=float(self.get_parameter("task_runtime_localization_lost_stop_s").value),
                extra={
                    "pose_age_s": pre_dispatch.get("pose_age_s"),
                    "pose_timeout_s": pre_dispatch.get("pose_timeout_s"),
                },
            ):
                return
            with self._data_lock:
                active = dict(self._settings.get("active_task") or active)
            self._mark_active_task_waiting(
                active,
                str(pre_dispatch["code"]),
                str(pre_dispatch["message"]),
            )
            return
        if pre_dispatch.get("action") == "wait":
            self._clear_active_task_runtime_loss(active)
            self._mark_active_task_waiting(
                active,
                str(pre_dispatch["code"]),
                str(pre_dispatch["message"]),
            )
            return
        if pre_dispatch.get("action") == "fail":
            self._clear_active_task_runtime_loss(active)
            self._mark_active_task_waiting(
                active,
                str(pre_dispatch.get("code") or "task_waiting"),
                str(pre_dispatch.get("message") or "任务等待现场状态恢复"),
            )
            return
        self._clear_active_task_runtime_loss(active)
        distance = float(pre_dispatch.get("distance_m"))
        self._update_active_task_progress(active, annotation, pose, distance, navigation_status)
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
        if active.get("status") != "running":
            return
        near_goal_decision = near_goal_wait_decision(
            active,
            annotation,
            distance=distance,
            goal_tolerance_m=float(self.get_parameter("goal_reached_tolerance_m").value),
            now_monotonic=time.monotonic(),
            now_text=now_text(),
        )
        if near_goal_decision.get("action") == "wait_for_nav2":
            if near_goal_decision.get("changed"):
                with self._data_lock:
                    current = dict(self._settings.get("active_task") or {})
                    result = prepare_near_goal_wait_update(current, active, near_goal_decision)
                    current = result["active"]
                    if result["action"] == "update":
                        self._settings["active_task"] = current
                        self._save_json("settings.json", self._settings)
                with self._data_lock:
                    active = dict(self._settings.get("active_task") or {})
            self._mark_active_task_waiting(
                active,
                str(near_goal_decision["reason"]),
                str(near_goal_decision["message"]),
            )
            return
        self._dispatch_active_goal(force=False)

    def _active_task_scan_guard(self, scan: Dict[str, Any]) -> Dict[str, Any]:
        timeout_s = max(0.2, float(self.get_parameter("task_runtime_scan_timeout_s").value))
        min_finite = max(1, int(self.get_parameter("task_runtime_scan_min_finite_ranges").value))
        last_update = scan.get("last_update")
        age_s = None
        if last_update is not None:
            try:
                age_s = max(0.0, time.time() - float(last_update))
            except (TypeError, ValueError):
                age_s = None
        finite_ranges = int(scan.get("finite_ranges", 0) or 0)
        if age_s is None:
            return {
                "ok": False,
                "reason": "scan_unavailable",
                "message": "任务执行中没有收到 /scan，已暂停；若 2 秒内不恢复将自动停止任务",
                "age_s": None,
                "finite_ranges": finite_ranges,
                "frame_id": scan.get("frame_id"),
            }
        if age_s > timeout_s:
            return {
                "ok": False,
                "reason": "scan_stale",
                "message": "任务执行中 /scan 已过期 %.1f 秒，已暂停；若 2 秒内不恢复将自动停止任务" % age_s,
                "age_s": age_s,
                "finite_ranges": finite_ranges,
                "frame_id": scan.get("frame_id"),
            }
        if finite_ranges < min_finite:
            return {
                "ok": False,
                "reason": "scan_sparse",
                "message": "任务执行中 /scan 有效点过少 %d/%d，已暂停；若 2 秒内不恢复将自动停止任务"
                % (finite_ranges, min_finite),
                "age_s": age_s,
                "finite_ranges": finite_ranges,
                "frame_id": scan.get("frame_id"),
            }
        return {
            "ok": True,
            "age_s": age_s,
            "finite_ranges": finite_ranges,
            "frame_id": scan.get("frame_id"),
        }

    def _handle_active_task_runtime_loss(
        self,
        active: Dict[str, Any],
        *,
        reason: str,
        message: str,
        timeout_s: float,
        extra: Optional[Dict[str, Any]] = None,
    ) -> bool:
        now_monotonic = time.monotonic()
        task_id = active.get("task_id")
        failure_payload = None
        with self._data_lock:
            current = dict(self._settings.get("active_task") or {})
            if current.get("status") != "running" or current.get("task_id") != task_id:
                return True
            timer_source = dict(current)
            if str(timer_source.get("localization_lost_reason") or "") != str(reason):
                timer_source.pop("localization_lost_started_monotonic", None)
            decision = localization_lost_timeout_decision(
                timer_source,
                reason=reason,
                now_monotonic=now_monotonic,
                timeout_s=max(0.5, float(timeout_s)),
            )
            action = str(decision.get("action") or "")
            if action == "fail":
                failure_extra = localization_lost_failure_extra(decision)
                failure_extra.update(
                    {
                        "reason": reason,
                        "message": message,
                    }
                )
                if extra:
                    failure_extra.update(extra)
                failure_payload = {
                    "task_id": task_id,
                    "message": str(decision.get("message") or message),
                    "extra": failure_extra,
                }
            elif action == "start_timer":
                result = apply_localization_lost_start_state(
                    current,
                    decision,
                    fallback_monotonic=now_monotonic,
                )
                updated = result["active"]
                updated["localization_lost_reason"] = reason
                updated["status_message"] = message
                event = localization_lost_start_event_payload(updated, decision)
                if result.get("changed") or current.get("localization_lost_reason") != reason:
                    self._append_active_task_timeline_event(
                        updated,
                        str(event["event"]),
                        str(event["message"]),
                        dict(event["extra"]),
                    )
                    self._settings["active_task"] = updated
                    self._save_json("settings.json", self._settings)
            elif action == "wait":
                updated = dict(current)
                updated["localization_lost_reason"] = reason
                updated["status_message"] = message
                self._settings["active_task"] = updated
                self._save_json("settings.json", self._settings)
        if failure_payload is not None:
            self._fail_active_task(
                failure_payload["task_id"],
                failure_payload["message"],
                failure_payload["extra"],
            )
            return True
        return False

    def _clear_active_task_runtime_loss(self, active: Dict[str, Any]) -> None:
        task_id = active.get("task_id")
        with self._data_lock:
            current = dict(self._settings.get("active_task") or {})
            if current.get("status") != "running" or current.get("task_id") != task_id:
                return
            changed = False
            for key in ("localization_lost_started_monotonic", "localization_lost_reason"):
                if key in current:
                    current.pop(key, None)
                    changed = True
            if changed:
                self._settings["active_task"] = current
                self._save_json("settings.json", self._settings)

    def _active_waypoint_waiting_cross_floor(
        self,
        active: Dict[str, Any],
        annotation: Dict[str, Any],
    ) -> bool:
        target_floor = str(annotation.get("floor") or "").strip()
        if not target_floor:
            return False
        with self._lock:
            current_floor = str(self._state.get("floor") or "").strip()
        if current_floor == target_floor:
            return False
        return (
            active.get("last_goal_annotation_id") == annotation.get("id")
            and active.get("last_floor_goal_annotation_id") == annotation.get("id")
            and bool(active.get("last_floor_goal_cross_floor"))
        )

    def _stop_task_if_cross_floor_unresponsive(
        self,
        active: Dict[str, Any],
        annotation: Dict[str, Any],
    ) -> bool:
        published_at = 0.0
        try:
            published_at = float(active.get("last_floor_goal_published_monotonic") or 0.0)
        except (TypeError, ValueError):
            published_at = 0.0
        if published_at <= 0.0:
            return False
        if active.get("last_transition_nav_status") or active.get("last_nav_status"):
            return False
        timeout_s = max(1.0, float(self.get_parameter("task_goal_accept_timeout_s").value))
        age_s = max(0.0, time.monotonic() - published_at)
        if age_s < timeout_s:
            return False
        self._fail_active_task(
            str(active.get("task_id") or ""),
            "跨楼层目标下发 %.1f 秒后未收到 floor_manager/Nav2 回应，已停止任务；请检查 floor_manager、/m20pro/stair_status 和 /m20pro/floor_goal"
            % timeout_s,
            {
                "reason": "cross_floor_goal_no_response",
                "annotation_id": annotation.get("id"),
                "label": annotation.get("label"),
                "target_floor": annotation.get("floor"),
                "age_s": age_s,
                "timeout_s": timeout_s,
            },
        )
        return True

    def _stop_task_if_cross_floor_transition_timed_out(
        self,
        active: Dict[str, Any],
        annotation: Dict[str, Any],
    ) -> bool:
        published_at = 0.0
        try:
            published_at = float(active.get("last_floor_goal_published_monotonic") or 0.0)
        except (TypeError, ValueError):
            published_at = 0.0
        if published_at <= 0.0:
            return False
        timeout_s = max(10.0, float(self.get_parameter("task_waypoint_timeout_s").value))
        age_s = max(0.0, time.monotonic() - published_at)
        if age_s < timeout_s:
            return False
        self._fail_active_task(
            str(active.get("task_id") or ""),
            "跨楼层目标下发 %.1f 秒后仍未切换到目标楼层，已停止任务；请查看最近楼梯阶段、/m20pro/stair_status 和 Nav2 状态"
            % timeout_s,
            {
                "reason": "cross_floor_transition_timeout",
                "annotation_id": annotation.get("id"),
                "label": annotation.get("label"),
                "source_floor": active.get("last_floor_goal_source_floor"),
                "target_floor": annotation.get("floor"),
                "last_transition_nav_status": active.get("last_transition_nav_status"),
                "last_transition_nav_label": active.get("last_transition_nav_label"),
                "last_transition_nav_payload": active.get("last_transition_nav_payload"),
                "last_nav_status": active.get("last_nav_status"),
                "age_s": age_s,
                "timeout_s": timeout_s,
            },
        )
        return True

    def _begin_waypoint_dwell_or_advance(self, annotation: Dict[str, Any], reason: str) -> None:
        self._publish_zero_cmd(samples=3)
        dwell_s = annotation_dwell_s(annotation)
        if dwell_s > 0.0:
            active_snapshot = None
            event_extra = None
            with self._data_lock:
                active = self._settings.get("active_task") or {}
                result = begin_waypoint_dwell_state(
                    active,
                    annotation,
                    dwell_s=dwell_s,
                    now_text=now_text(),
                    now_time=time.time(),
                    reason=reason,
                )
                if not result.get("changed"):
                    return
                active = result["active"]
                event_extra = dict(result["event_extra"])
                self._append_active_task_timeline_event(
                    active,
                    str(result["event"]),
                    str(result["message"]),
                    event_extra,
                )
                self._settings["active_task"] = active
                self._save_json("settings.json", self._settings)
                active_snapshot = dict(active)
            self._append_event(
                str(result["operator_event"]),
                dict(result["operator_payload"]),
            )
            self._publish_active_waypoint(annotation, active_snapshot or {}, "dwelling")
            return
        self._advance_active_task(annotation)

    def _update_active_task_progress(
        self,
        active: Dict[str, Any],
        annotation: Dict[str, Any],
        pose: Dict[str, Any],
        distance: float,
        navigation_status: Any,
    ) -> None:
        now_monotonic = time.monotonic()
        with self._data_lock:
            current = dict(self._settings.get("active_task") or {})
            if current.get("status") != "running" or current.get("task_id") != active.get("task_id"):
                return
            result = update_active_task_progress_state(
                current,
                annotation,
                pose,
                distance=distance,
                navigation_status=navigation_status,
                now_monotonic=now_monotonic,
                now_text=now_text(),
                goal_tolerance_m=float(self.get_parameter("goal_reached_tolerance_m").value),
                min_pose_movement_m=max(0.0, float(self.get_parameter("task_progress_min_pose_movement_m").value)),
                min_distance_delta_m=max(0.0, float(self.get_parameter("task_progress_min_distance_delta_m").value)),
            )
            current = result["active"]
            self._settings["active_task"] = current
            self._save_json("settings.json", self._settings)

    def _mark_active_task_waiting(
        self,
        active: Dict[str, Any],
        code: str,
        message: str,
    ) -> None:
        with self._data_lock:
            current = dict(self._settings.get("active_task") or {})
            if current.get("status") != "running" or current.get("task_id") != active.get("task_id"):
                return
            result = mark_active_task_waiting_state(
                current,
                code=code,
                message=message,
                now_text=now_text(),
            )
            current = result["active"]
            if result.get("should_record_event"):
                self._append_active_task_timeline_event(
                    current,
                    str(result["event"]),
                    str(result["message"]),
                    dict(result["event_extra"]),
                )
            if result.get("changed"):
                self._settings["active_task"] = current
                self._save_json("settings.json", self._settings)

    def _fail_active_task_from_payload(
        self,
        failure: Dict[str, Any],
        default_message: Optional[str] = None,
        *,
        task_id: Any = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        payload = active_task_failure_payload(
            failure,
            default_message=default_message,
            task_id=task_id,
            extra=extra,
        )
        self._fail_active_task(
            payload.get("task_id"),
            str(payload["message"]),
            payload.get("extra"),
        )

    def _fail_active_task(
        self,
        task_id: Optional[str],
        message: str,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") != "running":
                return
            if task_id and active.get("task_id") != task_id:
                return
            task_id = active.get("task_id")
            failed = fail_active_task_state(active, message=message, event_extra=extra or {})
            active = failed["active"]
            self._append_active_task_timeline_event(
                active,
                str(failed["event"]),
                str(failed["message"]),
                dict(failed["event_extra"]),
            )
            self._update_task_terminal_state_unlocked(
                task_id,
                active,
                str(failed["result_status"]),
                str(failed["message"]),
                dict(failed["event_extra"]),
            )
            self._settings["active_task"] = None
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        self._reset_navigation_session("navigation_error", clear_costmaps=True)
        self._append_event(str(failed["operator_event"]), failed["operator_payload"])

    def _advance_active_task(self, annotation: Dict[str, Any]) -> None:
        completed_task_id = None
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            result = advance_active_task_state(active, annotation, now_text=now_text())
            if not result.get("changed"):
                return
            active = result["active"]
            self._append_active_task_timeline_event(
                active,
                str(result["event"]),
                str(result["message"]),
                dict(result["event_extra"]),
            )
            if result.get("completed"):
                completed_task_id = result.get("task_id") or active.get("task_id")
                self._update_task_terminal_state_unlocked(
                    completed_task_id,
                    active,
                    str(result["result_status"]),
                    str(result["message"]),
                    dict(result["event_extra"]),
                )
            else:
                completed_task_id = None
            self._settings["active_task"] = None if completed_task_id else active
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        if completed_task_id:
            self._reset_navigation_session("task_completed", clear_costmaps=True)
            self._append_event(str(result["operator_event"]), result["operator_payload"])
        self._dispatch_active_goal(force=True)

    def _dispatch_active_goal(self, force: bool) -> None:
        with self._data_lock:
            active = self._settings.get("active_task") or {}
        if active.get("status") != "running":
            return
        annotation = self._active_annotation(active)
        if annotation is None:
            failure = active_annotation_missing_failure(active)
            self._fail_active_task_from_payload(failure, task_id=active.get("task_id"))
            return
        now_monotonic = time.monotonic()
        decision = goal_dispatch_decision(
            active,
            annotation,
            force=force,
            now_monotonic=now_monotonic,
            resend_interval_s=float(self.get_parameter("task_goal_resend_interval_s").value),
        )
        if decision.get("action") == "publish_status":
            self._publish_active_waypoint(annotation, active, str(decision.get("phase") or "navigating"))
            return
        if decision.get("action") != "send_goal":
            return
        operator_payload = decision.get("operator_payload") if isinstance(decision.get("operator_payload"), dict) else None
        if operator_payload:
            self._append_event(
                str(decision["operator_event"]),
                operator_payload,
        )
        goal = waypoint_goal_payload(annotation)
        if not goal.get("ok"):
            self._fail_active_task_from_payload(
                goal,
                task_id=active.get("task_id"),
                extra=waypoint_goal_failure_extra(annotation),
            )
            return
        with self._lock:
            current_path = dict(self._state.get("path") or {})
        goal_sent_path_version = current_path.get("version")
        missing_failure = None
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            current_annotation = self._active_annotation(active) if active.get("status") == "running" else None
            prepared = prepare_goal_send_state(
                active,
                annotation,
                current_annotation,
                goal,
                now_text=now_text(),
                now_monotonic=now_monotonic,
                path_version=goal_sent_path_version,
                goal_attempt_id=new_id("goal"),
                goal_semantics=annotation_semantics_payload(annotation),
            )
            action = str(prepared.get("action") or "")
            if action == "idle":
                return
            if action == "fail":
                missing_failure = prepared["failure"]
            elif action == "record_stale":
                self._append_active_task_timeline_event(
                    active,
                    str(prepared["event"]),
                    str(prepared["message"]),
                    dict(prepared["event_extra"]),
                )
                self._settings["active_task"] = active
                self._save_json("settings.json", self._settings)
                return
            elif action == "send_goal":
                active = prepared["active"]
                self._append_active_task_timeline_event(
                    active,
                    str(prepared["event"]),
                    str(prepared["message"]),
                    dict(prepared["event_extra"]),
                )
                self._settings["active_task"] = active
                self._save_json("settings.json", self._settings)
                active_snapshot = dict(active)
            else:
                return
        if missing_failure is not None:
            self._fail_active_task_from_payload(missing_failure)
            return
        with self._lock:
            source_floor = str(self._state.get("floor") or "").strip()
        self._publish_floor_goal(str(goal["floor"]), float(goal["x"]), float(goal["y"]), float(goal["yaw"]), float(goal["z"]))
        with self._data_lock:
            current = dict(self._settings.get("active_task") or {})
            if current.get("status") == "running" and current.get("task_id") == active_snapshot.get("task_id"):
                result = mark_floor_goal_published_state(
                    current,
                    annotation,
                    goal,
                    now_text=now_text(),
                    now_monotonic=time.monotonic(),
                    source_floor=source_floor,
                )
                current = result["active"]
                self._append_active_task_timeline_event(
                    current,
                    str(result["event"]),
                    str(result["message"]),
                    result.get("event_extra") if isinstance(result.get("event_extra"), dict) else {},
                )
                self._settings["active_task"] = current
                self._save_json("settings.json", self._settings)
                active_snapshot = dict(current)
        self._publish_active_waypoint(annotation, active_snapshot, "navigating")

    def _publish_floor_goal(self, floor: str, x: float, y: float, yaw: float, z: float = 0.0) -> None:
        if not floor:
            raise ValueError("floor is required")
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = floor
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        yaw_to_orientation(msg, yaw)
        self.floor_goal_pub.publish(msg)
        self.get_logger().info("web task published floor goal floor=%s x=%.2f y=%.2f yaw=%.2f" % (floor, x, y, yaw))

    def _publish_active_waypoint(
        self,
        annotation: Dict[str, Any],
        active: Dict[str, Any],
        phase: str,
    ) -> None:
        now_time = time.time()
        payload = build_active_waypoint_payload(
            active,
            annotation,
            phase=phase,
            now_text=now_text(),
            now_time=now_time,
            now_monotonic=time.monotonic(),
            waypoint=annotation_semantics_payload(annotation),
        )
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.active_waypoint_pub.publish(msg)

    def _validate_task_annotation_order(
        self,
        annotations: List[Dict[str, Any]],
    ) -> Optional[Dict[str, Any]]:
        normalized = []
        for annotation in annotations:
            normalize_annotation_semantics(annotation)
            normalized.append(annotation)
        return validate_task_annotation_order(normalized)

    def _active_annotation(self, active: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        with self._data_lock:
            return active_annotation_from_list(active, self._annotations)

    def _find_session(self, session_id: Optional[str]) -> Optional[Dict[str, Any]]:
        with self._data_lock:
            if session_id:
                return self._find_by_id(self._sessions, session_id)
            return self._sessions[-1] if self._sessions else None

    @staticmethod
    def _find_by_id(items: List[Dict[str, Any]], item_id: Optional[str]) -> Optional[Dict[str, Any]]:
        if not item_id:
            return None
        for item in items:
            if item.get("id") == item_id:
                return item
        return None

    def _command_context(self, session: Dict[str, Any]) -> Dict[str, str]:
        return mapping_command_context(
            session,
            factory_host=str(self.get_parameter("factory_host").value),
            factory_user=str(self.get_parameter("factory_user").value),
            factory_active_map=str(self.get_parameter("factory_active_map").value),
            map_archive_dir=str(self.map_archive_dir),
        )

    def _run_configured_command(self, param_name: str, context: Dict[str, str]) -> Dict[str, Any]:
        template = str(self.get_parameter(param_name).value or "").strip()
        if not template:
            return {
                "ok": False,
                "manual_required": True,
                "message": "该步骤的 106 原厂命令还没有配置。请先用手柄/官方工具完成该步骤，再执行拉取地图。",
                "command_parameter": param_name,
            }
        try:
            command = template.format(**context)
        except Exception as exc:
            return self._error("建图命令模板格式错误", {"error": str(exc), "template": template})
        timeout = float(self.get_parameter("mapping_command_timeout_s").value)
        try:
            result = subprocess.run(
                command,
                shell=True,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=timeout,
                check=False,
            )
        except Exception as exc:
            return self._error("建图命令执行失败", {"error": str(exc), "command": command})
        if result.returncode != 0:
            return self._error(
                "建图命令返回失败",
                {"command": command, "returncode": result.returncode, "output": result.stdout},
            )
        return {"ok": True, "command": command, "output": result.stdout}

    def _map_file_snapshot(self, map_id: Optional[str]) -> Dict[str, Any]:
        with self._data_lock:
            if not map_id:
                map_id = self._settings.get("selected_map_id")
            record = self._find_map_record_unlocked(map_id)
        if record is None:
            return {"available": False, "message": "map not selected"}
        yaml_path = FsPath(self._resolve_path(str(record.get("yaml_path") or "")))
        fingerprint = map_file_fingerprint(yaml_path)
        if fingerprint is None:
            return {"available": False, "map_id": map_id, "message": f"map yaml not found: {yaml_path}"}
        cache_key = self._map_cache_key(map_id, yaml_path)
        with self._map_file_cache_lock:
            cached = self._map_file_cache.get(cache_key)
            if cached and cached.get("fingerprint") == fingerprint:
                return dict(cached["payload"])
        started = time.monotonic()
        try:
            payload = load_map_file_payload(record, yaml_path)
        except Exception as exc:
            payload = {
                "available": False,
                "map_id": map_id,
                "message": str(exc),
            }
        elapsed_s = time.monotonic() - started
        self._log_slow_operation(
            "load_map_file_payload",
            elapsed_s,
            "map_id=%s yaml=%s" % (str(map_id or ""), str(yaml_path)),
        )
        with self._map_file_cache_lock:
            self._map_file_cache[cache_key] = {
                "fingerprint": fingerprint,
                "payload": dict(payload),
            }
        return payload

    def _serve_mjpeg(self, camera_name: str, handler: BaseHTTPRequestHandler) -> None:
        if not self._as_bool(self.get_parameter("enable_camera_proxy").value):
            handler.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "camera proxy disabled")
            return
        if self._camera_proxy_backend() == "opencv" and get_cv2() is None:
            detail = _CV2_IMPORT_ERROR or "python3-opencv is not installed"
            handler.send_error(HTTPStatus.SERVICE_UNAVAILABLE, f"OpenCV unavailable: {detail}")
            return

        worker = self._camera_worker(camera_name)
        worker.acquire_client()
        frame_timeout = max(0.5, float(self.get_parameter("camera_proxy_frame_timeout_s").value))

        try:
            self._configure_mjpeg_socket(handler)
            handler.send_response(HTTPStatus.OK)
            handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            handler.send_header("Pragma", "no-cache")
            handler.send_header("Expires", "0")
            handler.send_header("X-Accel-Buffering", "no")
            handler.send_header("Connection", "close")
            handler.send_header("Access-Control-Allow-Origin", "*")
            handler.end_headers()

            last_sequence = -1
            while True:
                sequence, payload, stamp, error = worker.wait_for_frame(last_sequence, frame_timeout)
                if payload is None or sequence == last_sequence:
                    if error:
                        self.get_logger().debug(f"{camera_name} camera waiting for frame: {error}")
                    continue
                last_sequence = sequence
                handler.wfile.write(b"--frame\r\n")
                handler.wfile.write(b"Content-Type: image/jpeg\r\n")
                handler.wfile.write(b"Cache-Control: no-store\r\n")
                handler.wfile.write(f"X-M20Pro-Frame-Seq: {sequence}\r\n".encode("ascii"))
                handler.wfile.write(f"X-M20Pro-Frame-Age-Ms: {max(0.0, (time.time() - stamp) * 1000.0):.1f}\r\n".encode("ascii"))
                handler.wfile.write(f"Content-Length: {len(payload)}\r\n\r\n".encode("ascii"))
                handler.wfile.write(payload)
                handler.wfile.write(b"\r\n")
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self.get_logger().warning(f"{camera_name} camera MJPEG proxy stopped: {exc}")
        finally:
            worker.release_client()

    def _serve_jpeg_snapshot(self, camera_name: str, handler: BaseHTTPRequestHandler) -> None:
        if not self._as_bool(self.get_parameter("enable_camera_proxy").value):
            handler.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "camera proxy disabled")
            return
        if self._camera_proxy_backend() == "opencv" and get_cv2() is None:
            detail = _CV2_IMPORT_ERROR or "python3-opencv is not installed"
            handler.send_error(HTTPStatus.SERVICE_UNAVAILABLE, f"OpenCV unavailable: {detail}")
            return

        worker = self._camera_worker(camera_name)
        keepalive_s = max(0.0, float(self.get_parameter("camera_proxy_snapshot_keepalive_s").value))
        frame_timeout = max(0.5, float(self.get_parameter("camera_proxy_frame_timeout_s").value))
        worker.extend_snapshot_lease(max(keepalive_s, frame_timeout + 0.5))
        last_sequence, last_stamp = worker.current_frame_state()
        last_sequence = last_sequence if last_stamp > 0.0 else -1

        try:
            sequence, payload, stamp, error = worker.wait_for_frame(last_sequence, frame_timeout)
            if payload is None or sequence == last_sequence:
                detail = error or "camera frame timeout"
                status = HTTPStatus.SERVICE_UNAVAILABLE if error else HTTPStatus.GATEWAY_TIMEOUT
                handler.send_error(status, f"{camera_name} camera unavailable: {detail}")
                return

            self._configure_mjpeg_socket(handler)
            handler.send_response(HTTPStatus.OK)
            handler.send_header("Content-Type", "image/jpeg")
            handler.send_header("Content-Length", str(len(payload)))
            handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            handler.send_header("Pragma", "no-cache")
            handler.send_header("Expires", "0")
            handler.send_header("X-Accel-Buffering", "no")
            handler.send_header("X-M20Pro-Frame-Seq", str(sequence))
            handler.send_header("X-M20Pro-Frame-Age-Ms", f"{max(0.0, (time.time() - stamp) * 1000.0):.1f}")
            handler.send_header("Access-Control-Allow-Origin", "*")
            handler.end_headers()
            handler.wfile.write(payload)
            handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self.get_logger().warning(f"{camera_name} camera JPEG snapshot stopped: {exc}")

    def _configure_mjpeg_socket(self, handler: BaseHTTPRequestHandler) -> None:
        if not self._as_bool(self.get_parameter("camera_proxy_low_latency").value):
            return
        connection = getattr(handler, "connection", None)
        if connection is None:
            return
        try:
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        try:
            send_buffer = int(self.get_parameter("camera_proxy_socket_send_buffer_bytes").value)
        except (TypeError, ValueError):
            send_buffer = 0
        if send_buffer > 0:
            try:
                connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, send_buffer)
            except OSError:
                pass

    def _camera_worker(self, camera_name: str) -> _CameraProxyWorker:
        if camera_name == "rear":
            url = str(self.get_parameter("rear_camera_url").value)
        else:
            camera_name = "front"
            url = str(self.get_parameter("front_camera_url").value)

        worker = self._camera_workers.get(camera_name)
        if worker is not None and worker.url == url and worker.backend == self._camera_proxy_backend():
            worker.start()
            return worker
        if worker is not None:
            worker.stop()
        worker = _CameraProxyWorker(self, camera_name, url)
        self._camera_workers[camera_name] = worker
        worker.start()
        return worker

    @staticmethod
    def _as_bool(value: Any) -> bool:
        return as_bool(value)

    @staticmethod
    def _error(message: str, extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        return api_error_payload(message, extra)

    def _start_http_server(self) -> _ReusableThreadingHTTPServer:
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        node = self

        class DashboardHandler(BaseHTTPRequestHandler):
            def do_OPTIONS(self) -> None:
                self.send_response(HTTPStatus.NO_CONTENT)
                self._send_common_headers("application/json; charset=utf-8", 0)

            def do_GET(self) -> None:
                started = time.monotonic()
                parsed = urlparse(self.path)
                query = parse_qs(parsed.query)
                try:
                    if parsed.path == "/":
                        self._send_bytes(_load_dashboard_html(), "text/html; charset=utf-8")
                    elif parsed.path in DASHBOARD_STATIC_FILES:
                        static_payload = _load_dashboard_static(parsed.path)
                        if static_payload is None:
                            self.send_error(HTTPStatus.NOT_FOUND)
                        else:
                            payload, content_type = static_payload
                            self._send_bytes(payload, content_type)
                    elif parsed.path == "/api/state":
                        include_debug = node._as_bool((query.get("debug") or ["1"])[0])
                        self._send_json(node._snapshot(include_debug=include_debug))
                    elif parsed.path == "/api/map":
                        self._send_json(node._map_snapshot())
                    elif parsed.path == "/api/map_file":
                        map_id = (query.get("map_id") or [None])[0]
                        self._send_json(node._map_file_snapshot(map_id))
                    elif parsed.path == "/api/projects":
                        self._send_json(node._projects_payload())
                    elif parsed.path == "/api/maps":
                        self._send_json(node._maps_payload())
                    elif parsed.path == "/api/annotations":
                        self._send_json(node._annotations_payload(query))
                    elif parsed.path == "/api/tasks":
                        self._send_json(node._tasks_payload(query))
                    elif parsed.path == "/api/preflight":
                        self._send_json(node._preflight_payload())
                    elif parsed.path in ("/camera/front.mjpg", "/camera/rear.mjpg"):
                        camera_name = "front" if parsed.path == "/camera/front.mjpg" else "rear"
                        node._serve_mjpeg(camera_name, self)
                    elif parsed.path in ("/camera/front.jpg", "/camera/rear.jpg"):
                        camera_name = "front" if parsed.path == "/camera/front.jpg" else "rear"
                        node._serve_jpeg_snapshot(camera_name, self)
                    elif parsed.path == "/healthz":
                        self._send_json({"ok": True})
                    else:
                        self.send_error(HTTPStatus.NOT_FOUND)
                finally:
                    node._log_slow_operation("GET %s" % parsed.path, time.monotonic() - started)

            def do_POST(self) -> None:
                started = time.monotonic()
                parsed = urlparse(self.path)
                payload = self._read_json_body()
                try:
                    if parsed.path == "/api/projects":
                        self._send_api(node._create_project(payload))
                    elif parsed.path == "/api/maps/select":
                        self._send_api(node._select_map(payload))
                    elif parsed.path == "/api/mapping/session":
                        self._send_api(node._create_mapping_session(payload))
                    elif parsed.path == "/api/mapping/check_environment":
                        self._send_api(node._check_mapping_environment())
                    elif parsed.path == "/api/mapping/start":
                        self._send_api(node._mapping_command("factory_mapping_start_command", payload.get("session_id")))
                    elif parsed.path == "/api/mapping/finish":
                        self._send_api(node._mapping_command("factory_mapping_finish_command", payload.get("session_id")))
                    elif parsed.path == "/api/mapping/cancel":
                        self._send_api(node._mapping_command("factory_mapping_cancel_command", payload.get("session_id")))
                    elif parsed.path == "/api/mapping/import_active_map":
                        self._send_api(node._import_active_map(payload))
                    elif parsed.path == "/api/annotations":
                        self._send_api(node._create_annotation(payload))
                    elif parsed.path == "/api/tasks":
                        self._send_api(node._create_task(payload))
                    elif parsed.path == "/api/tasks/update":
                        self._send_api(node._update_task(payload))
                    elif parsed.path == "/api/tasks/start":
                        self._send_api(node._start_task(payload))
                    elif parsed.path == "/api/tasks/stop":
                        self._send_api(node._stop_task(payload))
                    elif parsed.path == "/api/preflight/run":
                        self._send_api(node._run_preflight(payload))
                    elif parsed.path == "/api/localization/initialpose":
                        self._send_api(node._publish_initialpose(payload))
                    else:
                        self.send_error(HTTPStatus.NOT_FOUND)
                finally:
                    node._log_slow_operation("POST %s" % parsed.path, time.monotonic() - started)

            def do_DELETE(self) -> None:
                started = time.monotonic()
                parsed = urlparse(self.path)
                query = parse_qs(parsed.query)
                try:
                    if parsed.path == "/api/annotations":
                        annotation_id = (query.get("id") or [""])[0]
                        self._send_api(node._delete_annotation(annotation_id))
                    elif parsed.path == "/api/tasks":
                        task_id = (query.get("id") or [""])[0]
                        self._send_api(node._delete_task(task_id))
                    else:
                        self.send_error(HTTPStatus.NOT_FOUND)
                finally:
                    node._log_slow_operation("DELETE %s" % parsed.path, time.monotonic() - started)

            def log_message(self, fmt: str, *args: Any) -> None:
                node.get_logger().debug(fmt % args)

            def _read_json_body(self) -> Dict[str, Any]:
                length = int(self.headers.get("Content-Length") or 0)
                if length <= 0:
                    return {}
                raw = self.rfile.read(length)
                try:
                    data = json.loads(raw.decode("utf-8"))
                    return data if isinstance(data, dict) else {}
                except Exception:
                    return {}

            def _send_api(self, payload: Dict[str, Any]) -> None:
                status = HTTPStatus.OK if payload.get("ok", False) else HTTPStatus.BAD_REQUEST
                if payload.get("manual_required"):
                    status = HTTPStatus.OK
                self._send_json(payload, status=status)

            def _send_json(self, payload: Dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
                self._send_bytes(
                    json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
                    "application/json; charset=utf-8",
                    status=status,
                )

            def _send_bytes(
                self,
                payload: bytes,
                content_type: str,
                status: HTTPStatus = HTTPStatus.OK,
            ) -> None:
                self.send_response(status)
                self._send_common_headers(content_type, len(payload))
                self.wfile.write(payload)

            def _send_common_headers(self, content_type: str, length: int) -> None:
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(length))
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET,POST,DELETE,OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()

        server = _ReusableThreadingHTTPServer((host, port), DashboardHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.get_logger().info(f"M20Pro web console listening on http://{host}:{port}")
        self.get_logger().info(f"web data dir: {self.data_dir}; map archive dir: {self.map_archive_dir}")
        return server

    def destroy_node(self) -> bool:
        for worker in list(getattr(self, "_camera_workers", {}).values()):
            worker.stop()
        self._camera_workers.clear()
        if hasattr(self, "_server"):
            self._server.shutdown()
            self._server.server_close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = WebDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except RCLError:
            pass


if __name__ == "__main__":
    main()
