import csv
from concurrent.futures import ThreadPoolExecutor
import io
import json
import math
import os
import select
import signal
import shlex
import shutil
import socket
import subprocess
import threading
import time
import tarfile
import yaml
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FsPath
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, quote, urlparse

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray
from m20pro_navigation.command_mux_contract import (
    normalized_teleop_command,
    teleop_release_decision,
)

from .battery_contract import battery_pack_present
from .active_task_contract import (
    advance_active_task_state,
    active_annotation_from_list,
    active_annotation_missing_failure,
    active_task_failure_payload,
    append_active_task_timeline_event_state,
    begin_waypoint_dwell_state,
    apply_connector_status_state,
    connector_owns_navigation_status,
    create_active_task_state,
    dwell_tick_decision,
    fail_active_task_state,
    goal_dispatch_decision,
    idle_stop_task_response,
    mark_connector_started_state,
    mark_floor_goal_published_state,
    mark_active_task_waiting_state,
    normalize_stop_task_request,
    prepare_goal_send_state,
    resolve_runtime_floor_goal,
    task_uses_multiple_floors,
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
    update_annotation_record,
    normalize_annotation_semantics,
    resolve_annotation_dwell_s,
)
from .floor_identity_contract import (
    augment_floor_config,
    configured_floor_ids,
    normalize_floor_id,
    resolve_operational_floor,
    validate_floor_matches_map,
    validate_mapping_session_identity,
    validate_registered_floor,
    validate_runtime_map_floor,
)
from .floor_route_contract import (
    floor_route_public_payload,
    remove_floor_route,
    resolve_floor_switch_request,
    runtime_floor_config,
    upsert_floor_route,
    validate_floor_route,
    validate_floor_route_set,
)
from .floor_switch_transaction_contract import (
    advance_transaction,
    begin_transaction,
    completion_decision,
    next_map_epoch,
    recover_interrupted_transaction,
    request_admission,
)
from .localization_contract import (
    factory_localization_ok_from_sources,
    initialpose_api_response_payload,
    localization_status_payload,
    map_relocalization_clearance_payload,
    manual_relocalization_verification_payload,
    parse_initialpose_request,
    relocalization_sample_evidence,
    relocalization_response_payload,
    relocalization_stability_step,
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
    apply_map_delete_state,
    apply_map_cell_edits,
    build_imported_map_record,
    default_map_id,
    ensure_map_yaml_uses_local_image,
    find_map_record,
    find_map_yaml,
    load_builtin_maps_from_manifest,
    load_map_file_payload,
    map_file_fingerprint,
    map_file_metadata_payload,
    removable_map_archive_directory,
)
from .map_identity_contract import occupancy_grid_content_digest
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
    mark_mapping_floor_imported,
    mapping_command_context,
    mapping_start_precondition,
    prepare_mapping_session_create,
    select_mapping_floor,
)
from .multi_floor_contract import (
    build_multi_floor_workspace,
    cross_floor_task_context,
    stair_routes_from_config,
)
from .unified_navigation_contract import (
    build_unified_navigation_plan,
    navigation_plan_record,
    runtime_transition_for_annotation,
    summarize_plan,
    task_navigation_plan_state,
)
from m20pro_navigation.stair_executor_contract import connector_route_activation_decision
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
    local_costmap_odom_alignment_payload,
    navigation_readiness_payload,
)
from .pcd_derived import process_imported_map
from .perception_contract import perception_status_payload
from .path_display_contract import path_points_in_map_frame
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
from .radar_result_contract import radar_job_matches_query
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
    pose_distance_m,
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
    task_start_localization_gate_decision,
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
M20PRO_WS_DIR = FsPath(os.environ.get("M20PRO_WS") or FsPath.cwd())
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
        self._goal_dispatch_lock = threading.Lock()
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
        self._floor_routes = self._load_json("floor_routes.json", [])
        self._route_config_floor_ids = self._configured_route_floor_ids()
        self._settings = self._load_json(
            "settings.json",
            {
                "selected_map_id": None,
                "working_map_id": None,
                "active_task": None,
                "floor_switch_map_epoch": 0,
                "floor_switch_transaction": None,
            },
        )
        self._normalize_runtime_state_on_startup()
        self._mapping_processes: Dict[str, Dict[str, Any]] = {}
        self._camera_workers: Dict[str, _CameraProxyWorker] = {}
        self._last_preflight: Optional[Dict[str, Any]] = None
        self._preflight_lock = threading.Lock()
        self._preflight_run_lock = threading.Lock()
        self._preflight_running: Optional[Dict[str, Any]] = None
        self._auto_preflight_timer = None
        self._auto_preflight_started_monotonic = time.monotonic()
        self._last_auto_preflight_monotonic = 0.0
        self._recording_lock = threading.RLock()
        self._recording_process: Optional[subprocess.Popen] = None
        self._recording_state: Optional[Dict[str, Any]] = None
        self._recording_log_handle = None
        self._map_file_cache_lock = threading.Lock()
        self._map_file_cache: Dict[str, Dict[str, Any]] = {}
        self._map_file_summary_cache: Dict[str, Dict[str, Any]] = {}
        self._last_scan_overlay_update = 0.0
        self._last_scan_overlay_valid_update = 0.0
        self._last_scan_overlay_points: List[Dict[str, float]] = []
        self._startup_map_sync_timer = None
        self._startup_map_sync_attempts = 0
        self._startup_map_sync_lock = threading.Lock()
        self._startup_map_sync_inflight = False
        self._floor_switch_lock = threading.Lock()
        self._floor_switch_inflight: Optional[str] = None
        self._teleop_lock = threading.RLock()
        self._teleop_acquiring = False
        self._teleop_session: Dict[str, Any] = {
            "active": False,
            "session_id": None,
            "started_at": None,
            "last_heartbeat_monotonic": None,
            "last_sequence": -1,
            "command": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
            "last_stop_reason": "not_started",
        }
        self._motion_state_result_condition = threading.Condition()
        self._motion_state_result_seq = 0
        self._motion_state_command_lock = threading.Lock()

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
        self.stair_executor_start_pub = self.create_publisher(
            String,
            str(self.get_parameter("stair_executor_start_topic").value),
            10,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            10,
        )
        self.teleop_cmd_vel_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("teleop_cmd_vel_topic").value),
            10,
        )
        self.motion_state_cmd_pub = self.create_publisher(
            String,
            str(self.get_parameter("motion_state_command_topic").value),
            10,
        )
        self.charge_command_pub = self.create_publisher(
            String,
            str(self.get_parameter("charge_command_topic").value),
            10,
        )
        self.command_mux_mode_pub = self.create_publisher(
            String,
            str(self.get_parameter("command_mux_mode_request_topic").value),
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
        self.floor_switch_result_pub = self.create_publisher(
            String,
            str(self.get_parameter("floor_switch_result_topic").value),
            10,
        )
        self.floor_context_pub = self.create_publisher(
            String,
            str(self.get_parameter("floor_context_topic").value),
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
        self.inspection_control_client = self.create_client(
            SetBool,
            str(self.get_parameter("inspection_control_service").value),
        )
        self.command_mux_clients = {
            "locked": self.create_client(
                Trigger, str(self.get_parameter("command_mux_lock_service").value)
            ),
            "navigation": self.create_client(
                Trigger, str(self.get_parameter("command_mux_navigation_service").value)
            ),
            "teleop": self.create_client(
                Trigger, str(self.get_parameter("command_mux_teleop_service").value)
            ),
        }

        self._state: Dict[str, Any] = {
            "floor": None,
            "stair_status": None,
            "stair_executor_status": None,
            "terrain_guard": None,
            "command_mux_status": None,
            "gait_command": None,
            "gait_result": None,
            "motion_state_result": None,
            "charge_command_result": None,
            "motion_state": None,
            "usage_mode_result": None,
            "localization_ok": None,
            "navigation_status": None,
            "navigation_status_parsed": None,
            "battery": None,
            "pose": None,
            "path": {"version": 0, "points": []},
            "local_path": {"version": 0, "points": []},
            "pose_history": [],
            "map": None,
            "map_version": 0,
            "dynamic_obstacles": [],
            "detections": None,
            "inspection_status": None,
            "radar_inspection": None,
            "radar_inspection_results": {},
            "active_waypoint": None,
            "relocalization_result": None,
            "relocalization_attempt": None,
            "map_relocalization_required": None,
            "events": [],
            "topics": {},
        }

        self._create_subscriptions()
        self.create_timer(1.0, self._tick_active_task)
        teleop_watchdog_period = max(
            0.05,
            min(0.15, float(self.get_parameter("teleop_command_timeout_s").value) / 3.0),
        )
        self.create_timer(teleop_watchdog_period, self._tick_teleop_lease)
        self.create_timer(2.0, self._publish_selected_stair_zones)
        self._server = self._start_http_server()
        if bool(self.get_parameter("auto_preflight_enabled").value):
            self._auto_preflight_timer = self.create_timer(1.0, self._tick_auto_preflight)
        if bool(self.get_parameter("startup_sync_selected_map_to_nav2").value):
            delay_s = max(0.1, float(self.get_parameter("startup_sync_selected_map_delay_s").value))
            self._startup_map_sync_timer = self.create_timer(delay_s, self._sync_selected_map_to_nav2_on_startup)

    def _declare_parameters(self) -> None:
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("data_dir", "~/.m20pro_web")
        self.declare_parameter("map_archive_dir", "~/m20pro_maps")
        self.declare_parameter("map_manifest", "")
        self.declare_parameter("floor_config", "")
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
        self.declare_parameter("floor_switch_request_topic", "/m20pro/floor_switch_request")
        self.declare_parameter("floor_switch_result_topic", "/m20pro/floor_switch_result")
        self.declare_parameter("floor_context_topic", "/m20pro/set_current_floor")
        self.declare_parameter("cross_floor_factory_apply_timeout_s", 30.0)
        self.declare_parameter("cross_floor_platform_position_tolerance_m", 0.50)
        self.declare_parameter("cross_floor_platform_yaw_tolerance_rad", 0.45)
        # Production launches override this from navigation.goal.xy_tolerance_m;
        # keep the fallback aligned with the checked-in field profile.
        self.declare_parameter("goal_reached_tolerance_m", 0.35)
        self.declare_parameter("task_goal_resend_interval_s", 5.0)
        self.declare_parameter("task_goal_accept_timeout_s", 12.0)
        self.declare_parameter("task_waypoint_timeout_s", 180.0)
        self.declare_parameter("task_progress_min_pose_movement_m", 0.08)
        self.declare_parameter("task_progress_min_distance_delta_m", 0.12)
        self.declare_parameter("task_timeline_max_events", 80)
        self.declare_parameter("task_start_settle_s", 0.5)
        self.declare_parameter("task_start_pose_timeout_s", 3.0)
        self.declare_parameter("task_start_costmap_odom_tolerance_m", 0.75)
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
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("teleop_cmd_vel_topic", "/cmd_vel_teleop")
        self.declare_parameter("command_mux_mode_request_topic", "/m20pro/cmd_vel_mux/mode_request")
        self.declare_parameter("command_mux_status_topic", "/m20pro/cmd_vel_mux/status")
        self.declare_parameter("command_mux_lock_service", "/m20pro/cmd_vel_mux/lock")
        self.declare_parameter(
            "command_mux_navigation_service", "/m20pro/cmd_vel_mux/enable_navigation"
        )
        self.declare_parameter("command_mux_teleop_service", "/m20pro/cmd_vel_mux/enable_teleop")
        self.declare_parameter("teleop_command_timeout_s", 0.8)
        self.declare_parameter("teleop_max_forward_speed_mps", 0.18)
        self.declare_parameter("teleop_max_reverse_speed_mps", 0.12)
        self.declare_parameter("teleop_max_lateral_speed_mps", 0.18)
        self.declare_parameter("teleop_max_angular_speed_radps", 0.45)
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
        self.declare_parameter("stair_executor_start_topic", "/m20pro/stair_executor/start")
        self.declare_parameter(
            "stair_executor_status_topic", "/m20pro/stair_executor/status"
        )
        self.declare_parameter("terrain_guard_status_topic", "/m20pro/terrain_guard/status")
        self.declare_parameter("terrain_guard_status_timeout_s", 1.0)
        self.declare_parameter("gait_command_topic", "/m20pro/gait_command")
        self.declare_parameter("gait_result_topic", "/m20pro_tcp_bridge/gait_result")
        self.declare_parameter("motion_state_command_topic", "/m20pro/motion_state_command")
        self.declare_parameter("motion_state_result_topic", "/m20pro_tcp_bridge/motion_state_result")
        self.declare_parameter("motion_state_discovery_timeout_s", 2.0)
        self.declare_parameter("motion_state_result_timeout_s", 3.0)
        self.declare_parameter("charge_command_topic", "/m20pro/charge_command")
        self.declare_parameter("charge_result_topic", "/m20pro_tcp_bridge/charge_result")
        self.declare_parameter("charge_command_discovery_timeout_s", 2.0)
        self.declare_parameter("charge_command_timeout_s", 8.0)
        self.declare_parameter("usage_mode_result_topic", "/m20pro_tcp_bridge/usage_mode_result")
        self.declare_parameter("localization_ok_topic", "/m20pro_tcp_bridge/localization_ok")
        self.declare_parameter("navigation_status_topic", "/m20pro_tcp_bridge/navigation_status")
        self.declare_parameter("battery_topic", "/BATTERY_DATA")
        self.declare_parameter("motion_state_topic", "/m20pro_tcp_bridge/motion_state")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_overlay_max_points", 720)
        self.declare_parameter("scan_overlay_update_min_interval_s", 0.1)
        self.declare_parameter("scan_overlay_hold_s", 0.5)
        self.declare_parameter("scan_overlay_min_range_m", 0.05)
        self.declare_parameter("scan_overlay_max_range_m", 30.0)
        self.declare_parameter("scan_overlay_offset_x_m", 0.0)
        self.declare_parameter("scan_overlay_offset_y_m", 0.0)
        self.declare_parameter("scan_overlay_offset_yaw_rad", 0.0)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_topic", "/m20pro_tcp_bridge/map_pose")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("local_plan_topic", "/local_plan")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("local_costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("global_costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("local_costmap_updates_topic", "/local_costmap/costmap_updates")
        self.declare_parameter("global_costmap_updates_topic", "/global_costmap/costmap_updates")
        self.declare_parameter("dynamic_obstacle_topic", "/dynamic_obstacle_markers")
        self.declare_parameter("relocalization_result_topic", "/m20pro_tcp_bridge/relocalization_result")
        self.declare_parameter("detections_topic", "/m20pro_yolov8_inspection/detections")
        self.declare_parameter("events_topic", "/m20pro_yolov8_inspection/events")
        self.declare_parameter("inspection_status_topic", "/m20pro_yolov8_inspection/status")
        self.declare_parameter("inspection_control_service", "/m20pro_yolov8_inspection/set_enabled")
        self.declare_parameter("inspection_control_timeout_s", 20.0)
        self.declare_parameter("radar_inspection_status_topic", "/m20pro/radar_inspection/status")
        self.declare_parameter("radar_inspection_result_topic", "/m20pro/radar_inspection/result")
        self.declare_parameter("radar_inspection_events_topic", "/m20pro/radar_inspection/events")
        self.declare_parameter("wait_for_radar_inspection", False)
        self.declare_parameter("radar_inspection_timeout_s", 1800.0)
        self.declare_parameter("advance_on_radar_scan_release", True)
        self.declare_parameter("radar_results_dir", "~/.m20pro_radar_results")
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
        self.declare_parameter("auto_preflight_enabled", True)
        self.declare_parameter("auto_preflight_start_delay_s", 12.0)
        self.declare_parameter("auto_preflight_interval_s", 300.0)

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
        hidden = {
            str(item or "").strip()
            for item in (self._settings.get("hidden_builtin_map_ids") or [])
            if str(item or "").strip()
        }
        builtin_maps = [item for item in self._builtin_maps if str(item.get("id") or "") not in hidden]
        return all_map_records(builtin_maps, self._maps)

    def _find_map_record_unlocked(self, map_id: Optional[str]) -> Optional[Dict[str, Any]]:
        target = str(map_id or "").strip()
        if any(str(item.get("id") or "") == target for item in self._maps):
            return find_map_record([], self._maps, target)
        hidden = {
            str(item or "").strip()
            for item in (self._settings.get("hidden_builtin_map_ids") or [])
            if str(item or "").strip()
        }
        return None if target in hidden else find_map_record(self._builtin_maps, [], target)

    def _default_map_id_unlocked(self) -> Optional[str]:
        visible_builtin_ids = {
            str(item.get("id") or "")
            for item in self._all_maps_unlocked()
            if str(item.get("source") or "") == "project_builtin"
        }
        builtin_maps = [item for item in self._builtin_maps if str(item.get("id") or "") in visible_builtin_ids]
        default_builtin = self._default_builtin_map_id if self._default_builtin_map_id in visible_builtin_ids else None
        return default_map_id(builtin_maps, self._maps, default_builtin)

    def _normalize_runtime_state_on_startup(self) -> None:
        changed = False
        recovered_transaction = recover_interrupted_transaction(
            self._settings.get("floor_switch_transaction"),
            now_text=now_text(),
        )
        if recovered_transaction.get("changed"):
            self._settings["floor_switch_transaction"] = recovered_transaction["transaction"]
            active_after_restart = self._settings.get("active_task")
            if (
                isinstance(active_after_restart, dict)
                and str(active_after_restart.get("status") or "") == "running"
                and bool(active_after_restart.get("multi_floor"))
            ):
                failed_task = dict(active_after_restart)
                failed_task.update(
                    {
                        "status": "failed",
                        "status_message": "地图切换期间服务重启，任务已停止；请确认当前地图并重新定位",
                        "failure_code": "floor_switch_interrupted_restart",
                        "updated_at": now_text(),
                    }
                )
                self._settings["active_task"] = failed_task
            changed = True
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
                self._state["relocalization_attempt"] = None
                self._state["pose_history"] = []
                self._state["path"] = {"version": int(self._state.get("path", {}).get("version", 0) or 0) + 1, "points": []}
                self._state["local_path"] = {"version": int(self._state.get("local_path", {}).get("version", 0) or 0) + 1, "points": []}
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
        self.create_subscription(
            String,
            self._topic("floor_switch_request_topic"),
            self._on_floor_switch_request,
            10,
        )
        self.create_subscription(String, self._topic("stair_status_topic"), self._on_stair_status, 10)
        self.create_subscription(
            String,
            self._topic("stair_executor_status_topic"),
            self._on_stair_executor_status,
            10,
        )
        self.create_subscription(
            String,
            self._topic("terrain_guard_status_topic"),
            self._on_terrain_guard_status,
            10,
        )
        mux_status_qos = QoSProfile(depth=1)
        mux_status_qos.reliability = ReliabilityPolicy.RELIABLE
        mux_status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            self._topic("command_mux_status_topic"),
            self._on_command_mux_status,
            mux_status_qos,
        )
        self.create_subscription(String, self._topic("gait_command_topic"), self._on_gait_command, 10)
        self.create_subscription(String, self._topic("gait_result_topic"), self._on_gait_result, 10)
        self.create_subscription(String, self._topic("motion_state_result_topic"), self._on_motion_state_result, 10)
        self.create_subscription(String, self._topic("charge_result_topic"), self._on_charge_result, 10)
        self.create_subscription(String, self._topic("usage_mode_result_topic"), self._on_usage_mode_result, 10)
        self.create_subscription(Bool, self._topic("localization_ok_topic"), self._on_localization_ok, 10)
        self.create_subscription(String, self._topic("navigation_status_topic"), self._on_navigation_status, 10)
        if BatteryData is not None:
            self.create_subscription(BatteryData, self._topic("battery_topic"), self._on_battery, 10)
        else:
            self.get_logger().warning("drdds.msg.BatteryData is unavailable; battery display is disabled")
        self.create_subscription(Int32, self._topic("motion_state_topic"), self._on_motion_state, 10)
        self.create_subscription(LaserScan, self._topic("scan_topic"), self._on_scan, scan_qos)
        self.create_subscription(Odometry, self._topic("odom_topic"), self._on_odom, 10)
        self.create_subscription(PoseStamped, self._topic("pose_topic"), self._on_pose, 20)
        self.create_subscription(RosPath, self._topic("plan_topic"), self._on_path, 5)
        self.create_subscription(RosPath, self._topic("local_plan_topic"), self._on_local_path, 5)
        self.create_subscription(String, self._topic("active_waypoint_topic"), self._on_active_waypoint, 10)
        self.create_subscription(OccupancyGrid, self._topic("map_topic"), self._on_map, map_qos)
        self.create_subscription(OccupancyGrid, self._topic("local_costmap_topic"), self._on_local_costmap, 2)
        self.create_subscription(OccupancyGrid, self._topic("global_costmap_topic"), self._on_global_costmap, 2)
        self.create_subscription(
            OccupancyGridUpdate,
            self._topic("local_costmap_updates_topic"),
            self._on_local_costmap_update,
            10,
        )
        self.create_subscription(
            OccupancyGridUpdate,
            self._topic("global_costmap_updates_topic"),
            self._on_global_costmap_update,
            10,
        )
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
        self.create_subscription(
            String,
            self._topic("radar_inspection_status_topic"),
            self._on_radar_inspection_status,
            10,
        )
        self.create_subscription(
            String,
            self._topic("radar_inspection_result_topic"),
            self._on_radar_inspection_result,
            10,
        )
        self.create_subscription(
            String,
            self._topic("radar_inspection_events_topic"),
            self._on_event,
            10,
        )

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

    def _on_stair_executor_status(self, msg: String) -> None:
        parsed = parse_json_text(msg.data)
        with self._lock:
            self._state["stair_executor_status"] = {
                "last_update": time.time(),
                "raw": msg.data,
                "parsed": parsed,
            }
            self._mark_topic("stair_executor_status")
        if not isinstance(parsed, dict):
            return
        request_id = str(parsed.get("request_id") or "").strip()
        state = str(parsed.get("state") or "").strip().upper()
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            status_update = apply_connector_status_state(
                active,
                parsed,
                now_text=now_text(),
                now_monotonic=time.monotonic(),
            )
            if not status_update.get("matched"):
                return
            active = status_update["active"]
            self._settings["active_task"] = active
            if status_update.get("persistent_changed") and not status_update.get("terminal"):
                self._save_json("settings.json", self._settings)
        if status_update.get("failed"):
            self._fail_active_task(
                active.get("task_id"),
                str(parsed.get("message") or "楼梯连接边执行未完成，任务已停止"),
                {
                    "reason": str(parsed.get("code") or "stair_connector_failed"),
                    "connector_request_id": request_id,
                    "connector_state": state,
                },
            )
            return
        if not status_update.get("terminal"):
            return
        # A connector completion is the hand-off point back to the single
        # task executor.  Clearing only the dispatch marker lets the next tick
        # either send the final-floor goal or select the next ordered edge.
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") != "running":
                return
            active["connector_state"] = "COMPLETED"
            active["last_connector_request_id"] = request_id
            active.pop("connector_request_id", None)
            active["status_message"] = "楼梯连接边完成，继续统一导航计划"
            active["last_goal_annotation_id"] = None
            active["last_nav_goal_status"] = None
            active.pop("last_goal_attempt_id", None)
            self._append_active_task_timeline_event(
                active,
                "stair_connector_completed",
                active["status_message"],
                {
                    "connector_request_id": request_id,
                    "route_id": parsed.get("route_id"),
                    "target_floor": parsed.get("target_floor"),
                },
            )
            self._settings["active_task"] = active
            self._save_json("settings.json", self._settings)

    def _on_terrain_guard_status(self, msg: String) -> None:
        parsed = parse_json_text(msg.data)
        with self._lock:
            self._state["terrain_guard"] = {
                "last_update": time.time(),
                "raw": msg.data,
                "parsed": parsed,
            }
            self._mark_topic("terrain_guard_status")

    def _on_command_mux_status(self, msg: String) -> None:
        with self._lock:
            self._state["command_mux_status"] = {
                "last_update": time.time(),
                "raw": msg.data,
                "parsed": parse_json_text(msg.data),
            }
            self._mark_topic("command_mux_status")

    def _on_gait_command(self, msg: String) -> None:
        with self._lock:
            self._state["gait_command"] = msg.data
            self._mark_topic("gait_command")

    def _on_gait_result(self, msg: String) -> None:
        with self._lock:
            self._state["gait_result"] = msg.data
            self._mark_topic("gait_result")

    def _on_motion_state_result(self, msg: String) -> None:
        with self._lock:
            self._state["motion_state_result"] = msg.data
            self._mark_topic("motion_state_result")
        with self._motion_state_result_condition:
            self._motion_state_result_seq += 1
            self._motion_state_result_condition.notify_all()

    def _on_charge_result(self, msg: String) -> None:
        try:
            result = json.loads(msg.data)
        except (TypeError, ValueError):
            result = {"status": "failed", "message": msg.data, "error_code": -1}
        if not isinstance(result, dict):
            result = {"status": "failed", "message": "invalid charge result", "error_code": -1}
        with self._lock:
            self._state["charge_command_result"] = dict(result)
            self._state["charge_command_result"]["last_update"] = time.time()
            self._mark_topic("charge_result")

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
            if not battery_pack_present(item):
                continue
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

    def _on_motion_state(self, msg: Any) -> None:
        data = getattr(msg, "data", None)
        state_value = getattr(data, "state", data)
        try:
            state_value = int(state_value)
        except (TypeError, ValueError):
            state_value = None
        with self._lock:
            self._state["motion_state"] = {
                "state": state_value,
                "last_update": time.time(),
            }
            self._mark_topic("motion_state")

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
            if rebuilt_points:
                points = rebuilt_points
                self._last_scan_overlay_points = points
                self._last_scan_overlay_valid_update = now
            else:
                # A single LaserScan can legitimately contain no drawable
                # returns (packet loss, invalid ranges, or a reflective rear
                # sector). Keep the last valid contour briefly so the rear
                # outline does not blink out on every bad frame. The scan
                # health counters below still expose the current bad sample.
                hold_s = max(0.0, float(self.get_parameter("scan_overlay_hold_s").value))
                if now - self._last_scan_overlay_valid_update > hold_s:
                    points = []
                    self._last_scan_overlay_points = []
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
        self._store_path("path", msg, transform_to_map=False)

    def _on_local_path(self, msg: RosPath) -> None:
        self._store_path("local_path", msg, transform_to_map=True)

    def _store_path(self, state_key: str, msg: RosPath, *, transform_to_map: bool) -> None:
        max_points = max(2, int(self.get_parameter("max_path_points").value))
        raw_poses = list(msg.poses)
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
        source_frame = str(msg.header.frame_id or "").strip()
        transform_source = "identity"
        output_frame = source_frame
        if transform_to_map:
            with self._lock:
                map_pose = dict(self._state.get("pose") or {})
                odom_state = dict(self._state.get("odom") or {})
                odom_pose = dict(odom_state.get("pose") or {})
            transformed = path_points_in_map_frame(
                points,
                frame_id=source_frame,
                map_pose=map_pose,
                odom_pose=odom_pose,
            )
            if transformed is None:
                points = []
                transform_source = "unavailable"
            else:
                points = transformed
                transform_source = "map_pose_odom_alignment" if source_frame.lstrip("/") == "odom" else "identity"
                output_frame = "map"
        path_last_point = dict(points[-1]) if points else None
        with self._lock:
            current = dict(self._state.get(state_key) or {})
            self._state[state_key] = {
                "version": int(current.get("version", 0) or 0) + 1,
                "frame_id": output_frame,
                "source_frame_id": source_frame,
                "transform_source": transform_source,
                "points": points,
                "last_point": path_last_point,
                "point_count": len(points),
                "raw_point_count": len(raw_poses),
                "last_update": time.time(),
            }
            self._mark_topic(state_key)

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
        map_payload["content_digest"] = occupancy_grid_content_digest(map_payload)
        with self._lock:
            self._state["map"] = map_payload
            self._state["map_version"] = map_payload["version"]
            self._mark_topic("map")

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        info = msg.info
        origin = pose_to_dict(info.origin)
        with self._lock:
            self._state["local_costmap"] = {
                "last_update": time.time(),
                "stamp": stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "width": int(info.width),
                "height": int(info.height),
                "resolution": float(info.resolution),
                "origin": origin,
            }
            self._mark_topic("local_costmap")

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        info = msg.info
        origin = pose_to_dict(info.origin)
        with self._lock:
            self._state["global_costmap"] = {
                "last_update": time.time(),
                "stamp": stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "width": int(info.width),
                "height": int(info.height),
                "resolution": float(info.resolution),
                "origin": origin,
            }
            self._mark_topic("global_costmap")

    def _on_local_costmap_update(self, msg: OccupancyGridUpdate) -> None:
        self._on_costmap_update("local_costmap", msg)

    def _on_global_costmap_update(self, msg: OccupancyGridUpdate) -> None:
        self._on_costmap_update("global_costmap", msg)

    def _on_costmap_update(self, state_key: str, msg: OccupancyGridUpdate) -> None:
        with self._lock:
            payload = dict(self._state.get(state_key) or {})
            payload.update(
                {
                    "last_update": time.time(),
                    "stamp": stamp_to_float(msg.header.stamp),
                    "frame_id": msg.header.frame_id or payload.get("frame_id", ""),
                    "last_update_kind": "incremental",
                    "update_x": int(msg.x),
                    "update_y": int(msg.y),
                    "update_width": int(msg.width),
                    "update_height": int(msg.height),
                }
            )
            self._state[state_key] = payload
            self._mark_topic(state_key)

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

    def _on_radar_inspection_status(self, msg: String) -> None:
        parsed = parse_json_text(msg.data)
        with self._lock:
            self._state["radar_inspection"] = {
                "last_update": time.time(),
                "raw": msg.data,
                "parsed": parsed,
            }
            self._mark_topic("radar_inspection_status")

    def _on_radar_inspection_result(self, msg: String) -> None:
        parsed = parse_json_text(msg.data)
        key = ""
        if isinstance(parsed, dict):
            key = str(parsed.get("waypoint_key") or "").strip()
        with self._lock:
            if key:
                results = dict(self._state.get("radar_inspection_results") or {})
                results[key] = {
                    "last_update": time.time(),
                    "raw": msg.data,
                    "parsed": parsed,
                }
                self._state["radar_inspection_results"] = results
            self._state["radar_inspection"] = {
                "last_update": time.time(),
                "raw": msg.data,
                "parsed": parsed,
            }
            self._mark_topic("radar_inspection_result")

    def _on_relocalization_result(self, msg: String) -> None:
        now = time.time()
        with self._lock:
            self._state["relocalization_result"] = {
                "last_update": now,
                "raw": msg.data,
                "parsed": parse_json_text(msg.data),
            }
            attempt = self._state.get("relocalization_attempt")
            if isinstance(attempt, dict) and float(attempt.get("started_at", 0.0) or 0.0) <= now:
                attempt = dict(attempt)
                attempt["result"] = msg.data
                attempt["result_at"] = now
                self._state["relocalization_attempt"] = attempt
            self._mark_topic("relocalization_result")

    def _raw_factory_localization_ok(self, state: Dict[str, Any]) -> bool:
        nav_status_parsed = (
            state.get("navigation_status_parsed")
            if isinstance(state.get("navigation_status_parsed"), dict)
            else {}
        )
        if not nav_status_parsed and state.get("navigation_status") is not None:
            nav_status_parsed = parse_key_value_status(str(state.get("navigation_status") or ""))
        return factory_localization_ok_from_sources(state.get("localization_ok"), nav_status_parsed)

    def _factory_localization_ok(self, state: Dict[str, Any]) -> bool:
        # A failed or in-flight manual relocation is an explicit runtime gate.
        # Do not let the bridge's next healthy poll resurrect the old pose as
        # task-ready before a new relocation succeeds.
        attempt = state.get("relocalization_attempt")
        if isinstance(attempt, dict) and str(attempt.get("status") or "").strip().lower() in {
            "pending",
            "failed",
        }:
            return False
        return self._raw_factory_localization_ok(state)

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

    def _snapshot(self, include_debug: bool = True) -> Dict[str, Any]:
        now = time.time()
        with self._lock:
            snapshot = dict(self._state)
            snapshot["path"] = dict(self._state["path"])
            snapshot["local_path"] = dict(self._state["local_path"])
            snapshot["pose_history"] = list(self._state.get("pose_history") or [])
            snapshot["dynamic_obstacles"] = list(self._state["dynamic_obstacles"])
            snapshot["events"] = list(self._state["events"])
            for key in (
                "scan",
                "odom",
                "local_costmap",
                "global_costmap",
                "active_waypoint",
                "inspection_status",
                "radar_inspection",
                "command_mux_status",
            ):
                if isinstance(self._state.get(key), dict):
                    snapshot[key] = dict(self._state[key])
            if isinstance(self._state.get("relocalization_attempt"), dict):
                snapshot["relocalization_attempt"] = dict(self._state["relocalization_attempt"])
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
            transaction = self._settings.get("floor_switch_transaction")
            snapshot["floor_switch_transaction"] = (
                dict(transaction) if isinstance(transaction, dict) else None
            )
            snapshot["map_relocalization_required"] = self._settings.get("map_relocalization_required")
            snapshot["startup_map_sync"] = self._settings.get("startup_map_sync")
        reported_floor = snapshot.get("floor")
        snapshot["reported_floor"] = reported_floor
        snapshot["floor"] = self._operational_floor(reported_floor, snapshot.get("selected_map_id"))
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
            relocalization_attempt=snapshot.get("relocalization_attempt"),
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
        snapshot["inspection_control"] = self._inspection_control_status_payload(snapshot)
        snapshot["teleoperation"] = self._teleop_status_payload()
        for value in snapshot["topics"].values():
            last_update = value.get("last_update")
            value["age_sec"] = None if last_update is None else max(0.0, now - float(last_update))
        snapshot["debug_included"] = bool(include_debug)
        if not include_debug:
            snapshot["events"] = []
            snapshot["topics"] = {}
        return snapshot

    def _live_snapshot(
        self,
        *,
        path_version: Optional[int] = None,
        local_path_version: Optional[int] = None,
    ) -> Dict[str, Any]:
        now = time.time()
        with self._lock:
            pose = dict(self._state.get("pose") or {})
            scan = dict(self._state.get("scan") or {})
            active_waypoint = dict(self._state.get("active_waypoint") or {})
            path = dict(self._state.get("path") or {})
            local_path = dict(self._state.get("local_path") or {})
            payload = {
                "floor": self._state.get("floor"),
                "localization_ok": self._state.get("localization_ok"),
                "pose": pose or None,
                # LaserScan is already reduced to the bounded overlay point
                # set in _on_scan; carry that latest sample on the same fast
                # endpoint as pose/path instead of waiting for /api/state.
                "scan": scan or None,
                "active_waypoint": active_waypoint or None,
                "map_version": self._state.get("map_version", 0),
            }
        with self._data_lock:
            transaction = self._settings.get("floor_switch_transaction")
            payload["floor_switch_transaction"] = (
                dict(transaction) if isinstance(transaction, dict) else None
            )
        if path_version != int(path.get("version", 0) or 0):
            payload["path"] = path
        if local_path_version != int(local_path.get("version", 0) or 0):
            payload["local_path"] = local_path
        pose_age = pose_age_sec(pose, now)
        pose_timeout_s = max(0.5, float(self.get_parameter("task_start_pose_timeout_s").value))
        payload.update(
            {
                "ok": True,
                "node_time": now,
                "pose_age_sec": pose_age,
                "pose_timeout_s": pose_timeout_s,
                "pose_fresh": bool(
                    is_plausible_pose_dict(pose)
                    and pose_age is not None
                    and pose_age <= pose_timeout_s
                ),
            }
        )
        reported_floor = payload.get("floor")
        payload["reported_floor"] = reported_floor
        payload["floor"] = self._operational_floor(reported_floor)
        return payload

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

    def _inspection_control_status_payload(self, snapshot: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        source = snapshot if isinstance(snapshot, dict) else self._snapshot(include_debug=False)
        status_record = source.get("inspection_status")
        status = (
            status_record.get("parsed")
            if isinstance(status_record, dict) and isinstance(status_record.get("parsed"), dict)
            else {}
        )
        status_age = None
        if isinstance(status_record, dict):
            try:
                status_age = max(0.0, time.time() - float(status_record.get("last_update") or 0.0))
            except (TypeError, ValueError):
                status_age = None
        service_ready = bool(self.inspection_control_client.service_is_ready())
        # DDS service discovery can briefly lag behind the status topic after
        # the inspection node starts or releases RKNN. A fresh status proves
        # that the control node is present; POST still performs a bounded
        # wait_for_service before sending the actual command.
        available = service_ready or (status_age is not None and status_age <= 3.0)
        return {
            "available": available,
            "enabled": status.get("enabled") is True,
            "ready": status.get("ready") is True,
            "state": status.get("state") or ("unavailable" if not status else "unknown"),
            "backend": status.get("backend"),
            "message": status.get("last_error"),
        }

    def _inspection_live_payload(self) -> Dict[str, Any]:
        with self._lock:
            detections = self._state.get("detections")
            inspection_status = self._state.get("inspection_status")
            detections = dict(detections) if isinstance(detections, dict) else None
            inspection_status = (
                dict(inspection_status) if isinstance(inspection_status, dict) else None
            )
        return {
            "ok": True,
            "node_time": time.time(),
            "detections": detections,
            "inspection_status": inspection_status,
            "inspection_control": self._inspection_control_status_payload(
                {"inspection_status": inspection_status}
            ),
        }

    def _set_inspection_enabled(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if "enabled" not in payload:
            return self._error("缺少 enabled")
        enabled = self._as_bool(payload.get("enabled"))
        with self._lock:
            self._state["detections"] = None
        timeout_s = max(1.0, float(self.get_parameter("inspection_control_timeout_s").value))
        if not self.inspection_control_client.wait_for_service(timeout_sec=min(2.0, timeout_s)):
            return self._error(
                "YOLO 控制服务不可用",
                {"code": "inspection_control_unavailable", "enabled": enabled},
            )
        request = SetBool.Request()
        request.data = enabled
        future = self.inspection_control_client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return self._error(
                "YOLO 启停超时",
                {"code": "inspection_control_timeout", "enabled": enabled},
            )
        try:
            response = future.result()
        except Exception as exc:
            return self._error(
                "YOLO 启停失败",
                {"code": "inspection_control_failed", "enabled": enabled, "error": str(exc)},
            )
        if response is None or not bool(response.success):
            return self._error(
                "YOLO 启停失败",
                {
                    "code": "inspection_control_rejected",
                    "enabled": enabled,
                    "message": str(getattr(response, "message", "") or "节点拒绝请求"),
                },
            )
        return {
            "ok": True,
            "enabled": enabled,
            "message": str(response.message or ("YOLO 已启用" if enabled else "YOLO 已关闭")),
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
        raw_floors = payload.get("floors") or []
        if isinstance(raw_floors, str):
            raw_floors = raw_floors.split(",")
        floors = [normalize_floor_id(item) for item in raw_floors if str(item).strip()]
        if any(not item for item in floors):
            return self._error("项目楼层格式无效，请填写例如 7、F7 或 B1")
        project = {
            "id": new_id("project"),
            "name": name,
            "building": building,
            "floors": list(dict.fromkeys(floors)),
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

    def _floor_config_payload(self) -> Dict[str, Any]:
        path = self._floor_config_path()
        if path is None or not path.is_file():
            return {
                "ok": False,
                "config": {},
                "message": "运行时导航配置 runtime_navigation.yaml 不可用",
                "path": str(path or ""),
            }
        try:
            with path.open("r", encoding="utf-8") as file:
                base_config = yaml.safe_load(file) or {}
            if not isinstance(base_config, dict):
                raise ValueError("配置根节点必须是对象")
            with self._data_lock:
                projects = [dict(item) for item in self._projects]
                routes = [dict(item) for item in self._floor_routes]
            config = (
                runtime_floor_config(routes, mission=base_config.get("mission"))
                if routes
                else dict(base_config)
            )
            route_floors = configured_floor_ids(config)
            config = augment_floor_config(config, projects)
            return {
                "ok": True,
                "config": config,
                "path": str(path),
                "route_config_floors": route_floors,
                "dynamic_route_count": len(routes),
            }
        except Exception as exc:
            return {
                "ok": False,
                "config": {},
                "message": "读取跨楼层配置失败",
                "path": str(path),
                "error": str(exc),
            }

    def _static_route_floor_ids(self) -> set:
        path = self._floor_config_path()
        if path is None or not path.is_file():
            return set()
        try:
            with path.open("r", encoding="utf-8") as file:
                config = yaml.safe_load(file) or {}
        except Exception:
            return set()
        return set(configured_floor_ids(config if isinstance(config, dict) else {}))

    def _configured_route_floor_ids(self) -> set:
        if self._floor_routes:
            return set(configured_floor_ids(runtime_floor_config(self._floor_routes)))
        return self._static_route_floor_ids()

    def _operational_floor(self, reported_floor: Any, selected_map_id: Any = None) -> Any:
        reported = str(reported_floor or "").strip() or None
        with self._data_lock:
            map_id = str(selected_map_id or self._settings.get("selected_map_id") or "").strip()
            record = self._find_map_record_unlocked(map_id) if map_id else None
        return resolve_operational_floor(reported, record or {}, self._route_config_floor_ids)

    def _multi_floor_payload(self) -> Dict[str, Any]:
        config_payload = self._floor_config_payload()
        with self._lock:
            reported_floor = self._state.get("floor")
        with self._data_lock:
            maps = [dict(item) for item in self._all_maps_unlocked()]
            annotations = [dict(item) for item in self._annotations]
            sessions = [dict(item) for item in self._sessions]
            selected_map_id = self._settings.get("selected_map_id")
        current_floor = self._operational_floor(reported_floor, selected_map_id)
        workspace = build_multi_floor_workspace(
            floor_config=config_payload["config"],
            maps=maps,
            annotations=annotations,
            sessions=sessions,
            current_floor=current_floor,
            selected_map_id=selected_map_id,
        )
        workspace["config_available"] = bool(config_payload.get("ok"))
        workspace["config_path"] = config_payload.get("path")
        if not config_payload.get("ok"):
            workspace["ready"] = False
            workspace["message"] = config_payload.get("message")
            workspace["config_error"] = config_payload.get("error")
        return workspace

    def _route_map_yaml(self, record: Dict[str, Any]) -> str:
        value = str(record.get("yaml_path") or record.get("map_yaml") or "").strip()
        if not value:
            return ""
        try:
            path = FsPath(self._resolve_path(value))
        except Exception:
            return ""
        return str(path) if path.is_file() else ""

    def _floor_routes_payload(self) -> Dict[str, Any]:
        with self._data_lock:
            return floor_route_public_payload(
                self._floor_routes,
                annotations=self._annotations,
                maps=self._all_maps_unlocked(),
            )

    def _save_floor_route(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") == "running":
                return self._error("任务执行中不能修改跨楼层路线")
            maps = {str(item.get("id")): dict(item) for item in self._all_maps_unlocked() if item.get("id")}
            annotations = {
                str(item.get("id")): dict(item)
                for item in self._annotations
                if item.get("id")
            }
            existing = [dict(item) for item in self._floor_routes]
        route_id = str(payload.get("id") or "").strip() or new_id("floor_route")
        validated = validate_floor_route(
            payload,
            annotations_by_id=annotations,
            maps_by_id=maps,
            resolve_map_yaml=self._route_map_yaml,
            route_id=route_id,
            now_text=now_text(),
        )
        if not validated.get("ok"):
            return self._error(
                str(validated.get("message") or "楼梯路线无效"),
                {key: value for key, value in validated.items() if key not in ("ok", "message")},
            )
        route = dict(validated["route"])
        candidate_routes = upsert_floor_route(existing, route)
        route_set = validate_floor_route_set(candidate_routes)
        if not route_set.get("ok"):
            return self._error(
                str(route_set.get("message") or "跨楼层路线集合无效"),
                {key: value for key, value in route_set.items() if key not in ("ok", "message")},
            )
        with self._data_lock:
            self._floor_routes = candidate_routes
            self._save_json("floor_routes.json", self._floor_routes)
            self._route_config_floor_ids = self._configured_route_floor_ids()
        self._append_event(
            "跨楼层路线已保存",
            {
                "route_id": route["id"],
                "source_floor": route["source_floor"],
                "target_floor": route["target_floor"],
                "source_map_id": route["source_map_id"],
                "target_map_id": route["target_map_id"],
            },
        )
        return {"ok": True, "route": route, "message": "跨楼层路线已保存"}

    def _delete_floor_route(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        route_id = str(payload.get("id") or payload.get("route_id") or "").strip()
        if not route_id:
            return self._error("请指定要删除的跨楼层路线")
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") == "running":
                return self._error("任务执行中不能删除跨楼层路线")
            updated, removed = remove_floor_route(self._floor_routes, route_id)
            if not removed:
                return self._error("跨楼层路线不存在")
            self._floor_routes = updated
            self._save_json("floor_routes.json", self._floor_routes)
            self._route_config_floor_ids = self._configured_route_floor_ids()
        self._append_event("跨楼层路线已删除", {"route_id": route_id})
        return {"ok": True, "route_id": route_id, "message": "跨楼层路线已删除"}

    def _publish_floor_switch_result(self, payload: Dict[str, Any]) -> None:
        enriched = dict(payload)
        request_id = str(enriched.get("request_id") or "").strip()
        transaction = enriched.get("transaction")
        if isinstance(transaction, dict) and str(transaction.get("request_id") or "").strip() == request_id:
            for key in ("route_id", "plan_id", "map_epoch"):
                if transaction.get(key) is not None:
                    enriched.setdefault(key, transaction.get(key))
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
        if request_id and request_id == str(active.get("connector_request_id") or "").strip():
            for result_key, active_key in (
                ("route_id", "connector_route_id"),
                ("plan_id", "connector_plan_id"),
                ("map_epoch", "connector_map_epoch"),
            ):
                if active.get(active_key) is not None:
                    enriched.setdefault(result_key, active.get(active_key))
        message = String()
        message.data = json.dumps(enriched, ensure_ascii=False, separators=(",", ":"))
        self.floor_switch_result_pub.publish(message)
        self._append_event("跨楼层地图事务", enriched)

    def _persist_floor_switch_transaction(self, transaction: Dict[str, Any]) -> None:
        with self._data_lock:
            self._settings["floor_switch_transaction"] = dict(transaction)
            self._save_json("settings.json", self._settings)

    def _advance_floor_switch_transaction(
        self,
        transaction: Dict[str, Any],
        state: str,
        *,
        message: str,
        code: Optional[str] = None,
        **evidence: Any,
    ) -> Dict[str, Any]:
        result = advance_transaction(
            transaction,
            state,
            message=message,
            now_text=now_text(),
            code=code,
            **evidence,
        )
        if result.get("ok"):
            transaction.clear()
            transaction.update(result["transaction"])
            self._persist_floor_switch_transaction(transaction)
        return result

    def _fail_floor_switch_transaction(
        self,
        transaction: Optional[Dict[str, Any]],
        *,
        code: str,
        message: str,
        **evidence: Any,
    ) -> Dict[str, Any]:
        if not isinstance(transaction, dict):
            return {"ok": False, "code": code, "message": message}
        result = self._advance_floor_switch_transaction(
            transaction,
            "FAILED",
            code=code,
            message=message,
            **evidence,
        )
        return {
            "ok": False,
            "code": code,
            "message": message,
            "transaction": transaction,
            "phase_update": result,
            **evidence,
        }

    def _on_floor_switch_request(self, msg: String) -> None:
        try:
            request = json.loads(msg.data)
        except Exception as exc:
            self._publish_floor_switch_result(
                {"ok": False, "code": "floor_switch_request_invalid", "message": str(exc)}
            )
            return
        if not isinstance(request, dict):
            self._publish_floor_switch_result(
                {"ok": False, "code": "floor_switch_request_invalid", "message": "切层请求不是对象"}
            )
            return
        request_id = str(request.get("request_id") or "").strip()
        if not request_id:
            self._publish_floor_switch_result(
                {"ok": False, "code": "floor_switch_request_id_missing", "message": "切层请求缺少 request_id"}
            )
            return
        with self._data_lock:
            admission = request_admission(
                self._settings.get("floor_switch_transaction"),
                request_id,
            )
        if not admission.get("ok"):
            self._publish_floor_switch_result(
                {**admission, "request_id": request_id}
            )
            return
        with self._floor_switch_lock:
            if self._floor_switch_inflight:
                self._publish_floor_switch_result(
                    {
                        "ok": False,
                        "request_id": request_id,
                        "code": "floor_switch_busy",
                        "message": "已有跨楼层地图事务正在执行",
                        "active_request_id": self._floor_switch_inflight,
                    }
                )
                return
            self._floor_switch_inflight = request_id
        threading.Thread(
            target=self._run_floor_switch_transaction,
            args=(dict(request),),
            daemon=True,
            name=f"floor-switch-{request_id[-8:]}",
        ).start()

    def _run_floor_switch_transaction(self, request: Dict[str, Any]) -> None:
        request_id = str(request.get("request_id") or "")
        transaction: Optional[Dict[str, Any]] = None
        try:
            with self._data_lock:
                active = dict(self._settings.get("active_task") or {})
                routes = [dict(item) for item in self._floor_routes]
                selected_map_id = self._settings.get("selected_map_id")
            context = resolve_floor_switch_request(
                request,
                routes=routes,
                active_task=active,
                selected_map_id=selected_map_id,
            )
            if not context.get("ok"):
                self._publish_floor_switch_result(
                    {
                        **context,
                        "request_id": request_id,
                    }
                )
                return
            route = dict(context["route"])
            source_floor = str(context["source_floor"])
            target_floor = str(context["target_floor"])
            source_map_id = str(context["source_map_id"])
            task_id = str(context["task_id"])
            target_map_id = str(route.get("target_map_id") or "").strip()
            target_pose = dict(route.get("target_platform") or {})
            epoch = int(context["map_epoch"])
            started = begin_transaction(
                request=request,
                context=context,
                map_epoch=epoch,
                now_text=now_text(),
            )
            if not started.get("ok"):
                self._publish_floor_switch_result({**started, "request_id": request_id})
                return
            transaction = dict(started["transaction"])
            self._persist_floor_switch_transaction(transaction)
            activation = self._activate_cross_floor_target_map(target_map_id)
            if not activation.get("ok"):
                failed = self._fail_floor_switch_transaction(
                    transaction,
                    code=str(activation.get("code") or "floor_switch_map_failed"),
                    message=str(activation.get("message") or "104/106目标地图切换失败"),
                    map_activation=activation,
                )
                self._publish_floor_switch_result(
                    {
                        **failed,
                        "request_id": request_id,
                        "route_id": route.get("id"),
                        "source_floor": source_floor,
                        "target_floor": target_floor,
                        "target_map_id": target_map_id,
                    }
                )
                return
            if not self._floor_switch_task_is_active(task_id):
                failed = self._fail_floor_switch_transaction(
                    transaction,
                    code="floor_switch_task_cancelled",
                    message="跨楼层任务已停止，当前地图保持不动",
                    map_activation=activation,
                )
                self._publish_floor_switch_result(
                    {
                        **failed,
                        "request_id": request_id,
                        "route_id": route.get("id"),
                        "source_floor": source_floor,
                        "target_floor": target_floor,
                        "target_map_id": target_map_id,
                    }
                )
                return
            phase = self._advance_floor_switch_transaction(
                transaction,
                "RELOCALIZING",
                message="104/106目标地图已激活，正在执行2101重定位",
                map_activation=activation,
            )
            if not phase.get("ok"):
                failed = self._fail_floor_switch_transaction(
                    transaction,
                    code="floor_switch_phase_invalid",
                    message=str(phase.get("message") or "切图阶段更新失败"),
                )
                self._publish_floor_switch_result(
                    {
                        **failed,
                        "request_id": request_id,
                    }
                )
                return
            relocalization = self._publish_initialpose(
                {
                    "floor": target_floor,
                    "frame_id": "map",
                    "x": target_pose.get("x"),
                    "y": target_pose.get("y"),
                    "z": target_pose.get("z", 0.0),
                    "yaw": target_pose.get("yaw"),
                },
                allow_active_task=True,
                event_text="跨楼层自动重定位",
                pose_tolerance_m=float(self.get_parameter("cross_floor_platform_position_tolerance_m").value),
                yaw_tolerance_rad=float(self.get_parameter("cross_floor_platform_yaw_tolerance_rad").value),
                require_lifecycle=False,
                stability_window_s=0.0,
            )
            task_still_active = self._floor_switch_task_is_active(task_id)
            completion = completion_decision(
                transaction,
                task_active=task_still_active,
                target_map_id=target_map_id,
                map_activation=activation,
                relocalization=relocalization,
            )
            if not completion.get("ok"):
                failed = self._fail_floor_switch_transaction(
                    transaction,
                    code=str(completion.get("code") or "floor_switch_relocalization_failed"),
                    message=str(completion.get("message") or "目标地图重定位失败"),
                    map_activation=activation,
                    relocalization=relocalization,
                )
                self._publish_floor_switch_result(
                    {
                        **failed,
                        "request_id": request_id,
                        "route_id": route.get("id"),
                        "source_floor": source_floor,
                        "target_floor": target_floor,
                        "target_map_id": target_map_id,
                    }
                )
                return
            committed = self._advance_floor_switch_transaction(
                transaction,
                "COMMITTED",
                message="104/106目标地图、2101和目标图位姿已确认",
                map_activation=activation,
                relocalization=relocalization,
            )
            if not committed.get("ok"):
                failed = self._fail_floor_switch_transaction(
                    transaction,
                    code="floor_switch_commit_failed",
                    message=str(committed.get("message") or "切图结果写入失败"),
                )
                self._publish_floor_switch_result(
                    {
                        **failed,
                        "request_id": request_id,
                    }
                )
                return
            floor_message = String()
            floor_message.data = target_floor
            self.floor_context_pub.publish(floor_message)
            self._publish_floor_switch_result(
                {
                    "ok": True,
                    "request_id": request_id,
                    "route_id": route.get("id"),
                    "source_floor": source_floor,
                    "target_floor": target_floor,
                    "target_map_id": target_map_id,
                    "target_pose": target_pose,
                    "message": "104/106目标地图和2101目标图位姿已确认",
                    "map_activation": activation,
                    "relocalization": relocalization,
                    "transaction": transaction,
                }
            )
        except Exception as exc:
            self.get_logger().exception("cross-floor map transaction failed")
            failed = self._fail_floor_switch_transaction(
                transaction,
                code="floor_switch_exception",
                message=str(exc),
                exception=str(exc),
            )
            self._publish_floor_switch_result(
                {
                    **failed,
                    "request_id": request_id,
                }
            )
        finally:
            with self._floor_switch_lock:
                if self._floor_switch_inflight == request_id:
                    self._floor_switch_inflight = None

    def _floor_switch_task_is_active(self, task_id: str) -> bool:
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
        return (
            bool(task_id)
            and str(active.get("task_id") or "") == str(task_id)
            and str(active.get("status") or "") == "running"
            and bool(active.get("multi_floor"))
        )

    def _floor_identity_validation(
        self,
        floor: Any,
        *,
        subject: str,
        allow_runtime_map: bool = False,
    ) -> Dict[str, Any]:
        config_payload = self._floor_config_payload()
        if not config_payload.get("ok"):
            return {
                "ok": False,
                "code": "floor_registry_unavailable",
                "message": str(config_payload.get("message") or "楼层注册表不可用"),
                "path": config_payload.get("path"),
                "error": config_payload.get("error"),
            }
        validator = validate_runtime_map_floor if allow_runtime_map else validate_registered_floor
        return validator(floor, config_payload["config"], subject=subject)

    def _floor_map_identity_validation(
        self,
        floor: Any,
        map_record: Dict[str, Any],
        *,
        subject: str,
    ) -> Dict[str, Any]:
        config_payload = self._floor_config_payload()
        if not config_payload.get("ok"):
            return {
                "ok": False,
                "code": "floor_registry_unavailable",
                "message": str(config_payload.get("message") or "楼层注册表不可用"),
                "path": config_payload.get("path"),
            }
        return validate_floor_matches_map(
            floor,
            map_record,
            config_payload["config"],
            subject=subject,
            allow_unregistered_map=True,
        )

    def _preflight_payload(self) -> Dict[str, Any]:
        with self._preflight_lock:
            running = self._preflight_running_payload_unlocked()
            if running:
                return {"ok": True, "running": True, "preflight": running}
            return {"ok": True, "preflight": self._preflight_with_age_unlocked()}

    def _tick_auto_preflight(self) -> None:
        now_monotonic = time.monotonic()
        start_delay_s = max(1.0, float(self.get_parameter("auto_preflight_start_delay_s").value))
        interval_s = max(60.0, float(self.get_parameter("auto_preflight_interval_s").value))
        if self._last_auto_preflight_monotonic <= 0.0:
            if now_monotonic - self._auto_preflight_started_monotonic < start_delay_s:
                return
        elif now_monotonic - self._last_auto_preflight_monotonic < interval_s:
            return
        response = self._start_preflight_background(
            {"mode": "move", "site": "auto", "wait": False, "source": "automatic"}
        )
        running = response.get("preflight") if isinstance(response.get("preflight"), dict) else {}
        if running.get("request_id"):
            self._last_auto_preflight_monotonic = now_monotonic

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
            "m20pro_command_mux",
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
            self._topic("command_mux_status_topic"),
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
        with self._data_lock:
            current_state["map_relocalization_required"] = self._settings.get(
                "map_relocalization_required"
            )
        context = preflight_context(
            payload,
            localization_ok=current_state.get("localization_ok"),
            navigation_status=current_state.get("navigation_status"),
            map_relocalization_required=current_state.get("map_relocalization_required"),
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
        items.append(
            preflight_localization_item(
                loc_ok,
                map_relocalization_required=current_state.get("map_relocalization_required"),
            )
        )
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
                [
                    "/map_server",
                    "/controller_server",
                    "/planner_server",
                    "/recoveries_server",
                    "/bt_navigator",
                ]
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

    def _delete_map(self, map_id: str, *, cascade: bool = False) -> Dict[str, Any]:
        map_id = str(map_id or "").strip()
        if not map_id:
            return self._error("缺少地图 id")
        effective_map_id = str(self._effective_map_id() or "").strip()
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") == "running":
                return self._error("任务执行中不能删除地图，请先停止当前任务")
            route_dependencies = [
                str(item.get("id") or "")
                for item in self._floor_routes
                if map_id in (str(item.get("source_map_id") or ""), str(item.get("target_map_id") or ""))
            ]
            if route_dependencies:
                return self._error(
                    "地图正在被跨楼层路线使用，请先删除对应路线",
                    {"code": "map_used_by_floor_route", "route_ids": route_dependencies},
                )
            record = self._find_map_record_unlocked(map_id)
            if record is None:
                return self._error("地图不存在")
            is_archived = any(str(item.get("id") or "") == map_id for item in self._maps)
            is_builtin = not is_archived and any(
                str(item.get("id") or "") == map_id for item in self._builtin_maps
            )
            if not is_archived and not is_builtin:
                return self._error("地图不存在或不属于可管理地图")
            plan = apply_map_delete_state(
                archived_maps=[*self._maps, *([record] if is_builtin else [])],
                annotations=self._annotations,
                tasks=self._tasks,
                sessions=self._sessions,
                settings=self._settings,
                map_id=map_id,
                protected_map_ids=[
                    str(self._settings.get("selected_map_id") or ""),
                    str(self._settings.get("working_map_id") or ""),
                    effective_map_id,
                ],
                updated_at=now_text(),
            )
            if not plan.get("ok"):
                return self._error(str(plan.get("message") or "地图不能删除"), {"code": plan.get("code")})
            dependent_count = int(plan.get("deleted_annotations", 0)) + int(plan.get("deleted_tasks", 0))
            if dependent_count and not cascade:
                return self._error(
                    "地图仍有关联点位或任务，需要确认级联删除",
                    {
                        "code": "map_has_dependents",
                        "deleted_annotations": plan.get("deleted_annotations", 0),
                        "deleted_tasks": plan.get("deleted_tasks", 0),
                    },
                )
            if is_builtin:
                file_plan = {
                    "delete": False,
                    "path": str(record.get("directory") or ""),
                    "reason": "project_builtin_hidden",
                }
                plan["settings"]["hidden_builtin_map_ids"] = sorted({
                    *[
                        str(item or "").strip()
                        for item in (plan["settings"].get("hidden_builtin_map_ids") or [])
                        if str(item or "").strip()
                    ],
                    map_id,
                })
            else:
                file_plan = removable_map_archive_directory(
                    self.map_archive_dir,
                    record,
                    [*self._builtin_maps, *list(plan["maps"])],
                )
            originals = {
                "maps": list(self._maps),
                "annotations": list(self._annotations),
                "tasks": list(self._tasks),
                "sessions": list(self._sessions),
                "settings": dict(self._settings),
            }

        trash_path: Optional[FsPath] = None
        candidate = FsPath(str(file_plan.get("path") or ""))
        if file_plan.get("delete") and candidate.exists():
            trash_root = self.map_archive_dir / ".trash"
            trash_root.mkdir(parents=True, exist_ok=True)
            trash_path = trash_root / ("%s_%s" % (candidate.name, random_suffix(8)))
            try:
                os.replace(candidate, trash_path)
            except Exception as exc:
                return self._error("地图文件无法安全移入回收区，未删除任何索引", {"error": str(exc)})

        try:
            with self._data_lock:
                self._maps = [
                    item
                    for item in plan["maps"]
                    if not (is_builtin and str(item.get("id") or "") == map_id)
                ]
                self._annotations = list(plan["annotations"])
                self._tasks = list(plan["tasks"])
                self._sessions = list(plan["sessions"])
                self._settings = dict(plan["settings"])
                self._save_json("maps.json", self._maps)
                self._save_json("annotations.json", self._annotations)
                self._save_json("tasks.json", self._tasks)
                self._save_json("mapping_sessions.json", self._sessions)
                self._save_json("settings.json", self._settings)
        except Exception as exc:
            with self._data_lock:
                self._maps = originals["maps"]
                self._annotations = originals["annotations"]
                self._tasks = originals["tasks"]
                self._sessions = originals["sessions"]
                self._settings = originals["settings"]
                for name, value in (
                    ("maps.json", self._maps),
                    ("annotations.json", self._annotations),
                    ("tasks.json", self._tasks),
                    ("mapping_sessions.json", self._sessions),
                    ("settings.json", self._settings),
                ):
                    try:
                        self._save_json(name, value)
                    except Exception:
                        pass
            if trash_path is not None and trash_path.exists() and not candidate.exists():
                try:
                    os.replace(trash_path, candidate)
                except Exception:
                    pass
            return self._error("地图删除索引保存失败，已回滚", {"error": str(exc)})

        files_deleted = False
        if trash_path is not None and trash_path.exists():
            shutil.rmtree(trash_path, ignore_errors=True)
            files_deleted = not trash_path.exists()
        with self._map_file_cache_lock:
            self._map_file_cache.clear()
            self._map_file_summary_cache.clear()
        self._append_event(
            "删除地图",
            {
                "map_id": map_id,
                "name": record.get("name"),
                "deleted_annotations": plan.get("deleted_annotations", 0),
                "deleted_tasks": plan.get("deleted_tasks", 0),
                "updated_sessions": plan.get("updated_sessions", 0),
                "files_deleted": files_deleted,
                "file_reason": file_plan.get("reason"),
            },
        )
        return {
            "ok": True,
            "deleted": map_id,
            "name": record.get("name"),
            "deleted_annotations": plan.get("deleted_annotations", 0),
            "deleted_tasks": plan.get("deleted_tasks", 0),
            "updated_sessions": plan.get("updated_sessions", 0),
            "files_deleted": files_deleted,
            "file_reason": file_plan.get("reason"),
            "message": (
                "地图已从业务地图库移除，并同步清理关联点位和任务"
                if is_builtin
                else "地图已删除，并同步清理关联点位和任务"
            ),
        }

    def _edit_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        map_id = str(payload.get("map_id") or "").strip()
        cells = payload.get("cells") if isinstance(payload.get("cells"), list) else []
        if not map_id:
            return self._error("请先选择需要修饰的固定地图")
        if not cells:
            return self._error("地图没有需要保存的修改")
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") == "running":
                return self._error("任务执行中不能修改地图，请先停止当前任务")
            record = self._find_map_record_unlocked(map_id)
            if record is None:
                return self._error("需要修饰的地图不存在")
            selected_map_id = str(self._settings.get("selected_map_id") or "")
        if map_id != selected_map_id:
            return self._error("只能修饰当前已生效的固定地图，请先在顶部地图入口切换")

        source_yaml = FsPath(self._resolve_path(str(record.get("yaml_path") or "")))
        if not source_yaml.exists():
            return self._error("地图 yaml 不存在", {"yaml_path": str(source_yaml)})
        source_dir = source_yaml.parent
        requested_name = str(payload.get("name") or "").strip()
        default_name = "%s_修饰_%s" % (
            str(record.get("name") or record.get("id") or "地图"),
            time.strftime("%Y%m%d_%H%M%S", time.localtime()),
        )
        map_name = sanitize_name(requested_name, default_name)
        dest = self.map_archive_dir / map_name
        if dest.exists():
            dest = self.map_archive_dir / ("%s_%s" % (map_name, random_suffix(6)))
        staging = self.map_archive_dir / (".%s.staging.%s" % (dest.name, random_suffix(8)))
        try:
            shutil.copytree(source_dir, staging)
            edited_yaml = find_map_yaml(staging)
            if edited_yaml is None:
                raise RuntimeError("复制后的地图包没有 occupancy yaml")
            edit_result = apply_map_cell_edits(edited_yaml, cells)
            os.replace(staging, dest)
        except Exception as exc:
            if staging.exists():
                shutil.rmtree(staging, ignore_errors=True)
            if dest.exists():
                shutil.rmtree(dest, ignore_errors=True)
            return self._error("地图修饰保存失败", {"error": str(exc)})

        final_yaml = find_map_yaml(dest)
        if final_yaml is None:
            shutil.rmtree(dest, ignore_errors=True)
            return self._error("地图修饰保存失败：新地图缺少 yaml")
        created_at = now_text()
        edited_record = dict(record)
        edited_record.update(
            {
                "id": new_id("map"),
                "name": map_name,
                "directory": str(dest),
                "yaml_path": str(final_yaml),
                "source": "web_map_editor",
                "source_path": "",
                "factory_apply_path": "",
                "readonly": False,
                "parent_map_id": map_id,
                "edited_from_source_path": record.get("factory_apply_path") or record.get("source_path"),
                "created_at": created_at,
                "updated_at": created_at,
                "edit_summary": {
                    "changed_cells": int(edit_result.get("changed_cells", 0) or 0),
                    "requested_cells": len(cells),
                },
            }
        )
        edited_record.pop("factory_map_name", None)
        edited_record.pop("factory_source_reason", None)
        with self._data_lock:
            original_maps = list(self._maps)
            original_annotations = list(self._annotations)
            original_tasks = list(self._tasks)
            annotation_id_map: Dict[str, str] = {}
            cloned_annotations: List[Dict[str, Any]] = []
            for annotation in self._annotations:
                if str(annotation.get("map_id") or "") != map_id:
                    continue
                clone = dict(annotation)
                source_annotation_id = str(annotation.get("id") or "")
                clone["id"] = new_id("point")
                clone["map_id"] = edited_record["id"]
                clone["source_annotation_id"] = source_annotation_id
                clone["created_at"] = created_at
                clone["updated_at"] = created_at
                annotation_id_map[source_annotation_id] = clone["id"]
                cloned_annotations.append(clone)
            cloned_tasks: List[Dict[str, Any]] = []
            for task in self._tasks:
                if str(task.get("map_id") or "") != map_id:
                    continue
                source_ids = [str(item) for item in (task.get("annotation_ids") or [])]
                if not source_ids or any(item not in annotation_id_map for item in source_ids):
                    continue
                cloned_tasks.append(
                    {
                        "id": new_id("task"),
                        "name": str(task.get("name") or "巡检任务"),
                        "map_id": edited_record["id"],
                        "annotation_ids": [annotation_id_map[item] for item in source_ids],
                        "status": "ready",
                        "source_task_id": task.get("id"),
                        "created_at": created_at,
                    }
                )
            try:
                self._maps.append(edited_record)
                self._annotations.extend(cloned_annotations)
                self._tasks.extend(cloned_tasks)
                self._save_json("maps.json", self._maps)
                self._save_json("annotations.json", self._annotations)
                self._save_json("tasks.json", self._tasks)
            except Exception as exc:
                self._maps = original_maps
                self._annotations = original_annotations
                self._tasks = original_tasks
                for name, value in (
                    ("maps.json", original_maps),
                    ("annotations.json", original_annotations),
                    ("tasks.json", original_tasks),
                ):
                    try:
                        self._save_json(name, value)
                    except Exception:
                        pass
                shutil.rmtree(dest, ignore_errors=True)
                return self._error("地图修饰索引保存失败，已回滚新地图文件", {"error": str(exc)})
        self._append_event(
            "前端修饰地图已保存",
            {
                "map_id": edited_record["id"],
                "parent_map_id": map_id,
                "changed_cells": edit_result.get("changed_cells"),
                "cloned_annotations": len(cloned_annotations),
                "cloned_tasks": len(cloned_tasks),
                "directory": str(dest),
            },
        )
        return {
            "ok": True,
            "map": edited_record,
            "parent_map_id": map_id,
            "changed_cells": edit_result.get("changed_cells"),
            "cloned_annotations": len(cloned_annotations),
            "cloned_tasks": len(cloned_tasks),
            "message": "地图修饰已保存为新版本；原地图未覆盖，已继承 %d 个点位和 %d 个任务，请确认后切换使用"
            % (len(cloned_annotations), len(cloned_tasks)),
        }

    def _activate_cross_floor_target_map(self, map_id: str) -> Dict[str, Any]:
        """Activate already-staged 104/106 map assets in parallel."""
        with self._data_lock:
            previous_map_id = self._settings.get("selected_map_id")
            record = self._find_map_record_unlocked(map_id)
        if record is None:
            return {
                "ok": False,
                "code": "floor_switch_target_map_missing",
                "message": "目标地图不存在于104本地地图库",
            }
        yaml_path = self._route_map_yaml(record)
        factory_path = str(
            record.get("factory_apply_path") or record.get("source_path") or ""
        ).strip()
        if not yaml_path:
            return {
                "ok": False,
                "code": "floor_switch_target_nav2_map_missing",
                "message": "目标地图缺少104本地Nav2 yaml",
            }
        if not factory_path.startswith("/var/opt/robot/data/maps/") or factory_path.endswith(
            "/active"
        ):
            return {
                "ok": False,
                "code": "floor_switch_target_factory_map_missing",
                "message": "目标地图缺少106本地原厂地图包路径",
                "factory_path": factory_path or None,
            }

        timeout_s = max(
            5.0,
            min(
                60.0,
                float(self.get_parameter("cross_floor_factory_apply_timeout_s").value),
            ),
        )
        started_at = time.monotonic()
        with ThreadPoolExecutor(max_workers=2) as executor:
            nav2_future = executor.submit(
                self._load_selected_map_into_nav2,
                record,
                verify_observed=False,
            )
            factory_future = executor.submit(
                self._apply_cross_floor_factory_map,
                record,
                timeout_s=timeout_s,
            )
            nav2_load = nav2_future.result()
            factory_apply = factory_future.result()
        duration_s = max(0.0, time.monotonic() - started_at)
        if not nav2_load.get("ok") or not factory_apply.get("ok"):
            return {
                "ok": False,
                "code": (
                    str(nav2_load.get("code") or "floor_switch_nav2_map_failed")
                    if not nav2_load.get("ok")
                    else str(factory_apply.get("code") or "floor_switch_factory_map_failed")
                ),
                "message": (
                    str(nav2_load.get("message") or "104目标地图加载失败")
                    if not nav2_load.get("ok")
                    else str(factory_apply.get("message") or "106目标地图激活失败")
                ),
                "duration_s": duration_s,
                "nav2_load_map": nav2_load,
                "factory_apply_map": factory_apply,
            }

        with self._data_lock:
            selection = apply_selected_map_choice_state(
                self._settings,
                map_id=map_id,
                previous_map_id=previous_map_id,
                record=record,
                nav2_load=nav2_load,
                reason="cross_floor_transition",
                now_text=now_text,
            )
            self._settings = selection["settings"]
            self._save_json("settings.json", self._settings)
        if selection.get("clear_pose"):
            with self._lock:
                self._state["localization_ok"] = False
                self._state["pose"] = None
                self._state["relocalization_attempt"] = None
                self._state["pose_history"] = []
                self._state["path"] = {
                    "version": int(self._state.get("path", {}).get("version", 0) or 0)
                    + 1,
                    "points": [],
                }
                self._state["local_path"] = {
                    "version": int(
                        self._state.get("local_path", {}).get("version", 0) or 0
                    )
                    + 1,
                    "points": [],
                }
        self._remember_working_map_id(map_id, reason="cross_floor_transition")
        return {
            "ok": True,
            "code": "floor_switch_maps_activated",
            "message": "104和106目标地图已并行激活",
            "selected_map_id": map_id,
            "duration_s": duration_s,
            "nav2_load_map": nav2_load,
            "factory_apply_map": factory_apply,
            "map_relocalization_required": selection.get("relocalization_required"),
        }

    def _apply_cross_floor_factory_map(
        self,
        record: Dict[str, Any],
        *,
        timeout_s: float,
    ) -> Dict[str, Any]:
        """Run one blocking drmap apply; its exit status is the activation result."""
        source_path = str(
            record.get("factory_apply_path") or record.get("source_path") or ""
        ).strip()
        if not source_path.startswith("/var/opt/robot/data/maps/") or source_path.endswith(
            "/active"
        ):
            return {
                "ok": False,
                "code": "factory_map_path_invalid",
                "message": "106目标地图包路径无效",
                "source_path": source_path or None,
            }
        result = self._run_factory_shell(
            f"sudo -n drmap apply {shlex.quote(source_path)}",
            factory_host=str(self.get_parameter("factory_host").value).strip(),
            factory_user=str(self.get_parameter("factory_user").value).strip(),
            timeout=max(5.0, float(timeout_s)),
        )
        if not result.get("ok"):
            return {
                "ok": False,
                "code": "factory_map_apply_failed",
                "message": "106 drmap apply 失败",
                "source_path": source_path,
                "output": result.get("output"),
                "returncode": result.get("returncode"),
            }
        return {
            "ok": True,
            "code": "factory_map_applied",
            "message": "106 drmap apply 已成功返回",
            "source_path": source_path,
            "output": result.get("output"),
        }

    def _select_map(
        self,
        payload: Dict[str, Any],
        *,
        allow_active_task: bool = False,
        reason: str = "manual_select",
        factory_timeout_s: Optional[float] = None,
        map_min_update_time: Optional[float] = None,
    ) -> Dict[str, Any]:
        map_id = str(payload.get("map_id") or "").strip() or None
        record: Optional[Dict[str, Any]] = None
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") == "running" and not allow_active_task:
                return self._error("任务执行中不能切换地图，请先停止当前任务")
            previous_map_id = self._settings.get("selected_map_id")
            previous_record = self._find_map_record_unlocked(previous_map_id)
            if map_id:
                record = self._find_map_record_unlocked(map_id)
                if record is None:
                    return self._error("地图不存在")
        if record is not None:
            identity = self._floor_identity_validation(
                record.get("floor"),
                subject="地图楼层",
                allow_runtime_map=True,
            )
            if not identity.get("ok"):
                return self._error(
                    str(identity["message"]),
                    {key: value for key, value in identity.items() if key not in ("ok", "message")},
                )
        factory_timeout = (
            min(120.0, max(10.0, float(factory_timeout_s)))
            if factory_timeout_s is not None
            else min(120.0, max(10.0, float(self.get_parameter("map_import_timeout_s").value)))
        )
        if map_id:
            nav2_load = self._load_selected_map_into_nav2(
                record,
                min_update_time=map_min_update_time,
            )
            if not nav2_load.get("ok"):
                rollback_nav2 = (
                    self._load_selected_map_into_nav2(previous_record)
                    if previous_record is not None and nav2_load.get("map_change_possible")
                    else {"ok": True, "skipped": True, "message": "Nav2 地图未发生可见切换，无需回滚"}
                )
                return self._error(
                    str(nav2_load["message"]),
                    {
                        "code": "nav2_map_load_failed",
                        "nav2_load_map": nav2_load,
                        "rollback_nav2_map": rollback_nav2,
                        "state_uncertain": not bool(rollback_nav2.get("ok")),
                    },
                )
        else:
            nav2_load = {
                "ok": True,
                "skipped": True,
                "message": "已切换到实时 /map 观察；未调用 Nav2 load_map",
            }
        factory_apply = self._apply_selected_map_to_factory(
            record,
            timeout_s=factory_timeout,
            reject_active_alias=(reason == "cross_floor_transition"),
        ) if map_id and record else {
            "ok": True,
            "skipped": True,
            "message": "未选择固定地图；未调用 106 drmap apply",
        }
        if not factory_apply.get("ok"):
            rollback_nav2 = (
                self._load_selected_map_into_nav2(previous_record)
                if previous_record is not None
                else {"ok": True, "skipped": True, "message": "没有上一张固定地图可回滚"}
            )
            return self._error(
                str(factory_apply.get("message") or "106 原厂地图切换失败"),
                {
                    "code": "factory_map_apply_failed",
                    "nav2_load_map": nav2_load,
                    "factory_apply_map": factory_apply,
                    "rollback_nav2_map": rollback_nav2,
                    "state_uncertain": not bool(rollback_nav2.get("ok")),
                },
            )
        with self._data_lock:
            result = apply_selected_map_choice_state(
                self._settings,
                map_id=map_id,
                previous_map_id=previous_map_id,
                record=record,
                nav2_load=nav2_load,
                reason=reason,
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
                self._state["relocalization_attempt"] = None
                self._state["pose_history"] = []
                self._state["path"] = {"version": int(self._state.get("path", {}).get("version", 0) or 0) + 1, "points": []}
                self._state["local_path"] = {"version": int(self._state.get("local_path", {}).get("version", 0) or 0) + 1, "points": []}
        if record is not None and reason != "cross_floor_transition":
            floor_message = String()
            floor_message.data = str(record.get("floor") or "").strip()
            if floor_message.data:
                self.floor_context_pub.publish(floor_message)
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

    def _load_selected_map_into_nav2(
        self,
        record: Dict[str, Any],
        *,
        min_update_time: Optional[float] = None,
        verify_observed: bool = True,
    ) -> Dict[str, Any]:
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
        selected_map: Dict[str, Any] = {}
        if verify_observed:
            selected_map = self._map_file_snapshot(str(record.get("id") or ""))
            with self._lock:
                live_map = dict(self._state.get("map") or {})
            current_match = selected_map_status_payload(
                selected_map_id=str(record.get("id") or ""),
                live_map=live_map,
                selected_map=selected_map,
                min_update_time=min_update_time,
                now_text=now_text,
            )
            if current_match.get("ready"):
                return {
                    "ok": True,
                    "loaded": False,
                    "already_loaded": True,
                    "message": "Nav2 当前 /map 已经与前端选中地图一致",
                    "yaml_path": str(yaml_path),
                    "image_repair": image_repair,
                    "content_digest": selected_map.get("content_digest"),
                    "selected_map_status": current_match,
                }
        timeout_s = max(0.5, float(self.get_parameter("map_select_load_timeout_s").value))
        if not self.load_map_client.wait_for_service(timeout_sec=timeout_s):
            return {
                "ok": False,
                "code": "load_map_service_unavailable",
                "message": f"{self.load_map_client.srv_name} 不可用",
                "yaml_path": str(yaml_path),
                "image_repair": image_repair,
                "content_digest": selected_map.get("content_digest"),
            }
        request = LoadMap.Request()
        request.map_url = str(yaml_path)
        future = self.load_map_client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            return {
                "ok": False,
                "code": "load_map_timeout",
                "message": f"Nav2 load_map 超时 {timeout_s:.1f}s",
                "yaml_path": str(yaml_path),
                "image_repair": image_repair,
                "content_digest": selected_map.get("content_digest"),
                "map_change_possible": True,
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
                "content_digest": selected_map.get("content_digest"),
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
                "content_digest": selected_map.get("content_digest"),
            }
        self._clear_task_costmaps("select_map_load_nav2")
        if not verify_observed:
            return {
                "ok": True,
                "loaded": True,
                "message": "Nav2 load_map 已成功返回",
                "yaml_path": str(yaml_path),
                "result": result,
                "image_repair": image_repair,
            }
        match = self._wait_for_selected_map_match(
            selected_map,
            min_update_time=min_update_time,
        )
        if not match.get("ready"):
            return {
                "ok": False,
                "code": "selected_map_not_observed",
                "message": "Nav2 load_map 已返回成功，但 /map 未确认切换到目标地图",
                "yaml_path": str(yaml_path),
                "result": result,
                "selected_map_status": match,
                "image_repair": image_repair,
                "content_digest": selected_map.get("content_digest"),
                "map_change_possible": True,
            }
        return {
            "ok": True,
            "loaded": True,
            "message": str(match["message"]),
            "yaml_path": str(yaml_path),
            "result": result,
            "map_matched": bool(match.get("ready")),
            "selected_map_status": match,
            "image_repair": image_repair,
            "content_digest": selected_map.get("content_digest"),
        }

    def _wait_for_selected_map_match(
        self,
        selected_map: Dict[str, Any],
        *,
        min_update_time: Optional[float] = None,
    ) -> Dict[str, Any]:
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
                min_update_time=min_update_time,
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
        config_payload = self._floor_config_payload()
        if not config_payload.get("ok"):
            return self._error(
                str(config_payload.get("message") or "楼层注册表不可用"),
                {"code": "floor_registry_unavailable", "path": config_payload.get("path")},
            )
        identity = validate_mapping_session_identity(
            payload,
            config_payload["config"],
            allow_floor_registration=True,
        )
        if not identity.get("ok"):
            return self._error(
                str(identity["message"]),
                {key: value for key, value in identity.items() if key not in ("ok", "message")},
            )
        normalized_payload = dict(payload)
        normalized_payload.update(
            {
                "floors": identity["floors"],
                "active_floor": identity["active_floor"],
                "mode": identity["mode"],
            }
        )
        prepared = prepare_mapping_session_create(
            normalized_payload,
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
            elif prepared.get("updated_project"):
                project_id = str(prepared["updated_project"].get("id") or "")
                self._projects = [
                    prepared["updated_project"] if str(item.get("id") or "") == project_id else item
                    for item in self._projects
                ]
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

    def _select_mapping_session_floor(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        session_id = str(payload.get("session_id") or "").strip()
        floor = str(payload.get("floor") or "").strip()
        session = self._find_session(session_id or None)
        if session is None:
            return self._error("建图任务不存在，请先建立建图任务", {"code": "mapping_session_missing"})
        identity = self._floor_identity_validation(floor, subject="建图楼层")
        if not identity.get("ok"):
            return self._error(
                str(identity["message"]),
                {key: value for key, value in identity.items() if key not in ("ok", "message")},
            )
        result = select_mapping_floor(session, floor, updated_at=now_text())
        if not result.get("ok"):
            return self._error(
                str(result.get("message") or "无法切换建图楼层"),
                {key: value for key, value in result.items() if key not in ("ok", "message")},
            )
        with self._data_lock:
            session.clear()
            session.update(result["session"])
            self._save_json("mapping_sessions.json", self._sessions)
        self._append_event("切换建图楼层步骤", {"session_id": session["id"], "floor": floor})
        return {"ok": True, "session": session, "step": result.get("step")}

    def _mapping_command(self, param_name: str, session_id: Optional[str]) -> Dict[str, Any]:
        session = self._find_session(session_id)
        if session is None:
            return self._error("建图任务不存在，请先建立建图任务")
        if param_name == "factory_mapping_start_command":
            start_check = mapping_start_precondition(session)
            if not start_check.get("ok"):
                return self._error(
                    str(start_check["message"]),
                    {key: value for key, value in start_check.items() if key not in ("ok", "message")},
                )
        identity = self._floor_identity_validation(session.get("active_floor"), subject="当前建图楼层")
        if not identity.get("ok"):
            return self._error(
                str(identity["message"]),
                {key: value for key, value in identity.items() if key not in ("ok", "message")},
            )
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

    def _apply_selected_map_to_factory(
        self,
        record: Dict[str, Any],
        *,
        timeout_s: Optional[float] = None,
        reject_active_alias: bool = False,
    ) -> Dict[str, Any]:
        source_path = str(
            record.get("factory_apply_path")
            or record.get("source_path")
            or ""
        ).strip()
        base = "/var/opt/robot/data/maps/"
        if not source_path:
            return {"ok": True, "skipped": True, "message": "地图记录没有 106 原厂路径；未调用 drmap apply"}
        if source_path == "/var/opt/robot/data/maps/active" and reject_active_alias:
            return {
                "ok": False,
                "code": "factory_target_active_alias",
                "message": "目标地图只指向 106 active 软链接，无法证明它不是起始地图，拒绝跨层切换",
                "source_path": source_path,
            }
        if source_path == "/var/opt/robot/data/maps/active":
            return {
                "ok": True,
                "skipped": True,
                "message": "地图记录指向当前 106 active；无需重复调用 drmap apply",
                "source_path": source_path,
            }
        if not source_path.startswith(base):
            return {"ok": True, "skipped": True, "message": "地图不是 106 原厂地图包；未调用 drmap apply", "source_path": source_path}
        factory_host = str(self.get_parameter("factory_host").value).strip()
        factory_user = str(self.get_parameter("factory_user").value).strip()
        timeout = (
            min(120.0, max(10.0, float(timeout_s)))
            if timeout_s is not None
            else min(120.0, max(10.0, float(self.get_parameter("map_import_timeout_s").value)))
        )
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
            "returncode": result.get("returncode"),
            "message": "106 drmap apply 执行成功",
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
        identity = self._floor_identity_validation(floor, subject="拉取地图楼层")
        if not identity.get("ok"):
            return self._error(
                str(identity["message"]),
                {key: value for key, value in identity.items() if key not in ("ok", "message")},
            )
        if session and floor != str(session.get("active_floor") or "").strip():
            return self._error(
                "拉取地图楼层必须与当前建图步骤一致",
                {
                    "code": "mapping_import_floor_mismatch",
                    "floor": floor,
                    "active_floor": session.get("active_floor"),
                },
            )

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
                updated_session = mark_mapping_floor_imported(
                    session,
                    floor=floor,
                    map_id=str(map_record["id"]),
                    updated_at=now_text(),
                )
                session.clear()
                session.update(updated_session)
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
            configured = str(self.get_parameter("floor_config").value or "").strip()
            if configured:
                return FsPath(self._resolve_path(configured))
            return FsPath(get_package_share_directory("m20pro_bringup")) / "config" / "runtime_navigation.yaml"
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
        map_record = None
        with self._data_lock:
            if map_id and map_id != "live_map":
                map_record = self._find_map_record_unlocked(map_id)
                if map_record is None:
                    return self._error("地图不存在")
            if not map_id:
                map_id = self._effective_map_id(runtime_state=runtime_state)
            if not map_id and runtime_state.get("map"):
                map_id = "live_map"
            if not map_id:
                return self._error("没有可用地图，请等待实时 /map 或先选择固定地图")
            selected_map_id = self._effective_map_id(runtime_state=runtime_state)
            if map_record is None and map_id and map_id != "live_map":
                map_record = self._find_map_record_unlocked(map_id)
        if map_record is not None:
            identity = self._floor_map_identity_validation(
                context.get("floor"),
                map_record,
                subject="点位楼层",
            )
            if not identity.get("ok"):
                return self._error(
                    str(identity["message"]),
                    {key: value for key, value in identity.items() if key not in ("ok", "message")},
                )
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
        if map_id and map_id != "live_map":
            self._remember_working_map_id(map_id, reason="create_annotation")
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
            localization_ok = self._factory_localization_ok(self._state)
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

    def _recording_status_payload(self) -> Dict[str, Any]:
        with self._recording_lock:
            process = self._recording_process
            state = dict(self._recording_state or {})
            if process is not None:
                return_code = process.poll()
                if return_code is None:
                    state.update({"running": True, "pid": process.pid})
                else:
                    state.update(
                        {
                            "running": False,
                            "return_code": return_code,
                            "completed": return_code in (0, 124, 130),
                            "finished_at": state.get("finished_at") or now_text(),
                        }
                    )
                    self._recording_process = None
                    if self._recording_log_handle is not None:
                        self._recording_log_handle.close()
                        self._recording_log_handle = None
                    self._recording_state = dict(state)
            state.setdefault("running", False)
            prefix = str(state.get("prefix") or "")
            bag_dir = FsPath(str(state.get("bag_dir") or self._recording_bag_root()))
            if prefix and bag_dir.exists():
                matches = sorted(bag_dir.glob(prefix + "_*"), key=lambda item: item.stat().st_mtime, reverse=True)
                if matches:
                    state["bag_path"] = str(matches[0])
            return {"ok": True, "recording": state}

    @staticmethod
    def _recording_bag_root() -> FsPath:
        return FsPath("/home/user/bags")

    def _resolve_recording_path(self, bag_id: Any) -> Optional[FsPath]:
        """Resolve one direct child of the recording root; never accept traversal."""
        raw_id = str(bag_id or "").strip()
        if not raw_id or raw_id in (".", "..") or FsPath(raw_id).name != raw_id:
            return None
        root = self._recording_bag_root().resolve()
        raw_candidate = root / raw_id
        if raw_candidate.is_symlink():
            return None
        candidate = raw_candidate.resolve()
        if candidate == root or candidate.parent != root or not candidate.is_dir():
            return None
        return candidate

    @staticmethod
    def _recording_size_bytes(path: FsPath) -> int:
        # rosbag2 stores metadata and db3/mcap files directly in the bag
        # directory. Avoid recursively stat-ing potentially huge split bags
        # on every status-popover refresh.
        total = 0
        try:
            with os.scandir(path) as entries:
                for entry in entries:
                    if entry.is_file(follow_symlinks=False):
                        total += int(entry.stat(follow_symlinks=False).st_size)
        except OSError:
            return total
        return total

    def _recording_item_payload(self, path: FsPath) -> Dict[str, Any]:
        metadata_path = path / "metadata.yaml"
        metadata: Dict[str, Any] = {}
        if metadata_path.is_file():
            try:
                metadata_text = metadata_path.read_text(encoding="utf-8")
                # Standard rosbag metadata puts the large topic/QoS section
                # after these summary fields. Parse only the valid summary
                # document prefix so opening the list stays lightweight.
                summary_text, marker, _ = metadata_text.partition("\n  topics_with_message_count:")
                loaded = yaml.safe_load(summary_text if marker else metadata_text)
                if isinstance(loaded, dict):
                    metadata = loaded.get("rosbag2_bagfile_information") or loaded
            except (OSError, UnicodeError, yaml.YAMLError):
                metadata = {}
        try:
            modified_at = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(path.stat().st_mtime))
        except OSError:
            modified_at = None
        duration_ns = metadata.get("duration", {}).get("nanoseconds") if isinstance(metadata.get("duration"), dict) else None
        try:
            duration_s = float(duration_ns) / 1_000_000_000.0 if duration_ns is not None else None
        except (TypeError, ValueError):
            duration_s = None
        return {
            "id": path.name,
            "name": path.name,
            "size_bytes": self._recording_size_bytes(path),
            "modified_at": modified_at,
            "message_count": metadata.get("message_count"),
            "duration_s": duration_s,
            "storage_id": metadata.get("storage_identifier"),
        }

    def _recording_list_payload(self) -> Dict[str, Any]:
        root = self._recording_bag_root()
        items: List[Dict[str, Any]] = []
        try:
            root.mkdir(parents=True, exist_ok=True)
            candidates = [item for item in root.iterdir() if item.is_dir() and not item.is_symlink()]
        except OSError as exc:
            return self._error("读取录包目录失败：%s" % exc)
        for item in candidates:
            try:
                has_bag_files = (item / "metadata.yaml").is_file() or any(
                    child.is_file() and child.suffix.lower() in (".db3", ".mcap")
                    for child in item.iterdir()
                )
            except OSError:
                has_bag_files = False
            if has_bag_files:
                items.append(self._recording_item_payload(item))
        items.sort(key=lambda entry: str(entry.get("modified_at") or ""), reverse=True)
        return {"ok": True, "recordings": items}

    def _recording_is_active(self, path: FsPath) -> bool:
        with self._recording_lock:
            process = self._recording_process
            state = dict(self._recording_state or {})
            if process is None or process.poll() is not None:
                return False
            active_path = str(state.get("bag_path") or "").strip()
            if active_path and FsPath(active_path).resolve() == path.resolve():
                return True
            prefix = str(state.get("prefix") or "").strip()
            return bool(prefix and path.name.startswith(prefix + "_"))

    def _rename_recording(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        source = self._resolve_recording_path(payload.get("id"))
        if source is None:
            return self._error("录包不存在或名称无效")
        new_name = sanitize_name(str(payload.get("name") or ""), "")
        if not new_name:
            return self._error("新名称不能为空")
        if new_name == source.name:
            return {"ok": True, "recording": self._recording_item_payload(source)}
        if self._recording_is_active(source):
            return self._error("录包正在写入，停止后才能改名")
        target = self._resolve_recording_path(new_name)
        if target is not None or (self._recording_bag_root() / new_name).exists():
            return self._error("目标名称已存在")
        target = self._recording_bag_root().resolve() / new_name
        try:
            source.rename(target)
        except OSError as exc:
            return self._error("录包改名失败：%s" % exc)
        return {"ok": True, "recording": self._recording_item_payload(target), "message": "录包已改名"}

    def _delete_recording(self, bag_id: Any) -> Dict[str, Any]:
        path = self._resolve_recording_path(bag_id)
        if path is None:
            return self._error("录包不存在或名称无效")
        if self._recording_is_active(path):
            return self._error("录包正在写入，停止后才能删除")
        try:
            shutil.rmtree(path)
        except OSError as exc:
            return self._error("删除录包失败：%s" % exc)
        return {"ok": True, "deleted_id": path.name, "message": "录包已删除"}

    def _send_recording_download(self, bag_id: Any, handler: BaseHTTPRequestHandler) -> None:
        path = self._resolve_recording_path(bag_id)
        if path is None:
            handler.send_error(HTTPStatus.NOT_FOUND, "录包不存在或名称无效")
            return
        archive_name = "%s.tar.gz" % sanitize_name(path.name, "rosbag")
        ascii_name = archive_name.encode("ascii", "ignore").decode("ascii") or "rosbag.tar.gz"
        handler.send_response(HTTPStatus.OK)
        handler.send_header("Content-Type", "application/gzip")
        handler.send_header(
            "Content-Disposition",
            "attachment; filename=\"%s\"; filename*=UTF-8''%s" % (ascii_name, quote(archive_name)),
        )
        handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        handler.send_header("Pragma", "no-cache")
        handler.send_header("Expires", "0")
        handler.send_header("Access-Control-Allow-Origin", "*")
        handler.end_headers()
        try:
            with tarfile.open(fileobj=handler.wfile, mode="w|gz") as archive:
                archive.add(str(path), arcname=path.name, recursive=True)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _start_recording(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        try:
            duration_s = int(float(payload.get("duration_s", 300)))
        except (TypeError, ValueError):
            return self._error("录包时长无效")
        duration_s = max(10, min(3600, duration_s))
        prefix = sanitize_name(str(payload.get("prefix") or "field_test"), "field_test")
        with self._lock:
            scan = dict(self._state.get("scan") or {})
        scan_age_s = max(0.0, time.time() - float(scan.get("last_update") or 0.0))
        if int(scan.get("finite_ranges") or 0) < 20 or scan_age_s > 3.0:
            return self._error("/scan 不新鲜或有效点不足，为避免空包已拒绝开始录制")
        bag_dir = self._recording_bag_root()
        bag_dir.mkdir(parents=True, exist_ok=True)
        with self._recording_lock:
            if self._recording_process is not None and self._recording_process.poll() is None:
                return self._error("已有录包任务正在运行")
            log_path = FsPath("/tmp/m20pro_web_recording.log")
            log_handle = log_path.open("ab", buffering=0)
            unit_name = "m20pro-recording-%s" % random_suffix()
            shell_command = " ".join(
                [
                    "set -e;",
                    "export HOME=/root USER=root LOGNAME=root;",
                    "export ROS_LOG_DIR=/tmp/m20pro_ros_logs;",
                    "mkdir -p \"$ROS_LOG_DIR\";",
                    "source /opt/robot/scripts/setup_ros2.sh >/dev/null;",
                    "cd %s;" % shlex.quote(str(M20PRO_WS_DIR)),
                    "source install/setup.bash;",
                    "export M20PRO_BAG_DIR=%s;" % shlex.quote(str(bag_dir)),
                    "export M20PRO_RECORD_SKIP_CLI_GUARD=1;",
                    "set +e; ros2 run m20pro_bringup m20pro_record_real.sh %s %s;"
                    % (duration_s, shlex.quote(prefix)),
                    "rc=$?; chown -R user:user \"$M20PRO_BAG_DIR\"; exit $rc",
                ]
            )
            command = [
                "systemd-run",
                "--quiet",
                "--wait",
                "--pipe",
                "--collect",
                "--unit",
                unit_name,
                "/bin/bash",
                "-lc",
                shell_command,
            ]
            try:
                process = subprocess.Popen(
                    command,
                    cwd=str(M20PRO_WS_DIR),
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except Exception:
                log_handle.close()
                raise
            self._recording_process = process
            self._recording_log_handle = log_handle
            self._recording_state = {
                "running": True,
                "pid": process.pid,
                "prefix": prefix,
                "duration_s": duration_s,
                "bag_dir": str(bag_dir),
                "log_path": str(log_path),
                "systemd_unit": unit_name,
                "started_at": now_text(),
            }
        return self._recording_status_payload()

    def _stop_recording(self) -> Dict[str, Any]:
        with self._recording_lock:
            process = self._recording_process
            if process is None or process.poll() is not None:
                return self._error("当前没有正在运行的录包任务")
            unit_name = str((self._recording_state or {}).get("systemd_unit") or "")
            if unit_name:
                subprocess.run(["systemctl", "kill", "--signal=SIGINT", unit_name], check=False)
            else:
                os.killpg(process.pid, signal.SIGINT)
            if self._recording_state is not None:
                self._recording_state["stop_requested_at"] = now_text()
        return {"ok": True, "message": "已请求停止录包，正在完成落盘"}

    def _delete_annotation(self, annotation_id: str) -> Dict[str, Any]:
        if not annotation_id:
            return self._error("缺少点位 id")
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") == "running" and annotation_id in (active.get("annotation_ids") or []):
                return self._error("点位正在当前任务中执行，请先停止任务再删除")
            route_dependencies = [
                str(route.get("id") or "")
                for route in self._floor_routes
                if annotation_id
                in (
                    str(route.get("entry_annotation_id") or ""),
                    str(route.get("source_platform_annotation_id") or ""),
                    str(route.get("target_platform_annotation_id") or ""),
                    str(route.get("post_exit_annotation_id") or ""),
                )
            ]
            if route_dependencies:
                return self._error(
                    "点位正在被跨楼层路线使用，请先删除对应路线",
                    {"code": "annotation_used_by_floor_route", "route_ids": route_dependencies},
                )
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

    def _update_annotation(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        annotation_id = str(payload.get("id") or "").strip()
        if not annotation_id:
            return self._error("缺少点位 id")
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") == "running" and annotation_id in (active.get("annotation_ids") or []):
                return self._error("点位正在当前任务中执行，请先停止任务再修改")
            route_dependencies = [
                str(route.get("id") or "")
                for route in self._floor_routes
                if annotation_id
                in (
                    str(route.get("entry_annotation_id") or ""),
                    str(route.get("source_platform_annotation_id") or ""),
                    str(route.get("target_platform_annotation_id") or ""),
                    str(route.get("post_exit_annotation_id") or ""),
                )
            ]
            if route_dependencies:
                return self._error(
                    "点位正在被跨楼层路线使用，请先删除路线后再修改",
                    {"code": "annotation_used_by_floor_route", "route_ids": route_dependencies},
                )
            existing = next((dict(item) for item in self._annotations if item.get("id") == annotation_id), None)
        if existing is None:
            return self._error("点位不存在")

        requested_map_id = str(payload.get("map_id") or existing.get("map_id") or "").strip()
        if requested_map_id != str(existing.get("map_id") or ""):
            return self._error("修改点位不能跨地图迁移，请在目标地图新建点位")
        merged_payload = dict(payload)
        merged_payload["map_id"] = requested_map_id
        context = annotation_create_static_context(merged_payload, default_label_index=1)
        if not context.get("ok"):
            return self._error(str(context["message"]), {"code": context["code"]})
        with self._data_lock:
            map_record = self._find_map_record_unlocked(requested_map_id)
        if map_record is None:
            return self._error("点位绑定的地图不存在", {"code": "annotation_map_missing"})
        identity = self._floor_map_identity_validation(
            context.get("floor"),
            map_record,
            subject="点位楼层",
        )
        if not identity.get("ok"):
            return self._error(
                str(identity["message"]),
                {key: value for key, value in identity.items() if key not in ("ok", "message")},
            )
        target_map_payload = self._map_file_snapshot(requested_map_id)
        map_pose_error = annotation_map_pose_error_payload(dict(context["pose"]), target_map_payload)
        if map_pose_error:
            return self._error(
                str(map_pose_error["message"]),
                {"code": map_pose_error["code"], "detail": map_pose_error["detail"]},
            )
        updated = update_annotation_record(
            existing,
            merged_payload,
            context,
            dwell_s=self._resolve_dwell_s(merged_payload),
            now_text_value=now_text(),
        )
        with self._data_lock:
            self._annotations = [updated if item.get("id") == annotation_id else item for item in self._annotations]
            self._save_json("annotations.json", self._annotations)
        self._append_event("修改地图点位", {"annotation_id": annotation_id, "map_id": requested_map_id})
        return {"ok": True, "annotation": updated}

    def _tasks_payload(self, query: Optional[Dict[str, List[str]]] = None) -> Dict[str, Any]:
        include_all = self._as_bool((query or {}).get("include_all", [False])[0])
        requested_map_id = str(((query or {}).get("map_id") or [""])[0] or "").strip()
        effective_map_id = self._effective_map_id(requested_map_id or None)
        if (not requested_map_id or requested_map_id == "live_map") and effective_map_id:
            self._remember_working_map_id(effective_map_id, reason="tasks_effective_map")
        with self._data_lock:
            active_task = self._settings.get("active_task")
            floor_switch_transaction = self._settings.get("floor_switch_transaction")
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
            navigation_plan = task.get("navigation_plan")
            if not isinstance(navigation_plan, dict) or not navigation_plan.get("ok"):
                navigation_plan = self._build_unified_navigation_plan(
                    task.get("annotation_ids") or [],
                    known_annotations,
                )
            if navigation_plan.get("ok"):
                task["navigation_plan_summary"] = summarize_plan(navigation_plan)
                task["floor_sequence"] = list(navigation_plan.get("floor_sequence") or [])
                task["multi_floor"] = not bool(navigation_plan.get("single_floor"))
            else:
                task["multi_floor"] = len(task["floors"]) > 1
            task.pop("readiness", None)
            task["radar_results"] = self._radar_jobs_for_task(str(task.get("id") or ""))
        return {
            "ok": True,
            "tasks": tasks,
            "hidden_task_count": task_list["hidden_task_count"],
            "total_task_count": task_list["total_task_count"],
            "selected_map_id": selected_map_id,
            "effective_map_id": effective_map_id,
            "requested_map_id": requested_map_id or None,
            "selected_map_status": self._selected_map_status_payload(selected_map_id=selected_map_id),
            "map_relocalization_required": map_relocalization_required,
            "include_all": task_list["include_all"],
            "active_task": active_task,
            "floor_switch_transaction": (
                dict(floor_switch_transaction)
                if isinstance(floor_switch_transaction, dict)
                else None
            ),
            "preflight": preflight,
            "last_preflight_ok": bool(preflight and preflight.get("ok")),
        }

    def _radar_results_dir(self) -> FsPath:
        value = str(self.get_parameter("radar_results_dir").value or "").strip()
        if not value:
            if os.geteuid() == 0 and FsPath("/home/user").exists():
                value = "/home/user/m20pro_radar_results"
            else:
                value = "~/.m20pro_radar_results"
        return FsPath(os.path.expandvars(os.path.expanduser(value)))

    def _radar_manual_dir(self) -> FsPath:
        path = self._radar_results_dir() / "manual"
        path.mkdir(parents=True, exist_ok=True)
        return path

    def _radar_manual_path(self, task_id: str) -> FsPath:
        return self._radar_manual_dir() / ("%s.json" % sanitize_name(task_id, "task"))

    def _load_radar_manual_records(self, task_id: str) -> Dict[str, Any]:
        path = self._radar_manual_path(task_id)
        if not path.exists():
            return {"artifacts": {}, "measurements": {}}
        try:
            with path.open("r", encoding="utf-8") as file:
                payload = json.load(file)
        except Exception:
            return {"artifacts": {}, "measurements": {}}
        if not isinstance(payload, dict):
            return {"artifacts": {}, "measurements": {}}
        artifacts = payload.get("artifacts") if isinstance(payload.get("artifacts"), dict) else {}
        measurements = payload.get("measurements") if isinstance(payload.get("measurements"), dict) else {}
        return {"artifacts": artifacts, "measurements": measurements}

    def _save_radar_manual_records(self, task_id: str, payload: Dict[str, Any]) -> None:
        path = self._radar_manual_path(task_id)
        tmp = path.with_suffix(path.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as file:
            json.dump(payload, file, ensure_ascii=False, indent=2)
        tmp.replace(path)

    @staticmethod
    def _radar_run_id(job: Dict[str, Any]) -> str:
        return sanitize_name(
            "%s_%s_%s"
            % (
                str(job.get("waypoint_key") or "waypoint"),
                str(job.get("scan_mode") or "scan"),
                str(job.get("result_suffix") or job.get("scan_index") or "0"),
            ),
            "radar_run",
        )

    def _decorate_radar_job(self, job: Dict[str, Any], manual: Dict[str, Any]) -> Dict[str, Any]:
        item = dict(job)
        run_id = self._radar_run_id(item)
        item["run_id"] = run_id
        artifacts = manual.get("artifacts") if isinstance(manual.get("artifacts"), dict) else {}
        measurements = manual.get("measurements") if isinstance(manual.get("measurements"), dict) else {}
        artifact = artifacts.get(run_id)
        measurement = measurements.get(run_id)
        if isinstance(artifact, dict):
            item["manual_artifact"] = artifact
            item["artifact_status"] = artifact.get("status") or "imported"
        if isinstance(measurement, dict):
            item["manual_measurement"] = measurement
            item["manual_measure_status"] = "completed"
        elif item.get("manual_measure_required") and not item.get("manual_measure_status"):
            item["manual_measure_status"] = "pending"
        return item

    def _radar_jobs_from_payload(self, path: FsPath, payload: Dict[str, Any]) -> List[Dict[str, Any]]:
        active = payload.get("active_waypoint") or {}
        waypoint = active.get("waypoint") or {}
        results = payload.get("scan_results") if isinstance(payload.get("scan_results"), list) else [payload]
        jobs: List[Dict[str, Any]] = []
        for result in results:
            if not isinstance(result, dict):
                continue
            summary = result.get("summary") or {}
            request = result.get("request") or {}
            jobs.append(
                {
                    "ok": bool(result.get("ok", True)),
                    "job_path": str(path),
                    "waypoint_key": result.get("waypoint_key") or payload.get("waypoint_key"),
                    "taskId": result.get("taskId"),
                    "backend": result.get("backend") or payload.get("backend"),
                    "scan_mode": result.get("scan_mode") or request.get("mode"),
                    "scan_label": result.get("scan_label") or request.get("scanLabel"),
                    "scan_index": result.get("scan_index") if result.get("scan_index") is not None else request.get("scanIndex"),
                    "scan_count": result.get("scan_count") if result.get("scan_count") is not None else request.get("scanCount"),
                    "result_suffix": result.get("result_suffix") or request.get("result_suffix"),
                    "status": result.get("status"),
                    "state": result.get("state"),
                    "progress": result.get("progress"),
                    "artifact_status": result.get("artifact_status"),
                    "artifact_policy": result.get("artifact_policy") or request.get("artifact_policy"),
                    "manual_measure_required": bool(result.get("manual_measure_required") or request.get("manual_measure_required")),
                    "manual_measure_status": result.get("manual_measure_status"),
                    "started_at": result.get("started_at"),
                    "finished_at": result.get("finished_at"),
                    "duration_s": result.get("duration_s"),
                    "scan_released_at": result.get("scan_released_at"),
                    "scan_release_duration_s": result.get("scan_release_duration_s"),
                    "error": result.get("error"),
                    "result_fetch_status": result.get("result_fetch_status"),
                    "result_fetch_error": result.get("result_fetch_error"),
                    "raw_path": result.get("raw_path"),
                    "summary_path": result.get("summary_path"),
                    "task_info_path": result.get("task_info_path"),
                    "downloads": result.get("downloads") or [],
                    "active_waypoint": active,
                    "waypoint": waypoint,
                    "request": request,
                    "summary": summary,
                }
            )
        return jobs

    def _radar_jobs_for_task(self, task_id: str) -> List[Dict[str, Any]]:
        task_id = str(task_id or "").strip()
        if not task_id:
            return []
        jobs_dir = self._radar_results_dir() / "jobs"
        if not jobs_dir.exists():
            return []
        jobs: List[Dict[str, Any]] = []
        prefix = sanitize_name(task_id, "task")
        for path in sorted(jobs_dir.glob(f"{prefix}_*.json")):
            try:
                with path.open("r", encoding="utf-8") as file:
                    payload = json.load(file)
            except Exception as exc:
                jobs.append({"ok": False, "job_path": str(path), "error": str(exc)})
                continue
            if not isinstance(payload, dict):
                continue
            active = payload.get("active_waypoint") or {}
            if str(active.get("task_id") or "") != task_id:
                continue
            jobs.extend(self._radar_jobs_from_payload(path, payload))
        manual = self._load_radar_manual_records(task_id)
        jobs = [self._decorate_radar_job(job, manual) for job in jobs]
        jobs.sort(
            key=lambda item: (
                int((item.get("active_waypoint") or {}).get("index", 0) or 0),
                int(item.get("scan_index", 0) or 0),
                str(item.get("scan_mode") or ""),
            )
        )
        return jobs

    def _radar_all_jobs(self) -> List[Dict[str, Any]]:
        jobs_dir = self._radar_results_dir() / "jobs"
        if not jobs_dir.exists():
            return []
        jobs: List[Dict[str, Any]] = []
        manual_cache: Dict[str, Dict[str, Any]] = {}
        for path in sorted(jobs_dir.glob("*.json")):
            try:
                with path.open("r", encoding="utf-8") as file:
                    payload = json.load(file)
            except Exception as exc:
                jobs.append({"ok": False, "job_path": str(path), "error": str(exc)})
                continue
            if not isinstance(payload, dict):
                continue
            active = payload.get("active_waypoint") if isinstance(payload.get("active_waypoint"), dict) else {}
            task_id = str(active.get("task_id") or payload.get("taskId") or "").strip()
            if task_id not in manual_cache:
                manual_cache[task_id] = self._load_radar_manual_records(task_id) if task_id else {}
            for job in self._radar_jobs_from_payload(path, payload):
                job["task_id"] = task_id
                jobs.append(self._decorate_radar_job(job, manual_cache[task_id]) if task_id else job)
        jobs.sort(
            key=lambda item: (
                str(item.get("finished_at") or ""),
                str(item.get("started_at") or ""),
                str(item.get("taskId") or ""),
            ),
            reverse=True,
        )
        return jobs

    @staticmethod
    def _query_text(query: Dict[str, List[str]], name: str, default: str = "") -> str:
        return str((query.get(name) or [default])[0] or "").strip()

    @staticmethod
    def _query_int(query: Dict[str, List[str]], name: str, default: int, minimum: int, maximum: int) -> int:
        try:
            value = int(float((query.get(name) or [default])[0]))
        except (TypeError, ValueError):
            value = default
        return max(minimum, min(maximum, value))

    @staticmethod
    def _radar_job_metric_count(job: Dict[str, Any]) -> int:
        summary = job.get("summary") if isinstance(job.get("summary"), dict) else {}
        metrics = summary.get("metrics") if isinstance(summary.get("metrics"), list) else []
        return len(metrics)

    def _radar_public_job(self, job: Dict[str, Any]) -> Dict[str, Any]:
        item = dict(job)
        item["metric_count"] = self._radar_job_metric_count(item)
        item["result_unavailable"] = bool(
            item.get("state") == "result_unavailable" or item.get("result_fetch_status") == "failed"
        )
        return item

    def _radar_filter_jobs(self, query: Dict[str, List[str]]) -> Tuple[List[Dict[str, Any]], Dict[str, Any], int]:
        search_query = self._query_text(query, "q")
        task_id = self._query_text(query, "task_id")
        radar_task_id = self._query_text(query, "radar_task_id") or self._query_text(query, "taskId")
        run_id = self._query_text(query, "run_id")
        waypoint_key = self._query_text(query, "waypoint_key")
        status = self._query_text(query, "status")
        state = self._query_text(query, "state")
        mode = self._query_text(query, "mode") or self._query_text(query, "scan_mode")
        offset = self._query_int(query, "offset", 0, 0, 1000000)
        limit = self._query_int(query, "limit", 50, 1, 500)
        jobs = self._radar_all_jobs()
        if search_query:
            jobs = [job for job in jobs if radar_job_matches_query(job, search_query)]
        if task_id:
            jobs = [job for job in jobs if str(job.get("task_id") or "") == task_id]
        if radar_task_id:
            jobs = [job for job in jobs if str(job.get("taskId") or "") == radar_task_id]
        if run_id:
            jobs = [job for job in jobs if str(job.get("run_id") or "") == run_id]
        if waypoint_key:
            jobs = [job for job in jobs if str(job.get("waypoint_key") or "") == waypoint_key]
        if status:
            jobs = [job for job in jobs if str(job.get("status") or "") == status]
        if state:
            jobs = [job for job in jobs if str(job.get("state") or "") == state]
        if mode:
            jobs = [job for job in jobs if str(job.get("scan_mode") or "") == mode]
        filters = {
            "q": search_query or None,
            "task_id": task_id or None,
            "radar_task_id": radar_task_id or None,
            "run_id": run_id or None,
            "waypoint_key": waypoint_key or None,
            "status": status or None,
            "state": state or None,
            "mode": mode or None,
            "offset": offset,
            "limit": limit,
        }
        return jobs[offset : offset + limit], filters, len(jobs)

    def _radar_status_payload(self) -> Dict[str, Any]:
        with self._lock:
            latest = dict(self._state.get("radar_inspection") or {})
            topic_results = dict(self._state.get("radar_inspection_results") or {})
        latest_parsed = latest.get("parsed") if isinstance(latest.get("parsed"), dict) else None
        jobs = [self._radar_public_job(job) for job in self._radar_all_jobs()]
        return {
            "ok": True,
            "results_dir": str(self._radar_results_dir()),
            "latest": latest,
            "latest_parsed": latest_parsed,
            "latest_job": jobs[0] if jobs else None,
            "job_count": len(jobs),
            "topic_result_count": len(topic_results),
        }

    def _radar_manual_start(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        mode = str(payload.get("mode") or "measuring").strip().lower()
        if mode not in ("measuring", "modeling", "both"):
            return self._error("不支持的手动雷达模式")
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
        if active.get("status") == "running":
            return self._error("当前有导航任务正在执行，请先完成任务或停止任务后再手动扫描")
        with self._lock:
            localization_ok = self._state.get("localization_ok") is True
            pose = dict(self._state.get("pose") or {})
            floor = str(self._state.get("floor") or "").strip()
            latest = dict(self._state.get("radar_inspection") or {})
        latest_parsed = latest.get("parsed") if isinstance(latest.get("parsed"), dict) else {}
        latest_status = str(latest_parsed.get("status") or latest_parsed.get("state") or "").strip().lower()
        if latest_status in ("starting", "running", "pending", "analyzing", "analysis_pending"):
            return self._error("雷达当前正在扫描或分析，请等待本次结果完成")
        if not localization_ok:
            return self._error("当前定位未确认，无法用当前位姿启动手动雷达扫描")
        try:
            x = float(pose["x"])
            y = float(pose["y"])
            yaw = float(pose.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError):
            return self._error("当前位姿不可用，无法启动手动雷达扫描")
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return self._error("当前位姿不是有效数值，无法启动手动雷达扫描")
        if not floor:
            return self._error("当前楼层未知，无法启动手动雷达扫描")

        scan_modes = ("measuring", "modeling") if mode == "both" else (mode,)
        scans = [
            {
                "mode": scan_mode,
                "label": "实测实量" if scan_mode == "measuring" else "点云建模",
                "result_suffix": "measure" if scan_mode == "measuring" else "cloud",
                "artifact_policy": "auto_result" if scan_mode == "measuring" else "manual_import",
                "manual_measure_required": scan_mode == "modeling",
                "order": index,
            }
            for index, scan_mode in enumerate(scan_modes)
        ]
        run_id = new_id("manual_radar")
        annotation = {
            "id": run_id,
            "label": "手动雷达扫描",
            "type": "patrol",
            "manual_point_type": "task",
            "floor": floor,
            "pose": {"x": x, "y": y, "z": float(pose.get("z", 0.0) or 0.0), "yaw": yaw},
            "dwell_s": 0.0,
            "radar": {"enabled": True, "scans": scans},
        }
        active_snapshot = {
            "task_id": run_id,
            "task_name": "手动雷达扫描",
            "run_id": run_id,
            "index": 0,
            "status": "running",
            "waypoint_started_at": now_text(),
            "waypoint_started_monotonic": time.monotonic(),
            "last_robot_pose": {"x": x, "y": y, "yaw": yaw},
        }
        self._publish_active_waypoint(annotation, active_snapshot, "dwelling")
        return {
            "ok": True,
            "message": "已按当前位姿启动手动雷达扫描",
            "run_id": run_id,
            "mode": mode,
            "floor": floor,
            "pose": annotation["pose"],
            "waypoint_key": self._active_waypoint_key(annotation, active_snapshot),
        }

    def _radar_results_payload(self, query: Dict[str, List[str]]) -> Dict[str, Any]:
        jobs, filters, total = self._radar_filter_jobs(query)
        return {
            "ok": True,
            "results_dir": str(self._radar_results_dir()),
            "filters": filters,
            "total": total,
            "count": len(jobs),
            "results": [self._radar_public_job(job) for job in jobs],
        }

    def _radar_result_payload(self, query: Dict[str, List[str]]) -> Dict[str, Any]:
        jobs, filters, total = self._radar_filter_jobs(query)
        if not jobs:
            return self._error("未找到雷达结果", {"code": "radar_result_not_found", "filters": filters})
        return {
            "ok": True,
            "results_dir": str(self._radar_results_dir()),
            "filters": filters,
            "total": total,
            "result": self._radar_public_job(jobs[0]),
        }

    def _radar_task_summary(self, jobs: List[Dict[str, Any]]) -> Dict[str, Any]:
        status_counts: Dict[str, int] = {}
        state_counts: Dict[str, int] = {}
        metric_count = 0
        result_unavailable_count = 0
        for job in jobs:
            status = str(job.get("status") or "unknown")
            state = str(job.get("state") or "unknown")
            status_counts[status] = status_counts.get(status, 0) + 1
            state_counts[state] = state_counts.get(state, 0) + 1
            metric_count += self._radar_job_metric_count(job)
            if job.get("state") == "result_unavailable" or job.get("result_fetch_status") == "failed":
                result_unavailable_count += 1
        return {
            "scan_count": len(jobs),
            "metric_count": metric_count,
            "status_counts": status_counts,
            "state_counts": state_counts,
            "result_unavailable_count": result_unavailable_count,
        }

    def _radar_task_payload(self, query: Dict[str, List[str]]) -> Dict[str, Any]:
        task_id = self._query_text(query, "task_id")
        if not task_id:
            return self._error("缺少 task_id", {"code": "radar_task_bad_request"})
        payload = self._radar_task_export_payload(task_id)
        jobs = payload.get("results") if isinstance(payload.get("results"), list) else []
        payload["summary"] = self._radar_task_summary(jobs)
        return payload

    def _radar_task_export_payload(self, task_id: str) -> Dict[str, Any]:
        with self._data_lock:
            task = self._find_by_id(self._tasks, task_id)
        jobs = self._radar_jobs_for_task(task_id)
        return {
            "ok": True,
            "task": task,
            "task_id": task_id,
            "exported_at": now_text(),
            "results_dir": str(self._radar_results_dir()),
            "point_count": len(jobs),
            "results": jobs,
        }

    def _radar_record_artifact(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        task_id = str(payload.get("task_id") or "").strip()
        run_id = str(payload.get("run_id") or "").strip()
        artifact_path = str(payload.get("artifact_path") or payload.get("path") or "").strip()
        if not task_id or not run_id or not artifact_path:
            return self._error("缺少 task_id、run_id 或 artifact_path", {"code": "radar_artifact_bad_request"})
        records = self._load_radar_manual_records(task_id)
        artifacts = records.get("artifacts") if isinstance(records.get("artifacts"), dict) else {}
        artifact = {
            "status": "imported",
            "artifact_path": artifact_path,
            "artifact_type": str(payload.get("artifact_type") or "modeling_project"),
            "note": str(payload.get("note") or ""),
            "recorded_at": now_text(),
        }
        artifacts[run_id] = artifact
        records["artifacts"] = artifacts
        self._save_radar_manual_records(task_id, records)
        return {"ok": True, "artifact": artifact, "task_id": task_id, "run_id": run_id}

    def _radar_save_manual_measurement(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        task_id = str(payload.get("task_id") or "").strip()
        run_id = str(payload.get("run_id") or "").strip()
        measurements = payload.get("measurements")
        if not isinstance(measurements, list):
            measurements = []
        if not task_id or not run_id:
            return self._error("缺少 task_id 或 run_id", {"code": "radar_manual_bad_request"})
        records = self._load_radar_manual_records(task_id)
        saved = {
            "status": "completed",
            "measurements": measurements,
            "operator": str(payload.get("operator") or ""),
            "note": str(payload.get("note") or ""),
            "saved_at": now_text(),
        }
        manual_measurements = records.get("measurements") if isinstance(records.get("measurements"), dict) else {}
        manual_measurements[run_id] = saved
        records["measurements"] = manual_measurements
        self._save_radar_manual_records(task_id, records)
        return {"ok": True, "manual_measurement": saved, "task_id": task_id, "run_id": run_id}

    def _radar_task_export_csv(self, task_id: str) -> bytes:
        payload = self._radar_task_export_payload(task_id)
        output = io.StringIO()
        writer = csv.writer(output)
        writer.writerow(
            [
                "task_id",
                "waypoint_index",
                "waypoint_label",
                "building",
                "unit",
                "house",
                "floor",
                "room",
                "scan_point",
                "radar_task_id",
                "status",
                "state",
                "scan_mode",
                "scan_label",
                "started_at",
                "finished_at",
                "duration_s",
                "scan_released_at",
                "scan_release_duration_s",
                "artifact_status",
                "artifact_path",
                "manual_measure_status",
                "measurement_item_id",
                "measurement_item",
                "location",
                "value",
                "summary_path",
                "raw_path",
                "error",
            ]
        )
        for job in payload.get("results") or []:
            active = job.get("active_waypoint") or {}
            waypoint = job.get("waypoint") or {}
            summary = job.get("summary") or {}
            metrics = summary.get("metrics") if isinstance(summary, dict) else None
            manual_measurement = job.get("manual_measurement") if isinstance(job.get("manual_measurement"), dict) else {}
            manual_metrics = manual_measurement.get("measurements") if isinstance(manual_measurement.get("measurements"), list) else []
            if not isinstance(metrics, list) or not metrics:
                metrics = manual_metrics if manual_metrics else [{}]
            for metric in metrics:
                metric = metric if isinstance(metric, dict) else {}
                writer.writerow(
                    [
                        payload.get("task_id"),
                        active.get("index"),
                        waypoint.get("label") or waypoint.get("id"),
                        waypoint.get("building"),
                        waypoint.get("unit"),
                        waypoint.get("house"),
                        waypoint.get("floor"),
                        waypoint.get("room"),
                        waypoint.get("scan_point"),
                        job.get("taskId"),
                        job.get("status"),
                        job.get("state"),
                        job.get("scan_mode"),
                        job.get("scan_label"),
                        job.get("started_at"),
                        job.get("finished_at"),
                        job.get("duration_s"),
                        job.get("scan_released_at"),
                        job.get("scan_release_duration_s"),
                        job.get("artifact_status"),
                        (job.get("manual_artifact") or {}).get("artifact_path") if isinstance(job.get("manual_artifact"), dict) else "",
                        job.get("manual_measure_status"),
                        metric.get("measurementItemId") or metric.get("measurement_item_id") or metric.get("id"),
                        metric.get("measurementItem") or metric.get("measurement_item") or metric.get("name"),
                        metric.get("location"),
                        metric.get("displayValue") or metric.get("rawValue") or metric.get("value"),
                        job.get("summary_path"),
                        job.get("raw_path") or job.get("task_info_path"),
                        job.get("error") or job.get("result_fetch_error"),
                    ]
                )
        return output.getvalue().encode("utf-8-sig")

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
                ["/map_server", "/controller_server", "/planner_server", "/bt_navigator"]
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

    def _request_command_mux_mode(self, mode: str) -> None:
        message = String()
        message.data = str(mode)
        self.command_mux_mode_pub.publish(message)

    def _set_command_mux_mode(self, mode: str, timeout_s: float = 1.0) -> Dict[str, Any]:
        client = self.command_mux_clients.get(str(mode))
        if client is None:
            return self._error("未知速度仲裁模式", {"code": "command_mux_mode_invalid"})
        timeout_s = max(0.1, min(2.0, float(timeout_s)))
        if not client.wait_for_service(timeout_sec=min(0.5, timeout_s)):
            return self._error(
                "速度仲裁器未就绪，已禁止下发运动指令",
                {"code": "command_mux_unavailable", "mode": mode},
            )
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return self._error(
                "速度仲裁器切换超时，已禁止下发运动指令",
                {"code": "command_mux_timeout", "mode": mode},
            )
        try:
            response = future.result()
        except Exception as exc:
            return self._error(
                "速度仲裁器切换失败：%s" % exc,
                {"code": "command_mux_failed", "mode": mode},
            )
        if response is None or not bool(response.success):
            return self._error(
                str(getattr(response, "message", "") or "速度仲裁器拒绝切换"),
                {"code": "command_mux_rejected", "mode": mode},
            )
        return {"ok": True, "mode": mode, "message": str(response.message or "")}

    def _teleop_status_payload(self) -> Dict[str, Any]:
        now_monotonic = time.monotonic()
        with self._teleop_lock:
            session = dict(self._teleop_session)
            active = bool(session.get("active"))
            last_heartbeat = session.get("last_heartbeat_monotonic")
            command = dict(session.get("command") or {})
            acquiring = bool(self._teleop_acquiring)
        with self._lock:
            mux_status = dict(self._state.get("command_mux_status") or {})
        mux_parsed = mux_status.get("parsed") if isinstance(mux_status.get("parsed"), dict) else {}
        heartbeat_age = None
        if last_heartbeat is not None:
            heartbeat_age = max(0.0, now_monotonic - float(last_heartbeat))
        return {
            "available": bool(mux_parsed),
            "active": active,
            "acquiring": acquiring,
            "status": "active" if active else ("acquiring" if acquiring else "inactive"),
            "mux_mode": str(mux_parsed.get("mode") or "unknown"),
            "mux_reason": mux_parsed.get("reason"),
            "heartbeat_age_s": heartbeat_age,
            "command_timeout_s": float(self.get_parameter("teleop_command_timeout_s").value),
            "command": command,
            "last_stop_reason": session.get("last_stop_reason"),
            "limits": {
                "forward_mps": float(self.get_parameter("teleop_max_forward_speed_mps").value),
                "reverse_mps": float(self.get_parameter("teleop_max_reverse_speed_mps").value),
                "lateral_mps": float(self.get_parameter("teleop_max_lateral_speed_mps").value),
                "angular_radps": float(self.get_parameter("teleop_max_angular_speed_radps").value),
            },
        }

    def _publish_teleop_zero(self, samples: int = 3) -> None:
        for index in range(max(1, int(samples))):
            self.teleop_cmd_vel_pub.publish(Twist())
            if index + 1 < samples:
                time.sleep(0.02)

    def _acquire_teleop(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if payload.get("confirm") is not True:
            return self._error(
                "人工接管会终止当前自主任务，请显式确认",
                {"code": "teleop_confirmation_required"},
            )
        with self._teleop_lock:
            if self._teleop_session.get("active") or self._teleop_acquiring:
                return self._error(
                    "已有操作端正在接管机器狗",
                    {"code": "teleop_busy"},
                )
            self._teleop_acquiring = True
        try:
            motion = self._detect_motion_mode()
            if motion.get("mode") != "move":
                return self._error(
                    "当前不是 move 运动模式，不能启用网页遥控",
                    {"code": "teleop_motion_mode_blocked", "motion": motion},
                )
            stop_result = self._stop_task({"reason": "web_teleop_takeover"})
            if not stop_result.get("ok"):
                return stop_result
            lock_result = self._set_command_mux_mode("locked")
            if not lock_result.get("ok"):
                self._request_command_mux_mode("locked")
                return lock_result
            mux_result = self._set_command_mux_mode("teleop")
            if not mux_result.get("ok"):
                self._request_command_mux_mode("locked")
                return mux_result
            session_id = new_id("teleop")
            with self._teleop_lock:
                self._teleop_session = {
                    "active": True,
                    "session_id": session_id,
                    "started_at": now_text(),
                    "last_heartbeat_monotonic": time.monotonic(),
                    "last_sequence": -1,
                    "command": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
                    "last_stop_reason": None,
                }
            self._append_event("网页人工接管", {"source": "web_operator"})
            return {
                "ok": True,
                "session_id": session_id,
                "teleoperation": self._teleop_status_payload(),
                "message": "已终止自主任务并进入人工接管",
            }
        finally:
            with self._teleop_lock:
                self._teleop_acquiring = False

    def _teleop_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        session_id = str(payload.get("session_id") or "").strip()
        sequence = payload.get("sequence")
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            return self._error("遥控指令缺少有效序号", {"code": "teleop_sequence_invalid"})
        try:
            command = normalized_teleop_command(
                payload,
                max_forward_speed_mps=float(
                    self.get_parameter("teleop_max_forward_speed_mps").value
                ),
                max_reverse_speed_mps=float(
                    self.get_parameter("teleop_max_reverse_speed_mps").value
                ),
                max_lateral_speed_mps=float(
                    self.get_parameter("teleop_max_lateral_speed_mps").value
                ),
                max_angular_speed_radps=float(
                    self.get_parameter("teleop_max_angular_speed_radps").value
                ),
            )
        except (TypeError, ValueError, OverflowError) as exc:
            return self._error(str(exc), {"code": "teleop_command_invalid"})
        with self._teleop_lock:
            if not self._teleop_session.get("active"):
                return self._error("人工接管未启用或已超时", {"code": "teleop_inactive"})
            if session_id != str(self._teleop_session.get("session_id") or ""):
                return self._error("遥控会话不匹配", {"code": "teleop_session_mismatch"})
            last_sequence_value = self._teleop_session.get("last_sequence")
            last_sequence = -1 if last_sequence_value is None else int(last_sequence_value)
            if sequence <= last_sequence:
                return {
                    "ok": True,
                    "ignored": True,
                    "reason": "stale_sequence",
                    "last_sequence": last_sequence,
                }
            msg = Twist()
            msg.linear.x = float(command["linear_x"])
            msg.linear.y = float(command["linear_y"])
            msg.angular.z = float(command["angular_z"])
            self.teleop_cmd_vel_pub.publish(msg)
            self._teleop_session["last_heartbeat_monotonic"] = time.monotonic()
            self._teleop_session["last_sequence"] = sequence
            self._teleop_session["command"] = command
        return {"ok": True, "sequence": sequence, "command": command}

    def _publish_motion_state_command(self, action: str) -> Dict[str, Any]:
        action = str(action).strip().lower()
        if action not in ("stand", "lie", "soft_stop"):
            return self._error(
                "未知运动状态动作",
                {"code": "motion_state_action_invalid", "action": action},
            )
        # One command at a time keeps a late response from one posture change
        # from being mistaken for the response to the next one.
        if not self._motion_state_command_lock.acquire(blocking=False):
            return self._error(
                "上一条运动状态指令仍在等待原厂回执",
                {"code": "motion_state_command_busy"},
            )
        try:
            discovery_timeout_s = max(
                0.1, float(self.get_parameter("motion_state_discovery_timeout_s").value)
            )
            deadline = time.monotonic() + discovery_timeout_s
            while rclpy.ok() and self.motion_state_cmd_pub.get_subscription_count() <= 0:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return self._error(
                        "运动状态控制接口未就绪，已禁止下发动作",
                        {"code": "motion_state_command_unavailable", "timeout_s": discovery_timeout_s},
                    )
                time.sleep(min(0.05, remaining))
            if self.motion_state_cmd_pub.get_subscription_count() <= 0:
                return self._error(
                    "运动状态控制接口未就绪，已禁止下发动作",
                    {"code": "motion_state_command_unavailable", "timeout_s": discovery_timeout_s},
                )

            with self._motion_state_result_condition:
                result_seq = self._motion_state_result_seq
            message = String()
            message.data = action
            self.motion_state_cmd_pub.publish(message)
            result_timeout_s = max(
                0.1, float(self.get_parameter("motion_state_result_timeout_s").value)
            )
            deadline = time.monotonic() + result_timeout_s
            with self._motion_state_result_condition:
                while self._motion_state_result_seq <= result_seq:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        return self._error(
                            "运动状态指令已发送，但未收到 TCP 桥回执",
                            {
                                "code": "motion_state_result_timeout",
                                "action": action,
                                "timeout_s": result_timeout_s,
                            },
                        )
                    self._motion_state_result_condition.wait(timeout=remaining)
            with self._lock:
                result_text = str(self._state.get("motion_state_result") or "")
            if result_text.startswith("accepted:"):
                return {
                    "ok": True,
                    "action": action,
                    "pending": False,
                    "confirmed": True,
                    "result": result_text,
                }
            if result_text.startswith("sent_without_ack:"):
                return self._error(
                    "原厂未返回运动状态回执，未确认动作是否执行",
                    {"code": "motion_state_no_ack", "action": action, "result": result_text},
                )
            return self._error(
                "原厂拒绝运动状态动作",
                {"code": "motion_state_rejected", "action": action, "result": result_text},
            )
        finally:
            self._motion_state_command_lock.release()

    def _teleop_motion(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        action = str(payload.get("action") or "").strip().lower().replace("-", "_")
        aliases = {
            "stand": "stand",
            "standing": "stand",
            "起立": "stand",
            "lie": "lie",
            "lying": "lie",
            "趴下": "lie",
            "soft_stop": "soft_stop",
            "soft_emergency_stop": "soft_stop",
            "软急停": "soft_stop",
        }
        action = aliases.get(action, action)
        if action not in ("stand", "lie", "soft_stop"):
            return self._error(
                "未知运动状态动作",
                {"code": "motion_state_action_invalid", "action": action},
            )

        if action == "soft_stop":
            release = self._release_teleop(
                payload,
                force=True,
                reason="web_soft_emergency_stop",
            )
            if not release.get("ok"):
                return release
            stop_result = self._stop_task({"reason": "web_soft_emergency_stop"})
            motion = self._publish_motion_state_command("soft_stop")
            if not motion.get("ok"):
                return motion
            self._append_event("网页软急停", {"source": "web_operator"})
            return {
                "ok": True,
                "active_task": None,
                "teleoperation": self._teleop_status_payload(),
                "motion_state": motion,
                "message": str(stop_result.get("message") or "已执行软急停，运动已锁定"),
            }

        session_id = str(payload.get("session_id") or "").strip()
        with self._teleop_lock:
            active = bool(self._teleop_session.get("active"))
            expected = str(self._teleop_session.get("session_id") or "")
        if not active:
            return self._error("人工接管未启用，请先接管后执行运动状态动作", {"code": "teleop_inactive"})
        if session_id != expected:
            return self._error("遥控会话不匹配", {"code": "teleop_session_mismatch"})

        self._publish_teleop_zero()
        release = self._release_teleop(
            {"session_id": session_id},
            reason="web_motion_state_%s" % action,
        )
        if not release.get("ok"):
            return release
        motion = self._publish_motion_state_command(action)
        if not motion.get("ok"):
            return motion
        self._append_event(
            "网页运动状态动作",
            {"action": action, "source": "web_operator"},
        )
        return {
            "ok": True,
            "teleoperation": self._teleop_status_payload(),
            "motion_state": motion,
            "message": "已停止遥控并下发%s动作；动作完成后请重新人工接管" % ("起立" if action == "stand" else "趴下"),
        }

    def _release_teleop(
        self,
        payload: Dict[str, Any],
        *,
        force: bool = False,
        reason: str = "operator_release",
    ) -> Dict[str, Any]:
        session_id = str(payload.get("session_id") or "").strip()
        with self._teleop_lock:
            active = bool(self._teleop_session.get("active"))
            expected_session_id = str(self._teleop_session.get("session_id") or "")
            decision = teleop_release_decision(
                active=active,
                force=force,
                request_session_id=session_id,
                active_session_id=expected_session_id,
            )
            if not decision.get("ok"):
                return self._error("遥控会话不匹配", {"code": decision["code"]})
            if not decision.get("lock_mux"):
                return {
                    "ok": True,
                    "released": False,
                    "command_mux_confirmed": False,
                    "teleoperation": self._teleop_status_payload(),
                    "message": "人工接管已结束",
                }
            self._teleop_session["active"] = False
            self._teleop_session["last_stop_reason"] = reason
            self._teleop_session["command"] = {
                "linear_x": 0.0,
                "linear_y": 0.0,
                "angular_z": 0.0,
            }
            self._publish_teleop_zero()
        self._request_command_mux_mode("locked")
        mux_result = self._set_command_mux_mode("locked")
        if active:
            self._append_event("结束网页人工接管", {"reason": reason})
        return {
            "ok": True,
            "released": active,
            "command_mux_confirmed": bool(mux_result.get("ok")),
            "teleoperation": self._teleop_status_payload(),
            "message": "人工接管已结束，自主导航保持锁定；需重新开始任务才会恢复",
        }

    def _emergency_stop_teleop(self) -> Dict[str, Any]:
        self._release_teleop({}, force=True, reason="web_emergency_stop")
        result = self._stop_task({"reason": "web_emergency_stop"})
        return {
            "ok": True,
            "active_task": None,
            "teleoperation": self._teleop_status_payload(),
            "message": str(result.get("message") or "已停止任务和所有网页运动指令"),
        }

    def _tick_teleop_lease(self) -> None:
        timeout_s = max(0.1, float(self.get_parameter("teleop_command_timeout_s").value))
        expired = False
        with self._teleop_lock:
            last_heartbeat = self._teleop_session.get("last_heartbeat_monotonic")
            if (
                self._teleop_session.get("active")
                and last_heartbeat is not None
                and time.monotonic() - float(last_heartbeat) > timeout_s
            ):
                self._teleop_session["active"] = False
                self._teleop_session["last_stop_reason"] = "browser_lease_timeout"
                self._teleop_session["command"] = {
                    "linear_x": 0.0,
                    "linear_y": 0.0,
                    "angular_z": 0.0,
                }
                self._publish_teleop_zero()
                expired = True
        if expired:
            self._request_command_mux_mode("locked")
            self._append_event(
                "网页遥控心跳超时自动停车",
                {"timeout_s": timeout_s},
            )

    def _reset_navigation_session(
        self,
        reason: str,
        clear_costmaps: bool = True,
        publish_idle: bool = True,
    ) -> None:
        self._request_command_mux_mode("locked")
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
            local_version = int(self._state.get("local_path", {}).get("version", 0) or 0)
            self._state["local_path"] = {
                "version": local_version + 1,
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
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
        if connector_owns_navigation_status(active):
            return
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

    def _resolve_active_connector_transition(
        self,
        active: Dict[str, Any],
        annotation: Dict[str, Any],
        source_floor: str,
    ) -> Dict[str, Any]:
        """Resolve the next directed connector from the canonical route store."""
        with self._data_lock:
            known = {
                str(item.get("id")): dict(item)
                for item in self._annotations
                if item.get("id")
            }
        plan = self._build_unified_navigation_plan(active.get("annotation_ids") or [], known)
        if not plan.get("ok"):
            return {
                "ok": False,
                "code": str(plan.get("code") or "navigation_plan_invalid"),
                "message": str(plan.get("message") or "统一导航计划无法解析楼层连接"),
            }
        transition = runtime_transition_for_annotation(
            plan,
            annotation.get("id"),
            current_floor=source_floor,
        )
        if transition.get("action") != "transition":
            return {
                "ok": False,
                "code": str(transition.get("code") or "navigation_plan_transition_missing"),
                "message": str(transition.get("message") or "统一导航计划缺少下一条楼层连接"),
                "transition": transition,
            }
        edges = [item for item in transition.get("edges") or [] if isinstance(item, dict)]
        if not edges or not isinstance(edges[0].get("route"), dict):
            return {
                "ok": False,
                "code": "navigation_plan_route_geometry_missing",
                "message": "统一导航计划的下一条连接缺少路线几何",
                "transition": transition,
            }
        return {
            "ok": True,
            "transition": transition,
            "edge": edges[0],
            "route": dict(edges[0]["route"]),
        }

    def _publish_stair_connector_start(
        self,
        *,
        active: Dict[str, Any],
        route: Dict[str, Any],
        request_id: str,
        plan_id: str,
        map_epoch: int,
    ) -> None:
        message = String()
        message.data = json.dumps(
            {
                "request_id": request_id,
                "plan_id": plan_id,
                "map_epoch": int(map_epoch),
                "route": route,
                "task_id": active.get("task_id"),
                "run_id": active.get("run_id"),
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.stair_executor_start_pub.publish(message)
        self.get_logger().info(
            "web task started stair connector request=%s route=%s plan=%s"
            % (request_id, route.get("id"), plan_id)
        )

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
        with self._lock:
            current_pose = dict(self._state.get("pose") or {})
        current_pose_age_s = pose_age_sec(current_pose, time.time())
        pose_timeout_s = max(0.5, float(self.get_parameter("task_start_pose_timeout_s").value))
        fresh_pose_distance_m = None
        if (
            is_plausible_pose_dict(current_pose)
            and current_pose_age_s is not None
            and current_pose_age_s <= pose_timeout_s
        ):
            fresh_pose_distance_m = pose_distance_m(current_pose, annotation.get("pose"))
        decision = nav_success_completion_decision(
            active,
            annotation,
            status_text,
            goal_tolerance_m=float(self.get_parameter("goal_reached_tolerance_m").value),
            fresh_pose_distance_m=fresh_pose_distance_m,
            fresh_pose_age_s=current_pose_age_s,
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

    def _build_unified_navigation_plan(
        self,
        annotation_ids: Any,
        annotations_by_id: Dict[str, Dict[str, Any]],
    ) -> Dict[str, Any]:
        """Build the one task plan shape used by every task entry point."""
        config_payload = self._floor_config_payload()
        routes = (
            stair_routes_from_config(config_payload.get("config") or {})
            if config_payload.get("ok")
            else []
        )
        return build_unified_navigation_plan(
            annotation_ids,
            annotations_by_id=annotations_by_id,
            routes=routes,
        )

    def _task_navigation_plan_state(
        self,
        task: Dict[str, Any],
        annotations_by_id: Dict[str, Dict[str, Any]],
    ) -> Dict[str, Any]:
        """Validate/migrate the one plan used by task execution."""
        config_payload = self._floor_config_payload()
        routes = (
            stair_routes_from_config(config_payload.get("config") or {})
            if config_payload.get("ok")
            else []
        )
        return task_navigation_plan_state(
            task,
            annotations_by_id=annotations_by_id,
            routes=routes,
        )

    def _create_task(
        self,
        payload: Dict[str, Any],
        task_metadata: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
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
            unified_plan = self._build_unified_navigation_plan(
                static_context.get("annotation_ids") or [],
                known,
            )
            if not unified_plan.get("ok"):
                return self._error(
                    str(unified_plan.get("message") or "统一导航计划无效"),
                    {
                        "code": str(unified_plan.get("code") or "navigation_plan_invalid"),
                        "navigation_plan": summarize_plan(unified_plan),
                    },
                )
            task = build_task_create_record(
                static_context,
                task_id=new_id("task"),
                now_text_value=now_text(),
            )
            if task_metadata:
                task.update(task_metadata)
            # Compatibility fields remain projections of the unified plan;
            # they are never an independent source of route truth.
            task["navigation_plan"] = navigation_plan_record(unified_plan)
            task["navigation_plan_summary"] = summarize_plan(unified_plan)
            task["floor_sequence"] = list(unified_plan.get("floor_sequence") or [])
            task["route_plans"] = list(unified_plan.get("transition_paths") or [])
            task["multi_floor"] = not bool(unified_plan.get("single_floor"))
            self._tasks.append(task)
            self._save_json("tasks.json", self._tasks)
        return {"ok": True, "task": task}

    def _create_cross_floor_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        config_payload = self._floor_config_payload()
        if not config_payload.get("ok"):
            return self._error(
                str(config_payload.get("message") or "跨楼层配置不可用"),
                {
                    "code": "cross_floor_config_unavailable",
                    "path": config_payload.get("path"),
                    "error": config_payload.get("error"),
                },
            )
        with self._data_lock:
            known = {
                str(item.get("id")): dict(item)
                for item in self._annotations
                if item.get("id")
            }
        context = cross_floor_task_context(
            payload,
            annotations_by_id=known,
            routes=stair_routes_from_config(config_payload["config"]),
        )
        if not context.get("ok"):
            return self._error(
                str(context.get("message") or "跨楼层任务无效"),
                {key: value for key, value in context.items() if key not in ("ok", "message")},
            )
        created = self._create_task(
            {
                "name": context["name"],
                "annotation_ids": context["annotation_ids"],
                "map_id": context["task_map_id"],
            },
            task_metadata={
                "multi_floor": True,
                "floor_sequence": list(context["floor_sequence"]),
                "route_plans": list(context["route_plans"]),
            },
        )
        if not created.get("ok"):
            return created
        task = created["task"]
        self._append_event(
            "建立跨楼层任务",
            {
                "task_id": task.get("id"),
                "floors": context["floor_sequence"],
                "waypoint_count": len(context["annotation_ids"]),
            },
        )
        return {
            "ok": True,
            "task": task,
            "route_plans": context["route_plans"],
            "navigation_plan": task.get("navigation_plan"),
            "navigation_plan_summary": task.get("navigation_plan_summary"),
        }

    def _task_start_pre_runtime_context(
        self,
        payload: Dict[str, Any],
        *,
        require_nav_alignment: bool = False,
    ) -> Dict[str, Any]:
        with self._lock:
            localization_ok = self._state.get("localization_ok")
            pose = dict(self._state.get("pose") or {})
        with self._data_lock:
            map_relocalization_required = self._settings.get("map_relocalization_required")
        localization_gate = task_start_localization_gate_decision(
            localization_ok=localization_ok,
            pose=pose,
            pose_age=pose_age_sec(pose, time.time()),
            pose_timeout_s=float(self.get_parameter("task_start_pose_timeout_s").value),
            map_relocalization_required=map_relocalization_required,
        )
        if localization_gate.get("action") != "pass":
            return self._error(
                str(localization_gate.get("message") or "当前定位未满足任务启动条件"),
                {
                    "code": str(localization_gate.get("code") or "task_start_localization_not_ready"),
                    "localization_ok": localization_ok,
                    "pose_age_s": localization_gate.get("pose_age_s"),
                    "pose_timeout_s": localization_gate.get("pose_timeout_s"),
                },
            )
        odom_alignment = None
        if require_nav_alignment:
            with self._lock:
                local_costmap = dict(self._state.get("local_costmap") or {})
                odom = dict(self._state.get("odom") or {})
                if isinstance(odom.get("pose"), dict):
                    odom["pose"] = dict(odom["pose"])
            odom_alignment = local_costmap_odom_alignment_payload(
                local_costmap=local_costmap,
                odom=odom,
                tolerance_m=float(
                    self.get_parameter("task_start_costmap_odom_tolerance_m").value
                ),
            )
            if not odom_alignment.get("ready"):
                return self._error(
                    str(odom_alignment.get("message") or "Nav2 里程计与局部代价地图未对齐"),
                    {
                        "code": str(
                            odom_alignment.get("code")
                            or "local_costmap_alignment_unavailable"
                        ),
                        "odom_alignment": odom_alignment,
                    },
                )
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
            task_plan_state = None
            if static_context.get("ok"):
                task_validation = self._validate_task_annotations_for_map(
                    list(static_context.get("annotations") or []),
                    str(static_context["task_map_id"]),
                )
                if not task_validation:
                    task_plan_state = self._task_navigation_plan_state(task, known)
                    if not task_plan_state.get("ok"):
                        task_validation = {
                            "code": str(task_plan_state.get("code") or "navigation_plan_invalid"),
                            "message": str(task_plan_state.get("message") or "统一导航计划无效"),
                            "task_plan": task_plan_state,
                        }
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
            if task_plan_state and task_plan_state.get("ok"):
                plan = task_plan_state.get("plan") if isinstance(task_plan_state.get("plan"), dict) else {}
                record = task_plan_state.get("record") if isinstance(task_plan_state.get("record"), dict) else {}
                projection = {
                    "navigation_plan": record,
                    "navigation_plan_summary": summarize_plan(plan),
                    "floor_sequence": list(plan.get("floor_sequence") or []),
                    "route_plans": list(plan.get("transition_paths") or []),
                    "multi_floor": not bool(plan.get("single_floor")),
                }
                if any(task.get(key) != value for key, value in projection.items()):
                    task.update(projection)
                    self._save_json("tasks.json", self._tasks)
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
                "annotations": list(static_context.get("annotations") or []),
                "odom_alignment": odom_alignment,
            }

    def _start_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._teleop_lock:
            if self._teleop_session.get("active") or self._teleop_acquiring:
                return self._error(
                    "人工遥控正在接管，请先结束接管再开始任务",
                    {"code": "task_blocked_by_teleop"},
                )
        context = self._task_start_pre_runtime_context(payload)
        if not context.get("ok"):
            return context
        self._reset_navigation_session("before_start_task", clear_costmaps=True)
        settle_s = max(0.0, float(self.get_parameter("task_start_settle_s").value))
        if settle_s > 0.0:
            time.sleep(min(settle_s, 2.0))
        context = self._task_start_pre_runtime_context(
            payload,
            require_nav_alignment=True,
        )
        if not context.get("ok"):
            return context
        task = context.get("task")
        task_id = str(context.get("task_id") or "")
        task_map_id = str(context.get("task_map_id") or "live_map")
        task_for_start = dict(task or {})
        task_for_start["multi_floor"] = task_uses_multiple_floors(
            task_for_start,
            context.get("annotations") or [],
        )
        with self._teleop_lock:
            if self._teleop_session.get("active") or self._teleop_acquiring:
                return self._error(
                    "人工遥控已开始接管，本次任务未启动",
                    {"code": "task_blocked_by_teleop"},
                )
            mux_result = self._set_command_mux_mode("navigation")
            if not mux_result.get("ok"):
                return mux_result
            with self._data_lock:
                created = create_active_task_state(
                    task_for_start,
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

    def _one_key_charge(self) -> Dict[str, Any]:
        with self._teleop_lock:
            if self._teleop_session.get("active") or self._teleop_acquiring:
                return self._error(
                    "人工遥控正在接管，请先结束接管后再启动充电",
                    {"code": "charge_blocked_by_teleop"},
                )
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") == "running":
                return self._error(
                    "当前任务正在执行，请先停止任务后再启动充电",
                    {"code": "charge_blocked_by_task", "task_id": active.get("task_id")},
                )
            selected_map_id = self._effective_map_id()
            charge_points = [
                dict(item)
                for item in self._annotations
                if str(item.get("map_id") or "") == str(selected_map_id or "")
                and str(item.get("manual_point_type") or item.get("point_type") or "").strip().lower()
                in ("charge", "charging")
            ]
            generated = [
                dict(task)
                for task in self._tasks
                if task.get("system_generated") == "one_key_charge"
                and str(task.get("map_id") or "") == str(selected_map_id or "")
            ]
        if not selected_map_id:
            return self._error("当前没有选中的地图，无法启动充电")
        if len(charge_points) != 1:
            return self._error(
                "当前地图必须且只能有一个充电点，才能使用一键充电",
                {"code": "charge_point_ambiguous", "count": len(charge_points)},
            )
        charge_id = str(charge_points[0].get("id") or "")
        task = next(
            (
                item
                for item in generated
                if list(item.get("annotation_ids") or []) == [charge_id]
            ),
            None,
        )
        if task is None:
            created = self._create_task(
                {
                    "name": "一键充电",
                    "annotation_ids": [charge_id],
                    "map_id": selected_map_id,
                },
                task_metadata={"system_generated": "one_key_charge"},
            )
            if not created.get("ok"):
                return created
            task = created.get("task")
        task_id = str((task or {}).get("id") or "")
        if not task_id:
            return self._error("一键充电任务创建失败", {"code": "charge_task_invalid"})
        result = self._start_task({"task_id": task_id})
        if result.get("ok"):
            result["message"] = "已按当前地图充电点启动一键充电任务"
            result["charge_task_id"] = task_id
        return result

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

    def _publish_initialpose(
        self,
        payload: Dict[str, Any],
        *,
        allow_active_task: bool = False,
        event_text: str = "网页发布重定位",
        pose_tolerance_m: Optional[float] = None,
        yaw_tolerance_rad: Optional[float] = None,
        require_lifecycle: bool = False,
        stability_window_s: float = 0.0,
    ) -> Dict[str, Any]:
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") == "running" and not allow_active_task:
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
        with self._data_lock:
            selected_map_id = self._effective_map_id()
            selected_map = self._find_map_record_unlocked(selected_map_id)
        if selected_map is not None:
            floor = floor or str(selected_map.get("floor") or "").strip()
            identity = self._floor_map_identity_validation(
                floor,
                selected_map,
                subject="重定位楼层",
            )
        else:
            identity = self._floor_identity_validation(floor, subject="重定位楼层")
        if not identity.get("ok"):
            return self._error(
                str(identity["message"]),
                {key: value for key, value in identity.items() if key not in ("ok", "message")},
            )
        attempt_id = "reloc-%d" % int(request_started_at * 1000.0)
        with self._lock:
            self._state["relocalization_attempt"] = {
                "id": attempt_id,
                "status": "pending",
                "started_at": request_started_at,
                "requested_pose": dict(pose),
                "message": "正在等待本次重定位的原厂确认",
            }
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
            pose_tolerance_m=pose_tolerance_m,
            yaw_tolerance_rad=yaw_tolerance_rad,
            require_lifecycle=require_lifecycle,
            stability_window_s=stability_window_s,
        )
        if bool(verification.get("factory_pose_accepted")):
            with self._data_lock:
                self._settings.pop("map_relocalization_required", None)
                self._save_json("settings.json", self._settings)
        status = relocalization_response_payload(
            verification,
            now_text=now_text,
        )
        attempt_status = "confirmed" if bool(status.get("confirmed")) else "failed"
        with self._lock:
            attempt = dict(self._state.get("relocalization_attempt") or {})
            attempt.update(
                {
                    "id": attempt.get("id") or attempt_id,
                    "status": attempt_status,
                    "started_at": request_started_at,
                    "completed_at": time.time(),
                    "requested_pose": dict(pose),
                    "code": status.get("code"),
                    "message": status.get("message"),
                    "verification": verification,
                }
            )
            self._state["relocalization_attempt"] = attempt
        result = initialpose_api_response_payload(
            localization_status=status,
            verification=verification,
            topic=self.get_parameter("initialpose_topic").value,
            publish_repeats=repeats,
            frame_id=frame_id,
            floor=floor,
            pose={"x": x, "y": y, "z": z, "yaw": yaw},
        )
        self._append_event(str(event_text or "网页发布重定位"), result)
        return result

    def _wait_for_relocalization_verification(
        self,
        request_started_at: float,
        requested_pose: Dict[str, float],
        *,
        pose_tolerance_m: Optional[float] = None,
        yaw_tolerance_rad: Optional[float] = None,
        require_lifecycle: bool = False,
        stability_window_s: float = 0.0,
    ) -> Dict[str, Any]:
        timeout_s = max(0.5, float(self.get_parameter("relocalization_verify_timeout_s").value))
        pose_tolerance_m = max(
            0.1,
            float(
                self.get_parameter("relocalization_pose_tolerance_m").value
                if pose_tolerance_m is None
                else pose_tolerance_m
            ),
        )
        if yaw_tolerance_rad is not None:
            yaw_tolerance_rad = max(0.02, float(yaw_tolerance_rad))
        stable_window = max(0.0, float(stability_window_s))
        deadline = time.time() + timeout_s
        stable_since: Optional[float] = None
        previous_stable_pose: Optional[Dict[str, float]] = None
        stability_confirmed = stable_window <= 0.0
        evidence: Dict[str, Any] = {
            "tcp_2101_accepted": False,
            "tcp_2101_ambiguous": False,
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
                localization = self._raw_factory_localization_ok(self._state)
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
                yaw_tolerance_rad=yaw_tolerance_rad,
            )

            stability = relocalization_stability_step(
                evidence=evidence,
                current_pose=pose,
                previous_pose=previous_stable_pose,
                stable_since=stable_since,
                now_time=time.time(),
                stability_window_s=stable_window,
                pose_tolerance_m=pose_tolerance_m,
                yaw_tolerance_rad=yaw_tolerance_rad,
            )
            stable_since = stability.get("stable_since")
            previous_stable_pose = stability.get("previous_stable_pose")
            if stability.get("stable"):
                stability_confirmed = True
                break
            time.sleep(0.2)

        navigation_readiness = self._navigation_readiness_payload(check_lifecycle=require_lifecycle)
        return manual_relocalization_verification_payload(
            tcp_2101_accepted=bool(evidence.get("tcp_2101_accepted")),
            tcp_2101_ambiguous=bool(evidence.get("tcp_2101_ambiguous")),
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
            tcp_pose_near_request=evidence.get("tcp_pose_near_request"),
            tcp_pose_error_m=evidence.get("tcp_pose_error_m"),
            tcp_yaw_error_rad=evidence.get("tcp_yaw_error_rad"),
            pose_tolerance_m=pose_tolerance_m,
            stability_confirmed=stability_confirmed,
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
        if active.get("phase") == "charging":
            self._tick_charge_command(active, annotation)
            return
        if active.get("phase") == "dwelling":
            decision = dwell_tick_decision(active, now_time=time.time())
            if decision.get("action") == "wait":
                self._publish_active_waypoint(annotation, active, "dwelling")
                return
            if decision.get("action") == "advance":
                self._advance_active_task(annotation)
                return
        if connector_owns_navigation_status(active):
            self._publish_active_waypoint(annotation, active, "cross_floor")
            self._stop_task_if_connector_unresponsive(active, annotation)
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
            plan_code = str(pre_dispatch.get("code") or "")
            if plan_code.startswith("navigation_plan_"):
                self._fail_active_task(
                    str(active.get("task_id") or ""),
                    str(pre_dispatch.get("message") or "统一导航计划无效，已停止任务"),
                    {
                        "reason": str(pre_dispatch.get("reason") or "navigation_plan_invalid"),
                        "code": plan_code,
                        "transition": pre_dispatch.get("transition"),
                    },
                )
                return
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

    def _stop_task_if_connector_unresponsive(
        self,
        active: Dict[str, Any],
        annotation: Dict[str, Any],
    ) -> bool:
        try:
            started = float(active.get("connector_started_monotonic") or 0.0)
            last_status = float(active.get("connector_last_status_monotonic") or 0.0)
        except (TypeError, ValueError):
            started = 0.0
            last_status = 0.0
        reference = last_status if last_status > 0.0 else started
        if reference <= 0.0:
            return False
        timeout_s = max(2.0, float(self.get_parameter("task_goal_accept_timeout_s").value))
        age_s = max(0.0, time.monotonic() - reference)
        if age_s < timeout_s:
            return False
        reason = "stair_executor_status_stale" if last_status > 0.0 else "stair_executor_no_response"
        self._fail_active_task(
            str(active.get("task_id") or ""),
            "楼梯连接边执行器 %.1f 秒无状态回执，任务已停止" % timeout_s,
            {
                "reason": reason,
                "annotation_id": annotation.get("id"),
                "connector_request_id": active.get("connector_request_id"),
                "connector_state": active.get("connector_state"),
                "status_age_s": age_s,
                "timeout_s": timeout_s,
            },
        )
        return True

    def _begin_waypoint_dwell_or_advance(self, annotation: Dict[str, Any], reason: str) -> None:
        self._publish_zero_cmd(samples=3)
        if str(annotation.get("manual_point_type") or "").strip().lower() == "charge":
            self._start_charge_command(annotation, reason)
            return
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
        if self._hold_active_task_for_radar_inspection(annotation):
            return
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

    def _start_charge_command(self, annotation: Dict[str, Any], reason: str) -> None:
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") != "running":
                return
            if active.get("phase") == "charging":
                return
            semantics = annotation_semantics_payload(annotation)
            vendor = dict(semantics.get("vendor_navigation") or {})
            pose = dict(semantics.get("pose") or {})
            request_id = new_id("charge")
            updated = dict(active)
            updated.update(
                {
                    "phase": "charging",
                    "charge_command_id": request_id,
                    "charge_command_status": "pending",
                    "charge_command_started_at": now_text(),
                    "charge_command_started_monotonic": time.monotonic(),
                    "charge_command_message": "已到达充电点，正在请求原厂充电导航",
                    "last_reached_at": now_text(),
                    "last_reached_annotation_id": annotation.get("id"),
                    "last_reached_reason": reason,
                    "status_message": "已到达充电点，正在请求原厂进入充电",
                }
            )
            self._append_active_task_timeline_event(
                updated,
                "charge_command_sent",
                updated["status_message"],
                {
                    "annotation_id": annotation.get("id"),
                    "request_id": request_id,
                    "point_info": 3,
                },
            )
            self._settings["active_task"] = updated
            self._save_json("settings.json", self._settings)
            active_snapshot = dict(updated)
        discovery_timeout_s = max(
            0.1, float(self.get_parameter("charge_command_discovery_timeout_s").value)
        )
        discovery_deadline = time.monotonic() + discovery_timeout_s
        while rclpy.ok() and self.charge_command_pub.get_subscription_count() <= 0:
            remaining = discovery_deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.05, remaining))
        if self.charge_command_pub.get_subscription_count() <= 0:
            self._fail_active_task(
                str(active_snapshot.get("task_id") or ""),
                "充电命令桥未就绪，未下发原厂 PointInfo=3",
                {
                    "reason": "charge_command_unavailable",
                    "request_id": active_snapshot.get("charge_command_id"),
                    "discovery_timeout_s": discovery_timeout_s,
                },
            )
            return
        message = String()
        message.data = json.dumps(
            {
                "request_id": active_snapshot.get("charge_command_id"),
                "map_id": int(vendor.get("MapID", 0) or 0),
                "x": float(pose.get("x", vendor.get("PosX", 0.0))),
                "y": float(pose.get("y", vendor.get("PosY", 0.0))),
                "z": float(pose.get("z", vendor.get("PosZ", 0.0))),
                "yaw": float(pose.get("yaw", vendor.get("AngleYaw", 0.0))),
                "gait": int(vendor.get("Gait", 12) or 12),
                "speed": int(vendor.get("Speed", 1) or 1),
                "point_info": 3,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.charge_command_pub.publish(message)
        self._publish_active_waypoint(annotation, active_snapshot, "charging")

    def _tick_charge_command(self, active: Dict[str, Any], annotation: Dict[str, Any]) -> None:
        request_id = str(active.get("charge_command_id") or "")
        with self._lock:
            result = dict(self._state.get("charge_command_result") or {})
        if request_id and str(result.get("request_id") or "") == request_id:
            status = str(result.get("status") or "").strip().lower()
            message = str(result.get("message") or "").strip()
            if status == "accepted":
                with self._data_lock:
                    current = dict(self._settings.get("active_task") or active)
                    current["charge_command_status"] = "accepted"
                    current["charge_command_message"] = message or "原厂已接受充电导航 PointInfo=3"
                    current["status_message"] = current["charge_command_message"]
                    self._settings["active_task"] = current
                    self._save_json("settings.json", self._settings)
                self._publish_active_waypoint(annotation, current, "charging")
                self._advance_active_task(annotation)
                return
            if status == "failed":
                self._fail_active_task(
                    str(active.get("task_id") or ""),
                    message or "原厂充电命令被拒绝",
                    {
                        "reason": "charge_command_failed",
                        "request_id": request_id,
                        "error_code": result.get("error_code"),
                    },
                )
                return
        started = float(active.get("charge_command_started_monotonic") or 0.0)
        timeout_s = max(2.0, float(self.get_parameter("charge_command_timeout_s").value))
        if started > 0.0 and time.monotonic() - started > timeout_s:
            self._fail_active_task(
                str(active.get("task_id") or ""),
                "等待原厂充电命令回执超时",
                {"reason": "charge_command_timeout", "request_id": request_id, "timeout_s": timeout_s},
            )
            return
        self._publish_active_waypoint(annotation, active, "charging")

    def _hold_active_task_for_radar_inspection(self, annotation: Dict[str, Any]) -> bool:
        if not bool(self.get_parameter("wait_for_radar_inspection").value):
            return False
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
        if active.get("phase") != "dwelling":
            return False
        radar_state = self._radar_completion_for_active(annotation, active)
        if radar_state == "running":
            self._publish_active_waypoint(annotation, active, "dwelling")
            return True
        if radar_state == "failed":
            self._stop_task({"reason": "radar_inspection_failed"})
            return True
        return False

    @staticmethod
    def _active_waypoint_key(annotation: Dict[str, Any], active: Dict[str, Any]) -> str:
        base = "%s:%s:%s" % (
            str(active.get("task_id") or "manual"),
            str(active.get("index", 0)),
            str(annotation.get("id") or annotation.get("label") or "waypoint"),
        )
        run_id = str(active.get("run_id") or "").strip()
        return "%s:%s" % (run_id, base) if run_id else base

    def _radar_completion_for_active(self, annotation: Dict[str, Any], active: Dict[str, Any]) -> str:
        normalized = normalize_annotation_semantics(dict(annotation))
        radar = normalized.get("radar") if isinstance(normalized.get("radar"), dict) else {}
        if normalized.get("manual_point_type") != "task" or not radar.get("enabled", False):
            return "completed"
        scans = radar.get("scans") if isinstance(radar.get("scans"), list) else []
        if not scans:
            return "completed"
        timeout_s = max(0.0, float(self.get_parameter("radar_inspection_timeout_s").value))
        if timeout_s > 0.0:
            try:
                dwell_until = float(active.get("dwell_until", 0.0) or 0.0)
            except (TypeError, ValueError):
                dwell_until = 0.0
            if dwell_until > 0.0 and time.time() > dwell_until + timeout_s:
                return "failed"
        key = self._active_waypoint_key(annotation, active)
        with self._lock:
            results = dict(self._state.get("radar_inspection_results") or {})
            latest = dict(self._state.get("radar_inspection") or {})
        result = results.get(key)
        parsed = result.get("parsed") if isinstance(result, dict) else None
        if isinstance(parsed, dict):
            status = str(parsed.get("status") or "").strip()
            if status == "completed" and self._radar_payload_covers_plan(parsed, scans):
                return "completed"
            if status == "failed":
                return "failed"
            if self._radar_payload_covers_plan(parsed, scans) and self._is_radar_scan_released(parsed):
                return "completed"
        latest_parsed = latest.get("parsed") if isinstance(latest, dict) else None
        if isinstance(latest_parsed, dict) and str(latest_parsed.get("waypoint_key") or "") == key:
            status = str(latest_parsed.get("status") or "").strip()
            if status == "failed":
                return "failed"
            if self._radar_payload_covers_plan(latest_parsed, scans) and self._is_radar_scan_released(latest_parsed):
                return "completed"
            if status in ("starting", "running"):
                return "running"
        return "running"

    @staticmethod
    def _radar_payload_covers_plan(parsed: Dict[str, Any], scans: List[Dict[str, Any]]) -> bool:
        if parsed.get("scan_mode") == "plan":
            try:
                return int(parsed.get("scan_count") or 0) >= len(scans)
            except (TypeError, ValueError):
                return True
        try:
            scan_index = int(parsed.get("scan_index"))
            scan_count = int(parsed.get("scan_count") or len(scans))
        except (TypeError, ValueError):
            return len(scans) <= 1
        return scan_index >= len(scans) - 1 and scan_count >= len(scans)

    def _is_radar_scan_released(self, parsed: Dict[str, Any]) -> bool:
        if not bool(self.get_parameter("advance_on_radar_scan_release").value):
            return False
        status = str(parsed.get("status") or "").strip()
        state = str(parsed.get("state") or "").strip()
        if bool(parsed.get("scan_released")):
            return True
        if status in ("scan_complete", "analysis_pending"):
            return True
        return state == "analyzing" and bool(parsed.get("analysis_pending"))

    def _dispatch_active_goal(self, force: bool) -> None:
        with self._goal_dispatch_lock:
            self._dispatch_active_goal_once(force)

    def _dispatch_active_goal_once(self, force: bool) -> None:
        with self._data_lock:
            active = self._settings.get("active_task") or {}
        if active.get("status") != "running":
            return
        annotation = self._active_annotation(active)
        if annotation is None:
            failure = active_annotation_missing_failure(active)
            self._fail_active_task_from_payload(failure, task_id=active.get("task_id"))
            return
        with self._lock:
            localization_ok = self._state.get("localization_ok")
            pose = dict(self._state.get("pose") or {})
        with self._data_lock:
            map_relocalization_required = self._settings.get("map_relocalization_required")
        localization_gate = task_start_localization_gate_decision(
            localization_ok=localization_ok,
            pose=pose,
            pose_age=pose_age_sec(pose, time.time()),
            pose_timeout_s=float(self.get_parameter("task_start_pose_timeout_s").value),
            map_relocalization_required=map_relocalization_required,
        )
        if localization_gate.get("action") != "pass":
            self._mark_active_task_waiting(
                active,
                str(localization_gate.get("code") or "localization_not_confirmed"),
                str(localization_gate.get("message") or "当前定位未确认，已暂停下发目标"),
            )
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
            source_floor = str(self._state.get("floor") or "").strip()
        floor_resolution = resolve_runtime_floor_goal(
            goal,
            current_floor=source_floor,
            multi_floor=bool(active.get("multi_floor")),
        )
        goal = dict(floor_resolution["goal"])
        if floor_resolution["floor_overridden"]:
            self.get_logger().warning(
                "single-floor task map label %s differs from floor_manager %s; "
                "dispatching the same-map goal with the runtime floor label"
                % (
                    str(floor_resolution["annotation_floor"]),
                    str(floor_resolution["runtime_floor"]),
                )
            )
        cross_floor = bool(
            active.get("multi_floor")
            and source_floor
            and str(goal.get("floor") or "").strip()
            and source_floor != str(goal.get("floor") or "").strip()
        )
        connector_resolution: Optional[Dict[str, Any]] = None
        connector_route: Optional[Dict[str, Any]] = None
        connector_request_id = ""
        connector_plan_id = ""
        connector_map_epoch = 0
        if cross_floor:
            if (
                str(active.get("last_goal_annotation_id") or "") == str(annotation.get("id") or "")
                and str(active.get("connector_state") or "") not in {"", "COMPLETED", "FAILED", "STOPPED"}
            ):
                self._publish_active_waypoint(annotation, active, "cross_floor")
                return
            connector_resolution = self._resolve_active_connector_transition(
                active,
                annotation,
                source_floor,
            )
            if not connector_resolution.get("ok"):
                self._fail_active_task(
                    active.get("task_id"),
                    str(connector_resolution.get("message") or "统一导航计划无法启动楼层连接"),
                    {
                        "reason": connector_resolution.get("code") or "navigation_plan_transition_invalid",
                        "transition": connector_resolution.get("transition"),
                    },
                )
                return
            connector_route = dict(connector_resolution["route"])
            # Keep route validation in the executor contract so Web and ROS
            # cannot disagree about the four poses used by the same run.
            route_gate = connector_route_activation_decision(connector_route)
            if not route_gate.get("ok"):
                self._fail_active_task(
                    active.get("task_id"),
                    str(route_gate.get("message") or "楼梯连接边配置不完整"),
                    {
                        "reason": route_gate.get("code") or "connector_route_invalid",
                        "route_id": connector_route.get("id"),
                        "source_floor": connector_route.get("source_floor"),
                        "target_floor": connector_route.get("target_floor"),
                    },
                )
                return
            connector_request_id = str(active.get("connector_request_id") or "").strip()
            if not connector_request_id:
                connector_request_id = new_id("stair")
            connector_plan_id = str(active.get("connector_plan_id") or "").strip()
            if not connector_plan_id:
                connector_plan_id = "%s:%s" % (
                    str(active.get("task_id") or "task"),
                    str(active.get("run_id") or "run"),
                )
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
                goal_attempt_id=connector_request_id or new_id("goal"),
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
                if connector_resolution is not None and connector_route is not None:
                    reserved_epoch = next_map_epoch(self._settings)
                    connector_state = mark_connector_started_state(
                        active,
                        annotation,
                        goal,
                        transition=connector_resolution["edge"],
                        request_id=connector_request_id,
                        plan_id=connector_plan_id,
                        map_epoch=reserved_epoch,
                        now_text=now_text(),
                        now_monotonic=now_monotonic,
                    )
                    if not connector_state.get("changed"):
                        missing_failure = {
                            "message": "楼梯连接边身份无法持久化，任务已停止",
                            "reason": connector_state.get("reason")
                            or "connector_identity_invalid",
                        }
                    else:
                        connector_map_epoch = reserved_epoch
                        self._settings["floor_switch_map_epoch"] = reserved_epoch
                        active = connector_state["active"]
                        self._append_active_task_timeline_event(
                            active,
                            str(connector_state["event"]),
                            str(connector_state["message"]),
                            dict(connector_state["event_extra"]),
                        )
                else:
                    self._append_active_task_timeline_event(
                        active,
                        str(prepared["event"]),
                        str(prepared["message"]),
                        dict(prepared["event_extra"]),
                    )
                if missing_failure is None:
                    self._settings["active_task"] = active
                    self._save_json("settings.json", self._settings)
                    active_snapshot = dict(active)
            else:
                return
        if missing_failure is not None:
            self._fail_active_task_from_payload(missing_failure)
            return
        if connector_resolution is not None and connector_route is not None:
            self._publish_stair_connector_start(
                active=active_snapshot,
                route=connector_route,
                request_id=connector_request_id,
                plan_id=connector_plan_id,
                map_epoch=connector_map_epoch,
            )
            self._publish_active_waypoint(annotation, active_snapshot, "cross_floor")
            return
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
                    elif parsed.path in ("/lite", "/lite/"):
                        self._send_bytes(
                            b"",
                            "text/plain; charset=utf-8",
                            status=HTTPStatus.SEE_OTHER,
                            extra_headers={"Location": "/"},
                        )
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
                    elif parsed.path == "/api/live":
                        def optional_version(name: str) -> Optional[int]:
                            raw = (query.get(name) or [None])[0]
                            try:
                                return int(raw) if raw is not None else None
                            except (TypeError, ValueError):
                                return None

                        self._send_json(
                            node._live_snapshot(
                                path_version=optional_version("path_version"),
                                local_path_version=optional_version("local_path_version"),
                            )
                        )
                    elif parsed.path == "/api/inspection/state":
                        self._send_json(node._inspection_live_payload())
                    elif parsed.path == "/api/teleop/state":
                        self._send_json({"ok": True, "teleoperation": node._teleop_status_payload()})
                    elif parsed.path == "/api/map":
                        self._send_json(node._map_snapshot())
                    elif parsed.path == "/api/map_file":
                        map_id = (query.get("map_id") or [None])[0]
                        self._send_json(node._map_file_snapshot(map_id))
                    elif parsed.path == "/api/projects":
                        self._send_json(node._projects_payload())
                    elif parsed.path == "/api/maps":
                        self._send_json(node._maps_payload())
                    elif parsed.path == "/api/multi_floor":
                        self._send_json(node._multi_floor_payload())
                    elif parsed.path == "/api/floor_routes":
                        self._send_json(node._floor_routes_payload())
                    elif parsed.path == "/api/annotations":
                        self._send_json(node._annotations_payload(query))
                    elif parsed.path == "/api/tasks":
                        self._send_json(node._tasks_payload(query))
                    elif parsed.path == "/api/preflight":
                        self._send_json(node._preflight_payload())
                    elif parsed.path == "/api/recording/status":
                        self._send_json(node._recording_status_payload())
                    elif parsed.path == "/api/recording/list":
                        self._send_json(node._recording_list_payload())
                    elif parsed.path == "/api/recording/download":
                        node._send_recording_download((query.get("id") or [""])[0], self)
                    elif parsed.path == "/api/radar/status":
                        self._send_json(node._radar_status_payload())
                    elif parsed.path == "/api/radar/results":
                        self._send_json(node._radar_results_payload(query))
                    elif parsed.path == "/api/radar/result":
                        self._send_api(node._radar_result_payload(query))
                    elif parsed.path == "/api/radar/task":
                        self._send_api(node._radar_task_payload(query))
                    elif parsed.path == "/api/radar/task_export":
                        task_id = (query.get("task_id") or [""])[0]
                        export_format = (query.get("format") or ["json"])[0]
                        if export_format == "csv":
                            payload = node._radar_task_export_csv(task_id)
                            filename = f"radar_{sanitize_name(task_id, 'task')}.csv"
                            self._send_bytes(
                                payload,
                                "text/csv; charset=utf-8",
                                extra_headers={"Content-Disposition": f'attachment; filename="{filename}"'},
                            )
                        else:
                            filename = f"radar_{sanitize_name(task_id, 'task')}.json"
                            self._send_json(
                                node._radar_task_export_payload(task_id),
                                extra_headers={"Content-Disposition": f'attachment; filename="{filename}"'},
                            )
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
                    elif parsed.path == "/api/maps/edit":
                        self._send_api(node._edit_map(payload))
                    elif parsed.path == "/api/mapping/session":
                        self._send_api(node._create_mapping_session(payload))
                    elif parsed.path == "/api/mapping/select_floor":
                        self._send_api(node._select_mapping_session_floor(payload))
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
                    elif parsed.path == "/api/annotations/update":
                        self._send_api(node._update_annotation(payload))
                    elif parsed.path == "/api/tasks":
                        self._send_api(node._create_task(payload))
                    elif parsed.path == "/api/tasks/cross_floor":
                        self._send_api(node._create_cross_floor_task(payload))
                    elif parsed.path == "/api/floor_routes":
                        self._send_api(node._save_floor_route(payload))
                    elif parsed.path == "/api/floor_routes/delete":
                        self._send_api(node._delete_floor_route(payload))
                    elif parsed.path == "/api/tasks/update":
                        self._send_api(node._update_task(payload))
                    elif parsed.path == "/api/tasks/start":
                        self._send_api(node._start_task(payload))
                    elif parsed.path == "/api/tasks/stop":
                        self._send_api(node._stop_task(payload))
                    elif parsed.path == "/api/teleop/acquire":
                        self._send_api(node._acquire_teleop(payload))
                    elif parsed.path == "/api/teleop/command":
                        self._send_api(node._teleop_command(payload))
                    elif parsed.path == "/api/teleop/release":
                        self._send_api(node._release_teleop(payload))
                    elif parsed.path == "/api/teleop/emergency_stop":
                        self._send_api(node._emergency_stop_teleop())
                    elif parsed.path == "/api/teleop/motion":
                        self._send_api(node._teleop_motion(payload))
                    elif parsed.path == "/api/charge/one_key":
                        self._send_api(node._one_key_charge())
                    elif parsed.path == "/api/preflight/run":
                        self._send_api(node._run_preflight(payload))
                    elif parsed.path == "/api/inspection/toggle":
                        self._send_api(node._set_inspection_enabled(payload))
                    elif parsed.path == "/api/recording/start":
                        self._send_api(node._start_recording(payload))
                    elif parsed.path == "/api/recording/stop":
                        self._send_api(node._stop_recording())
                    elif parsed.path == "/api/recording/rename":
                        self._send_api(node._rename_recording(payload))
                    elif parsed.path == "/api/localization/initialpose":
                        self._send_api(node._publish_initialpose(payload))
                    elif parsed.path == "/api/radar/artifact":
                        self._send_api(node._radar_record_artifact(payload))
                    elif parsed.path == "/api/radar/manual_measurement":
                        self._send_api(node._radar_save_manual_measurement(payload))
                    elif parsed.path == "/api/radar/manual_start":
                        self._send_api(node._radar_manual_start(payload))
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
                    elif parsed.path == "/api/maps":
                        map_id = (query.get("id") or query.get("map_id") or [""])[0]
                        cascade = node._as_bool((query.get("cascade") or ["false"])[0])
                        self._send_api(node._delete_map(map_id, cascade=cascade))
                    elif parsed.path == "/api/recording":
                        bag_id = (query.get("id") or [""])[0]
                        self._send_api(node._delete_recording(bag_id))
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

            def _send_json(
                self,
                payload: Dict[str, Any],
                status: HTTPStatus = HTTPStatus.OK,
                extra_headers: Optional[Dict[str, str]] = None,
            ) -> None:
                self._send_bytes(
                    json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
                    "application/json; charset=utf-8",
                    status=status,
                    extra_headers=extra_headers,
                )

            def _send_bytes(
                self,
                payload: bytes,
                content_type: str,
                status: HTTPStatus = HTTPStatus.OK,
                extra_headers: Optional[Dict[str, str]] = None,
            ) -> None:
                self.send_response(status)
                self._send_common_headers(content_type, len(payload), extra_headers=extra_headers)
                self.wfile.write(payload)

            def _send_common_headers(
                self,
                content_type: str,
                length: int,
                extra_headers: Optional[Dict[str, str]] = None,
            ) -> None:
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(length))
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET,POST,DELETE,OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                for key, value in (extra_headers or {}).items():
                    self.send_header(str(key), str(value))
                self.end_headers()

        server = _ReusableThreadingHTTPServer((host, port), DashboardHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.get_logger().info(f"M20Pro web console listening on http://{host}:{port}")
        self.get_logger().info(f"web data dir: {self.data_dir}; map archive dir: {self.map_archive_dir}")
        return server

    def destroy_node(self) -> bool:
        with getattr(self, "_recording_lock", threading.RLock()):
            process = getattr(self, "_recording_process", None)
            if process is not None and process.poll() is None:
                unit_name = str((getattr(self, "_recording_state", None) or {}).get("systemd_unit") or "")
                if unit_name:
                    subprocess.run(["systemctl", "kill", "--signal=SIGINT", unit_name], check=False)
                else:
                    os.killpg(process.pid, signal.SIGINT)
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
