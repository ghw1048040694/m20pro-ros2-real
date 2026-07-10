import json
import os
import re
import threading
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


FINISHED_STATES = {"finished", "zipped"}
FAILED_STATES = {"failed"}
BUSY_STATES = {"busy"}
ANALYSIS_STATES = {"analyzing"}

MEASUREMENT_ITEMS = {
    1: "截面尺寸",
    3: "垂直度",
    5: "外门窗洞口尺寸",
    6: "室内净高",
    7: "阴阳角方正",
    8: "房间开间",
    9: "房间进深",
    10: "方正度",
    11: "户内门洞尺寸",
    12: "房间面积",
    21: "房间开间进深",
    51: "墙面面积",
    52: "墙面总面积",
    53: "顶板总面积",
    56: "墙面尺寸宽高",
    71: "地面平整度",
    72: "墙面平整度",
    73: "地面水平度极差",
    74: "顶板水平度极差",
}
IMPORTANT_MEASUREMENT_IDS = {3, 6, 7, 10, 21, 71, 72, 73, 74}
MEASUREMENT_ID_KEYS = {
    "id",
    "itemId",
    "itemID",
    "item_id",
    "measureId",
    "measureID",
    "measure_id",
    "measureItemId",
    "measureItemID",
    "measure_item_id",
    "measurementId",
    "measurementID",
    "measurementItemId",
    "measurementItemID",
    "measurement_item_id",
    "metricId",
    "metricID",
    "quotaId",
    "quotaID",
    "quota_id",
    "ruleId",
    "ruleID",
    "typeId",
    "typeID",
}
MEASUREMENT_VALUE_KEYS = (
    "values",
    "value",
    "measureValues",
    "measureValue",
    "measurementValues",
    "measurementValue",
    "realValues",
    "realValue",
    "actualValues",
    "actualValue",
    "resultValues",
    "resultValue",
    "deviation",
    "deviations",
    "error",
    "errors",
    "qualified",
    "isQualified",
    "pass",
    "status",
    "result",
    "results",
    "data",
)


class U360Client:
    def __init__(self, base_url: str, timeout_s: float) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s

    def start_scan(self, mode: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        if mode == "modeling":
            return self.post("/nuc/startModelingTask", self.modeling_payload(payload))
        if mode == "measuring":
            return self.post("/nuc/scan", self.measuring_payload(payload))
        raise ValueError("unsupported scan_mode: %s" % mode)

    def query_state(self, task_id: str) -> Dict[str, Any]:
        response = self.post("/nuc/queryState", {"taskId": task_id})
        response["parsedData"] = parse_device_data(response.get("data"))
        return response

    def get_result(self, task_id: str) -> Dict[str, Any]:
        response = self.post("/nuc/getResult", {"taskId": task_id})
        response["parsedData"] = parse_device_data(response.get("data"))
        return response

    def get_task_info(self, task_id: str) -> Dict[str, Any]:
        response = self.post("/nuc/getTaskInfo", {"taskId": task_id})
        response["parsedData"] = parse_device_data(response.get("data"))
        return response

    def download_file(self, file_url: str) -> bytes:
        payload = json.dumps({"fileUrl": file_url}).encode("utf-8")
        request = urllib.request.Request(
            self.base_url + "/nuc/downloadFile",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
            return response.read()

    def post(self, path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        request = urllib.request.Request(
            self.base_url + path,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                raw = response.read().decode("utf-8", errors="replace")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError("%s returned HTTP %s: %s" % (path, exc.code, detail[:300])) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError("%s request failed: %s" % (path, exc.reason)) from exc
        except OSError as exc:
            raise RuntimeError("%s request failed: %s" % (path, exc)) from exc
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise RuntimeError("%s returned non-JSON response: %s" % (path, raw[:300])) from exc
        if not isinstance(parsed, dict):
            raise RuntimeError("%s returned unexpected JSON: %r" % (path, parsed))
        return parsed

    @staticmethod
    def modeling_payload(payload: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "taskId": str(payload["taskId"]),
            "taskDesc": str(payload.get("taskDesc") or payload.get("description") or payload["taskId"]),
            "scene": str(payload.get("scene") or "modeling"),
            "enableCamera": str(payload.get("enableCamera", "false")).lower(),
            "scanDensity": str(payload.get("scanDensity") or "normal"),
        }

    @staticmethod
    def measuring_payload(payload: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "taskId": str(payload["taskId"]),
            "project": str(payload.get("project") or "M20巡检"),
            "building": str(payload.get("building") or "1栋"),
            "suite": str(payload.get("suite") or payload.get("unit") or "1单元"),
            "room": str(payload.get("room") or "000室"),
            "stage": str(payload.get("stage") or "设备安装"),
            "taskType": str(payload.get("taskType") or "Measuring"),
            "scanDensity": str(payload.get("scanDensity") or "low"),
        }


class RadarInspectionNode(Node):
    def __init__(self) -> None:
        super().__init__("m20pro_radar_inspection")
        self._declare_parameters()

        configured_output_dir = str(self.get_parameter("output_dir").value or "").strip()
        if not configured_output_dir:
            configured_output_dir = default_output_dir()
        self.output_dir = Path(os.path.expandvars(os.path.expanduser(configured_output_dir)))
        self.raw_dir = self.output_dir / "raw"
        self.summary_dir = self.output_dir / "summaries"
        self.download_dir = self.output_dir / "downloads"
        self.job_dir = self.output_dir / "jobs"
        for path in (self.raw_dir, self.summary_dir, self.download_dir, self.job_dir):
            path.mkdir(parents=True, exist_ok=True)

        self.backend = str(self.get_parameter("backend").value).strip().lower()
        self.scan_mode = str(self.get_parameter("scan_mode").value).strip().lower()
        self.enabled_waypoint_types = self._string_set(self.get_parameter("enabled_waypoint_types").value)
        self.trigger_phases = self._string_set(self.get_parameter("trigger_phases").value)
        self._lock = threading.Lock()
        self._active_key: Optional[str] = None
        self._released_keys: set = set()
        self._release_times: Dict[str, float] = {}
        self._completed_keys: set = set()
        self._worker: Optional[threading.Thread] = None

        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.result_pub = self.create_publisher(
            String,
            str(self.get_parameter("result_topic").value),
            10,
        )
        self.event_pub = self.create_publisher(
            String,
            str(self.get_parameter("event_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("active_waypoint_topic").value),
            self._on_active_waypoint,
            10,
        )
        self.get_logger().info(
            "radar inspection ready backend=%s scan_mode=%s output=%s"
            % (self.backend, self.scan_mode, self.output_dir)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("active_waypoint_topic", "/m20pro/active_waypoint")
        self.declare_parameter("status_topic", "/m20pro/radar_inspection/status")
        self.declare_parameter("result_topic", "/m20pro/radar_inspection/result")
        self.declare_parameter("event_topic", "/m20pro/radar_inspection/events")
        self.declare_parameter("backend", "dry_run")
        self.declare_parameter("device_url", "http://192.168.107.72:8080")
        self.declare_parameter("request_timeout_s", 10.0)
        self.declare_parameter("poll_interval_s", 2.0)
        self.declare_parameter("max_wait_s", 1800.0)
        self.declare_parameter("dry_run_duration_s", 2.0)
        self.declare_parameter("scan_mode", "measuring")
        self.declare_parameter("scan_density", "low")
        self.declare_parameter("release_on_analysis", True)
        self.declare_parameter("start_retry_timeout_s", 120.0)
        self.declare_parameter("start_retry_interval_s", 5.0)
        self.declare_parameter("result_retry_count", 5)
        self.declare_parameter("result_retry_interval_s", 2.0)
        self.declare_parameter("query_error_timeout_s", 120.0)
        self.declare_parameter("modeling_scene", "modeling")
        self.declare_parameter("modeling_enable_camera", False)
        self.declare_parameter("output_dir", "~/.m20pro_radar_results")
        self.declare_parameter("enabled_waypoint_types", ["task"])
        self.declare_parameter("trigger_phases", ["dwelling"])
        self.declare_parameter("project", "M20巡检")
        self.declare_parameter("building", "1栋")
        self.declare_parameter("building_no", "1")
        self.declare_parameter("unit", "1单元")
        self.declare_parameter("unit_no", "1")
        self.declare_parameter("stage", "设备安装")
        self.declare_parameter("task_type", "Measuring")

    def _on_active_waypoint(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("ignored non-JSON active waypoint")
            return
        waypoint = payload.get("waypoint") or {}
        if not isinstance(waypoint, dict):
            return
        phase = str(payload.get("phase") or "").strip()
        if self.trigger_phases and phase not in self.trigger_phases:
            return
        point_type = str(waypoint.get("manual_point_type") or waypoint.get("type") or "").strip()
        if self.enabled_waypoint_types and point_type not in self.enabled_waypoint_types:
            return

        key = waypoint_key(payload)
        plan = scan_plan_from_waypoint(payload, self.scan_mode, str(self.get_parameter("scan_density").value))
        if not plan.get("enabled") or not plan.get("scans"):
            with self._lock:
                self._completed_keys.add(key)
            return
        with self._lock:
            if key == self._active_key or key in self._released_keys or key in self._completed_keys:
                return
            if self._worker is not None and self._worker.is_alive():
                self.get_logger().warning("radar scan already running; ignored waypoint %s" % key)
                return
            self._active_key = key
            self._worker = threading.Thread(target=self._run_scan_job, args=(key, payload), daemon=True)
            self._worker.start()

    def _run_scan_job(self, key: str, active_waypoint: Dict[str, Any]) -> None:
        plan = scan_plan_from_waypoint(active_waypoint, self.scan_mode, str(self.get_parameter("scan_density").value))
        scans = list(plan.get("scans") or [])
        started = time.time()
        scan_results: List[Dict[str, Any]] = []
        first_request = self._request_from_waypoint(active_waypoint, scans[0] if scans else {}, 0, len(scans))
        current_request = dict(first_request)
        current_scan_started = started
        self._publish_status(
            key,
            "starting",
            active_waypoint,
            first_request,
            state="starting",
            progress=None,
            scan_plan=plan,
        )
        try:
            for index, scan in enumerate(scans):
                scan_started = time.time()
                request = self._request_from_waypoint(active_waypoint, scan, index, len(scans))
                current_request = dict(request)
                current_scan_started = scan_started
                request["defer_motion_release"] = index < len(scans) - 1
                request["scan_plan_count"] = len(scans)
                if self.backend == "dry_run":
                    result = self._run_dry_job(key, active_waypoint, request, scan_started)
                elif self.backend == "u360_http":
                    result = self._run_u360_job(key, active_waypoint, request, scan_started)
                else:
                    raise RuntimeError("unsupported backend: %s" % self.backend)
                scan_results.append(result)
                self._publish_result(result)
                self._publish_event(result)
            if not any(result.get("scan_released_at") for result in scan_results):
                self._mark_released(key)
            result = self._aggregate_scan_results(key, active_waypoint, plan, scan_results, started)
            self._save_job_record(key, result)
            self._publish_result(result)
            self._publish_event(result)
            with self._lock:
                self._completed_keys.add(key)
        except Exception as exc:
            failed_scan = self._base_result(key, active_waypoint, current_request, current_scan_started)
            failed_scan.update({"ok": False, "status": "failed", "state": "error", "error": str(exc)})
            scan_results.append(failed_scan)
            result = self._aggregate_scan_results(key, active_waypoint, plan, scan_results, started)
            result.update({"ok": False, "status": "failed", "state": "error", "error": str(exc)})
            self._save_job_record(key, result)
            self._publish_status(key, "failed", active_waypoint, current_request, state="error", error=str(exc))
            self._publish_result(result)
            self._publish_event(result)
            self.get_logger().error("radar inspection failed: %s" % exc)
            with self._lock:
                self._completed_keys.add(key)
        finally:
            with self._lock:
                if self._active_key == key:
                    self._active_key = None
                if self._worker is not None and self._worker is threading.current_thread():
                    self._worker = None

    def _run_dry_job(
        self,
        key: str,
        active_waypoint: Dict[str, Any],
        request: Dict[str, Any],
        started: float,
    ) -> Dict[str, Any]:
        duration = max(0.0, float(self.get_parameter("dry_run_duration_s").value))
        self._publish_status(key, "running", active_waypoint, request, state="dry_run", progress=10)
        if duration > 0.0:
            time.sleep(duration)
        result = self._base_result(key, active_waypoint, request, started)
        result.update(
            {
                "status": "completed",
                "state": "artifact_pending" if request.get("mode") == "modeling" else "finished",
                "progress": 100,
                "raw_result": {
                    "result": "success",
                    "data": {
                        "state": "artifact_pending" if request.get("mode") == "modeling" else "finished",
                        "progress": 100,
                        "taskId": request["taskId"],
                        "dryRun": True,
                    },
                },
                "summary": self._dry_summary(request, active_waypoint) if request.get("mode") == "measuring" else {},
            }
        )
        if request.get("mode") == "modeling":
            result["artifact_status"] = "pending_import"
            result["manual_measure_status"] = "pending"
            result["manual_measure_required"] = True
            result["artifact_policy"] = request.get("artifact_policy") or "manual_import"
        self._write_result_files(result)
        self._publish_status(key, "completed", active_waypoint, request, state="finished", progress=100)
        return result

    def _aggregate_scan_results(
        self,
        key: str,
        active_waypoint: Dict[str, Any],
        plan: Dict[str, Any],
        scan_results: List[Dict[str, Any]],
        started: float,
    ) -> Dict[str, Any]:
        base_request = scan_results[0].get("request") if scan_results else {"taskId": key}
        request = dict(base_request) if isinstance(base_request, dict) else {"taskId": key}
        request["taskId"] = str(active_waypoint.get("task_id") or key)
        result = self._base_result(key, active_waypoint, request, started)
        failed = [item for item in scan_results if not item.get("ok", True) or item.get("status") == "failed"]
        artifact_pending = any(item.get("artifact_status") == "pending_import" for item in scan_results)
        manual_pending = any(item.get("manual_measure_status") == "pending" for item in scan_results)
        result_unavailable = any(
            item.get("state") == "result_unavailable" or item.get("result_fetch_status") == "failed"
            for item in scan_results
        )
        result_fetch_errors = [
            str(item.get("result_fetch_error") or "")
            for item in scan_results
            if item.get("result_fetch_error")
        ]
        result.update(
            {
                "status": "failed" if failed else "completed",
                "state": "failed"
                if failed
                else ("result_unavailable" if result_unavailable else ("manual_pending" if manual_pending else "finished")),
                "progress": 100,
                "scan_mode": "plan",
                "scan_label": "雷达扫描计划",
                "scan_plan": plan,
                "scan_results": scan_results,
                "scan_count": len(scan_results),
                "artifact_status": "pending_import" if artifact_pending else "complete",
                "manual_measure_status": "pending" if manual_pending else "not_required",
                "manual_measure_required": manual_pending,
            }
        )
        if result_unavailable:
            result["result_fetch_status"] = "failed"
            if result_fetch_errors:
                result["result_fetch_error"] = "；".join(result_fetch_errors)
        self._write_result_files(result)
        return result

    def _run_u360_job(
        self,
        key: str,
        active_waypoint: Dict[str, Any],
        request: Dict[str, Any],
        started: float,
    ) -> Dict[str, Any]:
        client = U360Client(
            str(self.get_parameter("device_url").value),
            max(1.0, float(self.get_parameter("request_timeout_s").value)),
        )
        response = self._start_u360_scan_with_retry(client, key, active_waypoint, request, started)
        if str(response.get("result") or "").lower() != "success":
            raise RuntimeError("U360 start failed: %s" % json.dumps(response, ensure_ascii=False))
        self._publish_status(key, "running", active_waypoint, request, state="pending", progress=0)

        state_response, state, progress = self._poll_until_finished(client, key, active_waypoint, request, started)
        result = self._base_result(key, active_waypoint, request, started)
        result.update({"status": "completed", "state": state, "progress": progress, "state_response": state_response})

        mode = str(request.get("mode") or self.scan_mode)
        if mode == "measuring":
            try:
                raw_result = self._call_u360_with_retry(
                    key,
                    active_waypoint,
                    request,
                    "getResult",
                    lambda: client.get_result(request["taskId"]),
                    state,
                    progress,
                )
            except Exception as exc:
                result.update(
                    {
                        "state": "result_unavailable",
                        "result_fetch_status": "failed",
                        "result_fetch_error": str(exc),
                        "summary": self._unavailable_summary(request, active_waypoint, str(exc)),
                    }
                )
            else:
                if result.get("state") == "result_unavailable":
                    result["state"] = "finished"
                    result["progress"] = 100
                result["raw_result"] = raw_result
                result["summary"] = summarize_measurement_result(raw_result, request)
                result["result_fetch_status"] = "success"
        elif mode == "modeling":
            try:
                task_info = self._call_u360_with_retry(
                    key,
                    active_waypoint,
                    request,
                    "getTaskInfo",
                    lambda: client.get_task_info(request["taskId"]),
                    state,
                    progress,
                )
            except Exception as exc:
                result.update(
                    {
                        "state": "result_unavailable",
                        "result_fetch_status": "failed",
                        "result_fetch_error": str(exc),
                        "downloads": [],
                        "artifact_status": "pending_import",
                    }
                )
            else:
                if result.get("state") == "result_unavailable":
                    result["state"] = "finished"
                    result["progress"] = 100
                result["task_info"] = task_info
                result["downloads"] = self._download_modeling_files(client, request["taskId"], task_info)
                has_download = any(bool(item.get("ok")) for item in result.get("downloads") or [])
                result["artifact_status"] = "downloaded" if has_download else "pending_import"
                result["result_fetch_status"] = "success"
            result["manual_measure_status"] = "pending"
            result["manual_measure_required"] = True
            result["artifact_policy"] = (
                "auto_download" if result.get("artifact_status") == "downloaded" else "manual_import"
            )

        self._write_result_files(result)
        with self._lock:
            released_for_motion = (
                self._release_times.get(key) is not None and not bool(request.get("defer_motion_release"))
            )
        self._publish_status(
            key,
            "completed",
            active_waypoint,
            request,
            state=str(result.get("state") or state),
            progress=result.get("progress") if result.get("progress") is not None else progress,
            scan_released=released_for_motion,
            analysis_pending=False,
            result_fetch_status=result.get("result_fetch_status"),
            result_fetch_error=result.get("result_fetch_error"),
        )
        return result

    def _start_u360_scan_with_retry(
        self,
        client: U360Client,
        key: str,
        active_waypoint: Dict[str, Any],
        request: Dict[str, Any],
        started: float,
    ) -> Dict[str, Any]:
        timeout_s = max(0.0, float(self.get_parameter("start_retry_timeout_s").value))
        interval_s = max(0.2, float(self.get_parameter("start_retry_interval_s").value))
        deadline = time.time() + timeout_s
        attempt = 0
        last_response: Dict[str, Any] = {}
        while True:
            attempt += 1
            try:
                response = client.start_scan(str(request.get("mode") or self.scan_mode), request)
            except Exception as exc:
                detail = str(exc).lower()
                if "busy" not in detail and "忙" not in detail:
                    raise
                response = {"result": "busy", "error": str(exc)}
            last_response = response
            result_text = str(response.get("result") or "").lower()
            data_text = json.dumps(response, ensure_ascii=False).lower()
            if result_text == "success":
                return response
            if "busy" not in result_text and "busy" not in data_text and "忙" not in data_text:
                return response
            if time.time() >= deadline:
                return last_response
            self._publish_status(
                key,
                "waiting_for_device",
                active_waypoint,
                request,
                state="busy",
                progress=None,
                attempt=attempt,
                retry_after_s=interval_s,
            )
            remaining_s = max(0.0, deadline - time.time())
            time.sleep(min(interval_s, remaining_s))

    def _call_u360_with_retry(
        self,
        key: str,
        active_waypoint: Dict[str, Any],
        request: Dict[str, Any],
        operation_name: str,
        operation: Callable[[], Dict[str, Any]],
        state: str,
        progress: Optional[int],
    ) -> Dict[str, Any]:
        retry_count = max(1, int(self.get_parameter("result_retry_count").value))
        interval_s = max(0.2, float(self.get_parameter("result_retry_interval_s").value))
        last_error: Optional[Exception] = None
        for attempt in range(1, retry_count + 1):
            try:
                return operation()
            except Exception as exc:
                last_error = exc
                if attempt >= retry_count:
                    break
                self._publish_status(
                    key,
                    "communication_retry",
                    active_waypoint,
                    request,
                    state="%s_retry" % operation_name,
                    progress=progress,
                    error=str(exc),
                    attempt=attempt,
                    retry_after_s=interval_s,
                )
                time.sleep(interval_s)
        raise RuntimeError("%s unavailable after %d attempts: %s" % (operation_name, retry_count, last_error))

    def _poll_until_finished(
        self,
        client: U360Client,
        key: str,
        active_waypoint: Dict[str, Any],
        request: Dict[str, Any],
        started: float,
    ) -> Tuple[Dict[str, Any], str, Optional[int]]:
        max_wait_s = max(1.0, float(self.get_parameter("max_wait_s").value))
        poll_interval_s = max(0.2, float(self.get_parameter("poll_interval_s").value))
        last_response: Dict[str, Any] = {}
        state = "unknown"
        progress: Optional[int] = None
        released = False
        query_error_started_at: Optional[float] = None
        last_query_error = ""
        query_error_timeout_s = max(0.0, float(self.get_parameter("query_error_timeout_s").value))
        while time.time() - started <= max_wait_s:
            try:
                last_response = client.query_state(request["taskId"])
            except Exception as exc:
                now = time.time()
                if query_error_started_at is None:
                    query_error_started_at = now
                last_query_error = str(exc)
                if query_error_timeout_s > 0.0 and now - query_error_started_at >= query_error_timeout_s:
                    return (
                        {
                            "result": "unknown",
                            "state": "result_unavailable",
                            "progress": progress,
                            "fallback": "query_state_unavailable",
                            "error": last_query_error,
                        },
                        "result_unavailable",
                        progress,
                    )
                self._publish_status(
                    key,
                    "communication_retry",
                    active_waypoint,
                    request,
                    state="queryState_retry",
                    progress=progress,
                    error=last_query_error,
                    retry_after_s=poll_interval_s,
                )
                time.sleep(poll_interval_s)
                continue
            query_error_started_at = None
            state, progress = extract_state(last_response)
            if not bool(request.get("defer_motion_release")) and not released and self._should_release_for_motion(request, state):
                released = True
                self._mark_released(key)
                self._publish_status(
                    key,
                    "scan_complete",
                    active_waypoint,
                    request,
                    state=state,
                    progress=progress,
                    scan_released=True,
                    analysis_pending=True,
                )
            if state in FINISHED_STATES:
                if not released and not bool(request.get("defer_motion_release")):
                    self._mark_released(key)
                return last_response, state, progress
            if state in FAILED_STATES:
                raise RuntimeError("U360 scan failed state=%s response=%s" % (state, last_response))
            if state in BUSY_STATES:
                self._publish_status(
                    key,
                    "analysis_pending" if released else "waiting_for_device",
                    active_waypoint,
                    request,
                    state=state,
                    progress=progress,
                    scan_released=released,
                    analysis_pending=released,
                )
                time.sleep(poll_interval_s)
                continue
            self._publish_status(
                key,
                "analysis_pending" if released else "running",
                active_waypoint,
                request,
                state=state,
                progress=progress,
                scan_released=released,
                analysis_pending=released,
            )
            time.sleep(poll_interval_s)
        raise RuntimeError("U360 scan timeout after %.0fs; last_state=%s" % (max_wait_s, state))

    def _unavailable_summary(
        self,
        request: Dict[str, Any],
        active_waypoint: Dict[str, Any],
        error: str,
    ) -> Dict[str, Any]:
        summary = self._dry_summary(request, active_waypoint)
        summary.update(
            {
                "statusText": "雷达扫描已触发，结果暂不可用",
                "resultUnavailable": True,
                "resultFetchError": error,
            }
        )
        return summary

    def _should_release_for_motion(self, request: Dict[str, Any], state: str) -> bool:
        if str(request.get("mode") or self.scan_mode) != "measuring":
            return False
        return bool(self.get_parameter("release_on_analysis").value) and state in ANALYSIS_STATES

    def _mark_released(self, key: str) -> None:
        with self._lock:
            self._released_keys.add(key)
            self._release_times.setdefault(key, time.time())
            if self._active_key == key:
                self._active_key = None
            if self._worker is not None and self._worker is threading.current_thread():
                self._worker = None

    def _download_modeling_files(
        self,
        client: U360Client,
        task_id: str,
        task_info: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        downloads: List[Dict[str, Any]] = []
        for index, file_url in enumerate(find_file_urls(task_info)):
            try:
                payload = client.download_file(file_url)
            except Exception as exc:
                downloads.append({"fileUrl": file_url, "ok": False, "error": str(exc)})
                continue
            suffix = Path(file_url).suffix or ".bin"
            path = self.download_dir / ("%s_%02d%s" % (safe_name(task_id), index + 1, suffix))
            path.write_bytes(payload)
            downloads.append({"fileUrl": file_url, "ok": True, "path": str(path), "bytes": len(payload)})
        return downloads

    def _request_from_waypoint(
        self,
        active: Dict[str, Any],
        scan: Dict[str, Any],
        scan_index: int,
        scan_count: int,
    ) -> Dict[str, Any]:
        waypoint = active.get("waypoint") or {}
        floor = str(waypoint.get("floor") or "").strip()
        house = str(waypoint.get("house") or waypoint.get("house_no") or "").strip()
        room = str(waypoint.get("room") or "").strip()
        room_for_device = "_".join(part for part in (house, room) if part) or room or house
        label = str(waypoint.get("label") or waypoint.get("id") or "").strip()
        result_prefix = str(waypoint.get("result_file_prefix") or label or waypoint.get("id") or "").strip()
        mode = str(scan.get("mode") or self.scan_mode).strip().lower()
        result_suffix = str(scan.get("result_suffix") or ("measure" if mode == "measuring" else "cloud")).strip()
        building = str(waypoint.get("building") or self.get_parameter("building").value).strip()
        building_no = str(waypoint.get("building_no") or self.get_parameter("building_no").value).strip()
        unit = str(waypoint.get("unit") or self.get_parameter("unit").value).strip()
        unit_no = str(waypoint.get("unit_no") or self.get_parameter("unit_no").value).strip()
        task_id = generate_u360_task_id(
            building_no=building_no,
            unit_no=unit_no,
            floor=floor,
            room=house or room,
            point_index=int(active.get("index", 0)) + 1,
            fallback_prefix=result_prefix,
            scan_suffix=result_suffix,
        )
        request = {
            "taskId": task_id,
            "taskDesc": task_desc(waypoint),
            "mode": mode,
            "project": str(self.get_parameter("project").value),
            "building": building,
            "buildingNo": building_no,
            "suite": unit,
            "unit": unit,
            "unitNo": unit_no,
            "floor": floor,
            "house": house,
            "room": room_for_device or "000室",
            "roomNo": digits(house or room, 4),
            "space": room,
            "scanPoint": waypoint.get("scan_point"),
            "pointIndex": int(active.get("index", 0)) + 1,
            "scanIndex": int(scan_index),
            "scanCount": int(scan_count),
            "scanLabel": scan.get("label"),
            "stage": str(self.get_parameter("stage").value),
            "taskType": str(self.get_parameter("task_type").value),
            "scanDensity": str(scan.get("density") or self.get_parameter("scan_density").value),
            "scene": str(self.get_parameter("modeling_scene").value),
            "enableCamera": bool_text(self.get_parameter("modeling_enable_camera").value),
            "result_file_prefix": result_prefix,
            "result_suffix": result_suffix,
            "artifact_policy": scan.get("artifact_policy"),
            "manual_measure_required": bool(scan.get("manual_measure_required")),
            "scan": dict(scan),
        }
        return request

    def _base_result(
        self,
        key: str,
        active_waypoint: Dict[str, Any],
        request: Dict[str, Any],
        started: float,
    ) -> Dict[str, Any]:
        with self._lock:
            released_at = self._release_times.get(key)
        result = {
            "ok": True,
            "waypoint_key": key,
            "backend": self.backend,
            "scan_mode": request.get("mode") or self.scan_mode,
            "scan_label": request.get("scanLabel"),
            "scan_index": request.get("scanIndex"),
            "scan_count": request.get("scanCount"),
            "result_suffix": request.get("result_suffix"),
            "taskId": request["taskId"],
            "request": request,
            "active_waypoint": active_waypoint,
            "started_at": to_text(started),
            "finished_at": to_text(time.time()),
            "duration_s": round(max(0.0, time.time() - started), 3),
        }
        if released_at is not None:
            result["scan_released_at"] = to_text(released_at)
            result["scan_release_duration_s"] = round(max(0.0, released_at - started), 3)
        return result

    def _dry_summary(self, request: Dict[str, Any], active_waypoint: Dict[str, Any]) -> Dict[str, Any]:
        waypoint = active_waypoint.get("waypoint") or {}
        return {
            "taskId": request["taskId"],
            "mode": request.get("mode") or self.scan_mode,
            "statusText": "dry_run completed",
            "location": {
                "building": waypoint.get("building"),
                "unit": waypoint.get("unit"),
                "house": waypoint.get("house"),
                "floor": waypoint.get("floor"),
                "area": waypoint.get("area"),
                "room": waypoint.get("room"),
                "scanPoint": waypoint.get("scan_point"),
                "label": waypoint.get("label"),
            },
            "metricCount": 0,
            "metrics": [],
        }

    def _write_result_files(self, result: Dict[str, Any]) -> None:
        task_id = safe_name(str(result.get("taskId") or uuid.uuid4().hex))
        if "raw_result" in result:
            raw_path = self.raw_dir / ("%s.json" % task_id)
            write_json(raw_path, {"taskId": result.get("taskId"), "response": result["raw_result"]})
            result["raw_path"] = str(raw_path)
        if "summary" in result:
            summary_path = self.summary_dir / ("%s.json" % task_id)
            write_json(summary_path, result["summary"])
            result["summary_path"] = str(summary_path)
        if "task_info" in result:
            info_path = self.raw_dir / ("%s_task_info.json" % task_id)
            write_json(info_path, result["task_info"])
            result["task_info_path"] = str(info_path)

    def _save_job_record(self, key: str, result: Dict[str, Any]) -> None:
        job_path = self.job_dir / ("%s.json" % safe_name(key))
        write_json(job_path, result)

    def _publish_status(
        self,
        key: str,
        status: str,
        active_waypoint: Dict[str, Any],
        request: Dict[str, Any],
        state: str,
        progress: Optional[int] = None,
        error: Optional[str] = None,
        scan_released: bool = False,
        analysis_pending: bool = False,
        attempt: Optional[int] = None,
        retry_after_s: Optional[float] = None,
        scan_plan: Optional[Dict[str, Any]] = None,
        result_fetch_status: Optional[str] = None,
        result_fetch_error: Optional[str] = None,
    ) -> None:
        payload = {
            "waypoint_key": key,
            "status": status,
            "state": state,
            "progress": progress,
            "scan_released": bool(scan_released),
            "analysis_pending": bool(analysis_pending),
            "backend": self.backend,
            "scan_mode": request.get("mode") or self.scan_mode,
            "taskId": request.get("taskId"),
            "scan_label": request.get("scanLabel"),
            "scan_index": request.get("scanIndex"),
            "scan_count": request.get("scanCount"),
            "active_waypoint": active_waypoint,
            "updated_at": to_text(time.time()),
        }
        if scan_plan is not None:
            payload["scan_plan"] = scan_plan
        if attempt is not None:
            payload["attempt"] = attempt
        if retry_after_s is not None:
            payload["retry_after_s"] = retry_after_s
        if result_fetch_status is not None:
            payload["result_fetch_status"] = result_fetch_status
        if result_fetch_error is not None:
            payload["result_fetch_error"] = result_fetch_error
        if scan_released:
            with self._lock:
                released_at = self._release_times.get(key)
            if released_at is not None:
                payload["scan_released_at"] = to_text(released_at)
        if error:
            payload["error"] = error
        self._publish_json(self.status_pub, payload)

    def _publish_result(self, result: Dict[str, Any]) -> None:
        self._publish_json(self.result_pub, result)

    def _publish_event(self, result: Dict[str, Any]) -> None:
        event = {
            "type": "radar_inspection_result",
            "status": result.get("status"),
            "state": result.get("state"),
            "scan_mode": result.get("scan_mode"),
            "scan_label": result.get("scan_label"),
            "taskId": result.get("taskId"),
            "waypoint_key": result.get("waypoint_key"),
            "summary_path": result.get("summary_path"),
            "raw_path": result.get("raw_path"),
            "artifact_status": result.get("artifact_status"),
            "manual_measure_status": result.get("manual_measure_status"),
            "result_fetch_status": result.get("result_fetch_status"),
            "result_fetch_error": result.get("result_fetch_error"),
            "error": result.get("error"),
            "finished_at": result.get("finished_at"),
        }
        self._publish_json(self.event_pub, event)

    @staticmethod
    def _publish_json(pub: Any, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        pub.publish(msg)

    @staticmethod
    def _string_set(values: Sequence[Any]) -> set:
        if isinstance(values, str):
            return {item.strip() for item in values.split(",") if item.strip()}
        return {str(value).strip() for value in values if str(value).strip()}


def parse_device_data(data: Any) -> Any:
    if isinstance(data, str):
        text = data.strip()
        if not text:
            return data
        try:
            return parse_device_data(json.loads(text))
        except json.JSONDecodeError:
            return data
    if isinstance(data, list):
        return [parse_device_data(item) for item in data]
    if isinstance(data, dict):
        return {key: parse_device_data(value) for key, value in data.items()}
    return data


def extract_state(response: Dict[str, Any]) -> Tuple[str, Optional[int]]:
    parsed = response.get("parsedData")
    if not isinstance(parsed, dict):
        parsed = parse_device_data(response.get("data"))
    state = "unknown"
    progress: Optional[int] = None
    if isinstance(parsed, dict):
        state = str(parsed.get("state") or parsed.get("status") or "unknown").strip()
        raw_progress = parsed.get("progress")
        if raw_progress is not None:
            try:
                progress = int(float(raw_progress))
            except (TypeError, ValueError):
                progress = None
    elif isinstance(parsed, str):
        state = parsed.strip()
    return state or "unknown", progress


def summarize_measurement_result(response: Dict[str, Any], request: Dict[str, Any]) -> Dict[str, Any]:
    parsed = response.get("parsedData")
    source = parsed if parsed is not None else parse_device_data(response.get("data"))
    metrics = extract_measurements(source)
    if not metrics:
        metrics = extract_measurement_like_scalars(source)
    metrics.sort(
        key=lambda item: (
            0 if item.get("important") else 1,
            int(item.get("measurementItemId") or 9999)
            if str(item.get("measurementItemId") or "").isdigit()
            else 9999,
            str(item.get("location") or ""),
        )
    )
    return {
        "taskId": str(request.get("taskId") or ""),
        "createdAt": to_text(time.time()),
        "mode": str(request.get("mode") or "measuring"),
        "location": {
            "project": request.get("project"),
            "building": request.get("building"),
            "suite": request.get("suite") or request.get("unit"),
            "floor": request.get("floor"),
            "room": request.get("room"),
            "stage": request.get("stage"),
            "pointIndex": request.get("pointIndex"),
        },
        "statusText": find_status_text(source),
        "metricCount": len(metrics),
        "metrics": metrics,
    }


def extract_measurements(value: Any, path: str = "") -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    if isinstance(value, dict):
        if ".measurements[" in path:
            item_id = first_present(value, MEASUREMENT_ID_KEYS)
            item_value = measurement_value(value)
            if item_id is not None:
                try:
                    numeric_id = int(float(item_id))
                except (TypeError, ValueError):
                    numeric_id = -1
                cleaned = clean_values(item_value)
                if not cleaned:
                    return rows
                rows.append(
                    {
                        "measurementItemId": numeric_id,
                        "measurementItem": MEASUREMENT_ITEMS.get(numeric_id, str(item_id)),
                        "location": measurement_location(path),
                        "rawValue": cleaned if isinstance(item_value, list) else cleaned[0],
                        "displayValue": display_value(item_value),
                        "numericValue": to_number(cleaned[0]) if len(cleaned) == 1 else None,
                        "important": numeric_id in IMPORTANT_MEASUREMENT_IDS,
                        "path": path,
                    }
                )
        for key, item in value.items():
            next_path = "%s.%s" % (path, key) if path else str(key)
            rows.extend(extract_measurements(item, next_path))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            rows.extend(extract_measurements(item, "%s[%d]" % (path, index)))
    return rows


def extract_measurement_like_scalars(value: Any) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for path, item in walk_json(value):
        if len(rows) >= 200:
            break
        if not path or isinstance(item, (dict, list)):
            continue
        if not looks_like_measurement_path(path):
            continue
        rows.append(
            {
                "measurementItemId": "",
                "measurementItem": path.split(".")[-1].split("[")[0],
                "location": "",
                "rawValue": item,
                "displayValue": display_value(item),
                "numericValue": to_number(item),
                "important": False,
                "path": path,
            }
        )
    return rows


def walk_json(value: Any, path: str = "") -> List[Tuple[str, Any]]:
    rows: List[Tuple[str, Any]] = []
    if isinstance(value, dict):
        for key, item in value.items():
            next_path = "%s.%s" % (path, key) if path else str(key)
            rows.extend(walk_json(item, next_path))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            next_path = "%s[%d]" % (path, index) if path else "[%d]" % index
            rows.extend(walk_json(item, next_path))
    else:
        rows.append((path, value))
    return rows


def looks_like_measurement_path(path: str) -> bool:
    lowered = path.lower()
    keywords = (
        "flat",
        "vertical",
        "level",
        "angle",
        "deviation",
        "error",
        "value",
        "measure",
        "result",
        "平整",
        "垂直",
        "水平",
        "阴阳角",
        "方正",
        "偏差",
        "实测",
        "测量",
        "合格",
    )
    return any(key in lowered or key in path for key in keywords)


def first_present(data: Dict[str, Any], keys: Iterable[str]) -> Any:
    for key in keys:
        if key in data:
            return data[key]
    return None


def measurement_value(data: Dict[str, Any]) -> Any:
    value = first_present(data, MEASUREMENT_VALUE_KEYS)
    if value is not None:
        return value
    scalars = []
    for key, item in data.items():
        if key in MEASUREMENT_ID_KEYS:
            continue
        if isinstance(item, (str, int, float, bool)) or item is None:
            scalars.append({key: item})
    return scalars if scalars else data


def measurement_location(path: str) -> str:
    if path.startswith("room.measurements"):
        return "房间"
    if path.startswith("floor.measurements"):
        return "地面"
    if path.startswith("ceiling.measurements"):
        return "顶板"
    wall_match = re.match(r"walls\[(\d+)\]\.measurements", path)
    if wall_match:
        return "墙面%d" % (int(wall_match.group(1)) + 1)
    opening_match = re.match(r"openings\[(\d+)\]\.measurements", path)
    if opening_match:
        return "门窗洞口%d" % (int(opening_match.group(1)) + 1)
    prefix = path.split(".measurements[", maxsplit=1)[0]
    if not prefix:
        return "unknown"
    return prefix


def display_value(value: Any) -> str:
    cleaned = clean_values(value)
    numbers = [number for number in (to_number(item) for item in cleaned) if number is not None]
    if len(cleaned) > 1 and len(numbers) == len(cleaned):
        raw = ", ".join(format_number(number) for number in numbers[:12])
        if len(numbers) > 12:
            raw += " ... 共%d个" % len(numbers)
        return raw
    if isinstance(value, (list, tuple)):
        return ", ".join(display_value(item) for item in cleaned)
    if isinstance(value, dict):
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
    return "" if value is None else str(value)


def clean_values(value: Any) -> List[Any]:
    values = value if isinstance(value, list) else [value]
    cleaned = []
    for item in values:
        if item in (None, "", "unknow", "unknown"):
            continue
        cleaned.append(item)
    return cleaned


def to_number(value: Any) -> Optional[float]:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        match = re.search(r"-?\d+(?:\.\d+)?", value)
        if match:
            try:
                return float(match.group(0))
            except ValueError:
                return None
    return None


def format_number(value: float) -> str:
    if float(value).is_integer():
        return str(int(value))
    return ("%.2f" % value).rstrip("0").rstrip(".")


def find_status_text(source: Any) -> str:
    for path, value in walk_json(source):
        lowered = path.lower()
        if any(flag in lowered for flag in ("state", "status", "result", "conclusion")) or any(
            flag in path for flag in ("状态", "结果", "结论", "合格")
        ):
            if isinstance(value, (str, int, float, bool)):
                return str(value)
    return "measurement result fetched"


def find_file_urls(value: Any) -> List[str]:
    urls: List[str] = []
    if isinstance(value, dict):
        for key, item in value.items():
            lower = str(key).lower()
            if isinstance(item, str) and ("file" in lower or "url" in lower):
                if item.startswith("http") or item.startswith("/") or "." in item:
                    urls.append(item)
            if ("file" in lower or "url" in lower or "download" in lower) and isinstance(item, (dict, list)):
                urls.extend(find_url_like_strings(item))
            urls.extend(find_file_urls(item))
    elif isinstance(value, list):
        for item in value:
            urls.extend(find_file_urls(item))
    deduped: List[str] = []
    for item in urls:
        if item not in deduped:
            deduped.append(item)
    return deduped


def find_url_like_strings(value: Any) -> List[str]:
    urls: List[str] = []
    if isinstance(value, str):
        if value.startswith("http") or value.startswith("/") or "." in value:
            urls.append(value)
    elif isinstance(value, dict):
        for item in value.values():
            urls.extend(find_url_like_strings(item))
    elif isinstance(value, list):
        for item in value:
            urls.extend(find_url_like_strings(item))
    return urls


def scan_plan_from_waypoint(active: Dict[str, Any], default_mode: str, default_density: str) -> Dict[str, Any]:
    waypoint = active.get("waypoint") or {}
    radar = waypoint.get("radar") if isinstance(waypoint, dict) else None
    if not isinstance(radar, dict):
        radar = {}
    enabled = radar.get("enabled", True)
    if isinstance(enabled, str):
        enabled = enabled.strip().lower() not in ("0", "false", "no", "off", "disabled")
    scans: List[Dict[str, Any]] = []
    raw_scans = radar.get("scans")
    if isinstance(raw_scans, list):
        for index, raw in enumerate(raw_scans):
            item = raw if isinstance(raw, dict) else {"mode": raw}
            mode = str(item.get("mode") or "").strip().lower()
            if mode not in ("measuring", "modeling"):
                continue
            scans.append(normalize_scan_plan_item(item, index, default_density))
    if not scans and enabled:
        mode = str(default_mode or "measuring").strip().lower()
        if mode not in ("measuring", "modeling"):
            mode = "measuring"
        scans.append(normalize_scan_plan_item({"mode": mode}, 0, default_density))
    scans.sort(key=lambda item: int(item.get("order", 0)))
    return {"enabled": bool(enabled), "scans": scans}


def normalize_scan_plan_item(item: Dict[str, Any], index: int, default_density: str) -> Dict[str, Any]:
    mode = str(item.get("mode") or "measuring").strip().lower()
    if mode not in ("measuring", "modeling"):
        mode = "measuring"
    label = str(item.get("label") or ("实测实量" if mode == "measuring" else "点云建模")).strip()
    suffix = str(item.get("result_suffix") or ("measure" if mode == "measuring" else "cloud")).strip()
    density = str(item.get("density") or item.get("scan_density") or default_density or "").strip()
    artifact_policy = str(
        item.get("artifact_policy")
        or ("manual_import" if mode == "modeling" else "auto_result")
    ).strip()
    manual_required = item.get("manual_measure_required", mode == "modeling")
    if isinstance(manual_required, str):
        manual_required = manual_required.strip().lower() in ("1", "true", "yes", "on")
    return {
        "mode": mode,
        "label": label,
        "result_suffix": suffix,
        "density": density,
        "artifact_policy": artifact_policy,
        "manual_measure_required": bool(manual_required),
        "order": int(item.get("order", index)),
    }


def waypoint_key(active: Dict[str, Any]) -> str:
    waypoint = active.get("waypoint") or {}
    return "%s:%s:%s" % (
        str(active.get("task_id") or "manual"),
        str(active.get("index", 0)),
        str(waypoint.get("id") or waypoint.get("label") or "waypoint"),
    )


def generate_u360_task_id(
    building_no: str,
    unit_no: str,
    floor: str,
    room: str,
    point_index: int,
    fallback_prefix: str,
    scan_suffix: str = "",
) -> str:
    floor_number = digits(floor, 2)
    room_number = digits(room, 4)
    prefix = "B%s_U%s_F%s_R%s_P%02d" % (
        digits(building_no, 2),
        digits(unit_no, 2),
        floor_number,
        room_number,
        max(1, int(point_index)),
    )
    if room_number == "0000" and fallback_prefix:
        prefix = safe_name(fallback_prefix)[:50] or prefix
    if scan_suffix:
        prefix = "%s_%s" % (prefix, safe_name(scan_suffix))
    return "%s_%s" % (prefix, time.strftime("%Y%m%d_%H%M%S", time.localtime()))


def task_desc(waypoint: Dict[str, Any]) -> str:
    parts = [
        str(waypoint.get("building") or "").strip(),
        str(waypoint.get("unit") or "").strip(),
        str(waypoint.get("house") or "").strip(),
        str(waypoint.get("floor") or "").strip(),
        str(waypoint.get("area") or "").strip(),
        str(waypoint.get("room") or "").strip(),
        str(waypoint.get("scan_point") or "").strip(),
        str(waypoint.get("label") or waypoint.get("id") or "").strip(),
    ]
    return " ".join(part for part in parts if part) or "M20 radar inspection"


def digits(value: Any, width: int) -> str:
    text = str(value or "")
    match = re.findall(r"\d+", text)
    number = match[-1] if match else "0"
    return number.zfill(width)[-width:]


def safe_name(value: str) -> str:
    text = re.sub(r"[^\w.\-\u4e00-\u9fff]+", "_", str(value).strip(), flags=re.UNICODE)
    text = text.strip("._")
    return text or "radar_result"


def to_text(timestamp: float) -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp))


def write_json(path: Path, payload: Any) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=False, indent=2)
    tmp.replace(path)


def default_output_dir() -> str:
    if os.geteuid() == 0 and Path("/home/user").exists():
        return "/home/user/m20pro_radar_results"
    return "~/.m20pro_radar_results"


def bool_text(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    text = str(value).strip().lower()
    return "true" if text in ("1", "true", "yes", "on") else "false"


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RadarInspectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
