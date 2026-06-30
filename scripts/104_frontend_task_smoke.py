#!/usr/bin/env python3
"""Read-only browser smoke test for the M20Pro frontend task page.

The script opens the real web frontend through headless Chrome, switches to the
task tab, and verifies that the task page exposes the information needed before
field execution.  It does not click task start, publish goals, or send motion.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import urlparse


def find_chrome() -> str:
    for name in ("google-chrome", "chromium-browser", "chromium"):
        path = shutil.which(name)
        if path:
            return path
    raise RuntimeError("Chrome/Chromium is required for frontend smoke testing")


class CdpClient:
    def __init__(self, websocket_url: str) -> None:
        parsed = urlparse(websocket_url)
        if not parsed.hostname or not parsed.port:
            raise RuntimeError(f"invalid websocket URL: {websocket_url}")
        self.sock = socket.create_connection((parsed.hostname, parsed.port), timeout=5)
        self.sock.settimeout(None)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        request = (
            f"GET {parsed.path} HTTP/1.1\r\n"
            f"Host: {parsed.hostname}:{parsed.port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(request.encode("ascii"))
        response = self.sock.recv(4096)
        if b"101" not in response.split(b"\r\n", 1)[0]:
            raise RuntimeError("Chrome DevTools websocket handshake failed")
        self._message_id = 0

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def _send_frame(self, payload: bytes) -> None:
        header = bytearray([0x81])
        length = len(payload)
        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header.append(0x80 | 126)
            header.extend(struct.pack("!H", length))
        else:
            header.append(0x80 | 127)
            header.extend(struct.pack("!Q", length))
        mask = os.urandom(4)
        header.extend(mask)
        masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        self.sock.sendall(header + masked)

    def _recv_frame(self) -> Dict[str, Any]:
        first = self.sock.recv(2)
        if len(first) < 2:
            raise RuntimeError("short websocket read")
        opcode = first[0] & 0x0F
        length = first[1] & 0x7F
        if length == 126:
            length = struct.unpack("!H", self.sock.recv(2))[0]
        elif length == 127:
            length = struct.unpack("!Q", self.sock.recv(8))[0]
        masked = bool(first[1] & 0x80)
        mask = self.sock.recv(4) if masked else b""
        data = bytearray()
        while len(data) < length:
            chunk = self.sock.recv(length - len(data))
            if not chunk:
                raise RuntimeError("websocket closed during read")
            data.extend(chunk)
        if masked:
            data = bytearray(byte ^ mask[index % 4] for index, byte in enumerate(data))
        if opcode == 8:
            raise RuntimeError("Chrome DevTools websocket closed")
        return json.loads(data.decode("utf-8"))

    def call(self, method: str, params: Optional[Dict[str, Any]] = None, timeout_s: float = 10.0) -> Dict[str, Any]:
        self._message_id += 1
        message_id = self._message_id
        self._send_frame(
            json.dumps(
                {"id": message_id, "method": method, "params": params or {}},
                separators=(",", ":"),
            ).encode("utf-8")
        )
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.sock.settimeout(max(0.2, deadline - time.time()))
            message = self._recv_frame()
            if message.get("id") == message_id:
                self.sock.settimeout(None)
                return message
        raise TimeoutError(method)


def wait_for_devtools(port: int, timeout_s: float) -> str:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/json", timeout=1) as response:
                tabs = json.load(response)
            for tab in tabs:
                if tab.get("type") == "page" and tab.get("webSocketDebuggerUrl"):
                    return str(tab["webSocketDebuggerUrl"])
        except Exception:
            time.sleep(0.2)
    raise RuntimeError("Chrome DevTools did not become ready")


def evaluate_frontend(client: CdpClient) -> Dict[str, Any]:
    expression = r"""
(async () => {
  const tab = document.querySelector('button.tab[data-tab="tasks"]');
  if (tab) tab.click();
  await new Promise(resolve => setTimeout(resolve, 3500));
  if (typeof loadTasks === 'function') {
    await loadTasks();
    await new Promise(resolve => setTimeout(resolve, 500));
  }
	  const q = id => document.getElementById(id);
	  const canvas = q('mapCanvas');
	  let canvasSample = null;
	  if (canvas && canvas.width > 0 && canvas.height > 0) {
    const context = canvas.getContext('2d');
    const sampleX = Math.max(0, Math.min(canvas.width - 1, Math.floor(canvas.width * 0.5)));
    const sampleY = Math.max(0, Math.min(canvas.height - 1, Math.floor(canvas.height * 0.5)));
	    const data = context.getImageData(sampleX, sampleY, 1, 1).data;
	    canvasSample = Array.from(data);
	  }
	  const fieldSnapshot = window.m20proDebug && typeof window.m20proDebug.fieldSnapshot === 'function'
	    ? window.m20proDebug.fieldSnapshot()
	    : null;
  return {
    statusText: q('statusText') && q('statusText').textContent,
    floor: q('floor') && q('floor').textContent,
    localization: q('localization') && q('localization').textContent,
	    taskReadiness: q('taskReadinessSummary') && q('taskReadinessSummary').textContent,
	    taskNextStep: q('taskNextStepSummary') && q('taskNextStepSummary').textContent,
	    activeTaskSummary: q('activeTaskSummary') && q('activeTaskSummary').textContent,
		    activeTaskRaw: q('activeTask') && q('activeTask').textContent,
	    hiddenTaskCount: window.m20proDebug && window.m20proDebug.snapshot().lastTasksPayload
	      ? window.m20proDebug.snapshot().lastTasksPayload.hidden_task_count
	      : null,
	    selectedMapStatus: window.m20proDebug && window.m20proDebug.snapshot().lastTasksPayload
	      ? window.m20proDebug.snapshot().lastTasksPayload.selected_map_status
	      : (window.m20proDebug ? window.m20proDebug.snapshot().selectedMapStatus : null),
	    tasksPayloadIncludeAll: window.m20proDebug && window.m20proDebug.snapshot().lastTasksPayload
	      ? window.m20proDebug.snapshot().lastTasksPayload.include_all
		      : null,
		    currentMapTaskNotice: Array.from(document.querySelectorAll('#taskList .preflight-summary')).map(el => el.textContent),
		    taskPointCheckedCount: document.querySelectorAll('#taskPointList input:checked').length,
		    taskPointTotal: document.querySelectorAll('#taskPointList input[type="checkbox"]').length,
		    createTaskDisabled: q('createTaskBtn') ? q('createTaskBtn').disabled : null,
		    createTaskTitle: q('createTaskBtn') ? q('createTaskBtn').getAttribute('title') : null,
		    saveMarkButton: q('saveMarkBtn') ? {
		      disabled: q('saveMarkBtn').disabled,
		      title: q('saveMarkBtn').getAttribute('title'),
		    } : null,
		    useRobotPoseButton: q('useRobotPoseBtn') ? {
		      disabled: q('useRobotPoseBtn').disabled,
		      title: q('useRobotPoseBtn').getAttribute('title'),
		    } : null,
		    taskItems: Array.from(document.querySelectorAll('#taskList .item')).map(el => el.innerText),
	    copyCommandButtons: Array.from(document.querySelectorAll('[data-copy-command]')).map(btn => btn.dataset.copyCommand),
	    enabledCopyCommandButtons: Array.from(document.querySelectorAll('[data-copy-command]:not(:disabled)')).map(btn => btn.dataset.copyCommand),
	    hasFieldSnapshotButton: !!q('copyFieldSnapshotBtn'),
	    hasFieldSnapshotFunction: typeof buildFieldSnapshot === 'function',
	    fieldSnapshot,
	    hasMap3dButton: !!q('map3dBtn'),
	    hasMap2dButton: !!q('map2dBtn'),
	    cameraToggleButtons: Array.from(document.querySelectorAll('[data-camera-toggle]')).map(btn => btn.dataset.cameraToggle),
	    cameraToggleAllButtons: Array.from(document.querySelectorAll('[data-camera-toggle-all]')).map(btn => btn.dataset.cameraToggleAll),
	    hasCameraStatus: !!q('cameraStatus'),
	    hasFrontVideo: !!q('frontVideo'),
	    hasRearVideo: !!q('rearVideo'),
	    frontVideoSrc: document.getElementById('frontVideo') && document.getElementById('frontVideo').getAttribute('src'),
	    rearVideoSrc: document.getElementById('rearVideo') && document.getElementById('rearVideo').getAttribute('src'),
	    frontVideoDataSrc: document.getElementById('frontVideo') && document.getElementById('frontVideo').dataset.src,
	    rearVideoDataSrc: document.getElementById('rearVideo') && document.getElementById('rearVideo').dataset.src,
	    frontVideoBtn: q('frontVideoBtn') && q('frontVideoBtn').textContent,
	    rearVideoBtn: q('rearVideoBtn') && q('rearVideoBtn').textContent,
	    cameraStatus: q('cameraStatus') && q('cameraStatus').textContent,
	    canvasWidth: canvas && canvas.width,
	    canvasHeight: canvas && canvas.height,
	    canvasSample,
	    livePoseTracker: q('livePoseTracker') && q('livePoseTracker').textContent,
	    taskPoseTracker: q('taskPoseTracker') && q('taskPoseTracker').textContent,
	    stopTaskButton: q('stopTaskBtn') ? {
	      disabled: q('stopTaskBtn').disabled,
	      title: q('stopTaskBtn').getAttribute('title'),
	    } : null,
	    resetTaskSessionButton: q('resetTaskSessionBtn') ? {
	      disabled: q('resetTaskSessionBtn').disabled,
	      title: q('resetTaskSessionBtn').getAttribute('title'),
	    } : null,
	    startButtons: Array.from(document.querySelectorAll('[data-start-task]')).map(btn => ({
      text: btn.textContent,
      disabled: btn.disabled,
      title: btn.title,
      taskId: btn.dataset.startTask
    })),
    hasConfirmFunction: typeof taskStartConfirmText === 'function',
    hasStartRequestFunction: typeof taskStartRequest === 'function',
    hasActiveTaskSummaryFunction: typeof renderActiveTaskSummary === 'function',
    hasDebug: !!window.m20proDebug,
    latestActiveTask: window.m20proDebug && window.m20proDebug.snapshot().latest.active_task,
  };
})()
"""
    result = client.call(
        "Runtime.evaluate",
        {"expression": expression, "awaitPromise": True, "returnByValue": True},
        timeout_s=15,
    )
    payload = result.get("result", {}).get("result", {}).get("value")
    if not isinstance(payload, dict):
        raise RuntimeError(f"unexpected Runtime.evaluate result: {result}")
    return payload


def evaluate_active_summary(client: CdpClient) -> Dict[str, Any]:
    expression = r"""
(() => {
  const q = id => document.getElementById(id);
  const previous = q('activeTaskSummary') && q('activeTaskSummary').textContent;
  const previousTracker = q('taskPoseTracker') && q('taskPoseTracker').textContent;
  const activeTask = {
    task_name: 'smoke_task',
    last_goal_attempt_id: 'goal_smoke_1',
    goal_sent_path_version: 3,
    plan_goal_verified: true,
    plan_path_version: 4,
    last_floor_goal_published_at: '2026-06-27 09:29:00',
    floor_goal_publish_count: 1,
    runtime_guard: {code: 'ready'},
    timeline: [{message: 'synthetic latest event'}]
  };
  const waypoint = {
    waypoint: {label: 'F20_smoke_point', pose: {x: 5.0, y: 7.0, yaw: 1.2}},
    index: 0,
    phase: 'navigating',
    nav_goal_status: 'accepted',
    distance_m: 2.0,
    nav_feedback: {
      goal_seq: 42,
      distance_remaining: 1.5,
      navigation_time: 12,
      recoveries: 1,
      pose_x: 4.2,
      pose_y: 6.6,
      pose_yaw: 1.1
    },
    nav_goal_match: {matches: true, nav_goal_seq: 42},
    robot_pose: {x: 3.5, y: 6.0, yaw: 1.0},
    state_pose: {x: 4.1, y: 6.7, yaw: 1.08},
    nav_feedback_age_s: 2,
    goal_pose: {x: 5.0, y: 7.0, yaw: 1.2},
    path_goal_error_m: 0.12,
    goal_sent_path_version: 3,
    plan_goal_verified: true,
    plan_path_version: 4,
    last_floor_goal_published_at: '2026-06-27 09:29:00',
    floor_goal_publish_count: 1,
    elapsed_s: 18,
    goal_send_count: 1,
    stall_age_s: 5,
    runtime_guard: {code: 'ready'},
    last_progress_at: '2026-06-23 14:00:00',
    status_message: 'synthetic summary check'
  };
  const previousLatest = window.m20proDebug.snapshot().latest;
  const syntheticState = {
    localization_ok: true,
    pose_fresh: true,
    pose_age_sec: 0.4,
    pose: {x: 3.5, y: 6.0, yaw: 1.0, display_yaw: 1.0},
    active_task: activeTask,
    active_waypoint: {parsed: waypoint}
  };
  window.m20proDebug.snapshot().latest = syntheticState;
  renderActiveTaskSummary(activeTask, waypoint);
  renderPoseTracker('taskPoseTracker', syntheticState);
  const text = q('activeTaskSummary') && q('activeTaskSummary').textContent;
  const tracker = q('taskPoseTracker') && q('taskPoseTracker').textContent;
  renderActiveTaskSummary(null, null);
  const inactiveText = q('activeTaskSummary') && q('activeTaskSummary').textContent;
  renderActiveTaskSummary(null, null, {
    task_name: 'stale failed task',
    last_error: 'historical failure should not look active',
    last_result: {status: 'error', message: 'old result'}
  });
  const inactiveWithHistoryText = q('activeTaskSummary') && q('activeTaskSummary').textContent;
  renderActiveTaskSummary(null, null);
  window.m20proDebug.snapshot().latest = previousLatest;
  if (q('taskPoseTracker')) q('taskPoseTracker').textContent = previousTracker || '';
  return {
    previous,
    text,
    tracker,
    inactiveText,
    inactiveWithHistoryText,
    restored: q('activeTaskSummary') && q('activeTaskSummary').textContent
  };
})()
"""
    result = client.call(
        "Runtime.evaluate",
        {"expression": expression, "returnByValue": True},
        timeout_s=10,
    )
    payload = result.get("result", {}).get("result", {}).get("value")
    if not isinstance(payload, dict):
        raise RuntimeError(f"unexpected Runtime.evaluate result: {result}")
    return payload


def evaluate_manual_relocalization_status(client: CdpClient) -> Dict[str, Any]:
    expression = r"""
(() => {
  const q = id => document.getElementById(id);
  const box = q('localizationStatus');
  const hasRenderer = typeof renderLocalizationStatus === 'function';
  if (!box || !hasRenderer) {
    return {hasBox: !!box, hasRenderer};
  }
  const previousText = box.textContent;
  const previousClass = box.className;
	  renderLocalizationStatus({
	    localization_status: {
	      confirmed: false,
	      task_ready: false,
	      tcp_2101_required: true,
	      tcp_2101_accepted: false,
	      tcp_2101_failed: true,
	      factory_localization_ok: false,
	      pose_ok: false,
	      pose_fresh: false,
	      code: 'manual_tcp_2101_failed',
	      message: '开发手册 2101/1 返回失败，重定位未确认'
	    }
	  });
  const failedText = box.textContent;
  const failedClass = box.className;
  renderLocalizationStatus({
    localization_status: {
      confirmed: true,
	      task_ready: true,
	      tcp_2101_required: true,
	      tcp_2101_accepted: true,
	      tcp_2101_recent: true,
	      factory_localization_ok: true,
	      pose_ok: true,
	      pose_fresh: true,
	      code: 'ready',
	      message: '开发手册 2101/1 重定位已成功'
	    }
	  });
	  const successText = box.textContent;
	  const successClass = box.className;
	  renderLocalizationStatus({
	    localization_status: {
	      confirmed: true,
	      task_ready: false,
	      tcp_2101_required: true,
	      tcp_2101_accepted: true,
	      tcp_2101_recent: true,
	      factory_localization_ok: true,
	      pose_ok: true,
	      pose_fresh: true,
	      code: 'localized_task_not_ready',
	      message: '重定位成功：定位已确认；但任务页暂不可启动：电量 22% 低于任务要求 25%'
	    }
	  });
	  const taskBlockedText = box.textContent;
	  const taskBlockedClass = box.className;
	  renderLocalizationStatus({
	    localization_status: {
	      confirmed: false,
	      task_ready: false,
	      tcp_2101_required: true,
	      tcp_2101_accepted: true,
	      tcp_2101_recent: true,
	      factory_localization_ok: true,
	      pose_ok: true,
	      pose_fresh: true,
	      pose_age_sec: 0.2,
	      code: 'map_relocalization_required',
	      message: '收到 2101 success 回执，原厂定位和地图位姿也在更新；但当前固定地图的重定位锁还没有被网页确认清除，任务页不会允许开始任务',
	      map_relocalization_required: {reason: 'startup_sync'}
	    }
	  });
	  const partialText = box.textContent;
	  const partialClass = box.className;
	  renderLocalizationStatus({
	    localization_status: {
	      confirmed: false,
	      task_ready: false,
	      factory_localization_ok: true,
	      pose_ok: true,
	      pose_fresh: true,
	      pose_age_sec: 0.4,
	      code: 'map_relocalization_required',
	      message: '原厂定位未确认；任务页不会允许开始任务'
	    },
	    relocalization_result: {
	      raw: 'success: x=-10.771 y=-3.610 z=0.000 yaw=-1.587',
	      last_update: 101.0
	    },
	    task_readiness: {
	      ready: false,
	      code: 'map_relocalization_required',
	      message: 'Nav2 已加载当前固定地图，请先按开发手册2101完成重定位，再开始标点或任务',
	      map_relocalization_required: {reason: 'startup_sync'}
	    },
	    node_time: 102.2,
	    factory_localization_ok: true,
	    pose_fresh: true
	  });
	  const legacyText = box.textContent;
	  const legacyClass = box.className;
	  box.textContent = previousText;
	  box.className = previousClass;
	  return {
	    hasBox: true,
	    hasRenderer: true,
	    failedText,
	    failedClass,
	    successText,
	    successClass,
	    taskBlockedText,
	    taskBlockedClass,
	    partialText,
	    partialClass,
	    legacyText,
	    legacyClass,
	    restoredText: box.textContent,
	    restoredClass: box.className
	  };
	})()
"""
    result = client.call(
        "Runtime.evaluate",
        {"expression": expression, "returnByValue": True},
        timeout_s=10,
    )
    payload = result.get("result", {}).get("result", {}).get("value")
    if not isinstance(payload, dict):
        raise RuntimeError(f"unexpected Runtime.evaluate result: {result}")
    return payload


def assert_smoke(payload: Dict[str, Any], require_blocked: bool) -> None:
    failures = []
    if payload.get("statusText") != "已连接":
        failures.append(f"frontend is not connected: {payload.get('statusText')}")
    if payload.get("activeTaskSummary") != "无任务" or payload.get("latestActiveTask"):
        failures.append("an active task is visible during read-only smoke")
    if payload.get("activeTaskSummary") == "无任务":
        active_raw = str(payload.get("activeTaskRaw") or "")
        if active_raw != "无任务":
            failures.append("active task raw panel must show only no-task state when no active task exists")
        if "last_task_result" in active_raw:
            failures.append("active task raw panel must not render historical task results as current execution")
    stop_button = payload.get("stopTaskButton") or {}
    if payload.get("activeTaskSummary") == "无任务":
        if stop_button.get("disabled") is not True:
            failures.append("stop current task button must be disabled when no active task exists")
        if "当前没有前端任务" not in str(stop_button.get("title") or ""):
            failures.append("disabled stop button must explain that no frontend task is running")
    reset_button = payload.get("resetTaskSessionButton") or {}
    if not reset_button:
        failures.append("reset task session button is missing")
    elif "显式复位导航会话" not in str(reset_button.get("title") or ""):
        failures.append("reset task session button must be labeled as an explicit navigation reset")
    if not payload.get("hasConfirmFunction"):
        failures.append("taskStartConfirmText is missing")
    if not payload.get("hasStartRequestFunction"):
        failures.append("taskStartRequest is missing")
    if not payload.get("hasActiveTaskSummaryFunction"):
        failures.append("renderActiveTaskSummary is missing")
    if not payload.get("hasDebug"):
        failures.append("window.m20proDebug is missing")
    task_next_step = str(payload.get("taskNextStep") or "")
    selected_map_status = payload.get("selectedMapStatus") or {}
    map_status_blocked = isinstance(selected_map_status, dict) and selected_map_status.get("ready") is False
    map_status_text = str(selected_map_status.get("message") or "") if isinstance(selected_map_status, dict) else ""
    if map_status_blocked:
        if "Nav2 当前加载地图不一致" not in task_next_step and "Nav2 当前加载地图不一致" not in map_status_text:
            failures.append("task next-step summary must surface selected-map/Nav2-map mismatch")
    elif payload.get("taskReadiness") and "定位未确认" in str(payload.get("taskReadiness")):
        if "定位页" not in task_next_step or "重定位成功" not in task_next_step:
            failures.append("task next-step summary must point unlocalized operators to final relocalization success")
    save_mark_button = payload.get("saveMarkButton") or {}
    use_robot_pose_button = payload.get("useRobotPoseButton") or {}
    if map_status_blocked:
        if save_mark_button.get("disabled") is not True:
            failures.append("save-mark button must be disabled when selected map differs from Nav2 map")
        if "Nav2 当前加载地图不一致" not in str(save_mark_button.get("title") or ""):
            failures.append("disabled save-mark button must mention selected-map/Nav2-map mismatch")
        if use_robot_pose_button.get("disabled") is not True:
            failures.append("use-robot-pose button must be disabled until map and localization are usable")
    elif payload.get("taskReadiness") and "定位未确认" in str(payload.get("taskReadiness")):
        if save_mark_button.get("disabled") is not True:
            failures.append("save-mark button must be disabled until localization is confirmed")
        if "重定位成功" not in str(save_mark_button.get("title") or ""):
            failures.append("disabled save-mark button must mention final relocalization success")
        if use_robot_pose_button.get("disabled") is not True:
            failures.append("use-robot-pose mark button must be disabled until localization is confirmed")
        if "重定位成功" not in str(use_robot_pose_button.get("title") or ""):
            failures.append("disabled use-robot-pose button must mention final relocalization success")
    if int(payload.get("taskPointTotal") or 0) > 0 and int(payload.get("taskPointCheckedCount") or 0) != 0:
        failures.append("task creation checkboxes should not be selected by default")
    if int(payload.get("taskPointTotal") or 0) == 0:
        if payload.get("createTaskDisabled") is not True:
            failures.append("create task button must be disabled when the current map has no task points")
        create_task_title = str(payload.get("createTaskTitle") or "")
        if map_status_blocked:
            if "Nav2 当前加载地图不一致" not in create_task_title:
                failures.append("create task button must explain selected-map/Nav2-map mismatch")
        elif "当前地图还没有任务点" not in create_task_title:
            failures.append("create task button must explain that the current map has no task points")
    task_items = payload.get("taskItems") or []
    if task_items and not all("首点：" in item and "顺序：" in item for item in task_items):
        failures.append("task cards must show first waypoint and order")
    if task_items and any("104_frontend_task_field_run.sh" in item or "现场验证入口：" in item for item in task_items):
        failures.append("task cards should not show the removed field-run wrapper")
    copy_commands = payload.get("copyCommandButtons") or []
    enabled_copy_commands = payload.get("enabledCopyCommandButtons") or []
    if task_items and any("104_frontend_task_field_run.sh" in str(item) for item in copy_commands):
        failures.append("task cards should not expose a copy button for the removed field-run wrapper")
    current_map_notice = " ".join(str(item) for item in (payload.get("currentMapTaskNotice") or []))
    no_current_map_task = "当前地图还没有任务" in current_map_notice
    if no_current_map_task:
        if payload.get("tasksPayloadIncludeAll") is not False:
            failures.append("default /api/tasks payload must not include all historical tasks")
        if int(payload.get("hiddenTaskCount") or 0) <= 0:
            failures.append("default /api/tasks payload should report hidden old-map task count")
        if task_items:
            failures.append("old-map tasks must be hidden from the main task list")
        if "旧地图任务已隐藏" not in current_map_notice:
            failures.append("task list must report hidden old-map tasks instead of rendering old task cards")
        if "默认接口不会返回" not in current_map_notice:
            failures.append("task list must explain that old-map tasks are hidden from the default API")
        if any("104_watch_frontend_task.sh 180" in str(item) for item in enabled_copy_commands):
            failures.append("old-map tasks must not expose enabled watcher copy buttons")
        if any("104_watch_frontend_task.sh 180" in str(item) or "104_frontend_task_ready_check.py --task-id" in str(item) for item in task_items):
            failures.append("old-map task cards must not display field evidence commands")
        if any("\nready\n" in str(item) for item in task_items):
            failures.append("old-map task cards must not display raw ready status as the visible task tag")
    if task_items and not no_current_map_task and not any("104_frontend_task_ready_check.py --task-id" in str(item) for item in enabled_copy_commands):
        failures.append("task cards must expose a copy button for the ready-check command")
    if task_items and not no_current_map_task and not any("104_watch_frontend_task.sh 180" in str(item) for item in enabled_copy_commands):
        failures.append("task cards must expose a copy button for the watcher command")
    if task_items and not no_current_map_task and not all("开跑前记录：" in item and "104_watch_frontend_task.sh 180" in item for item in task_items):
        failures.append("current-map task cards must show the recommended watcher command")
    if task_items and not no_current_map_task and not all("开跑前验收：" in item and "104_frontend_task_ready_check.py --task-id" in item for item in task_items):
        failures.append("current-map task cards must show the task-specific ready-check command")
    if not payload.get("hasFieldSnapshotButton"):
        failures.append("field snapshot copy button is missing")
    if not payload.get("hasFieldSnapshotFunction"):
        failures.append("buildFieldSnapshot is missing")
    field_snapshot = payload.get("fieldSnapshot") or {}
    required_snapshot_keys = [
        "captured_at",
        "frontend",
        "robot",
        "perception",
        "task_readiness",
        "task_execution_evidence",
        "recommended_task",
        "task_pose_tracker_text",
    ]
    missing_snapshot_keys = [key for key in required_snapshot_keys if key not in field_snapshot]
    if missing_snapshot_keys:
        failures.append("field snapshot missing keys: " + ", ".join(missing_snapshot_keys))
    if field_snapshot and not isinstance(field_snapshot.get("perception"), dict):
        failures.append("field snapshot perception section is invalid")
    if field_snapshot and not isinstance(field_snapshot.get("task_execution_evidence"), dict):
        failures.append("field snapshot task_execution_evidence section is invalid")
    if field_snapshot and ("map_3d" in field_snapshot or "camera_proxy" in field_snapshot):
        failures.append("field snapshot should no longer include removed map_3d/camera_proxy sections")
    if payload.get("hasMap3dButton") or payload.get("hasMap2dButton"):
        failures.append("frontend 2D/3D map mode buttons should be removed")
    camera_buttons = set(payload.get("cameraToggleButtons") or [])
    if camera_buttons:
        failures.append(f"front/rear camera toggle buttons should be removed: {sorted(camera_buttons)}")
    camera_all = set(payload.get("cameraToggleAllButtons") or [])
    if camera_all:
        failures.append(f"all-camera on/off controls should be removed: {sorted(camera_all)}")
    if payload.get("hasCameraStatus"):
        failures.append("camera status diagnostics should be removed from the frontend")
    if not payload.get("hasFrontVideo") or not payload.get("hasRearVideo"):
        failures.append("front/rear camera images should remain visible")
    if payload.get("frontVideoSrc") or payload.get("rearVideoSrc"):
        failures.append("camera images should not auto-load camera streams")
    if payload.get("frontVideoDataSrc") != "/camera/front.mjpg" or payload.get("rearVideoDataSrc") != "/camera/rear.mjpg":
        failures.append("camera images should keep MJPEG stream endpoints in data-src for on-demand viewing")
    if payload.get("frontVideoBtn") != "打开" or payload.get("rearVideoBtn") != "打开":
        failures.append("camera buttons should default to closed on page load")
    if int(payload.get("canvasWidth") or 0) <= 0 or int(payload.get("canvasHeight") or 0) <= 0:
        failures.append("map canvas has no drawable size")
    sample = payload.get("canvasSample")
    if not isinstance(sample, list) or len(sample) != 4 or int(sample[3]) == 0:
        failures.append(f"2D map canvas did not produce an opaque sample: {sample}")
    if require_blocked:
        buttons = payload.get("startButtons") or []
        if buttons and not all(item.get("disabled") for item in buttons):
            failures.append("task start buttons should be disabled in the current blocked state")
    if failures:
        raise RuntimeError("; ".join(failures))


def assert_active_summary(payload: Dict[str, Any]) -> None:
    text = str(payload.get("text") or "")
    tracker = str(payload.get("tracker") or "")
    required = [
        "smoke_task",
        "点位 F20_smoke_point",
        "Nav2 accepted",
        "狗差",
        "Nav2差",
        "位姿差",
        "反馈差",
        "Nav2反馈 2s前",
        "路径差 0.12m",
        "路径已校验",
        "下发路径版 3",
        "校验路径版 4",
        "floor_goal已发 2026-06-27 09:29:00",
        "/floor_goal 1次",
        "低进展 5s",
        "链路守护 ready",
        "synthetic summary check",
    ]
    missing = [item for item in required if item not in text]
    if missing:
        raise RuntimeError("active task summary missing fields: " + ", ".join(missing))
    tracker_required = [
        "地图位姿",
        "Nav2反馈",
        "当前目标",
        "误差",
        "任务阶段",
        "狗-目标",
        "Nav2-目标",
        "狗-Nav2",
        "路径-目标 0.12m",
        "路径已校验",
        "F20_smoke_point",
    ]
    tracker_missing = [item for item in tracker_required if item not in tracker]
    if tracker_missing:
        raise RuntimeError("pose tracker missing fields: " + ", ".join(tracker_missing))
    if payload.get("inactiveText") != "无任务" or payload.get("inactiveWithHistoryText") != "无任务":
        raise RuntimeError("inactive task summary must not render historical task results as current execution")


def assert_manual_relocalization_status(payload: Dict[str, Any]) -> None:
    if not payload.get("hasBox"):
        raise RuntimeError("localizationStatus element is missing")
    if not payload.get("hasRenderer"):
        raise RuntimeError("renderLocalizationStatus is missing")
    failed_text = str(payload.get("failedText") or "")
    failed_class = str(payload.get("failedClass") or "")
    success_text = str(payload.get("successText") or "")
    success_class = str(payload.get("successClass") or "")
    partial_text = str(payload.get("partialText") or "")
    partial_class = str(payload.get("partialClass") or "")
    legacy_text = str(payload.get("legacyText") or "")
    legacy_class = str(payload.get("legacyClass") or "")
    failed_required = [
        "重定位失败",
        "2101回执",
        "失败回执",
        "原厂定位",
        "任务页不可启动",
        "重定位未确认",
    ]
    failed_missing = [item for item in failed_required if item not in failed_text]
    if failed_missing:
        raise RuntimeError("manual 2101 failure status missing fields: " + ", ".join(failed_missing))
    if "fail" not in failed_class.split():
        raise RuntimeError(f"manual 2101 failure status must use fail class: {failed_class}")
    success_required = [
        "重定位成功",
        "2101回执",
        "已收到回执",
        "原厂定位",
        "地图位姿",
        "任务页可启动",
    ]
    success_missing = [item for item in success_required if item not in success_text]
    if success_missing:
        raise RuntimeError("manual 2101 success status missing fields: " + ", ".join(success_missing))
    if "ok" not in success_class.split():
        raise RuntimeError(f"manual 2101 success status must use ok class: {success_class}")
    task_blocked_text = str(payload.get("taskBlockedText") or "")
    task_blocked_class = str(payload.get("taskBlockedClass") or "")
    task_blocked_required = [
        "重定位成功",
        "任务页不可启动",
        "电量 22%",
    ]
    task_blocked_missing = [item for item in task_blocked_required if item not in task_blocked_text]
    if task_blocked_missing:
        raise RuntimeError("task-blocked localization success missing fields: " + ", ".join(task_blocked_missing))
    if "ok" not in task_blocked_class.split():
        raise RuntimeError(f"task-blocked localization success must use ok class: {task_blocked_class}")
    partial_required = [
        "重定位失败",
        "已收到回执",
        "原厂定位",
        "地图位姿",
        "固定地图",
        "重定位锁未清除",
        "任务页不可启动",
    ]
    partial_missing = [item for item in partial_required if item not in partial_text]
    if partial_missing:
        raise RuntimeError("manual 2101 partial-success status missing fields: " + ", ".join(partial_missing))
    if "fail" not in partial_class.split():
        raise RuntimeError(f"manual 2101 partial-success status must use fail class: {partial_class}")
    legacy_required = [
        "重定位失败",
        "已收到回执",
        "原厂定位",
        "地图位姿",
        "固定地图",
        "重定位锁未清除",
        "任务页不可启动",
    ]
    legacy_missing = [item for item in legacy_required if item not in legacy_text]
    if legacy_missing:
        raise RuntimeError("legacy backend relocalization status missing fields: " + ", ".join(legacy_missing))
    if "fail" not in legacy_class.split():
        raise RuntimeError(f"legacy backend relocalization status must use fail class: {legacy_class}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://10.21.31.104:8080/", help="Frontend URL")
    parser.add_argument("--port", type=int, default=9223, help="Chrome remote debugging port")
    parser.add_argument(
        "--require-blocked",
        action="store_true",
        help="Require all start buttons to be disabled, useful when the robot is unlocalized",
    )
    parser.add_argument("--json-out", help="Optional path to save the DOM snapshot")
    args = parser.parse_args()

    chrome = find_chrome()
    profile = tempfile.mkdtemp(prefix="m20pro_chrome_")
    process = subprocess.Popen(
        [
            chrome,
            "--headless=new",
            "--no-sandbox",
            "--disable-gpu",
            f"--remote-debugging-port={args.port}",
            f"--user-data-dir={profile}",
            args.url,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    client: Optional[CdpClient] = None
    try:
        ws_url = wait_for_devtools(args.port, 15)
        client = CdpClient(ws_url)
        client.call("Runtime.enable")
        client.call("Page.enable")
        time.sleep(1.5)
        payload = evaluate_frontend(client)
        assert_smoke(payload, args.require_blocked)
        summary_payload = evaluate_active_summary(client)
        assert_active_summary(summary_payload)
        relocalization_payload = evaluate_manual_relocalization_status(client)
        assert_manual_relocalization_status(relocalization_payload)
        payload["syntheticActiveTaskSummary"] = summary_payload
        payload["syntheticManualRelocalizationStatus"] = relocalization_payload
        if args.json_out:
            Path(args.json_out).write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        print("[OK] frontend task smoke passed")
        return 0
    finally:
        if client is not None:
            client.close()
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
        shutil.rmtree(profile, ignore_errors=True)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        raise SystemExit(1)
