import json
import math
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


INDEX_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>M20Pro 实时看板</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f4f6f8;
      --panel: #ffffff;
      --line: #d6dde5;
      --text: #17212b;
      --muted: #667483;
      --accent: #1677ff;
      --good: #15803d;
      --warn: #b45309;
      --bad: #b91c1c;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    header {
      height: 56px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 0 20px;
      background: var(--panel);
      border-bottom: 1px solid var(--line);
    }
    h1 {
      margin: 0;
      font-size: 18px;
      font-weight: 650;
      letter-spacing: 0;
    }
    main {
      display: grid;
      grid-template-columns: minmax(460px, 1fr) 360px;
      gap: 14px;
      padding: 14px;
      min-height: calc(100vh - 56px);
    }
    .map-wrap, .side {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      min-width: 0;
    }
    .map-wrap {
      display: flex;
      flex-direction: column;
      overflow: hidden;
    }
    .map-toolbar {
      min-height: 44px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      padding: 8px 12px;
      border-bottom: 1px solid var(--line);
      color: var(--muted);
      font-size: 13px;
    }
    .canvas-box {
      position: relative;
      flex: 1;
      min-height: 520px;
      background: #cfd5dc;
    }
    canvas {
      display: block;
      width: 100%;
      height: 100%;
      image-rendering: pixelated;
    }
    .side {
      display: flex;
      flex-direction: column;
      gap: 12px;
      padding: 12px;
      overflow: auto;
    }
    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }
    .tile {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 9px 10px;
      min-height: 62px;
      background: #fbfcfe;
    }
    .label {
      color: var(--muted);
      font-size: 12px;
      line-height: 18px;
    }
    .value {
      margin-top: 2px;
      font-size: 17px;
      font-weight: 650;
      overflow-wrap: anywhere;
    }
    .section {
      border-top: 1px solid var(--line);
      padding-top: 10px;
    }
    .section:first-child { border-top: 0; padding-top: 0; }
    h2 {
      margin: 0 0 8px;
      font-size: 14px;
      font-weight: 650;
    }
    .mono {
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      background: #f7f9fb;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 8px;
      max-height: 220px;
      overflow: auto;
    }
    .pill {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      border-radius: 999px;
      border: 1px solid var(--line);
      padding: 4px 9px;
      background: #fbfcfe;
      font-size: 12px;
      color: var(--muted);
    }
    .dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: var(--bad);
    }
    .dot.ok { background: var(--good); }
    .dot.warn { background: var(--warn); }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 12px;
    }
    td {
      border-top: 1px solid var(--line);
      padding: 6px 2px;
      vertical-align: top;
    }
    td:last-child {
      color: var(--muted);
      text-align: right;
      width: 74px;
    }
    @media (max-width: 980px) {
      main { grid-template-columns: 1fr; }
      .canvas-box { min-height: 420px; }
    }
  </style>
</head>
<body>
  <header>
    <h1>M20Pro 实时看板</h1>
    <span class="pill"><span id="statusDot" class="dot"></span><span id="statusText">连接中</span></span>
  </header>
  <main>
    <section class="map-wrap">
      <div class="map-toolbar">
        <span id="mapTitle">等待地图</span>
        <span id="mapMeta">-</span>
      </div>
      <div class="canvas-box">
        <canvas id="mapCanvas"></canvas>
      </div>
    </section>
    <aside class="side">
      <section class="section">
        <div class="grid">
          <div class="tile">
            <div class="label">当前楼层</div>
            <div id="floor" class="value">-</div>
          </div>
          <div class="tile">
            <div class="label">楼梯状态</div>
            <div id="stair" class="value">-</div>
          </div>
          <div class="tile">
            <div class="label">步态指令</div>
            <div id="gait" class="value">-</div>
          </div>
          <div class="tile">
            <div class="label">机器人位姿</div>
            <div id="pose" class="value">-</div>
          </div>
        </div>
      </section>
      <section class="section">
        <h2>导航状态</h2>
        <div id="nav" class="mono">等待数据</div>
      </section>
      <section class="section">
        <h2>YOLO 检测</h2>
        <div id="detections" class="mono">等待数据</div>
      </section>
      <section class="section">
        <h2>事件</h2>
        <div id="events" class="mono">等待数据</div>
      </section>
      <section class="section">
        <h2>话题状态</h2>
        <table id="topics"></table>
      </section>
    </aside>
  </main>

  <script>
    const canvas = document.getElementById("mapCanvas");
    const ctx = canvas.getContext("2d");
    const state = {
      map: null,
      mapImage: null,
      latest: null,
      mapVersion: -1
    };

    function $(id) {
      return document.getElementById(id);
    }

    function text(value) {
      return value === null || value === undefined || value === "" ? "-" : String(value);
    }

    function fmtNumber(value, digits = 2) {
      return Number.isFinite(value) ? value.toFixed(digits) : "-";
    }

    function fmtAge(age) {
      if (age === null || age === undefined) return "-";
      if (age < 1.0) return "<1s";
      return `${age.toFixed(0)}s`;
    }

    function resizeCanvas() {
      const rect = canvas.parentElement.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.floor(rect.width * dpr));
      canvas.height = Math.max(1, Math.floor(rect.height * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      draw();
    }

    function buildMapImage(map) {
      const image = document.createElement("canvas");
      image.width = map.width;
      image.height = map.height;
      const ictx = image.getContext("2d");
      const imageData = ictx.createImageData(map.width, map.height);
      for (let y = 0; y < map.height; y += 1) {
        for (let x = 0; x < map.width; x += 1) {
          const srcIdx = y * map.width + x;
          const flippedY = map.height - 1 - y;
          const dstIdx = (flippedY * map.width + x) * 4;
          const occ = map.data[srcIdx];
          let c = 205;
          if (occ >= 65) c = 0;
          else if (occ >= 0 && occ <= 25) c = 255;
          else if (occ >= 0) c = 150;
          imageData.data[dstIdx] = c;
          imageData.data[dstIdx + 1] = c;
          imageData.data[dstIdx + 2] = c;
          imageData.data[dstIdx + 3] = 255;
        }
      }
      ictx.putImageData(imageData, 0, 0);
      return image;
    }

    function getView() {
      const map = state.map;
      const rect = canvas.getBoundingClientRect();
      if (!map) {
        return { scale: 1, ox: 0, oy: 0, rect };
      }
      const scale = Math.min(rect.width / map.width, rect.height / map.height);
      const drawW = map.width * scale;
      const drawH = map.height * scale;
      return {
        scale,
        ox: (rect.width - drawW) / 2,
        oy: (rect.height - drawH) / 2,
        rect
      };
    }

    function worldToCanvas(x, y) {
      const map = state.map;
      const view = getView();
      if (!map) return null;
      const mx = (x - map.origin.x) / map.resolution;
      const my = map.height - (y - map.origin.y) / map.resolution;
      return {
        x: view.ox + mx * view.scale,
        y: view.oy + my * view.scale
      };
    }

    function drawArrow(pose) {
      const p = worldToCanvas(pose.x, pose.y);
      if (!p) return;
      ctx.save();
      ctx.translate(p.x, p.y);
      ctx.rotate(-pose.yaw);
      ctx.fillStyle = "#1677ff";
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(15, 0);
      ctx.lineTo(-10, -8);
      ctx.lineTo(-6, 0);
      ctx.lineTo(-10, 8);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    }

    function drawPath(path) {
      if (!path || !path.points || path.points.length < 2) return;
      ctx.save();
      ctx.strokeStyle = "#ff7a00";
      ctx.lineWidth = 3;
      ctx.beginPath();
      let started = false;
      for (const point of path.points) {
        const p = worldToCanvas(point.x, point.y);
        if (!p) continue;
        if (!started) {
          ctx.moveTo(p.x, p.y);
          started = true;
        } else {
          ctx.lineTo(p.x, p.y);
        }
      }
      ctx.stroke();
      ctx.restore();
    }

    function drawObstacles(items) {
      if (!items || items.length === 0) return;
      ctx.save();
      for (const item of items) {
        const p = worldToCanvas(item.x, item.y);
        if (!p) continue;
        const radius = Math.max(5, Math.min(22, (item.scale_x || 0.4) / state.map.resolution * getView().scale * 0.5));
        ctx.fillStyle = "rgba(185, 28, 28, 0.82)";
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }
      ctx.restore();
    }

    function draw() {
      const rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      ctx.fillStyle = "#cfd5dc";
      ctx.fillRect(0, 0, rect.width, rect.height);
      if (!state.map || !state.mapImage) {
        ctx.fillStyle = "#667483";
        ctx.font = "15px system-ui, sans-serif";
        ctx.fillText("等待 /map 数据", 20, 30);
        return;
      }
      const view = getView();
      ctx.drawImage(
        state.mapImage,
        view.ox,
        view.oy,
        state.map.width * view.scale,
        state.map.height * view.scale
      );
      ctx.strokeStyle = "#4b5563";
      ctx.lineWidth = 1;
      ctx.strokeRect(view.ox, view.oy, state.map.width * view.scale, state.map.height * view.scale);
      const latest = state.latest;
      if (latest) {
        drawPath(latest.path);
        drawObstacles(latest.dynamic_obstacles);
        if (latest.pose) drawArrow(latest.pose);
      }
    }

    async function fetchJson(url) {
      const res = await fetch(url, { cache: "no-store" });
      if (!res.ok) throw new Error(`${url} ${res.status}`);
      return await res.json();
    }

    async function refreshMap(version) {
      if (version === state.mapVersion) return;
      const map = await fetchJson("/api/map");
      if (!map.available) return;
      state.map = map;
      state.mapImage = buildMapImage(map);
      state.mapVersion = map.version;
      $("mapTitle").textContent = `地图版本 ${map.version}`;
      $("mapMeta").textContent = `${map.width} x ${map.height}, ${map.resolution.toFixed(3)} m/格`;
    }

    function updateState(s) {
      state.latest = s;
      $("floor").textContent = text(s.floor);
      $("stair").textContent = text(s.stair_status);
      $("gait").textContent = text(s.gait_command);
      if (s.pose) {
        $("pose").textContent = `x ${fmtNumber(s.pose.x)} / y ${fmtNumber(s.pose.y)} / yaw ${fmtNumber(s.pose.yaw_deg, 0)}°`;
      } else {
        $("pose").textContent = "-";
      }
      $("nav").textContent = JSON.stringify({
        路径点数: s.path ? s.path.points.length : 0,
        动态障碍物: s.dynamic_obstacles ? s.dynamic_obstacles.length : 0,
        更新时间: s.node_time_text
      }, null, 2);
      const det = s.detections && (s.detections.parsed || s.detections.raw);
      $("detections").textContent = det ? JSON.stringify(det, null, 2) : "等待数据";
      $("events").textContent = s.events && s.events.length
        ? JSON.stringify(s.events.slice(-5), null, 2)
        : "等待数据";
      const table = $("topics");
      table.innerHTML = "";
      for (const [name, info] of Object.entries(s.topics || {})) {
        const tr = document.createElement("tr");
        const left = document.createElement("td");
        const right = document.createElement("td");
        left.textContent = name;
        right.textContent = info.available ? fmtAge(info.age_sec) : "无数据";
        tr.appendChild(left);
        tr.appendChild(right);
        table.appendChild(tr);
      }
    }

    async function loop() {
      const dot = $("statusDot");
      const label = $("statusText");
      try {
        const s = await fetchJson("/api/state");
        await refreshMap(s.map_version);
        updateState(s);
        dot.className = "dot ok";
        label.textContent = "已连接";
        draw();
      } catch (err) {
        dot.className = "dot warn";
        label.textContent = "等待服务";
        console.warn(err);
      } finally {
        setTimeout(loop, 1000);
      }
    }

    window.addEventListener("resize", resizeCanvas);
    resizeCanvas();
    loop();
  </script>
</body>
</html>
"""


def _stamp_to_float(stamp: Any) -> Optional[float]:
    if stamp is None:
        return None
    sec = float(getattr(stamp, "sec", 0))
    nanosec = float(getattr(stamp, "nanosec", 0))
    value = sec + nanosec * 1e-9
    return value if value > 0.0 else None


def _yaw_from_pose(pose: Pose) -> float:
    q = pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _pose_to_dict(pose: Pose) -> Dict[str, float]:
    yaw = _yaw_from_pose(pose)
    return {
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "z": float(pose.position.z),
        "yaw": yaw,
        "yaw_deg": math.degrees(yaw),
    }


def _parse_json_text(text: str) -> Any:
    try:
        return json.loads(text)
    except Exception:
        return None


class _ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


class WebDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("m20pro_web_dashboard")
        self._declare_parameters()

        self._lock = threading.Lock()
        self._state: Dict[str, Any] = {
            "floor": None,
            "stair_status": None,
            "gait_command": None,
            "pose": None,
            "path": {"version": 0, "points": []},
            "map": None,
            "map_version": 0,
            "dynamic_obstacles": [],
            "detections": None,
            "events": [],
            "topics": {},
        }

        self._create_subscriptions()
        self._server = self._start_http_server()

    def _declare_parameters(self) -> None:
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("current_floor_topic", "/m20pro/current_floor")
        self.declare_parameter("stair_status_topic", "/m20pro/stair_status")
        self.declare_parameter("gait_command_topic", "/m20pro/gait_command")
        self.declare_parameter("pose_topic", "/m20pro_tcp_bridge/map_pose")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("dynamic_obstacle_topic", "/dynamic_obstacle_markers")
        self.declare_parameter("detections_topic", "/m20pro_yolov8_inspection/detections")
        self.declare_parameter("events_topic", "/m20pro_yolov8_inspection/events")
        self.declare_parameter("annotated_image_topic", "/m20pro_yolov8_inspection/annotated_image")
        self.declare_parameter("subscribe_annotated_image", False)
        self.declare_parameter("max_path_points", 800)
        self.declare_parameter("max_events", 20)

    def _topic(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _create_subscriptions(self) -> None:
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(String, self._topic("current_floor_topic"), self._on_current_floor, 10)
        self.create_subscription(String, self._topic("stair_status_topic"), self._on_stair_status, 10)
        self.create_subscription(String, self._topic("gait_command_topic"), self._on_gait_command, 10)
        self.create_subscription(PoseStamped, self._topic("pose_topic"), self._on_pose, 20)
        self.create_subscription(Path, self._topic("plan_topic"), self._on_path, 5)
        self.create_subscription(OccupancyGrid, self._topic("map_topic"), self._on_map, map_qos)
        self.create_subscription(MarkerArray, self._topic("dynamic_obstacle_topic"), self._on_markers, 10)
        self.create_subscription(String, self._topic("detections_topic"), self._on_detections, 10)
        self.create_subscription(String, self._topic("events_topic"), self._on_event, 10)

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

    def _on_gait_command(self, msg: String) -> None:
        with self._lock:
            self._state["gait_command"] = msg.data
            self._mark_topic("gait_command")

    def _on_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            pose = _pose_to_dict(msg.pose)
            stamp = _stamp_to_float(msg.header.stamp)
            if stamp is not None:
                pose["stamp"] = stamp
            self._state["pose"] = pose
            self._mark_topic("pose")

    def _on_path(self, msg: Path) -> None:
        max_points = int(self.get_parameter("max_path_points").value)
        poses = msg.poses
        if len(poses) > max_points:
            step = max(1, math.ceil(len(poses) / max_points))
            poses = poses[::step]
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
            }
            self._mark_topic("path")

    def _on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        origin = _pose_to_dict(info.origin)
        map_payload = {
            "available": True,
            "version": int(time.time() * 1000),
            "frame_id": msg.header.frame_id,
            "stamp": _stamp_to_float(msg.header.stamp),
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
                "parsed": _parse_json_text(msg.data),
            }
            self._mark_topic("detections")

    def _on_event(self, msg: String) -> None:
        max_events = int(self.get_parameter("max_events").value)
        event = {
            "last_update": time.time(),
            "raw": msg.data,
            "parsed": _parse_json_text(msg.data),
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

    def _snapshot(self) -> Dict[str, Any]:
        now = time.time()
        with self._lock:
            snapshot = dict(self._state)
            snapshot["path"] = dict(self._state["path"])
            snapshot["dynamic_obstacles"] = list(self._state["dynamic_obstacles"])
            snapshot["events"] = list(self._state["events"])
            snapshot["topics"] = {
                key: dict(value)
                for key, value in self._state["topics"].items()
            }
            snapshot.pop("map", None)

        snapshot["ok"] = True
        snapshot["node_time"] = now
        snapshot["node_time_text"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
        for value in snapshot["topics"].values():
            last_update = value.get("last_update")
            value["age_sec"] = None if last_update is None else max(0.0, now - float(last_update))
        return snapshot

    def _map_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            current_map = self._state.get("map")
            if not current_map:
                return {"available": False}
            return dict(current_map)

    def _start_http_server(self) -> _ReusableThreadingHTTPServer:
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        node = self

        class DashboardHandler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                parsed = urlparse(self.path)
                if parsed.path == "/":
                    self._send_bytes(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
                elif parsed.path == "/api/state":
                    self._send_json(node._snapshot())
                elif parsed.path == "/api/map":
                    self._send_json(node._map_snapshot())
                elif parsed.path == "/healthz":
                    self._send_json({"ok": True})
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

            def log_message(self, fmt: str, *args: Any) -> None:
                node.get_logger().debug(fmt % args)

            def _send_json(self, payload: Dict[str, Any]) -> None:
                self._send_bytes(
                    json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
                    "application/json; charset=utf-8",
                )

            def _send_bytes(self, payload: bytes, content_type: str) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(payload)

        server = _ReusableThreadingHTTPServer((host, port), DashboardHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.get_logger().info(f"M20Pro web dashboard listening on http://{host}:{port}")
        return server

    def destroy_node(self) -> bool:
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
