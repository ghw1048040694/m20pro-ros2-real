import json
import math
import os
import re
import shlex
import shutil
import subprocess
import threading
import time
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FsPath
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from .pcd_derived import process_imported_map

try:
    from lifecycle_msgs.srv import GetState
except ImportError:  # pragma: no cover - ROS lifecycle package should exist on robot.
    GetState = None

try:
    from nav2_msgs.srv import ClearEntireCostmap
except ImportError:  # pragma: no cover - Nav2 package should exist on robot.
    ClearEntireCostmap = None

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


MANUAL_POINT_TYPES: Dict[str, Dict[str, Any]] = {
    "transition": {
        "label": "过渡点",
        "point_info": 0,
        "default_dwell_s": 0.0,
        "default_nav_mode": 0,
    },
    "task": {
        "label": "任务点",
        "point_info": 1,
        "default_dwell_s": 5.0,
        "default_nav_mode": 1,
    },
    "charge": {
        "label": "充电点",
        "point_info": 3,
        "default_dwell_s": 0.0,
        "default_nav_mode": 1,
    },
}

UI_TYPE_TO_MANUAL_POINT_TYPE = {
    "patrol": "task",
    "task": "task",
    "inspection": "task",
    "transition": "transition",
    "stair_entry": "transition",
    "stair_switch": "transition",
    "stair_exit": "transition",
    "charge": "charge",
    "charging": "charge",
}

DEFAULT_VENDOR_NAVIGATION = {
    "Value": 0,
    "MapID": 0,
    "Gait": 12,
    "Speed": 1,
    "Manner": 0,
    "ObsMode": 0,
    "NavMode": 1,
}


INDEX_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>M20Pro 现场操作台</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f3f5f7;
      --panel: #ffffff;
      --soft: #f8fafc;
      --line: #d8dee6;
      --text: #17212b;
      --muted: #667483;
      --accent: #0f6bff;
      --accent-soft: #e8f1ff;
      --good: #15803d;
      --warn: #b45309;
      --bad: #b91c1c;
      --orange: #f97316;
      --cyan: #0891b2;
      --violet: #7c3aed;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      overflow: hidden;
    }
    header {
      min-height: 58px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 14px;
      padding: 9px 16px;
      background: var(--panel);
      border-bottom: 1px solid var(--line);
    }
    h1 {
      margin: 0;
      font-size: 18px;
      font-weight: 700;
      letter-spacing: 0;
    }
    .subhead {
      margin-top: 2px;
      color: var(--muted);
      font-size: 12px;
    }
    main {
      display: grid;
      grid-template-columns: minmax(520px, 1fr) 420px;
      gap: 12px;
      padding: 12px;
      height: calc(100vh - 58px);
      min-height: 0;
    }
    .map-wrap, .side {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      min-width: 0;
      min-height: 0;
    }
    .map-wrap {
      display: flex;
      flex-direction: column;
      overflow: hidden;
    }
    .map-toolbar {
      min-height: 48px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      padding: 8px 12px;
      border-bottom: 1px solid var(--line);
      color: var(--muted);
      font-size: 13px;
    }
    .map-toolbar strong {
      color: var(--text);
      font-weight: 650;
    }
    .map-toolbar-left {
      min-width: 0;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .map-tools {
      display: flex;
      align-items: center;
      gap: 6px;
      flex-wrap: wrap;
      justify-content: flex-end;
    }
    .map-tools button {
      min-height: 30px;
      padding: 5px 9px;
      font-size: 12px;
    }
    .map-tools button.mode {
      min-width: 58px;
    }
    .zoom-readout {
      min-width: 48px;
      text-align: center;
      font-size: 12px;
      color: var(--muted);
    }
    button.active-tool {
      border-color: var(--accent);
      background: #eaf2ff;
      color: var(--accent);
    }
    .canvas-box {
      position: relative;
      flex: 1;
      min-height: 0;
      background: #cfd5dc;
    }
    canvas {
      display: block;
      width: 100%;
      height: 100%;
      image-rendering: pixelated;
      touch-action: none;
      cursor: crosshair;
    }
    canvas.panning {
      cursor: grabbing;
    }
    .crosshair {
      position: absolute;
      right: 10px;
      bottom: 10px;
      max-width: calc(100% - 20px);
      padding: 7px 9px;
      border-radius: 6px;
      background: rgba(255, 255, 255, 0.92);
      border: 1px solid var(--line);
      color: var(--muted);
      font-size: 12px;
      pointer-events: none;
    }
    .side {
      display: flex;
      flex-direction: column;
      overflow: hidden;
    }
    .tabs {
      display: grid;
      grid-template-columns: repeat(7, minmax(0, 1fr));
      gap: 4px;
      padding: 8px;
      border-bottom: 1px solid var(--line);
      background: var(--soft);
    }
    button, select, input {
      font: inherit;
    }
    button {
      border: 1px solid var(--line);
      border-radius: 6px;
      min-height: 34px;
      padding: 0 10px;
      background: #ffffff;
      color: var(--text);
      cursor: pointer;
    }
    button:disabled {
      opacity: 0.55;
      cursor: not-allowed;
    }
    button:hover { border-color: #9fb3c8; }
    button.primary {
      background: var(--accent);
      color: #ffffff;
      border-color: var(--accent);
      font-weight: 650;
    }
    button.danger {
      color: var(--bad);
      border-color: #efb2b2;
    }
    button.tab {
      min-height: 34px;
      padding: 0 6px;
      font-size: 12px;
      color: var(--muted);
      background: transparent;
      white-space: nowrap;
    }
    button.tab.active {
      color: var(--accent);
      background: #ffffff;
      border-color: #b7cdf0;
      font-weight: 650;
    }
    .content {
      flex: 1;
      overflow: auto;
      padding: 12px;
    }
    .panel {
      display: none;
    }
    .panel.active {
      display: block;
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
    .tile.wide {
      grid-column: 1 / -1;
    }
    .label {
      color: var(--muted);
      font-size: 12px;
      line-height: 18px;
    }
    .value {
      margin-top: 2px;
      font-size: 16px;
      font-weight: 650;
      overflow-wrap: anywhere;
    }
    .section {
      border-top: 1px solid var(--line);
      padding-top: 11px;
      margin-top: 12px;
    }
    .section:first-child { border-top: 0; padding-top: 0; margin-top: 0; }
    h2 {
      margin: 0 0 8px;
      font-size: 14px;
      font-weight: 700;
    }
    .row {
      display: grid;
      grid-template-columns: 118px minmax(0, 1fr);
      gap: 8px;
      align-items: center;
      margin-bottom: 8px;
    }
    .row label {
      color: var(--muted);
      font-size: 12px;
    }
    input, select {
      width: 100%;
      min-height: 34px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 0 9px;
      background: #ffffff;
      color: var(--text);
    }
    .actions {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 8px;
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
      max-height: 210px;
      overflow: auto;
    }
    .video-grid {
      display: grid;
      grid-template-columns: 1fr;
      gap: 8px;
    }
    .video-card {
      border: 1px solid var(--line);
      border-radius: 6px;
      overflow: hidden;
      background: #0b1117;
    }
    .video-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      padding: 7px 9px;
      background: #fbfcfe;
      color: var(--muted);
      font-size: 12px;
      border-bottom: 1px solid var(--line);
    }
    .video-head strong {
      color: var(--text);
      font-weight: 650;
    }
    .video-head a {
      color: var(--accent);
      text-decoration: none;
    }
    .video-card img {
      display: block;
      width: 100%;
      aspect-ratio: 16 / 9;
      object-fit: contain;
      background: #0b1117;
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
      white-space: nowrap;
    }
    .dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: var(--bad);
    }
    .dot.ok { background: var(--good); }
    .dot.warn { background: var(--warn); }
    .list {
      display: flex;
      flex-direction: column;
      gap: 8px;
    }
    .item {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 8px;
      background: #fbfcfe;
    }
    .item-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      margin-bottom: 4px;
      font-size: 13px;
      font-weight: 650;
    }
    .item-meta {
      color: var(--muted);
      font-size: 12px;
      line-height: 1.5;
      overflow-wrap: anywhere;
    }
    .small {
      font-size: 12px;
      color: var(--muted);
      line-height: 1.5;
    }
    .tag {
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      padding: 2px 7px;
      font-size: 11px;
      background: var(--accent-soft);
      color: var(--accent);
      white-space: nowrap;
    }
    .preflight-summary {
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 9px;
      background: #fbfcfe;
    }
    .preflight-summary.ok {
      border-color: #86efac;
      background: #f0fdf4;
    }
    .preflight-summary.warn {
      border-color: #fcd34d;
      background: #fffbeb;
    }
    .preflight-summary.fail {
      border-color: #fecaca;
      background: #fef2f2;
    }
    .check-row {
      display: grid;
      grid-template-columns: 72px minmax(0, 1fr);
      gap: 8px;
      align-items: start;
      border-top: 1px solid var(--line);
      padding: 7px 0;
      font-size: 12px;
    }
    .check-row:first-child {
      border-top: 0;
    }
    .check-status {
      font-weight: 700;
    }
    .check-status.ok { color: var(--good); }
    .check-status.warn { color: var(--warn); }
    .check-status.fail { color: var(--bad); }
    .checkline {
      display: grid;
      grid-template-columns: 22px minmax(0, 1fr);
      gap: 7px;
      align-items: start;
      margin: 7px 0;
      font-size: 13px;
    }
    .checkline input { width: 16px; min-height: 16px; margin-top: 2px; }
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
      width: 76px;
    }
    @media (max-width: 1080px) {
      body { overflow: auto; }
      main { grid-template-columns: 1fr; height: auto; min-height: calc(100vh - 58px); }
      .canvas-box { min-height: min(62vh, 560px); }
      .side { min-height: 520px; }
    }
    @media (max-width: 560px) {
      header { align-items: flex-start; flex-direction: column; }
      main { padding: 8px; }
      .tabs { grid-template-columns: repeat(3, minmax(0, 1fr)); }
      .grid { grid-template-columns: 1fr; }
      .row { grid-template-columns: 1fr; gap: 4px; }
    }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>M20Pro 现场操作台</h1>
      <div class="subhead">建图、拉图、选图、标点、任务与实时状态统一入口</div>
    </div>
    <span class="pill"><span id="statusDot" class="dot"></span><span id="statusText">连接中</span></span>
  </header>
  <main>
    <section class="map-wrap">
      <div class="map-toolbar">
        <span class="map-toolbar-left"><strong id="mapTitle">等待地图</strong> <span id="mapMeta">-</span></span>
        <span class="map-tools">
          <button id="zoomOutBtn" title="缩小地图">-</button>
          <span id="zoomReadout" class="zoom-readout">100%</span>
          <button id="zoomInBtn" title="放大地图">+</button>
          <button id="panModeBtn" title="开启后拖动地图，不会标点">平移</button>
          <button id="fitMapBtn" title="恢复整图适配">适配</button>
          <button id="centerRobotBtn" title="把机器人位置移动到视图中心">居中机器人</button>
          <button id="map2dBtn" class="mode active-tool" title="显示 2D 栅格地图">2D地图</button>
          <button id="map3dBtn" class="mode" title="显示 PCD 派生轻量 3D 地形">3D地图</button>
          <span id="mapMode" class="pill">实时 /map</span>
        </span>
      </div>
      <div class="canvas-box">
        <canvas id="mapCanvas"></canvas>
        <div id="cursor" class="crosshair">拖拽地图取点和朝向</div>
      </div>
    </section>
    <aside class="side">
      <nav class="tabs">
        <button class="tab active" data-tab="live">看板</button>
        <button class="tab" data-tab="localize">定位</button>
        <button class="tab" data-tab="mapping">建图</button>
        <button class="tab" data-tab="maps">地图</button>
        <button class="tab" data-tab="marks">标点</button>
        <button class="tab" data-tab="tasks">任务</button>
        <button class="tab" data-tab="preflight">自检</button>
      </nav>
      <div class="content">
        <section id="tab-live" class="panel active">
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
              <div class="label">模式/步态</div>
              <div id="gait" class="value">-</div>
            </div>
            <div class="tile wide">
              <div class="label">机器人位姿</div>
              <div id="pose" class="value">-</div>
            </div>
            <div class="tile">
              <div class="label">定位状态</div>
              <div id="localization" class="value">-</div>
            </div>
            <div class="tile">
              <div class="label">原厂导航</div>
              <div id="factoryNav" class="value">-</div>
            </div>
            <div class="tile">
              <div class="label">机器狗电量</div>
              <div id="battery" class="value">-</div>
            </div>
          </div>
          <div class="section">
            <h2>实时视频</h2>
            <div class="video-grid">
              <div class="video-card">
                <div class="video-head"><strong>前广角相机</strong><a href="/camera/front.mjpg" target="_blank">新窗口</a></div>
                <img id="frontVideo" src="/camera/front.mjpg" alt="前广角相机画面" />
              </div>
              <div class="video-card">
                <div class="video-head"><strong>后广角相机</strong><a href="/camera/rear.mjpg" target="_blank">新窗口</a></div>
                <img id="rearVideo" src="/camera/rear.mjpg" alt="后广角相机画面" />
              </div>
            </div>
            <div class="small" style="margin-top:8px;">视频由 103 RTSP 转为网页可看的 MJPEG；如果画面未出现，优先检查 103:8554 是否可达。</div>
          </div>
          <div class="section">
            <h2>导航状态</h2>
            <div id="nav" class="mono">等待数据</div>
          </div>
          <div class="section">
            <h2>YOLO 检测</h2>
            <div id="detections" class="mono">等待数据</div>
          </div>
          <div class="section">
            <h2>事件</h2>
            <div id="events" class="mono">等待数据</div>
          </div>
          <div class="section">
            <h2>话题状态</h2>
            <table id="topics"></table>
          </div>
        </section>

        <section id="tab-localize" class="panel">
          <div class="section">
            <h2>网页重定位</h2>
            <div class="row">
              <label>坐标 X/Y</label>
              <input id="locXY" placeholder="拖拽地图自动填入" />
            </div>
            <div class="row">
              <label>朝向角(rad)</label>
              <input id="locYaw" value="0.0" />
            </div>
            <div class="row">
              <label>楼层</label>
              <input id="locFloor" value="F20" />
            </div>
            <div class="actions">
              <button class="primary" id="sendInitialPoseBtn">执行重定位</button>
              <button id="useRobotPoseForLocBtn">使用当前机器人位姿</button>
            </div>
            <label class="checkline">
              <input id="scanOverlayToggle" type="checkbox" checked />
              <span>显示实时激光轮廓</span>
            </label>
            <div class="small" id="scanOverlayStatus">等待 /scan 数据</div>
            <div id="localizeLiveStatus" class="mono" style="margin-top:8px;">等待定位状态</div>
            <div class="small" style="margin-top:8px;">重定位按原厂 106 /initialpose 路径执行；导航不必预先就绪，但固定地图和 /scan/点云必须可用。</div>
            <div id="localizeLog" class="mono" style="margin-top:8px;">先在地图上拖箭头，让红色激光轮廓贴合地图后再执行重定位。</div>
          </div>
        </section>

        <section id="tab-mapping" class="panel">
          <div class="section">
            <h2>建图向导</h2>
            <div class="row">
              <label>项目名称</label>
              <input id="projectName" value="M20Pro 工地巡检" />
            </div>
            <div class="row">
              <label>建筑/区域</label>
              <input id="buildingName" value="主楼" />
            </div>
            <div class="row">
              <label>建图模式</label>
              <select id="mappingMode">
                <option value="multi">多楼层</option>
                <option value="single">单楼层</option>
              </select>
            </div>
            <div class="row">
              <label>楼层编号</label>
              <input id="mappingFloors" value="F19,F20,F21" />
            </div>
            <div class="row">
              <label>当前楼层</label>
              <input id="mappingActiveFloor" value="F20" />
            </div>
            <div class="row">
              <label>地图名称</label>
              <input id="mappingMapName" placeholder="留空自动按楼层和时间生成" />
            </div>
            <div class="actions">
              <button id="checkMappingEnvBtn">检查 106 环境</button>
              <button class="primary" id="createSessionBtn">建立建图任务</button>
              <button id="startMappingBtn">启动 106 建图</button>
              <button id="finishMappingBtn">完成/保存建图</button>
            </div>
            <div class="small" style="margin-top:8px;">
              默认按《M20 Pro 软件使用手册》通过 106 的 drmap 建图；需要 104 能免密 SSH 到 106，且 sudo drmap 不要求交互输入密码。
            </div>
          </div>
          <div class="section">
            <h2>拉取 106 当前地图</h2>
            <div class="row">
              <label>地图楼层</label>
              <input id="importFloor" value="F20" />
            </div>
            <div class="row">
              <label>地图名称</label>
              <input id="importName" placeholder="留空自动生成" />
            </div>
            <div class="actions">
              <button class="primary" id="importMapBtn">从 106 拉到 104</button>
            </div>
            <div id="mappingLog" class="mono" style="margin-top:8px;">等待操作</div>
          </div>
        </section>

        <section id="tab-maps" class="panel">
          <div class="section">
            <h2>地图选择</h2>
            <div class="row">
              <label>显示地图</label>
              <select id="mapSelect"></select>
            </div>
            <div class="actions">
              <button class="primary" id="selectMapBtn">设为当前显示</button>
              <button id="reloadMapsBtn">刷新列表</button>
            </div>
            <div class="small" style="margin-top:8px;">
              页面默认使用项目默认楼层固定地图；切换下拉框会立即显示对应 2D 栅格图，实时 `/map` 只作为临时调试视图。
            </div>
          </div>
          <div class="section">
            <h2>地图列表</h2>
            <div id="mapList" class="list"></div>
          </div>
        </section>

        <section id="tab-marks" class="panel">
          <div class="section">
            <h2>地图标点</h2>
            <div class="row">
              <label>点位类型</label>
              <select id="markType">
                <option value="patrol">巡检点</option>
                <option value="stair_entry">步态切换点</option>
                <option value="stair_switch">楼层切换点</option>
                <option value="stair_exit">出楼梯点</option>
                <option value="charge">充电点</option>
                <option value="transition">过渡点</option>
              </select>
            </div>
            <div class="row">
              <label>手册类型</label>
              <select id="manualPointType">
                <option value="task">任务点</option>
                <option value="transition">过渡点</option>
                <option value="charge">充电点</option>
              </select>
            </div>
            <div class="row">
              <label>楼层</label>
              <input id="markFloor" value="F20" />
            </div>
            <div class="row">
              <label>名称</label>
              <input id="markLabel" placeholder="例如 20层东区2008房间" />
            </div>
            <div class="row">
              <label>区域</label>
              <input id="markArea" placeholder="例如 东区、核心筒、样板段" />
            </div>
            <div class="row">
              <label>房间/部位</label>
              <input id="markRoom" placeholder="例如 2008房间、西侧走廊" />
            </div>
            <div class="row">
              <label>结果前缀</label>
              <input id="markResultPrefix" placeholder="留空自动生成昂锐雷达结果文件名前缀" />
            </div>
            <div class="row">
              <label>坐标 X/Y</label>
              <input id="markXY" placeholder="拖拽地图自动填入" />
            </div>
            <div class="row">
              <label>朝向角(rad)</label>
              <input id="markYaw" value="0.0" />
            </div>
            <div class="row">
              <label>停留(s)</label>
              <input id="markDwell" value="5.0" />
            </div>
            <div class="row">
              <label>步态</label>
              <select id="markGait">
                <option value="12">平地敏捷（12）</option>
              </select>
            </div>
            <div class="row">
              <label>速度</label>
              <select id="markSpeed">
                <option value="1">低速（1）</option>
                <option value="2">高速（2）</option>
              </select>
            </div>
            <div class="row">
              <label>行走方式</label>
              <select id="markManner">
                <option value="0">前进（0）</option>
                <option value="1">倒退（1）</option>
              </select>
            </div>
            <div class="row">
              <label>停避障</label>
              <select id="markObsMode">
                <option value="0">开启（0）</option>
                <option value="1">关闭（1）</option>
              </select>
            </div>
            <div class="row">
              <label>导航方式</label>
              <select id="markNavMode">
                <option value="1">自主导航（1）</option>
                <option value="0">直线导航（0）</option>
              </select>
            </div>
            <div class="actions">
              <button class="primary" id="saveMarkBtn">保存点位</button>
              <button id="useRobotPoseBtn">使用当前机器人位姿</button>
            </div>
            <div id="markSaveStatus" class="mono" style="margin-top:8px;">尚未保存点位</div>
          </div>
          <div class="section">
            <h2>当前地图点位</h2>
            <div id="annotationList" class="list"></div>
          </div>
        </section>

        <section id="tab-preflight" class="panel">
          <div class="section">
            <h2>作业前自检</h2>
            <div id="preflightSummary" class="preflight-summary">尚未自检</div>
            <div class="actions">
              <button class="primary" id="runPreflightBtn">开机基础自检</button>
              <button id="refreshPreflightBtn">刷新结果</button>
            </div>
            <div class="small" style="margin-top:8px;">
              基础自检只确认全量系统、网页、原始点云、电量和原厂状态链路；定位和代价地图需要到测试场地重定位后再看，不会阻止手柄原厂遥控。
            </div>
          </div>
          <div class="section">
            <h2>检查项</h2>
            <div id="preflightItems" class="list"></div>
          </div>
          <div class="section">
            <h2>原始结果</h2>
            <div id="preflightRaw" class="mono">等待自检</div>
          </div>
        </section>

        <section id="tab-tasks" class="panel">
          <div class="section">
            <h2>作业前状态</h2>
            <div id="taskPreflightSummary" class="preflight-summary">尚未自检</div>
            <div class="actions">
              <button id="taskRunPreflightBtn">开机基础自检</button>
            </div>
          </div>
          <div class="section">
            <h2>任务编排</h2>
            <div class="row">
              <label>任务名称</label>
              <input id="taskName" value="日常巡检任务" />
            </div>
            <div id="taskPointList" class="mono">请先选择地图并标点</div>
            <div class="actions">
              <button class="primary" id="createTaskBtn">生成任务</button>
              <button id="reloadTasksBtn">刷新任务</button>
            </div>
          </div>
          <div class="section">
            <h2>任务列表</h2>
            <div id="taskList" class="list"></div>
          </div>
          <div class="section">
            <h2>当前执行</h2>
            <div id="activeTask" class="mono">无任务</div>
            <div class="actions">
              <button class="danger" id="stopTaskBtn">停止当前任务</button>
              <button id="resetTaskSessionBtn">复位导航状态</button>
            </div>
          </div>
        </section>
      </div>
    </aside>
  </main>

  <script>
    const canvas = document.getElementById("mapCanvas");
    const ctx = canvas.getContext("2d");
    const state = {
      map: null,
      mapImage: null,
      latest: null,
      liveMapVersion: -1,
      selectedMapId: null,
      fileMapVersion: -1,
      maps: [],
      annotations: [],
      tasks: [],
      sessionId: null,
      markDraft: null,
      localizeDraft: null,
      markPointer: null,
      panPointer: null,
      view: {
        zoom: 1,
        panX: 0,
        panY: 0,
        panMode: false
      },
      preflight: null,
      lastRelocalizationStamp: null,
      relocalizationApiLogUntil: 0,
      scanOverlay: true,
      mapViewMode: "2d",
      mapModeLabel: "实时 /map",
      terrainMessage: "请选择固定地图；实时 /map 没有离线 PCD 派生 3D 地形。",
      terrain: null,
      stairZones: [],
      terrainView: {
        zoom: 1,
        panX: 0,
        panY: 0,
        rotX: 0.95,
        rotZ: -0.65,
        zScale: 1.8,
        pointer: null
      }
    };
    const manualPointTypeNames = {
      transition: "过渡点",
      task: "任务点",
      charge: "充电点"
    };
    const manualTypeByUiType = {
      patrol: "task",
      task: "task",
      stair_entry: "transition",
      stair_switch: "transition",
      stair_exit: "transition",
      transition: "transition",
      charge: "charge"
    };
    const defaultByManualType = {
      transition: { dwell: 0, gait: 12, speed: 1, manner: 0, obsMode: 0, navMode: 0 },
      task: { dwell: 5, gait: 12, speed: 1, manner: 0, obsMode: 0, navMode: 1 },
      charge: { dwell: 0, gait: 12, speed: 1, manner: 0, obsMode: 0, navMode: 1 }
    };
    const typeNames = {
      patrol: "巡检点",
      stair_entry: "步态切换点",
      stair_switch: "楼层切换点",
      stair_exit: "出楼梯点",
      charge: "充电点",
      transition: "过渡点"
    };
    const typeColors = {
      patrol: "#0f6bff",
      stair_entry: "#f97316",
      stair_switch: "#7c3aed",
      stair_exit: "#0891b2",
      charge: "#15803d",
      transition: "#b45309"
    };

    function $(id) { return document.getElementById(id); }
    function text(value) { return value === null || value === undefined || value === "" ? "-" : String(value); }
    function fmtNumber(value, digits = 2) { return Number.isFinite(value) ? value.toFixed(digits) : "-"; }
    function fmtAge(age) {
      if (age === null || age === undefined) return "-";
      if (age < 1.0) return "<1s";
      return `${age.toFixed(0)}s`;
    }
    function formatUsageMode(value) {
      if (value === null || value === undefined || value === "") return null;
      const map = {
        0: "常规",
        1: "导航",
        2: "辅助"
      };
      const key = Number(value);
      const label = Number.isFinite(key) && Object.prototype.hasOwnProperty.call(map, key) ? map[key] : String(value);
      return `使用模式 ${label}`;
    }
    function formatOoa(value) {
      if (value === null || value === undefined || value === "") return null;
      const map = {
        0: "未启动",
        1: "空闲中",
        2: "未触发避障",
        3: "主动避障中"
      };
      const key = Number(value);
      const label = Number.isFinite(key) && Object.prototype.hasOwnProperty.call(map, key) ? map[key] : String(value);
      return `辅助避障 ${label}`;
    }
    function setLog(id, payload) {
      $(id).textContent = typeof payload === "string" ? payload : JSON.stringify(payload, null, 2);
    }
    function setMarkStatus(payload) {
      if ($("markSaveStatus")) setLog("markSaveStatus", payload);
    }
    function sleepMs(ms) {
      return new Promise(resolve => setTimeout(resolve, ms));
    }
    function preflightStatusText(result) {
      if (!result) return "尚未自检";
      const ageText = result.age_sec === null || result.age_sec === undefined ? "" : ` / ${fmtAge(result.age_sec)}前`;
      if (result.running) return `${result.summary || "基础自检后台执行中，请稍候"}${ageText}`;
      if (result.summary) return `${result.summary}${ageText}`;
      if (result.ok && result.navigation_ready === false) return `最近一次基础自检通过；导航待重定位${ageText}`;
      if (result.ok) return `最近一次基础自检通过${ageText}`;
      return `最近一次基础自检未通过${ageText}`;
    }
    function renderPreflight(result) {
      state.preflight = result || null;
      const summaries = [$("preflightSummary"), $("taskPreflightSummary")];
      for (const box of summaries) {
        if (!box) continue;
        box.className = "preflight-summary";
        if (result) {
          const cls = result.running ? "warn" : (result.ok ? "ok" : "fail");
          box.classList.add(cls);
        }
        box.textContent = preflightStatusText(result);
      }
      const itemsBox = $("preflightItems");
      if (itemsBox) {
        itemsBox.innerHTML = "";
        const items = result && result.items ? result.items : [];
        if (!items.length) {
          itemsBox.innerHTML = `<div class="small">尚未自检。</div>`;
        } else {
          for (const item of items) {
            const row = document.createElement("div");
            row.className = "check-row";
            const statusClass = item.status === "ok" ? "ok" : (item.status === "warn" ? "warn" : (item.status === "info" ? "ok" : "fail"));
            const statusText = item.status === "ok" ? "通过" : (item.status === "warn" ? "提醒" : (item.status === "info" ? "信息" : "失败"));
            row.innerHTML = `
              <div class="check-status ${statusClass}">${statusText}</div>
              <div><strong>${item.label || item.key}</strong><div class="small">${item.message || ""}</div></div>
            `;
            itemsBox.appendChild(row);
          }
        }
      }
      if ($("preflightRaw")) $("preflightRaw").textContent = result ? JSON.stringify(result, null, 2) : "等待自检";
    }
    async function loadPreflight() {
      try {
        const payload = await fetchJson("/api/preflight");
        const result = payload.preflight || null;
        renderPreflight(result);
        return result;
      } catch (err) {
        renderPreflight(null);
        return null;
      }
    }
    async function pollPreflightUntilDone(maxMs = 90000) {
      const deadline = Date.now() + maxMs;
      let result = null;
      while (Date.now() < deadline) {
        await sleepMs(1500);
        result = await loadPreflight();
        if (result && !result.running) return result;
      }
      throw {ok: false, message: "后台自检仍在执行，请刷新自检结果或查看 m20pro-real.service 日志"};
    }
    async function runPreflight() {
      const buttons = [$("runPreflightBtn"), $("taskRunPreflightBtn")].filter(Boolean);
      for (const btn of buttons) btn.disabled = true;
      if ($("preflightSummary")) $("preflightSummary").textContent = "基础自检中（工位/未重定位时只确认基础链路）...";
      if ($("taskPreflightSummary")) $("taskPreflightSummary").textContent = "基础自检中（工位/未重定位时只确认基础链路）...";
      try {
        const payload = await apiWithTimeout("POST", "/api/preflight/run", {mode: "move", site: "auto", wait: false}, 10000);
        const result = payload.preflight || payload;
        renderPreflight(result);
        if (payload.running || (result && result.running)) await pollPreflightUntilDone();
        await loadTasks();
      } catch (err) {
        renderPreflight({
          ok: false,
          navigation_ready: false,
          summary: err.message || "自检请求失败",
          age_sec: 0,
          items: [{
            key: "preflight_request",
            label: "自检请求",
            status: "fail",
            message: err.message || JSON.stringify(err)
          }]
        });
        setLog("preflightRaw", err);
      } finally {
        for (const btn of buttons) btn.disabled = false;
      }
    }
    function currentAnnotationMapId() {
      return state.selectedMapId || "live_map";
    }
    function asNumber(id, fallback) {
      const value = Number($(id).value);
      return Number.isFinite(value) ? value : fallback;
    }
    function asInteger(id, fallback) {
      const value = Number.parseInt($(id).value, 10);
      return Number.isFinite(value) ? value : fallback;
    }
    function syncManualDefaults(force) {
      const manualType = $("manualPointType").value;
      const defaults = defaultByManualType[manualType] || defaultByManualType.task;
      if (force || !$("markDwell").value.trim()) $("markDwell").value = String(defaults.dwell);
      if (force || !$("markGait").value.trim()) $("markGait").value = String(defaults.gait);
      if (force || !$("markSpeed").value.trim()) $("markSpeed").value = String(defaults.speed);
      if (force || !$("markManner").value.trim()) $("markManner").value = String(defaults.manner);
      if (force || !$("markObsMode").value.trim()) $("markObsMode").value = String(defaults.obsMode);
      if (force || !$("markNavMode").value.trim()) $("markNavMode").value = String(defaults.navMode);
    }
    async function fetchJson(url) {
      const res = await fetch(url, { cache: "no-store" });
      const payload = await res.json();
      if (!res.ok || payload.ok === false) throw payload;
      return payload;
    }
    async function api(method, url, body) {
      const res = await fetch(url, {
        method,
        headers: {"Content-Type": "application/json"},
        body: body === undefined ? undefined : JSON.stringify(body)
      });
      const payload = await res.json();
      if (!res.ok || payload.ok === false) throw payload;
      return payload;
    }
    async function apiWithTimeout(method, url, body, timeoutMs) {
      const controller = new AbortController();
      const timer = setTimeout(() => controller.abort(), timeoutMs);
      try {
        const res = await fetch(url, {
          method,
          headers: {"Content-Type": "application/json"},
          body: body === undefined ? undefined : JSON.stringify(body),
          signal: controller.signal
        });
        const payload = await res.json();
        if (!res.ok || payload.ok === false) throw payload;
        return payload;
      } catch (err) {
        if (err && err.name === "AbortError") {
          throw {ok: false, message: `请求超时：${Math.round(timeoutMs / 1000)} 秒内未收到网页返回；请刷新自检结果或检查 m20pro-real.service`};
        }
        throw err;
      } finally {
        clearTimeout(timer);
      }
    }
    function mapPreferredByFloor(floor) {
      if (!state.maps.length) return "";
      const normalized = String(floor || "").trim();
      if (normalized) {
        const byId = state.maps.find(item => item.id === `builtin_${normalized}`);
        if (byId) return byId.id;
        const byFloor = state.maps.find(item => item.floor === normalized);
        if (byFloor) return byFloor.id;
      }
      const f20 = state.maps.find(item => item.id === "builtin_F20") || state.maps.find(item => item.floor === "F20");
      if (f20) return f20.id;
      const builtin = state.maps.find(item => item.source === "project_builtin");
      return builtin ? builtin.id : state.maps[0].id;
    }
    function resizeCanvas() {
      const before = getView();
      const rect = canvas.parentElement.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.floor(rect.width * dpr));
      canvas.height = Math.max(1, Math.floor(rect.height * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      if (state.map && before && before.rect) {
        state.view.panX += (before.rect.width - rect.width) * 0.5;
        state.view.panY += (before.rect.height - rect.height) * 0.5;
        clampView();
      } else if (state.map) {
        state.view.panX = 0;
        state.view.panY = 0;
      }
      updateZoomReadout();
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
    function getBaseView(rect = canvas.getBoundingClientRect()) {
      const map = state.map;
      if (!map) return { scale: 1, ox: 0, oy: 0, rect };
      const scale = Math.min(rect.width / map.width, rect.height / map.height);
      const drawW = map.width * scale;
      const drawH = map.height * scale;
      return { scale, ox: (rect.width - drawW) / 2, oy: (rect.height - drawH) / 2, rect };
    }
    function getView() {
      const base = getBaseView();
      const zoom = clampZoom(state.view.zoom);
      const scale = base.scale * zoom;
      const map = state.map;
      if (!map) return {...base, zoom: 1, baseScale: base.scale};
      const drawW = map.width * scale;
      const drawH = map.height * scale;
      return {
        scale,
        baseScale: base.scale,
        zoom,
        ox: (base.rect.width - drawW) / 2 + state.view.panX,
        oy: (base.rect.height - drawH) / 2 + state.view.panY,
        rect: base.rect
      };
    }
    function clampZoom(value) {
      const zoom = Number(value);
      if (!Number.isFinite(zoom)) return 1;
      return Math.max(0.25, Math.min(12, zoom));
    }
    function updateZoomReadout(view = getView()) {
      if (!$("zoomReadout")) return;
      if (state.mapViewMode === "3d") {
        const base = state.terrainView.baseZoom || state.terrainView.zoom || 1;
        const ratio = base > 0 ? state.terrainView.zoom / base : 1;
        $("zoomReadout").textContent = `${Math.round(ratio * 100)}%`;
        return;
      }
      $("zoomReadout").textContent = `${Math.round((view.zoom || state.view.zoom || 1) * 100)}%`;
    }
    function clampView() {
      if (!state.map) return;
      state.view.zoom = clampZoom(state.view.zoom);
      const view = getView();
      const drawW = state.map.width * view.scale;
      const drawH = state.map.height * view.scale;
      const margin = 80;
      if (drawW <= view.rect.width) {
        state.view.panX = 0;
      } else {
        const limitX = (drawW - view.rect.width) * 0.5 + margin;
        state.view.panX = Math.max(-limitX, Math.min(limitX, state.view.panX));
      }
      if (drawH <= view.rect.height) {
        state.view.panY = 0;
      } else {
        const limitY = (drawH - view.rect.height) * 0.5 + margin;
        state.view.panY = Math.max(-limitY, Math.min(limitY, state.view.panY));
      }
      updateZoomReadout();
    }
    function resetMapView(redraw = true) {
      state.view.zoom = 1;
      state.view.panX = 0;
      state.view.panY = 0;
      updateZoomReadout();
      if (redraw) draw();
    }
    function setZoomAt(clientX, clientY, nextZoom) {
      if (!state.map) return;
      const oldView = getView();
      const rect = oldView.rect;
      const cx = clientX - rect.left;
      const cy = clientY - rect.top;
      const mx = (cx - oldView.ox) / oldView.scale;
      const my = (cy - oldView.oy) / oldView.scale;
      state.view.zoom = clampZoom(nextZoom);
      const newView = getView();
      state.view.panX += cx - (newView.ox + mx * newView.scale);
      state.view.panY += cy - (newView.oy + my * newView.scale);
      clampView();
      draw();
    }
    function zoomBy(factor) {
      if (state.mapViewMode === "3d") {
        terrainZoomBy(factor);
        return;
      }
      const rect = canvas.getBoundingClientRect();
      setZoomAt(rect.left + rect.width * 0.5, rect.top + rect.height * 0.5, state.view.zoom * factor);
    }
    function terrainZoomBy(factor) {
      state.terrainView.zoom *= factor;
      const base = state.terrainView.baseZoom || state.terrainView.zoom || 1;
      state.terrainView.zoom = Math.max(base * 0.35, Math.min(base * 8, state.terrainView.zoom));
      updateZoomReadout();
      draw();
    }
    function resetTerrainView(redraw = true) {
      state.terrainView.panX = 0;
      state.terrainView.panY = 0;
      state.terrainView.baseZoom = null;
      updateZoomReadout();
      if (redraw) draw();
    }
    function updateMapModeUi() {
      $("map2dBtn").classList.toggle("active-tool", state.mapViewMode === "2d");
      $("map3dBtn").classList.toggle("active-tool", state.mapViewMode === "3d");
      if (state.mapViewMode === "3d") {
        $("mapMode").textContent = terrainPayload() ? "3D 派生地图" : "3D 暂无地形";
        $("cursor").textContent = "3D地图：拖动平移，滚轮缩放；切回2D后可标点/重定位";
      } else {
        $("mapMode").textContent = state.mapModeLabel || "实时 /map";
        $("cursor").textContent = state.view.panMode ? "平移模式" : "拖拽地图取点和朝向";
      }
      updateZoomReadout();
    }
    async function setMapViewMode(mode) {
      state.mapViewMode = mode === "3d" ? "3d" : "2d";
      if (state.mapViewMode === "3d") await loadTerrain();
      updateMapModeUi();
      draw();
    }
    function centerMapOnWorld(x, y) {
      if (!state.map || !Number.isFinite(Number(x)) || !Number.isFinite(Number(y))) return;
      const view = getView();
      const target = worldToCanvasWithView(Number(x), Number(y), view);
      if (!target) return;
      state.view.panX += view.rect.width * 0.5 - target.x;
      state.view.panY += view.rect.height * 0.5 - target.y;
      clampView();
      draw();
    }
    function centerTerrainOnWorld(x, y) {
      const terrain = terrainPayload();
      if (!terrain || !Number.isFinite(Number(x)) || !Number.isFinite(Number(y))) return false;
      const rect = canvas.getBoundingClientRect();
      const h = Number((terrain.bounds || {}).max_z || 0) + 0.3;
      const p = projectTerrainPoint(Number(x), Number(y), h, terrain, state.terrainView, rect);
      state.terrainView.panX += rect.width * 0.5 - p.x;
      state.terrainView.panY += rect.height * 0.5 - p.y;
      draw();
      return true;
    }
    function worldToCanvasWithView(x, y, view) {
      const map = state.map;
      if (!map) return null;
      const mx = (x - map.origin.x) / map.resolution;
      const my = map.height - (y - map.origin.y) / map.resolution;
      return { x: view.ox + mx * view.scale, y: view.oy + my * view.scale };
    }
    function worldToCanvas(x, y) {
      return worldToCanvasWithView(x, y, getView());
    }
    function canvasToWorld(clientX, clientY) {
      const map = state.map;
      if (!map) return null;
      const rect = canvas.getBoundingClientRect();
      const view = getView();
      const cx = clientX - rect.left;
      const cy = clientY - rect.top;
      const mx = (cx - view.ox) / view.scale;
      const my = (cy - view.oy) / view.scale;
      if (mx < 0 || my < 0 || mx > map.width || my > map.height) return null;
      return {
        x: map.origin.x + mx * map.resolution,
        y: map.origin.y + (map.height - my) * map.resolution
      };
    }
    function normalizeYaw(yaw) {
      let value = Number(yaw);
      if (!Number.isFinite(value)) return 0;
      while (value > Math.PI) value -= Math.PI * 2;
      while (value <= -Math.PI) value += Math.PI * 2;
      return value;
    }
    function currentMarkYaw() {
      return normalizeYaw($("markYaw").value);
    }
    function currentLocalizeYaw() {
      return normalizeYaw($("locYaw").value);
    }
    function setMarkDraft(pose, message) {
      state.markDraft = {
        x: Number(pose.x),
        y: Number(pose.y),
        yaw: normalizeYaw(pose.yaw)
      };
      $("markXY").value = `${state.markDraft.x.toFixed(3)}, ${state.markDraft.y.toFixed(3)}`;
      $("markYaw").value = state.markDraft.yaw.toFixed(4);
      $("cursor").textContent = message || `x ${state.markDraft.x.toFixed(3)} / y ${state.markDraft.y.toFixed(3)} / 朝向 ${state.markDraft.yaw.toFixed(3)} rad`;
      draw();
    }
    function setLocalizeDraft(pose, message) {
      state.localizeDraft = {
        x: Number(pose.x),
        y: Number(pose.y),
        yaw: normalizeYaw(pose.yaw)
      };
      $("locXY").value = `${state.localizeDraft.x.toFixed(3)}, ${state.localizeDraft.y.toFixed(3)}`;
      $("locYaw").value = state.localizeDraft.yaw.toFixed(4);
      $("cursor").textContent = message || `定位 x ${state.localizeDraft.x.toFixed(3)} / y ${state.localizeDraft.y.toFixed(3)} / 朝向 ${state.localizeDraft.yaw.toFixed(3)} rad`;
      draw();
    }
    function activeTabName() {
      const active = document.querySelector("button.tab.active");
      return active ? active.dataset.tab : "";
    }
    function drawArrow(pose, options = {}) {
      if (!Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) return;
      const p = worldToCanvas(pose.x, pose.y);
      if (!p) return;
      const color = options.color || "#0f6bff";
      const size = options.size || 1.0;
      const label = options.label || "";
      ctx.save();
      ctx.translate(p.x, p.y);
      ctx.rotate(-(Number(pose.yaw) || 0));
      ctx.fillStyle = color;
      ctx.strokeStyle = options.stroke || "#ffffff";
      ctx.lineWidth = options.lineWidth || 2;
      ctx.beginPath();
      ctx.moveTo(15 * size, 0);
      ctx.lineTo(-10 * size, -8 * size);
      ctx.lineTo(-6 * size, 0);
      ctx.lineTo(-10 * size, 8 * size);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      if (label) {
        ctx.save();
        ctx.font = "12px system-ui, sans-serif";
        ctx.fillStyle = "#17212b";
        ctx.fillText(label, p.x + 11, p.y - 9);
        ctx.restore();
      }
    }
    function drawPath(path) {
      if (!path || !path.points || path.points.length < 2) return;
      ctx.save();
      ctx.strokeStyle = "#f97316";
      ctx.lineWidth = 3;
      ctx.beginPath();
      let started = false;
      for (const point of path.points) {
        const p = worldToCanvas(point.x, point.y);
        if (!p) continue;
        if (!started) { ctx.moveTo(p.x, p.y); started = true; }
        else ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
      ctx.restore();
    }
    function drawObstacles(items) {
      if (!items || items.length === 0 || !state.map) return;
      const view = getView();
      ctx.save();
      for (const item of items) {
        const p = worldToCanvasWithView(item.x, item.y, view);
        if (!p) continue;
        const radius = Math.max(5, Math.min(22, (item.scale_x || 0.4) / state.map.resolution * view.scale * 0.5));
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
    function drawAnnotations() {
      if (!state.annotations || !state.map) return;
      for (const item of state.annotations) {
        const pose = item.pose || {};
        const color = typeColors[item.type] || "#0f6bff";
        const label = item.label || typeNames[item.type] || "point";
        drawArrow(
          {x: Number(pose.x), y: Number(pose.y), yaw: Number(pose.yaw) || 0},
          {color, size: 0.72, label}
        );
      }
    }
    function drawMarkDraft() {
      if (!state.markDraft) return;
      drawArrow(state.markDraft, {
        color: "#16a34a",
        stroke: "#f8fafc",
        lineWidth: 2.5,
        size: 0.92,
        label: "待保存"
      });
    }
    function drawLocalizeDraft() {
      if (!state.localizeDraft) return;
      drawArrow(state.localizeDraft, {
        color: "#dc2626",
        stroke: "#fef2f2",
        lineWidth: 2.5,
        size: 1.0,
        label: "定位"
      });
    }
    function drawScanOverlay() {
      if (!state.scanOverlay || !state.map || !state.latest || !state.latest.scan) return;
      const points = state.latest.scan.points || [];
      if (!points.length) return;
      let pose = state.latest.pose;
      const usingDraft = activeTabName() === "localize" && state.localizeDraft;
      if (usingDraft) pose = state.localizeDraft;
      if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) return;
      const yaw = normalizeYaw(pose.yaw || 0);
      const cosYaw = Math.cos(yaw);
      const sinYaw = Math.sin(yaw);
      const offset = state.latest.scan_overlay_offset || {};
      const offX = Number(offset.x || 0);
      const offY = Number(offset.y || 0);
      const offYaw = normalizeYaw(offset.yaw || 0);
      const cosOff = Math.cos(offYaw);
      const sinOff = Math.sin(offYaw);
      const view = getView();
      ctx.save();
      ctx.fillStyle = usingDraft ? "rgba(220, 38, 38, 0.72)" : "rgba(14, 165, 233, 0.78)";
      const radius = Math.max(1.4, Math.min(3.2, view.scale * 1.6));
      for (const point of points) {
        const px = Number(point.x);
        const py = Number(point.y);
        if (!Number.isFinite(px) || !Number.isFinite(py)) continue;
        const bx = offX + cosOff * px - sinOff * py;
        const by = offY + sinOff * px + cosOff * py;
        const wx = Number(pose.x) + cosYaw * bx - sinYaw * by;
        const wy = Number(pose.y) + sinYaw * bx + cosYaw * by;
        const p = worldToCanvasWithView(wx, wy, view);
        if (!p) continue;
        ctx.fillRect(p.x - radius * 0.5, p.y - radius * 0.5, radius, radius);
      }
      ctx.restore();
    }
    function draw() {
      const rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      if (state.mapViewMode === "3d") {
        drawTerrain();
        return;
      }
      ctx.fillStyle = "#cfd5dc";
      ctx.fillRect(0, 0, rect.width, rect.height);
      if (!state.map || !state.mapImage) {
        ctx.fillStyle = "#667483";
        ctx.font = "15px system-ui, sans-serif";
        ctx.fillText("等待地图数据", 20, 30);
        return;
      }
      const view = getView();
      updateZoomReadout(view);
      ctx.drawImage(state.mapImage, view.ox, view.oy, state.map.width * view.scale, state.map.height * view.scale);
      ctx.strokeStyle = "#4b5563";
      ctx.lineWidth = 1;
      ctx.strokeRect(view.ox, view.oy, state.map.width * view.scale, state.map.height * view.scale);
      const latest = state.latest;
      if (latest) {
        drawPath(latest.path);
        drawObstacles(latest.dynamic_obstacles);
      }
      drawScanOverlay();
      drawAnnotations();
      drawMarkDraft();
      drawLocalizeDraft();
      if (latest && latest.pose) {
        const robotPose = {
          x: latest.pose.x,
          y: latest.pose.y,
          yaw: Number.isFinite(Number(latest.pose.display_yaw)) ? latest.pose.display_yaw : latest.pose.yaw
        };
        drawArrow(robotPose);
      }
    }
    function terrainPayload() {
      return state.terrain && state.terrain.terrain ? state.terrain.terrain : null;
    }
    function terrainHeightAt(col, row, terrain) {
      const cols = Number(terrain.cols || 0);
      if (col < 0 || row < 0 || col >= cols) return null;
      const value = terrain.heights[row * cols + col];
      return value === null || value === undefined ? null : Number(value);
    }
    function projectTerrainPoint(x, y, z, terrain, view, rect) {
      const bounds = terrain.bounds || {};
      const cx = (Number(bounds.min_x || 0) + Number(bounds.max_x || 0)) * 0.5;
      const cy = (Number(bounds.min_y || 0) + Number(bounds.max_y || 0)) * 0.5;
      const cz = (Number(bounds.min_z || 0) + Number(bounds.max_z || 0)) * 0.5;
      const px = x - cx;
      const py = y - cy;
      const pz = (z - cz) * view.zScale;
      const cosZ = Math.cos(view.rotZ);
      const sinZ = Math.sin(view.rotZ);
      const rx = cosZ * px - sinZ * py;
      const ry = sinZ * px + cosZ * py;
      const cosX = Math.cos(view.rotX);
      const sinX = Math.sin(view.rotX);
      const sy = cosX * ry - sinX * pz;
      return {
        x: rect.width * 0.5 + view.panX + rx * view.zoom,
        y: rect.height * 0.55 + view.panY - sy * view.zoom
      };
    }
    function terrainColor(value, terrain) {
      const bounds = terrain.bounds || {};
      const minZ = Number(bounds.min_z || 0);
      const maxZ = Number(bounds.max_z || minZ + 1);
      const ratio = Math.max(0, Math.min(1, (Number(value) - minZ) / Math.max(0.001, maxZ - minZ)));
      if (ratio < 0.48) {
        const t = ratio / 0.48;
        return `rgb(${Math.round(37 + t * 6)}, ${Math.round(99 + t * 96)}, ${Math.round(235 - t * 114)})`;
      }
      const t = (ratio - 0.48) / 0.52;
      return `rgb(${Math.round(34 + t * 215)}, ${Math.round(197 - t * 82)}, ${Math.round(94 - t * 72)})`;
    }
    function drawTerrain() {
      const rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      ctx.fillStyle = "#eef2f6";
      ctx.fillRect(0, 0, rect.width, rect.height);
      const terrain = terrainPayload();
      if (!terrain || !terrain.heights) {
        ctx.fillStyle = "#667483";
        ctx.font = "15px system-ui, sans-serif";
        ctx.fillText(state.terrainMessage || "当前地图暂无 3D 地形，2D 地图仍可用。", 20, 32);
        updateZoomReadout();
        return;
      }
      const cols = Number(terrain.cols || 0);
      const rows = Number(terrain.rows || 0);
      const cell = Number(terrain.cell_size || 0.25);
      if (cols <= 0 || rows <= 0) return;
      const bounds = terrain.bounds || {};
      const span = Math.max(
        Number(bounds.max_x || 0) - Number(bounds.min_x || 0),
        Number(bounds.max_y || 0) - Number(bounds.min_y || 0),
        1
      );
      if (!state.terrainView.baseZoom) {
        state.terrainView.baseZoom = Math.min(rect.width, rect.height) * 0.78 / span;
        state.terrainView.zoom = state.terrainView.baseZoom;
      }
      const maxCells = 9000;
      const step = Math.max(1, Math.ceil(Math.sqrt((rows * cols) / maxCells)));
      const view = state.terrainView;
      ctx.save();
      for (let row = rows - step; row >= 0; row -= step) {
        for (let col = 0; col < cols - step; col += step) {
          const h00 = terrainHeightAt(col, row, terrain);
          const h10 = terrainHeightAt(Math.min(cols - 1, col + step), row, terrain);
          const h11 = terrainHeightAt(Math.min(cols - 1, col + step), Math.min(rows - 1, row + step), terrain);
          const h01 = terrainHeightAt(col, Math.min(rows - 1, row + step), terrain);
          if (h00 === null && h10 === null && h11 === null && h01 === null) continue;
          const h = [h00, h10, h11, h01].filter(v => v !== null).reduce((a, b) => a + b, 0) /
            [h00, h10, h11, h01].filter(v => v !== null).length;
          const x0 = Number(terrain.origin.x || 0) + col * cell;
          const y0 = Number(terrain.origin.y || 0) + row * cell;
          const x1 = x0 + step * cell;
          const y1 = y0 + step * cell;
          const p0 = projectTerrainPoint(x0, y0, h00 === null ? h : h00, terrain, view, rect);
          const p1 = projectTerrainPoint(x1, y0, h10 === null ? h : h10, terrain, view, rect);
          const p2 = projectTerrainPoint(x1, y1, h11 === null ? h : h11, terrain, view, rect);
          const p3 = projectTerrainPoint(x0, y1, h01 === null ? h : h01, terrain, view, rect);
          ctx.beginPath();
          ctx.moveTo(p0.x, p0.y);
          ctx.lineTo(p1.x, p1.y);
          ctx.lineTo(p2.x, p2.y);
          ctx.lineTo(p3.x, p3.y);
          ctx.closePath();
          ctx.fillStyle = terrainColor(h, terrain);
          ctx.fill();
        }
      }
      ctx.strokeStyle = "rgba(15, 23, 42, 0.08)";
      ctx.lineWidth = 1;
      for (let row = 0; row < rows; row += step * 4) {
        ctx.beginPath();
        let started = false;
        for (let col = 0; col < cols; col += step * 4) {
          const h = terrainHeightAt(col, row, terrain);
          if (h === null) continue;
          const p = projectTerrainPoint(Number(terrain.origin.x || 0) + col * cell, Number(terrain.origin.y || 0) + row * cell, h, terrain, view, rect);
          if (!started) { ctx.moveTo(p.x, p.y); started = true; }
          else ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
      }
      drawTerrainZones(terrain, view, rect);
      drawTerrainRobot(terrain, view, rect);
      drawTerrainLegend(rect);
      ctx.restore();
      updateZoomReadout();
    }
    function drawTerrainZones(terrain, view, rect) {
      const zones = state.stairZones || [];
      if (!zones.length) return;
      const baseZ = Number((terrain.bounds || {}).max_z || 0);
      for (const zone of zones) {
        const poly = zone.polygon || [];
        if (poly.length < 3) continue;
        ctx.beginPath();
        let started = false;
        for (const point of poly) {
          const p = projectTerrainPoint(Number(point.x), Number(point.y), baseZ + 0.2, terrain, view, rect);
          if (!started) { ctx.moveTo(p.x, p.y); started = true; }
          else ctx.lineTo(p.x, p.y);
        }
        ctx.closePath();
        ctx.fillStyle = zone.trigger_gait ? "rgba(249, 115, 22, 0.42)" : "rgba(124, 58, 237, 0.26)";
        ctx.strokeStyle = zone.trigger_gait ? "#c2410c" : "#7c3aed";
        ctx.lineWidth = 2;
        ctx.fill();
        ctx.stroke();
        if (zone.center) {
          const label = projectTerrainPoint(Number(zone.center.x), Number(zone.center.y), baseZ + 0.35, terrain, view, rect);
          ctx.fillStyle = "#17212b";
          ctx.font = "12px system-ui, sans-serif";
          ctx.fillText(zone.name || zone.id || "楼梯区", label.x + 4, label.y - 4);
        }
      }
    }
    function drawTerrainRobot(terrain, view, rect) {
      const pose = state.latest && state.latest.pose;
      if (!pose) return;
      const h = Number((terrain.bounds || {}).max_z || 0) + 0.3;
      const p = projectTerrainPoint(Number(pose.x), Number(pose.y), h, terrain, view, rect);
      ctx.fillStyle = "#dc2626";
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
    function drawTerrainLegend(rect) {
      const items = [
        ["低处", "#2563eb"],
        ["平面", "#22c55e"],
        ["高处", "#f97316"],
        ["楼梯区", "rgba(249,115,22,0.55)"],
        ["机器人", "#dc2626"]
      ];
      ctx.save();
      ctx.font = "12px system-ui, sans-serif";
      const width = Math.min(390, rect.width - 24);
      const height = 34;
      const x = 12;
      const y = rect.height - height - 12;
      ctx.fillStyle = "rgba(255, 255, 255, 0.9)";
      ctx.strokeStyle = "rgba(148, 163, 184, 0.75)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.rect(x, y, width, height);
      ctx.fill();
      ctx.stroke();
      let cursorX = x + 10;
      for (const [label, color] of items) {
        if (cursorX + 58 > x + width) break;
        ctx.fillStyle = color;
        ctx.fillRect(cursorX, y + 11, 12, 12);
        ctx.strokeStyle = "rgba(0, 0, 0, 0.18)";
        ctx.strokeRect(cursorX, y + 11, 12, 12);
        ctx.fillStyle = "#475569";
        ctx.fillText(label, cursorX + 17, y + 21);
        cursorX += label.length > 2 ? 62 : 50;
      }
      ctx.restore();
    }
    async function loadTerrain() {
      if (!state.selectedMapId) {
        state.terrain = null;
        state.stairZones = [];
        state.terrainMessage = "请选择固定地图；实时 /map 没有离线 PCD 派生 3D 地形。";
        updateMapModeUi();
        draw();
        return;
      }
      try {
        const [terrain, zones] = await Promise.all([
          fetchJson(`/api/map_3d?map_id=${encodeURIComponent(state.selectedMapId)}`),
          fetchJson(`/api/stair_zones?map_id=${encodeURIComponent(state.selectedMapId)}`)
        ]);
        state.terrain = terrain && terrain.available ? terrain : null;
        state.stairZones = zones && zones.zones ? zones.zones : [];
        state.terrainView.baseZoom = null;
        if (state.terrain) {
          const t = state.terrain.terrain || {};
          state.terrainMessage = `${(state.terrain.map || {}).name || state.selectedMapId} / ${t.cols || 0}x${t.rows || 0} 高度网格 / ${(state.stairZones || []).length} 个楼梯区域`;
        } else {
          state.terrainMessage = (terrain && terrain.message) || "当前地图暂无 3D 地形，2D 地图仍可用。";
        }
        updateMapModeUi();
        draw();
      } catch (err) {
        state.terrain = null;
        state.stairZones = [];
        state.terrainMessage = err.message || JSON.stringify(err);
        updateMapModeUi();
        draw();
      }
    }
    async function refreshLiveMap(version) {
      if (state.selectedMapId || version === state.liveMapVersion) return;
      const map = await fetchJson("/api/map");
      if (!map.available) return;
      const resetView = !state.map || state.map.width !== map.width || state.map.height !== map.height;
      state.map = map;
      state.mapImage = buildMapImage(map);
      state.selectedMapId = null;
      state.liveMapVersion = map.version;
      $("mapTitle").textContent = `实时地图版本 ${map.version}`;
      $("mapMeta").textContent = `${map.width} x ${map.height}, ${map.resolution.toFixed(3)} m/格`;
      state.mapModeLabel = "实时 /map";
      updateMapModeUi();
      await loadAnnotations();
      if (resetView) resetMapView(false);
      resizeCanvas();
    }
    async function loadFileMap(mapId) {
      if (!mapId) {
        state.selectedMapId = null;
        state.fileMapVersion = -1;
        state.map = null;
        state.mapImage = null;
        state.mapViewMode = "2d";
        $("mapTitle").textContent = "实时地图";
        $("mapMeta").textContent = "等待 /map 数据";
        state.mapModeLabel = "实时 /map";
        updateMapModeUi();
        return;
      }
      const map = await fetchJson(`/api/map_file?map_id=${encodeURIComponent(mapId)}`);
      if (!map.available) {
        const message = map.message || `地图 ${mapId} 不可用`;
        $("mapTitle").textContent = "固定地图加载失败";
        $("mapMeta").textContent = message;
        $("cursor").textContent = message;
        throw {ok: false, message};
      }
      state.map = map;
      state.mapImage = buildMapImage(map);
      state.selectedMapId = mapId;
      state.fileMapVersion = map.version;
      state.mapViewMode = "2d";
      const select = $("mapSelect");
      if (select && select.value !== mapId) select.value = mapId;
      $("mapTitle").textContent = map.name || `固定地图 ${mapId}`;
      $("mapMeta").textContent = `${map.floor || "-"} / ${map.width} x ${map.height}, ${map.resolution.toFixed(3)} m/格`;
      state.mapModeLabel = map.source === "project_builtin" ? "项目内置地图" : "固定地图";
      updateMapModeUi();
      await loadAnnotations();
      resetMapView(false);
      resizeCanvas();
      await loadTerrain();
    }
    function renderLocalizeLiveStatus(s) {
      const box = $("localizeLiveStatus");
      if (!box) return;
      const now = Number(s.node_time || Date.now() / 1000);
      const pose = s.pose || null;
      const scan = s.scan || {};
      const localCostmap = s.local_costmap || {};
      const globalCostmap = s.global_costmap || {};
      const poseAge = pose && pose.last_update ? Math.max(0, now - Number(pose.last_update)) : null;
      const scanAge = scan.last_update ? Math.max(0, now - Number(scan.last_update)) : null;
      const localAge = localCostmap.last_update ? Math.max(0, now - Number(localCostmap.last_update)) : null;
      const globalAge = globalCostmap.last_update ? Math.max(0, now - Number(globalCostmap.last_update)) : null;
      const poseText = pose
        ? `x ${fmtNumber(Number(pose.x))} / y ${fmtNumber(Number(pose.y))} / 朝向 ${fmtNumber(Number(pose.yaw_deg), 0)}deg / ${fmtAge(poseAge)}前`
        : "无有效地图位姿";
      box.textContent = [
        `定位: ${s.localization_ok === true ? "正常" : (s.localization_ok === false ? "异常/未定位" : "未知")}`,
        `地图位姿: ${poseText}`,
        `/scan: ${Number(scan.finite_ranges || 0)} 条有效距离 / ${fmtAge(scanAge)}前`,
        `代价地图: local ${localCostmap.width || "-"}x${localCostmap.height || "-"} / ${fmtAge(localAge)}前, global ${globalCostmap.width || "-"}x${globalCostmap.height || "-"} / ${fmtAge(globalAge)}前`,
        `原厂导航: ${s.navigation_status || "-"}`
      ].join("\n");
    }
    function updateState(s) {
      state.latest = s;
      $("floor").textContent = text(s.floor);
      $("stair").textContent = text(s.stair_status);
      const gaitParts = [];
      if (s.usage_mode_result) gaitParts.push(text(s.usage_mode_result));
      if (s.gait_result) gaitParts.push(text(s.gait_result));
      else if (s.gait_command) gaitParts.push(text(s.gait_command));
      const usageMode = s.navigation_status_parsed ? s.navigation_status_parsed.usage_mode : null;
      const ooa = s.navigation_status_parsed ? s.navigation_status_parsed.ooa : null;
      const usageModeText = formatUsageMode(usageMode);
      const ooaText = formatOoa(ooa);
      if (usageModeText) gaitParts.push(usageModeText);
      if (ooaText) gaitParts.push(ooaText);
      $("gait").textContent = gaitParts.length ? gaitParts.join(" / ") : "-";
      if (s.pose) {
        const yawDeg = Number.isFinite(Number(s.pose.display_yaw_deg)) ? s.pose.display_yaw_deg : s.pose.yaw_deg;
        const rawYaw = fmtNumber(s.pose.yaw_deg, 0);
        const shownYaw = fmtNumber(yawDeg, 0);
        const offsetDeg = Number(s.pose.display_yaw_offset_deg || 0);
        const offsetText = Math.abs(offsetDeg) > 0.01 ? ` / 显示偏置 ${fmtNumber(offsetDeg, 0)}°` : "";
        $("pose").textContent = `x ${fmtNumber(s.pose.x)} / y ${fmtNumber(s.pose.y)} / 朝向 ${shownYaw}° / 原始 ${rawYaw}°${offsetText}`;
      }
      else $("pose").textContent = "-";
      if (s.localization_ok === true) $("localization").textContent = "正常";
      else if (s.localization_ok === false) $("localization").textContent = "异常/未定位";
      else $("localization").textContent = "-";
      renderLocalizeLiveStatus(s);
      $("factoryNav").textContent = text(s.navigation_status);
      if ($("scanOverlayStatus")) {
        const scan = s.scan || {};
        const points = scan.points || [];
        if (points.length) {
          const age = scan.last_update ? Math.max(0, s.node_time - scan.last_update) : null;
          const mode = activeTabName() === "localize" && state.localizeDraft ? "红色=待重定位预览" : "蓝色=当前位姿";
          $("scanOverlayStatus").textContent = `激光轮廓 ${points.length} 点 / ${mode} / ${fmtAge(age)}前`;
        } else if (scan.finite_ranges) {
          $("scanOverlayStatus").textContent = `收到 /scan，但无可绘制轮廓点`;
        } else {
          $("scanOverlayStatus").textContent = "等待 /scan 数据";
        }
      }
      if (s.battery && s.battery.primary) {
        const pack = s.battery.primary;
        const tempText = Number.isFinite(Number(pack.temperature_c)) ? ` / ${fmtNumber(Number(pack.temperature_c), 1)}℃` : "";
        $("battery").textContent = `${text(pack.level)}% / ${fmtNumber(Number(pack.voltage_v), 1)}V / ${fmtNumber(Number(pack.current_a), 1)}A${tempText}`;
      } else {
        $("battery").textContent = "-";
      }
      $("nav").textContent = JSON.stringify({
        路径点数: s.path ? s.path.points.length : 0,
        动态障碍物: s.dynamic_obstacles ? s.dynamic_obstacles.length : 0,
        当前任务: s.active_task || null,
        电量: s.battery && s.battery.primary ? s.battery.primary : null,
        定位状态: s.localization_ok,
        原厂导航: s.navigation_status || null,
        更新时间: s.node_time_text
      }, null, 2);
      const det = s.detections && (s.detections.parsed || s.detections.raw);
      $("detections").textContent = det ? JSON.stringify(det, null, 2) : "等待数据";
      $("events").textContent = s.events && s.events.length ? JSON.stringify(s.events.slice(-5), null, 2) : "等待数据";
      if (
        s.relocalization_result
        && s.relocalization_result.last_update !== state.lastRelocalizationStamp
        && Date.now() > state.relocalizationApiLogUntil
      ) {
        state.lastRelocalizationStamp = s.relocalization_result.last_update;
        setLog("localizeLog", {
          重定位结果: s.relocalization_result.raw,
          更新时间: s.node_time_text
        });
      }
      $("activeTask").textContent = s.active_task ? JSON.stringify(s.active_task, null, 2) : "无任务";
      $("stopTaskBtn").disabled = false;
      $("stopTaskBtn").title = "无论页面是否显示任务执行中，都会发送停止和复位指令";
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
    async function loadMaps() {
      const payload = await fetchJson("/api/maps");
      state.maps = payload.maps || [];
      const selected = payload.selected_map_id || mapPreferredByFloor(
        (state.latest && state.latest.floor) || $("locFloor").value || "F20"
      );
      const select = $("mapSelect");
      select.innerHTML = "";
      const live = document.createElement("option");
      live.value = "";
      live.textContent = "实时 /map";
      select.appendChild(live);
      for (const map of state.maps) {
        const opt = document.createElement("option");
        opt.value = map.id;
        const sourceText = map.source === "project_builtin" ? "项目内置" : "106归档";
        opt.textContent = `${map.name || map.id} (${map.floor || "-"} / ${sourceText})`;
        select.appendChild(opt);
      }
      select.value = selected;
      if (selected && selected !== state.selectedMapId) {
        try {
          await api("POST", "/api/maps/select", {map_id: selected});
        } catch (err) {
          console.warn(err);
        }
        await loadFileMap(selected);
      } else if (!selected && state.selectedMapId) {
        await loadFileMap("");
      }
      renderMapList();
    }
    function renderMapList() {
      const box = $("mapList");
      box.innerHTML = "";
      if (!state.maps.length) {
        box.innerHTML = `<div class="small">当前没有可选固定地图，可先使用实时 /map 或从 106 拉取地图。</div>`;
        return;
      }
      for (const map of state.maps) {
        const el = document.createElement("div");
        el.className = "item";
        const sourceText = map.source === "project_builtin" ? "项目内置" : "106 归档";
        const note = map.source_note ? `<br>${map.source_note}` : "";
        el.innerHTML = `
          <div class="item-head"><span>${map.name || map.id}</span><span class="tag">${map.floor || "-"}</span></div>
          <div class="item-meta">${sourceText} / ${map.yaml_path || ""}<br>${map.created_at || ""}${note}</div>
        `;
        box.appendChild(el);
      }
    }
    async function loadAnnotations() {
      const mapId = currentAnnotationMapId();
      const payload = await fetchJson(`/api/annotations${mapId ? `?map_id=${encodeURIComponent(mapId)}` : ""}`);
      state.annotations = payload.annotations || [];
      renderAnnotations();
      renderTaskPoints();
      setMarkStatus(`当前地图已加载 ${state.annotations.length} 个点位`);
    }
    function renderAnnotations() {
      const box = $("annotationList");
      box.innerHTML = "";
      if (!state.annotations.length) {
        box.innerHTML = `<div class="small">当前地图还没有点位。</div>`;
        return;
      }
      for (const item of state.annotations) {
        const pose = item.pose || {};
        const place = [item.area, item.room, item.result_file_prefix ? `结果:${item.result_file_prefix}` : ""].filter(Boolean).join(" / ");
        const el = document.createElement("div");
        el.className = "item";
        el.innerHTML = `
          <div class="item-head">
            <span>${item.label || typeNames[item.type] || item.id}</span>
            <span class="tag">${typeNames[item.type] || item.type}</span>
          </div>
          <div class="item-meta">${item.floor || "-"} / ${manualPointTypeNames[item.manual_point_type] || item.manual_point_type || "-"} / x ${fmtNumber(Number(pose.x))}, y ${fmtNumber(Number(pose.y))}, 朝向 ${fmtNumber(Number(pose.yaw), 2)} / 停留 ${fmtNumber(Number(item.dwell_s || 0), 1)}s</div>
          ${place ? `<div class="item-meta">${place}</div>` : ""}
          <div class="actions"><button class="danger" data-delete-mark="${item.id}">删除</button></div>
        `;
        box.appendChild(el);
      }
      for (const btn of box.querySelectorAll("[data-delete-mark]")) {
        btn.addEventListener("click", async () => {
          await api("DELETE", `/api/annotations?id=${encodeURIComponent(btn.dataset.deleteMark)}`);
          await loadAnnotations();
          setMarkStatus("点位已删除，列表已刷新");
          draw();
        });
      }
    }
    function renderTaskPoints() {
      const box = $("taskPointList");
      if (!state.annotations.length) {
        box.textContent = "请先选择地图并标点";
        return;
      }
      box.innerHTML = "";
      for (const item of state.annotations) {
        const line = document.createElement("label");
        line.className = "checkline";
        const place = [item.area, item.room].filter(Boolean).join(" / ");
        line.innerHTML = `<input type="checkbox" value="${item.id}" checked><span>${item.floor || "-"} / ${item.label || item.id}${place ? ` / ${place}` : ""} / ${manualPointTypeNames[item.manual_point_type] || item.manual_point_type || typeNames[item.type] || item.type} / 朝向 ${fmtNumber(Number((item.pose || {}).yaw), 2)} / 停留 ${fmtNumber(Number(item.dwell_s || 0), 1)}s</span>`;
        box.appendChild(line);
      }
    }
    async function loadTasks() {
      const payload = await fetchJson("/api/tasks");
      state.tasks = payload.tasks || [];
      const box = $("taskList");
      box.innerHTML = "";
      if (!state.tasks.length) {
        box.innerHTML = `<div class="small">还没有任务。</div>`;
        return;
      }
      for (const task of state.tasks) {
        const activeTask = payload.active_task && payload.active_task.status === "running" ? payload.active_task : null;
        const active = !!activeTask;
        const isRunning = !!(activeTask && activeTask.task_id === task.id);
        const canStart = !active && !isRunning;
        const canDelete = !isRunning && !(payload.active_task && payload.active_task.task_id === task.id);
        const startLabel = isRunning ? "执行中" : (active ? "先停止当前任务" : "开始执行");
        const el = document.createElement("div");
        el.className = "item";
        el.innerHTML = `
          <div class="item-head"><span>${task.name || task.id}</span><span class="tag">${task.status || "ready"}</span></div>
          <div class="item-meta">${(task.annotation_ids || []).length} 个点 / ${task.created_at || ""}${task.updated_at ? ` / 更新 ${task.updated_at}` : ""}</div>
          <div class="actions">
            <button class="primary" data-start-task="${task.id}" ${canStart ? "" : "disabled"}>${startLabel}</button>
            <button data-rename-task="${task.id}">改名</button>
            <button class="danger" data-delete-task="${task.id}" ${canDelete ? "" : "disabled"}>删除</button>
          </div>
        `;
        box.appendChild(el);
      }
      for (const btn of box.querySelectorAll("[data-start-task]")) {
        btn.addEventListener("click", async () => {
          btn.disabled = true;
          btn.textContent = "启动中...";
          try {
            const payload = await api("POST", "/api/tasks/start", {task_id: btn.dataset.startTask});
            setLog("activeTask", payload.active_task || payload);
            await loadTasks();
          } catch (err) {
            setLog("activeTask", err);
          } finally {
            await loadTasks();
          }
        });
      }
      for (const btn of box.querySelectorAll("[data-rename-task]")) {
        btn.addEventListener("click", async () => {
          const task = state.tasks.find(item => item.id === btn.dataset.renameTask);
          if (!task) return;
          const name = window.prompt("请输入新的任务名称", task.name || "");
          if (name === null) return;
          const trimmed = name.trim();
          if (!trimmed) return;
          try {
            await api("POST", "/api/tasks/update", {task_id: task.id, name: trimmed});
            await loadTasks();
          } catch (err) { setLog("activeTask", err); }
        });
      }
      for (const btn of box.querySelectorAll("[data-delete-task]")) {
        btn.addEventListener("click", async () => {
          const task = state.tasks.find(item => item.id === btn.dataset.deleteTask);
          if (!task) return;
          if (!window.confirm(`确认删除任务“${task.name || task.id}”？点位不会被删除。`)) return;
          try {
            await api("DELETE", `/api/tasks?id=${encodeURIComponent(task.id)}`);
            await loadTasks();
          } catch (err) { setLog("activeTask", err); }
        });
      }
    }
    async function mainLoop() {
      const dot = $("statusDot");
      const label = $("statusText");
      try {
        const s = await fetchJson("/api/state");
        await refreshLiveMap(s.map_version);
        updateState(s);
        dot.className = "dot ok";
        label.textContent = "已连接";
        draw();
      } catch (err) {
        dot.className = "dot warn";
        label.textContent = "等待服务";
        console.warn(err);
      } finally {
        setTimeout(mainLoop, 1500);
      }
    }
    for (const btn of document.querySelectorAll("button.tab")) {
      btn.addEventListener("click", () => {
        document.querySelectorAll("button.tab").forEach(item => item.classList.remove("active"));
        document.querySelectorAll(".panel").forEach(item => item.classList.remove("active"));
        btn.classList.add("active");
        $(`tab-${btn.dataset.tab}`).classList.add("active");
        draw();
      });
    }
    canvas.addEventListener("pointerdown", (evt) => {
      if (state.mapViewMode === "3d") {
        evt.preventDefault();
        state.terrainView.pointer = {
          id: evt.pointerId,
          x: evt.clientX,
          y: evt.clientY,
          panX: state.terrainView.panX,
          panY: state.terrainView.panY,
        };
        canvas.classList.add("panning");
        canvas.setPointerCapture(evt.pointerId);
        return;
      }
      if (state.view.panMode || evt.button === 1 || evt.button === 2 || evt.shiftKey || evt.altKey) {
        evt.preventDefault();
        state.panPointer = {
          id: evt.pointerId,
          x: evt.clientX,
          y: evt.clientY,
          panX: state.view.panX,
          panY: state.view.panY
        };
        canvas.classList.add("panning");
        canvas.setPointerCapture(evt.pointerId);
        return;
      }
      const p = canvasToWorld(evt.clientX, evt.clientY);
      if (!p) return;
      evt.preventDefault();
      state.markPointer = {id: evt.pointerId, start: p, moved: false, mode: activeTabName() === "localize" ? "localize" : "mark"};
      canvas.setPointerCapture(evt.pointerId);
      if (state.markPointer.mode === "localize") {
        setLocalizeDraft(
          {x: p.x, y: p.y, yaw: currentLocalizeYaw()},
          `定位 x ${p.x.toFixed(3)} / y ${p.y.toFixed(3)} / 拖动设置朝向`
        );
      } else {
        setMarkDraft(
          {x: p.x, y: p.y, yaw: currentMarkYaw()},
          `x ${p.x.toFixed(3)} / y ${p.y.toFixed(3)} / 拖动设置朝向`
        );
      }
    });
    canvas.addEventListener("pointermove", (evt) => {
      if (state.mapViewMode === "3d") {
        const pointer = state.terrainView.pointer;
        if (pointer && pointer.id === evt.pointerId) {
          evt.preventDefault();
          state.terrainView.panX = pointer.panX + (evt.clientX - pointer.x);
          state.terrainView.panY = pointer.panY + (evt.clientY - pointer.y);
          draw();
        }
        $("cursor").textContent = state.terrainMessage || "3D地图：拖动平移，滚轮缩放";
        return;
      }
      if (state.panPointer && state.panPointer.id === evt.pointerId) {
        evt.preventDefault();
        state.view.panX = state.panPointer.panX + (evt.clientX - state.panPointer.x);
        state.view.panY = state.panPointer.panY + (evt.clientY - state.panPointer.y);
        clampView();
        draw();
        $("cursor").textContent = `地图缩放 ${Math.round(state.view.zoom * 100)}%`;
        return;
      }
      const p = canvasToWorld(evt.clientX, evt.clientY);
      if (!p) return;
      if (!state.markPointer || state.markPointer.id !== evt.pointerId) {
        $("cursor").textContent = `x ${p.x.toFixed(3)} / y ${p.y.toFixed(3)}`;
        return;
      }
      evt.preventDefault();
      const start = state.markPointer.start;
      const distance = Math.hypot(p.x - start.x, p.y - start.y);
      const yaw = distance > 0.03 ? Math.atan2(p.y - start.y, p.x - start.x) : (
        state.markPointer.mode === "localize" ? currentLocalizeYaw() : currentMarkYaw()
      );
      state.markPointer.moved = state.markPointer.moved || distance > 0.03;
      if (state.markPointer.mode === "localize") {
        setLocalizeDraft(
          {x: start.x, y: start.y, yaw},
          `定位 x ${start.x.toFixed(3)} / y ${start.y.toFixed(3)} / 朝向 ${normalizeYaw(yaw).toFixed(3)} rad`
        );
      } else {
        setMarkDraft(
          {x: start.x, y: start.y, yaw},
          `x ${start.x.toFixed(3)} / y ${start.y.toFixed(3)} / 朝向 ${normalizeYaw(yaw).toFixed(3)} rad`
        );
      }
    });
    function finishMarkPointer(evt) {
      if (state.terrainView.pointer && state.terrainView.pointer.id === evt.pointerId) {
        evt.preventDefault();
        if (canvas.hasPointerCapture(evt.pointerId)) canvas.releasePointerCapture(evt.pointerId);
        state.terrainView.pointer = null;
        canvas.classList.remove("panning");
        return;
      }
      if (state.panPointer && state.panPointer.id === evt.pointerId) {
        evt.preventDefault();
        if (canvas.hasPointerCapture(evt.pointerId)) canvas.releasePointerCapture(evt.pointerId);
        state.panPointer = null;
        canvas.classList.remove("panning");
        return;
      }
      if (!state.markPointer || state.markPointer.id !== evt.pointerId) return;
      evt.preventDefault();
      if (canvas.hasPointerCapture(evt.pointerId)) canvas.releasePointerCapture(evt.pointerId);
      const mode = state.markPointer.mode;
      const pose = mode === "localize" ? state.localizeDraft : state.markDraft;
      state.markPointer = null;
      if (pose) {
        $("cursor").textContent = `${mode === "localize" ? "待重定位" : "待保存"} x ${pose.x.toFixed(3)} / y ${pose.y.toFixed(3)} / 朝向 ${pose.yaw.toFixed(3)} rad`;
      }
    }
    canvas.addEventListener("pointerup", finishMarkPointer);
    canvas.addEventListener("pointercancel", finishMarkPointer);
    canvas.addEventListener("contextmenu", (evt) => evt.preventDefault());
    canvas.addEventListener("wheel", (evt) => {
      evt.preventDefault();
      if (state.mapViewMode === "3d") {
        terrainZoomBy(Math.exp(-evt.deltaY * 0.001));
        return;
      }
      if (!state.map) return;
      const factor = Math.exp(-evt.deltaY * 0.0012);
      setZoomAt(evt.clientX, evt.clientY, state.view.zoom * factor);
    }, {passive: false});
    $("zoomOutBtn").addEventListener("click", () => zoomBy(1 / 1.25));
    $("zoomInBtn").addEventListener("click", () => zoomBy(1.25));
    $("panModeBtn").addEventListener("click", () => {
      state.view.panMode = !state.view.panMode;
      $("panModeBtn").classList.toggle("active-tool", state.view.panMode);
      $("cursor").textContent = state.view.panMode ? "平移模式" : "拖拽地图取点和朝向";
    });
    $("fitMapBtn").addEventListener("click", () => {
      if (state.mapViewMode === "3d") resetTerrainView(true);
      else resetMapView(true);
    });
    $("centerRobotBtn").addEventListener("click", () => {
      const pose = state.latest && state.latest.pose;
      if (!pose) {
        $("cursor").textContent = "暂无机器人位姿，重定位成功后才能居中";
        return;
      }
      if (state.mapViewMode === "3d") {
        if (!centerTerrainOnWorld(pose.x, pose.y)) $("cursor").textContent = "当前地图暂无 3D 地形，不能在 3D 中居中机器人";
      } else {
        centerMapOnWorld(pose.x, pose.y);
      }
    });
    $("map2dBtn").addEventListener("click", () => setMapViewMode("2d"));
    $("map3dBtn").addEventListener("click", () => setMapViewMode("3d"));
    $("markYaw").addEventListener("input", () => {
      const pose = state.markDraft;
      if (!pose) return;
      state.markDraft = {x: pose.x, y: pose.y, yaw: currentMarkYaw()};
      draw();
    });
    $("markXY").addEventListener("input", () => {
      const [xText, yText] = $("markXY").value.split(",");
      const x = Number(xText);
      const y = Number(yText);
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;
      state.markDraft = {x, y, yaw: currentMarkYaw()};
      draw();
    });
    $("locYaw").addEventListener("input", () => {
      const pose = state.localizeDraft;
      if (!pose) return;
      state.localizeDraft = {x: pose.x, y: pose.y, yaw: currentLocalizeYaw()};
      draw();
    });
    $("locXY").addEventListener("input", () => {
      const [xText, yText] = $("locXY").value.split(",");
      const x = Number(xText);
      const y = Number(yText);
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;
      state.localizeDraft = {x, y, yaw: currentLocalizeYaw()};
      draw();
    });
    $("sendInitialPoseBtn").addEventListener("click", async () => {
      const btn = $("sendInitialPoseBtn");
      const oldText = btn.textContent;
      try {
        btn.disabled = true;
        btn.textContent = "重定位中";
        if (!state.map) throw {message: "还没有固定地图，请先在地图页选择 F20 或等待默认地图加载"};
        const [xText, yText] = $("locXY").value.split(",");
        const x = Number(xText);
        const y = Number(yText);
        const yaw = Number($("locYaw").value);
        if (!Number.isFinite(x) || !Number.isFinite(y)) throw {message: "定位坐标无效，请先在地图上拖箭头"};
        state.relocalizationApiLogUntil = Date.now() + 45000;
        setLog("localizeLog", "正在向 104 本机和 106 原厂 ROS 发布 /initialpose，最多等待 45 秒；页面会给出 106 SSH、定位、地图位姿和 costmap 检查结果。");
        const payload = await apiWithTimeout("POST", "/api/localization/initialpose", {
          x,
          y,
          z: 0,
          yaw: Number.isFinite(yaw) ? yaw : 0,
          floor: $("locFloor").value.trim()
        }, 45000);
        state.relocalizationApiLogUntil = Date.now() + 12000;
        setLog("localizeLog", payload);
        const accepted = !!(payload.verification && payload.verification.factory_pose_accepted);
        $("cursor").textContent = accepted
          ? `重定位已生效 x ${x.toFixed(3)} / y ${y.toFixed(3)} / 朝向 ${normalizeYaw(yaw).toFixed(3)} rad`
          : "重定位已发送但未确认生效，请看定位日志";
      } catch (err) {
        setLog("localizeLog", err);
        $("cursor").textContent = err.message || "重定位请求失败";
      } finally {
        btn.disabled = false;
        btn.textContent = oldText;
      }
    });
    $("useRobotPoseForLocBtn").addEventListener("click", () => {
      const pose = state.latest && state.latest.pose;
      if (!pose) return;
      setLocalizeDraft({x: pose.x, y: pose.y, yaw: pose.yaw}, "已取当前机器人位姿");
      if (state.latest.floor) $("locFloor").value = state.latest.floor;
    });
    $("scanOverlayToggle").addEventListener("change", () => {
      state.scanOverlay = $("scanOverlayToggle").checked;
      draw();
    });
    $("checkMappingEnvBtn").addEventListener("click", async () => {
      try { setLog("mappingLog", await api("POST", "/api/mapping/check_environment", {})); }
      catch (err) { setLog("mappingLog", err); }
    });
    $("createSessionBtn").addEventListener("click", async () => {
      try {
        const payload = await api("POST", "/api/mapping/session", {
          project_name: $("projectName").value,
          building: $("buildingName").value,
          mode: $("mappingMode").value,
          floors: $("mappingFloors").value.split(",").map(v => v.trim()).filter(Boolean),
          active_floor: $("mappingActiveFloor").value.trim(),
          map_name: $("mappingMapName").value.trim()
        });
        state.sessionId = payload.session.id;
        if (!$("importName").value.trim()) $("importName").value = payload.session.map_name || "";
        setLog("mappingLog", payload);
      } catch (err) { setLog("mappingLog", err); }
    });
    $("startMappingBtn").addEventListener("click", async () => {
      try { setLog("mappingLog", await api("POST", "/api/mapping/start", {session_id: state.sessionId})); }
      catch (err) { setLog("mappingLog", err); }
    });
    $("finishMappingBtn").addEventListener("click", async () => {
      try { setLog("mappingLog", await api("POST", "/api/mapping/finish", {session_id: state.sessionId})); }
      catch (err) { setLog("mappingLog", err); }
    });
    $("importMapBtn").addEventListener("click", async () => {
      try {
        const payload = await api("POST", "/api/mapping/import_active_map", {
          session_id: state.sessionId,
          floor: $("importFloor").value.trim(),
          map_name: $("importName").value.trim()
        });
        setLog("mappingLog", payload);
        await loadMaps();
      } catch (err) { setLog("mappingLog", err); }
    });
    async function applySelectedMap() {
      const mapId = $("mapSelect").value;
      await api("POST", "/api/maps/select", {map_id: mapId});
      if (mapId) await loadFileMap(mapId);
      else {
        await loadFileMap("");
        state.liveMapVersion = -1;
        await loadTerrain();
      }
      await loadAnnotations();
      draw();
    }
    $("selectMapBtn").addEventListener("click", async () => {
      try {
        await applySelectedMap();
      } catch (err) {
        console.warn(err);
        $("cursor").textContent = err.message || JSON.stringify(err);
      }
    });
    $("mapSelect").addEventListener("change", async () => {
      try {
        await applySelectedMap();
      } catch (err) {
        console.warn(err);
        $("cursor").textContent = err.message || JSON.stringify(err);
      }
    });
    $("reloadMapsBtn").addEventListener("click", loadMaps);
    $("markType").addEventListener("change", () => {
      const manualType = manualTypeByUiType[$("markType").value] || "task";
      $("manualPointType").value = manualType;
      syncManualDefaults(true);
    });
    $("manualPointType").addEventListener("change", () => syncManualDefaults(true));
    $("saveMarkBtn").addEventListener("click", async () => {
      try {
        setMarkStatus("正在保存点位...");
        if (!state.map) throw {message: "还没有地图，等地图加载后再保存点位"};
        const [xText, yText] = $("markXY").value.split(",");
        const x = Number(xText);
        const y = Number(yText);
        if (!Number.isFinite(x) || !Number.isFinite(y)) throw {message: "点位坐标无效，请先点击地图取点"};
        const yaw = Number($("markYaw").value);
        const payload = await api("POST", "/api/annotations", {
          map_id: currentAnnotationMapId(),
          type: $("markType").value,
          floor: $("markFloor").value.trim(),
          label: $("markLabel").value.trim(),
          area: $("markArea").value.trim(),
          room: $("markRoom").value.trim(),
          result_file_prefix: $("markResultPrefix").value.trim(),
          pose: {
            x,
            y,
            z: 0,
            yaw: Number.isFinite(yaw) ? yaw : 0
          },
          manual_point_type: $("manualPointType").value,
          dwell_s: asNumber("markDwell", 0),
          vendor_navigation: {
            Gait: asInteger("markGait", 12),
            Speed: asInteger("markSpeed", 1),
            Manner: asInteger("markManner", 0),
            ObsMode: asInteger("markObsMode", 0),
            NavMode: asInteger("markNavMode", 1)
          }
        });
        await loadAnnotations();
        state.markDraft = null;
        draw();
        $("markLabel").value = "";
        $("markResultPrefix").value = "";
        const saved = payload.annotation || {};
        const savedPose = saved.pose || {};
        const savedText = `已保存: ${saved.label || saved.id} / ${saved.floor || "-"} / x ${fmtNumber(Number(savedPose.x), 3)}, y ${fmtNumber(Number(savedPose.y), 3)}, yaw ${fmtNumber(Number(savedPose.yaw), 3)} / 当前列表 ${state.annotations.length} 个点位`;
        setMarkStatus(savedText);
        $("cursor").textContent = savedText;
      } catch (err) {
        setMarkStatus(err.message || JSON.stringify(err));
        $("cursor").textContent = err.message || JSON.stringify(err);
      }
    });
    $("useRobotPoseBtn").addEventListener("click", () => {
      const pose = state.latest && state.latest.pose;
      if (!pose) return;
      $("markXY").value = `${pose.x.toFixed(3)}, ${pose.y.toFixed(3)}`;
      $("markYaw").value = String(pose.yaw.toFixed(4));
      state.markDraft = {x: pose.x, y: pose.y, yaw: normalizeYaw(pose.yaw)};
      draw();
      if (state.latest.floor) $("markFloor").value = state.latest.floor;
    });
    $("createTaskBtn").addEventListener("click", async () => {
      try {
        const ids = Array.from($("taskPointList").querySelectorAll("input:checked")).map(item => item.value);
        await api("POST", "/api/tasks", {
          name: $("taskName").value.trim(),
          map_id: currentAnnotationMapId(),
          annotation_ids: ids
        });
        await loadTasks();
      } catch (err) { setLog("activeTask", err); }
    });
    $("reloadTasksBtn").addEventListener("click", loadTasks);
    $("runPreflightBtn").addEventListener("click", runPreflight);
    $("refreshPreflightBtn").addEventListener("click", loadPreflight);
    $("taskRunPreflightBtn").addEventListener("click", async () => {
      document.querySelectorAll("button.tab").forEach(item => item.classList.remove("active"));
      document.querySelectorAll(".panel").forEach(item => item.classList.remove("active"));
      document.querySelector('button.tab[data-tab="preflight"]').classList.add("active");
      $("tab-preflight").classList.add("active");
      await runPreflight();
    });
    $("stopTaskBtn").addEventListener("click", async () => {
      const btn = $("stopTaskBtn");
      const oldText = btn.textContent;
      btn.disabled = true;
      btn.textContent = "停止中...";
      try {
        const payload = await api("POST", "/api/tasks/stop", {reason: "web_manual_stop"});
        setLog("activeTask", payload.message || payload.active_task || "已发送停止指令");
        await loadTasks();
      } catch (err) {
        setLog("activeTask", err);
      } finally {
        btn.disabled = false;
        btn.textContent = oldText;
      }
    });
    $("resetTaskSessionBtn").addEventListener("click", async () => {
      const btn = $("resetTaskSessionBtn");
      btn.disabled = true;
      try {
        const payload = await api("POST", "/api/tasks/stop", {reason: "web_manual_reset"});
        setLog("activeTask", payload.active_task || "已复位导航状态");
        await loadTasks();
      } catch (err) {
        setLog("activeTask", err);
      } finally {
        btn.disabled = false;
      }
    });
    window.addEventListener("resize", resizeCanvas);
    resizeCanvas();
    updateMapModeUi();
    syncManualDefaults(false);
    loadMaps().then(loadAnnotations).then(loadTerrain).then(loadPreflight).then(loadTasks).catch(console.warn);
    mainLoop();
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


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def _is_finite_pose_dict(pose: Dict[str, float]) -> bool:
    return all(
        math.isfinite(float(pose.get(key, 0.0)))
        for key in ("x", "y", "z", "yaw", "yaw_deg")
    )


def _is_plausible_pose_dict(pose: Dict[str, float], max_abs_position: float = 10000.0) -> bool:
    if not _is_finite_pose_dict(pose):
        return False
    return all(abs(float(pose.get(key, 0.0))) <= max_abs_position for key in ("x", "y", "z"))


def _parse_json_text(text: str) -> Any:
    try:
        return json.loads(text)
    except Exception:
        return None


def _now_text() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def fmt_age_text(age: Optional[float]) -> str:
    if age is None:
        return "无时间"
    if age < 1.0:
        return "<1s"
    return f"{age:.0f}s前"


def _new_id(prefix: str) -> str:
    return f"{prefix}_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}"


def _sanitize_name(value: str, fallback: str) -> str:
    text = str(value or "").strip() or fallback
    text = re.sub(r"[^\w.\-\u4e00-\u9fff]+", "_", text)
    return text.strip("._") or fallback


def _yaw_to_orientation(msg: PoseStamped, yaw: float) -> None:
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = math.sin(yaw * 0.5)
    msg.pose.orientation.w = math.cos(yaw * 0.5)


class _ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


class _CameraProxyWorker:
    _opencv_env_lock = threading.Lock()

    def __init__(self, node: "WebDashboardNode", camera_name: str, url: str) -> None:
        self.node = node
        self.camera_name = camera_name
        self.url = url
        self._condition = threading.Condition()
        self._thread: Optional[threading.Thread] = None
        self._stopped = False
        self._latest_jpeg: Optional[bytes] = None
        self._latest_stamp = 0.0
        self._sequence = 0
        self._last_error: Optional[str] = None
        self._last_error_log_time = 0.0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._run,
            name=f"m20pro_camera_proxy_{self.camera_name}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        with self._condition:
            self._stopped = True
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def wait_for_frame(
        self,
        last_sequence: int,
        timeout_s: float,
    ) -> Tuple[int, Optional[bytes], float, Optional[str]]:
        deadline = time.monotonic() + max(0.1, timeout_s)
        with self._condition:
            while not self._stopped and self._sequence <= last_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(timeout=remaining)
            return self._sequence, self._latest_jpeg, self._latest_stamp, self._last_error

    def _run(self) -> None:
        cap = None
        reconnect_s = max(0.2, float(self.node.get_parameter("camera_proxy_reconnect_s").value))
        while not self._is_stopped():
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
        if cap is not None:
            cap.release()

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
            self._condition.notify_all()
        if error:
            now = time.monotonic()
            if now - self._last_error_log_time >= 5.0:
                self._last_error_log_time = now
                self.node.get_logger().warning(f"{self.camera_name} camera proxy: {error}")

    def _is_stopped(self) -> bool:
        with self._condition:
            return self._stopped


class WebDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("m20pro_web_dashboard")
        self._declare_parameters()

        self._lock = threading.Lock()
        self._data_lock = threading.Lock()
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
        self._default_builtin_floor: Optional[str] = None
        self._default_builtin_map_id: Optional[str] = None
        self._builtin_maps = self._load_builtin_maps()
        self._annotations = self._load_json("annotations.json", [])
        self._tasks = self._load_json("tasks.json", [])
        self._sessions = self._load_json("mapping_sessions.json", [])
        self._settings = self._load_json("settings.json", {"selected_map_id": None, "active_task": None})
        self._normalize_runtime_state_on_startup()
        self._mapping_processes: Dict[str, Dict[str, Any]] = {}
        self._camera_workers: Dict[str, _CameraProxyWorker] = {}
        self._last_preflight: Optional[Dict[str, Any]] = None
        self._preflight_lock = threading.Lock()
        self._preflight_run_lock = threading.Lock()
        self._preflight_running: Optional[Dict[str, Any]] = None

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
        self.lidar_points_relay_pub = None
        if bool(self.get_parameter("enable_lidar_points_relay").value):
            relay_topic = str(self.get_parameter("lidar_points_relay_topic").value)
            relay_qos = QoSProfile(depth=2)
            relay_qos.reliability = ReliabilityPolicy.RELIABLE
            self.lidar_points_relay_pub = self.create_publisher(PointCloud2, relay_topic, relay_qos)
            self.get_logger().info(
                "LIDAR pointcloud relay enabled: %s -> %s"
                % (str(self.get_parameter("lidar_points_topic").value), relay_topic)
            )
        self.clear_costmap_clients = []
        if ClearEntireCostmap is not None:
            self.clear_costmap_clients = [
                self.create_client(ClearEntireCostmap, str(service_name))
                for service_name in self.get_parameter("task_clear_costmap_services").value
            ]

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
            "map": None,
            "map_version": 0,
            "dynamic_obstacles": [],
            "detections": None,
            "relocalization_result": None,
            "events": [],
            "topics": {},
        }

        self._create_subscriptions()
        self.create_timer(1.0, self._tick_active_task)
        self.create_timer(2.0, self._publish_selected_stair_zones)
        self._server = self._start_http_server()

    def _declare_parameters(self) -> None:
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("data_dir", "~/.m20pro_web")
        self.declare_parameter("map_archive_dir", "~/m20pro_maps")
        self.declare_parameter("map_manifest", "")
        self.declare_parameter("factory_host", "10.21.31.106")
        self.declare_parameter("factory_user", "user")
        self.declare_parameter("factory_active_map", "/var/opt/robot/data/maps/active")
        self.declare_parameter(
            "factory_mapping_start_command",
            'ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new '
            '-i /home/user/.ssh/id_ed25519 -o IdentitiesOnly=yes '
            '-o UserKnownHostsFile=/home/user/.ssh/known_hosts {factory_user}@{factory_host} '
            '"nohup sudo -n /usr/local/bin/drmap mapping -s -n {map_name} > /tmp/m20pro_drmap_mapping_{session_id}.log 2>&1 &"',
        )
        self.declare_parameter(
            "factory_mapping_finish_command",
            'ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new '
            '-i /home/user/.ssh/id_ed25519 -o IdentitiesOnly=yes '
            '-o UserKnownHostsFile=/home/user/.ssh/known_hosts {factory_user}@{factory_host} '
            '"sudo -n /usr/local/bin/drmap stop_mapping"',
        )
        self.declare_parameter(
            "factory_mapping_cancel_command",
            'ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new '
            '-i /home/user/.ssh/id_ed25519 -o IdentitiesOnly=yes '
            '-o UserKnownHostsFile=/home/user/.ssh/known_hosts {factory_user}@{factory_host} '
            '"sudo -n /usr/local/bin/drmap stop_mapping"',
        )
        self.declare_parameter("mapping_command_timeout_s", 120.0)
        self.declare_parameter("map_import_timeout_s", 180.0)
        self.declare_parameter("enable_map_pcd_postprocess", True)
        self.declare_parameter("pcd_terrain_cell_size", 0.25)
        self.declare_parameter("stair_zones_topic", "/m20pro/stair_zones")
        self.declare_parameter("goal_reached_tolerance_m", 0.3)
        self.declare_parameter("task_goal_resend_interval_s", 5.0)
        self.declare_parameter("task_start_settle_s", 0.5)
        self.declare_parameter("task_start_pose_timeout_s", 3.0)
        self.declare_parameter("task_start_require_localization_ok", True)
        self.declare_parameter("task_start_require_pose_on_map", True)
        self.declare_parameter("task_start_require_current_floor_match", False)
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
        self.declare_parameter("initialpose_publish_repeats", 10)
        self.declare_parameter("initialpose_publish_interval_s", 0.15)
        self.declare_parameter("relocalization_verify_timeout_s", 8.0)
        self.declare_parameter("relocalization_pose_tolerance_m", 2.0)
        self.declare_parameter("factory_initialpose_remote_publish", True)
        self.declare_parameter("factory_initialpose_topic", "/initialpose")
        self.declare_parameter(
            "factory_initialpose_source_command",
            "source /opt/robot/scripts/setup_ros2.sh >/dev/null 2>&1 || source /opt/ros/foxy/setup.bash",
        )
        self.declare_parameter("factory_initialpose_command_timeout_s", 15.0)
        self.declare_parameter("factory_initialpose_ssh_identity_file", "/home/user/.ssh/id_ed25519")
        self.declare_parameter("factory_initialpose_ssh_known_hosts_file", "/home/user/.ssh/known_hosts")
        self.declare_parameter("robot_pose_display_yaw_offset_rad", 0.0)
        self.declare_parameter("current_floor_topic", "/m20pro/current_floor")
        self.declare_parameter("stair_status_topic", "/m20pro/stair_status")
        self.declare_parameter("gait_command_topic", "/m20pro/gait_command")
        self.declare_parameter("gait_result_topic", "/m20pro_tcp_bridge/gait_result")
        self.declare_parameter("usage_mode_result_topic", "/m20pro_tcp_bridge/usage_mode_result")
        self.declare_parameter("localization_ok_topic", "/m20pro_tcp_bridge/localization_ok")
        self.declare_parameter("navigation_status_topic", "/m20pro_tcp_bridge/navigation_status")
        self.declare_parameter("battery_topic", "/BATTERY_DATA")
        self.declare_parameter("lidar_points_topic", "/LIDAR/POINTS")
        self.declare_parameter("lidar_points_relay_subscribe_topic", "/m20pro/lidar_points_relay")
        self.declare_parameter("enable_lidar_points_relay", False)
        self.declare_parameter("lidar_points_relay_topic", "/m20pro/lidar_points_relay")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_overlay_max_points", 720)
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
        self.declare_parameter("annotated_image_topic", "/m20pro_yolov8_inspection/annotated_image")
        self.declare_parameter("subscribe_annotated_image", False)
        self.declare_parameter("enable_camera_proxy", False)
        self.declare_parameter("front_camera_url", "rtsp://10.21.31.103:8554/video1")
        self.declare_parameter("rear_camera_url", "rtsp://10.21.31.103:8554/video2")
        self.declare_parameter("camera_proxy_fps", 3.0)
        self.declare_parameter("camera_proxy_jpeg_quality", 55)
        self.declare_parameter("camera_proxy_max_width", 480)
        self.declare_parameter("camera_proxy_transport", "tcp")
        self.declare_parameter(
            "camera_proxy_ffmpeg_options",
            "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;500000|stimeout;3000000",
        )
        self.declare_parameter("camera_proxy_open_timeout_s", 3.0)
        self.declare_parameter("camera_proxy_read_timeout_s", 3.0)
        self.declare_parameter("camera_proxy_reconnect_s", 0.5)
        self.declare_parameter("camera_proxy_frame_timeout_s", 2.0)
        self.declare_parameter("max_path_points", 800)
        self.declare_parameter("max_events", 30)
        self.declare_parameter("preflight_topic_timeout_s", 5.0)
        self.declare_parameter("preflight_settle_wait_s", 6.0)
        self.declare_parameter("preflight_min_battery_level", 20)

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

    def _load_builtin_maps(self) -> List[Dict[str, Any]]:
        manifest_path = self._map_manifest_path()
        if manifest_path is None or not manifest_path.exists():
            return []
        try:
            with manifest_path.open("r", encoding="utf-8") as file:
                manifest = yaml.safe_load(file) or {}
        except Exception as exc:
            self.get_logger().warning(f"failed to read map manifest {manifest_path}: {exc}")
            return []

        map_set = manifest.get("map_set") or {}
        source_note = str(map_set.get("source_note") or "").strip()
        self._default_builtin_floor = str(map_set.get("default_floor") or "").strip() or None
        maps: List[Dict[str, Any]] = []
        floors = manifest.get("floors") or {}
        if not isinstance(floors, dict):
            return []
        for floor, info in floors.items():
            if not isinstance(info, dict):
                continue
            yaml_value = str(info.get("map_yaml") or "").strip()
            if not yaml_value:
                continue
            try:
                yaml_path = FsPath(self._resolve_path(yaml_value))
            except Exception as exc:
                self.get_logger().warning(f"failed to resolve builtin map {floor}: {exc}")
                continue
            pcd_value = str(info.get("pcd_map") or map_set.get("global_pcd") or "").strip()
            pcd_path = ""
            if pcd_value:
                try:
                    pcd_path = self._resolve_path(pcd_value)
                except Exception:
                    pcd_path = pcd_value
            derived = self._builtin_map_derived(yaml_path, str(floor), f"builtin_{floor}", pcd_path)
            maps.append(
                {
                    "id": f"builtin_{floor}",
                    "name": str(info.get("label") or floor),
                    "floor": str(floor),
                    "level": info.get("level"),
                    "directory": str(yaml_path.parent),
                    "yaml_path": str(yaml_path),
                    "source": "project_builtin",
                    "readonly": True,
                    "pcd_path": pcd_path,
                    "derived": derived,
                    "source_note": source_note,
                    "created_at": "项目内置地图",
                }
            )
        maps.sort(key=lambda item: (int(item.get("level") or 0), str(item.get("floor") or "")))
        if self._default_builtin_floor:
            for item in maps:
                if item.get("floor") == self._default_builtin_floor:
                    self._default_builtin_map_id = str(item.get("id") or "") or None
                    break
        return maps

    def _builtin_map_derived(
        self,
        yaml_path: FsPath,
        floor: str,
        map_id: str,
        pcd_path: str,
    ) -> Dict[str, Any]:
        derived_dir = yaml_path.parent / "derived"
        terrain_path = derived_dir / "terrain_mesh.json"
        zones_path = derived_dir / "stair_zones.json"
        if terrain_path.exists() and zones_path.exists():
            return {
                "status": "ready",
                "message": "项目内置地图已有 PCD 派生 3D 地形",
                "terrain_mesh": str(terrain_path.relative_to(yaml_path.parent)),
                "height_grid": str((derived_dir / "height_grid.json").relative_to(yaml_path.parent)),
                "stair_zones": str(zones_path.relative_to(yaml_path.parent)),
                "pcd_path": pcd_path,
            }
        if pcd_path and FsPath(pcd_path).exists():
            return {
                "status": "pending",
                "message": "项目内置地图可执行 PCD 派生；为避免启动时改动仓库文件，请在导入归档地图时自动生成",
                "pcd_path": pcd_path,
            }
        return {
            "status": "missing_pcd",
            "message": "项目内置地图未配置可用 PCD",
        }

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
        archived_ids = {item.get("id") for item in self._maps}
        return [
            dict(item)
            for item in self._builtin_maps
            if item.get("id") not in archived_ids
        ] + [dict(item) for item in self._maps]

    def _find_map_record_unlocked(self, map_id: Optional[str]) -> Optional[Dict[str, Any]]:
        if not map_id:
            return None
        record = self._find_by_id(self._maps, map_id)
        if record is not None:
            return record
        return self._find_by_id(self._builtin_maps, map_id)

    def _default_map_id_unlocked(self) -> Optional[str]:
        if self._default_builtin_map_id and self._find_map_record_unlocked(self._default_builtin_map_id):
            return self._default_builtin_map_id
        for item in self._builtin_maps:
            if item.get("id") == "builtin_F20" or item.get("floor") == "F20":
                return str(item.get("id") or "") or None
        for item in self._builtin_maps:
            if item.get("id"):
                return str(item.get("id"))
        for item in self._maps:
            if item.get("id"):
                return str(item.get("id"))
        return None

    def _normalize_runtime_state_on_startup(self) -> None:
        active = self._settings.get("active_task") or {}
        changed = False
        selected_map_id = self._settings.get("selected_map_id")
        if selected_map_id and not self._find_map_record_unlocked(str(selected_map_id)):
            self.get_logger().warning(f"selected map {selected_map_id} no longer exists; falling back to default map")
            self._settings["selected_map_id"] = None
            changed = True
        if not self._settings.get("selected_map_id"):
            default_map_id = self._default_map_id_unlocked()
            if default_map_id:
                self._settings["selected_map_id"] = default_map_id
                changed = True
        for item in self._annotations:
            before = json.dumps(item, ensure_ascii=False, sort_keys=True)
            self._normalize_annotation_semantics(item)
            after = json.dumps(item, ensure_ascii=False, sort_keys=True)
            if before != after:
                changed = True
        if active:
            task_id = active.get("task_id")
            task = self._find_by_id(self._tasks, task_id)
            if active.get("status") == "running" and task is not None:
                task["status"] = "stopped"
                task["updated_at"] = _now_text()
            self._settings["active_task"] = None
            changed = True
        for task in self._tasks:
            if task.get("status") == "running":
                task["status"] = "stopped"
                task["updated_at"] = _now_text()
                changed = True
        if changed:
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
            self._save_json("annotations.json", self._annotations)

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
        self.create_subscription(PointCloud2, self._topic("lidar_points_topic"), self._on_lidar_points, 2)
        relay_subscribe_topic = self._topic("lidar_points_relay_subscribe_topic")
        if relay_subscribe_topic and relay_subscribe_topic != self._topic("lidar_points_topic"):
            self.create_subscription(
                PointCloud2,
                relay_subscribe_topic,
                self._on_lidar_points_relay,
                2,
            )
        self.create_subscription(LaserScan, self._topic("scan_topic"), self._on_scan, scan_qos)
        self.create_subscription(Odometry, self._topic("odom_topic"), self._on_odom, 10)
        self.create_subscription(PoseStamped, self._topic("pose_topic"), self._on_pose, 20)
        self.create_subscription(RosPath, self._topic("plan_topic"), self._on_path, 5)
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
            self._state["navigation_status_parsed"] = self._parse_navigation_status(msg.data)
            self._mark_topic("navigation_status")

    @staticmethod
    def _parse_navigation_status(text: str) -> Dict[str, Any]:
        parsed: Dict[str, Any] = {}
        for token in str(text or "").replace(",", " ").split():
            key, sep, value = token.partition("=")
            if not sep or not key:
                continue
            normalized_key = key.strip().lower()
            raw_value = value.strip()
            if raw_value in ("", "None", "none", "null"):
                parsed[normalized_key] = None
                continue
            try:
                parsed[normalized_key] = int(raw_value, 0)
            except ValueError:
                parsed[normalized_key] = raw_value
        return parsed

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

    def _on_lidar_points(self, msg: PointCloud2) -> None:
        self._remember_lidar_points(msg, "raw")
        if self.lidar_points_relay_pub is not None:
            self.lidar_points_relay_pub.publish(msg)

    def _on_lidar_points_relay(self, msg: PointCloud2) -> None:
        self._remember_lidar_points(msg, "relay")

    def _remember_lidar_points(self, msg: PointCloud2, source: str) -> None:
        stamp = _stamp_to_float(msg.header.stamp)
        with self._lock:
            self._state["lidar_points"] = {
                "last_update": time.time(),
                "stamp": stamp,
                "frame_id": msg.header.frame_id,
                "width": int(msg.width),
                "height": int(msg.height),
                "point_step": int(msg.point_step),
                "row_step": int(msg.row_step),
                "is_dense": bool(msg.is_dense),
                "source": source,
            }
            self._mark_topic("lidar_points")

    def _on_scan(self, msg: LaserScan) -> None:
        ranges_count = len(msg.ranges)
        finite_count = sum(1 for value in msg.ranges if math.isfinite(float(value)))
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
        points = []
        for index in range(0, ranges_count, step):
            try:
                distance = float(msg.ranges[index])
            except (TypeError, ValueError):
                continue
            if not math.isfinite(distance) or distance < min_range or distance > max_range:
                continue
            angle = float(msg.angle_min) + index * float(msg.angle_increment)
            points.append(
                {
                    "x": distance * math.cos(angle),
                    "y": distance * math.sin(angle),
                }
            )
        with self._lock:
            self._state["scan"] = {
                "last_update": time.time(),
                "stamp": _stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "ranges": ranges_count,
                "finite_ranges": finite_count,
                "angle_min": float(msg.angle_min),
                "angle_max": float(msg.angle_max),
                "range_min": float(getattr(msg, "range_min", 0.0) or 0.0),
                "range_max": float(getattr(msg, "range_max", 0.0) or 0.0),
                "overlay_points": len(points),
                "points": points,
            }
            self._mark_topic("scan")

    def _on_odom(self, msg: Odometry) -> None:
        pose = _pose_to_dict(msg.pose.pose)
        with self._lock:
            self._state["odom"] = {
                "last_update": time.time(),
                "stamp": _stamp_to_float(msg.header.stamp),
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "pose": pose,
                "finite": _is_finite_pose_dict(pose),
            }
            self._mark_topic("odom")

    def _on_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            pose = _pose_to_dict(msg.pose)
            if not _is_plausible_pose_dict(pose):
                self._mark_topic("pose_invalid")
                return
            raw_display_offset = float(self.get_parameter("robot_pose_display_yaw_offset_rad").value)
            display_offset = raw_display_offset if math.isfinite(raw_display_offset) else 0.0
            pose["display_yaw_offset_rad"] = display_offset
            pose["display_yaw_offset_deg"] = math.degrees(display_offset)
            if abs(display_offset) > 1e-12:
                display_yaw = _wrap_angle(pose["yaw"] + display_offset)
                pose["display_yaw"] = display_yaw
                pose["display_yaw_deg"] = math.degrees(display_yaw)
            stamp = _stamp_to_float(msg.header.stamp)
            if stamp is not None:
                pose["stamp"] = stamp
            pose["last_update"] = time.time()
            self._state["pose"] = pose
            self._mark_topic("pose")

    def _on_path(self, msg: RosPath) -> None:
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
            "last_update": time.time(),
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

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        info = msg.info
        with self._lock:
            self._state["local_costmap"] = {
                "last_update": time.time(),
                "stamp": _stamp_to_float(msg.header.stamp),
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
                "stamp": _stamp_to_float(msg.header.stamp),
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
                "parsed": _parse_json_text(msg.data),
            }
            self._mark_topic("detections")

    def _on_relocalization_result(self, msg: String) -> None:
        with self._lock:
            self._state["relocalization_result"] = {
                "last_update": time.time(),
                "raw": msg.data,
                "parsed": _parse_json_text(msg.data),
            }
            self._mark_topic("relocalization_result")

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
            for key in ("lidar_points", "scan", "odom", "local_costmap", "global_costmap"):
                if key in self._state:
                    snapshot[key] = dict(self._state[key])
            snapshot["topics"] = {
                key: dict(value)
                for key, value in self._state["topics"].items()
            }
            snapshot.pop("map", None)
        with self._data_lock:
            snapshot["selected_map_id"] = self._settings.get("selected_map_id")
            snapshot["active_task"] = self._settings.get("active_task")
        with self._preflight_lock:
            snapshot["preflight"] = self._preflight_with_age_unlocked()

        snapshot["ok"] = True
        snapshot["node_time"] = now
        snapshot["node_time_text"] = _now_text()
        snapshot["scan_overlay_offset"] = {
            "x": float(self.get_parameter("scan_overlay_offset_x_m").value),
            "y": float(self.get_parameter("scan_overlay_offset_y_m").value),
            "yaw": float(self.get_parameter("scan_overlay_offset_yaw_rad").value),
        }
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

    def _projects_payload(self) -> Dict[str, Any]:
        with self._data_lock:
            return {"ok": True, "projects": list(self._projects)}

    def _create_project(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        name = str(payload.get("name") or payload.get("project_name") or "").strip()
        building = str(payload.get("building") or "").strip()
        if not name:
            return self._error("项目名称不能为空")
        project = {
            "id": _new_id("project"),
            "name": name,
            "building": building,
            "created_at": _now_text(),
        }
        with self._data_lock:
            self._projects.append(project)
            self._save_json("projects.json", self._projects)
        return {"ok": True, "project": project}

    def _maps_payload(self) -> Dict[str, Any]:
        with self._data_lock:
            return {
                "ok": True,
                "maps": self._all_maps_unlocked(),
                "selected_map_id": self._settings.get("selected_map_id"),
            }

    def _preflight_payload(self) -> Dict[str, Any]:
        with self._preflight_lock:
            running = self._preflight_running_payload_unlocked()
            if running:
                return {"ok": True, "running": True, "preflight": running}
            return {"ok": True, "preflight": self._preflight_with_age_unlocked()}

    def _preflight_with_age_unlocked(self) -> Optional[Dict[str, Any]]:
        if not self._last_preflight:
            return None
        payload = dict(self._last_preflight)
        timestamp = payload.get("timestamp")
        if timestamp is not None:
            payload["age_sec"] = max(0.0, time.time() - float(timestamp))
        return payload

    def _preflight_running_payload_unlocked(self) -> Optional[Dict[str, Any]]:
        if not self._preflight_running:
            return None
        payload = dict(self._preflight_running)
        timestamp = payload.get("timestamp")
        if timestamp is not None:
            payload["age_sec"] = max(0.0, time.time() - float(timestamp))
        return payload

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
                "time_text": _now_text(),
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
        request_id = _new_id("preflight")
        running = {
            "ok": True,
            "running": True,
            "navigation_ready": False,
            "relocalization_ready": False,
            "mode": str(payload.get("mode") or "move").strip() or "move",
            "site": str(payload.get("site") or "workstation").strip() or "workstation",
            "timestamp": now,
            "time_text": _now_text(),
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
                "time_text": _now_text(),
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
        mode = str(payload.get("mode") or "move").strip()
        if mode not in ("move", "shadow"):
            mode = "move"
        site = str(payload.get("site") or "auto").strip().lower()
        explicit_workstation = site in ("workstation", "bench", "desk", "office", "charging")
        auto_site = site in ("", "auto", "unknown")
        self._wait_for_preflight_baseline()
        now = time.time()
        timeout_s = max(2.0, min(8.0, float(self.get_parameter("preflight_topic_timeout_s").value)))
        items: List[Dict[str, Any]] = []

        def add(
            key: str,
            label: str,
            status: str,
            message: str = "",
            group: str = "base",
        ) -> None:
            items.append(
                {
                    "key": key,
                    "label": label,
                    "status": status,
                    "message": message,
                    "group": group,
                }
            )

        node_names = set(self.get_node_names())
        base_required_nodes = [
            "m20pro_tcp_bridge",
            "m20pro_pointcloud_fusion",
            "m20pro_web_dashboard",
            "map_server",
            "m20pro_nav2_startup_gate",
            "m20pro_floor_manager",
        ]
        navigation_required_nodes = [
            "controller_server",
            "planner_server",
            "bt_navigator",
            "waypoint_follower",
        ]
        missing_nodes = [name for name in base_required_nodes if name not in node_names]
        add(
            "nodes",
            "核心节点",
            "ok" if not missing_nodes else "fail",
            "全部在线" if not missing_nodes else "缺少：" + "、".join(f"/{name}" for name in missing_nodes),
        )
        missing_navigation_nodes = [name for name in navigation_required_nodes if name not in node_names]
        add(
            "navigation_nodes",
            "导航节点",
            "ok" if not missing_navigation_nodes else "warn",
            (
                "全部在线"
                if not missing_navigation_nodes
                else "导航节点未完全显示；重定位后若仍缺少再禁止移动任务："
                + "、".join(f"/{name}" for name in missing_navigation_nodes)
            ),
            group="navigation",
        )

        topic_names = {name for name, _types in self.get_topic_names_and_types()}
        base_topics = [
            self._topic("lidar_points_topic"),
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
        missing_topics = [topic for topic in base_topics if topic not in topic_names]
        add(
            "topics",
            "基础话题",
            "ok" if not missing_topics else "fail",
            "全部存在" if not missing_topics else "缺少：" + "、".join(missing_topics),
        )
        missing_navigation_topics = [topic for topic in navigation_topics if topic not in topic_names]
        add(
            "navigation_topics",
            "导航话题",
            "ok" if not missing_navigation_topics else "warn",
            "全部存在" if not missing_navigation_topics else "重定位后应出现：" + "、".join(missing_navigation_topics),
            group="navigation",
        )

        with self._lock:
            current_state = {
                key: self._state.get(key)
                for key in (
                    "lidar_points",
                    "scan",
                    "odom",
                    "pose",
                    "battery",
                    "localization_ok",
                    "navigation_status",
                    "map",
                    "local_costmap",
                    "global_costmap",
                )
            }

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

        lidar_ok, lidar_age, lidar = fresh("lidar_points")
        lidar_points = 0
        if isinstance(lidar, dict):
            lidar_points = int(lidar.get("width", 0)) * max(1, int(lidar.get("height", 1)))
        perception_ok = (lidar_ok and lidar_points > 0) or (scan_ok and finite_ranges > 0)
        if lidar_ok and lidar_points > 0:
            lidar_status = "ok"
            lidar_message = f"{lidar_points} 点 / {fmt_age_text(lidar_age)}"
        elif scan_ok and finite_ranges > 0:
            lidar_status = "warn"
            lidar_message = (
                f"未直接缓存原始点云，但 /scan 新鲜且有效距离 {finite_ranges}；"
                "工位自检按感知链路可用处理"
            )
        else:
            lidar_status = "fail"
            lidar_message = "未收到 /LIDAR/POINTS，也没有可用 /scan"
        add(
            "lidar_points",
            "原始点云",
            lidar_status,
            lidar_message,
        )

        add(
            "scan",
            "二维激光",
            "ok" if scan_ok and finite_ranges > 0 else "warn",
            (
                f"有效距离 {finite_ranges} / {fmt_age_text(scan_age)}"
                if scan_age is not None
                else "未收到 /scan；未定位或 TF 未建立时可能暂时没有"
            ),
            group="navigation",
        )

        odom_ok, odom_age, odom = fresh("odom")
        odom_finite = bool(isinstance(odom, dict) and odom.get("finite"))
        add(
            "odom",
            "原厂里程计",
            "ok" if odom_ok and odom_finite else "warn",
            (
                f"位姿有效 / {fmt_age_text(odom_age)}"
                if odom_age is not None and odom_finite
                else "未收到有效 /ODOM；原厂未定位时可能出现 inf/异常坐标"
            ),
            group="navigation",
        )

        pose = current_state.get("pose")
        pose_has_stamp = isinstance(pose, dict) and _is_plausible_pose_dict(pose)
        pose_age = None
        if isinstance(pose, dict) and pose.get("stamp"):
            pose_age = max(0.0, now - float(pose["stamp"]))
        add(
            "map_pose",
            "地图位姿",
            "ok" if pose_has_stamp else "warn",
            (
                f"x={float(pose.get('x', 0.0)):.2f} y={float(pose.get('y', 0.0)):.2f}"
                if pose_has_stamp
                else "未收到有效 /m20pro_tcp_bridge/map_pose；到测试场地后先重定位"
            ),
            group="navigation",
        )

        loc_ok = current_state.get("localization_ok") is True
        nav_status_text = str(current_state.get("navigation_status") or "")
        unlocalized = (not loc_ok) or ("location=1" in nav_status_text.lower())
        workstation_mode = explicit_workstation or (auto_site and unlocalized)
        add(
            "localization",
            "定位状态",
            "ok" if loc_ok else "warn",
            (
                "localization_ok=true"
                if loc_ok
                else "当前在工位/未重定位，定位未确认是预期状态；到测试场地后先重定位"
            ),
            group="navigation",
        )

        add(
            "navigation_status",
            "原厂导航状态",
            "ok" if nav_status_text else "warn",
            nav_status_text or "暂未收到 navigation_status",
        )

        map_ok = isinstance(current_state.get("map"), dict)
        add("map", "地图", "ok" if map_ok else "fail", "已加载 /map" if map_ok else "未收到 /map")

        local_ok, local_age, local_costmap = fresh("local_costmap")
        local_size_ok = bool(isinstance(local_costmap, dict) and local_costmap.get("width") and local_costmap.get("height"))
        global_ok, global_age, global_costmap = fresh("global_costmap")
        global_size_ok = bool(isinstance(global_costmap, dict) and global_costmap.get("width") and global_costmap.get("height"))
        costmap_deferred = workstation_mode or unlocalized
        if costmap_deferred:
            add(
                "local_costmap",
                "局部代价地图",
                "ok" if local_ok and local_size_ok else "info",
                (
                    f"{local_costmap.get('width')}x{local_costmap.get('height')} / {fmt_age_text(local_age)}"
                    if isinstance(local_costmap, dict)
                    else "未重定位前 Nav2/costmap 允许延后启动；先完成重定位再严格检查"
                ),
                group="navigation",
            )
            add(
                "global_costmap",
                "全局代价地图",
                "ok" if global_ok and global_size_ok else "info",
                (
                    f"{global_costmap.get('width')}x{global_costmap.get('height')} / {fmt_age_text(global_age)}"
                    if isinstance(global_costmap, dict)
                    else "未重定位前 Nav2/costmap 允许延后启动；先完成重定位再严格检查"
                ),
                group="navigation",
            )
        else:
            add(
                "local_costmap",
                "局部代价地图",
                "ok" if local_ok and local_size_ok else "warn",
                (
                    f"{local_costmap.get('width')}x{local_costmap.get('height')} / {fmt_age_text(local_age)}"
                    if isinstance(local_costmap, dict)
                    else "已定位但未收到 local_costmap；不要开始移动任务"
                ),
                group="navigation",
            )
            add(
                "global_costmap",
                "全局代价地图",
                "ok" if global_ok and global_size_ok else "warn",
                (
                    f"{global_costmap.get('width')}x{global_costmap.get('height')} / {fmt_age_text(global_age)}"
                    if isinstance(global_costmap, dict)
                    else "已定位但未收到 global_costmap；不要开始移动任务"
                ),
                group="navigation",
            )

        battery = current_state.get("battery")
        primary = battery.get("primary") if isinstance(battery, dict) else None
        battery_level = int(primary.get("level", 0)) if isinstance(primary, dict) else 0
        min_level = int(self.get_parameter("preflight_min_battery_level").value)
        add(
            "battery",
            "电量",
            "ok" if isinstance(primary, dict) and battery_level >= min_level else "fail",
            f"{battery_level}% / 最低要求 {min_level}%" if isinstance(primary, dict) else "未收到电池数据",
        )

        if workstation_mode or unlocalized:
            add(
                "nav2_lifecycle_deferred",
                "Nav2 生命周期",
                "info",
                "当前在工位/未重定位，Nav2 可由启动门延后激活；重定位后再确认 active",
                group="navigation",
            )
        else:
            lifecycle_results = self._check_lifecycle_nodes(
                ["/map_server", "/controller_server", "/planner_server", "/bt_navigator"]
            )
            for node_name, lifecycle in lifecycle_results.items():
                add(
                    f"lifecycle:{node_name}",
                    f"{node_name} 生命周期",
                    "ok" if lifecycle.get("active") else "warn",
                    lifecycle.get("message", ""),
                    group="navigation",
                )

        motion = self._detect_motion_mode()
        if mode == "move":
            motion_ok = motion.get("mode") == "move"
            add(
                "motion_mode",
                "运动模式",
                "ok" if motion_ok else "fail",
                motion.get("message") or "未确认 move 模式，请用 104_start_real_move.sh 全量启动",
            )
        else:
            add(
                "motion_mode",
                "运动模式",
                "ok" if motion.get("mode") in ("shadow", "move") else "warn",
                motion.get("message") or "未确认运动模式",
            )

        failures = [item for item in items if item["status"] == "fail" and item.get("group") == "base"]
        if not perception_ok and not any(item.get("key") == "lidar_points" for item in failures):
            failures.append(
                {
                    "key": "perception_chain",
                    "label": "感知链路",
                    "status": "fail",
                    "message": "原始点云和 /scan 都不可用",
                    "group": "base",
                }
            )
        navigation_failures = [
            item
            for item in items
            if item.get("group") == "navigation" and item["status"] in ("fail", "warn")
        ]
        warnings = [item for item in items if item["status"] == "warn"]
        relocalization_blockers = [
            item
            for item in failures
            if item.get("key") in ("nodes", "topics", "lidar_points", "perception_chain", "map")
        ]
        relocalization_ready = bool(map_ok and perception_ok and not relocalization_blockers)
        failure_labels = "、".join(str(item.get("label") or item.get("key")) for item in failures)
        if not failures:
            summary = (
                "基础自检通过，导航已就绪"
                if not navigation_failures
                else "基础自检通过，当前在工位，导航待到测试场地重定位后确认"
            )
        elif relocalization_ready:
            summary = (
                f"基础自检未通过：{len(failures)} 项失败"
                f"（{failure_labels}）；地图/点云/scan 可用，仍可先做重定位排查，不要开始移动任务"
            )
        else:
            summary = f"基础自检未通过：{len(failures)} 项失败"
        result = {
            "ok": not failures,
            "navigation_ready": not navigation_failures,
            "relocalization_ready": relocalization_ready,
            "mode": mode,
            "site": "workstation" if workstation_mode else site,
            "site_mode": "workstation" if workstation_mode else "field",
            "workstation_mode": workstation_mode,
            "timestamp": now,
            "time_text": _now_text(),
            "age_sec": 0.0,
            "items": items,
            "failures": len(failures),
            "navigation_warnings": len(navigation_failures),
            "warnings": len(warnings),
            "summary": summary,
        }
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
                lidar = dict(self._state.get("lidar_points") or {})
                scan = dict(self._state.get("scan") or {})
                battery = dict(self._state.get("battery") or {})
                navigation_status = self._state.get("navigation_status")
                map_seen = isinstance(self._state.get("map"), dict)
            lidar_ok = (
                bool(lidar.get("width"))
                and now - float(lidar.get("last_update", 0.0) or 0.0) <= 2.0
            )
            scan_ok = (
                int(scan.get("finite_ranges", 0) or 0) > 0
                and now - float(scan.get("last_update", 0.0) or 0.0) <= 2.0
            )
            battery_ok = now - float(battery.get("last_update", 0.0) or 0.0) <= 5.0
            if map_seen and (lidar_ok or scan_ok) and battery_ok and navigation_status:
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
            if "m20pro_bringup" in line and ("m20pro_real.launch.py" in line or "m20pro_real_full.sh" in line)
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
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") == "running":
                return self._error("任务执行中不能切换地图，请先停止当前任务")
            if map_id and not self._find_map_record_unlocked(map_id):
                return self._error("地图不存在")
            self._settings["selected_map_id"] = map_id
            self._save_json("settings.json", self._settings)
        return {"ok": True, "selected_map_id": map_id}

    def _create_mapping_session(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        project_name = str(payload.get("project_name") or payload.get("name") or "").strip()
        building = str(payload.get("building") or "").strip()
        mode = str(payload.get("mode") or "multi").strip()
        floors = payload.get("floors") or []
        if isinstance(floors, str):
            floors = [item.strip() for item in floors.split(",") if item.strip()]
        floors = [str(item).strip() for item in floors if str(item).strip()]
        active_floor = str(payload.get("active_floor") or (floors[0] if floors else "")).strip()
        map_name = _sanitize_name(
            str(payload.get("map_name") or ""),
            f"{active_floor or 'map'}_{time.strftime('%Y%m%d_%H%M%S', time.localtime())}",
        )
        if not project_name:
            project_name = "M20Pro 工地巡检"
        project = self._find_project(project_name, building)
        if project is None:
            project = {
                "id": _new_id("project"),
                "name": project_name,
                "building": building,
                "created_at": _now_text(),
            }
            self._projects.append(project)
        session = {
            "id": _new_id("map_session"),
            "project_id": project["id"],
            "project_name": project_name,
            "building": building,
            "mode": mode,
            "floors": floors,
            "active_floor": active_floor,
            "map_name": map_name,
            "status": "created",
            "created_at": _now_text(),
            "updated_at": _now_text(),
        }
        with self._data_lock:
            self._sessions.append(session)
            self._save_json("projects.json", self._projects)
            self._save_json("mapping_sessions.json", self._sessions)
        self._append_event(
            "建立建图任务",
            {"session_id": session["id"], "mode": mode, "floors": floors, "map_name": map_name},
        )
        return {"ok": True, "session": session}

    def _mapping_command(self, param_name: str, session_id: Optional[str]) -> Dict[str, Any]:
        session = self._find_session(session_id)
        if session is None:
            return self._error("建图任务不存在，请先建立建图任务")
        context = self._command_context(session)
        result = self._run_configured_command(param_name, context)
        if result.get("ok"):
            session["status"] = {
                "factory_mapping_start_command": "mapping",
                "factory_mapping_finish_command": "saved",
                "factory_mapping_cancel_command": "cancelled",
            }.get(param_name, session.get("status", "updated"))
        elif result.get("manual_required"):
            session["status"] = "waiting_manual"
        session["updated_at"] = _now_text()
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
        drmap = "/usr/local/bin/drmap"
        if factory_host in ("", "localhost", "127.0.0.1"):
            command = (
                "set -e; "
                "echo host=$(hostname); "
                "echo user=$(whoami); "
                f"echo drmap=$({shlex.quote(drmap)} --help >/dev/null 2>&1; command -v {shlex.quote(drmap)} || true); "
                f"echo active=$(readlink -f {shlex.quote(active_map)} || true); "
                f"test -d {shlex.quote(active_map)}; "
                f"sudo -n -l {shlex.quote(drmap)} mapping -s -n m20pro_preflight >/dev/null; "
                f"sudo -n -l {shlex.quote(drmap)} stop_mapping >/dev/null"
            )
        else:
            target = f"{factory_user}@{factory_host}" if factory_user else factory_host
            ssh_options, _used_identity, _used_known_hosts = self._factory_ssh_file_options(8)
            remote_probe = (
                "set -e; "
                "echo host=$(hostname); "
                "echo user=$(whoami); "
                f"echo drmap={shlex.quote(drmap)}; "
                f"test -x {shlex.quote(drmap)}; "
                f"echo active=$(readlink -f {shlex.quote(active_map)} || true); "
                f"test -d {shlex.quote(active_map)}; "
                f"sudo -n -l {shlex.quote(drmap)} mapping -s -n m20pro_preflight >/dev/null; "
                f"sudo -n -l {shlex.quote(drmap)} stop_mapping >/dev/null"
            )
            command = " ".join(
                shlex.quote(item)
                for item in ["ssh", *ssh_options, target, f"bash -lc {shlex.quote(remote_probe)}"]
            )
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
            payload["message"] = "106 建图环境可用：SSH、drmap、active map、sudo -n 权限均通过。"
        else:
            payload["message"] = (
                "106 建图环境未通过。常见原因：104 到 106 未配置 SSH 免密，"
                "或 106 上 sudo drmap 仍需要交互输入密码。"
            )
        return payload

    def _import_active_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        session_id = payload.get("session_id")
        session = self._find_session(str(session_id).strip() if session_id else None)
        floor = str(payload.get("floor") or (session or {}).get("active_floor") or "").strip()
        if not floor:
            return self._error("请指定地图楼层")

        source = str(payload.get("source") or self.get_parameter("factory_active_map").value).strip()
        factory_host = str(payload.get("factory_host") or self.get_parameter("factory_host").value).strip()
        factory_user = str(payload.get("factory_user") or self.get_parameter("factory_user").value).strip()
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        default_name = f"{floor}_{stamp}"
        map_name = _sanitize_name(str(payload.get("map_name") or ""), default_name)
        dest = self.map_archive_dir / map_name
        if dest.exists():
            dest = self.map_archive_dir / f"{map_name}_{uuid.uuid4().hex[:6]}"
        timeout = float(self.get_parameter("map_import_timeout_s").value)

        try:
            if factory_host in ("", "localhost", "127.0.0.1"):
                shutil.copytree(source, dest)
                command_text = f"copy {source} -> {dest}"
                command_output = ""
            else:
                remote = f"{factory_user}@{factory_host}:{source.rstrip('/')}"
                ssh_options, used_identity, used_known_hosts = self._factory_ssh_file_options(8)
                command = ["scp", "-r", *ssh_options, remote, str(dest)]
                result = subprocess.run(
                    command,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    timeout=timeout,
                    check=False,
                )
                command_text = " ".join(shlex.quote(item) for item in command)
                command_output = result.stdout or ""
                if result.returncode != 0:
                    return self._error(
                        "从 106 拉取地图失败，请确认 104 到 106 的 SSH/scp 可用",
                        {
                            "command": command_text,
                            "output": command_output,
                            "ssh_identity_file": used_identity,
                            "ssh_known_hosts_file": used_known_hosts,
                        },
                    )
        except Exception as exc:
            return self._error("从 106 拉取地图失败", {"error": str(exc)})

        yaml_path = self._find_map_yaml(dest)
        if yaml_path is None:
            return self._error(
                "地图已拉取，但没有找到 occ_grid.yaml/map.yaml/jueying.yaml",
                {"directory": str(dest), "command": command_text, "output": command_output},
            )
        self._rewrite_imported_map_image_path(yaml_path)

        map_record = {
            "id": _new_id("map"),
            "name": map_name,
            "floor": floor,
            "mode": (session or {}).get("mode"),
            "project_id": (session or {}).get("project_id"),
            "project_name": (session or {}).get("project_name"),
            "building": (session or {}).get("building"),
            "directory": str(dest),
            "yaml_path": str(yaml_path),
            "source": "106_active_map",
            "source_path": source,
            "created_at": _now_text(),
        }
        map_record["derived"] = self._generate_map_derived(map_record, dest, yaml_path, floor)
        with self._data_lock:
            self._maps.append(map_record)
            self._settings["selected_map_id"] = map_record["id"]
            if session:
                session["status"] = "imported"
                session["updated_at"] = _now_text()
            self._save_json("maps.json", self._maps)
            self._save_json("settings.json", self._settings)
            self._save_json("mapping_sessions.json", self._sessions)
        self._append_event("从 106 拉取地图完成", {"map_id": map_record["id"], "floor": floor})
        return {
            "ok": True,
            "map": map_record,
            "selected_map_id": map_record["id"],
            "command": command_text,
            "output": command_output,
        }

    def _factory_ssh_file_options(self, connect_timeout_s: int = 5) -> Tuple[List[str], str, str]:
        ssh_identity = str(self.get_parameter("factory_initialpose_ssh_identity_file").value).strip()
        ssh_known_hosts = str(self.get_parameter("factory_initialpose_ssh_known_hosts_file").value).strip()
        options = [
            "-o",
            "BatchMode=yes",
            "-o",
            f"ConnectTimeout={int(connect_timeout_s)}",
            "-o",
            "StrictHostKeyChecking=accept-new",
        ]
        used_identity = ""
        used_known_hosts = ""
        if ssh_identity and FsPath(ssh_identity).exists():
            used_identity = ssh_identity
            options.extend(["-i", ssh_identity, "-o", "IdentitiesOnly=yes"])
        if ssh_known_hosts and FsPath(ssh_known_hosts).exists():
            used_known_hosts = ssh_known_hosts
            options.extend(["-o", f"UserKnownHostsFile={ssh_known_hosts}"])
        return options, used_identity, used_known_hosts

    def _rewrite_imported_map_image_path(self, yaml_path: FsPath) -> None:
        try:
            with yaml_path.open("r", encoding="utf-8") as file:
                info = yaml.safe_load(file) or {}
            image_value = str(info.get("image") or "").strip()
            if not image_value:
                return
            local_image = yaml_path.parent / FsPath(image_value).name
            if local_image.exists() and image_value != local_image.name:
                info["image"] = local_image.name
                with yaml_path.open("w", encoding="utf-8") as file:
                    yaml.safe_dump(info, file, allow_unicode=True, sort_keys=False)
        except Exception as exc:
            self.get_logger().warning(f"failed to rewrite imported map image path: {exc}")

    def _generate_map_derived(
        self,
        map_record: Dict[str, Any],
        map_dir: FsPath,
        yaml_path: FsPath,
        floor: str,
    ) -> Dict[str, Any]:
        if not self._as_bool(self.get_parameter("enable_map_pcd_postprocess").value):
            return {
                "status": "disabled",
                "message": "PCD 后处理未启用",
            }
        try:
            floor_config = self._floor_config_path()
            result = process_imported_map(
                FsPath(map_dir),
                FsPath(yaml_path),
                floor,
                str(map_record.get("id") or ""),
                floor_config_path=floor_config,
                cell_size=float(self.get_parameter("pcd_terrain_cell_size").value),
            )
            self._append_event(
                "地图 PCD 派生完成",
                {
                    "map_id": map_record.get("id"),
                    "floor": floor,
                    "status": result.get("status"),
                    "message": result.get("message"),
                },
            )
            return result
        except Exception as exc:
            self.get_logger().warning("map PCD postprocess failed: %s" % exc)
            return {
                "status": "failed",
                "message": str(exc) or exc.__class__.__name__,
            }

    def _floor_config_path(self) -> Optional[FsPath]:
        try:
            return FsPath(get_package_share_directory("m20pro_bringup")) / "config" / "inspection_waypoints.yaml"
        except PackageNotFoundError:
            return None

    def _map_3d_payload(self, map_id: Optional[str]) -> Dict[str, Any]:
        with self._data_lock:
            if not map_id:
                map_id = self._settings.get("selected_map_id")
            record = self._find_map_record_unlocked(map_id)
        if record is None:
            return {"ok": True, "available": False, "message": "未选择固定地图"}
        derived = record.get("derived") if isinstance(record.get("derived"), dict) else {}
        terrain_rel = str(derived.get("terrain_mesh") or "").strip()
        if not terrain_rel:
            if (
                record.get("source") == "project_builtin"
                and derived.get("status") == "pending"
                and str(record.get("pcd_path") or "").strip()
                and self._as_bool(self.get_parameter("enable_map_pcd_postprocess").value)
            ):
                terrain_rel = self._ensure_builtin_map_derived(record, derived)
                derived = record.get("derived") if isinstance(record.get("derived"), dict) else derived
        if not terrain_rel:
            return {
                "ok": True,
                "available": False,
                "map_id": record.get("id"),
                "status": derived.get("status") or "missing",
                "message": derived.get("message") or "当前地图没有 PCD/3D 地形，2D 地图仍可用",
            }
        terrain_path = self._resolve_map_asset_path(record, terrain_rel)
        if terrain_path is None or not terrain_path.exists():
            return {
                "ok": True,
                "available": False,
                "map_id": record.get("id"),
                "status": "missing_file",
                "message": "3D 地形文件不存在，请重新从 106 拉取地图",
            }
        try:
            payload = self._read_json_file(terrain_path)
        except Exception as exc:
            return {
                "ok": True,
                "available": False,
                "map_id": record.get("id"),
                "status": "failed",
                "message": str(exc),
            }
        payload["ok"] = True
        payload["available"] = True
        payload["map"] = {
            "id": record.get("id"),
            "name": record.get("name"),
            "floor": record.get("floor"),
            "derived_status": derived.get("status"),
            "derived_message": derived.get("message"),
        }
        return payload

    def _ensure_builtin_map_derived(self, record: Dict[str, Any], derived: Dict[str, Any]) -> str:
        yaml_path = FsPath(self._resolve_path(str(record.get("yaml_path") or "")))
        pcd_path = FsPath(self._resolve_path(str(record.get("pcd_path") or "")))
        if not yaml_path.exists() or not pcd_path.exists():
            return ""
        cache_dir = self.data_dir / "builtin_derived" / _sanitize_name(str(record.get("id") or "builtin"), "builtin")
        cache_dir.mkdir(parents=True, exist_ok=True)
        result = process_imported_map(
            cache_dir,
            yaml_path,
            str(record.get("floor") or ""),
            str(record.get("id") or ""),
            floor_config_path=self._floor_config_path(),
            pcd_path_override=pcd_path,
            cell_size=float(self.get_parameter("pcd_terrain_cell_size").value),
        )
        result["base_dir"] = str(cache_dir)
        record["derived"] = result
        with self._data_lock:
            for builtin in self._builtin_maps:
                if builtin.get("id") == record.get("id"):
                    builtin["derived"] = result
                    break
        return str(result.get("terrain_mesh") or "")

    def _stair_zones_payload(self, map_id: Optional[str]) -> Dict[str, Any]:
        with self._data_lock:
            if not map_id:
                map_id = self._settings.get("selected_map_id")
            record = self._find_map_record_unlocked(map_id)
        if record is None:
            return {"ok": True, "available": False, "message": "未选择固定地图", "zones": []}
        derived = record.get("derived") if isinstance(record.get("derived"), dict) else {}
        zones_rel = str(derived.get("stair_zones") or "").strip()
        if not zones_rel:
            if (
                record.get("source") == "project_builtin"
                and derived.get("status") == "pending"
                and str(record.get("pcd_path") or "").strip()
                and self._as_bool(self.get_parameter("enable_map_pcd_postprocess").value)
            ):
                self._ensure_builtin_map_derived(record, derived)
                derived = record.get("derived") if isinstance(record.get("derived"), dict) else derived
                zones_rel = str(derived.get("stair_zones") or "").strip()
        if not zones_rel:
            return {
                "ok": True,
                "available": False,
                "map_id": record.get("id"),
                "floor": record.get("floor"),
                "message": "当前地图没有楼梯语义区",
                "zones": [],
            }
        zones_path = self._resolve_map_asset_path(record, zones_rel)
        if zones_path is None or not zones_path.exists():
            return {
                "ok": True,
                "available": False,
                "map_id": record.get("id"),
                "floor": record.get("floor"),
                "message": "楼梯语义区文件不存在",
                "zones": [],
            }
        try:
            payload = self._read_json_file(zones_path)
        except Exception as exc:
            return {
                "ok": True,
                "available": False,
                "map_id": record.get("id"),
                "floor": record.get("floor"),
                "message": str(exc),
                "zones": [],
            }
        payload["ok"] = True
        payload["available"] = True
        payload["map"] = {
            "id": record.get("id"),
            "name": record.get("name"),
            "floor": record.get("floor"),
            "derived_status": derived.get("status"),
        }
        return payload

    def _stair_pointcloud_payload(self, map_id: Optional[str], zone_id: Optional[str]) -> Dict[str, Any]:
        if not zone_id:
            return self._error("缺少 zone_id")
        zones_payload = self._stair_zones_payload(map_id)
        zones = zones_payload.get("zones") or []
        zone = next((item for item in zones if str(item.get("id")) == str(zone_id)), None)
        if zone is None:
            return {"ok": True, "available": False, "message": "楼梯区域不存在"}
        pointcloud_rel = str(zone.get("pointcloud") or "").strip()
        if not pointcloud_rel:
            return {"ok": True, "available": False, "message": "该楼梯区域没有局部点云"}
        with self._data_lock:
            record = self._find_map_record_unlocked(map_id or zones_payload.get("map_id"))
        if record is None:
            return {"ok": True, "available": False, "message": "地图不存在"}
        path = self._resolve_map_asset_path(record, pointcloud_rel)
        if path is None or not path.exists():
            return {"ok": True, "available": False, "message": "局部点云文件不存在"}
        try:
            payload = self._read_json_file(path)
        except Exception as exc:
            return {"ok": True, "available": False, "message": str(exc)}
        payload["ok"] = True
        payload["available"] = True
        return payload

    def _publish_selected_stair_zones(self) -> None:
        try:
            with self._data_lock:
                map_id = self._settings.get("selected_map_id")
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
                    "updated_at": _now_text(),
                },
                ensure_ascii=False,
                separators=(",", ":"),
            )
            self.stair_zones_pub.publish(msg)
        except Exception as exc:
            self.get_logger().debug("failed to publish stair zones: %s" % exc)

    def _resolve_map_asset_path(self, record: Dict[str, Any], relative_path: str) -> Optional[FsPath]:
        value = str(relative_path or "").strip()
        if not value:
            return None
        path = FsPath(os.path.expandvars(os.path.expanduser(value)))
        if path.is_absolute():
            return path
        derived = record.get("derived") if isinstance(record.get("derived"), dict) else {}
        base_dir = str(derived.get("base_dir") or "").strip()
        if base_dir:
            return FsPath(self._resolve_path(base_dir)) / path
        directory = str(record.get("directory") or "").strip()
        if not directory:
            return None
        return FsPath(self._resolve_path(directory)) / path

    @staticmethod
    def _read_json_file(path: FsPath) -> Dict[str, Any]:
        with FsPath(path).open("r", encoding="utf-8") as file:
            payload = json.load(file)
        if not isinstance(payload, dict):
            raise RuntimeError("JSON payload is not an object")
        return payload

    def _annotations_payload(self, query: Dict[str, List[str]]) -> Dict[str, Any]:
        map_id = (query.get("map_id") or [None])[0]
        with self._data_lock:
            annotations = list(self._annotations)
        if map_id:
            annotations = [item for item in annotations if item.get("map_id") == map_id]
        return {"ok": True, "annotations": annotations}

    def _create_annotation(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        pose = payload.get("pose") or {}
        try:
            x = float(pose.get("x"))
            y = float(pose.get("y"))
            z = float(pose.get("z", 0.0))
            yaw = float(pose.get("yaw", 0.0))
        except (TypeError, ValueError):
            return self._error("点位坐标无效，请先点击地图取点")
        floor = str(payload.get("floor") or "").strip()
        if not floor:
            return self._error("点位楼层不能为空")
        point_type = str(payload.get("type") or "patrol").strip()
        label = str(payload.get("label") or "").strip()
        if not label:
            label = f"{floor}_{point_type}_{len(self._annotations) + 1}"
        map_id = str(payload.get("map_id") or "").strip() or None
        if map_id == "live_map":
            map_id = "live_map"
        with self._data_lock:
            if map_id and map_id != "live_map" and not self._find_map_record_unlocked(map_id):
                return self._error("地图不存在")
            if not map_id:
                map_id = self._settings.get("selected_map_id")
            if not map_id and self._state.get("map"):
                map_id = "live_map"
            if not map_id:
                return self._error("没有可用地图，请等待实时 /map 或先选择固定地图")
        item = {
            "id": _new_id("point"),
            "map_id": map_id,
            "type": point_type,
            "floor": floor,
            "label": label,
            "area": str(payload.get("area") or payload.get("region") or "").strip(),
            "room": str(payload.get("room") or payload.get("place") or "").strip(),
            "result_file_prefix": str(payload.get("result_file_prefix") or "").strip(),
            "pose": {"x": x, "y": y, "z": z, "yaw": yaw},
            "dwell_s": self._resolve_dwell_s(payload),
            "manual_point_type": self._manual_point_type_from_payload(payload),
            "vendor_navigation": self._vendor_navigation_from_payload(payload),
            "camera": str(payload.get("camera") or "").strip(),
            "target_classes": self._string_list(payload.get("target_classes")),
            "notes": str(payload.get("notes") or "").strip(),
            "created_at": _now_text(),
        }
        self._normalize_annotation_semantics(item)
        with self._data_lock:
            self._annotations.append(item)
            self._save_json("annotations.json", self._annotations)
        self._append_event("保存地图点位", {"annotation_id": item["id"], "floor": floor, "type": item["type"]})
        return {"ok": True, "annotation": item}

    def _manual_point_type_from_payload(self, payload: Dict[str, Any]) -> str:
        value = str(payload.get("manual_point_type") or "").strip()
        if value in MANUAL_POINT_TYPES:
            return value
        legacy_type = str(payload.get("type") or "patrol").strip()
        return UI_TYPE_TO_MANUAL_POINT_TYPE.get(legacy_type, "task")

    def _resolve_dwell_s(self, payload: Dict[str, Any]) -> float:
        raw = payload.get("dwell_s", payload.get("inspect_duration_s", payload.get("stay_duration_s")))
        if raw is not None and str(raw).strip() != "":
            try:
                return max(0.0, float(raw))
            except (TypeError, ValueError):
                return 0.0
        manual_type = self._manual_point_type_from_payload(payload)
        if manual_type == "transition":
            return max(0.0, float(self.get_parameter("default_transition_dwell_s").value))
        if manual_type == "charge":
            return max(0.0, float(self.get_parameter("default_charge_dwell_s").value))
        return max(0.0, float(self.get_parameter("default_task_dwell_s").value))

    def _vendor_navigation_from_payload(self, payload: Dict[str, Any]) -> Dict[str, int]:
        manual_type = self._manual_point_type_from_payload(payload)
        defaults = dict(DEFAULT_VENDOR_NAVIGATION)
        defaults["PointInfo"] = int(MANUAL_POINT_TYPES[manual_type]["point_info"])
        defaults["NavMode"] = int(MANUAL_POINT_TYPES[manual_type]["default_nav_mode"])
        raw = payload.get("vendor_navigation") or {}
        if not isinstance(raw, dict):
            raw = {}
        aliases = {
            "value": "Value",
            "map_id": "MapID",
            "point_info": "PointInfo",
            "gait": "Gait",
            "speed": "Speed",
            "manner": "Manner",
            "obs_mode": "ObsMode",
            "nav_mode": "NavMode",
        }
        for key, canonical in aliases.items():
            if key in payload:
                raw[canonical] = payload[key]
        for key in list(defaults.keys()) + ["PointInfo"]:
            if key not in raw:
                continue
            try:
                defaults[key] = int(raw[key])
            except (TypeError, ValueError):
                pass
        return defaults

    def _normalize_annotation_semantics(self, item: Dict[str, Any]) -> Dict[str, Any]:
        legacy_type = str(item.get("type") or "patrol").strip()
        manual_type = str(item.get("manual_point_type") or "").strip()
        if manual_type not in MANUAL_POINT_TYPES:
            manual_type = UI_TYPE_TO_MANUAL_POINT_TYPE.get(legacy_type, "task")
        item["manual_point_type"] = manual_type

        vendor = item.get("vendor_navigation")
        if not isinstance(vendor, dict):
            vendor = {}
        merged = dict(DEFAULT_VENDOR_NAVIGATION)
        merged["PointInfo"] = int(MANUAL_POINT_TYPES[manual_type]["point_info"])
        merged["NavMode"] = int(MANUAL_POINT_TYPES[manual_type]["default_nav_mode"])
        for key in merged:
            if key not in vendor:
                continue
            try:
                merged[key] = int(vendor[key])
            except (TypeError, ValueError):
                pass
        item["vendor_navigation"] = merged

        if "dwell_s" not in item and "inspect_duration_s" in item:
            item["dwell_s"] = item.get("inspect_duration_s")
        try:
            item["dwell_s"] = max(0.0, float(item.get("dwell_s", MANUAL_POINT_TYPES[manual_type]["default_dwell_s"])))
        except (TypeError, ValueError):
            item["dwell_s"] = float(MANUAL_POINT_TYPES[manual_type]["default_dwell_s"])
        item["inspect_duration_s"] = item["dwell_s"]

        item["label"] = str(item.get("label") or item.get("name") or item.get("id") or "").strip()
        item["area"] = str(item.get("area") or item.get("region") or "").strip()
        item["room"] = str(item.get("room") or item.get("place") or item.get("location") or "").strip()
        result_prefix = str(item.get("result_file_prefix") or "").strip()
        item["result_file_prefix"] = result_prefix or self._annotation_result_prefix(item)

        if "camera" not in item:
            item["camera"] = ""
        item["target_classes"] = self._string_list(item.get("target_classes"))
        return item

    def _annotation_result_prefix(self, item: Dict[str, Any]) -> str:
        parts = [
            str(item.get("floor") or "").strip(),
            str(item.get("area") or "").strip(),
            str(item.get("room") or "").strip(),
            str(item.get("label") or item.get("id") or "").strip(),
        ]
        raw = "_".join(part for part in parts if part)
        return _sanitize_name(raw, str(item.get("id") or "inspection_result"))

    @staticmethod
    def _string_list(value: Any) -> List[str]:
        if value is None:
            return []
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        if isinstance(value, list):
            return [str(item).strip() for item in value if str(item).strip()]
        return []

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
            affected_tasks = []
            for task in self._tasks:
                ids = list(task.get("annotation_ids") or [])
                if annotation_id not in ids:
                    continue
                task["annotation_ids"] = [item for item in ids if item != annotation_id]
                task["updated_at"] = _now_text()
                if not task["annotation_ids"]:
                    task["status"] = "invalid"
                elif task.get("status") in ("ready", "stopped", "completed"):
                    task["status"] = "ready"
                affected_tasks.append(task.get("id"))
            self._save_json("annotations.json", self._annotations)
            self._save_json("tasks.json", self._tasks)
        return {"ok": True, "deleted": annotation_id, "affected_tasks": affected_tasks}

    def _tasks_payload(self) -> Dict[str, Any]:
        with self._data_lock:
            active_task = self._settings.get("active_task")
            active_running = active_task if isinstance(active_task, dict) and active_task.get("status") == "running" else None
            active_task_id = active_running.get("task_id") if active_running else None
            stale_changed = False
            for task in self._tasks:
                if task.get("status") == "running" and task.get("id") != active_task_id:
                    task["status"] = "stopped"
                    task["updated_at"] = _now_text()
                    stale_changed = True
            if stale_changed:
                self._save_json("tasks.json", self._tasks)
            tasks = list(self._tasks)
        with self._preflight_lock:
            preflight = self._preflight_with_age_unlocked()
        return {
            "ok": True,
            "tasks": tasks,
            "active_task": active_task,
            "preflight": preflight,
            "last_preflight_ok": bool(preflight and preflight.get("ok")),
        }

    def _update_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        task_id = str(payload.get("task_id") or payload.get("id") or "").strip()
        name = str(payload.get("name") or "").strip()
        if not task_id:
            return self._error("缺少任务 id")
        if not name:
            return self._error("任务名称不能为空")
        with self._data_lock:
            task = self._find_by_id(self._tasks, task_id)
            if task is None:
                return self._error("任务不存在")
            task["name"] = name
            task["updated_at"] = _now_text()
            active = self._settings.get("active_task") or {}
            if active.get("task_id") == task_id:
                active["task_name"] = name
                self._settings["active_task"] = active
                self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
            updated = dict(task)
        self._append_event("修改任务名称", {"task_id": task_id, "name": name})
        return {"ok": True, "task": updated}

    def _delete_task(self, task_id: str) -> Dict[str, Any]:
        task_id = str(task_id or "").strip()
        if not task_id:
            return self._error("缺少任务 id")
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") == "running" and active.get("task_id") == task_id:
                return self._error("任务正在执行，请先停止当前任务再删除")
            before = len(self._tasks)
            self._tasks = [item for item in self._tasks if item.get("id") != task_id]
            if len(self._tasks) == before:
                return self._error("任务不存在")
            if active.get("task_id") == task_id:
                self._settings["active_task"] = None
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
        self._append_event("复位导航会话", {"reason": reason, "clear_costmaps": clear_costmaps})

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
        payload = {
            "phase": "idle",
            "reason": reason,
            "updated_at": _now_text(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.active_waypoint_pub.publish(msg)

    def _handle_navigation_status_for_task(self, status_text: str) -> None:
        status_text = str(status_text or "").strip()
        if not status_text.startswith("error "):
            return
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            if active.get("status") != "running":
                return
            task_id = active.get("task_id")
            self._mark_task_status(task_id, "error")
            self._settings["active_task"] = None
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        self._reset_navigation_session("navigation_error", clear_costmaps=True)
        self._append_event("前端任务因导航错误停止", {"task_id": task_id, "status": status_text})

    def _create_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        annotation_ids = payload.get("annotation_ids") or []
        annotation_ids = [str(item) for item in annotation_ids if str(item).strip()]
        if not annotation_ids:
            return self._error("任务至少需要一个点位")
        with self._data_lock:
            known = {item["id"]: item for item in self._annotations}
            missing = [item for item in annotation_ids if item not in known]
            if missing:
                return self._error("任务中存在已删除的点位", {"missing": missing})
            validation_error = self._validate_task_annotation_order(
                [known[item] for item in annotation_ids]
            )
            if validation_error:
                return validation_error
            task = {
                "id": _new_id("task"),
                "name": str(payload.get("name") or "巡检任务").strip(),
                "map_id": str(payload.get("map_id") or "").strip() or self._settings.get("selected_map_id"),
                "annotation_ids": annotation_ids,
                "status": "ready",
                "created_at": _now_text(),
            }
            self._tasks.append(task)
            self._save_json("tasks.json", self._tasks)
        return {"ok": True, "task": task}

    def _start_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        task_id = str(payload.get("task_id") or "").strip()
        first_annotation = None
        with self._data_lock:
            current_active = self._settings.get("active_task") or {}
            if current_active.get("status") == "running":
                return self._error("已有任务正在执行，请先停止当前任务")
            task = self._find_by_id(self._tasks, task_id)
            if task is None:
                return self._error("任务不存在")
            if task.get("status") == "invalid":
                return self._error("任务点位已失效，请重新生成任务")
            if not task.get("annotation_ids"):
                return self._error("任务没有点位")
            known = {item["id"]: item for item in self._annotations}
            missing = [item for item in task.get("annotation_ids") or [] if item not in known]
            if missing:
                task["status"] = "invalid"
                task["updated_at"] = _now_text()
                self._save_json("tasks.json", self._tasks)
                return self._error("任务中存在已删除的点位，请重新生成任务", {"missing": missing})
            validation_error = self._validate_task_annotation_order(
                [known[item] for item in task.get("annotation_ids") or []]
            )
            if validation_error:
                return validation_error
            selected_map_id = self._settings.get("selected_map_id") or "live_map"
            task_map_id = str(task.get("map_id") or "").strip() or selected_map_id
            if task_map_id != selected_map_id:
                return self._error(
                    "当前地图与任务地图不一致，请先切换到任务对应地图",
                    {"task_map_id": task_map_id, "selected_map_id": selected_map_id},
                )
            first_annotation = known.get((task.get("annotation_ids") or [""])[0])
        ready_error = self._validate_task_start_readiness(first_annotation, task_map_id)
        if ready_error:
            return ready_error
        self._reset_navigation_session("before_start_task", clear_costmaps=True)
        settle_s = max(0.0, float(self.get_parameter("task_start_settle_s").value))
        if settle_s > 0.0:
            time.sleep(min(settle_s, 2.0))
        with self._data_lock:
            current_active = self._settings.get("active_task") or {}
            if current_active.get("status") == "running":
                return self._error("已有任务正在执行，请先停止当前任务")
            task = self._find_by_id(self._tasks, task_id)
            if task is None:
                return self._error("任务不存在")
            selected_map_id = self._settings.get("selected_map_id") or "live_map"
            task_map_id = str(task.get("map_id") or "").strip() or selected_map_id
            active = {
                "task_id": task["id"],
                "task_name": task.get("name"),
                "map_id": task_map_id,
                "status": "running",
                "index": 0,
                "annotation_ids": list(task.get("annotation_ids") or []),
                "started_at": _now_text(),
                "last_goal_annotation_id": None,
                "last_goal_sent_monotonic": 0.0,
                "phase": "navigating",
            }
            self._settings["active_task"] = active
            task["status"] = "running"
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        self._dispatch_active_goal(force=True)
        self._append_event("启动前端任务", {"task_id": task_id})
        with self._data_lock:
            return {
                "ok": True,
                "active_task": self._settings.get("active_task"),
            }

    def _stop_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        reason = str(payload.get("reason") or "web_manual_stop").strip() or "web_manual_stop"
        stopped_task_id = None
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            stopped_task_id = active.get("task_id")
            if stopped_task_id:
                self._mark_task_status(stopped_task_id, "stopped")
            self._settings["active_task"] = None
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        self._reset_navigation_session(reason, clear_costmaps=True)
        self._append_event("停止前端任务", {"task_id": stopped_task_id, "reason": reason})
        return {
            "ok": True,
            "active_task": None,
            "stopped_task_id": stopped_task_id,
            "message": "已发送停止/复位指令",
        }

    def _publish_initialpose(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") == "running":
                return self._error("任务执行中不能重定位，请先停止当前任务")
        request_started_at = time.time()
        try:
            x = float(payload.get("x"))
            y = float(payload.get("y"))
            z = float(payload.get("z", 0.0))
            yaw = float(payload.get("yaw", 0.0))
        except (TypeError, ValueError):
            return self._error("重定位坐标无效，请先在地图上拖箭头")
        frame_id = str(payload.get("frame_id") or "map").strip() or "map"
        floor = str(payload.get("floor") or "").strip()
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
        factory_initialpose = self._publish_initialpose_on_factory_host(
            x,
            y,
            z,
            yaw,
            frame_id,
            xy_cov,
            yaw_cov,
            repeats,
            interval_s,
        )
        verification = self._wait_for_relocalization_verification(
            request_started_at,
            {"x": x, "y": y, "z": z, "yaw": yaw},
        )
        result = {
            "ok": True,
            "navigation_ready": bool(verification.get("navigation_ready")),
            "message": verification.get("message", "已发布网页重定位请求"),
            "topic": str(self.get_parameter("initialpose_topic").value),
            "publish_repeats": repeats,
            "factory_initialpose": factory_initialpose,
            "frame_id": frame_id,
            "floor": floor,
            "pose": {"x": x, "y": y, "z": z, "yaw": yaw},
            "verification": verification,
        }
        if (
            factory_initialpose.get("enabled")
            and factory_initialpose.get("ok") is False
            and not verification.get("factory_pose_accepted")
        ):
            result["message"] = "106 /initialpose 远程发布失败；请先配置 104 到 106 SSH 免密，或临时回到 106 RViz 执行 2D Pose Estimate"
        self._append_event("网页发布重定位", result)
        return result

    def _publish_initialpose_on_factory_host(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        frame_id: str,
        xy_cov: float,
        yaw_cov: float,
        repeats: int,
        interval_s: float,
    ) -> Dict[str, Any]:
        if not bool(self.get_parameter("factory_initialpose_remote_publish").value):
            return {
                "enabled": False,
                "ok": None,
                "message": "106 remote /initialpose publish disabled",
            }
        factory_host = str(self.get_parameter("factory_host").value).strip()
        factory_user = str(self.get_parameter("factory_user").value).strip()
        if factory_host in ("", "localhost", "127.0.0.1"):
            return {
                "enabled": False,
                "ok": None,
                "message": "factory host is local/empty; only local /initialpose was published",
            }

        topic = str(self.get_parameter("factory_initialpose_topic").value).strip() or "/initialpose"
        source_command = str(self.get_parameter("factory_initialpose_source_command").value).strip()
        timeout_s = max(2.0, float(self.get_parameter("factory_initialpose_command_timeout_s").value))
        sleep_s = max(0.0, float(interval_s))
        count = max(1, int(repeats))
        py_code = "\n".join([
            "import math",
            "import time",
            "",
            "import rclpy",
            "from geometry_msgs.msg import PoseWithCovarianceStamped",
            "",
            "rclpy.init(args=None)",
            'node = rclpy.create_node("m20pro_factory_initialpose_once")',
            f"pub = node.create_publisher(PoseWithCovarianceStamped, {topic!r}, 10)",
            "deadline = time.monotonic() + 5.0",
            "subscriptions = 0",
            "while time.monotonic() < deadline:",
            "    subscriptions = pub.get_subscription_count()",
            "    if subscriptions > 0:",
            "        break",
            "    rclpy.spin_once(node, timeout_sec=0.1)",
            "    time.sleep(0.1)",
            "print(f'subscriptions={subscriptions}', flush=True)",
            f"for _ in range({count!r}):",
            "    msg = PoseWithCovarianceStamped()",
            "    msg.header.stamp = node.get_clock().now().to_msg()",
            f"    msg.header.frame_id = {frame_id!r}",
            f"    msg.pose.pose.position.x = {float(x)!r}",
            f"    msg.pose.pose.position.y = {float(y)!r}",
            f"    msg.pose.pose.position.z = {float(z)!r}",
            f"    msg.pose.pose.orientation.z = math.sin({float(yaw)!r} * 0.5)",
            f"    msg.pose.pose.orientation.w = math.cos({float(yaw)!r} * 0.5)",
            f"    msg.pose.covariance[0] = {float(xy_cov)!r}",
            f"    msg.pose.covariance[7] = {float(xy_cov)!r}",
            f"    msg.pose.covariance[35] = {float(yaw_cov)!r}",
            "    pub.publish(msg)",
            "    rclpy.spin_once(node, timeout_sec=0.05)",
            f"    time.sleep({sleep_s!r})",
            "node.destroy_node()",
            "rclpy.shutdown()",
            "",
        ])
        python_command = f"python3 -c {shlex.quote(py_code)}"
        payload = f"{{ {source_command}; }} && {python_command}" if source_command else python_command
        remote_command = f"bash -lc {shlex.quote(payload)}"
        target = f"{factory_user}@{factory_host}" if factory_user else factory_host
        ssh_options, used_identity, used_known_hosts = self._factory_ssh_file_options(5)
        command = [
            "ssh",
            *ssh_options,
            target,
            remote_command,
        ]
        try:
            result = subprocess.run(
                command,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=timeout_s,
                check=False,
            )
        except Exception as exc:
            return {
                "enabled": True,
                "ok": False,
                "host": factory_host,
                "topic": topic,
                "ssh_identity_file": used_identity,
                "ssh_known_hosts_file": used_known_hosts,
                "message": "106 /initialpose remote publish failed before command completed",
                "error": str(exc),
            }

        output = result.stdout or ""
        subscription_match = re.search(r"subscriptions=(\d+)", output)
        subscriptions = int(subscription_match.group(1)) if subscription_match else None
        command_ok = result.returncode == 0
        subscriber_ok = subscriptions is None or subscriptions > 0
        ok = command_ok and subscriber_ok
        return {
            "enabled": True,
            "ok": ok,
            "host": factory_host,
            "topic": topic,
            "returncode": result.returncode,
            "ssh_identity_file": used_identity,
            "ssh_known_hosts_file": used_known_hosts,
            "subscriptions": subscriptions,
            "output": output[-2000:],
            "message": (
                "106 /initialpose remote publish completed"
                if ok
                else "106 /initialpose has no subscribers"
                if command_ok and subscriptions == 0
                else "106 /initialpose remote publish command failed"
            ),
        }

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
        accepted = False
        result_text = ""
        result_age_ok = False
        pose_ok = False
        pose_near_request = False
        localization_ok = False
        scan_ok = False
        local_costmap_ok = False
        global_costmap_ok = False
        pose_error_m = None
        yaw_error_rad = None

        while time.time() < deadline:
            with self._lock:
                relocalization = dict(self._state.get("relocalization_result") or {})
                pose = dict(self._state.get("pose") or {})
                localization = self._state.get("localization_ok")
                scan = dict(self._state.get("scan") or {})
                local_costmap = dict(self._state.get("local_costmap") or {})
                global_costmap = dict(self._state.get("global_costmap") or {})

            result_age_ok = float(relocalization.get("last_update", 0.0) or 0.0) >= request_started_at
            if result_age_ok:
                result_text = str(relocalization.get("raw") or "")
                accepted = result_text.startswith("success")

            pose_update = float(pose.get("last_update", pose.get("stamp", 0.0)) or 0.0)
            pose_ok = pose_update >= request_started_at and _is_plausible_pose_dict(pose)
            if pose_ok:
                try:
                    pose_error_m = math.hypot(
                        float(pose.get("x", 0.0)) - float(requested_pose.get("x", 0.0)),
                        float(pose.get("y", 0.0)) - float(requested_pose.get("y", 0.0)),
                    )
                    yaw_error_rad = abs(
                        _wrap_angle(
                            float(pose.get("yaw", 0.0)) - float(requested_pose.get("yaw", 0.0))
                        )
                    )
                    pose_near_request = pose_error_m <= pose_tolerance_m
                except (TypeError, ValueError):
                    pose_near_request = False
            localization_ok = localization is True
            scan_ok = (
                float(scan.get("last_update", 0.0) or 0.0) >= request_started_at
                and int(scan.get("finite_ranges", 0) or 0) > 0
            )
            local_costmap_ok = (
                float(local_costmap.get("last_update", 0.0) or 0.0) >= request_started_at
                and bool(local_costmap.get("width"))
                and bool(local_costmap.get("height"))
            )
            global_costmap_ok = (
                float(global_costmap.get("last_update", 0.0) or 0.0) >= request_started_at
                and bool(global_costmap.get("width"))
                and bool(global_costmap.get("height"))
            )

            if localization_ok and pose_ok and pose_near_request:
                break
            time.sleep(0.2)

        factory_pose_accepted = localization_ok and pose_ok and pose_near_request
        checks = {
            "initialpose_published": "ok",
            "factory_initialpose": "ok" if factory_pose_accepted else "warn",
            "tcp_2101_diagnostic": (
                "ok"
                if accepted
                else ("warn" if result_text.startswith("failed:") or not result_text else "warn")
            ),
            "localization": "ok" if localization_ok else "warn",
            "map_pose": "ok" if pose_ok else "warn",
            "pose_near_request": "ok" if pose_near_request else "warn",
            "scan": "ok" if scan_ok else "warn",
            "local_costmap": "ok" if local_costmap_ok else "warn",
            "global_costmap": "ok" if global_costmap_ok else "warn",
        }
        required_checks = (
            checks["factory_initialpose"],
            checks["localization"],
            checks["map_pose"],
            checks["pose_near_request"],
            checks["scan"],
            checks["local_costmap"],
            checks["global_costmap"],
        )
        navigation_ready = all(value == "ok" for value in required_checks)
        vendor_failed = result_age_ok and result_text.startswith("failed:")
        if navigation_ready:
            message = "重定位已生效，导航链路已恢复"
        elif factory_pose_accepted:
            message = "原厂定位位姿已更新，但导航链路尚未全部恢复，请看 verification 检查项"
        elif accepted:
            message = "103 TCP 诊断路径接受了重定位请求，但还未看到 106 地图位姿更新"
        elif vendor_failed:
            message = "已发布 /initialpose，但未看到 106 原厂定位更新；103 TCP 诊断失败不作为网页重定位成败依据"
        else:
            message = "已发布 /initialpose；暂未确认 106 原厂定位生效，请继续按地图和激光轮廓调整后重试"
        return {
            "request_accepted": bool(factory_pose_accepted),
            "initialpose_published": True,
            "tcp_2101_accepted": accepted,
            "tcp_2101_diagnostic_only": True,
            "factory_pose_accepted": factory_pose_accepted,
            "navigation_ready": navigation_ready,
            "message": message,
            "result": (
                result_text
                or "未收到 103 TCP /m20pro_tcp_bridge/relocalization_result；网页重定位以 106 /initialpose 后的定位状态为准"
            ),
            "pose_error_m": pose_error_m,
            "yaw_error_rad": yaw_error_rad,
            "pose_tolerance_m": pose_tolerance_m,
            "checks": checks,
            "timeout_s": timeout_s,
        }

    def _tick_active_task(self) -> None:
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
        if active.get("status") != "running":
            return
        annotation = self._active_annotation(active)
        if annotation is None:
            return
        if active.get("phase") == "dwelling":
            until = float(active.get("dwell_until", 0.0) or 0.0)
            if time.time() < until:
                self._publish_active_waypoint(annotation, active, "dwelling")
                return
            self._advance_active_task(annotation)
            return
        with self._lock:
            pose = dict(self._state.get("pose") or {})
            current_floor = self._state.get("floor")
        if not pose:
            self._dispatch_active_goal(force=False)
            return
        if current_floor and annotation.get("floor") and current_floor != annotation.get("floor"):
            self._dispatch_active_goal(force=False)
            return
        target = annotation.get("pose") or {}
        try:
            distance = math.hypot(float(pose["x"]) - float(target["x"]), float(pose["y"]) - float(target["y"]))
        except (KeyError, TypeError, ValueError):
            return
        if distance <= float(self.get_parameter("goal_reached_tolerance_m").value):
            self._reset_navigation_session("waypoint_reached", clear_costmaps=False, publish_idle=False)
            dwell_s = self._annotation_dwell_s(annotation)
            if dwell_s > 0.0:
                with self._data_lock:
                    active = self._settings.get("active_task") or {}
                    if active.get("status") != "running":
                        return
                    active["phase"] = "dwelling"
                    active["dwell_s"] = dwell_s
                    active["dwell_until"] = time.time() + dwell_s
                    active["last_reached_at"] = _now_text()
                    active["last_reached_annotation_id"] = annotation.get("id")
                    self._settings["active_task"] = active
                    self._save_json("settings.json", self._settings)
                self._append_event(
                    "到达点位并开始停留",
                    {
                        "annotation_id": annotation.get("id"),
                        "label": annotation.get("label"),
                        "dwell_s": dwell_s,
                    },
                )
                self._publish_active_waypoint(annotation, active, "dwelling")
                return
            self._advance_active_task(annotation)
        else:
            self._dispatch_active_goal(force=False)

    def _advance_active_task(self, annotation: Dict[str, Any]) -> None:
        completed_task_id = None
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") != "running":
                return
            active["index"] = int(active.get("index", 0)) + 1
            active["phase"] = "navigating"
            active["dwell_s"] = 0.0
            active["dwell_until"] = None
            active["last_reached_at"] = _now_text()
            active["last_reached_annotation_id"] = annotation.get("id")
            if active["index"] >= len(active.get("annotation_ids") or []):
                active["status"] = "completed"
                self._mark_task_status(active.get("task_id"), "completed")
                completed_task_id = active.get("task_id")
            else:
                active["last_goal_annotation_id"] = None
            self._settings["active_task"] = None if completed_task_id else active
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        if completed_task_id:
            self._reset_navigation_session("task_completed", clear_costmaps=True)
            self._append_event("前端任务完成", {"task_id": completed_task_id})
        self._dispatch_active_goal(force=True)

    def _dispatch_active_goal(self, force: bool) -> None:
        with self._data_lock:
            active = self._settings.get("active_task") or {}
        if active.get("status") != "running":
            return
        annotation = self._active_annotation(active)
        if annotation is None:
            return
        now_monotonic = time.monotonic()
        if not force and active.get("last_goal_annotation_id") == annotation.get("id"):
            last_sent = float(active.get("last_goal_sent_monotonic", 0.0) or 0.0)
            resend_interval = max(1.0, float(self.get_parameter("task_goal_resend_interval_s").value))
            if now_monotonic - last_sent < resend_interval:
                return
        pose = annotation.get("pose") or {}
        try:
            self._publish_floor_goal(
                str(annotation.get("floor") or ""),
                float(pose.get("x")),
                float(pose.get("y")),
                float(pose.get("yaw", 0.0)),
                float(pose.get("z", 0.0)),
            )
        except (TypeError, ValueError):
            return
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") != "running" or active.get("task_id") is None:
                return
            active["last_goal_annotation_id"] = annotation.get("id")
            active["last_goal_label"] = annotation.get("label")
            active["last_goal_sent_at"] = _now_text()
            active["last_goal_sent_monotonic"] = now_monotonic
            active["phase"] = "navigating"
            active["last_goal_semantics"] = self._annotation_semantics_payload(annotation)
            self._settings["active_task"] = active
            self._save_json("settings.json", self._settings)
        self._publish_active_waypoint(annotation, active, "navigating")

    def _publish_floor_goal(self, floor: str, x: float, y: float, yaw: float, z: float = 0.0) -> None:
        if not floor:
            raise ValueError("floor is required")
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = floor
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        _yaw_to_orientation(msg, yaw)
        self.floor_goal_pub.publish(msg)
        self.get_logger().info("web task published floor goal floor=%s x=%.2f y=%.2f yaw=%.2f" % (floor, x, y, yaw))

    def _publish_active_waypoint(
        self,
        annotation: Dict[str, Any],
        active: Dict[str, Any],
        phase: str,
    ) -> None:
        payload = {
            "task_id": active.get("task_id"),
            "task_name": active.get("task_name"),
            "phase": phase,
            "index": int(active.get("index", 0)),
            "remaining_dwell_s": self._remaining_dwell_s(active),
            "waypoint": self._annotation_semantics_payload(annotation),
            "updated_at": _now_text(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.active_waypoint_pub.publish(msg)

    def _annotation_semantics_payload(self, annotation: Dict[str, Any]) -> Dict[str, Any]:
        self._normalize_annotation_semantics(annotation)
        pose = annotation.get("pose") or {}
        vendor = dict(annotation.get("vendor_navigation") or {})
        try:
            vendor["PosX"] = float(pose.get("x", 0.0))
            vendor["PosY"] = float(pose.get("y", 0.0))
            vendor["PosZ"] = float(pose.get("z", 0.0))
            vendor["AngleYaw"] = float(pose.get("yaw", 0.0))
        except (TypeError, ValueError):
            pass
        return {
            "id": annotation.get("id"),
            "label": annotation.get("label"),
            "area": annotation.get("area"),
            "room": annotation.get("room"),
            "result_file_prefix": annotation.get("result_file_prefix"),
            "floor": annotation.get("floor"),
            "type": annotation.get("type"),
            "manual_point_type": annotation.get("manual_point_type"),
            "manual_point_type_label": MANUAL_POINT_TYPES[annotation["manual_point_type"]]["label"],
            "pose": dict(pose),
            "yaw": float(pose.get("yaw", 0.0) or 0.0),
            "dwell_s": self._annotation_dwell_s(annotation),
            "camera": annotation.get("camera"),
            "target_classes": list(annotation.get("target_classes") or []),
            "vendor_navigation": vendor,
        }

    def _annotation_dwell_s(self, annotation: Dict[str, Any]) -> float:
        self._normalize_annotation_semantics(annotation)
        try:
            return max(0.0, float(annotation.get("dwell_s", 0.0)))
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _remaining_dwell_s(active: Dict[str, Any]) -> float:
        if active.get("phase") != "dwelling":
            return 0.0
        try:
            return max(0.0, float(active.get("dwell_until", 0.0)) - time.time())
        except (TypeError, ValueError):
            return 0.0

    def _validate_task_annotation_order(
        self,
        annotations: List[Dict[str, Any]],
    ) -> Optional[Dict[str, Any]]:
        for index, annotation in enumerate(annotations):
            self._normalize_annotation_semantics(annotation)
            if annotation.get("manual_point_type") == "charge" and index != len(annotations) - 1:
                return self._error(
                    "充电点必须放在任务最后。开发手册说明充电点到达后会自动进入充电并保持，不能继续串后续点位。",
                    {"annotation_id": annotation.get("id"), "label": annotation.get("label")},
                )
        return None

    def _validate_task_start_readiness(
        self,
        first_annotation: Optional[Dict[str, Any]],
        task_map_id: str,
    ) -> Optional[Dict[str, Any]]:
        if first_annotation is None:
            return self._error("任务首个点位不存在，请重新生成任务")
        with self._lock:
            pose = dict(self._state.get("pose") or {})
            localization_ok = self._state.get("localization_ok")
            current_floor = self._state.get("floor")
            live_map = dict(self._state.get("map") or {})
        now = time.time()
        pose_age = None
        if pose.get("last_update") is not None:
            try:
                pose_age = max(0.0, now - float(pose.get("last_update")))
            except (TypeError, ValueError):
                pose_age = None
        pose_timeout_s = max(0.5, float(self.get_parameter("task_start_pose_timeout_s").value))
        if bool(self.get_parameter("task_start_require_localization_ok").value) and localization_ok is not True:
            return self._error(
                "定位未确认，先在网页定位页完成重定位，再开始任务",
                {
                    "localization_ok": localization_ok,
                    "first_waypoint": first_annotation.get("label") or first_annotation.get("id"),
                },
            )
        if not _is_plausible_pose_dict(pose) or pose_age is None or pose_age > pose_timeout_s:
            return self._error(
                "地图位姿无效或已过期，先重定位并确认机器人位置稳定",
                {
                    "pose_age_sec": pose_age,
                    "pose_timeout_s": pose_timeout_s,
                    "pose": pose,
                },
            )
        target_floor = str(first_annotation.get("floor") or "").strip()
        if (
            bool(self.get_parameter("task_start_require_current_floor_match").value)
            and current_floor
            and target_floor
            and current_floor != target_floor
        ):
            return self._error(
                "当前楼层与任务首点楼层不一致，请先切换/确认地图和楼层",
                {"current_floor": current_floor, "target_floor": target_floor},
            )
        if bool(self.get_parameter("task_start_require_pose_on_map").value):
            robot_map_payload = live_map
            pose_error = self._pose_map_bounds_error(pose, robot_map_payload, "机器人当前位置")
            if pose_error:
                return pose_error
            target_map_payload = live_map
            if task_map_id and task_map_id != "live_map":
                target_map_payload = self._map_file_snapshot(task_map_id)
            target_pose = first_annotation.get("pose") or {}
            target_error = self._pose_map_bounds_error(target_pose, target_map_payload, "任务首点")
            if target_error:
                return target_error
            if current_floor and target_floor and current_floor == target_floor and task_map_id != "live_map":
                metadata_error = self._map_metadata_mismatch_error(live_map, target_map_payload)
                if metadata_error:
                    return metadata_error
        return None

    def _pose_map_bounds_error(
        self,
        pose: Dict[str, Any],
        map_payload: Dict[str, Any],
        label: str,
    ) -> Optional[Dict[str, Any]]:
        if not isinstance(map_payload, dict) or not map_payload.get("available"):
            return self._error(
                "当前地图不可用，不能开始任务",
                {"label": label, "map_message": map_payload.get("message") if isinstance(map_payload, dict) else None},
            )
        try:
            width = int(map_payload.get("width"))
            height = int(map_payload.get("height"))
            resolution = float(map_payload.get("resolution"))
            origin = map_payload.get("origin") or {}
            x = float(pose.get("x"))
            y = float(pose.get("y"))
            ox = float(origin.get("x", 0.0))
            oy = float(origin.get("y", 0.0))
        except (TypeError, ValueError):
            return self._error("地图或位姿数据无效，不能开始任务", {"label": label})
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return self._error("地图尺寸无效，不能开始任务", {"label": label})
        mx = (x - ox) / resolution
        my = (y - oy) / resolution
        if mx < 0.0 or my < 0.0 or mx >= float(width) or my >= float(height):
            return self._error(
                f"{label}不在当前地图范围内，请确认地图和重定位结果",
                {
                    "label": label,
                    "x": x,
                    "y": y,
                    "map_width": width,
                    "map_height": height,
                    "map_resolution": resolution,
                    "map_origin": origin,
                },
            )
        return None

    def _map_metadata_mismatch_error(
        self,
        live_map: Dict[str, Any],
        selected_map: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        if not isinstance(live_map, dict) or not live_map.get("available"):
            return self._error("Nav2 当前 /map 不可用，不能开始任务")
        if not isinstance(selected_map, dict) or not selected_map.get("available"):
            return self._error("当前选择的任务地图不可用，不能开始任务")
        try:
            live_origin = live_map.get("origin") or {}
            selected_origin = selected_map.get("origin") or {}
            checks = {
                "width": int(live_map.get("width")) == int(selected_map.get("width")),
                "height": int(live_map.get("height")) == int(selected_map.get("height")),
                "resolution": abs(float(live_map.get("resolution")) - float(selected_map.get("resolution"))) < 1e-6,
                "origin_x": abs(float(live_origin.get("x", 0.0)) - float(selected_origin.get("x", 0.0))) < 1e-4,
                "origin_y": abs(float(live_origin.get("y", 0.0)) - float(selected_origin.get("y", 0.0))) < 1e-4,
            }
        except (TypeError, ValueError):
            return self._error("地图元数据无效，不能开始任务")
        if all(checks.values()):
            return None
        return self._error(
            "网页选择地图与 Nav2 当前加载地图不一致，请先切换到正确地图并重定位",
            {
                "checks": checks,
                "live_map": {
                    "width": live_map.get("width"),
                    "height": live_map.get("height"),
                    "resolution": live_map.get("resolution"),
                    "origin": live_map.get("origin"),
                },
                "selected_map": {
                    "map_id": selected_map.get("map_id"),
                    "name": selected_map.get("name"),
                    "floor": selected_map.get("floor"),
                    "width": selected_map.get("width"),
                    "height": selected_map.get("height"),
                    "resolution": selected_map.get("resolution"),
                    "origin": selected_map.get("origin"),
                },
            },
        )

    def _active_annotation(self, active: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        ids = active.get("annotation_ids") or []
        index = int(active.get("index", 0))
        if index < 0 or index >= len(ids):
            return None
        with self._data_lock:
            return self._find_by_id(self._annotations, ids[index])

    def _mark_task_status(self, task_id: Optional[str], status: str) -> None:
        if not task_id:
            return
        task = self._find_by_id(self._tasks, task_id)
        if task is not None:
            task["status"] = status
            task["updated_at"] = _now_text()

    def _find_project(self, name: str, building: str) -> Optional[Dict[str, Any]]:
        for item in self._projects:
            if item.get("name") == name and item.get("building", "") == building:
                return item
        return None

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
        return {
            "session_id": str(session.get("id", "")),
            "project_name": str(session.get("project_name", "")),
            "building": str(session.get("building", "")),
            "mode": str(session.get("mode", "")),
            "active_floor": str(session.get("active_floor", "")),
            "map_name": _sanitize_name(str(session.get("map_name") or ""), str(session.get("id", "map"))),
            "floors": ",".join(str(item) for item in session.get("floors") or []),
            "factory_host": str(self.get_parameter("factory_host").value),
            "factory_user": str(self.get_parameter("factory_user").value),
            "factory_active_map": str(self.get_parameter("factory_active_map").value),
            "map_archive_dir": str(self.map_archive_dir),
        }

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

    def _find_map_yaml(self, directory: FsPath) -> Optional[FsPath]:
        preferred = ["occ_grid.yaml", "map.yaml", "jueying.yaml"]
        for name in preferred:
            candidate = directory / name
            if candidate.exists():
                return candidate
        for candidate in sorted(directory.rglob("*.yaml")):
            return candidate
        return None

    def _map_file_snapshot(self, map_id: Optional[str]) -> Dict[str, Any]:
        with self._data_lock:
            if not map_id:
                map_id = self._settings.get("selected_map_id")
            record = self._find_map_record_unlocked(map_id)
        if record is None:
            return {"available": False, "message": "map not selected"}
        yaml_path = FsPath(self._resolve_path(str(record.get("yaml_path") or "")))
        try:
            return self._load_map_file_payload(record, yaml_path)
        except Exception as exc:
            return {
                "available": False,
                "map_id": map_id,
                "message": str(exc),
            }

    def _load_map_file_payload(self, record: Dict[str, Any], yaml_path: FsPath) -> Dict[str, Any]:
        if not yaml_path.exists():
            raise FileNotFoundError(str(yaml_path))
        with yaml_path.open("r", encoding="utf-8") as file:
            info = yaml.safe_load(file) or {}
        image_value = str(info.get("image") or "").strip()
        if not image_value:
            raise RuntimeError("map yaml has no image field")
        image_path = FsPath(os.path.expandvars(os.path.expanduser(image_value)))
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        if not image_path.exists():
            fallback = yaml_path.parent / FsPath(image_value).name
            if fallback.exists():
                image_path = fallback
        width, height, max_value, pixels = self._read_pgm(image_path)
        resolution = float(info.get("resolution", 0.05))
        origin_raw = info.get("origin") or [0.0, 0.0, 0.0]
        origin = {
            "x": float(origin_raw[0]) if len(origin_raw) > 0 else 0.0,
            "y": float(origin_raw[1]) if len(origin_raw) > 1 else 0.0,
            "z": 0.0,
            "yaw": float(origin_raw[2]) if len(origin_raw) > 2 else 0.0,
            "yaw_deg": math.degrees(float(origin_raw[2]) if len(origin_raw) > 2 else 0.0),
        }
        negate = int(info.get("negate", 0))
        occupied_thresh = float(info.get("occupied_thresh", 0.65))
        free_thresh = float(info.get("free_thresh", 0.196))
        data = [-1] * (width * height)
        max_value = max(1, max_value)
        for y in range(height):
            for x in range(width):
                pixel = pixels[y * width + x] / max_value
                occ = pixel if negate else 1.0 - pixel
                if occ > occupied_thresh:
                    value = 100
                elif occ < free_thresh:
                    value = 0
                else:
                    value = -1
                data[(height - 1 - y) * width + x] = value
        version = int(max(yaml_path.stat().st_mtime, image_path.stat().st_mtime) * 1000)
        return {
            "available": True,
            "source": "file",
            "map_id": record.get("id"),
            "name": record.get("name"),
            "floor": record.get("floor"),
            "source": record.get("source"),
            "version": version,
            "frame_id": "map",
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin": origin,
            "data": data,
        }

    def _read_pgm(self, path: FsPath) -> Tuple[int, int, int, List[int]]:
        with path.open("rb") as file:
            def token() -> bytes:
                chars = bytearray()
                while True:
                    b = file.read(1)
                    if not b:
                        break
                    if b == b"#":
                        file.readline()
                        continue
                    if b.isspace():
                        if chars:
                            break
                        continue
                    chars.extend(b)
                return bytes(chars)

            magic = token()
            if magic not in (b"P5", b"P2"):
                raise RuntimeError(f"unsupported PGM format: {magic!r}")
            width = int(token())
            height = int(token())
            max_value = int(token())
            if magic == b"P5":
                if max_value <= 255:
                    raw = file.read(width * height)
                    pixels = list(raw)
                else:
                    raw = file.read(width * height * 2)
                    pixels = [
                        int.from_bytes(raw[index:index + 2], "big")
                        for index in range(0, len(raw), 2)
                    ]
            else:
                pixels = [int(token()) for _ in range(width * height)]
        if len(pixels) != width * height:
            raise RuntimeError(f"PGM pixel count mismatch: {path}")
        return width, height, max_value, pixels

    def _serve_mjpeg(self, camera_name: str, handler: BaseHTTPRequestHandler) -> None:
        if not self._as_bool(self.get_parameter("enable_camera_proxy").value):
            handler.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "camera proxy disabled")
            return
        if get_cv2() is None:
            detail = _CV2_IMPORT_ERROR or "python3-opencv is not installed"
            handler.send_error(HTTPStatus.SERVICE_UNAVAILABLE, f"OpenCV unavailable: {detail}")
            return

        worker = self._camera_worker(camera_name)
        frame_timeout = max(0.5, float(self.get_parameter("camera_proxy_frame_timeout_s").value))

        handler.send_response(HTTPStatus.OK)
        handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Pragma", "no-cache")
        handler.send_header("Access-Control-Allow-Origin", "*")
        handler.end_headers()

        last_sequence = -1
        try:
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
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self.get_logger().warning(f"{camera_name} camera MJPEG proxy stopped: {exc}")

    def _camera_worker(self, camera_name: str) -> _CameraProxyWorker:
        if camera_name == "rear":
            url = str(self.get_parameter("rear_camera_url").value)
        else:
            camera_name = "front"
            url = str(self.get_parameter("front_camera_url").value)

        worker = self._camera_workers.get(camera_name)
        if worker is not None and worker.url == url:
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
        if isinstance(value, bool):
            return value
        if value is None:
            return False
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    @staticmethod
    def _error(message: str, extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"ok": False, "message": message}
        if extra:
            payload.update(extra)
        return payload

    def _start_http_server(self) -> _ReusableThreadingHTTPServer:
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        node = self

        class DashboardHandler(BaseHTTPRequestHandler):
            def do_OPTIONS(self) -> None:
                self.send_response(HTTPStatus.NO_CONTENT)
                self._send_common_headers("application/json; charset=utf-8", 0)

            def do_GET(self) -> None:
                parsed = urlparse(self.path)
                query = parse_qs(parsed.query)
                if parsed.path == "/":
                    self._send_bytes(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
                elif parsed.path == "/api/state":
                    self._send_json(node._snapshot())
                elif parsed.path == "/api/map":
                    self._send_json(node._map_snapshot())
                elif parsed.path == "/api/map_file":
                    map_id = (query.get("map_id") or [None])[0]
                    self._send_json(node._map_file_snapshot(map_id))
                elif parsed.path == "/api/map_3d":
                    map_id = (query.get("map_id") or [None])[0]
                    self._send_json(node._map_3d_payload(map_id))
                elif parsed.path == "/api/stair_zones":
                    map_id = (query.get("map_id") or [None])[0]
                    self._send_json(node._stair_zones_payload(map_id))
                elif parsed.path == "/api/stair_pointcloud":
                    map_id = (query.get("map_id") or [None])[0]
                    zone_id = (query.get("zone_id") or [None])[0]
                    self._send_json(node._stair_pointcloud_payload(map_id, zone_id))
                elif parsed.path == "/api/projects":
                    self._send_json(node._projects_payload())
                elif parsed.path == "/api/maps":
                    self._send_json(node._maps_payload())
                elif parsed.path == "/api/annotations":
                    self._send_json(node._annotations_payload(query))
                elif parsed.path == "/api/tasks":
                    self._send_json(node._tasks_payload())
                elif parsed.path == "/api/preflight":
                    self._send_json(node._preflight_payload())
                elif parsed.path in ("/camera/front.mjpg", "/camera/rear.mjpg"):
                    camera_name = "front" if parsed.path == "/camera/front.mjpg" else "rear"
                    node._serve_mjpeg(camera_name, self)
                elif parsed.path == "/healthz":
                    self._send_json({"ok": True})
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

            def do_POST(self) -> None:
                parsed = urlparse(self.path)
                payload = self._read_json_body()
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

            def do_DELETE(self) -> None:
                parsed = urlparse(self.path)
                query = parse_qs(parsed.query)
                if parsed.path == "/api/annotations":
                    annotation_id = (query.get("id") or [""])[0]
                    self._send_api(node._delete_annotation(annotation_id))
                elif parsed.path == "/api/tasks":
                    task_id = (query.get("id") or [""])[0]
                    self._send_api(node._delete_task(task_id))
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

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
                self.send_header("Cache-Control", "no-store")
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
