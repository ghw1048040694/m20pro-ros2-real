import json
import math
import os
import re
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
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:  # Foxy does not expose this internal exception module.
    RCLError = Exception

try:
    import cv2
except ImportError:  # pragma: no cover - runtime dependency on the robot/operator PC
    cv2 = None


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
      min-height: calc(100vh - 58px);
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
    .canvas-box {
      position: relative;
      flex: 1;
      min-height: 590px;
      background: #cfd5dc;
    }
    canvas {
      display: block;
      width: 100%;
      height: 100%;
      image-rendering: pixelated;
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
      grid-template-columns: repeat(5, minmax(0, 1fr));
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
      main { grid-template-columns: 1fr; }
      .canvas-box { min-height: 460px; }
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
        <span><strong id="mapTitle">等待地图</strong> <span id="mapMeta">-</span></span>
        <span id="mapMode" class="pill">实时 /map</span>
      </div>
      <div class="canvas-box">
        <canvas id="mapCanvas"></canvas>
        <div id="cursor" class="crosshair">点击地图可取点</div>
      </div>
    </section>
    <aside class="side">
      <nav class="tabs">
        <button class="tab active" data-tab="live">看板</button>
        <button class="tab" data-tab="mapping">建图</button>
        <button class="tab" data-tab="maps">地图</button>
        <button class="tab" data-tab="marks">标点</button>
        <button class="tab" data-tab="tasks">任务</button>
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
              <div class="label">步态指令</div>
              <div id="gait" class="value">-</div>
            </div>
            <div class="tile">
              <div class="label">机器人位姿</div>
              <div id="pose" class="value">-</div>
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
              不选择归档地图时页面显示实时 `/map`；选择归档地图后，可直接在这张图上标点。
            </div>
          </div>
          <div class="section">
            <h2>已归档地图</h2>
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
              <label>楼层</label>
              <input id="markFloor" value="F20" />
            </div>
            <div class="row">
              <label>名称</label>
              <input id="markLabel" placeholder="例如 F20 楼梯上行切换点" />
            </div>
            <div class="row">
              <label>X / Y</label>
              <input id="markXY" placeholder="点击地图自动填入" />
            </div>
            <div class="row">
              <label>Yaw(rad)</label>
              <input id="markYaw" value="0.0" />
            </div>
            <div class="actions">
              <button class="primary" id="saveMarkBtn">保存点位</button>
              <button id="useRobotPoseBtn">使用当前机器人位姿</button>
            </div>
          </div>
          <div class="section">
            <h2>当前地图点位</h2>
            <div id="annotationList" class="list"></div>
          </div>
        </section>

        <section id="tab-tasks" class="panel">
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
              <button class="danger" id="stopTaskBtn" disabled>停止当前任务</button>
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
      clickPose: null
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
    function setLog(id, payload) {
      $(id).textContent = typeof payload === "string" ? payload : JSON.stringify(payload, null, 2);
    }
    function currentAnnotationMapId() {
      return state.selectedMapId || "live_map";
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
      if (!map) return { scale: 1, ox: 0, oy: 0, rect };
      const scale = Math.min(rect.width / map.width, rect.height / map.height);
      const drawW = map.width * scale;
      const drawH = map.height * scale;
      return { scale, ox: (rect.width - drawW) / 2, oy: (rect.height - drawH) / 2, rect };
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
    function drawArrow(pose) {
      const p = worldToCanvas(pose.x, pose.y);
      if (!p) return;
      ctx.save();
      ctx.translate(p.x, p.y);
      ctx.rotate(-pose.yaw);
      ctx.fillStyle = "#0f6bff";
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
      ctx.save();
      ctx.font = "12px system-ui, sans-serif";
      for (const item of state.annotations) {
        const pose = item.pose || {};
        const p = worldToCanvas(Number(pose.x), Number(pose.y));
        if (!p) continue;
        const color = typeColors[item.type] || "#0f6bff";
        ctx.fillStyle = color;
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 7, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = "#17212b";
        const label = item.label || typeNames[item.type] || "point";
        ctx.fillText(label, p.x + 10, p.y - 8);
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
        ctx.fillText("等待地图数据", 20, 30);
        return;
      }
      const view = getView();
      ctx.drawImage(state.mapImage, view.ox, view.oy, state.map.width * view.scale, state.map.height * view.scale);
      ctx.strokeStyle = "#4b5563";
      ctx.lineWidth = 1;
      ctx.strokeRect(view.ox, view.oy, state.map.width * view.scale, state.map.height * view.scale);
      const latest = state.latest;
      if (latest) {
        drawPath(latest.path);
        drawObstacles(latest.dynamic_obstacles);
      }
      drawAnnotations();
      if (latest && latest.pose) drawArrow(latest.pose);
    }
    async function refreshLiveMap(version) {
      if (state.selectedMapId || version === state.liveMapVersion) return;
      const map = await fetchJson("/api/map");
      if (!map.available) return;
      state.map = map;
      state.mapImage = buildMapImage(map);
      state.selectedMapId = null;
      state.liveMapVersion = map.version;
      $("mapTitle").textContent = `实时地图版本 ${map.version}`;
      $("mapMeta").textContent = `${map.width} x ${map.height}, ${map.resolution.toFixed(3)} m/格`;
      $("mapMode").textContent = "实时 /map";
      await loadAnnotations();
    }
    async function loadFileMap(mapId) {
      if (!mapId) {
        state.selectedMapId = null;
        state.fileMapVersion = -1;
        return;
      }
      const map = await fetchJson(`/api/map_file?map_id=${encodeURIComponent(mapId)}`);
      if (!map.available) return;
      state.map = map;
      state.mapImage = buildMapImage(map);
      state.selectedMapId = mapId;
      state.fileMapVersion = map.version;
      $("mapTitle").textContent = map.name || `归档地图 ${mapId}`;
      $("mapMeta").textContent = `${map.floor || "-"} / ${map.width} x ${map.height}, ${map.resolution.toFixed(3)} m/格`;
      $("mapMode").textContent = "归档地图";
      await loadAnnotations();
      draw();
    }
    function updateState(s) {
      state.latest = s;
      $("floor").textContent = text(s.floor);
      $("stair").textContent = text(s.stair_status);
      $("gait").textContent = text(s.gait_command);
      if (s.pose) $("pose").textContent = `x ${fmtNumber(s.pose.x)} / y ${fmtNumber(s.pose.y)} / yaw ${fmtNumber(s.pose.yaw_deg, 0)}°`;
      else $("pose").textContent = "-";
      $("nav").textContent = JSON.stringify({
        路径点数: s.path ? s.path.points.length : 0,
        动态障碍物: s.dynamic_obstacles ? s.dynamic_obstacles.length : 0,
        当前任务: s.active_task || null,
        更新时间: s.node_time_text
      }, null, 2);
      const det = s.detections && (s.detections.parsed || s.detections.raw);
      $("detections").textContent = det ? JSON.stringify(det, null, 2) : "等待数据";
      $("events").textContent = s.events && s.events.length ? JSON.stringify(s.events.slice(-5), null, 2) : "等待数据";
      $("activeTask").textContent = s.active_task ? JSON.stringify(s.active_task, null, 2) : "无任务";
      $("stopTaskBtn").disabled = !(s.active_task && s.active_task.status === "running");
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
      const selected = payload.selected_map_id || "";
      const select = $("mapSelect");
      select.innerHTML = "";
      const live = document.createElement("option");
      live.value = "";
      live.textContent = "实时 /map";
      select.appendChild(live);
      for (const map of state.maps) {
        const opt = document.createElement("option");
        opt.value = map.id;
        opt.textContent = `${map.name || map.id} (${map.floor || "-"})`;
        select.appendChild(opt);
      }
      select.value = selected;
      if (selected && selected !== state.selectedMapId) await loadFileMap(selected);
      renderMapList();
    }
    function renderMapList() {
      const box = $("mapList");
      box.innerHTML = "";
      if (!state.maps.length) {
        box.innerHTML = `<div class="small">还没有从 106 拉取归档地图。</div>`;
        return;
      }
      for (const map of state.maps) {
        const el = document.createElement("div");
        el.className = "item";
        el.innerHTML = `
          <div class="item-head"><span>${map.name || map.id}</span><span class="tag">${map.floor || "-"}</span></div>
          <div class="item-meta">${map.yaml_path || ""}<br>${map.created_at || ""}</div>
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
        const el = document.createElement("div");
        el.className = "item";
        el.innerHTML = `
          <div class="item-head">
            <span>${item.label || typeNames[item.type] || item.id}</span>
            <span class="tag">${typeNames[item.type] || item.type}</span>
          </div>
          <div class="item-meta">${item.floor || "-"} / x ${fmtNumber(Number(pose.x))}, y ${fmtNumber(Number(pose.y))}, yaw ${fmtNumber(Number(pose.yaw), 2)}</div>
          <div class="actions"><button class="danger" data-delete-mark="${item.id}">删除</button></div>
        `;
        box.appendChild(el);
      }
      for (const btn of box.querySelectorAll("[data-delete-mark]")) {
        btn.addEventListener("click", async () => {
          await api("DELETE", `/api/annotations?id=${encodeURIComponent(btn.dataset.deleteMark)}`);
          await loadAnnotations();
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
        line.innerHTML = `<input type="checkbox" value="${item.id}" checked><span>${item.floor || "-"} / ${item.label || item.id} / ${typeNames[item.type] || item.type}</span>`;
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
        const active = payload.active_task && payload.active_task.status === "running";
        const isRunning = task.status === "running";
        const canStart = !active && !isRunning;
        const el = document.createElement("div");
        el.className = "item";
        el.innerHTML = `
          <div class="item-head"><span>${task.name || task.id}</span><span class="tag">${task.status || "ready"}</span></div>
          <div class="item-meta">${(task.annotation_ids || []).length} 个点 / ${task.created_at || ""}</div>
          <div class="actions"><button class="primary" data-start-task="${task.id}" ${canStart ? "" : "disabled"}>${isRunning ? "执行中" : (active ? "先停止当前任务" : "开始执行")}</button></div>
        `;
        box.appendChild(el);
      }
      for (const btn of box.querySelectorAll("[data-start-task]")) {
        btn.addEventListener("click", async () => {
          const payload = await api("POST", "/api/tasks/start", {task_id: btn.dataset.startTask});
          setLog("activeTask", payload.active_task || payload);
          await loadTasks();
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
      });
    }
    canvas.addEventListener("click", (evt) => {
      const p = canvasToWorld(evt.clientX, evt.clientY);
      if (!p) return;
      state.clickPose = p;
      $("markXY").value = `${p.x.toFixed(3)}, ${p.y.toFixed(3)}`;
      $("cursor").textContent = `x ${p.x.toFixed(3)} / y ${p.y.toFixed(3)}`;
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
    $("selectMapBtn").addEventListener("click", async () => {
      try {
        const mapId = $("mapSelect").value;
        await api("POST", "/api/maps/select", {map_id: mapId});
        if (mapId) await loadFileMap(mapId);
        else {
          state.selectedMapId = null;
          state.liveMapVersion = -1;
        }
        await loadAnnotations();
      } catch (err) { console.warn(err); }
    });
    $("reloadMapsBtn").addEventListener("click", loadMaps);
    $("saveMarkBtn").addEventListener("click", async () => {
      try {
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
          pose: {
            x,
            y,
            z: 0,
            yaw: Number.isFinite(yaw) ? yaw : 0
          }
        });
        await loadAnnotations();
        draw();
        $("markLabel").value = "";
        $("cursor").textContent = `已保存 ${payload.annotation.label || payload.annotation.id}`;
      } catch (err) { $("cursor").textContent = err.message || JSON.stringify(err); }
    });
    $("useRobotPoseBtn").addEventListener("click", () => {
      const pose = state.latest && state.latest.pose;
      if (!pose) return;
      $("markXY").value = `${pose.x.toFixed(3)}, ${pose.y.toFixed(3)}`;
      $("markYaw").value = String(pose.yaw.toFixed(4));
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
    $("stopTaskBtn").addEventListener("click", async () => {
      try {
        const payload = await api("POST", "/api/tasks/stop", {});
        setLog("activeTask", payload.active_task || "无任务");
        await loadTasks();
      } catch (err) { setLog("activeTask", err); }
    });
    window.addEventListener("resize", resizeCanvas);
    resizeCanvas();
    loadMaps().then(loadAnnotations).then(loadTasks).catch(console.warn);
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


def _parse_json_text(text: str) -> Any:
    try:
        return json.loads(text)
    except Exception:
        return None


def _now_text() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


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
                if cv2 is None:
                    self._set_error("python3-opencv is not installed")
                    time.sleep(reconnect_s)
                    continue
                if cap is None or not cap.isOpened():
                    cap = self._open_capture()
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
                payload = self._encode_frame(frame)
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

    def _open_capture(self):
        if str(self.node.get_parameter("camera_proxy_transport").value).lower() == "tcp":
            options = str(self.node.get_parameter("camera_proxy_ffmpeg_options").value)
            with self._opencv_env_lock:
                os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = options
                if hasattr(cv2, "CAP_FFMPEG"):
                    cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
                else:
                    cap = cv2.VideoCapture(self.url)
        else:
            cap = cv2.VideoCapture(self.url)

        self._set_capture_property(cap, "CAP_PROP_BUFFERSIZE", 1)
        self._set_capture_property(
            cap,
            "CAP_PROP_OPEN_TIMEOUT_MSEC",
            int(float(self.node.get_parameter("camera_proxy_open_timeout_s").value) * 1000.0),
        )
        self._set_capture_property(
            cap,
            "CAP_PROP_READ_TIMEOUT_MSEC",
            int(float(self.node.get_parameter("camera_proxy_read_timeout_s").value) * 1000.0),
        )
        return cap

    def _set_capture_property(self, cap: Any, name: str, value: float) -> None:
        prop = getattr(cv2, name, None)
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

    def _encode_frame(self, frame: Any) -> Optional[bytes]:
        max_width = int(self.node.get_parameter("camera_proxy_max_width").value)
        if max_width > 0 and hasattr(frame, "shape") and frame.shape[1] > max_width:
            scale = max_width / float(frame.shape[1])
            height = max(1, int(frame.shape[0] * scale))
            frame = cv2.resize(frame, (max_width, height), interpolation=cv2.INTER_AREA)

        quality = max(30, min(95, int(self.node.get_parameter("camera_proxy_jpeg_quality").value)))
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
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
        self._annotations = self._load_json("annotations.json", [])
        self._tasks = self._load_json("tasks.json", [])
        self._sessions = self._load_json("mapping_sessions.json", [])
        self._settings = self._load_json("settings.json", {"selected_map_id": None, "active_task": None})
        self._normalize_runtime_state_on_startup()
        self._mapping_processes: Dict[str, Dict[str, Any]] = {}
        self._camera_workers: Dict[str, _CameraProxyWorker] = {}

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
        self.create_timer(1.0, self._tick_active_task)
        self._server = self._start_http_server()

    def _declare_parameters(self) -> None:
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("data_dir", "~/.m20pro_web")
        self.declare_parameter("map_archive_dir", "~/m20pro_maps")
        self.declare_parameter("factory_host", "10.21.31.106")
        self.declare_parameter("factory_user", "user")
        self.declare_parameter("factory_active_map", "/var/opt/robot/data/maps/active")
        self.declare_parameter(
            "factory_mapping_start_command",
            'ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} '
            '"nohup sudo -n drmap mapping -s -n {map_name} > /tmp/m20pro_drmap_mapping_{session_id}.log 2>&1 &"',
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
        self.declare_parameter("goal_reached_tolerance_m", 0.6)
        self.declare_parameter("floor_goal_topic", "/m20pro/floor_goal")
        self.declare_parameter("stop_task_topic", "/m20pro/stop_task")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
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

    def _normalize_runtime_state_on_startup(self) -> None:
        active = self._settings.get("active_task") or {}
        changed = False
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

        self.create_subscription(String, self._topic("current_floor_topic"), self._on_current_floor, 10)
        self.create_subscription(String, self._topic("stair_status_topic"), self._on_stair_status, 10)
        self.create_subscription(String, self._topic("gait_command_topic"), self._on_gait_command, 10)
        self.create_subscription(PoseStamped, self._topic("pose_topic"), self._on_pose, 20)
        self.create_subscription(RosPath, self._topic("plan_topic"), self._on_path, 5)
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
        with self._data_lock:
            snapshot["selected_map_id"] = self._settings.get("selected_map_id")
            snapshot["active_task"] = self._settings.get("active_task")

        snapshot["ok"] = True
        snapshot["node_time"] = now
        snapshot["node_time_text"] = _now_text()
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
                "maps": list(self._maps),
                "selected_map_id": self._settings.get("selected_map_id"),
            }

    def _select_map(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        map_id = str(payload.get("map_id") or "").strip() or None
        with self._data_lock:
            active = self._settings.get("active_task") or {}
            if active.get("status") == "running":
                return self._error("任务执行中不能切换地图，请先停止当前任务")
            if map_id and not self._find_by_id(self._maps, map_id):
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
        if factory_host in ("", "localhost", "127.0.0.1"):
            prefix = ""
        else:
            prefix = f"ssh -o BatchMode=yes -o ConnectTimeout=8 {factory_user}@{factory_host} "

        remote_probe = (
            "set -e; "
            "echo host=$(hostname); "
            "echo user=$(whoami); "
            "echo drmap=$(command -v drmap); "
            f"echo active=$(readlink -f {active_map} || true); "
            f"test -d {active_map}; "
            "sudo -n drmap mapping -h >/dev/null; "
            "sudo -n drmap stop_mapping -h >/dev/null"
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
            payload["message"] = "106 建图环境可用：SSH、drmap、active map、sudo -n 均通过。"
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

        yaml_path = self._find_map_yaml(dest)
        if yaml_path is None:
            return self._error(
                "地图已拉取，但没有找到 occ_grid.yaml/map.yaml/jueying.yaml",
                {"directory": str(dest), "command": command_text, "output": command_output},
            )

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
        map_id = str(payload.get("map_id") or "").strip() or None
        if map_id == "live_map":
            map_id = "live_map"
        with self._data_lock:
            if map_id and map_id != "live_map" and not self._find_by_id(self._maps, map_id):
                return self._error("地图不存在")
            if not map_id:
                map_id = self._settings.get("selected_map_id")
            if not map_id and self._state.get("map"):
                map_id = "live_map"
            if not map_id:
                return self._error("没有可用地图，请等待实时 /map 或先选择归档地图")
        item = {
            "id": _new_id("point"),
            "map_id": map_id,
            "type": str(payload.get("type") or "patrol").strip(),
            "floor": floor,
            "label": str(payload.get("label") or "").strip(),
            "pose": {"x": x, "y": y, "z": z, "yaw": yaw},
            "created_at": _now_text(),
        }
        if not item["label"]:
            item["label"] = f"{floor}_{item['type']}_{len(self._annotations) + 1}"
        with self._data_lock:
            self._annotations.append(item)
            self._save_json("annotations.json", self._annotations)
        self._append_event("保存地图点位", {"annotation_id": item["id"], "floor": floor, "type": item["type"]})
        return {"ok": True, "annotation": item}

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
            return {"ok": True, "tasks": list(self._tasks), "active_task": self._settings.get("active_task")}

    def _create_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        annotation_ids = payload.get("annotation_ids") or []
        annotation_ids = [str(item) for item in annotation_ids if str(item).strip()]
        if not annotation_ids:
            return self._error("任务至少需要一个点位")
        with self._data_lock:
            known = {item["id"] for item in self._annotations}
            missing = [item for item in annotation_ids if item not in known]
            if missing:
                return self._error("任务中存在已删除的点位", {"missing": missing})
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
            selected_map_id = self._settings.get("selected_map_id") or "live_map"
            task_map_id = str(task.get("map_id") or "").strip() or selected_map_id
            if task_map_id != selected_map_id:
                return self._error(
                    "当前地图与任务地图不一致，请先切换到任务对应地图",
                    {"task_map_id": task_map_id, "selected_map_id": selected_map_id},
                )
            active = {
                "task_id": task["id"],
                "task_name": task.get("name"),
                "map_id": task_map_id,
                "status": "running",
                "index": 0,
                "annotation_ids": list(task.get("annotation_ids") or []),
                "started_at": _now_text(),
                "last_goal_annotation_id": None,
            }
            self._settings["active_task"] = active
            task["status"] = "running"
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        self._dispatch_active_goal(force=True)
        self._append_event("启动前端任务", {"task_id": task_id})
        with self._data_lock:
            return {"ok": True, "active_task": self._settings.get("active_task")}

    def _stop_task(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        reason = str(payload.get("reason") or "web_stop").strip() or "web_stop"
        stopped_task_id = None
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
            stopped_task_id = active.get("task_id")
            if stopped_task_id:
                self._mark_task_status(stopped_task_id, "stopped")
            self._settings["active_task"] = None
            self._save_json("settings.json", self._settings)
            self._save_json("tasks.json", self._tasks)
        msg = String()
        msg.data = reason
        self.stop_task_pub.publish(msg)
        self.cmd_vel_pub.publish(Twist())
        self._append_event("停止前端任务", {"task_id": stopped_task_id, "reason": reason})
        return {"ok": True, "active_task": None, "stopped_task_id": stopped_task_id}

    def _tick_active_task(self) -> None:
        with self._data_lock:
            active = dict(self._settings.get("active_task") or {})
        if active.get("status") != "running":
            return
        annotation = self._active_annotation(active)
        if annotation is None:
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
            completed_task_id = None
            with self._data_lock:
                active = self._settings.get("active_task") or {}
                active["index"] = int(active.get("index", 0)) + 1
                active["last_reached_at"] = _now_text()
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
                self._append_event("前端任务完成", {"task_id": completed_task_id})
            self._dispatch_active_goal(force=True)
        else:
            self._dispatch_active_goal(force=False)

    def _dispatch_active_goal(self, force: bool) -> None:
        with self._data_lock:
            active = self._settings.get("active_task") or {}
        if active.get("status") != "running":
            return
        annotation = self._active_annotation(active)
        if annotation is None:
            return
        if not force and active.get("last_goal_annotation_id") == annotation.get("id"):
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
            active["last_goal_annotation_id"] = annotation.get("id")
            active["last_goal_label"] = annotation.get("label")
            active["last_goal_sent_at"] = _now_text()
            self._settings["active_task"] = active
            self._save_json("settings.json", self._settings)

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
            record = self._find_by_id(self._maps, map_id)
        if record is None:
            return {"available": False, "message": "map not selected"}
        yaml_path = FsPath(record.get("yaml_path") or "")
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
        if cv2 is None:
            handler.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "python3-opencv is not installed")
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
                elif parsed.path == "/api/projects":
                    self._send_json(node._projects_payload())
                elif parsed.path == "/api/maps":
                    self._send_json(node._maps_payload())
                elif parsed.path == "/api/annotations":
                    self._send_json(node._annotations_payload(query))
                elif parsed.path == "/api/tasks":
                    self._send_json(node._tasks_payload())
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
                elif parsed.path == "/api/tasks/start":
                    self._send_api(node._start_task(payload))
                elif parsed.path == "/api/tasks/stop":
                    self._send_api(node._stop_task(payload))
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

            def do_DELETE(self) -> None:
                parsed = urlparse(self.path)
                query = parse_qs(parsed.query)
                if parsed.path == "/api/annotations":
                    annotation_id = (query.get("id") or [""])[0]
                    self._send_api(node._delete_annotation(annotation_id))
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
