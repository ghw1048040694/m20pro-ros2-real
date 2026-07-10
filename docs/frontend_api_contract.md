# M20Pro 前端 API 对接契约

更新时间：2026-07-09 18:50 CST

本文档是新版前端、甲方前端和外部功能包对接 104 Web 后端的接口契约。接口实现位于 `m20pro_cloud_bridge.web_dashboard_node`。以后新增、删除或修改接口字段时，必须同步更新本文档。

## 基本约定

- 默认服务地址：`http://10.21.31.104:8080`
- 请求格式：`Content-Type: application/json; charset=utf-8`
- 响应格式：JSON
- CORS：当前允许 `*`
- 成功响应通常包含：

```json
{"ok": true}
```

- 失败响应通常为 HTTP 400，并包含：

```json
{"ok": false, "message": "错误说明", "code": "optional_code"}
```

- `GET /api/state` 可能返回较大的实时状态；甲方前端常规轮询建议用 `GET /api/state?debug=0`。
- 不要解析网页 DOM，不要读取前端本地变量。外部系统只依赖本文档列出的 API 字段。

## 地图身份模型

地图身份是当前最重要的契约，前端不要自己猜。

| 字段 | 含义 | 使用建议 |
| --- | --- | --- |
| `selected_map_id` | 操作员显式选择的固定地图。选择“实时 /map”时为 `null` | 只用于显示“当前下拉框选择” |
| `working_map_id` | 104/Nav2/任务点真正使用的固定工作地图，跨重启保存 | 用于解释断电重启后加载哪张地图 |
| `effective_map_id` | 后端根据显式选择、实时 `/map` 元数据和工作地图解析出的当前有效地图 | 点位、任务、雷达联动优先使用它 |

规则：

1. 如果显式选择固定地图，`effective_map_id = selected_map_id`。
2. 如果选择“实时 /map”，后端会比较实时 `/map` 与已知固定地图的尺寸、分辨率、origin。
3. 实时 `/map` 与某张固定地图一致时，`effective_map_id` 返回该固定地图 ID。
4. 实时 `/map` 可用但匹配不上任何固定地图时，`effective_map_id = null`，前端不得显示旧点位。
5. 切到“实时 /map”只是显示源变化，不会清除 `working_map_id`，也不会清除 `map_relocalization_required`。

甲方前端最推荐的做法：

```text
启动页面 -> GET /api/state
拿到 effective_map_id
用 /api/annotations?map_id=live_map 或 /api/annotations?map_id=<effective_map_id> 取点位
用 /api/tasks?map_id=live_map 或 /api/tasks?map_id=<effective_map_id> 取任务
```

## 状态接口

### GET `/healthz`

健康检查。

响应：

```json
{"ok": true}
```

### GET `/api/state`

读取机器人、地图、定位、任务、感知、视频代理等实时状态。

查询参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `debug` | `1` | `0` 时不返回事件和 topic 明细，适合高频轮询 |

稳定字段：

| 字段 | 说明 |
| --- | --- |
| `ok` | 固定为 `true` |
| `node_time` / `node_time_text` | 104 Web 节点时间 |
| `selected_map_id` | 显式选中的固定地图，实时显示时为 `null` |
| `working_map_id` | 当前工作地图 |
| `effective_map_id` | 当前有效地图 |
| `selected_map_status` | 当前有效地图与 Nav2 `/map` 是否一致 |
| `map_relocalization_required` | 切图后是否必须重新 2101 重定位 |
| `localization_status` | 重定位是否真正确认 |
| `pose` | 当前地图坐标系下机器人位姿 |
| `pose_fresh` / `pose_age_sec` | 位姿是否新鲜 |
| `floor` | 当前楼层 |
| `scan` | `/scan` 摘要和前端激光轮廓点 |
| `path` | 当前规划路径，用于画导航线 |
| `active_task` | 当前正在执行的任务状态，空闲时为 `null` |
| `active_waypoint` | 当前任务点 JSON 字符串和解析结果 |
| `detections` | YOLO 检测结果，来自 `/m20pro_yolov8_inspection/detections` |
| `inspection_status` | YOLO 检测节点运行状态，来自 `/m20pro_yolov8_inspection/status` |
| `camera_proxy` | 前后相机代理状态 |
| `battery` | 电量显示信息，只用于展示，不作为软件阻断条件 |
| `preflight` | 最近一次自检结果 |

定位成功判断：

```text
localization_status.confirmed == true
map_relocalization_required == null
selected_map_status.ready == true
```

不要把“调用过 `/api/localization/initialpose`”当成成功。只有后端返回确认成功，且状态接口也确认成功，才算真正重定位成功。

## 地图接口

### GET `/api/maps`

读取固定地图列表和当前地图身份。

响应核心字段：

```json
{
  "ok": true,
  "maps": [
    {
      "id": "map_1782442183242_ee7c6b76",
      "name": "F20（带工位）",
      "floor": "F20",
      "yaml_path": "/home/user/m20pro_maps/DESK_20260625_164234/occ_grid.yaml",
      "source": "106_active_map"
    }
  ],
  "selected_map_id": null,
  "working_map_id": "map_1782442183242_ee7c6b76",
  "effective_map_id": "map_1782442183242_ee7c6b76"
}
```

### POST `/api/maps/select`

切换固定地图，或切换到实时 `/map` 显示。

选择固定地图：

```json
{"map_id": "map_1782442183242_ee7c6b76"}
```

切到实时 `/map` 显示：

```json
{"map_id": ""}
```

响应核心字段：

```json
{
  "ok": true,
  "selected_map_id": null,
  "working_map_id": "map_1782442183242_ee7c6b76",
  "effective_map_id": "map_1782442183242_ee7c6b76",
  "map_relocalization_required": {
    "map_id": "map_1782442183242_ee7c6b76",
    "message": "当前固定地图已选择并同步到 Nav2，必须重新按开发手册2101定位"
  }
}
```

注意：

- 固定地图切换成功后，通常必须重新重定位。
- 选择实时 `/map` 不会调用 Nav2 `load_map`。
- 选择实时 `/map` 不会清除重定位要求。

### GET `/api/map`

读取实时 Nav2 `/map`。返回 `OccupancyGrid` 派生 JSON，包含 `data`，体积较大，只在需要绘制实时地图时请求。

### GET `/api/map_file?map_id=<id>`

读取固定地图文件，返回 `available`、`map_id`、`name`、`floor`、`width`、`height`、`resolution`、`origin`、`data` 等。体积较大，只在地图画布需要加载固定图时请求。

## 点位接口

### GET `/api/annotations`

读取点位列表。

查询参数：

| 参数 | 说明 |
| --- | --- |
| `map_id` | 可传固定地图 ID，也可传 `live_map` |

推荐：

```text
GET /api/annotations?map_id=live_map
```

响应核心字段：

```json
{
  "ok": true,
  "requested_map_id": "live_map",
  "effective_map_id": "map_1782442183242_ee7c6b76",
  "annotations": [
    {
      "id": "point_xxx",
      "map_id": "map_1782442183242_ee7c6b76",
      "type": "patrol",
      "floor": "F20",
      "label": "客厅P01",
      "area": "",
      "room": "客厅",
      "scan_point": "P01",
      "result_file_prefix": "B03_U01_H2008_F20_客厅_P01",
      "pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
      "dwell_s": 5.0,
      "manual_point_type": "task",
      "radar": {
        "enabled": true,
        "scans": [
          {"mode": "measuring", "label": "实测实量", "result_suffix": "measure"},
          {"mode": "modeling", "label": "点云建模", "result_suffix": "cloud"}
        ]
      }
    }
  ],
  "hidden_annotation_count": 3,
  "total_annotation_count": 8
}
```

昂锐雷达和甲方业务命名建议读取这些字段：

| 字段 | 用途 |
| --- | --- |
| `label` | 操作员给任务点起的名称 |
| `room` | 房间/部位 |
| `scan_point` | 扫描点编号 |
| `result_file_prefix` | 建议的结果文件前缀 |
| `radar` | 雷达模式配置，可包含多模式扫描 |
| `dwell_s` | 当前点位停留时间，雷达扫描耗时应由雷达流程自己判断完成，不要只依赖停留时间 |

### POST `/api/annotations`

保存点位。

推荐请求：

```json
{
  "map_id": "live_map",
  "source": "map_click",
  "type": "patrol",
  "floor": "F20",
  "label": "客厅P01",
  "area": "",
  "room": "客厅",
  "scan_point": "P01",
  "result_file_prefix": "B03_U01_H2008_F20_客厅_P01",
  "pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
  "manual_point_type": "task",
  "dwell_s": 5,
  "vendor_navigation": {
    "Gait": 12,
    "Speed": 1,
    "Manner": 0,
    "ObsMode": 0,
    "NavMode": 1
  },
  "radar": {
    "enabled": true,
    "scans": [
      {"mode": "measuring", "label": "实测实量", "result_suffix": "measure"},
      {"mode": "modeling", "label": "点云建模", "result_suffix": "cloud"}
    ]
  }
}
```

说明：

- `map_id` 推荐传 `effective_map_id`，也允许传 `live_map`。
- 后端会把 `live_map` 解析成当前有效固定地图。
- 如果当前实时 `/map` 不能匹配固定地图，保存会失败。
- 如果 `map_relocalization_required` 未清除，使用机器人实时位姿保存点位会被阻止。

### DELETE `/api/annotations?id=<annotation_id>`

删除点位。任务执行中涉及当前任务的点位不能删除。

## 任务接口

### GET `/api/tasks`

读取任务列表。

查询参数：

| 参数 | 说明 |
| --- | --- |
| `map_id` | 可传固定地图 ID，也可传 `live_map` |
| `include_all` | `true` 时返回所有任务，否则只返回当前有效地图任务 |

推荐：

```text
GET /api/tasks?map_id=live_map
```

响应中的任务会补充 `waypoints`，其中每个点位包含 `label`、`room`、`scan_point`、`result_file_prefix`、`radar` 等字段。雷达系统应该优先读取 `waypoints`，而不是再自己拼点位名称。

### POST `/api/tasks`

生成任务。

```json
{
  "name": "日常巡检任务",
  "map_id": "live_map",
  "annotation_ids": ["point_a", "point_b"]
}
```

响应：

```json
{
  "ok": true,
  "task": {
    "id": "task_xxx",
    "name": "日常巡检任务",
    "map_id": "map_1782442183242_ee7c6b76",
    "annotation_ids": ["point_a", "point_b"],
    "status": "ready"
  }
}
```

### POST `/api/tasks/start`

开始任务。

```json
{"task_id": "task_xxx"}
```

可选保护字段：

```json
{
  "task_id": "task_xxx",
  "expected_first_annotation_id": "point_a",
  "expected_task_map_id": "map_1782442183242_ee7c6b76"
}
```

任务启动前应确认：

```text
/api/state.localization_status.confirmed == true
/api/state.selected_map_status.ready == true
/api/state.map_relocalization_required == null
/api/preflight 或最近自检没有基础链路错误
```

### POST `/api/tasks/stop`

停止或复位当前任务。

```json
{"reason": "web_manual_stop"}
```

常用 reason：

| reason | 含义 |
| --- | --- |
| `web_manual_stop` | 操作员停止任务 |
| `web_manual_reset` | 操作员复位导航会话，会发送停止/零速度并清理导航显示状态 |

### POST `/api/tasks/update`

修改任务名称。

```json
{"task_id": "task_xxx", "name": "新的任务名"}
```

### DELETE `/api/tasks?id=<task_id>`

删除任务。运行中的任务不能直接删除。

## 当前任务点和雷达联动

HTTP 前端读取：

```text
GET /api/state?debug=0
```

关注：

```text
active_task
active_waypoint.parsed
```

`active_waypoint.parsed` 核心结构：

```json
{
  "task_id": "task_xxx",
  "task_name": "日常巡检任务",
  "phase": "navigating",
  "index": 0,
  "remaining_dwell_s": 0,
  "distance_m": 1.2,
  "goal_pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
  "waypoint": {
    "id": "point_xxx",
    "label": "客厅P01",
    "room": "客厅",
    "scan_point": "P01",
    "result_file_prefix": "B03_U01_H2008_F20_客厅_P01",
    "manual_point_type": "task",
    "dwell_s": 5,
    "radar": {"enabled": true}
  }
}
```

ROS 2 功能包也可以订阅：

```text
/m20pro/active_waypoint
```

消息类型是 `std_msgs/String`，内容为同一份 JSON。昂锐雷达如果跑在 ROS 2 包里，推荐订阅这个 topic 获取当前点位名称、房间、扫描点和雷达模式。

## 重定位接口

### POST `/api/localization/initialpose`

发布网页重定位请求，并等待开发手册 2101/1 回执和原厂定位位姿更新。

```json
{
  "x": 1.0,
  "y": 2.0,
  "z": 0.0,
  "yaw": 1.57,
  "floor": "F20"
}
```

成功判据：

```text
响应 confirmed == true
响应 localization_status.confirmed == true
随后 /api/state.localization_status.confirmed == true
随后 /api/state.map_relocalization_required == null
```

不要只看 HTTP 200，也不要只看 `initialpose_published`。这两个只能证明请求发出，不代表原厂定位真正成功。

## 自检接口

### GET `/api/preflight`

读取最近一次自检或正在运行的自检。

### POST `/api/preflight/run`

启动自检。

```json
{"mode": "move", "site": "auto", "wait": false}
```

说明：

- `wait=false`：后台执行，前端继续轮询 `/api/preflight`。
- `wait=true`：同步等待结果，容易阻塞前端，不建议高频使用。
- 电量只作为展示，不作为软件阻断条件。

## 建图和地图导入接口

建图接口主要给开发/现场工程前端使用，甲方业务前端如果不负责建图，可以不接。

### POST `/api/mapping/check_environment`

检查 106 建图环境和免密/命令可用性。

### POST `/api/mapping/session`

创建建图会话。

```json
{
  "project_name": "M20Pro 工地巡检",
  "building": "3栋",
  "mode": "single_floor",
  "floors": ["F20"],
  "active_floor": "F20",
  "map_name": "F20_test"
}
```

### POST `/api/mapping/start`

启动 106 建图。当前后端默认使用 `drmap mapping -b -s -n <map_name>`，即只建图，不立即切换为导航地图。

```json
{"session_id": "session_xxx"}
```

### POST `/api/mapping/finish`

保存/结束 106 建图。

```json
{"session_id": "session_xxx"}
```

### POST `/api/mapping/cancel`

取消建图。

```json
{"session_id": "session_xxx"}
```

### POST `/api/mapping/import_active_map`

把 106 建图结果拉到 104 归档，生成可供 `/api/maps` 使用的地图记录。

```json
{
  "session_id": "session_xxx",
  "floor": "F20",
  "map_name": "F20_test"
}
```

导入只归档，不自动启用。启用地图仍需调用 `/api/maps/select`。

## YOLO 和视频接口

### YOLO 检测

HTTP 前端从 `/api/state.detections` 读取最近 YOLO 检测结果。该字段来自 ROS 2 topic：

```text
/m20pro_yolov8_inspection/detections
```

检测节点运行状态从 `/api/state.inspection_status` 读取。该字段来自 ROS 2 topic：

```text
/m20pro_yolov8_inspection/status
```

检测节点统一发布 JSON。当前支持的推理后端：

| 后端 | 用途 |
| --- | --- |
| `ultralytics` | 直接加载 `src/m20pro_inspection/models/best.pt`，适合这两天快速验证模型效果 |
| `rknn` | RK3588/NPU 部署推荐路线，适合后续 104 常驻实机运行 |
| `onnx` | 笔记本或 CPU 验证中间模型 |
| `dry_run` | 无模型或未安装依赖时发布空检测，接口保持不崩 |

典型检测 payload：

```json
{
  "camera": "front_wide",
  "source_type": "rtsp",
  "backend": "ultralytics",
  "model_path": ".../models/best.pt",
  "image_width": 1280,
  "image_height": 720,
  "count": 1,
  "detections": [
    {
      "class_id": 0,
      "class_name": "未戴安全帽",
      "confidence": 0.87,
      "bbox_xyxy": [100.0, 80.0, 240.0, 320.0],
      "bbox_xywh": [100.0, 80.0, 140.0, 240.0]
    }
  ]
}
```

接口化消费方式：

| 消费方 | 推荐读取方式 |
| --- | --- |
| 甲方前端 / 调试前端 | `GET /api/state?debug=0`，读取 `detections.parsed` |
| ROS 2 后端包 | 订阅 `/m20pro_yolov8_inspection/detections` 和 `/m20pro_yolov8_inspection/events` |
| 需要绑定房间/点位的检测结果 | 同时订阅 `/m20pro/active_waypoint`，用当前任务点的 `label/room/scan_point/result_file_prefix` 关联结果 |

因此 YOLO 模型、前端页面和任务执行互不写死：换 `.pt`、`.onnx`、`.rknn` 只改检测节点启动参数；前端和任务包继续读同一套 JSON。

典型状态 payload：

```json
{
  "camera": "front_wide",
  "source_type": "rtsp",
  "requested_backend": "auto",
  "backend": "ultralytics",
  "model_path": ".../models/best.pt",
  "model_loaded": true,
  "ready": true,
  "frame_count": 120,
  "inference_count": 120,
  "last_frame_age_s": 0.2,
  "last_inference_ms": 85.4,
  "last_detection_count": 1,
  "last_error": null
}
```

前端判断建议：`inspection_status.parsed.ready` 表示节点活着；`last_error` 非空时展示错误；具体识别结果仍以 `detections.parsed.detections` 为准。

启动方式：

```bash
ros2 launch m20pro_inspection m20pro_inspection.launch.py \
  backend:=auto \
  model_path:=/home/user/m20pro_real_ros2_ws/install/m20pro_inspection/share/m20pro_inspection/models/best.pt \
  class_names_path:=/home/user/m20pro_real_ros2_ws/install/m20pro_inspection/share/m20pro_inspection/models/labels_zh.txt
```

104 的 `.pt + ultralytics` 快速验证路线使用独立依赖目录 `/home/user/m20pro_yolo_pydeps`。launch 默认只给 YOLO 节点注入该 `PYTHONPATH`，并在 104 上预加载 `/lib/aarch64-linux-gnu/libgomp.so.1` 以避免 torch 的静态 TLS 问题；这些环境变量不作用于 Nav2、web、tcp bridge 等节点。

当前 `best.pt` 的类别顺序为：未戴安全帽、未穿安全背心、跌倒、火灾、现场杂乱、配电箱打开。前端应展示 detection JSON 里的 `class_name`，不要在前端硬编码类别表。

全量 real 服务默认不启动 YOLO，避免影响导航算力。需要随服务启动时，在 104 的 `/etc/default/m20pro-real` 中显式设置：

```text
M20PRO_ENABLE_INSPECTION=true
M20PRO_INSPECTION_BACKEND=auto
M20PRO_INSPECTION_MODEL_PATH=/home/user/m20pro_real_ros2_ws/install/m20pro_inspection/share/m20pro_inspection/models/best.pt
M20PRO_INSPECTION_CLASS_NAMES_PATH=/home/user/m20pro_real_ros2_ws/install/m20pro_inspection/share/m20pro_inspection/models/labels_zh.txt
```

### 视频

| 接口 | 说明 |
| --- | --- |
| `GET /camera/front.mjpg` | 前相机 MJPEG |
| `GET /camera/rear.mjpg` | 后相机 MJPEG |
| `GET /camera/front.jpg` | 前相机单帧 |
| `GET /camera/rear.jpg` | 后相机单帧 |

视频是否可用可先看：

```text
GET /api/state?debug=0
camera_proxy
```

## 项目接口

### GET `/api/projects`

读取项目列表。

### POST `/api/projects`

创建项目。

```json
{"name": "M20Pro 工地巡检", "building": "3栋"}
```

## 推荐前端调用流程

### 页面启动

1. `GET /healthz`
2. `GET /api/state?debug=0`
3. `GET /api/maps`
4. `GET /api/annotations?map_id=live_map`
5. `GET /api/tasks?map_id=live_map`

### 切固定地图

1. `POST /api/maps/select {"map_id":"<fixed_map_id>"}`
2. 刷新 `GET /api/state?debug=0`
3. 如果 `map_relocalization_required != null`，要求操作员重新重定位。

### 切实时显示

1. `POST /api/maps/select {"map_id":""}`
2. 刷新 `GET /api/state?debug=0`
3. 继续用 `effective_map_id` 或 `map_id=live_map` 取点位/任务。

### 标点并生成任务

1. 确认 `effective_map_id` 非空。
2. 确认 `selected_map_status.ready == true`。
3. 点击地图得到 `pose`。
4. `POST /api/annotations` 保存点位。
5. `GET /api/annotations?map_id=live_map` 刷新点位。
6. `POST /api/tasks` 生成任务。

### 开始任务

1. `GET /api/state?debug=0`
2. 确认重定位和地图状态：
   - `localization_status.confirmed == true`
   - `map_relocalization_required == null`
   - `selected_map_status.ready == true`
3. `POST /api/tasks/start`
4. 轮询 `/api/state?debug=0` 显示：
   - `pose`
   - `path`
   - `active_task`
   - `active_waypoint.parsed`

## 稳定性边界

前端和外部包可以稳定依赖：

- 本文档列出的 URL、请求字段和核心响应字段；
- `effective_map_id` 作为点位/任务归属；
- `annotations[].label/room/scan_point/result_file_prefix/radar`;
- `tasks[].waypoints[]` 中的点位语义；
- `active_waypoint.parsed.waypoint` 的点位语义；
- `localization_status.confirmed` 作为重定位最终成功判断。

前端可以展示但不要当业务契约依赖：

- `events`;
- `topics`;
- `startup_map_sync` 的内部结构；
- `active_task` 的 timeline/诊断字段；
- Nav2 原始 status 字符串里的未文档字段。

## 维护规则

- 后端新增接口：先更新本文档，再给前端使用。
- 后端改字段含义：必须同步本文档、旧前端和新版封存前端。
- 新版前端和甲方前端不要复制后端判断逻辑，尤其不要自己推断实时 `/map` 属于哪张固定地图。
- 导航、重定位、点位、任务相关行为以 104 API 返回为准，不以页面文字或 rosbag 外的临时脚本为准。
