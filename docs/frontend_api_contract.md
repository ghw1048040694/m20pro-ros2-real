# M20Pro 前端 API 对接契约

更新时间：2026-07-22 14:18 CST

本文档是“M20 Pro ROS 2 跨楼层巡检导航系统”正式经典前端、甲方前端和外部功能包对接 104 Web 后端的接口契约。接口实现位于 `m20pro_cloud_bridge.web_dashboard_node`。以后新增、删除或修改接口字段时，必须同步更新本文档。

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
6. 项目 `floors` 是建图项目元数据，不是普通地图库白名单；普通地图只要自身楼层格式合法，就可以选择、绑定点位和重定位。
7. 只有跨楼层路线与跨楼层任务使用严格路线楼层注册表。前端不得用项目楼层列表拒绝普通地图。

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
| `relocalization_attempt` | 最近一次网页重定位事务（`pending`/`confirmed`/`failed`）；失败事务会覆盖旧的成功状态 |
| `pose` | 当前地图坐标系下机器人位姿 |
| `pose_fresh` / `pose_age_sec` | 位姿是否新鲜 |
| `floor` | 当前楼层 |
| `scan` | `/scan` 摘要和前端激光轮廓点 |
| `path` | 当前规划路径，用于画导航线 |
| `active_task` | 当前正在执行的任务状态，空闲时为 `null` |
| `active_waypoint` | 当前任务点 JSON 字符串和解析结果 |
| `charge_command_result` | 最近一次原厂充电导航请求的 JSON 回执；只在充电任务阶段使用 |
| `detections` | YOLO 检测结果，来自 `/m20pro_yolov8_inspection/detections` |
| `inspection_status` | YOLO 检测节点运行状态，来自 `/m20pro_yolov8_inspection/status` |
| `camera_proxy` | 前后相机代理状态 |
| `battery` | 电量显示信息，只用于展示，不作为软件阻断条件；`count` 和 `packs` 只统计原厂电池数组中存在有效遥测的电池，断开的全零占位槽位不会计入 |
| `motion_state` | 原厂基础状态中的当前运动姿态和 `last_update`；前端仅使用 3 秒内的新鲜值切换起立/趴下 |
| `preflight` | 最近一次自检结果 |

定位成功判断：

```text
localization_status.confirmed == true
map_relocalization_required == null
selected_map_status.ready == true
```

不要把“调用过 `/api/localization/initialpose`”当成成功。只有后端返回确认成功，且状态接口也确认成功，才算真正重定位成功。若 `relocalization_attempt.status=failed`，旧的地图位姿不能继续作为本次重定位成功依据。

## 雷达接口

雷达前端优先使用以下 JSON API，不需要读取后端结果目录或解析网页 DOM。

### GET `/api/radar/status`

读取雷达模块最新状态、最新结果和结果目录。

稳定字段：

| 字段 | 说明 |
| --- | --- |
| `ok` | 固定为 `true` |
| `results_dir` | 雷达结果根目录 |
| `latest_parsed` | 最新 `/m20pro/radar_inspection/status` 或 result 的解析 JSON |
| `latest_job` | 最近一次落盘的雷达扫描结果 |
| `job_count` | 当前结果目录中的扫描结果数量 |

### GET `/api/radar/results`

读取雷达扫描结果列表。

查询参数：

| 参数 | 说明 |
| --- | --- |
| `task_id` | Web 任务 ID |
| `radar_task_id` / `taskId` | 雷达设备任务 ID |
| `run_id` | 后端生成的单次扫描 ID |
| `waypoint_key` | `/m20pro/active_waypoint` 对应点位 key |
| `status` | `completed`、`failed` 等 |
| `state` | `finished`、`result_unavailable` 等 |
| `mode` / `scan_mode` | `measuring` 或 `modeling` |
| `limit` / `offset` | 分页，默认 `limit=50` |

响应核心字段：

```json
{
  "ok": true,
  "total": 1,
  "count": 1,
  "results": [
    {
      "task_id": "api_radar_test_003",
      "taskId": "B01_U01_F20_R2008_P01_measure_20260710_164336",
      "status": "completed",
      "state": "finished",
      "scan_mode": "measuring",
      "metric_count": 12,
      "result_fetch_status": "success",
      "summary": {"metrics": []},
      "summary_path": "/home/user/m20pro_radar_results/summaries/xxx.json",
      "raw_path": "/home/user/m20pro_radar_results/raw/xxx.json"
    }
  ]
}
```

雷达结果不是仅供前端临时显示的内存数据。104 会把以下内容持久化到 `radar_results_dir`（正式环境默认为 `/home/user/m20pro_radar_results`）：

- `jobs/`：每次任务/点位的完整执行记录，前端结果列表从这里分页读取；
- `raw/`：雷达设备返回的原始 JSON 和任务信息；
- `summaries/`：平整度、极差等结构化指标摘要；
- `downloads/`：仅当雷达 HTTP 接口返回可下载文件 URL 时保存点云工程或压缩包；
- `manual/`：人工登记的点云路径和人工测量回填。

当前系统没有甲方服务器“上传成功确认”接口，也没有自动清理策略，因此这些文件会持续积累。正式对接时应由上传组件读取任务结果，收到甲方服务器持久化 ACK 后再按任务清理本地副本；没有 ACK 时不得自动删除。前端使用分页接口，不会把全部历史结果一次渲染到页面。

### GET `/api/radar/result`

读取单条雷达结果。支持 `run_id`、`radar_task_id`、`taskId`、`waypoint_key` 等参数；没有匹配结果时返回 HTTP 400。

### GET `/api/radar/task`

按 Web 任务 ID 读取雷达任务汇总。

查询参数：

| 参数 | 说明 |
| --- | --- |
| `task_id` | 必填，Web 任务 ID |

返回字段包含 `task`、`results`、`summary`。`summary.result_unavailable_count > 0` 表示雷达已触发但后端未拿到结果。

### GET `/api/radar/task_export`

下载任务级雷达结果，兼容人工交付：

```text
GET /api/radar/task_export?task_id=<task_id>&format=json
GET /api/radar/task_export?task_id=<task_id>&format=csv
```

### POST `/api/radar/artifact`

登记点云建模结果文件路径。

### POST `/api/radar/manual_measurement`

登记人工测量回填。

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

### DELETE `/api/maps`

删除 104 上的普通归档地图：

```text
DELETE /api/maps?id=<map_id>&cascade=true
```

- 项目内置地图可以从业务地图库移除，删除状态保存在 104 的 `hidden_builtin_map_ids`，后续部署不会重新显示；仓库中的只读源文件保留，不直接修改源码资产；
- 当前生效地图、工作地图和实时 `/map` 实际匹配的地图不能删除，必须先切换到其他地图；
- 任务执行中不能删除地图；
- `cascade=true` 会同步删除该地图的点位和依赖任务，并清除建图会话里的导入引用；
- 只会物理删除 `map_archive_dir` 内由 104 管理且未被其他地图共用的目录，不会删除 106 原厂地图或工程目录外文件；
- 删除地图不会删除项目的楼层元数据；项目楼层元数据也不能影响其他普通地图的选择和重定位；
- 雷达历史结果是独立的交付证据，不随地图删除。

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

### POST `/api/maps/edit`

把当前已生效固定地图上的稀疏栅格修改保存为一个新地图版本。坐标使用地图图片坐标：左上角为 `(0, 0)`；`value` 只能是 `0`（可通行）、`100`（障碍）或 `-1`（未知）。

```json
{
  "map_id": "map_1782442183242_ee7c6b76",
  "name": "F20（带工位）_修饰",
  "cells": [
    {"x": 120, "y": 85, "value": 0},
    {"x": 121, "y": 85, "value": 100}
  ]
}
```

接口约束：

- 只允许修改当前已经生效的固定地图，实时 `/map` 和任务执行期间拒绝写入；
- 原地图永不覆盖，新地图记录使用 `source=web_map_editor` 并记录 `parent_map_id`；
- 新版本继承原地图的点位，并为单地图任务生成可独立执行的任务副本；
- Web 修饰只改变 104/Nav2 使用的二维占据栅格，不伪造 106 原厂地图路径；切换新版本时不会把 106 错误切回未修饰地图；
- 保存接口不自动切图，调用方需要在操作者确认后再调用 `/api/maps/select`。

响应核心字段：

```json
{
  "ok": true,
  "map": {"id": "map_new", "parent_map_id": "map_old", "source": "web_map_editor"},
  "parent_map_id": "map_old",
  "changed_cells": 42,
  "cloned_annotations": 5,
  "cloned_tasks": 2,
  "message": "地图修饰已保存为新版本；原地图未覆盖"
}
```

### GET `/api/map`

读取实时 Nav2 `/map`。返回 `OccupancyGrid` 派生 JSON，包含 `data`，体积较大，只在需要绘制实时地图时请求。

### GET `/api/map_file?map_id=<id>`

读取固定地图文件，返回 `available`、`map_id`、`name`、`floor`、`width`、`height`、`resolution`、`origin`、`data` 等。体积较大，只在地图画布需要加载固定图时请求。

### GET `/api/multi_floor`

读取普通地图工作区、项目楼层元数据和跨楼层路线摘要。核心字段包括 `current_floor`、`selected_map_id`、`floors[]`、`routes[]`、`latest_mapping_session` 和 `identity_issues`。

`floors[].route_configured=true` 才表示该楼层属于严格跨楼层路线配置；`registry_source=project` 仅表示建图项目登记，不能作为地图选择白名单。没有显式跨楼层路线注册表时，普通地图楼层会进入工作区；存在路线注册表时，跨楼层工作区只暴露路线已注册数据，但这不影响普通地图通过 `/api/maps/select` 选择和重定位。

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
    "Speed": 1
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
- `type` 的正式前端取值依次为 `patrol`（任务点）、`transition`、`charge`、`stair_entry`（爬楼梯点）、`stair_exit`、`stair_switch`。
- `manual_point_type` 由 `type` 推导；外部调用可以显式传入，但不能与 `type` 表达相反的业务含义。
- `vendor_navigation` 只接受 `Gait` 和 `Speed` 调整；后端始终固定 `Manner=0`、`ObsMode=0`、`NavMode=1`，并根据点位语义生成 `PointInfo`。
- `stair_entry` 未显式传 `Gait` 时默认 `14`，其他点位默认 `12`。实际跨楼层上下楼方向及步态切换由 `floor_manager` 路线配置负责。

### DELETE `/api/annotations?id=<annotation_id>`

删除点位。任务执行中涉及当前任务的点位不能删除。

### POST `/api/annotations/update`

修改已有点位。请求体与新增点位一致，并额外传入 `id`。点位 `id`、`map_id` 和 `created_at` 保持不变；不允许通过修改接口把点位迁移到另一张地图。点位正在当前任务中执行时不允许修改。

```json
{
  "id": "point_xxx",
  "map_id": "map_xxx",
  "type": "patrol",
  "floor": "F20",
  "label": "修改后的点位",
  "pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5},
  "manual_point_type": "task",
  "dwell_s": 10
}
```

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

后端在发送第一个 `/m20pro/floor_goal` 前还会核对 Nav2 local costmap
滚动窗口中心与小写 `/odom`。重定位后两者未对齐时任务不会启动：

```json
{
  "ok": false,
  "code": "local_costmap_odom_mismatch",
  "message": "局部代价地图仍停留在重定位前位置，与 /odom 相差 12.01 m；未下发任务目标",
  "odom_alignment": {
    "ready": false,
    "error_m": 12.009,
    "tolerance_m": 0.75
  }
}
```

`local_costmap_alignment_unavailable` 表示尚未收到带滚动窗口原点的完整
local costmap 或 `/odom` 信息不完整。两种情况都应保持任务停止，等待导航链路恢复或重新重定位，不能由前端绕过。

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

## 人工遥控接管

遥控不直接抢占 Nav2 的 `/cmd_vel`。Nav2 发布 `/cmd_vel_nav`，网页发布 `/cmd_vel_teleop`，`m20pro_command_mux` 以 `navigation / teleop / locked` 三态仲裁后才向 TCP bridge 的 `/cmd_vel` 输出。任何模式切换、指令超时或异常值都先输出零速度。

安全规则：

- 只有 104 以 `move` 模式运行、速度仲裁器就绪且楼梯感知会话未激活时可以接管。
- 接管前后端先停止当前任务；结束接管后仲裁器进入 `locked`，旧任务不会自动恢复。
- 同一时间只允许一个遥控会话；指令必须带会话 ID 和单调递增序号，迟到指令被忽略。
- 浏览器必须在 `teleoperation.command_timeout_s` 内续租。松键、失焦、页面隐藏、断网或后端异常都会零速停车；仲裁器还有独立的第二道超时保护。
- 前端用一个姿态按钮在起立/趴下之间切换。姿态依据 `m20pro_tcp_bridge` 从原厂 2Hz `Type=1002/Command=6` 主动基础状态生成的 `/m20pro_tcp_bridge/motion_state`；`MotionState=0/2/3/4` 时目标为起立，`1/6/8` 时目标为趴下，状态缺失或超过 3 秒时按钮显示“姿态未知”并禁用。起立、趴下和软急停都先发送零速度并锁定速度仲裁器；起立/趴下动作结束后必须重新申请人工接管。原厂动作分别对应 `Type=2/Command=22` 的 `MotionParam=1/4`，软急停为 `MotionParam=2`。
- 原厂手柄不经过本项目的 ROS 仲裁器，不能与网页遥控同时操作；需要切换到手柄时先结束网页接管并确认状态锁定。
- 本接口是运动控制面，在 VPN、身份认证和访问控制完成前不得经内网穿透直接暴露到公网。

### GET `/api/teleop/state`

读取遥控可用性、接管状态、仲裁模式、心跳年龄和限速。不返回当前会话 ID。`GET /api/state` 的 `teleoperation` 字段与此结构一致。

```json
{
  "ok": true,
  "teleoperation": {
    "available": true,
    "active": false,
    "acquiring": false,
    "status": "inactive",
    "mux_mode": "locked",
    "stair_session_active": false,
    "command_timeout_s": 0.35,
    "limits": {
      "forward_mps": 0.18,
      "reverse_mps": 0.12,
      "lateral_mps": 0.18,
      "angular_radps": 0.45
    }
  }
}
```

### POST `/api/teleop/acquire`

显式确认并申请人工接管。

```json
{"confirm": true}
```

成功后返回本操作端专用的 `session_id`。调用方必须立即开始发送指令心跳，不得将该 ID 持久化或转发给其他操作端。

```json
{
  "ok": true,
  "session_id": "teleop_xxx",
  "message": "已终止自主任务并进入人工接管"
}
```

### POST `/api/teleop/command`

三个轴都是 `[-1, 1]` 的归一化值，后端再按统一现场参数换算为实际速度。`sequence` 从 `0` 开始严格递增。

```json
{
  "session_id": "teleop_xxx",
  "sequence": 12,
  "linear_x": 1.0,
  "linear_y": 0.0,
  "angular_z": 0.0
}
```

停止但保持接管时发三轴全 `0`。不允许超范围、非有限数、无会话或无序号指令。

### POST `/api/teleop/release`

```json
{"session_id": "teleop_xxx"}
```

发布多个零速样本、结束会话并把仲裁器切到 `locked`。返回成功不代表恢复自主任务。

### POST `/api/teleop/emergency_stop`

无请求字段。停止当前任务、人工接管和所有网页运动指令，最终保持 `locked`。该接口可用于无法取得当前遥控会话 ID 时的安全停止。

### POST `/api/teleop/motion`

执行人工接管弹层中的运动状态动作。`stand`（起立）和 `lie`（趴下）必须携带当前遥控会话；两者执行前会结束遥控并锁定速度。`soft_stop`（软急停）不要求会话，可在无法取得会话 ID 时直接执行。

```json
{"action": "stand", "session_id": "teleop_xxx"}
```

```json
{"action": "soft_stop"}
```

### POST `/api/charge/one_key`

按当前有效地图的唯一 `manual_point_type=charge` 点位启动一键充电任务。没有充电点、存在多个充电点、当前任务运行中或网页遥控接管中都会拒绝；不会猜测充电桩坐标。

执行链路是固定的：先由 Nav2 导航到充电点；到点后任务进入 `phase=charging`，104 通过 `/m20pro/charge_command` 请求 TCP 桥下发原厂 `Type=1003/Command=1`，并强制使用 `PointInfo=3`；只有收到 `/m20pro_tcp_bridge/charge_result` 的 `status=accepted` 才将任务标记为完成。原厂拒绝、TCP 超时或桥未就绪都会将任务标记为失败，不会把“已到点”伪装成“已充电”。

录包时应同时保留以下两个话题，用于核对请求和回执：

```text
/m20pro/charge_command
/m20pro_tcp_bridge/charge_result
```

当前 103 固件不提供可用的独立充电状态查询接口，因此 `accepted` 的含义是“原厂已接受充电导航命令”，不是软件伪造的“正在充电”。如果原厂回执包含 `Status`/`Value`，会原样保存在回执的 `vendor_status`/`vendor_value` 字段；电流方向只能通过 `/BATTERY_DATA` 辅助观察。

```json
{"ok": true, "charge_task_id": "task_xxx", "message": "已按当前地图充电点启动一键充电任务"}
```

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
  "yaw": 1.57
}
```

正式前端不显示也不提交重定位楼层。后端从当前选中的固定地图取得楼层，并校验请求与地图身份一致。`floor` 仅为跨楼层内部事务和兼容调用保留；外部前端不得让操作者在重定位弹层中另选楼层。

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

## 录包接口

### GET `/api/recording/status`

返回当前录包进程、名称、开始时间和最新 bag 路径。

### POST `/api/recording/start`

```json
{"duration_s": 300, "prefix": "testfield"}
```

`duration_s` 限制为 10~3600 秒。后端调用正式 `m20pro_record_real.sh`，不使用 `ros2 bag record -a`。

### POST `/api/recording/stop`

向当前录包进程发送 `SIGINT`，等待 rosbag 完成落盘。

### GET `/api/recording/list`

读取 104 `/home/user/bags` 下的已保存录包。只列出直接子目录中的有效 rosbag 目录，按最近修改时间倒序返回；`size_bytes`、`message_count` 和 `duration_s` 用于前端展示。

### POST `/api/recording/rename`

修改录包目录名称，不改动包内 `metadata.yaml` 和数据文件。

```json
{"id": "testfield_20260722_113000", "name": "工地长距离测试"}
```

录包正在写入时不能改名；名称会经过路径安全清洗，不能包含目录分隔符。

### DELETE `/api/recording?id=<id>`

删除一个已保存录包目录。录包正在写入时拒绝删除，删除不可恢复。

### GET `/api/recording/download?id=<id>`

以流式 `tar.gz` 附件下载一个录包目录，避免将大包完整加载到 Web 进程内存。下载后解压得到以录包名称命名的目录，可直接交给 `ros2 bag info` 或 `ros2 bag play`。

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
  "mode": "multi",
  "floors": ["7", "8", "9"],
  "map_name": "3栋现场图"
}
```

`mode` 取 `single` 或 `multi`。楼层可写数字、`F` 编号或地下层编号，例如 `7`、`F7`、`B1`；接口会统一保存为 `F7`、`B1`。`floors` 在多楼层模式下表示本次项目要建立的全部现场楼层，不表示机器狗当前实时楼层。正式前端不提交 `active_floor`，后端按 `floors` 顺序选择第一层作为初始步骤；API 自动化工具可以显式传入 `active_floor`，但必须属于 `floors`。后续步骤切换使用建图会话的逐层状态，不在创建表单重复选择。

`map_name` 可留空，后端会生成 `<active_floor>_<时间>` 形式的唯一名称。每次新的建图操作都必须创建新会话；正式前端只允许复用当前页面尚未结束的 `created/ready/pending/waiting_manual` 会话，不会从 `latest_mapping_session` 恢复已保存、已拉取或已取消的历史会话。

创建建图会话是登记新项目楼层的唯一入口，历史地图和点位不会反向创建楼层。登记楼层不会创建跨楼层路线，跨层任务仍要求显式路线配置（历史 F19/F20/F21 示例位于 `docs/archived_route_profiles/legacy_inspection_waypoints_f19_f20_f21.yaml`，默认运行时不加载）。

### POST `/api/mapping/start`

启动 106 建图。当前后端默认使用 `drmap mapping -b -s -n <map_name>`，即只建图，不立即切换为导航地图。

```json
{"session_id": "session_xxx"}
```

启动前置条件：

- `created/ready/pending/waiting_manual` 会话可以启动；
- `mapping` 返回 `code=mapping_session_busy`，防止重复启动；
- `saved/imported/cancelled` 返回 `code=mapping_session_terminal`，调用方必须重新创建会话；
- 进度 UI 只能根据本次接口响应和本次会话状态推进，不能把历史会话终态渲染成新任务进度。

### POST `/api/mapping/finish`

保存/结束 106 建图。

```json
{"session_id": "session_xxx"}
```

### POST `/api/mapping/select_floor`

多楼层建图完成当前层后，切换会话的下一建图步骤。目标楼层必须属于会话 `floors`；会话处于 `mapping` 时拒绝切换。

```json
{"session_id": "session_xxx", "floor": "F8"}
```

响应返回更新后的 `session` 和当前 `step`。切换步骤不会自动启动建图，也不会自动拉取地图；调用方仍需依次调用 `/api/mapping/start`、`/api/mapping/finish` 和 `/api/mapping/import_active_map`。

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

存在建图会话时，后端按该会话的 `map_name` 查找 106 上最新的同名时间戳目录；不会因为 104 删除过同名地图就把旧会话重新当成新任务。`/api/maps` 删除只负责 104 业务地图库，如需清理 106 原厂包，必须另行确认 `/var/opt/robot/data/maps/active` 不指向目标目录后再精确删除。

## 跨楼层路线接口

跨楼层路线不是由项目楼层或地图名称自动推导的。每条路线必须绑定两张正式地图和四个实测语义点，并按方向保存；`F1 -> F2` 不会隐式生成 `F2 -> F1`。

### GET `/api/floor_routes`

返回已保存的有向路线、可用于路线配置的楼梯语义点和地图摘要。只有 `stair_entry`、`stair_switch`、`stair_exit` 且位姿有效的点会进入候选列表。

### POST `/api/floor_routes`

```json
{
  "name": "1层到2层东楼梯",
  "entry_annotation_id": "point_f1_entry",
  "source_platform_annotation_id": "point_f1_switch",
  "target_platform_annotation_id": "point_f2_switch",
  "post_exit_annotation_id": "point_f2_exit"
}
```

保存时后端强制检查：两侧点位类型、楼层、地图和坐标一致；两张地图均具备 104 Nav2 yaml 和非 `active` 的 106 原厂地图包；同一楼层的所有路线只能引用同一张正式地图。路线持久化到 Web `data_dir/floor_routes.json`，并通过 transient-local `/m20pro/floor_route_config` 动态下发给 `floor_manager`。

### POST `/api/floor_routes/delete`

```json
{"id": "floor_route_xxx"}
```

路线被删除前，其地图和四个点位不能删除或修改。任务执行中禁止增删路线。

### 内部切层协议

`floor_manager` 到达起始层共享平台后发布 `/m20pro/floor_switch_request`；Web 校验当前任务、路线、起始地图和目标身份后，依次切换 104 Nav2 地图、执行 106 `drmap apply`、发布目标层初始位姿并等待 2101/定位/位姿证据。只有全部确认才通过 `/m20pro/floor_switch_result` 返回成功。地图切换、重定位、任务取消或异常失败时执行 104/106 回滚；回滚不完整会返回 `state_uncertain=true`，`floor_manager` 清空当前楼层并停止任务。

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

正式 104 进程默认以休眠态常驻，不加载 RKNN、不连接 RTSP。前端启停使用：

```http
POST /api/inspection/toggle
Content-Type: application/json

{"enabled":true}
```

Web 后端通过 `/m20pro_yolov8_inspection/set_enabled` 的 `std_srvs/SetBool` 服务控制节点。启用时才加载 RKNN/NPU 并启动 RTSP 最新帧线程；关闭时释放 RKNN、摄像头和缓存帧，不重启 `m20pro-real.service` 或 Nav2。

轻量检测刷新接口：

| 接口 | 说明 |
| --- | --- |
| `GET /api/inspection/state` | 只返回最近检测 JSON、YOLO 状态和启停服务状态；正式前端约每 200 ms 调用一次，仅在前摄像头和 YOLO 同时开启时调用 |

前端先打开 `http://10.21.31.104:8888/video1/` 对应的 H.264 低延迟 HLS 视频，再在视频卡片内启用 YOLO。检测框由浏览器 Canvas 叠加到这条视频上。系统不再发布或提供第二路 ROS Image、JPEG 或 MJPEG 标注视频，避免重复编码和第二个播放器造成卡顿。

检测节点统一发布 JSON。当前支持的推理后端：

| 后端 | 用途 |
| --- | --- |
| `ultralytics` | 仅用于 x86_64 上位机模型基准与导出，不部署到 104 |
| `rknn` | 104 正式后端，使用 RK3588 NPU 按需推理 |
| `onnx` | 笔记本或 CPU 验证中间模型 |
| `dry_run` | 无模型或未安装依赖时发布空检测，接口保持不崩 |

典型检测 payload：

```json
{
  "camera": "front_wide",
  "source_type": "rtsp",
  "backend": "ultralytics",
  "model_path": ".../models/best_rk3588_fp16.rknn",
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
  "enabled": true,
  "state": "ready",
  "camera": "front_wide",
  "source_type": "rtsp",
  "requested_backend": "auto",
  "backend": "ultralytics",
  "model_path": ".../models/best_rk3588_fp16.rknn",
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

前端判断：`enabled=false` 表示进程休眠且资源已释放；`enabled=true && ready=true` 才表示模型、RTSP 和新鲜帧均就绪；`last_error` 非空时展示错误；具体识别结果仍以 `detections.parsed.detections` 为准。

启动方式：

```bash
ros2 launch m20pro_inspection m20pro_inspection.launch.py \
  enabled:=true \
  backend:=rknn \
  model_path:=/home/user/m20pro_real_ros2_ws/install/m20pro_inspection/share/m20pro_inspection/models/best_rk3588_fp16.rknn \
  class_names_path:=/home/user/m20pro_real_ros2_ws/install/m20pro_inspection/share/m20pro_inspection/models/labels_zh.txt
```

104 的正式节点只注入 `/home/user/m20pro_rknn_pydeps`，其中只有 RKNNLite；不再注入 Torch `PYTHONPATH`，也不再使用 `LD_PRELOAD=libgomp`。

当前 `best.pt` 的类别顺序为：未戴安全帽、未穿安全背心、跌倒、火灾、现场杂乱、配电箱打开。前端应展示 detection JSON 里的 `class_name`，不要在前端硬编码类别表。

全量 real 服务始终启动轻量控制节点，但默认 `enabled=false`，不会加载模型或占用摄像头/NPU。可由前端随时启停；若要求服务启动后直接推理，在 104 的 `/etc/default/m20pro-real` 中设置：

```text
M20PRO_ENABLE_INSPECTION=true
M20PRO_INSPECTION_BACKEND=rknn
M20PRO_INSPECTION_MODEL_PATH=/home/user/m20pro_real_ros2_ws/install/m20pro_inspection/share/m20pro_inspection/models/best_rk3588_fp16.rknn
M20PRO_INSPECTION_CLASS_NAMES_PATH=/home/user/m20pro_real_ros2_ws/install/m20pro_inspection/share/m20pro_inspection/models/labels_zh.txt
```

### 视频

| 接口 | 说明 |
| --- | --- |
| `http://10.21.31.104:8888/video1/` | 前相机 H.264 低延迟 HLS 播放器 |
| `http://10.21.31.104:8888/video2/` | 后相机 H.264 低延迟 HLS 播放器 |

`/camera/front.*` 与 `/camera/rear.*` 属于已停用的旧通用代理接口，正式原始视频仍走 8888 H.264 网关。YOLO 不再提供 `/camera/yolo.*` 标注视频接口。

视频是否可用可检查：

```text
GET http://10.21.31.104:8888/video1/index.m3u8
GET http://10.21.31.104:8888/video2/index.m3u8
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
- 后端改字段含义：必须同步本文档、正式经典前端和合同测试；封存前端不再继续维护。
- 正式经典前端和甲方前端不要复制后端判断逻辑，尤其不要自己推断实时 `/map` 属于哪张固定地图。
- 导航、重定位、点位、任务相关行为以 104 API 返回为准，不以页面文字或 rosbag 外的临时脚本为准。
