# 单层导航架构与拆分边界

本文档用于两件事：

- 让后续 AI 或新同事先理解当前系统边界，再改代码；
- 让项目复盘和面试说明有一条清晰主线。

当前阶段只把单层楼导航做扎实。跨楼层、复杂重定位、视频、3D 地形展示都不能抢在单层任务闭环之前扩张。

## 手册优先

真机接口语义以《山猫M20系列软件开发手册V0.0.9》和《山猫M20 Pro软件使用手册V0.0.1》为准。代码可以通过 ROS topic 或前端 API 包装这些接口，但不能把包装层的成功当成原厂接口成功。

当前必须追溯到手册的点：

- 重定位：开发手册 1.4.1，TCP `Type=2101`、`Command=1`，`ErrorCode=0` 才是手册接口成功证据。网页发布 `/initialpose` 只是触发该链路的 ROS 包装，不是成功证据。
- 地图坐标：软件使用手册 3.7 和开发手册 1.4.1 坐标换算说明。点位保存和任务启动必须确认坐标落在当前固定地图范围和可通行栅格上。
- 单点导航：开发手册 1.4.4，TCP `Type=1003`、`Command=1`。`PointInfo`、`Gait`、`Speed`、`Manner`、`ObsMode`、`NavMode` 必须按手册解释。
- 任务状态：开发手册 1.4.6。前端/日志中的 Nav2 状态不能替代原厂导航任务状态，只能作为本项目 Nav2 包装链路的证据。

如果代码实测和手册冲突，记录冲突证据，默认先把手册接口证据暴露出来，而不是把异常吞掉或用脚本绕过。

## 当前目标

单层任务链路的最低验收标准是：

1. 前端完成基础自检和重定位后，任务页允许启动同楼层任务。
2. 点击开始前，前端展示首点、点位顺序、当前地图和点位合法性。
3. 后端只向当前任务首点下发一次明确的 Nav2/floor goal，并记录 goal attempt、annotation id、目标位姿和路径版本。
4. Nav2 当前路径终点必须和当前任务点匹配；不匹配时任务失败，不允许继续往残留点跑。
5. 机器人到达第一个点后进入 dwell，等待配置的 `dwell_s`。
6. dwell 结束后切到下一个点；如果任务结束，任务状态、active task 状态和 rosbag 证据一致。
7. 前端、Nav2 状态、`/m20pro/active_waypoint` 和 rosbag 能解释任务到底是运行、等待、dwell、卡滞、超时、路径不匹配还是完成。

这些证据没有在真实前端任务里跑通之前，不认为单层导航完成。

## 主链路

正常单层任务链路如下：

```text
dashboard.html/css/js
  -> HTTP /api/tasks/start
  -> web_dashboard_node.py
  -> task_contract.py 校验任务、点位、前端启动期望
  -> active_task_contract.py 创建 active task 并下发当前点
  -> /m20pro/floor_goal
  -> floor_manager.py
  -> Nav2
  -> /m20pro/floor_nav_status、/plan、feedback
  -> nav_status_contract.py / task_plan_contract.py / task_progress_contract.py
  -> web_dashboard_node.py 更新 active task、任务状态和 /m20pro/active_waypoint
  -> dashboard.js 实时显示
```

`web_dashboard_node.py` 仍然是 ROS/HTTP 集成层，但不应该继续吞掉所有业务判断。能纯函数化的规则，应放进 contract 模块并加离线测试。

## 任务执行证据链

现场任务失败时，先按证据链定位断点，不凭前端一句“导航中”或“定位成功”下结论：

1. `/api/tasks/start` 成功返回，只能证明后端接受任务启动请求。
2. `floor_goal_published` timeline、`last_floor_goal_published_at`、`last_floor_goal_annotation_id`、`last_floor_goal_pose` 和 `floor_goal_publish_count` 才证明 Web 已经真正发布 `/m20pro/floor_goal`。
3. `/m20pro/floor_nav_status`、Nav2 feedback 和 `nav_goal_match` 证明 floor manager / Nav2 接收到的是当前任务点，而不是旧点或残留点。
4. `plan_goal_verified=true` 和 `path_goal_error_m` 证明当前路径终点匹配当前任务点。
5. `/m20pro/active_waypoint` 和 rosbag 中的 `/m20pro/floor_goal`、`/m20pro/floor_nav_status`、`/plan`、`/scan`、`/m20pro/lidar_points_relay` 共同用于复盘“发没发、接没接、路对不对、卡在哪”。

如果任务页显示导航中，但 rosbag 里没有 `/m20pro/floor_goal`，断点在 Web 发布目标前；如果已经发布但 Nav2 没有 accepted/feedback，断点在 `/m20pro/floor_goal` 到 floor manager / Nav2；如果 Nav2 接收了但 `path_goal_error_m` 大或 `plan_goal_verified=false`，断点是路径目标不匹配；如果路径正确但机器人不动或漂移，再看定位、避障和底盘状态。

## 模块边界

### 前端静态文件

- `static/dashboard.html`：页面结构。
- `static/dashboard.css`：视觉样式。
- `static/dashboard.js`：页面状态、渲染、HTTP 请求、前端确认文案。

前端负责展示和用户确认，不负责决定任务是否可执行。任务启动由后端接口校验和 contract 静态规则决定；Nav2、地图、点云等运行条件由自检和现场 rosbag 复盘确认，不再在任务启动时重复套一层 readiness。

当前前端只保留 2D 地图任务操作。3D 地图模式、3D/楼梯点云 HTTP 接口已经移除，不作为单层导航闭环的关键路径。前/后摄像头画面保留为按需打开的现场辅助视图；旧的摄像头全开关和诊断面板已移除。内部 `/m20pro/stair_zones` topic 仍保留，作为后续跨楼层逻辑的数据基础。

### `web_dashboard_node.py`

职责：

- ROS 订阅/发布；
- HTTP API；
- 参数和持久化；
- active task、任务状态和 `/m20pro/active_waypoint`；
- 调用 contract 后执行实际副作用。

不应该继续扩张的内容：

- 纯任务规则；
- 点位顺序规则；
- Nav2 状态匹配规则；
- 路径终点匹配规则；
- 卡滞/超时判断；
- 为某次现场问题临时增加的一次性脚本入口。

### `task_contract.py`

负责任务创建/启动前的静态接口规则：

- 任务状态是否允许开始；
- 点位顺序；
- 充电点必须在最后；
- 前端传来的 expected task id、annotation id、首点位姿、地图是否和后端当前任务一致。

### `active_task_contract.py`

负责 active task 状态机：

- 当前目标是否应该下发或重发；
- goal sent 状态和计数；
- 切换点位时清理旧 Nav2 状态；
- 到点后的 dwell 状态；
- dwell 结束后的 advance 或 task completed。

这里是“到第一个点、等待、切下一个点”的核心。

### `nav_status_contract.py`

负责解析并匹配 Nav2/floor-manager 状态：

- annotation id；
- frontend goal attempt；
- Nav2 `goal_seq`；
- 目标 x/y/z/yaw。

Nav2 的 accepted/succeeded 只有匹配当前 active waypoint 时才有效。残留目标、旧目标、固定点成功都不能推进任务。

### `task_plan_contract.py`

负责校验当前 Nav2 路径终点：

- 路径终点和当前任务点的误差；
- 等待 fresh plan；
- `path_goal_mismatch`；
- `plan_update_timeout`。

这个模块直接防止“前端任务点是 A，但 Nav2 实际规划去 B”。

### `task_progress_contract.py`

负责运行过程进展判断：

- 机器人是否在接近目标；
- 是否卡滞；
- 是否 waypoint timeout；
- 是否接近目标但 Nav2 长时间不返回成功。

它不直接发停止命令，只给出明确原因，副作用仍由 `web_dashboard_node.py` 执行。

### `active_waypoint_contract.py`

只负责 `/m20pro/active_waypoint` 这条任务/雷达接口：

- 当前任务点；
- 当前目标位姿；
- 当前任务阶段和粗略 Nav2 目标状态；
- dwell 剩余时间；
- 点位语义，包括房间、扫描点、结果前缀和昂锐雷达扫描模式。

路径版本、反馈明细、失败证据、watcher/analyzer 已从 active-waypoint 接口移除。现场复盘统一看 rosbag 和 active task 运行状态。

## 实习生分工

### A：复杂环境重定位

A 的边界是“让定位可信、可解释、可验收”。

建议范围：

- 重定位前后 pose 证据；
- 地图匹配质量；
- scan/pointcloud 与地图一致性；
- 前端定位成功/失败的判据；
- 定位状态和导航诊断中的 `localization_ok` 证据链。

A 不应该直接改 active task 状态机，也不应该绕过任务静态 contract 去允许开跑。如果需要新增定位字段，应先进入 localization/nav readiness 或 active-waypoint 接口，再由前端展示。

### B：跨楼层逻辑

B 的边界是“在单层任务稳定后，把多层任务拆成可靠的单层片段和楼层切换片段”。

建议范围：

- 楼层任务编排；
- 楼梯/电梯/过渡点；
- map id 与 floor id 切换；
- `/m20pro/floor_goal` 的跨楼层调度；
- 每层进入前后的重定位或确认策略。

B 不应该绕过单层任务 contract。跨楼层任务应复用同一个单层 waypoint 下发、路径校验、Nav2 状态匹配、dwell/advance 机制。

## 开发原则

1. 先单层，后跨楼层。
2. 先证据，后结论。
3. 能删除的入口不保留。
4. 能纯函数测试的规则不要藏在 ROS 回调里。
5. 现场脚本只用于采集证据或启动标准链路，不用于替代核心逻辑。
6. 每次改进后维护 `m20pro日志.md`。

## 当前未完成证据

现在已有静态测试和 contract 测试，但还缺真实运动闭环证据：

- 从前端点击开始；
- 机器人去的是当前任务首点；
- 前端实时显示机器人、Nav2、目标误差；
- 到点后进入 dwell；
- dwell 后切到下一个点；
- 任务状态、`/m20pro/active_waypoint` 和 Nav2 状态一致。

拿到这组证据前，不能把单层导航目标标记为完成。

## 上车阻断处理

目标模式不能继续上车推进时，先区分前端问题和底层门槛。104 当前以 `/api/state` 和 rosbag 为准，不把历史故障写死成永久阻断；如果现场出现下列状态，再按对应底层链路处理：

- `perception_status.code=scan_unavailable`：106 edge scan 或 DDS 轻量 `/scan` 链路没有有效输出；必须恢复后才能导航。

电量通过前端顶栏或 `./scripts/104_goal_mode_battery_gate.py --url http://10.21.31.104:8080` 只读显示给操作员参考，不作为软件自检、部署、重定位或任务启动门槛。

在感知链路硬故障未恢复前，只允许做只读诊断、文档维护和离线 contract 测试；不调用重定位、不启动任务、不发布运动目标。需要重启本工程服务前，先确认 `active_task=None`；不要从本工程脚本重启原厂 multicast/lidar 服务。

## 充电后上车顺序

电量恢复到 `25%` 以上后，不直接测试任务运动。先按下面顺序恢复底层证据：

1. 只读运行 `./scripts/104_goal_mode_battery_gate.py --url http://10.21.31.104:8080`，确认电量和 active task 状态。
2. 部署本地 pending 的 Web/contract 减法改动，只构建 `m20pro_cloud_bridge`，只重启 `m20pro-real.service`。
3. 重新读取 `/api/state`，确认：
   - `startup_map_sync.ok=true`;
   - `selected_map_status.ready=true`;
   - `localization_status` 明确提示当前是否需要重定位；
   - `perception_status` 明确给出当前感知状态。
4. 如果感知异常，先处理 `scan_unavailable`，直到 `106 edge scan -> /scan -> perception_status` 全链路恢复。
5. 感知恢复后，按开发手册 2101 跑前端重定位。成功判据必须是 TCP `Type=2101`、`Command=1`、`ErrorCode=0`，并且前端显示定位已确认、当前地图和点云状态正常。
6. 只在当前固定地图上重新标当前地图任务点，创建当前地图任务。不要复用旧地图任务。
7. 运动前先启动 rosbag 记录，确认网页首点、顺序、地图、pose、scan、lidar 状态都符合现场预期。
8. 明确允许运动后，才从前端点击任务开始。验收只看真实闭环证据：首点、dwell、下一个点、任务状态、`/m20pro/active_waypoint` 和 rosbag。

如果任一步失败，先记录证据并回到对应 contract 或底层链路，不通过临时脚本绕过。
