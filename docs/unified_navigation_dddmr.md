# 统一导航与 DDDMR 适配边界

状态：集成分支 `feature/unified-navigation-dddmr`，当前工作分支
`feature/unified-navigation-v2`；统一计划已接入任务创建，尚未接管真实运动。

## 目标

系统只保留一套导航任务模型。单层不是另一种运行模式，而是楼层段数量为
`n=1` 的统一计划；跨楼层只是同一计划中增加有向连接边。地图、点位、任务
顺序、到点判断和停止策略不因为 `n` 的大小复制第二套机制。

统一计划的结构为：

```text
unified_navigation_plan
  floor segments:  [map + ordered waypoints]
  transitions:     [validated directed connector edges + terrain profile identity]
```

当只有一个楼层时，`segments` 只有一项，`transitions` 为空；这不是走一个
“单层专用入口”，而是同一个计划执行器的最小输入。

任务落盘时只保存计划的紧凑记录：有序点位 ID、连续楼层段、地图 ID 和有向
连接 ID。点位详情仍由点位库维护，路线几何仍由路线配置维护，任务不会复制
第二份地图或路线数据库。`F1 -> F2 -> F1` 会保存为三个连续楼层段，不能把
前后两个 F1 合并。

任务启动前会用当前点位库和有向路线表重建并核对这份记录：没有计划的历史任务
只迁移一次；已有计划发生顺序、地图或连接边变化时拒绝启动。运行时按当前点位
所在的连续 segment 选择 transition，多个楼层的连接边保留 `path_step_index`
顺序，不把多跳路线压缩成一个未经验证的直达目标。

每条 connector transition 还绑定唯一的 `terrain_guard` 身份：
`profile_id`、`corridor_version`、走廊 `width_m/lookahead_m`、`motion_policy` 和数据来源。它只绑定 106
本地点云影子感知的版本，不把几何阈值复制到 104；路线编辑接口不能把自身标成
已认证运动。未标定路线的 corridor 保持为空。新路线默认是 `shadow-v1 + stop_only`，感知和楼梯执行验收完成前，
统一计划仍不能越过 `stair_execution_retired`。

## 从 DDDMR 借鉴什么

DDDMR Navigation（BSD-3-Clause）真正值得借鉴的是数据流和职责边界，而不是
把它整套 ROS 2 Humble/Docker 导航栈搬进本工程：

1. 用静态地面/可通行图表达地图，不把所有三维点云直接当成二维障碍物。
2. 把实时点云处理成动态层：障碍物标记/清除、限速区和禁行区，而不是改变
   原始地图或复制一套 Nav2。
3. 全局路线、局部安全和运动控制分开，感知层只提供有界、可解释的状态。

## M20Pro 的落地方式

```text
106 原厂 /LIDAR/POINTS
  -> 106 本地楼梯走廊三维分析（terrain_guard，平地不经过此分支）
  -> 轻量 terrain safety / speed-limit 状态
  -> 104 统一导航计划执行器
  -> 平地 Nav2 或经验证的连接器执行
```

106 不向 104 重新传输原始点云，现有 `/scan` 平地避障链保持不变。DDDMR
思路的第一阶段只作为影子感知和计划约束输入，不接管 `/scan`、Nav2 代价地图、
103 TCP 控制或原厂定位。任何未知、过期或矛盾的地形状态都只能阻止连接边
执行，不能让平地导航失效或猜测放行。

## 本分支实施顺序

1. `unified_navigation_contract.py`：统一验证 `n=1` 和 `n>1` 的计划形状，
   先在无 ROS 环境中建立回归测试。
2. [已完成] 将普通任务和跨楼层 API 的任务编排统一适配到同一个计划，保留旧
   字段只用于兼容投影，不再新增 `single_floor`/`cross_floor` 两套执行器。
3. [已完成影子合同] 在 106 通过现有 `m20pro_navigation` 包提供只读
   `terrain_guard_106` 节点：只对显式楼梯连接边的局部走廊分析原厂点云，输出
   台阶连续性、局部高障碍和数据新鲜度；默认不进入运动链。节点用固定上限的
   确定性抽样和定时评估隔离厂商点云回调，避免重复计算。
4. [已完成语义接线] `stair_executor` 适配器将连接边身份和 terrain 请求表示为
   reducer 动作，并将 106 状态送回同一个 reducer；唯一
   `stair_action_orchestrator` 把请求/释放动作发布到 106，避免执行器旁路产生第二套
   副作用。准备阶段等待初始点云，运动阶段 fail-closed。两者仍默认关闭。
5. [已完成动作编排合同] `stair_action_orchestrator` 是语义动作到既有
   `/m20pro/floor_goal`、`/m20pro/terrain_guard/request`、
   `/m20pro/floor_switch_request` 和 `/m20pro/stop_task`
   的唯一适配边界。步态和连接器运动只生成 `dispatchable=false` 意图；未认证
   路线在 Web 下发前即以 `stair_execution_retired` 失败，防止 floor_manager
   收到无法执行的跨层普通目标。
6. 使用 `ros2 run m20pro_navigation terrain_guard_replay` 对 `/LIDAR/POINTS` 录包
   或 JSONL 帧进行统一回放，验证 terrain guard 的误报/漏报、状态转移和评估耗时；
   再结合 106 现场 CPU/内存观测决定是否把
   状态接入连接边安全门。这个状态不能替换平地 `/scan`，也不能改变 Nav2
   的代价地图输入。
7. 只有实测通过后，才实现楼梯连接边执行；普通楼层段仍使用当前已验证的
   `/scan` + Nav2 链路。坡道若未来需要三维判定，也必须作为单独版本化连接边
   配置，不能把楼梯算法默认扩展到所有平地。

楼梯执行器的第一版合同已经落在 `m20pro_navigation/stair_executor_contract.py`：
它是一个纯状态 reducer，只输出 `dispatch_entry_goal`、`set_gait`、`stop`、
`request_floor_switch` 和 `dispatch_exit_goal` 等语义动作，不直接发布 `cmd_vel`
或修改地图。默认 `shadow-v1/stop_only` 连接边在创建时返回
`stair_execution_retired`；只有路线、profile、点云状态和现场认证全部通过，
动作编排器才会把安全动作接入现有切层事务。步态和楼梯运动仍必须由单独
验收后的运动适配器消费，不能把语义意图当作已经可以行走。

对应的 `stair_executor` 与 `stair_action_orchestrator` ROS JSON 适配器默认
`enabled=false`，当前不由 real launch 拉起；前者订阅连接边启动/事件并发布动作
和状态 JSON，后者消费安全动作并回送 Nav2/切层事件。入口和出口继续复用现有
`/m20pro/floor_goal`，阶段身份与 `floor_goal` 回执标签分开保存；Web 在连接边期间不消费
内部 Nav2 回执。执行器以 1 秒心跳维持阶段活性并由 reducer 执行阶段超时，Web 不再用
旧普通跨层目标超时提前截断连接边。多跳计划从中转层继续剩余边，不要求中转层存在
任务点。启用前必须先完成
106 影子录包、共享平台切图故障注入、原厂步态和现有仲裁验收，不能因为节点能
启动就认为跨楼层已可用。

## 明确不做

- 不把 DDDMR 的 ROS1/ROS2 Humble 专用包、Docker、TensorRT 或自定义定位器
  直接放进 104/106 生产启动。
- 不因为开始跨楼层开发就修改 `main`、现有 `/scan` 或当前主机网络。
- 不在没有录包和真机验收前启用三维状态对运动的硬控制。
