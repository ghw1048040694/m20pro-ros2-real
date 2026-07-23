# 106 楼梯三维安全感知影子节点

`terrain_guard_106` 是运行在 106 的只读 ROS 2 适配器。它把 106 本机的
`/LIDAR/POINTS` 转成局部楼梯走廊的结构化状态，供后续专用楼梯执行器读取。

实现位于 `src/m20pro_navigation/m20pro_navigation/`；本目录只保留运行说明和请求示例，
不再保留一份可独立运行的副本。

这不是平地导航链路，也不是运动控制节点：

- 平地仍然是 `106 /LIDAR/POINTS -> edge scan -> /scan -> 104 Nav2`。
- 本节点不发布 `/scan`、`/cmd_vel`、步态或姿态命令。
- 本节点不向 104 发送原始点云，只发布少量 JSON 状态。
- `traversable` 只代表剖面证据完整；`permit_motion` 和 `certified_motion` 默认始终为 `false`。
- 点云过期、坐标系不一致、走廊缺口、横向覆盖不足、步高异常或方向混乱时均 fail-closed。

## 运行

在已经具备 ROS 2 Foxy 和厂商点云消息的 106 上构建并运行：

```bash
colcon build --packages-select m20pro_navigation --symlink-install
source install/setup.bash
ros2 run m20pro_navigation terrain_guard_106
```

节点默认最多使用每帧 `30000` 个点，按点云存储顺序确定性抽样；可通过
`--ros-args -p max_points:=20000` 调整。原始点数、抽样点数、实际有效点数和
抽样步长会随状态一起发布，便于在 106 上核对 CPU 与延迟。

## 离线回放

在上位机或 104 上可以不启动节点，直接复盘 PointCloud2 录包：

```bash
ros2 run m20pro_navigation terrain_guard_replay \
  ~/bags/stairs_bag \
  --request tools/terrain_guard_106/request_example.json \
  --topic /LIDAR/POINTS \
  --without-records --json
```

也可以使用不依赖 ROS 中间件的 JSON/JSONL 帧文件。每帧是一个对象，至少包含
`points: [[x, y, z], ...]`，可选 `stamp_s` 和 `cloud_age_s`。回放输出每个状态和
原因的计数、状态转移、可通行/阻塞比例以及平均评估耗时；它只调用纯合同，不发布
任何 ROS 话题或运动命令。

默认话题：

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 输入 | `/LIDAR/POINTS` | `sensor_msgs/msg/PointCloud2` |
| 输入 | `/m20pro/terrain_guard/request` | `std_msgs/msg/String` |
| 输出 | `/m20pro/terrain_guard/status` | `std_msgs/msg/String` |

请求是 JSON，必须包含矩形走廊和方向。例如：

`request_id`、`route_id`、`profile_id` 和 `corridor_version` 是必需身份字段；缺失任一
字段时节点只返回 `unknown`，不会沿用上一条请求的结果。

```json
{
  "enabled": true,
  "request_id": "shadow-request-001",
  "route_id": "stairs-a-up",
  "profile_id": "stairs-a-up:terrain",
  "corridor_version": "corridor-v1",
  "direction": "forward",
  "corridor": {
    "width_m": 1.0,
    "lookahead_m": 2.4,
    "bin_size_m": 0.12,
    "min_lateral_span_m": 0.4,
    "min_step_height_m": 0.05,
    "max_step_height_m": 0.24,
    "obstacle_height_m": 0.22,
    "min_points_per_bin": 4,
    "min_step_count": 2,
    "min_coverage": 0.55
  }
}
```

停止影子检查时发布 `{"enabled": false}`。请求只能由后续已经通过路线、地图和共享平台事务校验的楼梯执行器生成；当前还没有把它接入真实运动。

## 当前状态

这是阶段 3 的影子感知组件。需要在真实楼梯上用录包验证空楼梯、人员遮挡、箱体、缺口、上楼和下楼后，才能设计执行器闭环；在此之前不得移除 `stair_execution_retired`，不得把 `permit_motion` 改为 `true`。
