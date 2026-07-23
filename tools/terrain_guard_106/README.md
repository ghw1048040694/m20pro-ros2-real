# 106 楼梯三维安全感知影子节点

`m20pro_terrain_guard_106.py` 是运行在 106 的只读适配器。它把 106 本机的
`/LIDAR/POINTS` 转成局部楼梯走廊的结构化状态，供后续专用楼梯执行器读取。

这不是平地导航链路，也不是运动控制节点：

- 平地仍然是 `106 /LIDAR/POINTS -> edge scan -> /scan -> 104 Nav2`。
- 本节点不发布 `/scan`、`/cmd_vel`、步态或姿态命令。
- 本节点不向 104 发送原始点云，只发布少量 JSON 状态。
- `traversable` 只代表剖面证据完整；`permit_motion` 和 `certified_motion` 默认始终为 `false`。
- 点云过期、坐标系不一致、走廊缺口、步高异常或方向混乱时均 fail-closed。

## 运行

在已经具备 ROS 2 Foxy 和厂商点云消息的 106 上运行：

```bash
python3 tools/terrain_guard_106/m20pro_terrain_guard_106.py
```

默认话题：

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 输入 | `/LIDAR/POINTS` | `sensor_msgs/msg/PointCloud2` |
| 输入 | `/m20pro/terrain_guard/request` | `std_msgs/msg/String` |
| 输出 | `/m20pro/terrain_guard/status` | `std_msgs/msg/String` |

请求是 JSON，必须包含矩形走廊和方向。例如：

```json
{
  "enabled": true,
  "route_id": "stairs-a-up",
  "corridor_version": "corridor-v1",
  "direction": "forward",
  "corridor": {
    "width_m": 1.0,
    "lookahead_m": 2.4,
    "bin_size_m": 0.12,
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
