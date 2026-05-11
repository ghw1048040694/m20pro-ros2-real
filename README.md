# M20 Pro ROS 2 Navigation Project

这个 workspace 面向 M20 Pro 的 104 通用主机部署，也可以先在 Humble 上位机编译调试。设计按厂商手册拆成三层：

- `m20pro_description`: 使用官方 URDF 和 meshes，mesh 路径已改成 `package://m20pro_description/...`。
- `m20pro_navigation`: ROS 2 节点。`tcp_bridge` 通过官方 TCP JSON 协议连接 103 AOS，真机部署时由 Nav2 输出 `/cmd_vel`，再由 `tcp_bridge` 转成厂商轴指令。官方 Tk demo 已集成为 `control_gui`。
- `m20pro_bringup`: 参数和 launch。

## 主机与数据流

- 103 AOS: `10.21.31.103:30001`，官方本体监控 TCP 协议。用于查询地图坐标、定位/避障状态，并下发轴指令或原生导航任务。
- 106 NOS: `10.21.31.106`，负责建图、定位、导航、避障。地图包通常在 `/var/opt/robot/data/maps/active`，可把 `occ_grid.yaml`/`occ_grid.pgm` 给 104 的 `map_server` 使用。
- 104 GOS: 运行本工程。推荐使用 `m20pro_real.launch.py` 启动 Nav2 真机链路；`/goal_pose` 由 Nav2 消费，`tcp_bridge` 默认不会再把目标转发给 103/106 原生导航。

## 编译

在 104 Foxy 或上位机 Humble 上：

```bash
cd /home/user/m20pro_ros2_ws
source /opt/ros/foxy/setup.bash       # 104 主机
# source /opt/ros/humble/setup.bash   # Humble 上位机
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

当前目录中的 workspace 路径是：

```bash
cd /home/fabu/桌面/M20Pro/m20pro_ros2_ws
```

## 运行

项目里已经内置了一张地图，路径是：

```bash
src/m20pro_bringup/maps/working_1-20260429-162852/occ_grid.yaml
```

直接启动即可：

```bash
ros2 launch m20pro_bringup m20pro_real.launch.py
```

真机默认使用手册中的 `/cloud_nav` 生成 `/scan`，并在点云 `frame_id` 不是 `base_link` 时通过 TF 转到 `base_link` 再投影。这样真机和仿真都走同一条链路：`/cloud_nav -> pointcloud_fusion -> /scan -> Nav2 costmap`。如果要改用原始雷达点云：

```bash
ros2 launch m20pro_bringup m20pro_real.launch.py cloud_topic:=/LIDAR/POINTS
```

或仿真：

```bash
ros2 launch m20pro_bringup m20pro_sim.launch.py
```

如果要换成别的地图，再从 106 导出或复制激活地图，例如：

```bash
scp -r user@10.21.31.106:/var/opt/robot/data/maps/active ~/m20pro_active_map
```

启动：

```bash
ros2 launch m20pro_bringup m20pro_real.launch.py \
  map:=$HOME/m20pro_active_map/occ_grid.yaml
```

发送一个 Nav2 目标：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

常用状态话题：

- `/m20pro_tcp_bridge/map_pose`: 103 返回的地图坐标系位姿。
- `/m20pro_tcp_bridge/localization_ok`: 定位是否正常。
- `/m20pro_tcp_bridge/obstacle_active`: 是否处于避障/停障状态。
- `/scan`: 前后雷达融合后的 2D 激光数据，供 Nav2 costmap 使用。
- `/cmd_vel`: Nav2 输出，最终由 `tcp_bridge` 归一化为厂商轴指令。

## 上位机仿真

没有 M20 Pro 真机话题时，可以用 `sim_bridge` 做 2D 运动学闭环。它会模拟发布 `/m20pro_tcp_bridge/map_pose`、`/odom`、定位正常和无障碍状态，不连接 103 主机。

```bash
source install/setup.bash
ros2 launch m20pro_bringup m20pro_sim.launch.py
```

如需切换地图：

```bash
ros2 launch m20pro_bringup m20pro_sim.launch.py \
  map:=/path/to/your/occ_grid.yaml
```

仿真 launch 会自动加载项目内的 RViz 配置，默认显示 `Map`、`RobotModel`、`TF`、`/m20pro_tcp_bridge/map_pose`、`/plan`、`/scan` 和 `/odom`。顶部工具栏可以直接用：

- `2D Goal Pose`：向 `/goal_pose` 发目标点
- `2D Pose Estimate`：向 `/initialpose` 重置仿真初始位姿

然后发目标点：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

需要重置仿真起点时，向 `/initialpose` 发布一个 `PoseStamped` 即可。

项目里也带了一个轻量级的动态障碍仿真器，会发布往返移动的圆柱障碍物，并在 RViz 里显示为 `/dynamic_obstacle_markers`。仿真雷达会把地图障碍和动态障碍一起 raycast 成 `/cloud_nav`，再由 fusion 生成 `/scan` 给 Nav2 costmap 使用。

为了贴近《开发手册》的接口，仿真里的导航主点云是：

- `/cloud_nav` (`sensor_msgs/msg/PointCloud2`, 10Hz)
- `/scan` (`sensor_msgs/msg/LaserScan`, 10Hz)

默认 RViz 只显示 `/scan`，前后原始调试点云也默认不发布，避免 RViz 被点云拖慢。如确实要看仿真的前/后雷达点云，仿真启动后打开：

```bash
ros2 param set /m20pro_dual_lidar_simulator publish_debug_lidars true
```

会额外发布 `/LIDAR/FRONT/POINTS` 和 `/LIDAR/REAR/POINTS`。

## 官方控制台

原来的 `M20Pro_ws/control.py` 已集成到本 workspace：

```bash
source install/setup.bash
ros2 run m20pro_navigation control_gui
```

它仍然按官方 demo 连接 `10.21.31.103:30001`，用于运动状态、轴指令、单点导航、定位和避障状态查询。

## 地图编辑

项目里带了一个简单的地图编辑器，可以直接处理 `occ_grid.yaml` / `occ_grid.pgm`：

```bash
source install/setup.bash
ros2 run m20pro_navigation map_editor
```

也可以指定别的地图：

```bash
ros2 run m20pro_navigation map_editor /path/to/occ_grid.yaml
```

编辑器支持：

- `Black / White` 两种画笔
- 调整画笔半径
- `+ / -` 按钮和鼠标滚轮缩放
- `Save As`：另存为新的地图目录，生成新的 `occ_grid.yaml` 和 `occ_grid.pgm`

保存后可直接拿新地图启动仿真：

```bash
ros2 launch m20pro_bringup m20pro_sim.launch.py map:=/path/to/edited_map/occ_grid.yaml
```

## 已迁移内容

旧目录 `M20Pro_ws` 中的内容已经迁入：

- `M20Pro_ws/urdf/M20.urdf` -> `src/m20pro_description/urdf/M20.urdf`
- `M20Pro_ws/meshes/*.STL` -> `src/m20pro_description/meshes/`
- `M20Pro_ws/control.py` -> `src/m20pro_navigation/m20pro_navigation/control_gui.py`

确认新 workspace 编译和运行正常后，旧的 `M20Pro_ws` 可以不再保留。

## 部署注意

- 手册建议轴指令 20Hz，本工程默认按 20Hz 向 103 发送 `Type=2, Command=21`。
- 厂商导航任务在导航模式下执行，轴指令在常规模式下执行。使用本工程的 104 路径跟随时，请确保机器人处于可接受轴指令的常规/运动状态。
- 默认没有开启心跳主动上报，避免主动报文和请求响应共用 TCP 连接时串包。需要时可在 `m20pro_bringup/config/m20pro.yaml` 中把 `send_heartbeat` 改为 `true`。
- M20 Pro 上是 Foxy，上位机是 Humble 时，跨主机 DDS 通信要统一 `ROS_DOMAIN_ID`，并确认两侧网络可互通。若 DDS 不稳定，优先在 104 上运行本工程，只从 106 复制地图文件。
