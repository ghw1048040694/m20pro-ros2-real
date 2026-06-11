# M20 Pro ROS 2 巡检导航系统

这是面向云深处 M20 Pro 的 ROS 2 二次开发 workspace。当前主要用于：

- 在 104 通用主机上运行 Nav2、楼层管理、点云融合和网页操作台；
- 使用 106 原厂建图结果做 2D 导航地图；
- 对接 103 官方 TCP JSON 协议读取位姿/状态，并在允许时下发运动控制；
- 支持单楼层巡检、多楼层地图切换、巡检点编排、YOLO 检测和现场录包。

详细调试日志、实测记录、问题排查过程放在：

```text
m20pro日志.md
```

现场执行脚本 Word 放在：

```text
/home/fabu/桌面/脚本.docx
```

## 主机分工

| 主机 | 地址 | 作用 |
| --- | --- | --- |
| 103 AOS | `10.21.31.103` | 运动控制、官方 TCP 协议、相机 RTSP |
| 104 GOS | `10.21.31.104` | 运行本工程、Nav2、网页前端 |
| 106 NOS | `10.21.31.106` | 原厂建图、定位、导航、点云发布 |

104 推荐固定工作区：

```text
/home/user/m20pro_ros2_ws
```

104 上进入环境的固定顺序：

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
cd /home/user/m20pro_ros2_ws
source install/setup.bash
```

## 编译

104/Foxy：

```bash
source /opt/robot/scripts/setup_ros2.sh
cd /home/user/m20pro_ros2_ws
source install/setup.bash
colcon build --packages-select m20pro_bringup m20pro_cloud_bridge m20pro_navigation --symlink-install
source install/setup.bash
```

上位机仿真：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 常用脚本

现场人员直接使用仓库根目录 `scripts/`，不要和 `src/m20pro_bringup/scripts/` 混用。

```bash
./scripts/104_start_real_shadow.sh         # 启动 real，不放开运动控制
./scripts/104_start_real_move.sh           # 启动 real，放开运动控制
./scripts/104_stop_real.sh                 # 停止 real
./scripts/104_record_bag.sh 180 m20_test   # 录包
./scripts/104_check_lidar.sh               # 检查 /LIDAR/POINTS
./scripts/104_status.sh                    # 查看服务状态
./scripts/104_start_web.sh                 # 仅开发预览网页，不用于真机测试
./scripts/104_stop_web.sh                  # 停止单独预览网页
```

在上位机拉回 104 录包：

```bash
./scripts/local_pull_bags.sh
```

现场真机测试只走全量 real 启动：`104_start_real_shadow.sh` 或 `104_start_real_move.sh`。全量 real 会同时拉起 tcp_bridge、Nav2、点云融合和网页前端。

`104_start_real_move.sh` 会放开运动控制，只能在现场有人看护、手柄急停可用时执行。`104_start_web.sh` 只用于开发预览网页界面，不会拉起 tcp_bridge/Nav2/点云融合，不能作为重定位、标点、下发任务的现场流程。

## 启动方式

仿真：

```bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=sim
```

真机测试推荐用脚本启动。开一个 104 终端，按固定环境顺序进入后执行：

```bash
./scripts/104_start_real_shadow.sh
```

影子模式用于看定位、地图、点云、路径和网页任务逻辑，不放开运动控制。

确认现场安全、手柄急停可用后，运动模式执行：

```bash
./scripts/104_start_real_move.sh
```

开发预览网页：

```bash
ros2 launch m20pro_bringup m20pro_web_dashboard.launch.py port:=8080
```

这只适合改前端样式或检查页面是否能打开。真机重定位、标点、下发任务必须在全量 real 启动后操作。

浏览器访问：

```text
http://10.21.31.104:8080
```

`127.0.0.1:8080` 只表示“当前这台机器自己访问自己”。前端跑在 104 上时，笔记本或手柄访问必须用 `10.21.31.104:8080`。

## 网页操作流程

真机测试时，网页前端是全量 real 系统的一部分，跑在 104 上。笔记本、手柄或调试电脑连接机器狗 WiFi/机器狗内网后访问 `http://10.21.31.104:8080`。

基本流程：

1. `建图`：记录项目、建筑、单层/多层、楼层编号。
2. `建图`：用 106 原厂 `drmap` 建图，或手动在 106 上建图。
3. `地图`：选择项目内置地图，或从 106 active map 拉取到 104 归档。
4. `地图`：切换要查看和标点的楼层地图。
5. `定位`：如果机器人位置不准，在地图上拖箭头并执行网页重定位。
6. `标点`：在地图上按住并拖出箭头，保存巡检点、过渡点、充电点、楼梯点。
7. `任务`：勾选点位生成任务。
8. `任务`：点击开始执行，必要时点击停止当前任务。

当前网页一次显示一张单层地图。跨楼层任务通过点位携带的楼层字段和 `/m20pro/floor_goal` 交给 `floor_manager` 处理。

当前项目内置地图入口有 `F19`、`F20`、`F21`。其中 `F19` 和 `F21` 目前仍复用编辑后的 `F20` 地图产品，真实交付前应替换为各楼层实测建图结果。

网页重定位通过 `/initialpose` 发布当前位置和朝向，`m20pro_tcp_bridge` 会转成 M20 Pro 原厂 `2101/1` 初始化定位请求。执行任务时不能重定位；需要先停止当前任务，再到 `定位` 页拖箭头并点击 `执行重定位`。

如果网页箭头方向看起来不对，不要直接改 180 度偏置。先检查：

1. 网页看板里的 `定位状态` 是否正常。
2. `/m20pro_tcp_bridge/map_pose` 的 yaw 是否和实际朝向一致。
3. `/ODOM` 是否有 `inf/nan` 或明显漂移。
4. 当前加载地图是否就是机器狗所在环境。

只有定位源头正确、地图正确、仅网页绘制方向不一致时，才调整前端显示逻辑。默认 `robot_pose_display_yaw_offset_rad=0.0`，不做猜测性旋转。

## 巡检点字段

巡检点不是单纯的 x/y 坐标。每个点位应明确：

| 字段 | 含义 |
| --- | --- |
| `label` | 点位名称，给现场人员和巡检报告使用 |
| `area` / `room` | 区域、房间或构件部位 |
| `result_file_prefix` | 昂锐雷达、YOLO 等检测结果落盘时使用的文件名前缀 |
| `pose.x/y/z/yaw` | 地图坐标和到点朝向，yaw 单位 rad |
| `manual_point_type` | 手册点位类型：`transition`、`task`、`charge` |
| `dwell_s` | 到点后停留秒数 |
| `vendor_navigation` | 原厂单点导航任务字段 |

网页标点时不用手动估算 yaw：在地图上按下作为点位位置，向机器狗到点后应面对的方向拖出箭头，松开后会自动填入 `x/y/yaw`。如果已有精确数值，也可以直接编辑坐标和朝向角输入框。

按《山猫 M20 系列软件开发手册》：

```text
PointInfo=0  过渡点
PointInfo=1  任务点
PointInfo=3  充电点
Gait=12      平地敏捷步态
Speed=1      低速
Manner=0     前进行走
ObsMode=0    开启停避障
NavMode=0    直线导航
NavMode=1    自主导航
```

默认策略：

- 任务点：`PointInfo=1`，默认停留 `5s`，默认 `NavMode=1`；
- 过渡点：`PointInfo=0`，默认停留 `0s`，默认 `NavMode=0`；
- 充电点：`PointInfo=3`，必须放在任务最后。

任务执行时会发布：

```text
/m20pro/floor_goal
/m20pro/active_waypoint
/m20pro/stop_task
```

`/m20pro/active_waypoint` 是 JSON，包含当前点位名称、区域、房间/部位、结果文件名前缀、点位类型、yaw、停留时间和原厂导航字段。昂锐雷达检测节点应优先使用这里的 `waypoint.result_file_prefix` 命名结果文件。

## 真机测试顺序

当前现场测试顺序：

1. 本工程 real 影子测试：全量启动，不放开运动控制，确认点云、定位、地图、路径、网页状态。
2. 同楼层真导航：短距离、长距离和避障连续测试，使用 `104_start_real_move.sh`。
3. 跨楼层真导航：确认各楼层真实地图、楼梯点和原厂步态后再测。

任务 2 和任务 3 必须录包。出现原地转圈、明显偏航、贴障碍物、路径穿墙、地图和当前位置明显不匹配时，立即点击网页停止任务或使用手柄急停。

## 关键文件

```text
src/m20pro_bringup/config/m20pro.yaml                 # 真机/仿真基础参数
src/m20pro_bringup/config/nav2_params_real.yaml       # 104/Foxy 真机 Nav2 参数
src/m20pro_bringup/config/nav2_params_sim.yaml        # 上位机仿真 Nav2 参数
src/m20pro_bringup/config/map_manifest.yaml           # 地图资产总表
src/m20pro_bringup/config/inspection_waypoints.yaml   # 楼层、楼梯、巡检点模板
src/m20pro_bringup/maps/                              # PGM 地图
src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py
src/m20pro_navigation/m20pro_navigation/floor_manager.py
src/m20pro_navigation/m20pro_navigation/tcp_bridge_node.py
```

## Package

| Package | 作用 |
| --- | --- |
| `m20pro_bringup` | launch、参数、地图、RViz、脚本 |
| `m20pro_navigation` | TCP 桥、点云融合、楼层管理、目标桥、健康检查 |
| `m20pro_cloud_bridge` | 网页操作台 |
| `m20pro_inspection` | YOLOv8/RKNN 巡检检测 |
| `m20pro_description` | URDF 和 mesh |

## 常用检查命令

点云：

```bash
ros2 topic list | grep LIDAR
ros2 topic hz /LIDAR/POINTS
ros2 topic echo /LIDAR/POINTS --no-arr
```

Nav2：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 topic echo --once /local_costmap/costmap --no-arr
```

位姿和 TF：

```bash
ros2 topic echo /m20pro_tcp_bridge/map_pose
ros2 run tf2_ros tf2_echo map m20pro_base_link
```

网页接口：

```bash
curl http://localhost:8080/healthz
curl http://localhost:8080/api/state
curl http://localhost:8080/api/tasks
```

清 costmap：

```bash
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

## 地图来源

项目内置地图总表：

```text
src/m20pro_bringup/config/map_manifest.yaml
```

106 原厂地图一般在：

```text
/var/opt/robot/data/maps/active
```

手动复制示例：

```bash
scp -r user@10.21.31.106:/var/opt/robot/data/maps/active "$HOME/m20pro_active_map"
```

指定地图启动：

```bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=real \
  map:=$HOME/m20pro_active_map/occ_grid.yaml
```

## 注意事项

- 不要把原厂导航任务和本工程 Nav2 轴指令同时用于控制机器狗。
- 真机第一次测试先用 `enable_axis_command:=false`。
- `scripts/` 是现场人工脚本入口。
- `src/m20pro_bringup/scripts/` 是 ROS package 内部脚本。
- README 只保留上手和使用说明；过程记录统一写入仓库根目录 `m20pro日志.md`。
