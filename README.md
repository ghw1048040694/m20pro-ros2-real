# M20 Pro ROS 2 Navigation Project

这是面向云深处 M20 Pro 的 ROS 2 二次开发 workspace。当前工程重点是：

- 在 104 通用主机上运行 Nav2、楼层管理、感知融合和网页看板；
- 对接 103 AOS 官方 TCP JSON 协议，读取位姿/状态并下发轴指令、步态切换和重定位；
- 使用 106 建图结果中的 `occ_grid.yaml`/`occ_grid.pgm` 做 2D 全局导航；
- 在仿真中使用 PGM + PCD 模拟局部雷达点云，并闭环验证 F19/F20/F21 跨楼层导航；
- 接入 YOLOv8/RKNN 巡检检测，并把导航、地图、路径、检测状态通过本地网页展示。

## 版权与边界

本项目是面向已购 M20 Pro 设备的非官方 ROS 2 集成工程，不是云深处/DEEP Robotics 官方项目。

- `M20 Pro`、`DEEP Robotics`、`云深处` 及相关产品名称归原权利方所有。
- 厂商开发手册、官方 URDF、STL mesh、官方示例控制台代码和官方协议说明归原权利方所有；它们在本项目中仅用于设备适配、调试和学习。
- 本项目自行编写的 ROS 2 桥接、仿真、融合、地图编辑、Nav2 bringup、巡检检测和网页看板代码按各 package 的 `package.xml` 声明使用。
- 真实地图、点云、现场数据、模型权重和手册 PDF 不建议直接公开。公开仓库前应确认授权，或替换为脱敏示例数据。

## 主机与数据流

- 103 AOS：默认 `10.21.31.103:30001`，提供官方本体监控 TCP 协议。工程通过它查询地图位姿、定位/避障状态，并下发轴指令、步态切换、重定位等命令。
- 106 NOS：负责原厂建图、定位、导航、避障。地图通常位于 `/var/opt/robot/data/maps/active`，可复制 `occ_grid.yaml`、`occ_grid.pgm`、`full_cloud.pcd` 到 104 或本 workspace。
- 104 GOS：运行本工程。推荐统一入口 `m20pro.launch.py`，用 `mode:=sim` 或 `mode:=real` 区分仿真/真机。

当前导航主链路：

```text
PGM 地图 -> nav2_map_server -> Nav2 全局/局部规划
真机点云或仿真点云 -> pointcloud_fusion -> /scan -> Nav2 costmap
Nav2 /cmd_vel -> tcp_bridge -> 103 官方轴指令
floor_manager -> 地图切换、楼层重定位、步态切换、跨楼层目标续航
```

## Package 总览

| Package | 用途 | 常用入口 |
| --- | --- | --- |
| `m20pro_bringup` | launch、参数、地图、RViz 配置 | `m20pro.launch.py`、`m20pro_sim.launch.py`、`m20pro_real.launch.py` |
| `m20pro_navigation` | 真机 TCP 桥、仿真桥、点云融合、楼层管理、目标点桥、配置检查、动态障碍物 | `tcp_bridge`、`sim_bridge`、`floor_manager`、`floor_goal_bridge`、`system_check` |
| `m20pro_description` | M20 Pro URDF 和 meshes | 由 bringup 自动加载 |
| `m20pro_inspection` | YOLOv8/RKNN 巡检检测 | `m20pro_inspection.launch.py` |
| `m20pro_cloud_bridge` | 本地网页看板/后续云端上报雏形 | `web_dashboard` |

## 编译

104 真机一般是 ROS 2 Foxy，上位机可用 Humble。根据机器环境选择一个 source：

```bash
source /opt/ros/foxy/setup.bash       # 104 主机
# source /opt/ros/humble/setup.bash   # 上位机仿真调试

rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

只改了某几个包时可以局部编译：

```bash
colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install
source install/setup.bash
```

## 104 真机移植状态

当前项目已经完成一次向 M20 Pro 104 GOS 主机的实机移植验证，状态如下：

- 连接方式：开发电脑通过 RJ45 直连 104，开发电脑网口配置为 `10.21.31.200/24`，104 地址为 `10.21.31.104`。
- 104 环境：Ubuntu 20.04.6、ARM64、ROS 2 Foxy，具备 `colcon`、`rosdep`、`git` 和 Python 3.8。
- 代码位置：已把当前 workspace 部署到 104 的 `/home/user/m20pro_ros2_ws_20260529_173921`。
- 编译状态：`m20pro_description`、`m20pro_navigation`、`m20pro_cloud_bridge`、`m20pro_inspection`、`m20pro_bringup` 五个包已经在 104 上编译通过。
- 依赖状态：104 原本缺少 Nav2 和 `sensor_msgs_py`，已通过离线 deb 包方式安装 Foxy/ARM64 版本。
- 已验证 Nav2 包：`nav2_bringup`、`nav2_map_server`、`nav2_lifecycle_manager`、`nav2_controller`、`nav2_planner`、`nav2_navfn_planner`、`nav2_dwb_controller`、`nav2_msgs`、`sensor_msgs_py`。
- 安全策略：104 上默认保持 `enable_axis_command:=false`，先观察地图、点云、TF、costmap 和 Nav2 状态，不直接驱动机器狗运动。

104 上的安全启动命令：

```bash
cd ~/m20pro_ros2_ws_20260529_173921
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=real \
  rviz:=false \
  enable_web_dashboard:=false \
  enable_axis_command:=false \
  cloud_topic:=/LIDAR/POINTS
```

本次移植后的烟雾测试结果：

- Nav2 map server、lifecycle manager、controller、planner、recoveries、BT navigator、waypoint follower 可以启动；
- `floor_manager`、`floor_goal_bridge`、`pointcloud_fusion`、`tcp_bridge` 可以进入 real 启动链路；
- map server 可以加载当前 F20 的 `occ_grid.yaml/pgm`；
- TCP bridge 可以连接 103 的官方控制协议端口；
- 106 侧 multicast-relay 启动顺序修复后，104 可以稳定看到 `/LIDAR/POINTS`；
- `/LIDAR/POINTS` 已接入 `pointcloud_fusion` 并转换为 `/scan`，real 模式下 `/scan` 进入 Nav2 local costmap 后可以产生 lethal/inflated 障碍单元；
- real 模式下保留原厂 `map_pose` 的 z 信息，同时把给 Nav2 使用的 `/odom` 和 `odom -> m20pro_base_link` TF 压平到 z=0，避免 2D costmap 被楼层高度干扰。

因此，当前真机侧已经不是“Nav2 缺包/工作区无法编译”的状态，而是进入“低速实机闭环验证”的阶段。打开 `enable_axis_command:=true` 前，必须先确认 `/LIDAR/POINTS`、`/scan`、`/map`、`/local_costmap/costmap`、`/global_costmap/costmap`、TF 高度和 Nav2 lifecycle 都稳定。

## 快速启动

仿真：

```bash
source install/setup.bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=sim
```

真机：

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=real
```

仍然可以直接用旧入口 `m20pro_sim.launch.py` 和 `m20pro_real.launch.py`，但日常建议记统一入口。

本地网页看板会默认随 sim/real 启动，浏览器打开：

```text
http://localhost:8080
```

如果在 104 上启动，其他同网段设备访问：

```text
http://104的IP:8080
```

换端口：

```bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=sim web_dashboard_port:=18080
```

网页操作台会随 sim/real 默认启动。它现在不是单纯看板，而是一套现场流程入口：

- 建图向导：记录项目、建筑、单层/多层模式、楼层编号；
- 106 地图拉取：把 106 原厂 active map 复制归档到 104；
- 地图选择：在网页中选择实时 `/map` 或归档地图；
- 地图标点：在 PGM 地图上点击生成巡检点、楼梯切换点、充电点等；
- 任务编排：用点位生成巡检任务，并向 `/m20pro/floor_goal` 下发楼层目标；
- 实时看板：显示楼层、位姿、路径、动态障碍物、YOLO 检测和事件。

关闭网页看板：

```bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=sim enable_web_dashboard:=false
```

## m20pro_bringup

`m20pro_bringup` 负责把导航、地图、楼层管理、RViz、网页看板等节点组合起来。

主要文件：

- `config/m20pro.yaml`：真机 TCP、仿真初始位姿、点云融合、动态障碍物、PCD 感知仿真等参数。
- `config/map_manifest.yaml`：PGM/PCD 地图资产总表，用来收拢 F19/F20/F21 的地图、点云和高度范围。
- `config/nav2_params.yaml`：Humble/上位机 Nav2 参数。
- `config/nav2_params_foxy.yaml`：104/Foxy Nav2 参数。
- `config/inspection_waypoints.yaml`：F19/F20/F21 楼层、楼梯路线、巡检点模板。
- `maps/F19`、`maps/F20`、`maps/F21`：当前多楼层 PGM 地图。
- `maps/Original_map/full_cloud.pcd`：仿真局部点云来源。
- `rviz/m20pro_sim.rviz`：RViz 配置。

### 仿真 launch

```bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=sim
```

常用参数：

```bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=sim \
  initial_floor:=F20 \
  map:=/path/to/occ_grid.yaml \
  rviz:=true \
  enable_dynamic_obstacles:=true \
  enable_web_dashboard:=true \
  web_dashboard_port:=8080 \
  web_dashboard_map_archive_dir:=~/m20pro_maps
```

当前要求：动态障碍物默认必须显示，`enable_dynamic_obstacles` 默认是 `true`。

### 真机 launch

```bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=real
```

常用参数：

```bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=real \
  cloud_topic:=/LIDAR/POINTS \
  initial_floor:=F20 \
  rviz:=true \
  enable_axis_command:=false \
  enable_web_dashboard:=true \
  web_dashboard_map_archive_dir:=~/m20pro_maps
```

说明：

- real 模式下 `cloud_topic` 默认使用 `/LIDAR/POINTS`；sim 模式下仍由仿真点云链路提供 `/cloud_nav`。
- `enable_axis_command:=false` 用于只观察链路、不向真机下发轴指令。
- `enable_initialpose_relocalization` 默认开启，RViz 的 `2D Pose Estimate` 会触发厂商重定位接口。
- `enable_initialpose_3d_adapter:=true` 时，会把普通 `/initialpose` 补上当前楼层 z 值后转发到 `/m20pro/initialpose_3d`。
- `config_audit` 会在启动时检查地图总表和楼层路线配置。
- `system_check` 会持续检查地图、点云、`/scan`、代价地图、Nav2 和楼层管理是否正常；real 模式还会检查 `/scan` 内容、local costmap 标障结果和 TF 高度。

### 独立网页看板

如果不想启动完整 sim/real，只想打开网页状态面板：

```bash
ros2 launch m20pro_bringup m20pro_web_dashboard.launch.py port:=8080
```

常用参数：

```bash
ros2 launch m20pro_bringup m20pro_web_dashboard.launch.py \
  port:=8080 \
  data_dir:=~/.m20pro_web \
  map_archive_dir:=~/m20pro_maps \
  factory_host:=10.21.31.106 \
  factory_user:=user \
  factory_active_map:=/var/opt/robot/data/maps/active
```

说明：`factory_mapping_start_command`、`factory_mapping_finish_command`、`factory_mapping_cancel_command` 是 106 原厂建图命令钩子，可按现场需要覆盖。

默认命令已按《山猫 M20 Pro 软件使用手册 V0.0.1》配置为：

```bash
sudo drmap mapping -s -n <地图名称>
sudo drmap stop_mapping
```

网页节点会通过 SSH 在 106/NOS 上执行它们。要让按钮直接可用，需要 104 能免密 SSH 到 `user@10.21.31.106`，并且 106 上 `sudo drmap ...` 不需要交互输入密码。若现场暂时没配好，仍可在 106 上手动执行 `drmap`，再回到网页点击“从 106 拉到 104”。

## m20pro_navigation

`m20pro_navigation` 是核心功能包，包含真机桥接、仿真桥接、点云处理、多楼层切换和辅助工具。

### tcp_bridge

真机 TCP 桥，连接 103 AOS：

```bash
ros2 run m20pro_navigation tcp_bridge --ros-args --params-file src/m20pro_bringup/config/m20pro.yaml
```

实际使用时通常由 `m20pro_real.launch.py` 启动，不需要单独运行。

主要功能：

- 连接 `10.21.31.103:30001`；
- 发布 `/m20pro_tcp_bridge/map_pose`、`/odom`、TF；
- 发布定位、避障、原始状态、重定位结果、步态切换结果；
- 订阅 `/cmd_vel` 并按 20Hz 转成厂商轴指令；
- 订阅 `/m20pro/gait_command` 并转成厂商步态切换；
- 订阅 `/initialpose` 或 `/m20pro/initialpose_3d` 执行厂商定位初始化。

常用状态：

```bash
ros2 topic echo /m20pro_tcp_bridge/map_pose
ros2 topic echo /m20pro_tcp_bridge/localization_ok
ros2 topic echo /m20pro_tcp_bridge/obstacle_active
ros2 topic echo /m20pro_tcp_bridge/relocalization_result
ros2 topic echo /m20pro_tcp_bridge/gait_result
```

### sim_bridge

仿真运动学桥。它不连接真机，只根据 `/cmd_vel` 更新虚拟位姿，并发布与真机相同的位姿话题，方便 Nav2 闭环测试。

通常由 `m20pro_sim.launch.py` 启动。初始位置在 `m20pro.yaml` 的 `m20pro_tcp_bridge` 参数段中配置：

```yaml
initial_x: -5.0
initial_y: 0.0
initial_yaw: 0.0
```

注意：仿真里 executable 是 `sim_bridge`，但节点名故意叫 `m20pro_tcp_bridge`，这样 sim 和 real 的话题名保持一致。

### pointcloud_fusion

点云转 2D 激光：

```text
/cloud_nav 或 /LIDAR/POINTS -> pointcloud_fusion -> /scan
```

Nav2 costmap 主要消费 `/scan`。真机和仿真都走这条链路。

### dual_lidar_simulator

仿真点云发生器。它使用 `Original_map/full_cloud.pcd`，按机器人当前位置裁剪局部点云，并叠加动态障碍物，发布：

- `/cloud_nav`
- `/grid_map_3d`
- 可选 `/LIDAR/FRONT/POINTS`
- 可选 `/LIDAR/REAR/POINTS`

默认不发布前后调试点云，避免 RViz 卡顿。需要时：

```bash
ros2 param set /m20pro_dual_lidar_simulator publish_debug_lidars true
```

### dynamic_obstacle_simulator

仿真动态障碍物，默认随 sim 启动并显示：

- `/dynamic_obstacles`
- `/dynamic_obstacle_markers`
- `/dynamic_obstacle_active`

RViz 中主要看 `/dynamic_obstacle_markers`。

### floor_manager

多楼层管理节点。当前按 X30 风格的“共享楼梯平台”语义改造，但仍保持 M20Pro 自己的接口。

跨楼层流程：

```text
收到 /m20pro/floor_goal
-> 导航到 entry
-> 发布 stair_up 或 stair_down 步态
-> 导航到 source_platform
-> 切换地图
-> 在目标楼层 target_platform 重定位
-> 仍用楼梯步态导航到 post_exit
-> 发布 flat 步态
-> 继续导航到最终目标点
```

发布跨楼层目标：

```bash
ros2 topic pub --once /m20pro/floor_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'F21'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

其中 `header.frame_id` 写目标楼层，例如 `F19`、`F20`、`F21`。

楼层状态：

```bash
ros2 topic echo /m20pro/current_floor
ros2 topic echo /m20pro/stair_status
ros2 topic echo /m20pro/gait_command
```

RViz 里不能只靠普通 `2D Goal Pose` 表达“目标楼层”，因为普通目标没有楼层字段。当前提供了楼层专用 RViz 目标话题：

- `/m20pro/rviz_goal_current`
- `/m20pro/rviz_goal_f19`
- `/m20pro/rviz_goal_f20`
- `/m20pro/rviz_goal_f21`

默认 RViz 配置已经放了这些目标工具：

- `当前楼层目标`：发布到 `/m20pro/rviz_goal_current`，由目标点桥补成当前楼层目标；
- `19楼目标`：发布到 `/m20pro/rviz_goal_f19`；
- `20楼目标`：发布到 `/m20pro/rviz_goal_f20`；
- `21楼目标`：发布到 `/m20pro/rviz_goal_f21`。

使用 `19楼目标`、`20楼目标`、`21楼目标` 点目标时，`floor_manager` 会根据工具对应的话题补齐目标楼层，再自动执行跨楼层路线。当前这是基于标准 RViz `SetGoal` 工具实现的，不依赖云深处 X30 Pro 的 3D RViz 插件。

命令行也可以用更短的字符串目标：

```bash
ros2 topic pub --once /m20pro/goal_command std_msgs/msg/String "{data: 'F21 2.0 0.0 0.0'}"
```

也可以直接发送巡检点 id，例如：

```bash
ros2 topic pub --once /m20pro/goal_command std_msgs/msg/String "{data: 'f21_demo_check'}"
```

### initialpose_3d_adapter

把 RViz 的 `/initialpose` 补上当前楼层 z 值，转发为 `/m20pro/initialpose_3d`。主要用于真机多楼层重定位实验：

```bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=real enable_initialpose_3d_adapter:=true
```

### map_editor

PGM 地图编辑器：

```bash
ros2 run m20pro_navigation map_editor
```

指定地图：

```bash
ros2 run m20pro_navigation map_editor /path/to/occ_grid.yaml
```

支持黑白画笔、画笔半径、缩放和另存为新地图目录。编辑 PGM 只影响 2D 栅格导航，不会自动同步修改 PCD；如果要做 PGM/PCD 一致性评估，应尽量保持二者来自同一轮建图数据。

### control_gui

官方 TCP 协议 Tk 控制台：

```bash
ros2 run m20pro_navigation control_gui
```

用于查询状态、发送轴指令、单点导航、定位/避障状态测试。

### floor_goal_bridge

目标点桥。它把 RViz 当前楼层目标、楼层专用目标和简单字符串命令统一转换为 `/m20pro/floor_goal`，避免用户手写很长的 `PoseStamped`。

常用命令：

```bash
ros2 topic pub --once /m20pro/goal_command std_msgs/msg/String "{data: 'F21 2.0 0.0 0.0'}"
ros2 topic pub --once /m20pro/goal_command std_msgs/msg/String "{data: 'f20_demo_check'}"
```

### system_check 和 config_audit

`config_audit` 是启动前配置检查，检查 `map_manifest.yaml`、`inspection_waypoints.yaml` 和实际地图文件是否能对上。

`system_check` 是运行时健康检查，随 sim/real 启动。用于检查 `/map`、点云、`/scan`、costmap、robot model、Nav2 lifecycle、楼层管理和动态障碍物是否正常。real 模式额外检查 `/scan` 是否有有效束、近距离障碍出现时 local costmap 是否真的标障、`odom -> m20pro_base_link` 的 z 是否保持在合理范围内。仿真启动正常时会看到类似：

```text
M20PRO SIM OK: required topics, nodes, maps and Nav2 are active
```

## m20pro_description

`m20pro_description` 存放 M20 Pro URDF 和 meshes。一般不需要手动启动，由 bringup 自动加载：

```text
m20pro_bringup launch -> zero_joint_state_publisher -> robot_state_publisher -> RViz RobotModel
```

如果 RViz RobotModel 报错，优先检查：

```bash
ros2 topic echo --once /robot_description
ros2 run tf2_ros tf2_echo map base_link
```

大多数情况下，模型显示异常不是 mesh 丢失，而是启动早期 TF/costmap 尚未稳定。

## m20pro_inspection

`m20pro_inspection` 负责 YOLOv8/RKNN 巡检检测。

默认前广角相机：

```text
rtsp://10.21.31.103:8554/video1
```

后广角相机：

```text
rtsp://10.21.31.103:8554/video2
```

启动前摄：

```bash
ros2 launch m20pro_inspection m20pro_inspection.launch.py
```

启动后摄：

```bash
ros2 launch m20pro_inspection m20pro_inspection.launch.py \
  camera_name:=rear_wide \
  rtsp_url:=rtsp://10.21.31.103:8554/video2
```

RK3588 真机建议使用 `.rknn` 模型。默认模型路径：

```text
src/m20pro_inspection/models/inspection.rknn
```

如果要用上位机 ONNX 调试：

```bash
ros2 launch m20pro_inspection m20pro_inspection.launch.py \
  backend:=onnx \
  model_path:=/path/to/best.onnx
```

输出话题：

```bash
ros2 topic echo /m20pro_yolov8_inspection/detections
ros2 topic echo /m20pro_yolov8_inspection/events
```

`detections` 是 JSON 字符串，包含相机名、图片尺寸、检测数量、类别、置信度和 bbox。`events` 用于异常事件上报。当前 YOLO 不参与底层避障安全闭环，主要用于巡检记录和告警。

## m20pro_cloud_bridge

`m20pro_cloud_bridge` 是 M20Pro 现场网页操作台和后续云端上报的基础包。它在 104 上启动一个轻量 HTTP 服务，安卓手柄浏览器、SSH 端口转发、本地电脑或甲方服务器反向代理访问的都是同一套页面。

独立启动：

```bash
ros2 launch m20pro_bringup m20pro_web_dashboard.launch.py port:=8080
```

浏览器：

```text
http://localhost:8080
```

接口：

```bash
curl http://localhost:8080/healthz
curl http://localhost:8080/api/state
curl http://localhost:8080/api/map
curl http://localhost:8080/api/maps
curl http://localhost:8080/api/annotations
curl http://localhost:8080/api/tasks
```

页面当前包含：

- `看板`：当前楼层、楼梯状态、步态、机器人位姿、PGM 地图、路径、动态障碍物、YOLO 检测/事件和话题状态；
- `建图`：建立单层/多层建图任务，预留 106 原厂建图启动/保存命令钩子，从 106 active map 拉取地图到 104；
- `地图`：管理 104 归档地图并选择当前显示地图；
- `标点`：点击地图生成巡检点、步态切换点、楼层切换点、出楼梯点、充电点、过渡点；
- `任务`：用标点生成任务，并按顺序向 `/m20pro/floor_goal` 发布楼层目标。

持久化数据默认放在 `~/.m20pro_web`，归档地图默认放在 `~/m20pro_maps`。拉取 106 地图依赖 104 能通过 `scp` 访问 `user@10.21.31.106:/var/opt/robot/data/maps/active`。

当前不直接修改 106 的 active map。前端拉到 104 的地图是工作副本，用于 104 Nav2、标点和任务编排；106 仍保留原厂建图/定位链路。

## 多楼层地图与目标点

当前默认楼层是：

- `F19`
- `F20`
- `F21`

对应配置在 `inspection_waypoints.yaml`。每层地图只表达本层和上下半层楼梯区域：

- 右侧楼梯区域表示上楼；
- 左侧楼梯区域表示下楼；
- 楼梯中间黑线用于阻断不符合实际的“就近穿越”；
- 楼层切换点设置在半层平台附近；
- 切换后从目标楼层相反一侧平台出来，再走到 `post_exit` 切回平地步态。

跨楼层发点示例，从当前楼层去 F21 的 `(2, 0)`：

```bash
ros2 topic pub --once /m20pro/floor_goal geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'F21'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

获取 RViz 中点选的 x/y/yaw：

```bash
ros2 topic echo /m20pro/rviz_goal_current
```

如果使用楼层专用 RViz 目标话题，则 echo 对应话题：

```bash
ros2 topic echo /m20pro/rviz_goal_f21
```

项目默认 RViz 工具栏已经提供：

- `当前楼层目标`
- `19楼目标`
- `20楼目标`
- `21楼目标`

其中 `19楼目标`、`20楼目标`、`21楼目标` 用于直接在 RViz 中下达跨楼层目标。

## 地图与点云放置

推荐结构：

```text
src/m20pro_bringup/maps/
  F19/
    occ_grid.yaml
    occ_grid.pgm
  F20/
    occ_grid.yaml
    occ_grid.pgm
  F21/
    occ_grid.yaml
    occ_grid.pgm
  Original_map/
    full_cloud.pcd
```

地图资产总表在：

```text
src/m20pro_bringup/config/map_manifest.yaml
```

路线、楼梯点、巡检点仍在：

```text
src/m20pro_bringup/config/inspection_waypoints.yaml
```

真机运行主要依赖 PGM/YAML 做 Nav2 全局地图；实时点云用于局部 costmap。仿真中 PCD 用来生成局部点云，更贴近真机雷达输入，但当前没有做完整 3D 全局路径规划。

从 106 复制地图示例：

```bash
scp -r user@10.21.31.106:/var/opt/robot/data/maps/active "$HOME/m20pro_active_map"
```

指定地图启动：

```bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=real \
  map:=$HOME/m20pro_active_map/occ_grid.yaml
```

## 常用 ROS 调试命令

看话题是否存在：

```bash
ros2 topic list | sort
```

看点云发布者：

```bash
ros2 topic info -v /LIDAR/POINTS
ros2 topic hz /LIDAR/POINTS
```

看 Nav2/costmap：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 topic echo --once /local_costmap/costmap
ros2 topic echo --once /map
```

看 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

清 costmap：

```bash
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
```

杀掉残留 RViz：

```bash
pkill -f rviz2
```

## 离线诊断采集

机器狗无法联网时，可以把只读采集脚本复制到 104 或 106，现场运行后把压缩包带回有网电脑分析：

```bash
bash tools/collect_ros_snapshot.sh
```

如果只复制单个脚本到机器狗：

```bash
bash collect_ros_snapshot.sh
```

脚本会生成：

```text
m20pro_ros_snapshot_<host>_<time>.tar.gz
```

它会采集 ROS topic/node/service/action 列表、关键话题频率、节点参数、TF、进程、网络环境和 106 地图目录信息。脚本只读，不会发运动命令、不会重启服务、不会修改地图。

## 部署注意

- 手册建议轴指令 20Hz，本工程默认按 20Hz 向 103 发送 `Type=2, Command=21`。
- 楼梯/平地步态按手册 `Type=2, Command=23` 执行；`flat` 转成 `GaitParam=1`，`stair_up`/`stair_down` 转成 `GaitParam=14`。
- 厂商导航任务和本工程 Nav2 轴指令不要混着抢控制权。使用本工程导航时，建议由 Nav2 负责 `/cmd_vel`，不要同时在手柄里执行原厂任务。
- 默认没有开启心跳主动上报，避免主动报文和请求响应共用 TCP 连接时串包。需要时可在 `m20pro.yaml` 中把 `send_heartbeat` 改为 `true`。
- Foxy/Humble 跨主机 DDS 通信时要统一 `ROS_DOMAIN_ID`，并确认 multicast relay、网络路由和防火墙配置。若 DDS 不稳定，优先在 104 上运行本工程。
- 真机第一次测试建议先 `enable_axis_command:=false` 观察位姿、点云、地图、costmap 是否正确，再打开轴指令。
