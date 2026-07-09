# M20 Pro ROS 2 巡检导航系统

这是面向云深处 M20 Pro 的 ROS 2 二次开发 workspace。当前主要用于：

- 在 104 通用主机上运行 Nav2、楼层管理、点云融合和网页操作台；
- 使用 106 原厂建图结果做 2D 导航地图，并保留面向后续跨楼层的楼梯语义基础；
- 对接 103 官方 TCP JSON 协议读取位姿/状态，并在允许时下发运动控制；
- 支持单楼层巡检、多楼层地图切换、巡检点编排、YOLO 检测和现场录包。

详细调试日志、实测记录、问题排查过程放在：

```text
m20pro日志.md
```

单层导航架构、模块边界和实习生分工放在：

```text
docs/single_floor_navigation_architecture.md
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
/home/user/m20pro_real_ros2_ws
```

104 上进入环境的固定顺序：

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
cd /home/user/m20pro_real_ros2_ws
source install/setup.bash
```

## 编译

104/Foxy：

```bash
source /opt/robot/scripts/setup_ros2.sh
cd /home/user/m20pro_real_ros2_ws
source install/setup.bash
colcon build --packages-select m20pro_bringup m20pro_cloud_bridge m20pro_navigation --symlink-install
source install/setup.bash
```

上位机只做 real 代码构建检查：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

说明：本仓库现在按 real-only 维护，模型/URDF/mesh 不放在这里；需要仿真时使用单独的 sim 仓库。

## 现场主入口

现场人员直接使用仓库根目录 `scripts/`，不要和 `src/m20pro_bringup/scripts/` 混用。

```bash
./scripts/104_start_real_shadow.sh         # 启动 real，不放开运动控制
./scripts/104_start_real_move.sh           # 启动 real，放开运动控制
./scripts/104_preflight_check.sh move      # 终端备用自检，网页自检异常时使用
./scripts/104_stop_real.sh                 # 停止 real
./scripts/104_record_bag.sh 180 m20_test   # 录包
./scripts/104_enable_autostart.sh move     # 安装开机自启动全量 real
./scripts/104_autostart_status.sh          # 查看自启动服务状态
./scripts/104_disable_autostart.sh         # 关闭并移除自启动服务
```

诊断和开发入口只在排查时使用：

```bash
./scripts/104_diagnose_preflight.sh
./scripts/104_check_lidar.sh
./scripts/104_status.sh
./scripts/104_start_web.sh                 # 仅开发预览网页，不用于真机测试
./scripts/104_stop_web.sh
```

在上位机拉回 104 录包：

```bash
./scripts/local_pull_bags.sh
```

现场真机测试只走全量 real 启动：`104_start_real_shadow.sh` 或 `104_start_real_move.sh`。全量 real 会同时拉起 tcp_bridge、Nav2、点云融合和网页前端。

全量 real 启动会先拉起点云 relay，但默认不等待 `/LIDAR/POINTS` 样本才启动 Nav2 和网页。原因是现场需要网页可用来显示 `perception_status`、地图、定位和任务状态。只看到 topic 名或 publisher count 不算感知通过；任务前在网页自检、`/api/state` 或 `104_check_lidar.sh` 中确认 `/LIDAR/POINTS -> lidar_relay -> /scan` 链路恢复。此时只停止本工程 real stack，不要手动清 `/dev/shm/fastrtps_*`，不要从本工程脚本重启原厂 multicast/lidar 服务。

104 现场默认保持原厂 FastDDS 口径：主栈、Nav2、网页、点云融合和原始点云 relay 均使用 factory profile。2026-07-07 已实测：`project_udp` 主栈 + factory relay 的混合配置虽然能降低 `/dev/shm`，但会让 relay 状态新鲜而 `/scan` 断流，不能作为正式链路默认值。项目内 `m20pro_fastdds_udp.xml` 只保留为 DDS/SHM 专项实验或只读诊断用；strict UDP-only 已实测无法稳定订阅原始 `/LIDAR/POINTS`，不要作为正式链路切换。开机脚本只会用 `fuser` 保护性清理没有进程占用的陈旧 `fastrtps_*` SHM 文件，不清正在使用的通信段。

`104_start_real_move.sh` 会放开运动控制，只能在现场有人看护、手柄急停可用时执行。启动后打开网页，在“自检”页点一次“开机基础自检”。基础自检用于确认全量系统、网页、原始点云和原厂状态链路；电量只在界面显示给操作员参考，不作为软件自检或任务启动条件；定位、`/scan`、Nav2 生命周期和代价地图需要到测试场地重定位后再确认。`104_start_web.sh` 只用于开发预览网页界面，不会拉起 tcp_bridge/Nav2/点云融合，不能作为重定位、标点、下发任务的现场流程。

## 开机自启动

104 可以安装 `m20pro-real.service`，开机后自动启动全量 real 系统和网页前端，但不会自动执行任务。手柄或笔记本连上机器狗网络后访问 `http://10.21.31.104:8080`，先在网页“自检”页点一次“开机基础自检”，确认基础链路正常；到测试场地后再做网页重定位，导航项恢复正常后再标点和开始任务。

安装自启动：

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
cd /home/user/m20pro_real_ros2_ws
./scripts/104_enable_autostart.sh move
systemctl start m20pro-real.service
./scripts/104_autostart_status.sh
```

停用自启动：

```bash
su
cd /home/user/m20pro_real_ros2_ws
./scripts/104_disable_autostart.sh
```

自启动服务只启动本工程，不修改原厂 multicast/FastDDS 服务。`move` 模式会把运动控制链路准备好，但任务仍必须在网页中人工点击开始。

自启动同样会先启动点云 relay，但默认不因 `/LIDAR/POINTS` 暂时无样本而阻塞网页。点云未就绪时，网页仍应起来并在自检/任务页显示感知链路故障；恢复时先用网页状态或 `./scripts/104_check_lidar.sh` 确认样本，再继续重定位、标点和任务启动。默认环境变量应保持：

```text
M20PRO_FASTDDS_PROFILE=factory
M20PRO_LIDAR_RELAY_FASTDDS_PROFILE=factory
M20PRO_CLEAN_STALE_FASTDDS_SHM=1
```

## 现场复盘

现场任务问题统一用 rosbag 复盘，不再依赖前端 watcher、ready-check 或失败快照脚本。

104 上录包：

```bash
./scripts/104_record_bag.sh 180 field_task
```

上位机拉回录包：

```bash
./scripts/local_pull_bags.sh
```

`m20pro_real_full.sh move` 会在 `/tmp` 生成运行时参数文件，把 `m20pro_tcp_bridge.enable_axis_command` 明确覆盖为 `true`；`shadow` 会覆盖为 `false`。这样不会改动原始 `m20pro_real.yaml`，也能避免 Foxy 中节点专属参数压过 launch 参数的问题。

## 启动方式

真机测试推荐用脚本启动。开一个 104 终端，按固定环境顺序进入后执行：

```bash
./scripts/104_start_real_shadow.sh
```

影子模式用于看定位、地图、点云、路径和网页任务逻辑，不放开运动控制。

确认现场安全、手柄急停可用后，运动模式执行：

```bash
./scripts/104_start_real_move.sh
```

终端自检脚本作为备用排查工具。网页自检异常、或现场需要保存终端输出时再另开 104 终端执行：

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
cd /home/user/m20pro_real_ros2_ws
source install/setup.bash
./scripts/104_preflight_check.sh move
```

正常作业时，浏览器访问 `http://10.21.31.104:8080`，在网页“自检”页点一次“开机基础自检”。基础自检失败时先处理基础链路；如果基础自检通过但提示“导航待重定位后确认”，先在测试场地完成网页重定位，再看定位、`/scan`、代价地图和 Nav2 状态是否恢复正常。

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
2. `建图`：用 106 原厂 `drmap mapping -b` 建图，或手动在 106 上建图；建图保存后不立即用于导航。
3. `地图`：选择项目内置地图，或按建图名称把 106 地图包拉取到 104 归档。
4. `地图`：切换要查看和标点的楼层地图；这一步才会把可用的 106 原厂地图包 apply 成 active。
5. 左侧大地图：查看当前单层 2D 栅格地图、机器人位姿、任务点和路径。
6. `定位`：如果机器人位置不准，在地图上拖箭头并执行网页重定位。
7. `标点`：在地图上按住并拖出箭头，保存巡检点、过渡点、充电点。
8. `任务`：勾选点位生成任务。
9. `任务`：点击开始执行，必要时点击停止当前任务。

当前网页一次显示一张单层地图。跨楼层任务通过点位携带的楼层字段和 `/m20pro/floor_goal` 交给 `floor_manager` 处理。

当前项目内置地图入口有 `F19`、`F20`、`F21`。其中 `F19` 和 `F21` 目前仍复用编辑后的 `F20` 地图产品，真实交付前应替换为各楼层实测建图结果。

当前前端只保留单层 2D 地图任务操作。3D 地图展示、3D/楼梯点云 HTTP 接口不在单层导航闭环关键路径中，已经从前端/网页 API 移除。前/后摄像头画面仍保留为按需打开的现场辅助视图，但旧的摄像头全开关和诊断面板已移除。内部 `/m20pro/stair_zones` 发布仍保留，供后续跨楼层逻辑复用。

## 视频画面

网页上的前/后摄像头来自 103 的 RTSP：

```text
rtsp://10.21.31.103:8554/video1
rtsp://10.21.31.103:8554/video2
```

104 的 `web_dashboard` 默认用 FFmpeg 拉流并转成 MJPEG，前端只在点击“打开”后才建立视频连接。当前前端不会直接把 `/camera/front.mjpg` 塞给 `<img>` 让浏览器自行缓存，而是用 `fetch(...).body.getReader()` 读取 MJPEG 长连接，按 `Content-Length` 解析每帧 JPEG，并通过 latest-frame-only 方式显示：浏览器来不及显示的旧帧会被丢弃，只保留最新帧进入绘制。

如果现场感觉视频有 5 秒以上延时，先强制刷新网页，推荐 `Ctrl+F5`，确认浏览器加载的是带版本号的前端脚本：

```text
/static/dashboard.js?v=20260630-video-latest
```

如果强刷新后仍然有明显长延时，优先检查 103 源码流、RTSP 服务端缓存、编码 GOP 或网络链路。104 上可以先确认代理空闲和当前后端配置：

```bash
curl http://127.0.0.1:8080/api/state | python3 -m json.tool
pgrep -af '[f]fmpeg .*rtsp://10.21.31.103' || true
```

停止观看视频后，`camera_proxy.cameras.front.clients` 应回到 `0`，`running=false`，并且不应残留 FFmpeg 拉流进程。要继续接近原厂手柄的低延时手感，下一步应考虑 WebRTC 或 H.264/H.265 硬解链路，而不是继续在 MJPEG 显示层反复调参。

网页重定位以作业页重定位区最终结论为准：`/initialpose` 只是网页侧触发动作，开发手册 TCP `2101/1` 回执也只是必要证据之一，不能单独算成功。现场必须在作业页看到“重定位成功”，任务区才允许开始任务。执行任务时不能重定位；需要先停止当前任务，再到作业页重定位区拖箭头并点击 `执行重定位`。

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

`/m20pro/active_waypoint` 是轻量 JSON，包含当前任务阶段、目标位姿、剩余停留时间和 `waypoint` 点位语义。昂锐雷达检测节点应优先使用这里的 `waypoint.result_file_prefix`、`waypoint.room`、`waypoint.scan_point` 和 `waypoint.radar.scans` 命名并区分扫描结果。

## 真机测试顺序

当前现场测试顺序：

1. 本工程 real 影子测试：全量启动，不放开运动控制，确认点云、定位、地图、路径、网页状态。
2. 同楼层真导航：使用 `104_start_real_move.sh` 全量启动，基础自检通过后在测试场地重定位；导航项正常后再做短距离、长距离和避障连续测试。
3. 跨楼层真导航：确认各楼层真实地图、楼梯点和原厂步态后再测。

任务 2 和任务 3 必须录包。出现原地转圈、明显偏航、贴障碍物、路径穿墙、地图和当前位置明显不匹配时，立即点击网页停止任务或使用手柄急停。

## 关键文件

```text
src/m20pro_bringup/config/m20pro_real.yaml            # 104 真机基础参数
src/m20pro_bringup/config/nav2_params_real.yaml       # 104/Foxy 真机 Nav2 参数
src/m20pro_bringup/config/map_manifest.yaml           # 地图资产总表
src/m20pro_bringup/config/inspection_waypoints.yaml   # 楼层、楼梯、巡检点模板
src/m20pro_bringup/maps/                              # PGM 地图
src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py
src/m20pro_cloud_bridge/m20pro_cloud_bridge/static/   # 前端 HTML/CSS/JS
src/m20pro_cloud_bridge/m20pro_cloud_bridge/*_contract.py
src/m20pro_navigation/m20pro_navigation/floor_manager.py
src/m20pro_navigation/m20pro_navigation/tcp_bridge_node.py
docs/single_floor_navigation_architecture.md          # 单层导航架构和拆分边界
```

拆分后的代码按功能看更容易：

- 前端界面和视频显示：`static/dashboard.html`、`static/dashboard.css`、`static/dashboard.js`。
- HTTP/ROS 集成和现场副作用：`web_dashboard_node.py`。
- 地图读取、选图、派生地图和启动同步：`map_*_contract.py`、`startup_map_sync_contract.py`。
- 重定位结论和导航 readiness：`localization_contract.py`、`navigation_readiness_contract.py`。
- 任务创建、启动、当前点推进和任务/雷达接口：`task_*_contract.py`、`active_task_contract.py`、`active_waypoint_contract.py`。
- Nav2 状态、路径终点匹配和运行进展判断：`nav_status_contract.py`、`task_plan_contract.py`、`task_progress_contract.py`。
- 感知、自检、运行时守护和 ROS 消息转换：`perception_contract.py`、`preflight_contract.py`、`web_runtime_contract.py`、`ros_message_contract.py`。

如果要继续分工开发，优先按这些功能边界改对应 contract 和测试；`web_dashboard_node.py` 主要保留实际 ROS 发布、订阅、HTTP 接口和落盘。

## Package

| Package | 作用 |
| --- | --- |
| `m20pro_bringup` | launch、参数、地图、RViz、脚本 |
| `m20pro_navigation` | TCP 桥、点云融合、楼层管理、目标桥、健康检查 |
| `m20pro_cloud_bridge` | 网页操作台 |
| `m20pro_inspection` | YOLOv8/RKNN 巡检检测 |

说明：real-only 仓库已移除 `m20pro_description` 和 sim 启动链路。真机运行使用 `m20pro_base_link` 主链路，不再依赖 URDF/mesh、`robot_state_publisher` 或零关节发布器；仿真模型资源留在 sim 仓库维护。

## 常用检查命令

点云：

```bash
./scripts/104_check_lidar.sh 12
# 或手动确认样本：
timeout 8 ros2 topic echo /LIDAR/POINTS --no-arr
```

Nav2：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
timeout 8 ros2 topic echo /local_costmap/costmap --no-arr
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

106 当前生效的原厂地图由 active 软链接指向：

```text
/var/opt/robot/data/maps/active
```

网页建图默认按软件使用手册使用 `drmap mapping -b -s -n <map_name>`：只建图，不立即激活为导航地图。保存后，前端会按 `<map_name>-日期-时间` 在 `/var/opt/robot/data/maps` 下查找最新地图包并拉到 104；只有在前端选择/切换该地图时，才会对有真实 `source_path` 的地图调用 `drmap apply`。

如果导入时报“没有生成可供前端/Nav2使用的栅格 yaml”，说明 106 的地图包还没有成功生成 `occ_grid.yaml`/`map.yaml` 这类 2D 栅格地图，不能加入前端地图列表，需要重新按原厂流程完成建图保存。

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
