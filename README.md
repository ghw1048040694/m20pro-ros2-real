# M20 Pro ROS 2 跨楼层巡检导航系统

这是面向云深处 M20 Pro 的 ROS 2 二次开发 workspace。当前主要用于：

- 在 104 通用主机上运行 Nav2、楼层管理、点云融合和网页操作台；
- 使用 106 原厂建图结果做 2D 导航地图，支持跨楼层地图、楼梯语义和巡检任务编排；
- 对接 103 官方 TCP JSON 协议读取位姿/状态，并在允许时下发运动控制；
- 支持单楼层巡检、多楼层地图切换、巡检点编排、YOLO 检测、U360 雷达巡检和现场录包。

详细调试日志、实测记录、问题排查过程放在：

```text
m20pro日志.md
```

单层导航架构、模块边界和实习生分工放在：

```text
docs/single_floor_navigation_architecture.md
```

正式经典前端、甲方前端和外部功能包对接 API 契约放在：

```text
docs/frontend_api_contract.md
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
colcon build --packages-select \
  m20pro_bringup \
  m20pro_cloud_bridge \
  m20pro_navigation \
  m20pro_inspection \
  m20pro_radar_inspection \
  --symlink-install
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
./scripts/104_check_edge_scan.sh
./scripts/104_status.sh
./scripts/104_start_web.sh                 # 仅开发预览网页，不用于真机测试
./scripts/104_stop_web.sh
```

## 现场参数统一配置

现场允许调整的点云、楼梯安全、楼梯过渡、Nav2 和定位稳定性参数只有一个源文件：

```text
src/m20pro_bringup/config/m20pro_field_profile.yaml
```

修改后必须先检查再整体应用：

```bash
./scripts/apply_field_profile.sh --check
./scripts/apply_field_profile.sh
```

应用入口会拒绝任务执行期间换参，先校验字段、范围和参数耦合，再按完整部署流程同时更新 106 和 104。106 的 `/etc/m20pro-edge-scan-106.env` 是自动生成物，不可手工编辑；Nav2 参数文件中的现场项也是不可直接运行的占位符，只能由该配置启动时重写。104 和 106 会携带并核对同一份配置的 SHA-256，不一致时拒绝进入楼梯模式，不提供旧参数回退或热更新。

当前 schema v3 共开放 74 个现场参数，其中 `navigation` 28 个、`teleoperation` 7 个；导航按 `controller / goal / progress / local_planner / costmap / global_planner` 分组，遥控统一配置速度上限、指令租约和仲裁看门狗。另有 `scan`、`stair`、`stair_safety`、`stair_transition` 和 `localization`。插件类型、话题、坐标系、机器人 footprint、行为树结构和固定安全开关仍属于工程架构，不允许从现场配置改变。

导航减速度在统一配置中填写正数幅值，运行时自动转换成 DWB 要求的负值；同一最大线速度同时驱动 `max_vel_x/max_speed_xy`，同一障碍、清障和膨胀参数同时驱动局部与全局代价地图。校验器会拒绝停止阈值大于最大速度、发布频率高于刷新频率、清障范围小于障碍范围以及定位阈值相互冲突等组合。

`stair.max_step_height_m` 是感知分类阈值，不代表机器狗的机械爬升能力。现场提高它之前必须先确认厂家给出的步态能力并用录包和有人看护的低风险测试验证；不能靠改大参数让机器狗强行通过超出物理能力的台阶。

在上位机拉回 104 录包：

```bash
./scripts/local_pull_bags.sh
```

现场真机测试只走全量 real 启动：`104_start_real_shadow.sh` 或 `104_start_real_move.sh`。全量 real 会同时拉起 tcp_bridge、Nav2 和网页前端；二维激光唯一来源是 106 edge scan 发布的 `/scan`。

任务前必须在网页自检、`/api/state` 或 `104_check_edge_scan.sh` 中确认 106 edge scan 输出的 `/scan` 新鲜、frame 为 `m20pro_base_link` 且有效距离不少于 20。感知不通时网页仍可用，但 Nav2 启动门和任务运行保护会禁止运动。

104 正式主栈固定使用 `project_udp` profile，只接收 106 发布的轻量 `/scan`，不再订阅跨主机原始点云。开机脚本只会用 `fuser` 保护性清理没有进程占用的陈旧 `fastrtps_*` SHM 文件。

`104_start_real_move.sh` 会放开运动控制，只能在现场有人看护、手柄急停可用时执行。启动后打开网页，等待顶部状态栏“自检”从“启动中”更新为自动自检结果。服务启动 12 秒后会执行首次基础自检，之后每 300 秒自动刷新，用于确认全量系统、网页、106 edge scan 和原厂状态链路；定位、Nav2 生命周期和代价地图需要到测试场地重定位后再确认。`104_start_web.sh` 只用于开发预览网页界面，不能作为现场任务流程。

## 开机自启动

104 可以安装 `m20pro-real.service`，开机后自动启动全量 real 系统和网页前端，但不会自动执行任务。手柄或笔记本连上机器狗网络后访问 `http://10.21.31.104:8080`，先确认顶部状态栏的自动自检结果；到测试场地后从顶部“定位”进入重定位，导航项恢复正常后再标点和开始任务。

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

自启动只消费 `/scan`。点云未就绪时，网页仍应起来并在顶部“点云/自检”状态和任务流程中显示感知链路故障；恢复时先用 `./scripts/104_check_edge_scan.sh` 确认样本。默认环境变量应保持：

```text
M20PRO_FASTDDS_PROFILE=project_udp
M20PRO_CLEAN_STALE_FASTDDS_SHM=1
M20PRO_SCAN_TOPIC=/scan
```

YOLO 模型在 x86_64 上位机由 `best.pt` 导出 ONNX，再用 RKNN Toolkit2 编译为 RK3588 FP16 模型。104 只安装 RKNNLite 和 `librknnrt`，不安装或运行 Torch。正式配置为：

```text
M20PRO_ENABLE_INSPECTION=true
M20PRO_INSPECTION_BACKEND=rknn
M20PRO_INSPECTION_MODEL_PATH=/home/user/m20pro_real_ros2_ws/install/m20pro_inspection/share/m20pro_inspection/models/best_rk3588_fp16.rknn
M20PRO_INSPECTION_CLASS_NAMES_PATH=/home/user/m20pro_real_ros2_ws/install/m20pro_inspection/share/m20pro_inspection/models/labels_zh.txt
```

转换环境固定为 Ultralytics 8.3.40、CPU Torch 2.4.1、ONNX 1.16.1 和 RKNN Toolkit2 2.3.2，执行 `scripts/convert_yolo_to_rknn.py` 可重复生成模型。104 的 YOLO 进程默认休眠，前端先打开前摄像头，再启用 YOLO；进程启动 RTSP 最新帧线程和 NPU 推理，线程持续排空 30fps 源流且只保留最新帧，NPU 启用后按 3Hz 消费，避免推理积压旧画面。原始 H.264 视频仍保持 30fps 硬件解码，不额外转码降帧；检测页把检测框用 Canvas 叠加在同一个 H.264 视频上，并以轻量检测接口刷新 JSON，不再传输第二路 ROS Image/MJPEG。当前类别为：未戴安全帽、未穿安全背心、跌倒、火灾、现场杂乱、配电箱打开。

104 重装运行时使用 `scripts/104_install_rknn_runtime.sh`。脚本默认从 `/tmp` 读取 RKNNLite 2.3.2 arm64 wheel、`librknnrt.so` 和模型，安装后会执行一次 NPU 初始化检查。

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

正常作业时，浏览器访问 `http://10.21.31.104:8080`，等待顶部状态栏显示自动基础自检结果。基础自检失败时先处理基础链路；如果基础自检通过但提示“导航待重定位后确认”，从顶部“定位”进入重定位，再看定位、`/scan`、代价地图和 Nav2 状态是否恢复正常。

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

1. `建图`：记录项目、建筑、单层/多层和现场实际楼层；单层填写 `7`、`F7` 或 `B1`，多楼层填写 `7,8,9`，系统会规范化并登记到当前项目。地图名称可留空，由系统生成带楼层和时间的唯一名称。
2. `建图`：点击“启动 106 建图”会检查 106、创建全新建图会话并执行原厂 `drmap mapping -b`。已保存、已拉取或已取消的历史会话不会作为新任务复用；页面进度只按本次会话的真实返回推进。
3. `建图`：完成现场采集后点击“完成/保存建图”，再从“拉取 106 建图结果”把同名最新原厂地图包归档到 104。保存、拉取和切换是三个独立动作，建图完成不会自动用于导航。
4. 顶部 `地图`：查看当前地图，或在展开浮层中切换要查看和标点的楼层地图；这一步才会把可用的 106 原厂地图包 apply 成 active。
5. 左侧大地图：查看当前单层 2D 栅格地图、机器人位姿、任务点和路径；“修饰地图”会另存新版本、复制点位和单地图任务，绝不覆盖原地图，也不会自动切换。
6. 顶部 `定位`：如果机器人位置不准，打开浮层后在地图上拖箭头并执行网页重定位。楼层由当前固定地图自动确定，重定位界面不再要求人工选楼层。
7. `标点`：在地图上按住并拖出箭头，保存任务点、过渡点、充电点及楼梯语义点。
8. `建图 -> 跨楼层路线`：从两层地图中依次选择起始层入口、起始层切换点、目标层切换点和目标层出口，保存一条有向路线；返程必须另存一条反向路线。
9. `任务`：单层任务按当前地图编排；跨层任务在“跨楼层编排”中按实际执行顺序加入不同楼层任务点，路线预览通过后生成任务。
10. `任务`：点击开始执行，必要时点击停止当前任务。

当前网页一次显示一张单层地图。跨楼层任务通过点位楼层和持久化有向路线交给 `floor_manager`；到达共享楼梯平台后，Web 统一执行 104 Nav2 地图切换、106 `drmap apply` 和目标层 2101 重定位，三项全部确认后才更新当前楼层并继续任务。失败会停止任务并回滚两端地图；回滚不能确认时楼层状态会置为未知，禁止把一次地图显示变化误报成跨层成功。楼梯运动前必须由 106 原始三维点云连续确认通道净空；楼梯阶段 Nav2 只使用剔除正常台阶后的异常障碍扫描，并使用禁止倒退、旋转和清图恢复的专用行为树。净空为 `blocked/unknown`、数据超过 1 秒未更新或心跳租约失效时立即停止，不把二维 `/scan` 中的台阶轮廓直接当作普通障碍。地图可在地图浮层删除；当前地图、任务执行期间和跨楼层路线引用的地图受保护。删除普通地图会同步清理 104 上的点位、任务和建图会话引用，不会删除独立保存的雷达历史结果，也不会自动删除 106 原厂地图包。需要清理 106 时必须先确认 `active` 不指向目标包，再精确删除对应原厂目录。

仓库内置的 `F19`、`F20`、`F21` 只用于跨楼层验证模板，不是现场建图的固定楼层范围。其中 `F19` 和 `F21` 目前仍复用编辑后的 `F20` 地图产品，不能作为其他工地的真实楼层。新建图任务必须填写现场实际楼层；登记新楼层只建立项目元数据，不是普通地图库白名单，也不会自动生成楼梯/电梯路线。普通地图按自身 `floor` 身份选择、标点和重定位；跨楼层路线仍需使用实测坐标单独配置并严格校验注册楼层。

多楼层建图时，“现场实际楼层”表示本次项目要建立的全部楼层集合，不是机器狗实时所在楼层。系统按输入顺序把第一层设为初始建图步骤；会话建立后，只在建图进度和逐层步骤中显示当前层，表单不再提供另一个“当前建图楼层”输入。完成并拉取当前层后，再从真实步骤列表切换下一层。

完整现场流程、四点含义、上下行配置和失败边界见 [docs/cross_floor_navigation.md](docs/cross_floor_navigation.md)。

当前前端只保留单层 2D 地图任务操作。3D 地图展示、3D/楼梯点云 HTTP 接口不在单层导航闭环关键路径中，已经从前端/网页 API 移除。前/后摄像头画面仍保留为按需打开的现场辅助视图，但旧的摄像头全开关和诊断面板已移除。内部 `/m20pro/stair_zones` 发布仍保留，供后续跨楼层逻辑复用。

## 视频画面

网页上的前/后摄像头来自 103 的 RTSP：

```text
rtsp://10.21.31.103:8554/video1
rtsp://10.21.31.103:8554/video2
```

103 使用 Rockchip MPP 硬件编码 H.264 Constrained Baseline（1280x720、30 fps、GOP 15）。104 的独立 MediaMTX 网关按需拉取 RTSP，只做封装转换，不解码、不缩放、不转 JPEG；网页通过低延迟 HLS 播放 H.264，由浏览器硬件解码。生产链路为：

```text
103 camera -> Rockchip H.264 -> RTSP -> 104 MediaMTX remux -> LL-HLS -> browser
```

前端只在点击“打开”后创建 Hls.js 播放器，关闭或切换相机时立即销毁播放器并移除视频源，使 104 网关停止无消费者的上游拉流。播放器启用低延迟同步、短缓冲和直播点追赶；`web_dashboard` 的 `enable_camera_proxy` 必须保持为 `false`，生产进程中不应存在摄像头 FFmpeg/MJPEG 转码。

如果现场仍加载旧页面，先强制刷新网页，推荐 `Ctrl+F5`，确认浏览器加载的是当前版本脚本：

```text
/static/dashboard.js?v=20260710-camera-h264
```

如果强刷新后仍然有明显长延时，优先检查 103 源码流、编码 GOP、104 网关和网络链路：

```bash
systemctl status m20pro-camera-webrtc-104.service
curl http://127.0.0.1:8888/video1/index.m3u8
pgrep -af '[f]fmpeg .*rtsp://10.21.31.103' || true
```

YOLO 独立消费同一条 H.264 RTSP，在推理进程内只解码一次；网页只接收检测 JSON 并在同一条原始视频上绘制边框，禁止从网页帧或 MJPEG 代理取图。当前 MediaMTX v1.4.2 的 WebRTC 与现代 Chrome 的 ICE 协商不可靠，因此生产前端使用已验证的低延迟 HLS；升级网关并通过真实浏览器 ICE 验证后才可切换 WebRTC。

网页重定位以顶部“定位”浮层的最终结论为准：`/initialpose` 只是网页侧触发动作，开发手册 TCP `2101/1` 回执也只是必要证据之一，不能单独算成功。现场必须看到“重定位成功”，任务区才允许开始任务。执行任务时不能重定位；需要先停止当前任务，再打开顶部“定位”，在地图上拖箭头并点击 `执行重定位`。

如果网页箭头方向看起来不对，不要直接改 180 度偏置。先检查：

1. 顶部状态栏里的 `定位` 是否正常。
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

网页标点以地图箭头为唯一坐标源：按下位置作为点位，向机器狗到点后应面对的方向拖动，松开后自动形成 `x/y/yaw`；也可以直接采用当前机器人实时位姿。页面只显示取点结果，不再提供容易与地图草稿不一致的手工坐标输入。

按《山猫 M20 系列软件开发手册》：

```text
PointInfo=0  过渡点
PointInfo=1  任务点
PointInfo=3  充电点
Gait=12      平地敏捷步态
Gait=14      爬楼梯步态
Speed=1      低速
Manner=0     前进行走
ObsMode=0    开启停避障
NavMode=1    自主导航
```

默认策略：

- 任务点：`PointInfo=1`，默认停留 `5s`；
- 过渡点：`PointInfo=0`，默认停留 `0s`；
- 充电点：`PointInfo=3`，必须放在任务最后。
- 爬楼梯点默认 `Gait=14`，出楼梯点及普通点默认 `Gait=12`；正式跨楼层执行仍由 `floor_manager` 根据已配置路线决定 `stair_up/stair_down`，点位名称不能替代楼层路线配置。
- 所有点位固定 `Manner=0`、`ObsMode=0`、`NavMode=1`；前端和外部 API 都不能关闭避障或切回直线导航，只保留步态和速度为可配置字段。

任务执行时会发布：

```text
/m20pro/floor_goal
/m20pro/active_waypoint
/m20pro/stop_task
```

`/m20pro/active_waypoint` 是轻量 JSON，包含当前任务阶段、目标位姿、剩余停留时间和 `waypoint` 点位语义。昂锐雷达检测节点应优先使用这里的 `waypoint.result_file_prefix`、`waypoint.room`、`waypoint.scan_point` 和 `waypoint.radar.scans` 命名并区分扫描结果。

## 网页人工接管

状态栏“遥控”用于自主任务无法继续时的低速脱困。点击“人工接管”会先终止当前任务，再将速度仲裁器从自主导航切到遥控；结束接管后保持运动锁定，旧任务不会自动恢复，只有重新开始任务才会再次放行 Nav2。

遥控指令只在 `move` 模式且速度仲裁器就绪时生效。按键松开立即发零速度；浏览器失焦、页面隐藏、网络中断或心跳超过 `teleoperation.command_timeout_s` 时自动停车并锁定导航。楼梯三维感知会话活跃时禁止网页遥控，台阶上脱困必须使用原厂手柄并由现场人员看护。

Nav2、遥控和最终执行速度分别使用 `/cmd_vel_nav`、`/cmd_vel_teleop` 和 `/cmd_vel`；只有 `m20pro_command_mux` 可以向最终 `/cmd_vel` 输出。未部署 VPN、身份认证和访问控制前，不得将这组运动 API 直接暴露到公网。

雷达扫描计划示例：

```json
{
  "waypoint": {
    "building": "3栋",
    "unit": "1单元",
    "house": "2008户",
    "floor": "F20",
    "room": "客厅",
    "scan_point": "P01",
    "result_file_prefix": "3栋_1单元_2008户_F20_客厅_P01",
    "radar": {
      "enabled": true,
      "scans": [
        {"mode": "measuring", "label": "实测实量", "result_suffix": "measure"},
        {"mode": "modeling", "label": "点云建模", "result_suffix": "cloud", "artifact_policy": "manual_import", "manual_measure_required": true}
      ]
    }
  }
}
```

## U360 雷达巡检

雷达巡检包默认不启动；需要接入 U360RSE/UCL360 时，在全量 real 启动前设置环境变量：

```bash
export M20PRO_ENABLE_RADAR_INSPECTION=true
export M20PRO_RADAR_BACKEND=u360_http
export M20PRO_RADAR_SCAN_MODE=measuring      # 或 modeling
export M20PRO_RADAR_DEVICE_URL=http://192.168.107.72:8080
export M20PRO_RADAR_OUTPUT_DIR=/home/user/m20pro_radar_results
./scripts/104_start_real_move.sh
```

节点订阅 `/m20pro/active_waypoint`，任务点进入 `dwelling` 阶段后触发扫描，并发布：

```text
/m20pro/radar_inspection/status
/m20pro/radar_inspection/result
/m20pro/radar_inspection/events
```

网页标点时可以为每个任务点选择“不触发雷达 / 仅实测实量 / 仅点云建模 / 实测实量 + 点云建模”。测量模式下默认在 U360 进入 `analyzing` 后允许机器人去下一个点，最终结果继续后台收集；点云建模模式如果设备不能直接返回点云文件，会把该次扫描标记为 `pending_import`，由人工从雷达导出点云工程后在任务卡片点击“登记点云”填入路径。

网页“检测”页会显示最近一次雷达扫描和结构化测量结果；任务列表按任务点展示对应雷达结果、点云登记状态和人工回填状态。任务卡片提供 `雷达JSON` / `雷达CSV` 导出，也可以直接访问：

```bash
curl -o radar_task.json "http://127.0.0.1:8080/api/radar/task_export?task_id=<task_id>&format=json"
curl -o radar_task.csv  "http://127.0.0.1:8080/api/radar/task_export?task_id=<task_id>&format=csv"
```

人工登记接口：

```bash
curl -X POST http://127.0.0.1:8080/api/radar/artifact \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"<task_id>","run_id":"<run_id>","artifact_path":"/path/to/u360/project"}'

curl -X POST http://127.0.0.1:8080/api/radar/manual_measurement \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"<task_id>","run_id":"<run_id>","measurements":[{"name":"开关高度","value":"1.32m"}]}'
```

## 真机测试顺序

当前现场测试顺序：

1. 本工程 real 影子测试：全量启动，不放开运动控制，确认点云、定位、地图、路径、网页状态。
2. 同楼层真导航：使用 `104_start_real_move.sh` 全量启动，基础自检通过后在测试场地重定位；导航项正常后再做短距离、长距离和避障连续测试。
3. 跨楼层静态验收：确认每层只有一张路线正式地图、上下行有向路线齐全、四点均来自对应地图、起始地图和重定位已确认。
4. 楼梯感知静态验收：在入口静止观察任务状态，空楼梯应连续通过净空检查；在前方放置高于踏面的箱体或由人员遮挡时必须显示阻塞并禁止起步。现场必须清除低于单级台阶高度、外形与台阶相似的物体，不能要求单一几何雷达可靠区分二者。
5. 跨楼层真导航：先单次相邻楼层空载测试，再做返程，最后按任务点顺序做多楼层巡检；任何一步只要 104/106 地图、定位或楼梯净空未确认就停止。

任务 2 和任务 5 必须录包。出现原地转圈、明显偏航、贴障碍物、路径穿墙、地图和当前位置明显不匹配时，立即点击网页停止任务或使用手柄急停。

## 关键文件

```text
src/m20pro_bringup/config/m20pro_real.yaml            # 104 真机基础参数
src/m20pro_bringup/config/nav2_params_real.yaml       # 104/Foxy 真机 Nav2 参数
src/m20pro_bringup/config/map_manifest.yaml           # 地图资产总表
src/m20pro_bringup/config/runtime_navigation.yaml     # 默认无路线运行时配置
docs/archived_route_profiles/legacy_inspection_waypoints_f19_f20_f21.yaml  # 仅供显式迁移/历史路线参考
docs/cross_floor_navigation.md                        # 正式跨楼层建图、配置和测试流程
src/m20pro_bringup/maps/                              # PGM 地图
src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py
src/m20pro_cloud_bridge/m20pro_cloud_bridge/static/   # 前端 HTML/CSS/JS
src/m20pro_radar_inspection/                          # U360 雷达巡检节点
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
| `m20pro_navigation` | TCP 桥、楼层管理、目标桥、健康检查 |
| `m20pro_cloud_bridge` | 网页操作台 |
| `m20pro_inspection` | YOLOv8/RKNN 巡检检测 |
| `m20pro_radar_inspection` | U360 雷达任务点扫描与结果导出 |

说明：real-only 仓库已移除 `m20pro_description` 和 sim 启动链路。真机运行使用 `m20pro_base_link` 主链路，不再依赖 URDF/mesh、`robot_state_publisher` 或零关节发布器；仿真模型资源留在 sim 仓库维护。

## 常用检查命令

edge scan：

```bash
./scripts/104_check_edge_scan.sh
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

网页建图默认按软件使用手册使用 `drmap mapping -b -s -n <map_name>`：只建图，不立即激活为导航地图。每次新建图必须创建新的会话；终态历史会话不能再次启动，留空名称会自动生成唯一名称。保存后，前端只按本次会话的 `<map_name>-日期-时间` 在 `/var/opt/robot/data/maps` 下查找最新同名地图包并拉到 104；只有在前端选择/切换该地图时，才会对有真实 `source_path` 的地图调用 `drmap apply`。

地图修饰是 104 上的版本化操作：原 `yaml/pgm` 不修改，新版本使用独立地图 ID 和目录，记录 `parent_map_id`，并复制原地图点位及可独立执行的单地图任务。修饰结果只影响 104/Nav2 二维栅格，不伪造 106 原厂 `source_path`；保存后必须由操作者确认并切换到新版本，验证无误后才能删除旧版本。

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
