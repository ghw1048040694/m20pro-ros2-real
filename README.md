# M20 Pro ROS 2 Navigation Project

这个 workspace 面向 M20 Pro 的 104 通用主机部署，也可以先在 ROS 2 Humble 上位机编译调试。设计按厂商手册拆成三层：

- `m20pro_description`: 使用官方 URDF 和 meshes，mesh 路径已改成 `package://m20pro_description/...`。
- `m20pro_navigation`: ROS 2 节点。`tcp_bridge` 通过官方 TCP JSON 协议连接 103 AOS，真机部署时由 Nav2 输出 `/cmd_vel`，再由 `tcp_bridge` 转成厂商轴指令。官方 Tk demo 已集成为 `control_gui`。
- `m20pro_bringup`: 参数和 launch。

## 版权与归属

本项目是面向 M20 Pro 机器人二次开发的非官方 ROS 2 集成工程，不是云深处/DEEP Robotics 官方项目，也不代表官方技术支持或官方发布版本。

- `M20 Pro`、`DEEP Robotics`、`云深处` 及相关产品名称归原权利方所有。
- 厂商开发手册、官方 URDF、STL mesh、官方示例控制台代码和官方协议说明归原权利方所有。它们在本项目中仅用于已购设备的适配、调试和学习，不表示这些官方资源被重新授权为本项目的开源代码。
- 本项目自行编写的 ROS 2 桥接、仿真、融合、地图编辑、Nav2 bringup 和配置代码按各 package 的 `package.xml` 声明使用；厂商资源、导出的真实地图和现场数据不包含在该开源授权范围内。
- 如果将仓库公开用于求职展示或对外分发，请先确认官方资源和真实场地地图是否允许公开。无法确认时，建议移除或替换 `src/m20pro_description/meshes/`、官方 URDF、真实工地地图、手册 PDF 和任何包含现场信息的数据。

## 主机与数据流

- 103 AOS: `10.21.31.103:30001`，官方本体监控 TCP 协议。用于查询地图坐标、定位/避障状态，并下发轴指令或原生导航任务。
- 106 NOS: `10.21.31.106`，负责建图、定位、导航、避障。地图包通常在 `/var/opt/robot/data/maps/active`，可把 `occ_grid.yaml`/`occ_grid.pgm` 给 104 的 `map_server` 使用。
- 104 GOS: 运行本工程。推荐使用 `m20pro_real.launch.py` 启动 Nav2 真机链路；`/goal_pose` 由 Nav2 消费，`tcp_bridge` 默认不会再把目标转发给 103/106 原生导航。

## 编译

克隆仓库后，在 workspace 根目录执行：

```bash
source /opt/ros/foxy/setup.bash       # 104 主机
# source /opt/ros/humble/setup.bash   # Humble 上位机
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

## 运行

项目里当前内置的默认地图是新录制的 F20，F19/F21 暂时复用同一份 PGM 作为跨楼层仿真占位：

```bash
src/m20pro_bringup/maps/F20/occ_grid.yaml
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
scp -r user@10.21.31.106:/var/opt/robot/data/maps/active "$HOME/m20pro_active_map"
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
- `/m20pro_tcp_bridge/relocalization_result`: RViz `/initialpose` 转发给厂商定位初始化接口后的结果。
- `/m20pro_tcp_bridge/gait_result`: `/m20pro/gait_command` 转发为厂商步态切换接口后的结果。
- `/scan`: 前后雷达融合后的 2D 激光数据，供 Nav2 costmap 使用。
- `/cmd_vel`: Nav2 输出，最终由 `tcp_bridge` 归一化为厂商轴指令。

真机重定位：

真机 launch 默认会订阅 `/initialpose`。在 RViz 中使用 `2D Pose Estimate`，`tcp_bridge` 会按照厂商手册的定位初始化接口向 103 发送 `Type=2101, Command=1`，字段为 `PosX`、`PosY`、`PosZ`、`Yaw`。这用于把 106/厂商定位重置到你在地图上指定的位姿。

```bash
ros2 topic echo /m20pro_tcp_bridge/relocalization_result
```

如果只想观察导航链路、不希望 104 改动真机定位，可以关闭：

```bash
ros2 launch m20pro_bringup m20pro_real.launch.py \
  enable_axis_command:=false \
  enable_initialpose_relocalization:=false
```

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

需要重置仿真起点时，在 RViz 使用 `2D Pose Estimate`，或向 `/initialpose` 发布一个 `PoseWithCovarianceStamped` 即可。真机中同一个话题会触发厂商定位初始化；仿真中只是重置 `sim_bridge` 的虚拟位姿。

项目里也带了一个轻量级的动态障碍仿真器，会发布往返移动的圆柱障碍物，并在 RViz 里显示为 `/dynamic_obstacle_markers`。仿真感知默认从 `F20/full_cloud.pcd` 按机器人当前位置裁剪局部点云，再叠加动态障碍并发布为 `/cloud_nav`，随后由 fusion 生成 `/scan` 给 Nav2 costmap 使用。

为了贴近《开发手册》的接口，仿真里的导航主点云是：

- `/cloud_nav` (`sensor_msgs/msg/PointCloud2`, 10Hz)
- `/scan` (`sensor_msgs/msg/LaserScan`, 10Hz)

默认 RViz 只显示 `/scan`，前后原始调试点云也默认不发布，避免 RViz 被点云拖慢。如确实要看仿真的前/后雷达点云，仿真启动后打开：

```bash
ros2 param set /m20pro_dual_lidar_simulator publish_debug_lidars true
```

会额外发布 `/LIDAR/FRONT/POINTS` 和 `/LIDAR/REAR/POINTS`。

## 官方控制台

项目提供了一个基于官方 TCP 协议的 Tk 控制台：

```bash
source install/setup.bash
ros2 run m20pro_navigation control_gui
```

它连接 `10.21.31.103:30001`，用于运动状态、轴指令、单点导航、定位和避障状态查询。

## YOLO 巡检感知

项目新增了 `m20pro_inspection` 包，用于把自训练 YOLOv8 巡检模型接入 M20 Pro。它默认按手册中的 RTSP 地址读取前广角相机：

```text
rtsp://10.21.31.103:8554/video1
```

后广角相机为：

```text
rtsp://10.21.31.103:8554/video2
```

RK3588 真机建议使用 RKNN 模型。把转换好的模型放到：

```text
src/m20pro_inspection/models/inspection.rknn
```

板端需要安装 Rockchip 的 RKNN runtime / `rknn-toolkit-lite2`，否则节点会提示缺少 `rknnlite`。如果手里只有 YOLOv8 的 `best.pt`，先导出 ONNX，再用 RKNN-Toolkit2 转成 RK3588 的 `.rknn`。

如果需要显示类别名称，可再放一个一行一个类别名的文件：

```text
src/m20pro_inspection/models/classes.txt
```

默认 launch 会优先使用安装目录中的模型；如果没有找到，也会自动回退查找当前 workspace 下的 `src/m20pro_inspection/models/inspection.rknn` 和 `$HOME/m20pro_models/inspection.rknn`。

重新编译并启动：

```bash
colcon build --packages-select m20pro_inspection --symlink-install
source install/setup.bash
ros2 launch m20pro_inspection m20pro_inspection.launch.py
```

查看结果：

```bash
ros2 topic echo /m20pro_yolov8_inspection/detections
ros2 topic echo /m20pro_yolov8_inspection/events
```

如果要用后摄像头：

```bash
ros2 launch m20pro_inspection m20pro_inspection.launch.py \
  camera_name:=rear_wide \
  rtsp_url:=rtsp://10.21.31.103:8554/video2
```

上位机调试时也可以传入 ONNX 模型：

```bash
ros2 launch m20pro_inspection m20pro_inspection.launch.py \
  backend:=onnx \
  model_path:=/path/to/best.onnx
```

典型巡检流程是：Nav2 到达任务点后停车，YOLO 节点检测数帧，巡检任务管理器读取 `/m20pro_yolov8_inspection/events`，记录截图和异常结果，再继续下一个点位。导航避障仍然以雷达 `/cloud_nav -> /scan -> costmap` 为主，YOLO 不参与底层安全闭环。

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

## 离线诊断采集

机器狗无法联网时，可以把只读采集脚本复制到 104 或 106，现场运行后把压缩包带回有网的电脑分析：

```bash
bash tools/collect_ros_snapshot.sh
```

如果只复制单个脚本到机器狗：

```bash
bash collect_ros_snapshot.sh
```

脚本会生成类似下面的压缩包：

```text
m20pro_ros_snapshot_<host>_<time>.tar.gz
```

它会采集 ROS topic/node/service/action 列表、关键话题频率、节点参数、TF、进程、网络环境和 106 地图目录信息。脚本只读，不会发运动命令、不会重启服务、不会修改地图。

## 部署注意

- 手册建议轴指令 20Hz，本工程默认按 20Hz 向 103 发送 `Type=2, Command=21`。
- 楼梯/平地步态按手册 `Type=2, Command=23` 执行；`/m20pro/gait_command` 中的 `flat` 会转成 `GaitParam=1`，`stair_up`/`stair_down` 会转成 `GaitParam=14`。
- 厂商导航任务在导航模式下执行，轴指令在常规模式下执行。使用本工程的 104 路径跟随时，请确保机器人处于可接受轴指令的常规/运动状态。
- 默认没有开启心跳主动上报，避免主动报文和请求响应共用 TCP 连接时串包。需要时可在 `m20pro_bringup/config/m20pro.yaml` 中把 `send_heartbeat` 改为 `true`。
- M20 Pro 上是 Foxy，上位机是 Humble 时，跨主机 DDS 通信要统一 `ROS_DOMAIN_ID`，并确认两侧网络可互通。若 DDS 不稳定，优先在 104 上运行本工程，只从 106 复制地图文件。
