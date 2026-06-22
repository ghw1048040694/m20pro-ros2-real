# M20 Pro ROS 2 Real

这是 M20 Pro 真机项目，只服务 104 真机运行链路。仿真已经拆到独立仓库，不再从本仓库启动或维护。

## 主机分工

| 主机 | 地址 | 作用 |
| --- | --- | --- |
| 103 AOS | `10.21.31.103` | 运动控制、官方 TCP 协议、相机 RTSP |
| 104 GOS | `10.21.31.104` | 运行本工程、Nav2、网页前端 |
| 106 NOS | `10.21.31.106` | 原厂建图、定位、导航、点云发布 |

104 推荐工作区：

```bash
/home/user/m20pro_ros2_ws
```

进入 104 环境的固定顺序：

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
cd /home/user/m20pro_ros2_ws
source install/setup.bash
```

## 编译

```bash
source /opt/robot/scripts/setup_ros2.sh
cd /home/user/m20pro_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 现场入口

现场人员直接使用根目录 `scripts/`：

```bash
./scripts/104_start_real_shadow.sh
./scripts/104_start_real_move.sh
./scripts/104_preflight_check.sh move
./scripts/104_diagnose_preflight.sh
./scripts/104_stop_real.sh
./scripts/104_record_bag.sh 180 m20_test
./scripts/104_check_lidar.sh
./scripts/104_status.sh
./scripts/104_enable_autostart.sh move
```

`shadow` 不放开运动控制；`move` 会放开运动控制，必须现场有人看护并准备手柄急停。

全量 real 启动会拉起：

- `m20pro_tcp_bridge`
- 点云 relay 和点云融合
- `/scan`
- Nav2
- 网页前端
- 楼层/任务管理
- 可选巡检检测

浏览器访问：

```text
http://10.21.31.104:8080
```

## 开机自启

```bash
./scripts/104_enable_autostart.sh move
systemctl start m20pro-real.service
./scripts/104_autostart_status.sh
```

停用：

```bash
./scripts/104_disable_autostart.sh
```

自启动只启动系统和网页，不会自动执行任务。

## 关键链路

原始雷达入口使用原厂 DDS profile：

```text
/LIDAR/POINTS  -> /m20pro/lidar_points_relay
/LIDAR/POINTS2 -> /m20pro/lidar_points2_relay
```

项目主链路默认使用 UDP-only FastDDS 配置：

```text
src/m20pro_bringup/config/m20pro_fastdds_udp.xml
```

点云 relay 会先降采样和限频，再给后端融合链路使用，避免 104 长时间高压。

## 关键文件

```text
src/m20pro_bringup/launch/m20pro_real.launch.py
src/m20pro_bringup/config/m20pro_real.yaml
src/m20pro_bringup/config/nav2_params_real.yaml
src/m20pro_bringup/config/map_manifest.yaml
src/m20pro_bringup/config/inspection_waypoints.yaml
src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py
src/m20pro_navigation/m20pro_navigation/tcp_bridge_node.py
src/m20pro_navigation/m20pro_navigation/pointcloud_fusion.py
```

## 安全规则

- 不要清 `/dev/shm/fastrtps_*`。
- 不要从本工程脚本重启原厂 multicast/lidar 服务。
- 不要同时使用原厂导航任务和本工程 Nav2 轴指令控制机器狗。
- `/scan` 或雷达点云缺失是硬故障，不能因为在工位就忽略。
- 工位未重定位时，Nav2/costmap 可以延后确认；点云、`/scan`、地图、网页和电量不能缺失。

## 历史记录

开发记录和现场问题复盘保留在：

```text
m20pro日志.md
```
