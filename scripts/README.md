# M20Pro现场常用脚本

这些脚本是给现场人员直接执行的快捷入口，避免每次测试都手敲长命令。

约定：
- 仓库根目录 `scripts/` 是人工入口。
- `src/m20pro_bringup/scripts/` 是 ROS 包内部脚本，不作为现场人员主要入口。
- 现场文档 `/home/fabu/桌面/脚本.docx` 中默认引用的就是本目录脚本。

在104上使用时，先按固定顺序进入环境：

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
cd /home/user/m20pro_ros2_ws
source install/setup.bash
```

常用命令：

```bash
./scripts/104_start_web.sh
./scripts/104_stop_web.sh
./scripts/104_start_real_shadow.sh
./scripts/104_start_real_move.sh
./scripts/104_stop_real.sh
./scripts/104_record_bag.sh 180 m20_test
./scripts/104_check_lidar.sh
./scripts/104_status.sh
```

在上位机拉回104录包：

```bash
./scripts/local_pull_bags.sh
```

说明：
- `shadow` 不放开运动控制。
- `move` 会放开运动控制，现场必须有人看护，并准备手柄急停。
- 这些脚本不重启原厂 multicast 服务。
- `104_stop_web.sh` 只停止网页前端。
- `104_stop_real.sh` 停止本工程 real launch，不停止原厂服务。
