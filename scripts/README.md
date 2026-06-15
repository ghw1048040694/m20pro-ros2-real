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
./scripts/104_start_real_shadow.sh
./scripts/104_start_real_move.sh
./scripts/104_preflight_check.sh move
./scripts/104_stop_real.sh
./scripts/104_record_bag.sh 180 m20_test
./scripts/104_check_lidar.sh
./scripts/104_status.sh
./scripts/104_start_web.sh
./scripts/104_stop_web.sh
./scripts/104_enable_autostart.sh move
./scripts/104_autostart_status.sh
./scripts/104_disable_autostart.sh
```

在上位机拉回104录包：

```bash
./scripts/local_pull_bags.sh
```

第二台测试机代码同步：

```bash
# 当前可用：先在上位机 git pull，再同步到第二台 104 并编译
./scripts/local_deploy_to_test_robot.sh

# 未来当 104 能直接访问公司 GitLab 后，在 104 上执行
./scripts/104_update_from_gitlab.sh

# 如果公司 GitLab 不通，但测试机能访问 GitHub/Gitee 镜像仓库
./scripts/104_update_from_mirror.sh
```

说明：
- 真机现场测试只用 `104_start_real_shadow.sh` 或 `104_start_real_move.sh` 全量启动。
- 全量 real 会同时拉起 tcp_bridge、Nav2、点云融合和网页前端；笔记本/手柄访问 `http://10.21.31.104:8080`。
- 网页“自检”页是开机基础自检主入口；点一次“开机基础自检”，确认全量系统、网页、原始点云、电量和原厂状态链路。
- 定位、`/scan`、代价地图和 Nav2 生命周期需要到测试场地重定位后再确认；未重定位时这些项可能是 WARN。
- `104_preflight_check.sh move` 是终端备用基础自检；网页自检异常、或现场需要保存终端输出时使用。
- `104_start_web.sh` 只用于开发预览网页界面，不会拉起 tcp_bridge/Nav2/点云融合，不能作为重定位、标点、下发任务的现场流程。
- `127.0.0.1:8080` 只适合在运行前端的那台机器本机自测。
- `shadow` 不放开运动控制。
- `move` 会放开运动控制，现场必须有人看护，并准备手柄急停。
- 这些脚本不重启原厂 multicast 服务。
- `104_stop_web.sh` 只停止单独预览网页。
- `104_stop_real.sh` 停止本工程 real launch，不停止原厂服务。
- 录包脚本会记录 `/m20pro/active_waypoint`，里面包含当前任务点类型、yaw、停留时间和开发手册对应的导航字段。
- `104_enable_autostart.sh move` 安装开机自启动全量 real；服务只拉起系统和网页，不会自动执行任务。
- `104_autostart_status.sh` 查看自启动服务、8080 端口和最近日志。
- `104_disable_autostart.sh` 停止并移除自启动服务。
- `local_deploy_to_test_robot.sh` 从上位机同步当前工作区到测试机，不同步 `.git/build/install/log/bags`。
- `104_update_from_gitlab.sh` 是 104 直连 GitLab 后使用的更新入口；如果当前网络访问不到 `git.fabu.ai`，先不要用它。
- `104_update_from_mirror.sh` 是测试机直连镜像仓库的入口，默认使用 `git@github.com:ghw1048040694/m20pro-ros2-navigation.git`；如果改用 Gitee，可执行：

```bash
M20PRO_REMOTE_URL=git@gitee.com:<你的命名空间>/<仓库名>.git ./scripts/104_update_from_mirror.sh
```

- 镜像仓库如果是私有仓库，需要把测试机 104 的公钥加入 GitHub/Gitee 的 Deploy Key 或个人 SSH Key。
