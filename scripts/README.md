# M20Pro现场常用脚本

这些脚本是给现场人员直接执行的快捷入口，避免每次测试都手敲长命令。

约定：
- 仓库根目录 `scripts/` 是人工入口。
- `src/m20pro_bringup/scripts/` 是 ROS 包内部脚本，不作为现场人员主要入口。
- 现场文档 `/home/fabu/桌面/脚本.docx` 中默认引用的就是本目录脚本。

在104上使用时，先按固定顺序进入环境；`104_diagnose_preflight.sh` 和
`104_preflight_check.sh` 也可以在上位机仓库根目录直接执行，脚本会优先
SSH 到 `user@10.21.31.104` 并在 104 上检查：

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
./scripts/104_diagnose_preflight.sh
./scripts/104_stop_real.sh
./scripts/104_record_bag.sh 180 m20_test
./scripts/104_check_lidar.sh
./scripts/104_check_initialpose_to_106.sh 0.0 0.0 0.0
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
- 全量 real 启动前必须实际收到 `/LIDAR/POINTS` 的 `PointCloud2` 样本；只看到 topic 名不算通过。点云未就绪时脚本会停止启动，避免反复创建 DDS 参与者。
- 全量 real 会尝试把可选第二路 `/LIDAR/POINTS2` relay 到 `/m20pro/lidar_points2_relay` 并融合进 `/scan`；如果当前机器狗没有发布 `/LIDAR/POINTS2`，只记录提示并继续使用主雷达，不阻塞开机自检。
- 网页“自检”页是开机基础自检主入口；点一次“开机基础自检”，确认全量系统、网页、原始点云、电量和原厂状态链路。
- 定位、`/scan`、代价地图和 Nav2 生命周期需要到测试场地重定位后再确认；网页自检会把未重定位前的 costmap/Nav2 延后启动显示为信息项，不再作为 WARN 阻塞重定位。
- `104_preflight_check.sh move` 是终端备用基础自检；网页自检异常、或现场需要保存终端输出时使用。它会自动使用项目 UDP-only FastDDS 配置观察 relay 点云，避免 root 服务和 user 终端之间的 SHM 隔离造成“echo 不出点云”的假阴性。
- `104_diagnose_preflight.sh` 是只读诊断汇总：会收集网页自检、点云/scan/costmap、新版 Nav2 启动门、辅助模式状态字段和最近日志，不会下发运动、步态或辅助模式命令。
- `104_diagnose_preflight.sh` 也会打印 104 的默认路由、DNS、git 工作区状态、`/LIDAR/POINTS2` 是否存在、两路 relay/fusion 状态，并用项目 UDP-only FastDDS 配置做一次临时订阅探针。若网页和 relay/fusion 都显示点云新鲜，但普通 `ros2 topic echo/info` 显示无样本或 publisher 为 0，优先按 DDS profile/graph 发现问题处理。
- `check_preflight_policy.py` 是本地防回归检查，改动自检、Nav2 启动门或辅助模式显示后先跑一遍：

```bash
./scripts/check_preflight_policy.py
```

- `104_check_initialpose_to_106.sh x y yaw` 用于排查网页重定位链路：104 发布 `/initialpose`，106 临时监听一次，确认 106 是否真的收到。
- `104_start_web.sh` 只用于开发预览网页界面，不会拉起 tcp_bridge/Nav2/点云融合，不能作为重定位、标点、下发任务的现场流程。
- 如果 `/LIDAR/POINTS` 进入 topic 可见但无样本状态，只停止本工程 real stack；不要清 `/dev/shm/fastrtps_*`，不要从本工程脚本重启原厂 multicast/lidar 服务。
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
- 如果 104 上 `/home/user/m20pro_ros2_ws` 不是 git 工作区，`git pull` 不会工作；先用 `104_diagnose_preflight.sh` 确认 `git_repo=yes/no`，非 git 工作区继续用上位机 `local_deploy_to_test_robot.sh`，或在网络稳定后用 `104_update_from_gitlab.sh`/`104_update_from_mirror.sh` 转成 git 工作区。
- 如果 104 需要通过 103 上网，104 必须能拿到默认路由和 DNS，103 自己也必须有 Wi-Fi/上游默认路由、IPv4 转发、NAT 和 dnsmasq；只看到 104 能 ping 到 `10.21.31.103` 不等于能访问 GitHub。
- `104_update_from_gitlab.sh` 是 104 直连 GitLab 后使用的更新入口；如果当前网络访问不到 `git.fabu.ai`，先不要用它。
- `104_update_from_mirror.sh` 是测试机直连镜像仓库的入口，默认使用 `git@github.com:ghw1048040694/m20pro-ros2-navigation.git`；如果改用 Gitee，可执行：

```bash
M20PRO_REMOTE_URL=git@gitee.com:<你的命名空间>/<仓库名>.git ./scripts/104_update_from_mirror.sh
```

- 镜像仓库如果是私有仓库，需要把测试机 104 的公钥加入 GitHub/Gitee 的 Deploy Key 或个人 SSH Key。
