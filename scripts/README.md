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
cd /home/user/m20pro_real_ros2_ws
source install/setup.bash
```

现场主入口：

```bash
./scripts/104_start_real_shadow.sh
./scripts/104_start_real_move.sh
./scripts/104_preflight_check.sh move
./scripts/104_stop_real.sh
./scripts/104_record_bag.sh 180 m20_test
./scripts/104_enable_autostart.sh move
./scripts/104_autostart_status.sh
./scripts/104_disable_autostart.sh
```

诊断和开发入口：

```bash
./scripts/104_diagnose_preflight.sh
./scripts/104_check_edge_scan.sh
./scripts/104_goal_mode_battery_gate.py
./scripts/104_status.sh
./scripts/104_start_web.sh
./scripts/104_stop_web.sh
```

在上位机拉回104录包：

```bash
./scripts/local_pull_bags.sh
```

第二台测试机代码同步：

```bash
# 先在上位机 git pull；该入口会部署 106 edge scan，再事务更新 104
./scripts/local_deploy_to_test_robot.sh

# 未来当 104 能直接访问公司 GitLab 后，在 104 上执行
./scripts/104_update_from_gitlab.sh

# 如果公司 GitLab 不通，但测试机能访问 Gitee 部署镜像
./scripts/104_update_from_mirror.sh
```

现场参数只修改 `src/m20pro_bringup/config/m20pro_field_profile.yaml`：

```bash
./scripts/apply_field_profile.sh --check  # 只校验，不接触机器狗
./scripts/apply_field_profile.sh          # 空闲时原子更新 106 和 104
```

schema v4 共 67 个现场参数，其中 `navigation` 30 个、`teleoperation` 7 个，覆盖速度/加速度、纯旋转最低有效速度、到点、卡住判定、DWB 采样、局部/全局代价地图、全局规划、遥控限速、失联停车、跨层路线元数据与定位稳定性。旧楼梯感知参数已删除。不要直接编辑 106 的 `/etc/m20pro-edge-scan-106.env`、Nav2 参数占位符或定位参数占位符；整狗部署会从唯一现场参数文件按固定顺序同步 106 和 104，任务运行中不支持热更新。

说明：
- 真机现场测试只用 `104_start_real_shadow.sh` 或 `104_start_real_move.sh` 全量启动。
- 全量 real 会同时拉起 tcp_bridge、Nav2 和网页前端；感知唯一输入是 106 edge scan 发布的 `/scan`。
- 任务前通过网页自检、`/api/state` 或 `104_check_edge_scan.sh` 确认 `/scan` 新鲜、frame 正确且有效距离足够。
- 服务启动 12 秒后自动执行基础自检，之后每 300 秒刷新；结果显示在网页顶部状态栏，用于确认全量系统、网页、106 edge scan 和原厂状态链路。
- U360 雷达巡检默认关闭；需要联动任务点扫描时，启动前设置 `M20PRO_ENABLE_RADAR_INSPECTION=true`、`M20PRO_RADAR_BACKEND=u360_http` 和 `M20PRO_RADAR_DEVICE_URL=http://192.168.107.72:8080`。结果默认写到 `M20PRO_RADAR_OUTPUT_DIR`，未设置时使用 `/home/user/m20pro_radar_results`。
- 定位、`/scan`、代价地图和 Nav2 生命周期需要到测试场地重定位后再确认；网页自检会把未重定位前的 costmap/Nav2 延后启动显示为信息项，不再作为 WARN 阻塞重定位。
- `104_preflight_check.sh move` 是终端备用基础自检；网页自检异常、或现场需要保存终端输出时使用。104 正式服务和诊断终端都应使用项目 UDP-only FastDDS 配置；104 不再观察或转发原始点云。
- `104_diagnose_preflight.sh` 是只读诊断汇总：会收集网页自检、点云/scan/costmap、新版 Nav2 启动门、辅助模式状态字段和最近日志，不会下发运动、步态或辅助模式命令。
- `104_diagnose_preflight.sh` 会打印 104 的默认路由、DNS、Git 状态、`/scan` 和 Nav2 状态。
- `104_goal_mode_battery_gate.py` 现在只作为电量显示探针：只读查询 `http://10.21.31.104:8080/api/state` 并打印当前电量参考值；不会因为低电或读不到电量返回失败，不会调用 `/api/tasks/start`，不会发布 `/m20pro/floor_goal`，不会重定位，不会发运动命令。
- 现场任务问题统一录 rosbag 复盘，不再使用前端 watcher、ready-check、失败快照或 smoke 脚本作为正式流程。录包用 `./scripts/104_record_bag.sh 180 <label>`，上位机拉回用 `./scripts/local_pull_bags.sh`。
- 历史前端 watcher/ready-check/analyzer/smoke 脚本已删除，不再作为维护对象。

- 重定位排查以网页顶部“定位”浮层、`localization_status` 和开发手册 TCP `2101/1` 回执为准；不要再用“106 是否收到 `/initialpose`”作为成功判断。
- 当前地图、重定位和录包都从网页顶部状态栏进入，不再占用独立侧栏面板。
- `104_start_web.sh` 只用于开发预览网页界面，不会拉起 tcp_bridge/Nav2，不能作为现场任务流程。
- `127.0.0.1:8080` 只适合在运行前端的那台机器本机自测。
- `shadow` 不放开运动控制。
- `move` 会放开运动控制，现场必须有人看护，并准备手柄急停。
- 这些脚本不重启原厂 multicast 服务。
- `104_stop_web.sh` 只停止单独预览网页。
- `104_stop_real.sh` 停止本工程 real launch，不停止原厂服务。
- 录包脚本会记录 `/m20pro/active_waypoint`，里面包含当前任务阶段、目标位姿、停留时间和点位语义字段。
- `104_enable_autostart.sh move` 安装开机自启动全量 real；服务只拉起系统和网页，不会自动执行任务。
- `104_autostart_status.sh` 查看自启动服务、8080 端口和最近日志。
- `104_disable_autostart.sh` 停止并移除自启动服务。
- `local_deploy_to_test_robot.sh` 是整狗部署入口：先用最小文件集在 106 编译、安装并启用 edge scan，再把上位机源码同步到 104 暂存目录；停服务切换后只在最终 `/home/user/m20pro_real_ros2_ws` 路径执行 `colcon --symlink-install`，失败自动恢复上一工作区和 systemd 配置。
- `local_deploy_to_test_robot.sh` 在接触主机前校验唯一现场参数文件，按 106→104→106 的固定顺序同步和验收；106 的 env 只由该文件生成，不再保留可编辑模板。
- 104 网页订阅者起来后，部署入口会重启一次 106 edge publisher，并要求 `/api/state` 明确返回 `edge_scan`、`m20pro_base_link` 和至少 20 个有效距离才算完成。不要只同步 104，也不要把暂存目录中的 symlink install 直接改名投入运行。
- `local_deploy_edge_scan_to_106.sh` 是只补装 106 edge 组件的维护入口；正常整狗更新直接运行 `local_deploy_to_test_robot.sh`。
- 如果 104 上 `/home/user/m20pro_real_ros2_ws` 不是 git 工作区，`git pull` 不会工作；先用 `104_diagnose_preflight.sh` 确认 `git_repo=yes/no`，非 git 工作区继续用上位机 `local_deploy_to_test_robot.sh`，或在网络稳定后用 `104_update_from_gitlab.sh`/`104_update_from_mirror.sh` 转成 git 工作区。
- 如果 104 需要通过 103 上网，104 必须能拿到默认路由和 DNS，103 自己也必须有 Wi-Fi/上游默认路由、IPv4 转发、NAT 和 dnsmasq；只看到 104 能 ping 到 `10.21.31.103` 不等于能访问 GitHub。
- `104_update_from_gitlab.sh` 是 104 直连 GitLab 后使用的更新入口；如果当前网络访问不到 `git.fabu.ai`，先不要用它。
- `104_update_from_mirror.sh` 是机器狗无法访问公司 GitLab 时的直接更新入口，默认只拉取 Gitee 部署镜像 `git@gitee.com:gggghw/m20pro-ros2-real.git` 的已验证 `main`。
- Gitee 部署镜像是私有仓库。每台机器狗首次使用前，必须把 104 的公钥加入该仓库的只读部署公钥；脚本优先使用既有 `/home/user/.ssh/id_ed25519_m20pro_test_gitlab` 私钥，该文件不存在时由 SSH 使用 104 的标准密钥，不生成第二套密钥。
- 三个仓库职责固定：GitLab 是公司开发源，允许保留开发分支；GitHub 是个人成果仓库，只保留 `main`；Gitee 是机器狗部署镜像，也只保留经过验证的 `main`。
