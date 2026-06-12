# M20 Pro Project Notes

Last updated: 2026-06-12 10:52 CST

This file is maintained by Codex as the local M20 Pro project memory for future ChatGPT review. It records the current architecture, important decisions, recent changes, verification status, and next steps.

Naming note: this file replaced the previous local-only `codex.md`. Going forward, maintain `m20pro.md`.

## 2026-06-12 pre-departure field check

- Local repo was clean before the check and latest commit was present on both GitHub and GitLab.
- Desktop `/home/fabu/桌面/脚本.docx` was verified readable, uses Songti fonts, and contains only:
  - `任务一：同楼层真导航`
  - `任务二：跨楼层真导航`
- 104 workspace structure was checked:
  - packages are only under `/home/user/m20pro_ros2_ws/src`;
  - no stray `package.xml` exists at workspace root or `src` root;
  - `colcon list --base-paths src` sees the expected five packages.
- 104 field scripts were verified synchronized with local checksums.
- No M20Pro real/web/Nav2 process was left running before departure.
- Disk note:
  - 104 has about 5.2 GB free on `/home/user`;
  - `/home/user/bags/m20_real_nav_20260611_153008` still occupies about 6.5 GB;
  - it was not deleted because no local backup was found during the check.
- Fixed a field-critical compatibility issue:
  - 104 ROS 2 Foxy does not accept `ros2 topic echo --once`;
  - `scripts/104_preflight_check.sh` now reads one message through `ros2 topic echo ... --no-arr` and exits after the first message separator;
  - README check commands were updated to avoid `--once`.
- Do not change or restart factory multicast/FastDDS services during the field run unless explicitly doing recovery diagnostics.

## 2026-06-04 desktop field script regenerated

- User asked to delete the previous M20Pro Word documents from the desktop and regenerate a clean script document named `脚本.word`.
- Deleted desktop Word files:
  - `/home/fabu/桌面/M20Pro真机现场测试脚本.docx`;
  - `/home/fabu/桌面/M20Pro真机现场测试脚本_旧版_20260604.docx`;
  - `/home/fabu/桌面/M20Pro系统使用说明书.docx`.
- Generated:

```text
/home/fabu/桌面/脚本.word
/home/fabu/桌面/脚本.docx
```

- LibreOffice successfully converted the document to PDF for validation, so the file format is readable.
- The new document includes:
  - field safety requirements;
  - 104 raw pointcloud verification using the known-good `source /opt/robot/scripts/setup_ros2.sh -> su -> no extra source` sequence;
  - official M20Pro relocalization procedure from the software manual;
  - factory baseline bag recording;
  - M20Pro real shadow navigation startup;
  - `/scan` and costmap checks;
  - M20Pro shadow bag recording using `m20pro_record_real.sh`;
  - short same-floor movement test only after all read-only checks pass;
  - bag pullback command;
  - read-only diagnostics if pointcloud disappears.
- Manual relocalization finding from `/home/fabu/桌面/M20Pro/山猫M20 Pro软件使用手册V0.0.1.pdf`, section `3.5 初始化定位`:

```bash
su
source /opt/ros/foxy/setup.bash
export XAUTHORITY=/home/user/.Xauthority
rviz2
```

  - In RViz, open config:

```text
/opt/robot/share/localization/conf/localization.rviz
```

  - Check whether real-time pointcloud overlaps the map.
  - If not, use RViz `2D Pose Estimate`: click the robot's real map location and drag the arrow along the real heading.
  - If initialization fails, repeat from the beginning.
- Current project relation:
  - `tcp_bridge_node.py` already supports forwarding `/initialpose` to the vendor relocalization API with `Type=2101`, `Command=1`, and `PosX/PosY/PosZ/Yaw`;
  - however, the field script now recommends using the official RViz localization procedure first to confirm the factory localization is correct before running M20Pro Nav2 tests.

## 2026-06-04 17:33 factory baseline bag review and script cleanup

- User recorded a new bag on 104:

```text
/home/user/bags/rosbag2_2026_06_04-17_32_57
```

- Bag summary:
  - size: 650.1 MiB;
  - duration: 23.662 s;
  - total messages: 12883;
  - `/LIDAR/POINTS`: 237 messages, about 10 Hz;
  - `/LIDAR/POINTS2`: 0 messages;
  - `/LIDAR/IMU201`: 3548 messages;
  - `/LIDAR/IMU202`: 3520 messages;
  - `/IMU`: 3314 messages;
  - `/ODOM`: 238 messages;
  - `/tf`: 239 messages;
  - `/GRID_MAP`: 1 message;
  - `/LOCATION_STATUS`: 47 messages;
  - `/LOCATION_STATUS/MATCHING_ERROR`: 239 messages;
  - `/scan`, `/map`, local/global costmaps, `/cmd_vel`: not present because this was a factory baseline bag, not an M20Pro Nav2 shadow-run bag.
- Interpretation:
  - this bag proves the factory raw lidar/IMU/ODOM/TF path is alive on 104;
  - it does not yet prove `pointcloud_fusion -> /scan -> Nav2 costmap` works, because our real stack was not running during this bag;
  - next useful bag is a shadow-navigation bag with our real launch running and `enable_axis_command:=false`.
- Important workflow correction:
  - for raw pointcloud checks and factory baseline bag recording, use the exact known-good sequence:

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
# root shell: do not source anything else before checking raw pointcloud
ros2 topic echo /LIDAR/POINTS --no-arr
```

- Avoid restarting or editing multicast services during ordinary field testing.
  - The field script now only includes read-only `systemctl status multicast-relay.service --no-pager` as a later diagnostic.
  - It no longer tells the user to restart systemd/multicast as part of the normal flow.
- Code/scripts updated:
  - `m20pro_record_real.sh` now refuses to run as ordinary user unless `M20PRO_ALLOW_USER_RECORD=1` is explicitly set;
  - it prompts the operator to use the known-good `source -> su -> root` environment;
  - it probes `/LIDAR/POINTS` for real samples before recording;
  - it no longer re-sources `/opt/robot/scripts/setup_ros2.sh` if `ROS_DISTRO` is already present, so it does not disturb the proven root environment;
  - recorded topics now include factory lidar IMU/status, ODOM, GRID_MAP, LOCATION_STATUS matching error, STEER/HANDLE_STEER, battery, and fault status.
- Health check updated:
  - `system_check_node.py` now has a `cloud_reliability` parameter;
  - real launch passes `cloud_reliability: reliable`;
  - this avoids false "no cloud" reports when checking factory `/LIDAR/POINTS`, whose bag metadata shows reliable QoS.
- Deployment/verification:
  - local build passed for `m20pro_navigation` and `m20pro_bringup`;
  - workspace was synced to 104;
  - 104 build passed for `m20pro_navigation` and `m20pro_bringup`;
  - 104 installed `m20pro_record_real.sh` shows the new root-environment guard and lidar sample probe.
- Desktop field-test Word script updated:

```text
/home/fabu/桌面/M20Pro真机现场测试脚本.docx
```

  - old version backed up as:

```text
/home/fabu/桌面/M20Pro真机现场测试脚本_旧版_20260604.docx
```

  - new version validates with LibreOffice conversion to PDF;
  - new version removes normal-flow multicast restart instructions;
  - new version splits tests into factory baseline, M20Pro shadow navigation, M20Pro perception/costmap check, and later short movement test.

## 2026-06-04 Real robot pointcloud status and correct verification method

- User rebooted the robot and manually verified that both 104 and 106 can see lidar pointcloud data.
- Correct 104 verification sequence is:

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
# root password is the single quote character: '
# after su, do not source anything else
ros2 topic list | grep LIDAR
ros2 topic info -v /LIDAR/POINTS
ros2 topic echo /LIDAR/POINTS --no-arr
```

- 104 verification result:
  - `/LIDAR/POINTS` and `/LIDAR/POINTS2` are visible;
  - `/LIDAR/POINTS` has 2 DDS publishers;
  - publishers show as `_CREATED_BY_BARE_DDS_APP_`;
  - publisher QoS is reliable/volatile;
  - `ros2 topic echo /LIDAR/POINTS --no-arr` outputs live `sensor_msgs/msg/PointCloud2`;
  - `frame_id` is `lidar_link`;
  - point count commonly appears around 40k-60k points per frame;
  - `point_step` is 26, with x/y/z/intensity/ring/timestamp fields.
- Important lesson:
  - do not judge pointcloud availability from only `ros2 topic list`;
  - do not mix too many shell variants during diagnosis;
  - the reference method is now the exact sequence above: user shell sources `/opt/robot/scripts/setup_ros2.sh`, then `su`, then root directly runs ROS 2 commands without another source.
- Earlier Codex confusion:
  - several automated tests used ordinary user echo, root after a different source order, or QoS/hz variants;
  - these variants produced misleading "topic exists but no samples" symptoms;
  - current truth is that 104 can receive raw lidar pointcloud through the official ROS/DDS path when using the exact root sequence above.

## 2026-06-04 Factory DDS/multicast notes

- Cloud Deep Robotics support reported fixing startup ordering by adding a `Wants` line in 106's `/lib/systemd/system/multicast-relay.service`.
- After reboot, 104 can see raw lidar pointcloud again.
- Current working interpretation:
  - the radar itself and 106-side raw pointcloud publisher are healthy;
  - the failure mode is mainly DDS/multicast startup ordering and environment sensitivity;
  - if pointcloud disappears again, first check 106 `multicast-relay.service`, lidar/localization/passable services, and the exact shell/source/su sequence.
- Useful 106 diagnostic command:

```bash
source /opt/robot/scripts/setup_ros2.sh
su
drddsctl list | egrep 'Publisher:.*(LIDAR/POINTS|LIDAR/POINTS2|LOC_BODY_POINTS)|Subscriber:.*(LIDAR/POINTS|LIDAR/POINTS2|LOC_BODY_POINTS)'
ros2 topic echo /LIDAR/POINTS --no-arr
```

- On 106, `/opt/robot/fastdds.xml` was checked. A backup was made before adding `10.21.31.106` to the interface whitelist, because 106 has both a `10.21.33.106` side and a `10.21.31.106` side. Do not remove this line unless the vendor explicitly asks, because 104/PC access currently depends on the `10.21.31.x` network.

## 2026-06-04 Temporary pointcloud bridge removed

- A temporary fallback bridge had been prototyped while raw pointcloud visibility was unstable:
  - a 106-side drdds/TCP sender idea;
  - 104-side TCP pointcloud receiver;
  - `/m20pro/relay/lidar_points` relay topic.
- After the official pointcloud path recovered and user confirmed 104/106 can both see `/LIDAR/POINTS`, this fallback path was removed to avoid introducing a second competing lidar source.
- Removed locally:
  - `src/m20pro_drdds_bridge/`;
  - `m20pro_cloud_bridge` TCP pointcloud sender/receiver entry points;
  - `pointcloud_tcp_sender.py`;
  - `pointcloud_tcp_receiver.py`;
  - `m20pro_pointcloud_sender_106.sh`;
  - `m20pro_pointcloud_receiver_104.sh`;
  - `/m20pro/relay/lidar_points` from the real bag record script.
- Verification after cleanup:

```bash
rg -n "pointcloud_tcp|drdds_pointcloud|m20pro_drdds_bridge|relay/lidar_points|m20pro_pointcloud_(sender|receiver)|M20PRO_RELAY_OUTPUT_TOPIC" -S .
colcon build --packages-select m20pro_bringup m20pro_cloud_bridge --event-handlers console_direct+
```

- Result:
  - no bridge keywords remained;
  - no bridge process was running;
  - `m20pro_bringup` and `m20pro_cloud_bridge` built successfully.
- Current policy:
  - use the factory `/LIDAR/POINTS` path directly;
  - do not reintroduce a pointcloud TCP bridge unless the official DDS path becomes impossible to stabilize.

## 2026-06-04 Current real-machine testing boundary

- Real-machine status:
  - 104 and 106 network are reachable over `10.21.31.x`;
  - 104 can now see raw `/LIDAR/POINTS`;
  - real launch and Nav2 can be started in read-only/safe mode;
  - the robot should not be commanded to move until map alignment, pointcloud-to-scan conversion, TF height behavior, and local costmap marking are confirmed in the actual test area.
- Recommended safe launch on 104:

```bash
cd /home/user/m20pro_ros2_ws
source /opt/robot/scripts/setup_ros2.sh
source install/setup.bash
ros2 launch m20pro_bringup m20pro.launch.py mode:=real \
  rviz:=false \
  enable_axis_command:=false \
  enable_web_dashboard:=true \
  cloud_topic:=/LIDAR/POINTS
```

- Before enabling movement, verify:

```bash
ros2 topic echo /LIDAR/POINTS --no-arr
ros2 topic echo /scan --no-arr
ros2 topic echo /map --no-arr
ros2 topic echo /local_costmap/costmap --no-arr
ros2 topic echo /m20pro_tcp_bridge/map_pose --no-arr
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

- Movement test should start with:
  - small open area;
  - correct loaded map;
  - known initial pose;
  - `enable_axis_command:=false` first;
  - then only switch to `enable_axis_command:=true` after costmap and pose are clearly correct.

## 2026-06-03 to 2026-06-04 Real/sim robustness summary

- Real experience fed back into sim/launch logic:
  - pointcloud availability must be checked by actual sample reception, not only topic existence;
  - local costmap should be treated as unhealthy if `/scan` exists but carries no useful ranges or no obstacles are marked;
  - TF z drift and map/environment mismatch can make Nav2 appear to run while the result is meaningless;
  - startup checks should distinguish "node started" from "data stream useful".
- Sim side was strengthened to mirror real risks:
  - sim and real launch paths use clearer cloud topic separation;
  - sim uses PCD-derived pointcloud assumptions to test pointcloud fusion and costmap marking;
  - `system_check`/health checks are expected to catch map, scan, local costmap, TF, and lifecycle problems sooner.
- Practical conclusion:
  - sim cross-floor logic is already a useful closed-loop check;
  - real validation still needs a controlled short-path test with bag recording;
  - stair behavior on real robot must still rely on factory gait/stair-control behavior, not Nav2 alone.

## 2026-06-02 Unified web operation console

- User asked for one frontend that can be used from the Android controller, SSH/local 104 access, or a customer server path, instead of making customers type ROS/YAML commands.
- Extended `m20pro_cloud_bridge/web_dashboard_node.py` from a passive dashboard into a workflow console:
  - keeps the existing live ROS dashboard for floor, stair state, gait command, robot pose, path, dynamic obstacles, YOLO detections/events, topic freshness, and live `/map`;
  - adds persistent workflow data under `data_dir`, default `~/.m20pro_web`;
  - adds map archive management under `map_archive_dir`, default `~/m20pro_maps`;
  - adds project/mapping session APIs;
  - adds archived map list/select APIs;
  - adds PGM/YAML loading for archived maps, so operators can mark points without requiring live `/map`;
  - adds map annotation APIs for patrol points, stair entry/switch/exit points, charge points, and transition points;
  - adds simple task APIs that sequence selected annotations and publish `/m20pro/floor_goal`.
- Frontend tabs are now:
  - `看板`;
  - `建图`;
  - `地图`;
  - `标点`;
  - `任务`.
- 106 factory mapping integration:
  - added configurable parameters `factory_host`, `factory_user`, `factory_active_map`;
  - added command hooks `factory_mapping_start_command`, `factory_mapping_finish_command`, `factory_mapping_cancel_command`;
  - user pointed out that the M20Pro software manual includes the mapping commands;
  - confirmed from `/home/fabu/桌面/M20Pro/山猫M20 Pro软件使用手册V0.0.1.pdf`:
    - start mapping on 106/NOS: `sudo drmap mapping`;
    - named map: `sudo drmap mapping -n xxx`;
    - no-RViz mapping: `sudo drmap mapping -s`;
    - finish/save mapping: `sudo drmap stop_mapping`;
    - maps live under `/var/opt/robot/data/maps`, and `active` is the active map symlink;
  - default web command hooks now run the manual's `drmap` flow over SSH:
    - start: `ssh ... "nohup sudo -n drmap mapping -s -n {map_name} > /tmp/m20pro_drmap_mapping_{session_id}.log 2>&1 &"`;
    - finish/cancel: `ssh ... "sudo -n drmap stop_mapping"`;
  - this requires SSH from 104 to 106 and non-interactive sudo for `drmap`; otherwise the frontend will show command failure and the operator can still run `drmap` manually on 106 before importing the active map.
- Map import behavior:
  - default source is `user@10.21.31.106:/var/opt/robot/data/maps/active`;
  - import uses `scp -r` unless `factory_host` is `localhost`/`127.0.0.1`, in which case it copies locally;
  - imports never modify the 106 active map;
  - each import creates a timestamped/local archive record and selects it in the frontend.
- Launch integration:
  - `m20pro_web_dashboard.launch.py` exposes all new parameters;
  - `m20pro_sim.launch.py` and `m20pro_real.launch.py` pass the same parameters to the dashboard node;
  - unified `m20pro.launch.py` also forwards these parameters to sim/real.
- README updated:
  - web dashboard section now describes the operation console, not just passive status viewing;
  - quick start explains `data_dir`, `map_archive_dir`, 106 active map import, and command hook boundaries.
- Verification:
  - `python3 -m py_compile` passed for web node and modified launch files;
  - `colcon build --packages-select m20pro_cloud_bridge m20pro_bringup --symlink-install` passed;
  - temporary launch on port `18081` succeeded with test dirs;
  - `/healthz`, `/api/state`, `/api/maps`, HTML serving passed;
  - created a mapping session through `/api/mapping/session`;
  - simulated importing a 106 active map by copying a local F20 map directory;
  - `/api/map_file` successfully parsed archived `occ_grid.yaml/pgm`;
  - created an annotation through `/api/annotations`;
  - created and started a task through `/api/tasks` and `/api/tasks/start`;
  - node published a floor goal for the started task without errors.
- Current boundary:
  - frontend can guide and store workflow now;
  - one-click 106 mapping now has the documented `drmap` command path, but still needs real-machine permission validation for SSH and non-interactive sudo;
  - exporting annotations back into `inspection_waypoints.yaml` or generating a full customer-facing route config is a next step if needed.

## 2026-06-02 Mapping environment check

- User asked to test what can be solved or checked immediately.
- Network/SSH checks from the development laptop:
  - `ping 10.21.31.104` passed;
  - `ping 10.21.31.106` passed;
  - non-interactive SSH to 104 passed;
  - non-interactive SSH to 106 failed with `Permission denied (publickey,password)`.
- Added a frontend/API environment check:
  - new button in the `建图` tab: `检查 106 环境`;
  - new endpoint: `POST /api/mapping/check_environment`;
  - it checks SSH, `drmap`, the active map path, and `sudo -n drmap mapping -h` / `sudo -n drmap stop_mapping -h`;
  - it does not start mapping.
- Tested the new API against real 106:
  - returned `ok: false`;
  - command failed at SSH authentication before reaching `drmap`;
  - current actionable blocker is setting up passwordless SSH from the host running the web node to `user@10.21.31.106`.
- Local fake check also confirmed that the API surfaces `sudo: 需要密码` when non-interactive sudo is not configured.
- Next setup needed on/for 106:
  - install/copy SSH public key for the user that runs the web node;
  - configure a narrow sudoers rule for `drmap mapping` and `drmap stop_mapping`, or provide an official service wrapper.

## 2026-06-01 Documentation update

- Updated `README.md` with the 104 real-machine migration status:
  - RJ45 direct connection info;
  - 104 OS/ROS environment;
  - deployed workspace path `/home/user/m20pro_ros2_ws_20260529_173921`;
  - five-package build result on 104;
  - offline Nav2 and `sensor_msgs_py` installation status;
  - safe real launch command with `enable_axis_command:=false`;
  - current next validation focus: live `/LIDAR/POINTS`, `/scan`, TF, costmaps, and Nav2 lifecycle.

## 2026-05-29 Integration and automation pass

- User asked to implement the first four integration directions after backing up the workspace.
- Implemented a more X30-like integration layer while keeping M20Pro interfaces:
  - added `m20pro_bringup/config/map_manifest.yaml` as the PGM/PCD/floor asset manifest;
  - added `m20pro_bringup/launch/m20pro.launch.py` as the unified startup entry with `mode:=sim` or `mode:=real`;
  - kept direct `m20pro_sim.launch.py` and `m20pro_real.launch.py` compatibility.
- Added new `m20pro_navigation` helpers:
  - `map_manifest.py` for shared package-path and manifest loading;
  - `config_audit` to validate map manifest, floor route config, and map files at startup;
  - `floor_goal_bridge` to convert RViz current-floor goals and short string commands into `/m20pro/floor_goal`;
  - `system_check` to replace the narrower sim-only health monitor with a sim/real runtime health monitor.
- Launch integration:
  - sim and real launch now start `config_audit`, `floor_goal_bridge`, and `system_check` by default;
  - sim point cloud generation can read `map_manifest.yaml` for PCD path and per-floor z ranges;
  - real launch also delays RViz by `rviz_delay_s` to reduce early TF/model/costmap confusion;
  - dynamic obstacles remain enabled by default in sim.
- RViz usability:
  - normal current-floor goal now publishes to `/m20pro/rviz_goal_current` and is bridged into floor-aware goals;
  - floor tools are named in Chinese: current floor, 19楼, 20楼, 21楼.
- Short goal command examples:
  - `ros2 topic pub --once /m20pro/goal_command std_msgs/msg/String "{data: 'F21 2.0 0.0 0.0'}"`
  - waypoint ids from `inspection_waypoints.yaml` such as `f21_demo_check` can also be sent on `/m20pro/goal_command`.
- Verification:
  - `python3 -m py_compile` passed for new/changed Python and launch files;
  - `colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install` passed;
  - `timeout 25s ros2 launch m20pro_bringup m20pro.launch.py mode:=sim rviz:=false enable_web_dashboard:=false` passed;
  - startup output included `configuration audit OK: 3 floors, 0 warnings`;
  - PCD map loaded successfully with 490545 indexed points;
  - dynamic obstacle simulator started with 5 obstacles;
  - final health output was `M20PRO SIM OK: required topics, nodes, maps and Nav2 are active`.

## 2026-05-29 104 direct RJ45 deployment check

- User connected current development laptop directly to M20Pro GOS 104 over RJ45.
- Local wired interface:
  - `enp4s0` was configured as `10.21.31.200/24`;
  - route `10.21.31.0/24 dev enp4s0 src 10.21.31.200` was added;
  - ping to `10.21.31.104` succeeded with roughly 0.3-0.5 ms latency.
- SSH to 104:
  - host `10.21.31.104`;
  - user `user`;
  - public key from the laptop was added to `/home/user/.ssh/authorized_keys`;
  - non-interactive SSH now works from the laptop.
- 104 environment found:
  - Ubuntu 20.04.6 LTS;
  - kernel `5.10.198`;
  - architecture `aarch64`;
  - ROS 2 Foxy under `/opt/ros/foxy`;
  - `colcon`, `rosdep`, `git`, Python 3.8.10 available;
  - disk free about 7.3G;
  - network only has `10.21.31.0/24` on `eth0`, no default gateway, so 104 cannot apt install from the internet by itself.
- Important dependency finding and fix:
  - 104 initially lacked Nav2 packages and `sensor_msgs_py`, so full real navigation could not start;
  - because 104 has no default internet route, Nav2 was installed offline by resolving arm64 Foxy deb URLs from 104 apt metadata, downloading them on the laptop, copying them to `~/nav2_debs`, and installing them on 104 with `dpkg -i`;
  - installed and verified key packages:
    - `nav2_bringup`;
    - `nav2_map_server`;
    - `nav2_lifecycle_manager`;
    - `nav2_controller`;
    - `nav2_planner`;
    - `nav2_navfn_planner`;
    - `nav2_dwb_controller`;
    - `nav2_msgs`;
    - `sensor_msgs_py`.
- Deployment action:
  - copied the local workspace to `/home/user/m20pro_ros2_ws_20260529_173921`;
  - excluded local `build/install/log/bags`, X30 reference folders, and scripts;
  - built all five M20Pro packages on 104 successfully:
    - `m20pro_description`;
    - `m20pro_navigation`;
    - `m20pro_cloud_bridge`;
    - `m20pro_inspection`;
    - `m20pro_bringup`.
- Real launch adjustment:
  - `m20pro_real.launch.py` now detects missing Nav2 packages and falls back to observation mode instead of crashing;
  - observation mode starts config audit, robot state publisher, zero joint publisher, TCP bridge, and system check;
  - it does not start map_server, Nav2, floor_manager, floor_goal_bridge, or pointcloud_fusion when Nav2 is unavailable.
- Safety adjustment:
  - `m20pro.yaml` default `enable_axis_command` changed to `false`;
  - verified 104 real observation launch prints `shadow mode; axis command disabled`.
- 104 observation-mode test result before Nav2 install:
  - command used on 104:
    - `ros2 launch m20pro_bringup m20pro.launch.py mode:=real rviz:=false enable_web_dashboard:=false cloud_topic:=/LIDAR/POINTS`
  - result:
    - Nav2 missing warning shown, observation mode selected;
    - `configuration audit OK: 3 floors, 0 warnings`;
    - TCP bridge connected to 103 at `10.21.31.103:30001`;
    - axis command stayed disabled;
    - health check only waits on `/LIDAR/POINTS`.
- 104 full-stack startup smoke test after Nav2 install:
  - all five M20Pro packages rebuilt successfully on 104 after Nav2 installation;
  - `ros2 pkg prefix` verified the key Nav2 packages and `sensor_msgs_py`;
  - safe real launch was tested with:
    - `mode:=real rviz:=false enable_web_dashboard:=false enable_axis_command:=false cloud_topic:=/LIDAR/POINTS`;
  - Nav2 map server, lifecycle managers, controller, planner, recoveries, BT navigator, waypoint follower, floor manager, pointcloud fusion, and TCP bridge all started;
  - map server loaded F20 `occ_grid.yaml/pgm`;
  - TCP bridge connected to `10.21.31.103:30001`;
  - command output still showed waiting for live `/LIDAR/POINTS`/derived `/scan` during the short smoke test.
- Current blocker for full real navigation:
  - ensure `/LIDAR/POINTS` is actually published on 104 after starting the vendor lidar/relay stack;
  - verify real pose/TF, `/scan`, local/global costmaps, and Nav2 lifecycle stay healthy with live lidar;
  - only after those two are solved should full `mode:=real` navigation be tested with `enable_axis_command:=false` first, then carefully with axis command enabled.

## 2026-05-28 X30 core nav/slam package review

- User added `x30_core_nav_slam/`, containing installed ROS1 package artifacts for X30 navigation/SLAM:
  - `slam`;
  - `localization`;
  - `nav`;
  - `planner`;
  - `rviz_plugins`;
  - selected `system/conf` files.
- Key RViz finding:
  - X30's 3D tools come from `pose_3d_plugin`;
  - plugin classes are `pose_3d_plugin/SetInitialPose3D` and `pose_3d_plugin/SetGoal3D`;
  - `SetInitialPose3D` still publishes `/initialpose`;
  - `SetGoal3D` still publishes `/move_base_simple/goal`;
  - the key difference is UI support for choosing z by right-click/drag, not a separate 3D goal topic.
- Key mapping/floor finding:
  - `set_floor_label.sh` publishes `std_msgs/Int8` once to `/mapping/floor_label`;
  - occupancy mapping launch has `label_file_path` defaulting to `system/maps/default/details/labels.txt`;
  - X30 floor labels are recorded alongside keyframe poses/frames and are used to generate multiple occupancy maps from one mapped dataset.
- Key map product finding:
  - mapping creates timestamped map folders under `system/maps/<name>-<time>` and symlinks `system/maps/default` plus optionally `system/map`;
  - full 3D map is generated from `details/poses_opt.txt` or `details/poses_ori.txt` and `details/frames` into `jueying.pcd`;
  - occupancy map is saved as `jueying.yaml/pgm` from `/projected_map`;
  - `pcd2map` config filters PCD by height (`minh: 0.1`, `maxh: 1.0`) and publishes projected occupancy map.
- Key localization finding:
  - localization uses `system/map/jueying.pcd` as the global registration map;
  - `map_server` simultaneously loads `system/map/jueying.yaml`;
  - localization config includes x/y/z initial pose, PCD offline preprocessing, GICP/fast-ICP parameters, and localization health monitoring thresholds.
- Key planning finding:
  - global planning is still ROS1 move_base style with `global_planner/GlobalPlanner` and 2D costmaps;
  - local/global costmaps use `spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer` on `/lidar_points`, so local obstacle reasoning is 3D/voxel-aware;
  - RViz displays `/globalmap`, 2D costmaps, `LocalCostMap3D` voxel grid, global path, and local path;
  - action definitions are still `geometry_msgs/PoseStamped target_pose`, including z but not a custom full 3D planner interface.
- Key local planner finding:
  - local planner consumes point cloud, filters relative z range, and outputs `/cmd_vel_corrected` according to changelog;
  - changelog explicitly mentions collision detection, autonomous navigation obstacle stopping, direct-line navigation, saved path execution, and stair gait stop distance tuning.
- Practical M20Pro implications:
  - X30 confirms the product-level "3D navigation" is an integration of PCD localization, PGM/global planning, 3D voxel local obstacle layers, z-aware RViz goal/initialpose tools, floor labeling, saved-path/direct-line actions, and gait/task integration;
  - our M20Pro stack can improve along the same integration path without implementing a full volumetric global planner first;
  - high-value next steps are: floor/map manifest, z-aware goal/initialpose handling, saved-route/task abstractions, and a stronger local point-cloud obstacle layer.

## 2026-05-28 RViz floor goal usability pass

- User compared X30 Pro RViz interaction with the current M20Pro stack:
  - X30 Pro RViz has `2D Pose Estimate`, `2D Goal Pose`, `3D Pose Estimate`, and `3D Goal Pose`;
  - M20Pro stack currently uses standard RViz2 tools and does not include the vendor 3D RViz plugin.
- Current M20Pro boundary:
  - true X30-style 3D RViz tools would require a custom RViz plugin or vendor plugin;
  - our existing `initialpose_3d_adapter` can add floor-aware z to normal `/initialpose`, but the UI is still standard 2D RViz.
- Practical improvement made:
  - updated `m20pro_sim.rviz` tool names and added floor-specific `SetGoal` tools;
  - tools now include `Same Floor Goal`, `F19 Goal`, `F20 Goal`, and `F21 Goal`;
  - floor tools publish to `/m20pro/rviz_goal_f19`, `/m20pro/rviz_goal_f20`, `/m20pro/rviz_goal_f21`, which `floor_manager` already converts into `/m20pro/floor_goal` semantics.
- README updated to explain the RViz floor goal workflow and the difference from X30 Pro's 3D plugin.

## 2026-05-28 X30 Pro scripts review

- User placed X30 Pro shell scripts under local `scripts/`.
- Most `._*` files are macOS metadata sidecar files and should be ignored.
- Real script set is mostly a high-level control layer for the vendor `jy_cog` stack, not the underlying SLAM/localization/planner implementation.
- Useful architectural clues:
  - mapping startup explicitly stops localization and navigation first, then starts SLAM mapping;
  - map saving is separated from mapping;
  - occupancy map generation is separated from full-map construction;
  - `set_floor_label.sh` calls into vendor SLAM scripts and suggests X30 multi-floor mapping uses explicit floor labels/indexes;
  - localization, global planning, and local planning are started as separate stages;
  - startup scripts sequence drivers, URDF, transfer/broker, CPU monitor, video/charge services, and only then optional localization/planning;
  - X30 startup also references `vmap`, `launcher`, `net_point`, `motion_rl`, and `patrolserver`, which are vendor-side modules not present in this M20Pro workspace.
- Direct reuse is not recommended:
  - scripts assume ROS1 Kinetic/Noetic and `/home/ysc/jy_cog`;
  - they call vendor internal scripts such as `slam/scripts/mapping.sh`, `set_floor_label.sh`, `pcd2map.sh`, `save_map.sh`, `localization.sh`, `plan.sh`, and `local_planner.sh`, which are not included here.
- Worth borrowing into M20Pro workflow:
  - add an explicit map workflow guide or helper wrapper with stages: stop nav/localization, record/build map, set/record floor metadata, construct PGM/PCD products, save/export, then restart localization/nav;
  - preserve floor metadata in our `inspection_waypoints.yaml` or a future map manifest, rather than treating folder names alone as floor truth;
  - treat PCD/full map and occupancy PGM as two products of one map session and keep their provenance together;
  - consider systemd services only after real-machine startup stabilizes.

## 2026-05-27 README package usage refresh

- Rewrote `README.md` into a current project usage guide.
- README now covers all active packages:
  - `m20pro_bringup`;
  - `m20pro_navigation`;
  - `m20pro_description`;
  - `m20pro_inspection`;
  - `m20pro_cloud_bridge`.
- Added current quick-start commands for sim, real, and the local web dashboard.
- Added current multi-floor usage:
  - `/m20pro/floor_goal` with `header.frame_id` as target floor;
  - RViz floor-specific goal topics `/m20pro/rviz_goal_f19`, `/m20pro/rviz_goal_f20`, `/m20pro/rviz_goal_f21`;
  - shared stair-platform route semantics.
- Added current map/PCD placement guidance and common ROS diagnostic commands.
- Preserved project ownership/copyright warnings and deployment cautions.

## 2026-05-27 Local web dashboard MVP

- Added a new `m20pro_cloud_bridge` package instead of placing more nodes under `m20pro_navigation`.
- Added `web_dashboard` node:
  - starts a lightweight HTTP server, default `0.0.0.0:8080`;
  - serves `/` as a browser dashboard;
  - serves `/api/state`, `/api/map`, and `/healthz`;
  - subscribes to current floor, stair status, gait command, robot pose, global path, map, dynamic obstacle markers, YOLO detections, and YOLO events.
- Added `m20pro_web_dashboard.launch.py` for independent startup:
  - `ros2 launch m20pro_bringup m20pro_web_dashboard.launch.py port:=8080`
- Integrated the dashboard into both sim and real launch files with:
  - `enable_web_dashboard` default `true`;
  - `web_dashboard_port` default `8080`.
- Current scope:
  - local browser visibility only;
  - no customer-server protocol yet;
  - no video streaming yet, only detection/event JSON and navigation/map overlays.
- Verification:
  - Python compile check passed;
  - `colcon build --packages-select m20pro_cloud_bridge m20pro_bringup --symlink-install` passed;
  - temporary launch on port `18080` returned healthy `/healthz`, `/api/state`, and HTML page.

## 2026-05-27 Dynamic obstacles default policy

- User explicitly requires dynamic obstacles to be displayed in simulation by default.
- `m20pro_sim.launch.py` now defaults `enable_dynamic_obstacles:=true`.
- RViz `DynamicObstacles` display is already enabled and subscribes to `/dynamic_obstacle_markers`.
- Do not disable dynamic obstacles by default in future sim changes. Only set `enable_dynamic_obstacles:=false` for focused debugging when the user asks for an isolated test.

## 2026-05-27 Real launch and 3D relocalization pass

- User asked to make the real launch path match the new multi-floor logic and to review X30 Pro features beyond stair switching.
- `m20pro_real.launch.py` now starts `m20pro_floor_manager` by default, with launch arguments:
  - `floor_config`;
  - `enable_floor_manager`;
  - `initial_floor`;
  - `load_initial_floor`.
- Real launch still keeps M20Pro interfaces and defaults:
  - map defaults to F20;
  - cloud topic defaults to `/cloud_nav`, but can be overridden to `/LIDAR/POINTS`;
  - axis command remains disabled by default.
- Added z-aware initial pose support inspired by X30 Pro's 3D Pose Estimate workflow:
  - `tcp_bridge_node.py` now supports `/m20pro/initialpose_3d` as a `PoseStamped` relocalization input and forwards `PosX/PosY/PosZ/Yaw` to the M20 body protocol.
  - `/initialpose` 2D relocalization remains supported.
  - `m20pro.yaml` now declares `initialpose_3d_topic`.
- Added `initialpose_3d_adapter`:
  - subscribes to RViz `/initialpose`;
  - adds z from the active floor in `inspection_waypoints.yaml`, falling back to `initialpose_3d_z`;
  - republishes `/m20pro/initialpose_3d`.
  - Intended use: keep ordinary RViz 2D Pose Estimate operation, but route it through a floor-aware z value when `enable_initialpose_3d_adapter:=true`.
- `inspection_waypoints.yaml` now includes explicit `z` fields on floor initial poses and stair transition poses. They are currently 0.0 placeholders and should be updated from real map/PCD/localization data before serious real multi-floor testing.
- Verification:
  - Python compile check for `floor_manager.py`, `tcp_bridge_node.py`, `initialpose_3d_adapter.py`, and `m20pro_real.launch.py`;
  - YAML z-aware route validation;
  - `colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install`;
  - short real launch checks with `rviz:=false`, `cloud_topic:=/LIDAR/POINTS`, and `enable_initialpose_3d_adapter:=true` showed the floor manager and initialpose adapter start correctly.

## 2026-05-27 X30-inspired multi-floor logic pass

- User asked to move the current M20Pro multi-floor logic closer to Deep Robotics X30 Pro's official multi-floor idea, while keeping all M20Pro interfaces unchanged.
- Important boundary kept:
  - still M20Pro, not X30 Pro;
  - no public topic/service/interface rename;
  - `/m20pro/floor_goal`, `/m20pro/gait_command`, `/m20pro/current_floor`, `/m20pro/stair_status`, map loading, and TCP bridge integration remain the M20Pro-side interfaces.
- Manual-inspired logic now modeled explicitly as shared stair-platform transitions:
  - navigate to stair entry;
  - switch to stair gait;
  - navigate to source-floor `source_platform`;
  - switch floor map at the platform;
  - reset pose at target-floor `target_platform`;
  - navigate to `post_exit` while still in stair gait;
  - switch back to flat gait;
  - continue final floor goal or next floor hop.
- `inspection_waypoints.yaml` now declares:
  - `mission.multi_floor_model: shared_stair_platform`;
  - default transition metadata: transition point, stairs terrain, straight nav mode, low speed, stop-only obstacle policy, 0.8 m entry margin;
  - active stair routes use `source_platform` and `target_platform` instead of the older `traverse_to` / `target_exit` names.
- `floor_manager.py` now:
  - reads the new transition metadata but keeps compatibility with old route fields;
  - validates stair route configs on startup and warns for missing entry/source platform/target platform/post-exit points;
  - logs status names around platform semantics: `navigating_to_stair_entry`, `navigating_to_stair_platform`, `switching_map_at_platform`, `navigating_from_platform_to_flat`;
  - rejects shared-platform stair traversal if no source platform pose is configured, instead of silently falling back to a timed stair traversal.
- Verification:
  - `python3 -m py_compile src/m20pro_navigation/m20pro_navigation/floor_manager.py`
  - YAML safe-load and route assertion for `inspection_waypoints.yaml`
  - `colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install`

## 2026-05-26 Sim startup robustness pass

- User reported repeated unstable sim startup symptoms: missing local costmap, robot model display problems, and dynamic obstacle display surprises.
- Reproduced a clean no-RViz launch and found the backend can start correctly:
  - F20 map loads;
  - local costmap configures and activates;
  - `/cloud_nav`, `/scan`, `/local_costmap/costmap`, `/global_costmap/costmap`, and Nav2 lifecycle all come up.
- Real startup issue found: `local_costmap` may start checking TF before `sim_bridge` has published `odom`, producing short `base_link -> odom` transform wait messages. If RViz starts too early, it can show misleading missing-model/costmap states even when the backend recovers.
- Changes:
  - `m20pro_sim.launch.py` now defaults `enable_dynamic_obstacles:=false`; enable explicitly for dynamic-obstacle tests.
  - RViz is launched through a 5 s `TimerAction` delay, configurable with `rviz_delay_s`.
  - Added `sim_health_monitor` node to report whether `/map`, `/cloud_nav`, `/scan`, `/local_costmap/costmap`, `/global_costmap/costmap`, `/robot_description`, key nodes, and Nav2 lifecycle are actually healthy.
  - `nav2_params.yaml` now sets `always_send_full_costmap: true` for local and global costmaps so RViz receives full costmap updates more reliably.
- Verification:
  - `python3 -m py_compile src/m20pro_navigation/m20pro_navigation/sim_health_monitor.py`
  - `colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install`
  - `timeout 25s ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false enable_dynamic_obstacles:=false`
  - Health monitor reported: `SIM HEALTH OK: map, robot model, scan, cloud_nav, costmaps and Nav2 are active`.

## 2026-05-26 Post-stair exit waypoint logic

- User observed that after switching to a new floor, the robot should not immediately plan to the final target. It should first leave the stair region and reach the stair-to-flat gait switch point.
- Updated `floor_manager.py` so each stair route can use `post_exit` from `inspection_waypoints.yaml`.
- New cross-floor sequence:
  - navigate to stair `entry`;
  - switch to stair gait;
  - navigate to stair `traverse_to`;
  - switch map and reset pose at `target_exit`;
  - navigate to `post_exit` while still in stair gait;
  - switch back to flat gait;
  - continue to the final floor goal, or start the next stair route if the final target is still another floor.
- Current `post_exit` for F19/F20/F21 routes is the common flat/stair gait switch point:
  - `x=8.803242683410645`
  - `y=15.900266647338867`
  - `yaw=1.5707958336440224`
- Verification:
  - `python3 -m py_compile src/m20pro_navigation/m20pro_navigation/floor_manager.py`
  - YAML safe-load check for `inspection_waypoints.yaml`
  - `colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install`

## 2026-05-26 Cross-floor RViz sim test

- Cleaned stale sim/RViz/Nav2 processes, rebuilt `m20pro_navigation` and `m20pro_bringup`, and launched `m20pro_sim.launch.py` with RViz.
- Used `enable_dynamic_obstacles:=false` for this test to isolate floor-switch behavior from obstacle-avoidance behavior.
- Pre-test checks passed:
  - `/robot_description`, `/map`, `/local_costmap/costmap`, `/scan`, `/cloud_nav`, `/m20pro_tcp_bridge/map_pose`, and `/m20pro/current_floor` all had publishers;
  - map server, controller, planner, and local costmap lifecycle nodes were `active`;
  - RViz was subscribed to robot description and local costmap.
- Published a cross-floor goal:

```bash
ros2 topic pub --once /m20pro/floor_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'F21'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

- Observed successful sequence:
  - current floor before goal: `F20`;
  - navigate to stair entry `(8.80, 15.90)` succeeded;
  - gait command switched to `stair_up`;
  - stair traverse goal `(8.70, 10.09)` succeeded;
  - floor manager switched map from `F20` to `F21`;
  - gait command switched back to `flat`;
  - final F21 goal `(2.0, 0.0)` succeeded.
- During final navigation RPP briefly reported `detected collision ahead`, but recovery/navigation continued and the final goal succeeded.

## 2026-05-26 Parameter section audit

- Audited launch node names, Python node names, and YAML top-level parameter sections after the `initial_x` mismatch.
- Current custom YAML sections in `m20pro.yaml` match active custom node names where they are expected:
  - `m20pro_tcp_bridge`: used by real `tcp_bridge` and sim `sim_bridge` because both publish the same `/m20pro_tcp_bridge/...` topic namespace.
  - `m20pro_dual_lidar_simulator`
  - `m20pro_pointcloud_fusion`
  - `m20pro_dynamic_obstacle_simulator`
  - `m20pro_grid_planner` and `m20pro_path_follower` are optional old/custom nodes and are not launched by the current main sim/real launch.
- `m20pro_floor_manager` does not have a top-level YAML section in `m20pro.yaml`; it receives the active config file and initial floor inline from `m20pro_sim.launch.py`.
- `zero_joint_state_publisher` also has no YAML section; it receives `robot_description` inline.
- `m20pro_inspection/config/yolov8_inspection.yaml` uses `m20pro_yolov8_inspection`, matching the inspection launch node name.
- Runtime sim parameter check passed for the active custom nodes:
  - `/m20pro_tcp_bridge initial_x = -5.0`;
  - `/m20pro_dual_lidar_simulator pcd_map_path = package://m20pro_bringup/maps/Original_map/full_cloud.pcd`;
  - `/m20pro_pointcloud_fusion input_cloud_topic = /cloud_nav`;
  - `/m20pro_dynamic_obstacle_simulator obstacle_specs` loaded;
  - `/m20pro_floor_manager initial_floor = F20`.
- Note: `m20pro_tcp_bridge` contains both real-only TCP params and sim-only initial pose params because the sim node intentionally uses the same node name/topic namespace as the real bridge. Undeclared parameters are ignored by the node that does not use them.

## 2026-05-26 Sim initial pose parameter fix

- User changed `initial_x` in `m20pro.yaml` but RViz did not show any initial pose change.
- Root cause: `m20pro_sim.launch.py` starts the `sim_bridge` executable with node name `m20pro_tcp_bridge` so that sim topics match real topics such as `/m20pro_tcp_bridge/map_pose`.
- Therefore ROS 2 reads the `m20pro_tcp_bridge:` YAML section for sim startup too; the old `m20pro_sim_bridge:` YAML section was not applied.
- Fixed `m20pro.yaml` by moving sim startup parameters into the active `m20pro_tcp_bridge.ros__parameters` section:
  - `initial_x`
  - `initial_y`
  - `initial_yaw`
  - `update_rate_hz`
  - `tf_future_offset_s`
- Removed the unused `m20pro_sim_bridge:` section to avoid future confusion.
- Verification:
  - runtime `/m20pro_tcp_bridge initial_x` parameter reported `-5.0`;
  - `/m20pro_tcp_bridge/map_pose` published `x=-5.0`, `y=0.0` after clean sim startup.

## 2026-05-26 RViz startup display check

- User reported RViz RobotModel error, missing costmap, and missing dynamic obstacles.
- A clean `m20pro_sim.launch.py` launch showed the backend was healthy:
  - `/robot_description` published by `robot_state_publisher` and subscribed by RViz;
  - `/local_costmap/costmap` published and subscribed by RViz;
  - `/dynamic_obstacle_markers` published by `m20pro_dynamic_obstacle_simulator` and subscribed by RViz;
  - `/map`, `/scan`, and Nav2 lifecycle nodes were active.
- The concrete RViz error found in logs was TF future extrapolation from `base_link` to `map`, not a missing URDF/model file.
- Added `m20pro_sim_bridge.tf_future_offset_s` parameter in `sim_bridge_node.py` with default `0.1 s`, and TF is now stamped slightly ahead of pose/odom to absorb RViz/message-filter timing jitter in simulation.
- Rebuilt `m20pro_navigation` and retested with RViz:
  - future extrapolation error disappeared;
  - map/local costmap/dynamic obstacle marker topics were all present;
  - the only remaining early warning was Nav2 waiting briefly for `odom`, which is expected during startup before sim TF begins publishing.

## 2026-05-25 Stair transition waypoint update

- Updated `inspection_waypoints.yaml` with the latest stair transition poses from RViz.
- Flat-to-stair gait switch pose:
  - `x=8.803242683410645`, `y=15.900266647338867`, yaw about `-90 deg`.
  - Reverse yaw about `+90 deg` is used where the same point is approached in the opposite stair direction.
- Up-floor switch pose:
  - `x=8.69885540008545`, `y=10.093945503234863`, yaw about `-0.94 deg`.
  - Target exit yaw is this pose rotated by 180 deg, about `179.06 deg`.
- Down-floor switch pose:
  - `x=10.25889778137207`, `y=10.056803703308105`, yaw about `179.89 deg`.
  - Target exit yaw is this pose rotated by 180 deg, about `-0.11 deg`.
- Configured routes:
  - `F19 -> F20` and `F20 -> F21` use flat-to-stair entry, up-floor `traverse_to`, and flipped up-floor `target_exit`.
  - `F20 -> F19` and `F21 -> F20` use reversed flat-to-stair entry, down-floor `traverse_to`, and flipped down-floor `target_exit`.
- YAML parsing passed and `colcon build --packages-select m20pro_bringup --symlink-install` passed.

## 2026-05-25 Rebuilt edited F19/F20/F21 maps

- User re-edited the floor PGM maps and replaced the active map set with new `F19`, `F20`, and `F21` directories.
- Source map set now contains:
  - `src/m20pro_bringup/maps/F19/occ_grid.yaml` + `occ_grid.pgm`
  - `src/m20pro_bringup/maps/F20/occ_grid.yaml` + `occ_grid.pgm`
  - `src/m20pro_bringup/maps/F21/occ_grid.yaml` + `occ_grid.pgm`
  - `src/m20pro_bringup/maps/Original_map/full_cloud.pcd`
- The edited floor dirs no longer contain their own `full_cloud.pcd`, so `m20pro.yaml` now points the PCD-backed simulator to `package://m20pro_bringup/maps/Original_map/full_cloud.pcd`.
- Removed stale installed map artifacts and rebuilt `m20pro_bringup` / `m20pro_navigation` with `--symlink-install`.
- Verification:
  - installed maps now contain only `F19`, `F20`, `F21`, and `Original_map`;
  - `map_server` loaded the rebuilt `F20/occ_grid.yaml` as `436 x 515 @ 0.1 m/cell`;
  - `dual_lidar_simulator` loaded `Original_map/full_cloud.pcd` successfully;
  - `/cloud_nav` published a local cloud in `base_link`.

## 2026-05-25 Sim twitch / stale local costmap root cause

- User reported that after re-running sim, the robot model twitched and the local costmap appeared at the previous sim end location.
- Root cause found locally: orphaned processes from May 22 were still alive and publishing same-name topics/TF:
  - two old `sim_bridge` processes publishing `/m20pro_tcp_bridge/map_pose`, `/odom`, and `map->odom->base_link` TF;
  - two old `floor_manager` processes;
  - two old `zero_joint_state_publisher` processes;
  - one old `dynamic_obstacle_simulator`.
- This caused duplicate TF/pose/joint publishers, so the new sim and old sim states fought each other.
- Cleaned the stale processes and restarted the ROS daemon.
- Verification with a clean no-RViz launch:
  - initial `/m20pro_tcp_bridge/map_pose` was `(0.0, 0.0)`;
  - `/local_costmap/costmap` origin was `(-2.45, -2.45)`, matching the normal 5 m rolling window around the initial robot pose;
  - no duplicate nodes remained after shutdown.
- Practical rule: if sim looks twitchy or starts from an old location, first check for stale processes before changing maps or Nav2 parameters.

## 2026-05-22 Official firmware update: 104 can see `/LIDAR/POINTS`

- Deep Robotics official staff assisted onsite debugging and installed a new robot firmware.
- After the firmware update, host 104 can now see `/LIDAR/POINTS`.
- This is a key real-deployment progress point: the 104-side custom navigation stack now has a plausible live lidar point cloud input path instead of relying only on offline maps or simulation clouds.
- Next checks:
  - verify `/LIDAR/POINTS` message rate, bandwidth, QoS, timestamp, and frame id on 104;
  - verify TF from the lidar frame to `base_link` / `map`;
  - test whether `m20pro_real.launch.py` can use `/LIDAR/POINTS` as `cloud_topic` if `/cloud_nav` is still unavailable;
  - record a focused bag containing `/LIDAR/POINTS`, pose, TF, gait/motion state, and navigation commands.

## 2026-05-22 Sim RViz / local costmap cleanup

- User reported that sim RViz local costmap was visually filled blue and RViz had three `2D Goal Pose` toolbar buttons.
- Cause:
  - The three toolbar buttons came from three RViz `SetGoal` tools for F19/F20/F21. RViz displays all of them with the same `2D Goal Pose` tool icon/name.
  - The PCD-backed sim point cloud can include low ground / self / mapping-remnant points. When those points are projected into `/scan`, local costmap inflation can visually fill narrow corridors.
- Changes:
  - `m20pro_sim.rviz` now has one normal `SetGoal` tool publishing `/goal_pose`.
  - Sim launch overrides `pointcloud_fusion` with stricter sim-only filtering: `height_min=0.05`, `height_max=0.85`, `robot_radius=0.45`.
  - Humble sim `nav2_params.yaml` local inflation reduced to `0.28` with faster decay; global inflation reduced to `0.30`.
  - `dual_lidar_simulator` now subscribes to `/map` and filters static PCD points through the current occupancy grid. Static PCD points in PGM-free cells are dropped, so old mapping remnants in corridor centers do not become local obstacles again.
  - `pointcloud_fusion` now publishes LaserScan no-return bins as `inf` by default instead of finite `range_max`.
  - `m20pro_sim.launch.py` now keeps `dynamic_obstacle_simulator` disabled by default. Turn it back on with `enable_dynamic_obstacles:=true` only when testing dynamic avoidance. The demo obstacles start near the robot and can make the local costmap look much more blue.
  - RViz `LocalCostmap` alpha reduced to `0.35`, so the inflated layer no longer hides the base map as aggressively.
- Verification:
  - A clean launch received `/map` correctly and logged `occupancy filter map received: 436x515 occupied_or_kept=7390`.
  - With default dynamic obstacles disabled, `/scan` nearest finite obstacle was about `0.62 m`; `/local_costmap/costmap` was `6871/10000` free cells, `316/10000` occupied cells, and the rest inflated cost.
- Debug note:
  - Killing only the `ros2 launch` parent can leave orphaned sim/Nav2 child processes. Duplicate same-name nodes can make map_server/costmap behavior misleading. Clean old sim processes before judging RViz output.
- Note: this only cleans the sim/RViz workflow. Real launch still defaults to the manual-aligned `/cloud_nav -> /scan` chain and should be tuned separately after live point-cloud inspection.

## 2026-05-22 Manual-grounded project audit

- Rechecked the project against `山猫M20系列软件开发手册V0.0.9.pdf`.
- Confirmed correct manual bindings:
  - TCP body protocol server: `10.21.31.103:30001`.
  - RTSP front/rear wide cameras: `rtsp://10.21.31.103:8554/video1` and `video2`.
  - AOS 103 DDS topics: `/IMU_YESENSE`, `/LIDAR/POINTS`, `/cloud_nav`, `/DEPTH_IMAGE`.
  - Axis command bridge: `Type=2`, `Command=21`, 20 Hz normalized X/Y/Yaw.
  - Relocalization bridge: `/initialpose` -> `Type=2101`, `Command=1`, `PosX/PosY/PosZ/Yaw`.
  - Map pose/status polling: `Type=1007 Command=2` and `Type=2002 Command=1`.
- Fixed manual mismatch:
  - `floor_manager` publishes `/m20pro/gait_command`; `tcp_bridge` now converts it to the manual gait-switch request `Type=2`, `Command=23`, `GaitParam`.
  - `flat` maps to `GaitParam=1`; `stair_up` and `stair_down` map to `GaitParam=14`.
  - `control_gui` navigation-task default gait now uses manual value `Gait=12` instead of `0x3002`.
  - `control_gui` motion-state selector now includes manual `MotionParam=6` standard motion mode.
- Fixed robustness issues:
  - Native goal yaw and grid planner final yaw now use the common quaternion-to-yaw helper instead of assuming x/y quaternion components are zero.
  - README now points to `maps/F20/occ_grid.yaml` and describes the PCD-backed `/cloud_nav` simulation chain.
  - `tools/collect_ros_snapshot.sh` now records checks for `/IMU_YESENSE`, `/DEPTH_IMAGE`, `/STEER`, `/REAL_STEER`, and `multicast-relay`.
- Cleaned generated install-map residue:
  - Removed stale `F1`, `F2`, and `F3` directories from `install/m20pro_bringup/share/m20pro_bringup/maps`.
  - Remaining installed map dirs are `F19`, `F20`, `F21`, and `original_map`.

## 2026-05-22 Correction: factory lidar/cloud topics are AOS 103 DDS topics

- Rechecked `山猫M20系列软件开发手册V0.0.9.pdf`.
- The manual explicitly says section "运动主机（AOS）DDS 话题" describes DDS topics in the motion host `10.21.31.103`.
- Sensor driver topics on AOS 103:
  - `/IMU_YESENSE`: `sensor_msgs/msg/Imu`, 200 Hz.
  - `/LIDAR/POINTS`: `sensor_msgs/msg/PointCloud2`, 10 Hz.
- Height-map related AOS 103 topics:
  - `/cloud_nav`: `sensor_msgs/msg/PointCloud2`, described as obstacle point cloud data.
  - `/DEPTH_IMAGE`: `sensor_msgs::msg::Image`, panoramic depth image.
- Correction to earlier assumption: `/cloud_nav` is not only a local simulation topic; it is documented as a factory AOS DDS topic. Our simulator reused the factory topic name.
- If 104/106 do not consistently see `/LIDAR/POINTS` or `/cloud_nav`, treat it first as a DDS discovery / multicast relay / QoS / network visibility problem, not proof that AOS 103 is not publishing.

## 2026-05-22 Update: F20 map replaces F1/F2/F3 demo maps

- Replaced the active sim map set with real recorded F20 map data.
- New active map directories:
  - `src/m20pro_bringup/maps/F19`
  - `src/m20pro_bringup/maps/F20`
  - `src/m20pro_bringup/maps/F21`
- `F19` and `F21` currently reuse the same `occ_grid.pgm` / `occ_grid.yaml` as `F20`.
- `src/m20pro_bringup/maps/F20/full_cloud.pcd` stores the single PCD used by the PCD-backed local perception simulator.
- `m20pro.yaml` now points `pcd_map_path` to `package://m20pro_bringup/maps/F20/full_cloud.pcd`.
- `m20pro_sim.launch.py` and `m20pro_real.launch.py` now default to the F20 map; sim `initial_floor` defaults to `F20`.
- `inspection_waypoints.yaml` now defines floors `F19`, `F20`, and `F21`, with levels `19`, `20`, and `21`.
- RViz floor-goal tools now use `Goal F19`, `Goal F20`, `Goal F21` and topics `/m20pro/rviz_goal_f19`, `/m20pro/rviz_goal_f20`, `/m20pro/rviz_goal_f21`.
- Build verified:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install
```

## 2026-05-22 Update: Web live camera streaming requirement

- Client requested real-time robot camera video to be pushed or served to their web system.
- This should be treated as a separate video transport chain from navigation.
- The M20 software development manual states that the front/rear wide cameras use RTSP:
  - front wide camera: `rtsp://10.21.31.103:8554/video1`
  - rear wide camera: `rtsp://10.21.31.103:8554/video2`
- Existing `m20pro_inspection` config already defaults to RTSP input using `rtsp://10.21.31.103:8554/video1`.
- ROS image topics seen in bags (`/HEIGHT_IMAGE`, `/DEPTH_IMAGE`) had 0 messages and should not be treated as the primary front/rear camera source unless later verified live.
- Preferred implementation depends on the real camera output:
  - RTSP/H.264 source: relay/transmux to WebRTC or HLS for browser viewing.
  - ROS2 `sensor_msgs/Image` source: encode/compress first, then publish through a web gateway.
  - OpenCV-only source: capture/encode and push to a media server.
- For low latency teleoperation-like viewing, WebRTC is the best target; MJPEG is easiest for LAN demos but wastes bandwidth.
- Need to confirm on the real robot whether the camera is exposed as ROS2 Image, RTSP, USB/V4L2, or vendor SDK.

## 2026-05-21 Update: map_editor factory YAML compatibility

- `map_editor.py` now tolerates factory `occ_grid.yaml` files whose `image:` field points to an unavailable robot-side absolute path such as `/var/opt/robot/data/maps/.../occ_grid.pgm`.
- If that absolute image path does not exist locally, the editor falls back to a PGM with the same basename next to the YAML file.
- Verified with `map-20260520-205606/occ_grid.yaml`: it resolves to local `map-20260520-205606/occ_grid.pgm` and reads as `436 x 515`.

## 2026-05-21 Update: 104 mapping bag and map analysis

- User added `bags/map` and `bags/rosbag104` from a 104-host recording during factory mapping.
- The rosbag under `bags/map/rosbag2_2026_05_20-20_58_07` and `bags/rosbag104/rosbag2_2026_05_20-20_58_07` are byte-identical DB3 files.
- Bag duration is about 285.8 s with 245488 messages.
- Nonzero useful topics include `/SLAM_ODOM`, `/slam_optimized_pose`, `/IMU`, `/LIDAR/IMU201`, `/LIDAR/IMU202`, `/JOINTS_DATA`, `/MOTION_INFO`, `/STEER`, `/HANDLE_STEER`, `/GAIT`, `/BATTERY_DATA`, `/FAULT_STATUS`, `/CPU_104`, and `/CPU_106`.
- No usable point cloud was recorded: `/SLAM_ALIGNED_POINTS` exists as `sensor_msgs/msg/PointCloud2` but has 0 messages; `/LIDAR/POINTS` and `/cloud_nav` are absent.
- `/SLAM_ODOM` decodes correctly and covers roughly x `[-12.672, 13.316]`, y `[-0.588, 23.369]`, z `[-2.304, 2.066]`, with about 230.6 m XY path length.
- The factory map folder `bags/map/map-20260520-205606` contains `occ_grid.pgm`, `occ_grid.yaml`, `full_cloud.pcd`, block/session files, and 300 per-frame session lidar PCD files.
- `full_cloud.pcd` has 1434319 binary points with fields `x y z intensity`; rough bounds are x `[-17.912, 26.230]`, y `[-17.235, 37.440]`, z `[-4.015, 9.927]`.
- `occ_grid.pgm` is `436 x 515`, resolution `0.1`, origin `[-17.7, -14.3, 0.0]`.

## 2026-05-21 Update: 104 real deployment perception requirement

- Current `m20pro_real.launch.py` expects a real-time obstacle source that can become `/scan`.
- The current chain is `PointCloud2 -> m20pro_pointcloud_fusion -> /scan -> Nav2 costmaps/path follower`.
- The real deployment does not require the topic to be named `/LIDAR/POINTS` or `/cloud_nav`; it only requires a live `PointCloud2` or `LaserScan` source with usable frame/timing.
- If 104 exposes no live point cloud/scan topic, ROS2/Nav2 cannot create real obstacle data by itself.
- Viable paths are:
  - find and use a factory-published live `PointCloud2`/`LaserScan` topic during mapping/navigation;
  - start or install the vendor lidar/perception driver on 104;
  - bridge a private/vendor DDS or another-host lidar stream into a normal ROS2 topic;
  - change architecture so our code sends high-level goals to factory navigation instead of owning local obstacle avoidance.
- Publishing local PCD crops on 104 can make Nav2 run, but it is static-map simulation, not real-time obstacle perception.

## 2026-05-20 Update: PGM + PCD Simulation Sensor Chain

The simulation sensor chain was upgraded from a PGM-only raycast model to a PCD-backed local perception model.

Current design:

```text
occ_grid.pgm/yaml
  -> Nav2 map_server
  -> 2D global planning and costmaps

full_cloud.pcd
  -> m20pro_dual_lidar_simulator
  -> local PCD crop in base_link
  -> /cloud_nav and /grid_map_3d
  -> pointcloud_fusion
  -> /scan
  -> Nav2 obstacle layers
```

Important details:

- `dual_lidar_simulator.py` keeps the same executable and output topics, but no longer does PGM raycasting.
- It loads a factory PCD map at startup, voxel-downsamples it, builds a 2D XY cell index, and crops nearby 3D points around `/m20pro_tcp_bridge/map_pose`.
- Dynamic obstacle simulation is still injected into the local point cloud.
- `/cloud_nav` remains the main simulated navigation cloud, aligned with the real robot topic name.
- `/grid_map_3d` is also published from the same local PCD crop for closer comparison with factory rosbag data.
- `pointcloud_fusion.py` remains unchanged and still converts `/cloud_nav` into `/scan`.

Changed files:

- `src/m20pro_navigation/m20pro_navigation/dual_lidar_simulator.py`
- `src/m20pro_bringup/config/m20pro.yaml`

Current PCD config in `m20pro.yaml`:

```yaml
m20pro_dual_lidar_simulator:
  ros__parameters:
    pcd_map_path: "working_1-20260429-162852/full_cloud.pcd"
    pcd_voxel_size: 0.08
    pcd_index_cell_size: 1.0
    floor_z_ranges:
      - "F1:-1.0:1.5:0.0"
      - "F2:-1.0:1.5:0.0"
      - "F3:-1.0:1.5:0.0"
```

Reason for using the old `working_1-20260429-162852/full_cloud.pcd` by default:

- The installed `F1/F2/F3` PGM maps currently come from the old map.
- For metric comparisons, the PGM and PCD should come from the same factory map package.
- If switching to `map-20260519-204926/occ_grid.yaml`, also switch `pcd_map_path` to `map-20260519-204926/full_cloud.pcd`.

Verification completed:

```bash
python3 -m py_compile src/m20pro_navigation/m20pro_navigation/dual_lidar_simulator.py
colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install
ros2 run m20pro_navigation dual_lidar_simulator --ros-args --params-file src/m20pro_bringup/config/m20pro.yaml
```

Observed startup result:

```text
loaded PCD map .../working_1-20260429-162852/full_cloud.pcd
points=257820
PCD local perception simulator ready: 257820 indexed points -> /cloud_nav
```

Manual topic check:

- Published a test pose to `/m20pro_tcp_bridge/map_pose`.
- `/cloud_nav` published local cloud with about 4.8k points.
- `/grid_map_3d` published local cloud with about 4.8k points.

Engineering caveat:

- This is still not full factory-grade 3D navigation.
- Nav2 still plans globally on 2D PGM/YAML.
- PCD is now used to make simulated local perception closer to the real 3D map, and to enable future comparison with factory `/grid_map_3d` / `/local_map` rosbag data.

## 2026-05-20 Decision: Navigation Authority Between PGM and PCD

Current authority split:

```text
PGM/YAML
  -> map_server
  -> Nav2 global map and global planning
  -> decides where the robot is allowed to plan globally

PCD
  -> dual_lidar_simulator
  -> /cloud_nav and /grid_map_3d
  -> pointcloud_fusion
  -> /scan
  -> local obstacle perception in simulation
```

Therefore:

- Global navigation is currently authoritative to the edited PGM/YAML.
- Local simulated perception is currently authoritative to the PCD crop.
- If edited PGM and raw PCD disagree, Nav2 may plan through an area that the simulated PCD still reports as occupied.

Known mismatch risk:

- User edits PGM to remove black dots caused by people or temporary objects during mapping.
- Raw PCD still contains those people/temporary objects.
- Result: global planner thinks the area is free, but local costmap may see a simulated obstacle from PCD and stop or avoid.

Practical rule:

```text
For route planning: trust the cleaned PGM.
For local perception realism: use PCD.
For stable final simulation: clean or filter the PCD to match the cleaned PGM.
```

Recommended future map package layout:

```text
F20/
  occ_grid.yaml
  occ_grid.pgm
  full_cloud.pcd
  full_cloud_cleaned.pcd

F21/
  occ_grid.yaml
  occ_grid.pgm
  full_cloud.pcd
  full_cloud_cleaned.pcd
```

Then point `pcd_map_path` at `full_cloud_cleaned.pcd` for stable simulation.

Mapping strategy decision:

- Avoid one continuous 2D map for multiple complete floors because the exported PGM compresses floors into one plane.
- Prefer one map per floor plus only local stair connection areas.
- For a middle floor such as F20, map:
  - the full F20 inspection area,
  - the lower stair connection / arrival area from F19,
  - the upper stair connection / departure area to F21.
- Do not include the full F19 and full F21 spaces in the same F20 map.

## Project Goal

Build a ROS 2 based M20 Pro inspection and navigation stack for real robot deployment:

- Use the official M20 Pro 106/NOS map and localization as the real robot coordinate authority.
- Run custom planning, dynamic obstacle handling, patrol logic, and inspection perception on 104/GOS.
- Send low-level velocity commands to 103/AOS through the official TCP body protocol.
- Eventually support autonomous multi-floor inspection without relying on hand-controller waypoint navigation.

## Main Architecture

Real robot chain:

```text
106 NOS active map
  -> 106 localization
  -> 103 TCP 1007/2 current map pose
  -> 104 tcp_bridge publishes /m20pro_tcp_bridge/map_pose, /odom, TF
  -> 104 Nav2 uses a same-origin occ_grid map for planning
  -> /cmd_vel
  -> tcp_bridge converts to M20 axis command Type=2 Command=21
  -> 103 AOS executes motion
```

Sensor chain:

```text
/cloud_nav
  -> pointcloud_fusion
  -> /scan
  -> Nav2 local/global costmaps
```

Inspection chain:

```text
M20 Pro RTSP camera
  -> m20pro_inspection YOLOv8 node
  -> /m20pro_yolov8_inspection/detections
  -> /m20pro_yolov8_inspection/events
  -> future patrol/inspection manager
```

## Host Roles

- `103 AOS`: motion control, system monitoring, sensor topics, official TCP/UDP body protocol. TCP server is `10.21.31.103:30001`.
- `106 NOS`: mapping, localization, official navigation, obstacle avoidance, autonomous charging. Current active map is `/var/opt/robot/data/maps/active`.
- `104 GOS`: user development host. This workspace should run custom Nav2, TCP bridge, pointcloud fusion, patrol logic, and YOLO inspection.

## Map and Localization Decisions

- On the real robot, the 106 active map is the primary map because 106 localization is based on it.
- The 104 map is a planning/display copy. It must come from the same 106 active map package.
- Editing obstacle pixels in `occ_grid.pgm` is acceptable, but do not crop, rotate, rescale, or change `origin` / `resolution` / image dimensions.
- If 104 and 106 maps diverge, poses and waypoints drift.
- AMCL is not currently the main localization source. The project consumes 106 localization through the TCP bridge.
- True real-robot relocalization still needs a clean implementation: subscribe to `/initialpose`, call official TCP `2101/1`, wait for `Location=0`, then clear Nav2 costmaps.

## Important Packages

- `src/m20pro_navigation`: TCP bridge, pointcloud fusion, sim bridge, lidar simulation, map editor, control GUI.
- `src/m20pro_bringup`: launch files, maps, Nav2 params, RViz configs.
- `src/m20pro_description`: official-style URDF and meshes. Marked as vendor/proprietary resource in package metadata.
- `src/m20pro_inspection`: YOLOv8 inspection package added for RK3588/RKNN deployment.

## Launch Files

Intended main launch files:

- `m20pro_sim.launch.py`
- `m20pro_real.launch.py`
- `m20pro_inspection.launch.py`

Known issue to fix:

- `m20pro_real.launch.py` currently defaults to `working_1-20260429-162852_edited/occ_grid.yaml`, but this directory is not present in the repo. Align the real default map with an existing edited map or require explicit `map:=...` from the copied 106 active map.

## Current Git State

Remote:

```text
origin git@github.com:ghw1048040694/m20pro-ros2-navigation.git
```

Existing pushed commits:

- `c260443 Initial M20Pro ROS2 navigation stack`
- `eb95946 Document vendor asset attribution`

Current local uncommitted changes:

- Added new `src/m20pro_inspection/` package.
- Updated `.gitignore` to ignore `*.rknn`.
- Updated `README.md` with YOLO inspection usage.
- Updated `src/m20pro_bringup/package.xml` to depend on `m20pro_inspection`.
- Added this `codex.md` project memory file.

## Recent YOLO Inspection Work

User has a self-trained YOLOv8 model and wants to integrate it for inspection on RK3588.

Implemented:

- New ROS 2 Python package: `m20pro_inspection`.
- Node executable: `yolov8_inspection`.
- Launch file: `src/m20pro_inspection/launch/m20pro_inspection.launch.py`.
- Config file: `src/m20pro_inspection/config/yolov8_inspection.yaml`.
- Default RTSP input follows the software manual:
  - Front wide camera: `rtsp://10.21.31.103:8554/video1`
  - Rear wide camera: `rtsp://10.21.31.103:8554/video2`
- Default model path:
  - `src/m20pro_inspection/models/inspection.rknn`
- Optional class list:
  - `src/m20pro_inspection/models/classes.txt`
- Published topics:
  - `/m20pro_yolov8_inspection/detections`
  - `/m20pro_yolov8_inspection/events`
  - `/m20pro_yolov8_inspection/annotated_image`
- Backends:
  - `rknn` for RK3588 NPU via `rknnlite`
  - `onnx` for laptop validation via `onnxruntime`
  - `dry_run` for launch testing without a model

Verification completed:

```bash
python3 -m py_compile src/m20pro_inspection/m20pro_inspection/yolov8_inspection_node.py
colcon build --packages-select m20pro_inspection m20pro_bringup --symlink-install
source install/setup.bash
timeout 4s ros2 launch m20pro_inspection m20pro_inspection.launch.py backend:=dry_run source_type:=image_topic image_topic:=/camera/image_raw
```

All passed. The dry-run launch exits by timeout code `124`, which is expected.

## Model Deployment Notes

For RK3588 real robot deployment:

1. Export YOLOv8 `.pt` to ONNX.
2. Convert ONNX to RKNN with Rockchip RKNN-Toolkit2 targeting `rk3588`.
3. Install RKNN runtime / `rknn-toolkit-lite2` on the robot.
4. Place `inspection.rknn` under `src/m20pro_inspection/models/`.
5. Build and launch `m20pro_inspection`.

Model artifacts are intentionally ignored by Git:

```text
*.pt
*.onnx
*.rknn
*.engine
```

## Near-Term Next Steps

1. Commit and push the new `m20pro_inspection` package and this `codex.md`.
2. Fix `m20pro_real.launch.py` default map path.
3. Add real-robot `/initialpose` relocalization support in `tcp_bridge`.
4. Add a waypoint/inspection manager:
   - Navigate to task point.
   - Stop.
   - Trigger or sample YOLO detections.
   - Save inspection result.
   - Continue to next point.
5. Start single-floor real robot validation before multi-floor map switching.

## Python vs C++ Performance

User asked whether using Python hurts performance.

Current assessment:

- The custom packages are almost entirely Python:
  - `m20pro_navigation`: TCP bridge, sim bridge, pointcloud fusion, lidar simulation, dynamic obstacle simulation, map editor, GUI.
  - `m20pro_inspection`: YOLO/RKNN wrapper node.
- The real navigation core is still C++ because it is provided by Nav2:
  - controller server
  - planner server
  - costmaps
  - map server
  - behavior server
  - velocity smoother
  - DWB / RotationShim controller plugins
- Python is acceptable for:
  - TCP protocol bridge at 5-20 Hz
  - status parsing
  - launch/config glue
  - GUI tools
  - dynamic obstacle toy simulation
  - YOLO wrapper when inference is actually done by RKNN/NPU or ONNXRuntime native backend
- Python is risky for:
  - high-rate point cloud processing
  - Python raycasting in simulation
  - large map/grid loops
  - anything expected to run at 20-50 Hz with many points

Main candidates for future C++ rewrite if performance becomes a bottleneck:

1. `pointcloud_fusion.py`
2. `dual_lidar_simulator.py` / `lidar_simulator.py` for simulation only
3. `grid_planner_node.py` only if the custom planner is used again instead of Nav2

Practical recommendation:

- Do not rewrite everything in C++ now.
- First validate real robot behavior because real deployment uses `/cloud_nav` from the robot rather than Python raycast simulation.
- Profile CPU/topic rates before rewriting:
  - `top` / `htop`
  - `ros2 topic hz /cloud_nav`
  - `ros2 topic hz /scan`
  - `ros2 topic hz /cmd_vel`
  - `ros2 topic bw /cloud_nav`
- If `/scan` generation or CPU use becomes the bottleneck, rewrite only the point cloud to scan/fusion node in C++.

## Nav2 Turning / Inching Investigation

User reported that the robot still turns by inching forward little by little.

Runtime findings:

- `/cmd_vel_nav` can command high angular velocity, up to about `1.10 rad/s`.
- `/cmd_vel` from `velocity_smoother` mostly follows `/cmd_vel_nav`.
- Therefore the main issue is not the velocity smoother or `sim_bridge`; it is the controller decision itself.
- `/scan` is stable at about `10 Hz`.
- The old `pointcloud_fusion.py` timestamp issue was already fixed with `max_source_age_s`.
- The raw Navfn global path to a test goal had 206 poses and 23 obvious heading jumps.
- `SmoothPath` reduced the same path to 1 obvious heading jump, but the previous BT was not calling the smoother.
- RotationShim had `angular_dist_threshold=0.55` but default `angular_disengage_threshold=0.785`, which is a bad hysteresis pairing and can make shim engagement unreliable.
- DWB often selected about `0.105 m/s`, meaning it was choosing the smallest positive velocity sample around turns.

Navigation changes now applied:

- Added `SmoothPath` to `m20pro_navigate_to_pose_backup_first.xml`.
- `SmoothPath` is wrapped with `ComputePathToPose` inside a `Sequence` because `RateController` accepts exactly one child.
- Added `nav2_smooth_path_action_bt_node` to the BT plugin list.
- Added explicit `smoother_server` config using `nav2_smoother::SimpleSmoother`.
- Added explicit `nav2_smoother` dependency in `m20pro_bringup/package.xml`.
- Tuned RotationShim:
  - `angular_disengage_threshold: 0.30`
  - `rotate_to_heading_angular_vel: 0.95`
  - `closed_loop: true`
- Tuned DWB to be less timid:
  - `sim_time: 0.85`
  - `min_speed_xy: 0.05`
  - `min_speed_theta: 0.18`
  - `PathAlign.scale: 14.0`
  - `PathDist.scale: 16.0`
  - `GoalAlign.scale: 6.0`
  - `GoalDist.scale: 12.0`
  - `RotateToGoal.scale: 12.0`
  - `PreferForward.scale: 4.0`

Reasoning:

- The global path should be smoothed before local following.
- DWB should not simulate too far ahead in tight right-angle turns; a long horizon makes it overly conservative.
- Reverse remains allowed in the local planner, but `PreferForward` still discourages using reverse as the first answer.

Verification:

```bash
python3 - <<'PY'
import xml.etree.ElementTree as ET
import yaml
ET.parse('src/m20pro_bringup/behavior_trees/m20pro_navigate_to_pose_backup_first.xml')
yaml.safe_load(open('src/m20pro_bringup/config/nav2_params.yaml'))
PY
colcon build --packages-select m20pro_bringup --symlink-install
ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false
```

Result:

- XML/YAML parse passed.
- `m20pro_bringup` build passed.
- Sim/Nav2 launched successfully after the BT `Sequence` fix.
- Launch logs confirmed `smoother_server` loaded `simple_smoother`.
- A smoke `NavigateToPose` goal was accepted.
- Logs confirmed `smoother_server`: `Received a path to smooth.`

## Nav2 Controller Switch To RPP

User reported that after the smoother and RotationShim tuning, a 90 degree turn still looked stiff: the robot turned about 20 degrees, moved forward, turned another 20 degrees, and repeated.

Assessment:

- This is a controller behavior issue, not a point-cloud timestamp issue.
- DWB is a sampling/scoring local planner. In right-angle turns it can produce fragmented stop-turn-go behavior because it keeps choosing short locally valid trajectories.
- RotationShim helped only partially because DWB still took over after each partial heading alignment.

Change applied:

- Switched `controller_server.FollowPath` from DWB/RotationShim to `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.
- Kept path smoothing in the BT before following.
- Set RPP to rotate toward heading when the heading error exceeds `0.30 rad`.
- Disabled ordinary reverse tracking with `allow_reversing: false`; reverse is still available through the BT `BackUp` recovery behavior.
- Reduced RPP collision prediction window to `0.4 s` after smoke testing showed it was too sensitive to a dynamic obstacle near the start pose.
- Added `nav2_regulated_pure_pursuit_controller` as a bringup runtime dependency.

RPP parameters now used:

```yaml
desired_linear_vel: 0.55
lookahead_dist: 0.85
min_lookahead_dist: 0.45
max_lookahead_dist: 1.25
use_velocity_scaled_lookahead_dist: true
use_rotate_to_heading: true
rotate_to_heading_min_angle: 0.30
rotate_to_heading_angular_vel: 1.05
max_angular_accel: 4.0
allow_reversing: false
max_allowed_time_to_collision_up_to_carrot: 0.4
```

Verification:

```bash
python3 - <<'PY'
import yaml
yaml.safe_load(open('src/m20pro_bringup/config/nav2_params.yaml'))
PY
colcon build --packages-select m20pro_bringup --symlink-install
ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false
```

Result:

- YAML parse passed.
- `m20pro_bringup` build passed.
- Launch logs confirmed: `Created controller : FollowPath of type nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.
- A smoke `NavigateToPose` goal was accepted and then canceled.

## Dynamic Obstacle Avoidance After RPP

User reported that RPP made turning much more flexible, but the robot now tends to stop in front of dynamic obstacles instead of going around them.

Assessment:

- This is expected with RPP if moving obstacles only exist in the local costmap.
- RPP is a path follower with collision checking, not a DWB-style local trajectory sampler.
- If the global path still goes through the moving obstacle, RPP will stop because its collision arc is blocked.
- To make RPP go around a dynamic obstacle, the global planner must receive that obstacle and generate a new path around it.

Change applied:

- Added `obstacle_layer` back to `global_costmap`, using `/scan`.
- Kept the obstacle persistence short so moving obstacles do not remain in the global map for too long:
  - `observation_persistence: 0.2`
  - `expected_update_rate: 0.3`
- Increased global costmap update rate:
  - `update_frequency: 5.0`
  - `publish_frequency: 2.0`
- Increased planner expectation:
  - `expected_planner_frequency: 10.0`
- Increased BT replanning rate:
  - `RateController hz="2.0"`
- Kept RPP local collision checking enabled with a short prediction window:
  - `max_allowed_time_to_collision_up_to_carrot: 0.4`

Reasoning:

- Local costmap still protects the robot at close range.
- Global costmap now receives short-lived dynamic obstacle marks, so `NavfnPlanner` can re-route around them.
- RPP then follows the newly smoothed path instead of simply stopping on the old blocked path.

Verification:

```bash
python3 - <<'PY'
import xml.etree.ElementTree as ET
import yaml
ET.parse('src/m20pro_bringup/behavior_trees/m20pro_navigate_to_pose_backup_first.xml')
yaml.safe_load(open('src/m20pro_bringup/config/nav2_params.yaml'))
PY
colcon build --packages-select m20pro_bringup --symlink-install
ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false
```

Result:

- XML/YAML parse passed.
- `m20pro_bringup` build passed.
- Launch logs confirmed `global_costmap` loaded `static_layer`, `obstacle_layer`, and `inflation_layer`.
- Launch logs confirmed RPP and smoother still load.

## Costmap Strictness Tuning

User asked what red and blue blocks in `local_costmap` mean and reported that the robot often freezes even when a route is already planned, because the route passes through red/blue costmap cells.

Explanation:

- In RViz `Color Scheme: costmap`, red/high-cost cells generally mean lethal or occupied obstacle cells. The controller should not drive through these.
- Blue/gradient cells are usually inflated cost around obstacles. They are a safety buffer, not always an absolute wall, but RPP and Nav2 cost checks become conservative when the path or robot footprint overlaps them.
- A global path drawn through a local red/blue patch means the global route and the local safety map disagree. RPP will obey the local safety map and stop.

Change applied:

- Reduced `local_costmap` inflation strictness:
  - `inflation_radius: 0.35`
  - `cost_scaling_factor: 7.0`
- Reduced local obstacle marking range and persistence:
  - `obstacle_max_range: 2.5`
  - `raytrace_max_range: 3.0`
  - `observation_persistence: 0.1`
  - `expected_update_rate: 0.3`
- Reduced `global_costmap` inflation strictness:
  - `inflation_radius: 0.35`
  - `cost_scaling_factor: 7.0`
- Reduced global dynamic obstacle persistence:
  - `observation_persistence: 0.1`
  - `expected_update_rate: 0.3`

Reasoning:

- Keep lethal obstacle safety intact.
- Shrink the blue/inflated buffer so narrow usable passages are not treated as blocked.
- Make moving obstacle marks disappear faster so the robot does not freeze on stale dynamic obstacle cost.

Verification:

```bash
python3 - <<'PY'
import yaml
yaml.safe_load(open('src/m20pro_bringup/config/nav2_params.yaml'))
PY
colcon build --packages-select m20pro_bringup --symlink-install
```

Result:

- YAML parse passed.
- `m20pro_bringup` build passed.

## RViz Robot Model Flicker / Jitter

User reported that the robot model was flickering and jittering badly.

Finding:

- This was not caused by RPP or costmap tuning.
- Multiple stale `robot_state_publisher` processes from old interrupted launches were still alive.
- `/tf` had 7 publishers, including several duplicate `robot_state_publisher` publishers.
- `/joint_states` had 6 subscribers, also indicating duplicate robot state publishers.
- RViz was receiving multiple transforms for the same robot links, which made the model flash and jump.

Cleanup performed:

- Killed stale launch/Nav2/sim processes and orphaned `robot_state_publisher` processes.
- Verified no relevant processes remained.
- Verified `ros2 node list` was empty.
- Verified `/tf` no longer existed after cleanup.

Practical note:

- If this happens again, fully stop old launches before starting a new sim.
- Killing only the parent `ros2 launch` process can leave children orphaned; if RViz warns about duplicate node names or the model flickers, check:

```bash
ps -eo pid,ppid,cmd | rg 'robot_state_publisher|m20pro_sim|nav2_|m20pro_navigation'
ros2 topic info /tf -v
ros2 node list
```

Real-robot rewrite priority if additional inspection workloads are added:

1. `pointcloud_fusion.py`: highest priority. It is on the real navigation sensor path (`/cloud_nav -> /scan`) and currently loops through every point in Python while also doing optional TF transforms and binning. Rewrite this in C++ or replace it with an installed C++ pointcloud-to-laserscan component if available.
2. `yolov8_inspection_node.py`: keep Python orchestration only if RKNN/NPU inference does the heavy work in native code. Avoid CPU ONNX inference on RK3588 during navigation. Run at a bounded rate such as 3-5 Hz unless the task requires more.
3. `tcp_bridge_node.py`: usually acceptable in Python because it runs at 5 Hz pose polling and 20 Hz command output. Rewrite only if profiling shows jitter or TCP serialization becomes unstable.
4. `zero_joint_state_publisher.py`: low priority; replace with a static/joint-state alternative only if it appears in profiling.
5. Do not spend time rewriting sim-only nodes for real deployment:
   - `dual_lidar_simulator.py`
   - `lidar_simulator.py`
   - `dynamic_obstacle_simulator.py`
   - `sim_bridge_node.py`
   - `map_editor.py`

Real robot launch should normally use `rviz:=false` and avoid running GUI/tools on the robot host. Use a laptop for RViz visualization when possible.

Check before rewriting:

```bash
htop
ros2 topic hz /cloud_nav
ros2 topic hz /scan
ros2 topic bw /cloud_nav
ros2 topic bw /scan
ros2 topic delay /scan
```

If `/scan` drops below the configured 10 Hz or `pointcloud_fusion` consumes a large CPU core share, rewrite it first.

Implementation strategy for optimization without breaking simulation:

- Do not directly replace the existing Python `pointcloud_fusion` node first.
- Add an optimized implementation as an optional alternative, ideally a separate C++ package/executable such as `m20pro_navigation_cpp/pointcloud_fusion_cpp`.
- Keep the same topics and compatible parameters:
  - input: `/cloud_nav` or front/rear lidar topics
  - output: `/scan`
  - frame: `base_link`
- Add a launch argument such as `fusion_impl:=python|cpp`.
- Default can remain `python` until the C++ node is validated.
- Simulation should continue to run because:
  - sim-only nodes (`dual_lidar_simulator`, `dynamic_obstacle_simulator`, `sim_bridge`) remain unchanged
  - the optimized node only replaces `/cloud_nav -> /scan`
  - rollback is just launching with `fusion_impl:=python`
- Safer first step before a full C++ rewrite: optimize the existing Python `pointcloud_fusion.py` with NumPy vectorization and point subsampling parameters, then add C++ only if profiling still shows a bottleneck.

## Point Cloud Timing During Turns

User suspected that the robot may be moving in small increments during turns because it waits for the point cloud to rotate/update.

Assessment:

- Nav2 should not conceptually need to "wait for the point cloud to rotate" if the scan frame and timestamps are correct.
- However, the existing `pointcloud_fusion.py` had a timing issue:
  - it converted `/cloud_nav` to ranges on cloud callback
  - then republished stored ranges on a timer
  - the output `/scan` used the current time, not the original cloud timestamp
  - if no fresh cloud arrived, old ranges could be repeatedly published with fresh timestamps
- During rotation, this can make local costmap treat slightly stale obstacle geometry as current, causing local-map lag or smear-like behavior.

Changes made:

- `pointcloud_fusion.py` now stores the output stamp for each processed cloud.
- If the input cloud is already in `base_link`, the output `/scan` uses the original cloud timestamp.
- If a latest-TF transform is used for a non-target frame, the output stamp uses current time because the points were transformed using latest TF.
- Added freshness tracking by receipt/update time.
- Added parameter:
  - `max_source_age_s: 0.25`
- `_publish_scan()` now refuses to republish stale cloud-derived ranges.
- `m20pro.yaml` sets:
  - `m20pro_pointcloud_fusion.max_source_age_s: 0.25`

Verification:

```bash
python3 -m py_compile src/m20pro_navigation/m20pro_navigation/pointcloud_fusion.py
python3 - <<'PY'
import yaml
from pathlib import Path
for p in ['src/m20pro_bringup/config/m20pro.yaml', 'src/m20pro_bringup/config/nav2_params.yaml']:
    yaml.safe_load(Path(p).read_text(encoding='utf-8'))
print('yaml ok')
PY
colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install
source install/setup.bash
ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false
timeout 5s ros2 topic hz /scan
ros2 topic echo /scan --once --field header
```

Result:

- Build passed.
- Sim launched.
- `/scan` publishes at about 10 Hz.
- `/scan.header.frame_id` is `base_link`.

## Build Hygiene

User reported a `colcon build` failure after forgetting to remove the external `dddmr_navigation` project. Current workspace check shows `src` now only contains:

- `m20pro_bringup`
- `m20pro_description`
- `m20pro_navigation`
- `m20pro_inspection`

Verification on 2026-05-12:

```bash
colcon build --symlink-install
```

Result: all 4 packages build successfully. If `dddmr_navigation` is added again for reference, keep it outside `src` or add a `COLCON_IGNORE` file inside that directory so colcon does not try to build it.

User later noticed stale generated artifacts under `build/` and `install/`, including `dddmr_*` and `cloud_msgs`. These were old build outputs, not current project source. Clean rebuild was performed:

```bash
rm -rf build install log
colcon build --symlink-install
```

Result: build succeeded. `build/` and `install/` now only contain:

- `m20pro_bringup`
- `m20pro_description`
- `m20pro_navigation`
- `m20pro_inspection`

## Reverse-Capable Local Avoidance

User observed that obstacle avoidance felt stiff: the M20 Pro is wheel-legged and can reverse, so it should sometimes back up slightly instead of always turning in place.

Change made in `src/m20pro_bringup/config/nav2_params.yaml`:

- DWB local planner now samples small reverse motion:
  - `min_vel_x: -0.25`
  - `max_vel_x: 0.75`
  - `max_speed_xy: 0.75`
- Acceleration/deceleration softened:
  - `acc_lim_x: 1.8`
  - `decel_lim_x: -1.8`
- More forward/reverse velocity samples:
  - `vx_samples: 28`
- Shorter trajectory simulation:
  - `sim_time: 1.6`
- Lower stopped threshold:
  - `trans_stopped_velocity: 0.08`
- Added `PreferForward` critic with low scale:
  - `PreferForward.scale: 2.0`
- Added a M20Pro-specific NavigateToPose behavior tree:
  - `src/m20pro_bringup/behavior_trees/m20pro_navigate_to_pose_backup_first.xml`
  - Recovery order is now `clear costmaps -> BackUp -> Spin -> Wait`.
  - BackUp recovery uses `backup_dist="0.35"` and `backup_speed="0.08"`.
- `src/m20pro_bringup/CMakeLists.txt` now installs the `behavior_trees` directory.

Intent:

- Prefer forward motion during normal tracking.
- Allow slow reverse trajectories when the front is blocked.
- Reduce the "must rotate in place" behavior around close obstacles.
- If controller recovery is needed, try a small reverse before spinning in place.
- Current practical guidance: this is a local-avoidance improvement, not a full "car-like maneuver planner". Test first with nearby dynamic obstacles. If it still rotates too much, reduce `RotateToGoal.scale`, lower `max_vel_theta`, or move from DWB to MPPI/TEB-style control later.

Verification:

```bash
python3 - <<'PY'
import yaml
with open('src/m20pro_bringup/config/nav2_params.yaml', 'r', encoding='utf-8') as f:
    yaml.safe_load(f)
print('yaml ok')
PY
python3 - <<'PY'
import xml.etree.ElementTree as ET
ET.parse('src/m20pro_bringup/behavior_trees/m20pro_navigate_to_pose_backup_first.xml')
print('xml ok')
PY
colcon build --packages-select m20pro_bringup --symlink-install
source install/setup.bash
timeout 12s ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false
```

The launch verified that `PreferForward` loads as `dwb_critics::PreferForwardCritic`, and the custom BT XML path is accepted by `bt_navigator`.

Follow-up tuning after user observed hesitation in front of periodic dynamic obstacles:

- Goal: make the robot more decisive when a good global route already exists, instead of repeatedly stopping, backing up, and replanning as a moving obstacle crosses back.
- `controller_server.failure_tolerance` increased from `0.3` to `1.2` seconds so short dynamic blockage does not immediately trigger recovery.
- DWB reverse is still available but less attractive:
  - `min_vel_x: -0.18`
  - `PreferForward.scale: 6.0`
- DWB now avoids near-zero crawling:
  - `min_speed_xy: 0.06`
  - `trans_stopped_velocity: 0.05`
- DWB is less over-constrained to the exact global path and less spin-heavy:
  - `PathAlign.scale: 12.0`
  - `PathDist.scale: 14.0`
  - `GoalAlign.scale: 10.0`
  - `GoalDist.scale: 12.0`
  - `RotateToGoal.scale: 20.0`
  - `max_vel_theta: 0.85`
- Local costmap has more context but slightly less conservative inflation:
  - `width: 5`
  - `height: 5`
  - local `inflation_radius: 0.50`
- Global costmap now uses only `static_layer` and `inflation_layer`. Transient `/scan` obstacles stay in the local costmap, so moving obstacles do not constantly perturb the global route.
- Behavior tree global replanning rate reduced:
  - `RateController hz="0.5"`
- BackUp recovery reduced slightly:
  - `backup_dist="0.30"`
  - `backup_speed="0.07"`

Verification completed:

```bash
python3 - <<'PY'
import yaml
from pathlib import Path
data = yaml.safe_load(Path('src/m20pro_bringup/config/nav2_params.yaml').read_text(encoding='utf-8'))
assert data['global_costmap']['global_costmap']['ros__parameters']['plugins'] == ['static_layer', 'inflation_layer']
print('yaml ok')
PY
python3 - <<'PY'
import xml.etree.ElementTree as ET
ET.parse('src/m20pro_bringup/behavior_trees/m20pro_navigate_to_pose_backup_first.xml')
print('xml ok')
PY
colcon build --packages-select m20pro_bringup --symlink-install
source install/setup.bash
timeout 12s ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false
```

All passed. Launch logs confirm the local costmap still loads `obstacle_layer`, while the global costmap loads only `static_layer` and `inflation_layer`.

Follow-up tuning after user observed turns were too slow:

- Increased Nav2 angular capability without returning to aggressive spin behavior:
  - `max_vel_theta: 1.10`
  - `acc_lim_theta: 3.4`
  - `decel_lim_theta: -3.4`
  - `vtheta_samples: 44`
- Strengthened heading alignment for normal path corners:
  - `PathAlign.scale: 16.0`
  - `PathAlign.forward_point_distance: 0.45`
  - `GoalAlign.scale: 12.0`
  - `GoalAlign.forward_point_distance: 0.45`
  - `RotateToGoal.scale: 24.0`
- Matched behavior server rotation limits:
  - `max_rotational_vel: 1.10`
  - `min_rotational_vel: 0.35`
  - `rotational_acc_lim: 3.4`
- Raised bridge/custom follower angular limits in `m20pro.yaml`:
  - `m20pro_tcp_bridge.max_angular_z: 1.1`
  - `m20pro_path_follower.angular_gain: 1.1`
  - `m20pro_path_follower.max_angular_z: 0.9`

Verification:

```bash
python3 - <<'PY'
import yaml
from pathlib import Path
for p in ['src/m20pro_bringup/config/nav2_params.yaml', 'src/m20pro_bringup/config/m20pro.yaml']:
    yaml.safe_load(Path(p).read_text(encoding='utf-8'))
print('yaml ok')
PY
colcon build --packages-select m20pro_bringup --symlink-install
source install/setup.bash
timeout 12s ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false
```

Result: passed. Next possible step if turning is still slow is to add explicit `velocity_smoother` limits, because it is currently using defaults from `nav2_bringup`.

Follow-up clarification: user meant the robot was not angular-speed limited, but hesitated before ordinary obstacle-free right-angle turns.

Diagnosis:

- The custom BT previously did `ComputePathToPose -> FollowPath` directly.
- That passes raw grid/Navfn paths with hard right-angle corners into DWB.
- DWB critics then fight at corners: stay on path, point to goal, avoid reverse, and satisfy obstacle/path-distance costs. This looks like hesitation before a clear turn.

Changes made:

- Added `nav2_smooth_path_action_bt_node` to `bt_navigator.plugin_lib_names`.
- Added a `smoother_server` section with `nav2_smoother::SimpleSmoother`.
- Updated `m20pro_navigate_to_pose_backup_first.xml`:
  - `ComputePathToPose` writes `{raw_path}`.
  - `SmoothPath` produces `{path}`.
  - `FollowPath` follows the smoothed `{path}`.
  - `SmoothPath` uses `check_for_collisions="true"` and `max_smoothing_duration="0.5"`.
- Adjusted DWB corner behavior:
  - `debug_trajectory_details: false`
  - `sim_time: 1.5`
  - `vtheta_samples: 40`
  - `PathAlign.forward_point_distance: 0.70`
  - `GoalAlign.forward_point_distance: 0.70`
  - `GoalAlign.scale: 6.0`
  - `PathDist.scale: 10.0`
  - `GoalDist.scale: 8.0`
  - `RotateToGoal.scale: 18.0`

Intent:

- Smooth hard map-grid corners before local control.
- Let DWB anticipate turns earlier.
- Reduce conflict between path-following and far-goal alignment.
- Reduce DWB debug overhead at corners.

Verification:

```bash
python3 - <<'PY'
import yaml
from pathlib import Path
data = yaml.safe_load(Path('src/m20pro_bringup/config/nav2_params.yaml').read_text(encoding='utf-8'))
assert 'nav2_smooth_path_action_bt_node' in data['bt_navigator']['ros__parameters']['plugin_lib_names']
assert 'smoother_server' in data
print('yaml ok')
PY
python3 - <<'PY'
import xml.etree.ElementTree as ET
ET.parse('src/m20pro_bringup/behavior_trees/m20pro_navigate_to_pose_backup_first.xml')
print('xml ok')
PY
colcon build --packages-select m20pro_bringup --symlink-install
source install/setup.bash
timeout 14s ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false
```

Result: passed. Logs show `smoother_server` configured and activated, and `bt_navigator` configured without BT plugin errors.

Follow-up rollback after user observed the robot became hesitant even on straight lines and no longer wanted to follow the global route:

Diagnosis:

- The previous SmoothPath intervention and relaxed DWB path weights made the local controller too free.
- `PathAlign/PathDist` were too low relative to the desired behavior, so DWB could choose local trajectories that did not visibly adhere to the global route.
- `forward_point_distance: 0.70` was too far for this map/robot behavior and made the controller anticipate too much.

Changes made:

- Removed `SmoothPath` from the active BT chain.
- Removed `nav2_smooth_path_action_bt_node` from `bt_navigator.plugin_lib_names`.
- Removed the explicit `smoother_server` parameter block from `nav2_params.yaml`; note that Nav2 may still start its default `smoother_server`, but the custom behavior tree no longer calls it.
- Restored direct path flow:
  - `ComputePathToPose -> FollowPath`
- Strengthened global-route adherence:
  - critic order now puts `PathAlign` and `PathDist` before `GoalAlign` and `GoalDist`
  - `PathAlign.scale: 22.0`
  - `PathDist.scale: 24.0`
  - `PathAlign.forward_point_distance: 0.35`
  - `GoalAlign.forward_point_distance: 0.35`
- Kept moderate anti-hesitation settings:
  - `debug_trajectory_details: false`
  - `sim_time: 1.35`
  - reverse still available but discouraged by `PreferForward.scale: 6.0`

Verification:

```bash
python3 - <<'PY'
import yaml
from pathlib import Path
data = yaml.safe_load(Path('src/m20pro_bringup/config/nav2_params.yaml').read_text(encoding='utf-8'))
assert 'nav2_smooth_path_action_bt_node' not in data['bt_navigator']['ros__parameters']['plugin_lib_names']
assert 'smoother_server' not in data
print('yaml ok')
PY
python3 - <<'PY'
import xml.etree.ElementTree as ET
ET.parse('src/m20pro_bringup/behavior_trees/m20pro_navigate_to_pose_backup_first.xml')
print('xml ok')
PY
colcon build --packages-select m20pro_bringup --symlink-install
source install/setup.bash
timeout 12s ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false
```

Result: passed. Launch logs show DWB loads critics in the new order: `PathAlign` and `PathDist` before `GoalAlign` and `GoalDist`.

Follow-up investigation after user reported turns were still not flexible:

Findings:

- `navigation_launch.py` remaps controller output:
  - `controller_server cmd_vel -> /cmd_vel_nav`
  - `velocity_smoother cmd_vel_nav -> /cmd_vel`
- Runtime check showed `velocity_smoother` was not the main angular bottleneck. Its default angular max was higher than the bridge limit:
  - `max_velocity: [0.5, 0.0, 2.5]`
  - `max_accel: [2.5, 0.0, 3.2]`
- `/cmd_vel` showed multiple publishers because `behavior_server` creates publishers for recovery behaviors plus `velocity_smoother`. This is normal for Nav2, but it means `/cmd_vel` publisher count alone is not a clean diagnostic.
- The real issue is likely DWB corner behavior:
  - DWB was directly responsible for both turning and path tracking at hard right-angle grid corners.
  - `min_speed_xy: 0.06` prevented true stop-and-turn decisions.
  - High path adherence made DWB reluctant to make a decisive heading correction.

Changes made:

- Wrapped DWB with `nav2_rotation_shim_controller::RotationShimController`.
- `FollowPath` now uses:
  - `plugin: nav2_rotation_shim_controller::RotationShimController`
  - `primary_controller: dwb_core::DWBLocalPlanner`
  - `angular_dist_threshold: 0.55`
  - `forward_sampling_distance: 0.65`
  - `rotate_to_heading_angular_vel: 0.85`
  - `max_angular_accel: 3.4`
  - `simulate_ahead_time: 1.0`
  - `rotate_to_goal_heading: false`
  - `closed_loop: false`
- DWB remains the inner controller for normal tracking and obstacle handling.
- `min_speed_xy` changed back to `0.0` so the robot can make an explicit low-speed/pivot heading correction when needed.
- DWB path adherence was kept moderate:
  - `PathAlign.scale: 20.0`
  - `PathDist.scale: 22.0`
  - `PathAlign.forward_point_distance: 0.45`
  - `GoalAlign.forward_point_distance: 0.45`
- Added explicit `velocity_smoother` config aligned to M20 limits:
  - `max_velocity: [0.80, 0.0, 1.10]`
  - `min_velocity: [-0.18, 0.0, -1.10]`
  - `max_accel: [2.2, 0.0, 3.4]`
  - `max_decel: [-2.2, 0.0, -3.4]`
- Added explicit bringup dependencies:
  - `nav2_dwb_controller`
  - `nav2_rotation_shim_controller`
  - `nav2_velocity_smoother`

Verification:

```bash
python3 - <<'PY'
import yaml
from pathlib import Path
data = yaml.safe_load(Path('src/m20pro_bringup/config/nav2_params.yaml').read_text(encoding='utf-8'))
fp = data['controller_server']['ros__parameters']['FollowPath']
assert fp['plugin'] == 'nav2_rotation_shim_controller::RotationShimController'
assert fp['primary_controller'] == 'dwb_core::DWBLocalPlanner'
assert fp['min_speed_xy'] == 0.0
print('yaml ok')
PY
colcon build --packages-select m20pro_bringup --symlink-install
source install/setup.bash
timeout 14s ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false
```

Result: passed. Launch logs confirm:

- `Created controller : FollowPath of type nav2_rotation_shim_controller::RotationShimController`
- `Created internal controller for rotation shimming: FollowPath of type dwb_core::DWBLocalPlanner`

## VLA-Lite Direction

User asked whether M20 Pro can support a simple VLA-style workflow: give a natural-language command like "go to somewhere", let the robot infer the place, and navigate there.

Current recommendation:

- Feasible, but implement it as an engineering "VLA-lite" stack first, not as end-to-end vision-language-action control.
- Preferred chain:

```text
speech/text command
  -> intent and place parser
  -> semantic place registry / waypoint database
  -> resolve target floor + waypoint pose
  -> Nav2 goal
  -> existing 106 localization + 104 planning + 103 motion chain
```

- Example commands:
  - "去一楼配电箱"
  - "去二楼楼梯口"
  - "去安全帽检测点巡检"
- Compute requirement is moderate if the LLM/VLM only parses commands or matches a place name. It does not need to run continuous robot control.
- RK3588 can handle small local models or rule/template parsing, but larger language or vision-language models should run on a laptop/server/cloud if needed.
- Do not let a large model directly publish `/cmd_vel`. It should only output structured goals such as `{floor, place_id, task_type}`. Navigation safety remains with Nav2, costmaps, and the robot safety stack.
- Required future components:
  - `places.yaml` or database: floor, place id, aliases, pose, task type.
  - `command_interpreter` node: text to structured goal.
  - `patrol_manager` node: sends Nav2 goals and triggers inspection after arrival.
  - optional VLM grounding for "the red door / the elevator entrance" only after the deterministic waypoint flow works.

## Closer VLA Scenario

User clarified a more VLA-like command: "去三楼看看有没有人在抽烟".

Interpretation:

- This is not just a waypoint command. It contains:
  - target floor: third floor
  - search/exploration scope: inspection viewpoints or patrol route on that floor
  - visual task: detect whether a person is smoking
  - report requirement: return evidence and result

Recommended architecture is a layered VLA system, not an end-to-end model that controls velocity:

```text
natural language command
  -> task planner / LLM creates structured plan
  -> floor_manager handles floor transition and map switching
  -> patrol_manager executes deterministic navigation skills
  -> perception skills run YOLO/VLM/action recognition
  -> report generator summarizes result with images/time/place
```

Example structured plan:

```json
{
  "task": "inspect_smoking",
  "floor": "F3",
  "route": "default_inspection_route",
  "skills": ["go_to_floor", "patrol_viewpoints", "look_around", "detect_smoking", "report"]
}
```

Compute recommendation:

- RK3588 can run navigation, RTSP decode if optimized, and YOLOv8 RKNN for person/cigarette/smoke-like detections.
- Reliable "smoking" recognition is harder than generic object detection because cigarettes are small and may be occluded. A robust solution likely needs person detection + cigarette/smoke detection + temporal action or VLM verification.
- Larger VLM/LLM reasoning should run on a laptop/server/cloud, while RK3588 runs real-time local perception and robot control.

Safety boundary:

- VLA/LLM/VLM should output goals, skills, and semantic decisions.
- It must not directly publish `/cmd_vel`.
- Navigation remains handled by 106 localization + 104 Nav2 + 103 motion command bridge.

## Strong VLA vs ROS 2 / Direct Control

User asked whether the strongest VLA still needs ROS 2, and whether a large model can directly control robot actions.

Current conclusion:

- The strongest VLA systems can output robot actions in research settings, but they still need a robot runtime underneath: device drivers, state estimation, safety watchdogs, low-level controllers, transforms, time sync, logging, and emergency stop.
- ROS 2 is not theoretically mandatory, but some middleware/control framework is practically mandatory. For this project, ROS 2 remains the most useful integration layer because M20Pro work already uses ROS 2 topics, Nav2, TF, RViz, point cloud processing, and custom nodes.
- Direct model-to-motor control is not recommended for M20Pro navigation/inspection:
  - M20Pro motion must still pass through 103 AOS or official DDS/TCP interfaces.
  - 106 localization and maps are already the reliable coordinate authority.
  - Construction-site locomotion has safety-critical dynamic obstacles and fall/collision risks.
  - End-to-end VLA would require M20Pro-specific demonstration data, action-space definition, simulation/training, safety validation, and fallback controllers.
- Better high-end architecture:

```text
VLA/LLM/VLM
  -> chooses skill and parameters
  -> skill runtime calls ROS 2/Nav2/perception services
  -> safety monitor validates commands
  -> tcp_bridge sends bounded motion command to 103 AOS
```

- For "go to third floor and check smoking", the VLA should plan and choose inspection behaviors, while deterministic skills perform navigation, map switching, look-around motions, detection, and reporting.
- RK3588 can run local perception and smaller models, but frontier VLA/VLM inference should usually run on an external GPU/server/cloud, with RK3588 handling real-time ROS 2 integration and safety.
- User summarized this as the mainstream "skills" approach. Confirmed:
  - For this project, use VLA/LLM as a high-level skill selector/planner.
  - Implement deterministic ROS 2 skills for navigation, floor switching, relocalization, look-around, YOLO inspection, photo capture, report generation, and emergency stop.
  - This is closer to deployable robotics than end-to-end direct action control on M20Pro.

## VLA With Enough NVIDIA Compute

User asked what VLA could achieve if the M20 Pro later adds enough NVIDIA compute.

Current conclusion:

- Extra GPU mainly enables stronger onboard VLM/VLA reasoning and heavier perception. It does not remove the need for maps, localization, safety, and ROS 2 skill execution.
- Practical capability tiers:
  1. Semantic command execution: parse "去三楼看看有没有人在抽烟" into floor, route, inspection task, and report action.
  2. Active semantic patrol: choose viewpoints, turn the camera/body, revisit uncertain areas, and ask for clarification if the command is ambiguous.
  3. Open-vocabulary inspection: combine YOLO/RKNN with VLM verification for objects/events not covered by the detector.
  4. Semantic mapping: attach labels like "配电箱", "楼梯口", "吸烟疑似区域", "人员聚集" to map locations.
  5. Limited learned local skills: approach a visually grounded target, center camera on an object, or perform visual servoing under speed/safety limits.
- Still not recommended for direct full-body/end-to-end locomotion control on M20Pro unless there is a large M20Pro-specific dataset, simulation pipeline, safety validation, and a deterministic fallback controller.
- Best high-compute architecture remains:

```text
onboard/external NVIDIA GPU
  -> VLM/VLA planner and visual verifier
  -> ROS 2 skill API
  -> Nav2 / map switching / relocalization / YOLO / report skills
  -> safety monitor
  -> tcp_bridge
  -> 103 AOS
```

- For construction-site inspection, the strongest value is not "model directly walks"; it is "model understands vague tasks, chooses where to inspect, interprets visual evidence, and creates reports", while the robot runtime executes safely.

## Ongoing Maintenance Rule

After future user questions or code changes, update this file with:

- New decisions or corrected assumptions.
- Files changed.
- Commands run and verification results.
- Current blockers or next steps.

## Navigation Stutter / Dynamic Obstacle Jitter Fix

User reported that the robot could get stuck with a valid path, then later that dynamic obstacles and the whole RViz scene were jittering.

Findings:

- A previous stuck-navigation failure was reproduced with a goal from `(0,0)` to `(8,1)`.
- The main failure was not global planning. The BT `SmoothPath` node aborted repeatedly with `Smoothed path leads to a collision`, which canceled `FollowPath` and triggered backup/spin/wait recovery loops.
- After changing the BT smoother collision check, the same goal reached successfully in about 24 seconds with `0` recoveries.
- The later "everything jittering" symptom was most likely caused by duplicate/running test launch processes interfering with the user's session. A clean process check after stopping test launches showed no remaining `robot_state_publisher`, Nav2, `m20pro_navigation`, or `rviz2` processes.
- Clean sim diagnostics after rebuild:
  - `/tf` publisher count was `2`, expected: `robot_state_publisher` plus `m20pro_tcp_bridge`.
  - `/dynamic_obstacles` was stable at `10 Hz`.
  - `/dynamic_obstacle_markers` was stable at `5 Hz`.
  - `/scan` was stable at `10 Hz`.

Files changed:

- `src/m20pro_bringup/behavior_trees/m20pro_navigate_to_pose_backup_first.xml`
  - Replan rate changed from `2.0 Hz` to `1.0 Hz`.
  - `SmoothPath` collision checking disabled so transient dynamic obstacles do not abort the whole navigation pipeline.
  - Smoothing duration reduced to `0.3 s`.
- `src/m20pro_bringup/config/nav2_params.yaml`
  - Progress checker tightened to `required_movement_radius: 0.25` and `movement_time_allowance: 6.0`.
  - Simple smoother reduced to `max_its: 200`, `do_refinement: false`.
  - Costmap `observation_persistence` set to `0.3`.
- `src/m20pro_navigation/m20pro_navigation/dynamic_obstacle_simulator.py`
  - Added separate marker publish throttling and marker lifetime to reduce RViz flicker.
  - Marker visualization is now separate from the obstacle pose stream used by the simulated LiDAR.
- `src/m20pro_bringup/config/m20pro.yaml`
  - Dynamic obstacle pose publishing set to `10 Hz`.
  - Marker visualization set to `5 Hz` with `0.35 s` lifetime.

Commands run:

- `colcon build --symlink-install --packages-select m20pro_navigation m20pro_bringup`
- `ros2 launch m20pro_bringup m20pro_sim.launch.py rviz:=false`
- `ros2 topic info /tf -v`
- `ros2 topic hz /dynamic_obstacles`
- `ros2 topic hz /dynamic_obstacle_markers`
- `ros2 topic hz /scan`

Operational note:

- Only one sim launch should be running at a time. If the model, markers, costmaps, or robot all flicker together, first check duplicate processes:

```bash
ps -eo pid,ppid,cmd | rg 'm20pro_sim|robot_state_publisher|nav2_|m20pro_navigation|rviz2'
ros2 topic info /tf -v
```

## GitHub Upload Checkpoint

User confirmed the current simulation/navigation behavior is good and requested uploading this version to GitHub.

Current checkpoint contents:

- Stable `m20pro_sim.launch.py` flow with Nav2, simulated dual LiDAR, pointcloud fusion, dynamic obstacles, RViz config, and map server.
- RPP-based local controller with smoother-backed global path handling.
- Dynamic obstacle visualization throttling to reduce RViz flicker.
- YOLO/RKNN inspection scaffold under `src/m20pro_inspection`.
- README, `.gitignore`, and vendor attribution updates.
- `codex.md` maintained for future review.

Repository hygiene before upload:

- `.gitignore` excludes `build/`, `install/`, `log/`, model weights, PDF manuals, local env/secrets, and local third-party reference workspace `dddmr_navigation/`.
- No generated colcon outputs should be committed.
- Stable navigation checkpoint committed as `4c49487 Stabilize M20Pro simulation navigation` and pushed to `origin/main`.

## Behavior Tree Explanation

User asked what `m20pro_navigate_to_pose_backup_first.xml` does.

Explanation captured:

- This file is the Nav2 NavigateToPose behavior tree selected by `default_nav_to_pose_bt_xml` in `nav2_params.yaml`.
- It defines the runtime navigation logic after a target pose is received:
  - plan a global path with `ComputePathToPose`;
  - smooth the path with `SmoothPath`;
  - follow it with `FollowPath`;
  - if failures happen, run recovery actions.
- The tree replans at `1.0 Hz` through `RateController`, which is intentionally calmer than the earlier more aggressive replanning.
- `SmoothPath` has `check_for_collisions="false"` so transient dynamic obstacles do not abort the whole navigation pipeline. Collision safety is still handled by the controller and costmaps during path following.
- Recovery order is adapted for a wheel-legged robot:
  1. clear local/global costmaps;
  2. back up `0.30 m`;
  3. spin `1.20 rad`;
  4. wait `3 s`.
- This is why the robot now avoids the old behavior of repeatedly aborting on smoothed-path dynamic obstacle collisions.

## Multi-Floor Map Switching Plan

User wants autonomous inspection across floors. The robot must navigate to a stair entrance, switch to stair gait, climb, switch to the target floor map, relocalize, return to flat/agile gait, and continue inspection.

Decision:

- Do not put multi-floor logic inside the Nav2 behavior tree. The behavior tree should remain responsible for one-floor `NavigateToPose` behavior only.
- Add a higher-level `floor_manager` / `mission_manager` node that owns floor transitions, gait switching, map loading, relocalization, and inspection task sequencing.
- Treat stairs as special graph edges between floor maps, not as ordinary 2D traversable space in an occupancy grid.

Manual-backed control points:

- Axis command is already bridged as Type `2`, Command `21`, sent at about `20 Hz`.
- Gait switching should be exposed as a ROS service in `tcp_bridge`: Type `2`, Command `23`, Items `{"GaitParam": value}`.
- Stair gait is `GaitParam=14` in standard motion mode.
- Flat/agile navigation gait is `Gait=12` in navigation-task context; confirm whether direct gait switching to `12` is accepted on the real robot, otherwise switch motion mode to agile first.
- Relocalization should be exposed through `/initialpose` or a ROS service in `tcp_bridge`: Type `2101`, Command `1`, Items `{"PosX", "PosY", "PosZ", "Yaw"}`.
- 106/NOS map packages live under `/var/opt/robot/data/maps`, and `/var/opt/robot/data/maps/active` points to the active package. The software manual says `drmap unpack` activates a map package and requires restart, so true online 106 map switching must be verified before relying on it.

Recommended state machine:

```text
IDLE
 -> NAV_TO_STAIR_ENTRY on current floor
 -> PREPARE_STAIR
    - cancel Nav2 goal
    - publish zero cmd_vel
    - clear local/global costmaps
    - switch stair gait: GaitParam=14
 -> CLIMB_STAIR
    - first version: guarded/supervised stair skill
    - later version: send bounded axis commands and monitor timeout/status/pitch/pose
 -> ARRIVE_TARGET_FLOOR_ANCHOR
    - stop motion
    - switch/load target floor map on 104 map_server
    - switch/activate corresponding map on 106 if an online mechanism is confirmed
    - relocalize at the known landing anchor with Type=2101 Command=1
    - wait for `/m20pro_tcp_bridge/localization_ok`
    - clear Nav2 costmaps
 -> SWITCH_FLAT_GAIT
 -> NAV_TO_TARGET on target floor
 -> INSPECT / REPORT
```

Recommended config shape:

```yaml
floors:
  F1:
    map_yaml: ".../maps/F1/occ_grid.yaml"
    stairs:
      stair_A_up:
        entry_pose: {x: 1.2, y: 3.4, yaw: 1.57}
        target_floor: F2
        target_anchor: stair_A_down
  F2:
    map_yaml: ".../maps/F2/occ_grid.yaml"
    anchors:
      stair_A_down:
        relocalization_pose: {x: 0.8, y: -2.1, yaw: 1.57}
        resume_pose: {x: 1.0, y: -1.7, yaw: 0.0}
gaits:
  stair: 14
  flat_nav: 12
```

Implementation tasks:

1. Add `tcp_bridge` services:
   - `/m20pro/set_gait`
   - `/m20pro/relocalize`
   - optional `/m20pro/set_mode`
   - optional `/m20pro/cancel_native_nav`
2. Add `floor_manager`:
   - Nav2 `NavigateToPose` action client
   - `nav2_msgs/srv/LoadMap` client for 104 map_server
   - costmap clear clients
   - gait/relocalization service clients
   - floor graph loaded from `floors.yaml`
3. Validate in this order:
   - single-floor real navigation
   - manual stair gait switch while stationary
   - supervised stair climb with emergency stop ready
   - landing relocalization on the new floor
   - only then combine into autonomous floor transition.

## Workspace Relocation Note

User asked whether moving the workspace out of `~/桌面` would require many path edits.

Current scan result:

- Source and launch files mostly use ROS package lookup such as `get_package_share_directory`, so moving the workspace should not require broad source edits.
- Hardcoded local paths found mainly in documentation examples:
  - `README.md` line with `cd /home/fabu/桌面/M20Pro/m20pro_ros2_ws`
- Generated colcon directories can contain absolute paths:
  - `build/`
  - `install/`
  - `log/`

Recommended move procedure:

```bash
cd /new/parent
mv /home/fabu/桌面/M20Pro/m20pro_ros2_ws .
cd m20pro_ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

After moving, update only shell aliases, terminal bookmarks, IDE workspace settings, and README command examples if needed.

## README GitHub Cleanup

User wanted the public README to avoid local machine details before publishing to GitHub.

Changes made:

- Removed the local absolute path example:
  - `/home/fabu/桌面/M20Pro/m20pro_ros2_ws`
- Removed the old `M20Pro_ws` migration-history section.
- Changed wording from "personal integration project" to "unofficial ROS 2 integration project".
- Changed compile instructions to say "run from workspace root" instead of a local directory.
- Changed `~/m20pro_active_map` and `~/m20pro_models/...` references to `$HOME/...` style.
- Reworded the control GUI section so it describes the current package instead of a local old script.

Verification:

```bash
rg -n "个人|/home/|~|M20Pro_ws|桌面|fabu" README.md
```

No matches remain.

## ROS Bag Recording Notes

User asked how to record ROS bag data for this project.

Recommended default is to record a lightweight navigation debug bag instead of `ros2 bag record -a`, because point clouds and annotated images can grow quickly.

Lightweight navigation topics:

```bash
mkdir -p bags
ros2 bag record -o bags/nav_debug_$(date +%Y%m%d_%H%M%S) \
  /tf /tf_static \
  /map /odom \
  /m20pro_tcp_bridge/map_pose \
  /m20pro_tcp_bridge/localization_ok \
  /m20pro_tcp_bridge/obstacle_active \
  /m20pro_tcp_bridge/navigation_status \
  /scan /cmd_vel /goal_pose /plan
```

Simulation adds:

```bash
  /dynamic_obstacles /dynamic_obstacle_active /dynamic_obstacle_markers
```

Sensor-heavy debugging may add `/cloud_nav`, but avoid recording it by default. YOLO inspection may add:

```bash
/m20pro_yolov8_inspection/detections
/m20pro_yolov8_inspection/events
```

Avoid `/m20pro_yolov8_inspection/annotated_image` unless image playback is explicitly needed.

Useful commands:

```bash
ros2 bag info bags/<bag_name>
ros2 bag play bags/<bag_name>
ros2 bag record -a -x "/m20pro_yolov8_inspection/annotated_image|/cloud_nav"
```

On Humble, `ros2 bag record` supports `--compression-mode file --compression-format zstd`, `-b/--max-bag-size`, and `-d/--max-bag-duration`.

Clarification:

- To record this project's behavior, the project must be running on the robot side or on 104/GOS connected to the real robot. Otherwise topics such as `/m20pro_tcp_bridge/map_pose`, fused `/scan`, Nav2 `/plan`, local costmaps, and project-generated `/cmd_vel` will not exist.
- Recording while the robot uses the factory navigation only captures factory/official behavior and visible official sensor topics. It is useful for learning real sensor data and official pose/map behavior, but it cannot debug this project's Nav2 planner/controller behavior.
- Safe first real-robot step should be a "shadow/record-only" mode: run perception, pose bridge, map copy, pointcloud fusion, and Nav2 planning on 104, but do not send `/cmd_vel` axis commands to 103. Current `tcp_bridge` lacks a clean `enable_axis_command:=false` switch, so add that before shadow-mode bagging if needed.

Detailed rosbag workflow:

1. Before bagging:
   - Source the correct ROS environment.
   - Start the target launch.
   - Verify important topics with `ros2 topic list`, `ros2 topic hz`, and `ros2 node list`.
2. Real robot first pass:
   - use `enable_axis_command:=false`.
   - record map/pose/scan/plan/cmd_vel without sending commands to 103.
3. Real robot control pass:
   - use `enable_axis_command:=true` only in a safe open area.
   - record the same lightweight navigation topics, optionally adding `/cloud_nav`.
4. Playback safety:
   - never play a bag containing `/cmd_vel` while `tcp_bridge` is connected with axis command enabled.
   - replay on a laptop/RViz-only environment, or remap `/cmd_vel:=/cmd_vel_replay`.
5. Use diagnostics and bags together:
   - `tools/collect_ros_snapshot.sh` captures system structure.
   - rosbag captures runtime behavior.

## First 104 Deployment Flow

User asked what to do after packaging `src/`, placing it on 104, and building.

Recommended sequence:

1. Create a real ROS 2 workspace on 104:
   - `mkdir -p ~/m20pro_ros2_ws/src`
   - copy repository `src/` into that workspace
   - source `/opt/ros/foxy/setup.bash`
   - run `rosdep install --from-paths src -y --ignore-src`
   - run `colcon build --symlink-install`
2. Copy the 106 active map package or at least `occ_grid.yaml`/`occ_grid.pgm` to 104, because 104 planning map must match 106 localization map.
3. First launch in shadow mode:
   - run `m20pro_real.launch.py` with `enable_axis_command:=false`
   - this allows pose polling, TF, map server, pointcloud fusion, Nav2 planning, RViz, and bag recording without sending axis commands to 103.
4. Verify:
   - `/m20pro_tcp_bridge/map_pose`
   - `/m20pro_tcp_bridge/localization_ok`
   - `/cloud_nav`
   - `/scan`
   - `/tf`
   - `/plan`
5. Record a lightweight navigation bag.
6. Only after map/pose/scan/planning are correct, launch with `enable_axis_command:=true` in a safe open area and send a short nearby goal.

Code changes made for this flow:

- `tcp_bridge_node.py`
  - Added `enable_axis_command` parameter.
  - When false, the node still polls status/pose but does not create the timer that sends Type `2`, Command `21` axis commands.
- `m20pro_real.launch.py`
  - Added launch argument `enable_axis_command`, default `false` for safe shadow mode.
  - Fixed default real map path to existing `working_1-20260429-162852_edited3/occ_grid.yaml`.
- `m20pro.yaml`
  - Added `enable_axis_command: true` as the bridge config default, while real launch overrides it unless explicitly enabled.

Verification:

```bash
python3 -m py_compile src/m20pro_navigation/m20pro_navigation/tcp_bridge_node.py src/m20pro_bringup/launch/m20pro_real.launch.py
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select m20pro_navigation m20pro_bringup
```

Both passed.

## 104 Nav2 Availability / Offline Install

User noted the robot has no internet and suspected 104/GOS may not have Navigation2 installed.

Assessment:

- `m20pro_real.launch.py` depends on Nav2 packages such as `nav2_bringup`, `nav2_map_server`, and the lifecycle manager.
- If 104 lacks Nav2, the custom packages can build, but real/sim Nav2 launches will fail.
- First verify on 104:

```bash
source /opt/ros/foxy/setup.bash
ros2 pkg prefix nav2_bringup
ros2 pkg prefix nav2_controller
dpkg -l | grep -E 'ros-foxy-(navigation2|nav2-bringup|nav2-controller|nav2-map-server)'
```

Recommended install paths:

1. Best practical path:
   - temporarily give 104 internet through a separate interface, such as phone USB tethering, Wi-Fi, or USB Ethernet;
   - do not change the internal `10.21.31.x` interface;
   - install:

```bash
sudo apt update
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup
```

2. If 104 must stay fully offline:
   - use `apt-offline` from 104 to create an install signature;
   - use an internet machine to download a bundle;
   - copy the bundle back to 104 and install it.

Example:

```bash
# on 104
sudo apt-offline set nav2.sig --update --install-packages ros-foxy-navigation2 ros-foxy-nav2-bringup

# on internet-connected machine
apt-offline get --bundle nav2_bundle.zip nav2.sig

# back on 104
sudo apt-offline install nav2_bundle.zip
```

3. Source-building Nav2 on 104 is possible but not recommended for this project stage because dependencies and build time are substantial.

Important:

- 104 is likely Ubuntu 20.04 / ROS 2 Foxy / arm64, so package architecture must match the robot.
- Do not use x86_64 `.deb` packages from a desktop on the arm64 robot.
- For project momentum, keep the internal robot network static and use a separate network path for internet access.

## Foxy/Humble Compatibility Split

User emphasized that the M20 Pro robot is ROS 2 Foxy while the current development PC is Humble, and asked to be careful about compatibility.

Finding:

- The previous `nav2_params.yaml` is suitable for the Humble simulation side, but it contains Nav2 features/configuration that may not be available or compatible on Foxy:
  - `nav2_regulated_pure_pursuit_controller`
  - `nav2_smoother` / `SmoothPath`
  - `behavior_server` / `nav2_behaviors`
  - `velocity_smoother`
  - Humble-style costmap parameters such as `obstacle_max_range` / `raytrace_max_range`
  - Humble-style AMCL `robot_model_type`
- Foxy Nav2 uses older names/config patterns such as:
  - `recoveries_server`
  - `nav2_recoveries/*`
  - `default_bt_xml_filename`
  - singular `goal_checker_plugin`
  - costmap `obstacle_range` / `raytrace_range`

Changes made:

- Added Foxy-specific BT:
  - `src/m20pro_bringup/behavior_trees/m20pro_navigate_to_pose_foxy.xml`
- Added Foxy-specific Nav2 params:
  - `src/m20pro_bringup/config/nav2_params_foxy.yaml`
- Updated `m20pro_real.launch.py`:
  - real robot default now uses `nav2_params_foxy.yaml`;
  - added `nav2_params_file` launch argument for overrides;
  - kept `enable_axis_command:=false` default for shadow mode.
- Updated `m20pro_sim.launch.py`:
  - sim keeps using the Humble-oriented `nav2_params.yaml`;
  - added `nav2_params_file` launch argument.
- Updated `m20pro_bringup/package.xml`:
  - Foxy uses `nav2_recoveries`;
  - non-Foxy/Humble uses `nav2_behaviors`, RPP, rotation shim, smoother, and velocity smoother dependencies.

Verification:

```bash
python3 -m py_compile src/m20pro_bringup/launch/m20pro_real.launch.py src/m20pro_bringup/launch/m20pro_sim.launch.py
python3 - <<'PY'
import yaml
from pathlib import Path
for p in ['src/m20pro_bringup/config/nav2_params.yaml','src/m20pro_bringup/config/nav2_params_foxy.yaml','src/m20pro_bringup/config/inspection_waypoints.yaml']:
    yaml.safe_load(Path(p).read_text())
    print('ok', p)
PY
python3 - <<'PY'
import xml.etree.ElementTree as ET
for p in ['src/m20pro_bringup/behavior_trees/m20pro_navigate_to_pose_backup_first.xml','src/m20pro_bringup/behavior_trees/m20pro_navigate_to_pose_foxy.xml','src/m20pro_bringup/package.xml']:
    ET.parse(p)
    print('ok', p)
PY
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select m20pro_bringup m20pro_navigation
```

All checks passed locally on Humble. Foxy runtime still needs validation on 104 after Nav2 packages are confirmed/installed.

Operational rule:

- On the Humble PC/simulation:

```bash
ros2 launch m20pro_bringup m20pro_sim.launch.py
```

- On the Foxy robot/104:

```bash
ros2 launch m20pro_bringup m20pro_real.launch.py \
  map:=$HOME/m20pro_active_map/occ_grid.yaml \
  enable_axis_command:=false
```

- Do not run Humble Nav2 params on Foxy unless explicitly testing and prepared to fix plugin/parameter errors.

## Fisheye / Low-Resolution Camera Strategy

User noted that M20 Pro camera images look fisheye-distorted and low-resolution, and asked whether this should be handled in YOLO training or in this ROS 2 workspace.

Current code state:

- `m20pro_inspection` reads RTSP or image topic frames and sends the raw frame directly into `_preprocess()`.
- There is currently no camera calibration or undistortion step in `yolov8_inspection_node.py`.

Decision:

- Do not treat this as training-vs-ROS2 either/or.
- For detection robustness, train/validate on images with the same distortion and resolution as deployment.
- For geometry, measurement, human-friendly visualization, or cross-camera consistency, add optional runtime undistortion later.

Recommended approach:

1. Short term:
   - collect real M20 Pro raw camera frames;
   - label/train YOLO on raw fisheye images;
   - include edge-of-image samples, blur, low-light, motion blur, compression artifacts, and small-object examples;
   - keep runtime inference on raw images so train/inference distribution matches.
2. Middle term:
   - calibrate front/rear cameras with checkerboard/AprilTag board;
   - add optional `enable_undistort` and `calibration_yaml` params in the inspection node;
   - if enabled, undistort both training images and runtime frames, not just one side.
3. Avoid relying on upscaling/super-resolution to "fix" low pixel count. Better options:
   - choose closer inspection waypoints;
   - face the camera toward the target with correct yaw;
   - use ROI/crops if the target area is known;
   - improve lighting/exposure;
   - use a model/input size that matches actual object pixel size.

Important:

- Undistortion can crop field of view and adds interpolation blur/CPU cost.
- If raw images are very distorted but consistent, YOLO can learn them well as long as training data matches deployment.
- If runtime undistortion is added, use OpenCV precomputed maps (`initUndistortRectifyMap` + `remap`) for efficiency on RK3588.

## M20 Pro Camera Input in `m20pro_inspection`

User asked whether `m20pro_inspection` already subscribes to the robot camera topic.

Current implementation:

- Default mode is not ROS topic subscription.
- `m20pro_inspection` defaults to direct RTSP input:
  - front wide: `rtsp://10.21.31.103:8554/video1`
  - rear wide: `rtsp://10.21.31.103:8554/video2`
- The node uses OpenCV `cv2.VideoCapture(rtsp_url)` when `source_type: rtsp`.
- It can subscribe to a ROS image topic only when launched with:

```bash
source_type:=image_topic image_topic:=/camera/image_raw
```

- The current robot `ros2 topic list` shared by the user did not show an obvious camera image topic, so RTSP is probably the right default for real M20 Pro deployment.

Published outputs:

```text
/m20pro_yolov8_inspection/detections
/m20pro_yolov8_inspection/events
/m20pro_yolov8_inspection/annotated_image
```

Test command on 104:

```bash
ros2 launch m20pro_inspection m20pro_inspection.launch.py \
  backend:=dry_run \
  source_type:=rtsp \
  rtsp_url:=rtsp://10.21.31.103:8554/video1
```

## Laptop as SSH Jump Host

User has two Ubuntu computers: a desktop running Codex and a laptop connected to the robot. User wants the desktop to reach the robot through the laptop.

Topology:

```text
desktop/Codex -> laptop -> robot 104/106
```

Laptop responsibilities:

- Stay connected to the robot network.
- Verify robot reachability:

```bash
ping -c 3 10.21.31.104
ping -c 3 10.21.31.106
ssh user@10.21.31.104
```

- Run an SSH server so the desktop can connect to it:

```bash
sudo apt install openssh-server
sudo systemctl enable --now ssh
hostname -I
```

Desktop responsibilities:

- Verify desktop can SSH to the laptop:

```bash
ssh <laptop_user>@<laptop_ip>
```

- Use laptop as jump host:

```bash
ssh -J <laptop_user>@<laptop_ip> user@10.21.31.104
ssh -J <laptop_user>@<laptop_ip> user@10.21.31.106
```

- Copy files through the jump host:

```bash
scp -J <laptop_user>@<laptop_ip> tools/collect_ros_snapshot.sh user@10.21.31.104:~/
scp -J <laptop_user>@<laptop_ip> user@10.21.31.104:~/m20pro_ros_snapshot_*.tar.gz ./robot_snapshots/
```

Alternative if ProxyJump fails:

```bash
ssh -L 2222:10.21.31.104:22 <laptop_user>@<laptop_ip>
ssh -p 2222 user@127.0.0.1
```

Safety:

- Keep robot internal IPs unchanged.
- Laptop can use one interface for robot network and another for internet/Wi-Fi.
- Desktop only needs internet plus SSH access to the laptop.

## 104 Internet via Phone USB Tethering

User tried the M20 Pro Type-C port with phone USB tethering, but it did not work.

Troubleshooting decision:

- Do not modify the internal `10.21.31.x` robot network while testing internet.
- First determine whether 104/GOS sees the phone as a USB network device.
- The Type-C port may not be connected to 104 as a USB host; it may be for power, debug, another host, or a device-mode function.

Checks on 104 before and after plugging the phone:

```bash
ip -br link
ip -br addr
nmcli dev status
lsusb
dmesg -T | tail -80
```

If a new interface appears, such as `usb0`, `enx...`, `rndis0`, or `enp...`, use DHCP on only that interface:

```bash
sudo nmcli dev set <iface> managed yes
sudo nmcli con add type ethernet ifname <iface> con-name phone-usb ipv4.method auto ipv6.method ignore
sudo nmcli con up phone-usb
ip route
ping -c 3 8.8.8.8
ping -c 3 mirrors.tuna.tsinghua.edu.cn
```

If IP ping works but domain ping fails, it is DNS only; set DNS on the phone-usb connection.

If no USB device/interface appears:

- Try a known data-capable cable.
- On Android, enable "USB tethering" after plugging in.
- Try a USB-A/Type-C hub or Ethernet dongle if the port supports host mode.
- Prefer a separate USB Ethernet/Wi-Fi dongle or laptop NAT if Type-C tethering is not exposed to 104.

## M12-to-RJ45 Direct Robot Network

User said the company purchased a 4-pin M12-to-RJ45 cable and asked whether it can connect a computer to the robot while preserving computer internet access.

Answer:

- Yes, if the cable is the correct vendor-confirmed M12 Ethernet cable/pinout for the M20 Pro port.
- Many 4-pin M12 Ethernet connectors are D-coded 100M Ethernet, but not every 4-pin M12 connector is Ethernet. Do not plug into unknown power/sensor M12 ports.
- To keep internet and robot SSH simultaneously:
  - use Wi-Fi or another Ethernet adapter for internet;
  - use the M12/RJ45 Ethernet interface only for the robot internal subnet.
- Configure the computer robot-side NIC with a static address and no default gateway/DNS:

```text
10.21.31.200/24
gateway: empty
DNS: empty
never-default: true
```

Then test:

```bash
ping -c 3 10.21.31.103
ping -c 3 10.21.31.104
ping -c 3 10.21.31.106
ssh user@10.21.31.104
ssh user@10.21.31.106
```

If the connector exposes the `10.21.33.0/24` side instead, try `10.21.33.200/24` and test `10.21.33.106`.

NetworkManager example on the computer:

```bash
nmcli dev status
sudo nmcli con mod "<robot-wired-connection>" \
  ipv4.method manual \
  ipv4.addresses 10.21.31.200/24 \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv4.never-default yes \
  ipv6.method ignore
sudo nmcli con up "<robot-wired-connection>"
```

Important:

- Avoid IP conflicts with 103/104/106.
- Do not enable "shared to other computers" or a DHCP server on the robot internal network unless explicitly creating an isolated recovery link.
- If the computer only has one wired NIC and uses wired internet, add Wi-Fi or a USB Ethernet adapter for the robot link.

Clarification:

- For installing project dependencies, transferring code, SSH, running rosbag, and launching the custom 104 stack, only 104 needs to be reachable from the development computer.
- 103/AOS and 106/NOS do not need internet access.
- Do not change 103/106 network settings for dependency installation on 104.
- 104 still needs to keep its internal route to 103/106, because:
  - 103 executes TCP motion commands;
  - 106 provides map/localization and map files;
  - 104 may query or copy from them.
- Best setup:
  - computer internet via Wi-Fi;
  - computer Ethernet via M12/RJ45 to 104/robot subnet;
  - 104 optional internet via a separate interface if packages must be installed directly on 104;
  - 103/106 remain isolated on their robot internal network.

## Work Before M12/RJ45 Cable Arrives

User asked what can still be done while waiting for the network cable, toward the final autonomous inspection goal.

Highest-value work:

1. Build a single-floor inspection mission layer before multi-floor work:
   - waypoint YAML for named inspection points;
   - mission manager that sends Nav2 goals, waits for result, stops, triggers YOLO sampling, records result, and continues.
2. Prepare real-robot deployment assets:
   - 104 deployment checklist;
   - offline Nav2 dependency plan;
   - rosbag and snapshot collection commands;
   - safety startup sequence with `enable_axis_command:=false` first.
3. Prepare perception:
   - convert YOLOv8 model to ONNX/RKNN for RK3588;
   - create class list and confidence thresholds;
   - define what counts as inspection event: e.g. smoke, helmet missing, fire, person, etc.
4. Prepare maps/waypoints:
   - clean one-floor map;
   - define stable patrol points and inspection headings;
   - avoid relying on 106 hand-controller points for the 104 mission.
5. Prepare validation:
   - use sim to test the mission manager end-to-end;
   - record simulated bags;
   - test recovery behavior when a waypoint fails.

Recommended priority:

- Do not prioritize full 104-side localization or multi-floor stair autonomy yet.
- First complete a robust single-floor autonomous inspection loop:

```text
start mission -> navigate to point -> stop -> inspect -> record -> next point -> finish report
```

Suggested next code task:

- Add `m20pro_mission` or add a `mission_manager` node under `m20pro_navigation`/new package.
- Add config:

```text
src/m20pro_bringup/config/inspection_waypoints.yaml
```

- Node behavior:
  - load named waypoints;
  - send `NavigateToPose` goals;
  - subscribe/read YOLO events;
  - save JSON/CSV report under `inspection_reports/`;
  - support pause/cancel/retry.

## Inspection Waypoint Setup

User asked how to set inspection points.

Decision:

- Inspection points should be maintained on the 104 side, not through the 106 hand-controller waypoint system.
- Each point needs:
  - stable navigation pose in `map`: `x`, `y`, `yaw`;
  - camera direction/heading;
  - inspection dwell time;
  - target YOLO classes / event policy;
  - retry/failure policy.

Added template:

```text
src/m20pro_bringup/config/inspection_waypoints.yaml
```

Recommended ways to capture a waypoint:

1. Simulation / RViz:
   - run `ros2 topic echo --once /goal_pose`;
   - click `2D Goal Pose` in RViz;
   - copy `position.x`, `position.y`, and convert quaternion to yaw.
2. Real robot:
   - move the robot to a safe inspection pose;
   - rotate it so the camera faces the target;
   - read pose from `/m20pro_tcp_bridge/map_pose` or TF:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

   - copy translation x/y and RPY yaw into `inspection_waypoints.yaml`.

Important:

- For real robot deployment, the waypoint coordinates must correspond to the 106 active map copied to 104.
- Do not crop/rotate/rescale maps after setting waypoints.
- Prefer 3 to 5 stable single-floor waypoints first, not a large patrol list.
- Keep points away from walls/doors/stairs by at least a practical safety margin; exact value depends on the local costmap footprint/inflation.

## Hand-Controller Waypoint Reuse

User noted that the M20 Pro hand-controller/factory UI can add path points, task points, and charging points by driving to a pose and pressing one button. User asked whether subscribing/importing those points into 104 would be ideal.

Assessment:

- Yes, this would be an excellent acquisition workflow.
- But it should not be assumed to be a ROS topic.
- The developer manual clearly documents sending a single navigation target with `MapID`, `PosX`, `PosY`, `PosZ`, `AngleYaw`, and `PointInfo`, but no query/list API for previously saved hand-controller waypoints has been confirmed.
- The hand-controller points likely live on 106/NOS in a map package, database, JSON/YAML/config file, or internal service.

Recommended paths:

1. Best if discoverable:
   - add one obvious point from the hand-controller;
   - inspect 106 for recently modified files;
   - find the waypoint database/file;
   - write an importer from 106 format to `inspection_waypoints.yaml`.
2. If points are published or services exist:
   - use `ros2 topic list`, `ros2 service list`, and diagnostics snapshots on 106;
   - subscribe/call that interface from 104 if stable.
3. If official storage/interface is opaque:
   - implement 104-side teach mode:
     - drive robot to a pose;
     - press a keyboard/web/ROS service button on 104;
     - capture `/m20pro_tcp_bridge/map_pose`;
     - append `{x, y, yaw}` to `inspection_waypoints.yaml`.

106 discovery commands after adding a test waypoint:

```bash
date
find /var/opt/robot/data/maps -type f -mmin -10 -ls | sort -k 8,9
find /var/opt/robot/data -type f -mmin -10 -ls | sort -k 8,9
find /var/opt/robot -type f \( -name '*.db' -o -name '*.sqlite' -o -name '*.json' -o -name '*.yaml' -o -name '*.xml' \) -mmin -30 -ls
```

If a database is found:

```bash
sqlite3 <file.db> '.tables'
```

Design decision:

- Even if 106 hand-controller points can be imported, the final autonomous inspection should still run from 104's mission config and mission manager.
- 106/hand-controller should be treated as a convenient waypoint acquisition tool, not the runtime mission authority.

## Inspecting 106 Waypoint Storage

User is SSH-connected to 106 and asked how to view waypoint information.

Recommended read-only workflow:

1. Identify active map package:

```bash
readlink -f /var/opt/robot/data/maps/active
find /var/opt/robot/data/maps/active -maxdepth 3 -type f | sort
```

2. Before adding a new point, record file state:

```bash
find /var/opt/robot/data -type f -printf '%T@ %p\n' | sort -n | tail -80
```

3. Add a very obvious test waypoint/task point from the hand-controller, e.g. `codex_test_001`.

4. Immediately search modified files:

```bash
find /var/opt/robot/data -type f -mmin -10 -ls | sort -k 8,9
find /var/opt/robot -type f \( -name '*.db' -o -name '*.sqlite' -o -name '*.json' -o -name '*.yaml' -o -name '*.xml' -o -name '*.csv' -o -name '*.txt' \) -mmin -30 -ls
```

5. Search for the point name or likely fields:

```bash
grep -RIn --exclude='*.pgm' --exclude='*.pcd' --exclude='*.bin' 'codex_test_001\|PointInfo\|PosX\|AngleYaw\|task\|point\|waypoint' /var/opt/robot/data 2>/dev/null
```

6. If a sqlite database is found:

```bash
sqlite3 <file.db> '.tables'
sqlite3 <file.db> '.schema'
```

Important:

- Copy candidate files out before opening/editing.
- Do not modify `/var/opt/robot/data/maps/active` directly.
- Do not run `drmap unpack` or switch active maps during waypoint discovery.

Update:

- User ran `ros2 topic list` on the robot and found several official navigation/point topics:

```text
/GOAL
/HANDLER_POINTS_DEBUG
/NAV_CMD
/NAV_POINTS
/TRACK_PATH
/GRID_MAP
/LOCATION_STATUS
/LIO_ODOM
/ODOM
/cloud_nav
```

Assessment:

- `/NAV_POINTS` is now the highest-priority candidate for hand-controller saved path/task/charging points.
- `/HANDLER_POINTS_DEBUG` is also suspicious and may expose points added from the handler/hand-controller workflow.
- `/GOAL` likely exposes current official navigation target.
- `/TRACK_PATH` and `/path` may expose planned/executed path rather than saved point database.

Next commands to run on 106:

```bash
for t in /NAV_POINTS /HANDLER_POINTS_DEBUG /GOAL /TRACK_PATH /NAV_CMD /GRID_MAP /LOCATION_STATUS /LIO_ODOM /ODOM; do
  echo "===== $t ====="
  ros2 topic info -v "$t"
  ros2 topic type "$t"
done
```

Then inspect message definitions:

```bash
ros2 interface show <message_type>
```

For candidate point topics, try both normal and transient-local echo:

```bash
ros2 topic echo --once /NAV_POINTS
ros2 topic echo --qos-durability transient_local --once /NAV_POINTS
ros2 topic echo --once /HANDLER_POINTS_DEBUG
ros2 topic echo --qos-durability transient_local --once /HANDLER_POINTS_DEBUG
```

If echo blocks, add a new point from the hand-controller while echo is running, or record a short bag:

```bash
ros2 bag record -o waypoint_probe /NAV_POINTS /HANDLER_POINTS_DEBUG /GOAL /TRACK_PATH /NAV_CMD /LOCATION_STATUS /ODOM /LIO_ODOM
```

Goal:

- Identify a stable topic/message containing point id/name/type/x/y/yaw/map id.
- Then implement a 104-side importer/subscriber to convert official points into `inspection_waypoints.yaml`.

Observed topic types:

```text
/NAV_POINTS: sensor_msgs/msg/PointCloud2
/HANDLER_POINTS_DEBUG: sensor_msgs/msg/PointCloud2
```

Publisher is `_CREATED_BY_BARE_DDS_APP_`, reliable/volatile QoS.

Implication:

- These are not structured waypoint-list messages with names/types by default.
- They are likely official navigation/handler visualization/debug point clouds.
- They may still contain xyz coordinates of saved points, but probably not semantic metadata such as point name, task/charging type, inspect duration, or class targets.

Next minimal commands on 106:

```bash
ros2 topic echo --once --qos-reliability reliable /NAV_POINTS --field header
ros2 topic echo --once --qos-reliability reliable /NAV_POINTS --field width
ros2 topic echo --once --qos-reliability reliable /NAV_POINTS --field fields
ros2 topic echo --once --qos-reliability reliable /HANDLER_POINTS_DEBUG --field fields
```

If fields include only `x/y/z` or `x/y/z/rgb`, the topic can at most help extract point coordinates. If it includes additional fields such as `yaw`, `id`, `type`, or `label`, it may be importable into 104 inspection waypoints.

## Localization Before Waypoint Collection

User asked whether the robot should first follow the software manual to initialize/set its position before collecting or inspecting waypoints.

Answer:

- Yes. 106/NOS localization must be initialized and visually verified before:
  - adding hand-controller path/task/charging points;
  - recording waypoints for 104;
  - running 104 `m20pro_real.launch.py`;
  - recording real robot bags for navigation debugging.
- If localization is wrong, every saved point and every `/m20pro_tcp_bridge/map_pose` sample will be wrong.

Manual-backed workflow:

1. Ensure the correct 106 active map is loaded:

```bash
readlink -f /var/opt/robot/data/maps/active
```

2. On 106/NOS, open official localization RViz:

```bash
su
source /opt/ros/foxy/setup.bash
export XAUTHORITY=/home/user/.Xauthority
rviz2
```

3. In RViz, open:

```text
/opt/robot/share/localization/conf/localization.rviz
```

4. Check whether live point cloud overlaps the map.
5. If not, use `2D Pose Estimate` at the robot's real map position and drag the arrow to match robot heading.
6. Only after point cloud and map overlap should the user:
   - add official hand-controller points;
   - read `/NAV_POINTS` / `/HANDLER_POINTS_DEBUG`;
   - read `/m20pro_tcp_bridge/map_pose`;
   - save 104 inspection waypoints.

Validation commands:

```bash
ros2 topic echo /LOCATION_STATUS
ros2 topic echo /ODOM
ros2 topic echo /LIO_ODOM
```

Exact fields need to be confirmed on 106, but the practical visual check remains: live cloud must align with the active map.

## Real-Robot Goal Publishing Decision

User asked where to publish path points/goals after deploying to 104: RViz `2D Goal Pose` on 104, or hand-controller navigation points.

Decision:

- For this project, goals should be published into the 104 ROS 2/Nav2 stack, normally through RViz `2D Goal Pose` on topic `/goal_pose`.
- The hand-controller navigation mode writes/uses 106 official navigation waypoints. Those points belong to the factory navigation stack and are not automatically consumed by 104 Nav2.
- Do not run 106 factory navigation and 104 Nav2 axis-command control at the same time. If the hand-controller/factory navigation is active, keep 104 in shadow mode.

Practical goal entry options:

1. RViz on 104 via VNC:
   - launch `m20pro_real.launch.py`
   - use `2D Goal Pose`
   - goal goes to `/goal_pose`
2. RViz on a laptop:
   - same network and matching `ROS_DOMAIN_ID`
   - source the workspace if needed
   - connect to 104 DDS and use `2D Goal Pose`
3. SSH/no GUI:
   - publish a `geometry_msgs/msg/PoseStamped` once to `/goal_pose`.

Real-robot caution:

- RViz `2D Pose Estimate` is not yet a reliable real-robot relocalization path because `tcp_bridge` still needs `/initialpose -> Type=2101 Command=1` support.
- First use `enable_axis_command:=false` to confirm `/plan` and RViz alignment, then use `enable_axis_command:=true` only for a short nearby test goal.

Long-term patrol:

- Do not rely on manually clicking RViz goals for inspection missions.
- Add a 104-side waypoint/mission YAML and a mission manager that sends goals to Nav2 and triggers YOLO inspection at each task point.

## Real Launch `map:=` Meaning

User asked what path `map:=$HOME/m20pro_active_map/occ_grid.yaml` refers to.

Clarification:

- The `map:=...` launch argument is the map YAML loaded by the 104-side Nav2 `map_server`.
- It does not switch the 106/NOS active map.
- On the real robot it should point to a copy of the 106/NOS current active map package, because 106 provides localization and 104 plans using that copied map.
- The expected source on 106 is usually:

```text
/var/opt/robot/data/maps/active/occ_grid.yaml
/var/opt/robot/data/maps/active/occ_grid.pgm
```

- After copying to 104, an example path is:

```text
$HOME/m20pro_active_map/occ_grid.yaml
```

Example:

```bash
scp -r user@10.21.31.106:/var/opt/robot/data/maps/active "$HOME/m20pro_active_map"
ros2 launch m20pro_bringup m20pro_real.launch.py \
  map:=$HOME/m20pro_active_map/occ_grid.yaml \
  enable_axis_command:=false
```

Important:

- `occ_grid.yaml` and its referenced image file, usually `occ_grid.pgm`, must stay in the same copied map directory unless the YAML image path is edited correctly.
- For real testing, prefer the copied 106 active map over the repository's built-in sample/edited maps.

## 106 Bypass Decision

User asked whether the project can completely bypass 106/NOS and only use 106 for mapping.

Assessment:

- Bypassing 106 factory navigation/planning is already the current project direction:
  - 104 runs Nav2 planning/control.
  - 103 receives velocity/axis commands through `tcp_bridge`.
  - 106 official waypoint navigation is not used for the main control loop.
- Completely bypassing 106 localization is possible in theory, but not recommended as the first real-robot deployment path.
- Current real chain still depends on 106 for map-frame pose:
  - `tcp_bridge` queries `Type=1007 Command=2`.
  - `/m20pro_tcp_bridge/map_pose`, `/odom`, and TF come from that official pose.
- To remove 106 from runtime, the project would need a 104-side localization stack:
  - LiDAR odometry or scan matching.
  - Map localization such as AMCL or slam_toolbox localization mode.
  - Proper TF tree: `map -> odom -> base_link`.
  - Real sensor TF calibration for `/cloud_nav` / LiDAR frames.
  - Initial pose/relocalization strategy.
  - Failure detection and recovery.

Recommended route:

1. Short term:
   - keep 106 as localization authority;
   - bypass only 106 factory navigation;
   - validate 104 Nav2, pointcloud fusion, and TCP velocity control on one floor.
2. Middle term:
   - run a 104-side localization stack in shadow mode while still consuming 106 pose;
   - compare 104 estimated pose against 106 pose using rosbag/RViz.
3. Long term:
   - switch 104 to own localization only after drift, relocalization, and dynamic-scene robustness are verified.

Reason:

- For inspection, stable localization matters more than architectural purity.
- 106 already owns the real robot's tested map/localization path.
- Replacing 106 localization too early increases risk without improving the immediate navigation/inspection demo.

## 104-Side Localization Feasibility

User asked whether 104 can directly perform localization.

Answer:

- Yes, 104 can run localization in principle.
- Current project does not yet implement a complete 104-side real localization stack.
- The current `/odom` and TF in real mode are derived from 106 pose via `tcp_bridge`, so they cannot be treated as independent odometry if the goal is to bypass 106.

Requirements for 104-side localization:

- A map loaded on 104, usually copied from 106's active map package.
- Live LiDAR or point cloud topic on 104:
  - `/cloud_nav` or raw LiDAR topics.
- Correct sensor-to-base TF calibration.
- An independent continuous odometry source:
  - body/wheel/leg odometry from 103 if available, or
  - LiDAR odometry / scan matching on 104.
- A localization node:
  - AMCL with `/scan` plus independent `/odom`, or
  - `slam_toolbox` localization mode, or
  - 3D LiDAR(-IMU) localization if reliable 3D sensor/IMU data is exposed.
- Initial pose and relocalization workflow.
- Drift/loss detection and recovery.

Recommended progression:

1. Keep 106 localization as the authority for first real deployment.
2. In parallel, run 104-side localization in shadow mode and compare it with `/m20pro_tcp_bridge/map_pose`.
3. Only switch navigation to 104 localization when it stays aligned over real patrol routes, dynamic obstacles, turns, and temporary feature-poor areas.

Implementation idea for a future 104 localization branch:

```text
/cloud_nav -> pointcloud_fusion -> /scan
independent odom source -> /odom
map_server -> /map
AMCL/slam_toolbox -> map->odom
robot_state_publisher/static TF -> base_link/lidar frames
Nav2 consumes map->odom->base_link
```

Key risk:

- Without a real independent `/odom`, AMCL/slam_toolbox localization will be weak or circular. Reusing `/odom` produced from 106 pose does not remove the 106 dependency.

## Codex on 106/NOS

User asked whether running Codex directly on 106 with internet access would help.

Answer:

- Yes, it would help substantially because 106/NOS is the official mapping/localization/navigation host.
- Internet access mainly lets Codex run through the API; the real value is local read access to 106 files, ROS environment, running processes, topics, services, map packages, logs, and official scripts.
- First pass should be read-only. Do not modify 106 official configs, services, startup scripts, maps, or installed packages until the active system is understood and backed up.

Useful read-only discovery targets on 106:

```bash
hostname
ip addr
printenv | sort | rg 'ROS|RMW|CYCLONE|FAST|DOMAIN'
ps -eo pid,ppid,cmd | rg 'ros|slam|map|nav|local|dr|lidar|camera'
ros2 topic list
ros2 node list
ros2 service list
ros2 topic echo --once /cloud_nav
ls -lah /var/opt/robot/data/maps
readlink -f /var/opt/robot/data/maps/active
find /var/opt/robot/data/maps/active -maxdepth 2 -type f | sort
drmap -h
```

Safety guidance:

- Prefer copying maps/logs/config snippets out of 106 for analysis.
- Avoid `apt install`, service restarts, map activation changes, deleting files, or editing official configs on 106 during first exploration.
- If Codex is run on 106, keep a separate Git workspace under the user's home directory and do not place project files inside official system directories.

## Offline Robot Diagnostics for Codex

User asked how Codex can inspect real running nodes/topics when the robot has no internet.

Solution:

- Add a read-only collection script:
  - `tools/collect_ros_snapshot.sh`
- The script can be copied to 104 or 106 and run without internet.
- It creates:
  - `m20pro_ros_snapshot_<host>_<time>.tar.gz`
- Bring the `.tar.gz` back by USB/scp and place it in this workspace so Codex can inspect it locally.

Collected data:

- ROS topic/node/service/action lists.
- `ros2 topic info -v` for discovered topics.
- `hz`/`bw` for key topics that actually exist.
- one-shot echo for key non-heavy topics.
- node info and parameter dumps.
- TF frames via `tf2_tools view_frames` when available.
- `tf2_echo` for `map->base_link` and `odom->base_link`.
- system env, network, process list, robot-related services.
- 106 map directory listing and active `occ_grid.yaml` if present.
- recent journal logs.

Safety:

- The script is read-only.
- It does not publish motion commands.
- It does not restart services.
- It does not modify maps or official configs.

README was updated with an "offline diagnostics" section.

## Direct Desktop-to-Robot Network Access

User asked whether a desktop PC can connect to the M20 Pro by cable so Codex can inspect the robot directly. The robot has an Ethernet interface, but apparently not a normal RJ45 port.

Answer:

- Yes, if the desktop can reach the robot hosts over IP, Codex running on the desktop can inspect the robot by SSH/SCP. The robot itself does not need internet.
- Physical connection still needs the correct adapter/cable:
  - vendor Ethernet-to-RJ45 adapter/cable if the robot uses an aviation/industrial Ethernet connector;
  - or a supported USB Ethernet adapter on the target host if allowed;
  - do not improvise pinouts on the robot Ethernet connector.
- For a direct cable link with no DHCP, set the desktop Ethernet interface to a static IP in the robot subnet, for example `10.21.31.200/24`, avoiding robot addresses:
  - 103 AOS: `10.21.31.103`
  - 104 GOS: likely `10.21.31.104`
  - 106 NOS: `10.21.31.106`
- Then test:

```bash
ping -c 3 10.21.31.104
ping -c 3 10.21.31.106
ssh user@10.21.31.104
ssh user@10.21.31.106
```

If SSH works from the desktop, Codex can run read-only commands on the robot via SSH from the local workspace, for example:

```bash
ssh user@10.21.31.106 'source /opt/ros/foxy/setup.bash; ros2 topic list -t'
```

Safety:

- Avoid IP conflicts with 103/104/106.
- Prefer read-only inspection first.
- Do not restart services or modify official 106 map/localization files during initial exploration.

## 104 Network Recovery Note

User changed the 104 host IPv4 setting from manual/static to automatic/DHCP on the network used for inter-host robot communication, then lost SSH access to 104.

Likely cause:

- 104 originally needed a static address on the robot internal subnet, likely `10.21.31.104/24`.
- Switching to automatic/DHCP probably removed that address. If no DHCP server exists on that internal link, 104 has no reachable IP.

Recovery plan:

1. Do not change 103/106 network settings.
2. Get local access to 104 via monitor/keyboard, vendor desktop/VNC if still reachable through another interface, or serial/maintenance access if available.
3. Restore the internal Ethernet interface to static:

```bash
nmcli con show
nmcli dev status
sudo nmcli con mod "<connection-name>" ipv4.method manual ipv4.addresses 10.21.31.104/24 ipv4.gateway "" ipv4.dns ""
sudo nmcli con down "<connection-name>"
sudo nmcli con up "<connection-name>"
ip addr
```

If NetworkManager is not used, inspect `/etc/netplan/*.yaml` and restore static config, then run:

```bash
sudo netplan apply
```

4. From the desktop, test:

```bash
ping -c 3 10.21.31.104
ssh user@10.21.31.104
```

If local 104 access is impossible, try reaching 106 first and scan/ping the robot subnet from 106, but if 104 has no IP only local recovery will work.

Follow-up:

- User said 104 cannot be connected to a display.
- Recovery must first try remote discovery:
  1. Connect desktop to the robot internal network and assign the desktop a safe static address such as `10.21.31.200/24`.
  2. Verify 103/106 are reachable.
  3. Scan the `10.21.31.0/24` subnet with `arp-scan` or `nmap -sn`.
  4. Try mDNS/hostname discovery with `avahi-browse -art`, `ssh user@<hostname>.local`, and router/DHCP lease tables if any DHCP server was connected.
  5. If reachable through 106, scan from 106 as well.
- If 104 got no IP at all after switching to DHCP and has no local console, the practical recovery routes are:
  - vendor maintenance tool/port;
  - USB/serial/OTG console if the 104 carrier exposes it;
  - temporarily provide a DHCP server on the internal link, then SSH into the leased address and restore static config;
  - remove/access the 104 compute module storage only if the vendor hardware design permits it.
- Do not reset or change 103/106 network settings while trying to recover 104.

Update:

- User can still connect to 106.
- Use 106 as a read-only recovery vantage point:
  - inspect 106 interfaces/routes;
  - ping `10.21.31.104`;
  - check ARP/neighbor tables;
  - scan `10.21.31.0/24`;
  - try mDNS/IPv6 link-local discovery;
  - only if 104 is found, SSH into 104 and restore its static IPv4 config.
- If 106 cannot see 104 at all, 104 probably has no IPv4 address after switching to DHCP; then recovery needs DHCP injection, vendor maintenance access, serial/USB console, or other hardware-level access.

Observed from 106:

```text
eth0: 10.21.33.106/24
eth1: 10.21.31.106/24
route: 10.21.31.0/24 via eth1
ping 10.21.31.104: success, 0% packet loss
arp/neigh: 10.21.31.104 reachable on eth1, MAC 02:fd:f5:c1:ad:cd
```

Conclusion:

- 104 is still alive and reachable from 106 at `10.21.31.104`.
- The user's desktop SSH problem is likely a routing/subnet issue, not total 104 network loss.
- Next steps:
  - From 106, try `ssh user@10.21.31.104`.
  - From desktop, use 106 as a jump host:

```bash
ssh -J user@10.21.31.106 user@10.21.31.104
```

  - Or set up a local tunnel:

```bash
ssh -L 2222:10.21.31.104:22 user@10.21.31.106
ssh -p 2222 user@127.0.0.1
```

- If direct desktop access is desired, put the desktop NIC on `10.21.31.0/24` instead of a different robot subnet.

Further update:

- User tried `ssh user@10.21.31.104` from 106.
- SSH added the host key, then immediately printed:

```text
Connection closed by 10.21.31.104 port 22
```

Meaning:

- 104 is reachable and something is listening on TCP/22.
- The SSH server closes before password authentication.
- Next diagnostics should be non-destructive:

```bash
nc -vz 10.21.31.104 22
ssh -vvv user@10.21.31.104
ssh -o PreferredAuthentications=password -o PubkeyAuthentication=no -vvv user@10.21.31.104
```

Also try likely alternate usernames if documented by the vendor or previously used.

Resolved access update:

- User ran `ssh -vvv user@10.21.31.104` from 106.
- Authentication succeeded with password.
- 104 prompt confirms:

```text
You are now in gos(104)
aos(103) gos(104) nos(106)
```

Conclusion:

- 104 is reachable from 106 at `10.21.31.104`.
- The previous "connection closed" was not a network-loss proof; verbose SSH showed normal password authentication and shell allocation.
- Next actions on 104:
  - inspect `ip -br addr`, `ip route`, `nmcli dev status`, `nmcli con show --active`;
  - verify whether the 10.21.31 interface is still static or became DHCP;
  - if it became DHCP, persistently restore the connection to `10.21.31.104/24` without changing 103/106.

Final recovery update:

- User found direct SSH to 104 works again.
- The earlier IPv4 "automatic" change likely was not saved/applied, or it did not affect the active internal address persistently.
- No network repair is needed unless later checks show DHCP or missing static config.
- Recommended baseline to capture on 104:

```bash
ip -br addr
ip route
nmcli dev status
nmcli con show --active
nmcli con show
```

- For future internet access, do not change the robot internal interface. Use another interface such as Wi-Fi, USB Ethernet, USB tethering, or desktop NAT/shared network.

TF warning update:

- User reported RViz TF warnings while running `m20pro_sim`, such as:

```text
No transform from [lf_foot_link] to [map]
```

- Current project URDF `src/m20pro_description/urdf/M20.urdf` does not contain `lf_foot_link`.
- The actual simulated robot links are:

```text
base_link
fl_hipx fl_hipy fl_knee fl_wheel
fr_hipx fr_hipy fr_knee fr_wheel
hl_hipx hl_hipy hl_knee hl_wheel
hr_hipx hr_hipy hr_knee hr_wheel
```

- `m20pro_sim.launch.py` starts:
  - `zero_joint_state_publisher`
  - `robot_state_publisher`
  - `sim_bridge`
  - lidar/fusion/dynamic obstacle simulators
  - Nav2 map server/navigation
  - RViz with fixed frame `map`
- `zero_joint_state_publisher.py` extracts all non-fixed joints from `/robot_description` and publishes `/joint_states`.
- Therefore, persistent warnings about `lf_foot_link` probably mean RViz or ROS graph is seeing stale/foreign robot model frames, not the current M20 URDF.
- First recovery steps:

```bash
pkill -f "ros2 launch m20pro_bringup m20pro_sim.launch.py"
pkill -f "rviz2"
ps -eo pid,ppid,cmd | rg 'm20pro_sim|robot_state_publisher|zero_joint_state|sim_bridge|nav2_|map_server|lifecycle_manager|rviz2|pointcloud|dual_lidar|dynamic_obstacle'
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch m20pro_bringup m20pro_sim.launch.py
```

- If warnings remain, verify:

```bash
ros2 topic info /robot_description -v
ros2 topic info /tf -v
ros2 topic info /tf_static -v
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map fl_wheel
ros2 run tf2_ros tf2_echo map lf_foot_link
```

- Expected:
  - `map -> base_link` works.
  - `map -> fl_wheel` works.
  - `map -> lf_foot_link` fails, because this project has no such link.

Local costmap size note:

- Sim launch uses `src/m20pro_bringup/config/nav2_params.yaml`.
- Real robot launch uses `src/m20pro_bringup/config/nav2_params_foxy.yaml`.
- Local costmap size is configured under:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
```

- `width` and `height` are in meters.
- `resolution` is meters per grid cell.
- Current 5 m x 5 m with 0.05 m resolution is about 100 x 100 cells.

Sim waypoint test note:

- `src/m20pro_bringup/config/inspection_waypoints.yaml` is currently a mission/inspection waypoint configuration template.
- The immediately runnable waypoint mechanism in simulation is Nav2's `waypoint_follower` action:

```bash
/follow_waypoints
nav2_msgs/action/FollowWaypoints
```

- Confirm it exists after launching sim:

```bash
ros2 action list | grep waypoint
ros2 action info /follow_waypoints
```

- Example action goal using the current sample points:

```bash
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints "{poses: [{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {z: 1.0, w: 0.0}}}]}"
```

- `waypoint_pause_duration` in `nav2_params.yaml` controls pause time at each waypoint. The current value is `200` ms.

Local costmap inflation correction:

- User clarified that the intended change was obstacle inflation, not local costmap width/height.
- Inflation is configured in `inflation_layer`, not `width`/`height`.
- Sim file:

```text
src/m20pro_bringup/config/nav2_params.yaml
```

- Real Foxy file:

```text
src/m20pro_bringup/config/nav2_params_foxy.yaml
```

- Current local costmap values:

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 6.0
  inflation_radius: 0.40
```

- Meaning:
  - `inflation_radius`: how far obstacle cost expands, in meters.
  - `cost_scaling_factor`: how fast the cost decays. Larger value means cost drops faster and the high-cost band becomes narrower.
- For less conservative local obstacle avoidance, first try:

```yaml
cost_scaling_factor: 8.0
inflation_radius: 0.35
```

- Keep `robot_radius: 0.25` in mind; do not reduce inflation too aggressively on the real robot.

Weekly report update:

- User provided `/home/fabu/桌面/耿浩威5.15周报.docx` and asked to polish/expand it based on project records.
- The original file had a `.docx` extension but was actually an old WPS/Word compound document.
- Created backup:

```text
/home/fabu/桌面/耿浩威5.15周报_原稿备份.docx
```

- Rewrote the main report as a standard Word 2007+ `.docx` at:

```text
/home/fabu/桌面/耿浩威5.15周报.docx
```

- Expanded sections include:
  - weekly overview;
  - YOLOv8 inspection classes and RKNN deployment interface;
  - Nav2 simulation, local control, dynamic obstacle handling, costmap inflation, and TF warning diagnosis;
  - real robot deployment preparation across 103/104/106;
  - Foxy/Humble compatibility;
  - map/localization preparation;
  - ROS bag and offline diagnostics;
  - waypoint configuration and multi-floor inspection logic;
  - current risks and next week plan.
- Avoided claiming that many Python nodes had already been converted to C++ because the current repo remains mainly Python; phrased that part as deployment/performance preparation instead.

GitHub update note:

- User asked to update GitHub.
- Before committing, checked pending changes and cleaned a duplicated dataset URL in `src/m20pro_inspection/models/README.md`.
- Ran:

```bash
git diff --check
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

- Build result:

```text
Summary: 4 packages finished
```

- Planned commit contents include:
  - README cleanup for public GitHub usage;
  - Foxy-compatible real-robot Nav2 config and behavior tree;
  - inspection waypoint template;
  - offline ROS snapshot collection script;
  - safer real launch axis-command toggle;
  - costmap/navigation tuning updates;
  - inspection model README cleanup;
  - project notes in `codex.md`.

Repository hygiene correction:

- User noticed `codex.md` should not have been committed to GitHub.
- Decision: keep `codex.md` as a local recap file, remove it from Git tracking, and add it to `.gitignore`.
- This preserves the local file for future ChatGPT/Codex review while avoiding future public pushes.
- Since it was already pushed once, a normal cleanup commit only removes it from the current branch tip; old Git history may still contain it unless history is rewritten.

Git history cleanup update:

- User asked to further handle the already-pushed `codex.md`.
- Used `git filter-branch` to remove `codex.md` from all commits on `main`.
- Deleted `refs/original`, expired reflogs, and ran garbage collection.
- Verified locally:

```bash
git log --all -- codex.md
```

returned no commits.
- The local recap file still exists and is ignored by Git:

```text
codex.md
```

- Remote `main` before force update was:

```text
b49cb50509a49e2730ad8b22e078f85876905abf
```

- Rewritten local `main` is:

```text
f1ae622e5cf7c35f0a624ef3f85d93d535575b91
```

Remote cleanup confirmation:

- User checked GitHub repo page and saw the repo was updated just now and `codex.md` was gone.
- Local verification:

```bash
git ls-remote origin refs/heads/main
```

returned:

```text
f1ae622e5cf7c35f0a624ef3f85d93d535575b91 refs/heads/main
```

- This matches local `HEAD`.
- No background `git push`, `pack-objects`, or GitHub SSH process remained.
- Current `codex.md` is local-only and ignored by Git.

RViz gait visualization note:

- User noticed that in RViz simulation, in-place rotation looks like the robot body rotates directly while legs do not move, similar to an omni/mecanum visual effect.
- Explanation:
  - Current RViz simulation is a kinematic/navigation visualization, not a gait or physics simulation.
  - `zero_joint_state_publisher.py` publishes all non-fixed URDF joint positions as `0.0`, so legs/wheels stay visually static.
  - `sim_bridge_node.py` integrates `/cmd_vel` into `map_pose`/`odom` and rotates `base_link` directly.
  - Therefore RViz only proves Nav2 planning, TF, costmaps, and high-level velocity command behavior, not leg motion.
- On the real robot:
  - Nav2 publishes `/cmd_vel`.
  - `tcp_bridge_node.py` converts `/cmd_vel` linear/yaw velocity into vendor axis commands.
  - The 103/AOS low-level controller chooses the actual gait/leg motion according to the robot's current mode/gait.
  - Real in-place turning should therefore be executed by the robot's own legged/wheeled-legged control, not by RViz-style sliding.
- Caveat:
  - Real behavior depends on the selected gait/control mode and whether `enable_axis_command:=true` is enabled in `m20pro_real.launch.py`.

Gait animation note:

- User asked whether RViz simulation needs a vendor gait `.pt` model to make the legs move.
- Answer:
  - Not necessarily.
  - A `.pt` file is only relevant if the gait controller is a PyTorch/learned policy.
  - Many commercial quadruped/wheeled-legged robots use proprietary firmware/C++ controllers rather than an exposed `.pt` model.
- For this project there are three possible levels:
  1. Visual-only RViz gait animation:
     - publish non-zero `/joint_states` based on `/cmd_vel`;
     - replace or extend `zero_joint_state_publisher.py`;
     - no vendor `.pt` needed;
     - useful for presentation, but not physically accurate.
  2. Data-driven gait animation:
     - record real `/JOINTS_DATA` or `/JOINTS_DATA_10HZ` while the robot walks/turns;
     - fit or replay joint cycles in simulation;
     - closer visual match, still not real dynamics.
  3. True physics/gait simulation:
     - requires dynamics/contact simulation in Gazebo/Ignition/Isaac/MuJoCo;
     - needs actuator limits, inertial parameters, contact/friction, controller logic;
     - vendor gait controller or very good recorded data would help;
     - a `.pt` policy is only one possible implementation, not a requirement.
- For current Nav2 development, visual-only gait animation is enough if the goal is to make RViz look less like a sliding base.

Real-robot relocalization implementation:

- User asked whether the project should implement relocalization according to the M20 Pro software manual.
- Manual section `3.5 初始化定位` describes using RViz `2D Pose Estimate` to initialize/reset localization.
- The migrated vendor GUI code already uses:

```text
Type=2101
Command=1
Items={"PosX", "PosY", "PosZ", "Yaw"}
```

- Implemented in `src/m20pro_navigation/m20pro_navigation/tcp_bridge_node.py`:
  - subscribes to `/initialpose` as `geometry_msgs/msg/PoseWithCovarianceStamped`;
  - converts orientation to yaw;
  - sends the vendor TCP request `2101/1`;
  - publishes result to `/m20pro_tcp_bridge/relocalization_result`;
  - refreshes map pose/navigation status after success.
- Added parameters in `src/m20pro_bringup/config/m20pro.yaml`:

```yaml
enable_initialpose_relocalization: true
initialpose_topic: "/initialpose"
relocalization_response_timeout_s: 2.0
```

- Added `m20pro_real.launch.py` launch argument:

```bash
enable_initialpose_relocalization:=true|false
```

- Updated README with real-robot usage.
- Verification:

```bash
python3 -m py_compile src/m20pro_navigation/m20pro_navigation/tcp_bridge_node.py src/m20pro_bringup/launch/m20pro_real.launch.py
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

- Build passed:

```text
Summary: 4 packages finished
```

Real camera / inspection test plan:

- User asked whether connecting the robot and computer by Ethernet allows camera use in RViz and YOLO inspection testing.
- Current `m20pro_inspection` defaults to RTSP input:

```text
front wide: rtsp://10.21.31.103:8554/video1
rear wide:  rtsp://10.21.31.103:8554/video2
```

- RViz cannot display RTSP directly. It displays ROS `sensor_msgs/msg/Image`.
- The inspection node reads RTSP and publishes:

```text
/m20pro_yolov8_inspection/annotated_image
/m20pro_yolov8_inspection/detections
/m20pro_yolov8_inspection/events
```

- First network checks after Ethernet connection:

```bash
ping -c 3 10.21.31.103
nc -vz 10.21.31.103 8554
```

- First stream-only test:

```bash
ros2 launch m20pro_inspection m20pro_inspection.launch.py \
  backend:=dry_run \
  rtsp_url:=rtsp://10.21.31.103:8554/video1
```

- In RViz add an `Image` display and select:

```text
/m20pro_yolov8_inspection/annotated_image
```

- Desktop model test should normally use ONNX:

```bash
ros2 launch m20pro_inspection m20pro_inspection.launch.py \
  backend:=onnx \
  model_path:=/path/to/best.onnx
```

- RKNN model test should run on RK3588/104 with RKNN runtime installed.

Mac SSH GUI / RViz note:

- User is using a Mac laptop to SSH into the Ubuntu development host where Codex and the ROS 2 workspace run.
- Codex runs on the Ubuntu host because the command is executed inside the SSH shell; the Mac is only the terminal frontend.
- RViz is a Linux GUI/OpenGL application, so it will not appear on the Mac unless a GUI forwarding/display solution is used.
- Options:
  1. XQuartz + SSH X11 forwarding:
     - install XQuartz on Mac;
     - run `ssh -Y user@ubuntu-host`;
     - check `echo $DISPLAY`;
     - run `rviz2`;
     - works for simple GUI, but RViz/OpenGL performance can be poor.
  2. Remote desktop to Ubuntu:
     - use NoMachine, TurboVNC/VirtualGL, xrdp, or similar;
     - usually better for RViz because the GUI and GPU/OpenGL stay on Ubuntu.
  3. Web visualization:
     - use Foxglove/rosbridge style workflow for topics;
     - good for camera, TF, paths, point clouds;
     - not a full replacement for every RViz interaction but useful for remote inspection.
- Recommendation for this project:
- Use remote desktop for RViz/Nav2 tuning.
- Use SSH terminal for Codex/build/launch.
- Use Web/Foxglove for lightweight camera/topic monitoring when available.

Mac Homebrew repair note:

- User installed XQuartz manually and asked to focus on repairing Homebrew.
- Observed Mac-side symptoms:
  - `brew update` fails because `homebrew/services` points to `https://mirrors.ustc.edu.cn/homebrew-services.git`, which returns not found.
  - Homebrew also tries to download portable Ruby from `https://mirrors.ustc.edu.cn/homebrew-bottles/...`, returning 404.
  - `/opt/homebrew` remote is official GitHub.
  - `homebrew-core` and `homebrew-cask` tap directories do not exist locally.
- Recommended repair:
  1. Remove the broken services tap directory manually, without invoking `brew`.
  2. Clear `HOMEBREW_BOTTLE_DOMAIN` / `HOMEBREW_API_DOMAIN` and any persistent mirror env vars from shell startup files.
  3. Retap official `homebrew/core`, `homebrew/cask`, and optionally `homebrew/services`.
  4. Run `brew update-reset` if ordinary `brew update` still fails.
  5. Verify with `brew doctor` and `brew config`.

VLA career/project direction:

- User asked what else to do for applying to VLA roles this year after finishing the current M20 Pro project.
- Judgment:
  - The current project is a strong robotics systems base: ROS 2, Nav2, M20 Pro real-robot integration, maps, relocalization, YOLO inspection, RTSP camera input, waypoint/floor logic.
  - For VLA roles, this is not enough by itself. Add a model/data/learning layer so the project demonstrates embodied AI, not only navigation engineering.
- Best project extension:
  - Turn M20 Pro into a "VLA-lite inspection robot" built around skills:
    - natural language instruction;
    - scene/perception understanding;
    - task decomposition;
    - skill execution through ROS 2/Nav2;
    - safety checks and logging.
- Recommended modules:
  1. Data collection pipeline:
     - record synchronized RGB/RTSP frames, robot pose, goal, cmd_vel, detections, events, task labels, map/floor id.
     - export to a training-friendly dataset format.
  2. Language-to-skill planner:
     - input: "去三楼看看有没有人在抽烟";
     - output skill graph: switch floor -> go to waypoints -> inspect smoke/person -> report.
  3. VLM/VLA policy prototype:
     - input image + instruction + map/waypoint context;
     - output next skill or next waypoint, not raw motor torque.
  4. Evaluation:
     - task success rate, false alarms, time to inspect, navigation recovery count, relocalization success, dynamic obstacle handling.
  5. Demo/documentation:
     - short video, architecture diagram, dataset sample, model card, README section explaining why this is embodied AI/VLA-adjacent.
- Do not abandon the current robotics engineering work. It is the hard-to-copy part. Add AI on top instead.

NoMachine Chinese input note:

- User reported that Chinese input does not work through NoMachine: Ubuntu Sogou input and Mac-side Chinese keyboard both type English only.
- Likely causes:
  - NoMachine usually forwards key events, not the Mac client's local IME committed Chinese text.
  - Chinese IME must run inside the remote Ubuntu session.
  - NoMachine virtual sessions often do not inherit `fcitx`/`ibus` environment variables.
  - Sogou for Linux typically depends on `fcitx`, and can fail in a NoMachine session if `GTK_IM_MODULE`, `QT_IM_MODULE`, and `XMODIFIERS` are missing.
- Diagnostics on the remote Ubuntu session:

```bash
echo $XDG_SESSION_TYPE
echo $GTK_IM_MODULE
echo $QT_IM_MODULE
echo $XMODIFIERS
ps -ef | grep -E 'fcitx|ibus|sogou' | grep -v grep
```

- Recommended fix:
  - Use the remote Ubuntu input method, not Mac's local IME.
  - Ensure `fcitx` starts in the NoMachine session.
  - Add environment variables to `~/.xprofile` or `~/.xsessionrc`.
  - Restart the NoMachine session after changing input method configuration.

Mac XQuartz RViz OpenGL issue:

- User installed XQuartz and confirmed normal X11 windows can be forwarded.
- RViz2 failed with:

```text
Failed to create an OpenGL context. BadValue
RenderingAPIException: Unable to create a suitable GLXContext
```

- Diagnosis:
  - XQuartz X11 forwarding works for simple GUI but often fails for RViz2/Ogre/OpenGL.
  - This is an OpenGL/GLX context limitation, not a Nav2 or launch bug.
- Workarounds to try from the SSH terminal:

```bash
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_INDIRECT=1
rviz2
```

- If that still fails, do not spend much time on XQuartz. Recommended options:
  - run RViz on Ubuntu physical desktop;
  - use NoMachine / xrdp / VNC+VirtualGL to remote into Ubuntu desktop;
  - use XQuartz only for simple GUI tools, not RViz tuning.

Mac Microsoft Windows App / RDP note:

- User asked whether the Mac "Windows App" can forward Ubuntu/RViz windows.
- Clarification:
  - Microsoft Windows App / Remote Desktop is an RDP client.
  - It does not do X11 per-window forwarding like XQuartz.
  - It can connect to Ubuntu only if Ubuntu runs an RDP server such as `xrdp`.
  - It shows a remote desktop session, not a single forwarded RViz window.
- For RViz:
  - xrdp may work, often with software OpenGL; performance can be acceptable for simple RViz but not ideal for point clouds/costmaps.
  - NoMachine is usually a better first choice for RViz/OpenGL on Ubuntu.
  - XQuartz is useful for simple GUI, but RViz2 often fails with GLX context errors.

Remote desktop decision:

- User decided to stop spending time on NoMachine because Chinese input and overall remote desktop experience were not worth the friction.
- Practical recommendation:
  - For general desktop control and Chinese input, prefer ToDesk or Sunlogin if they behave better in the user's environment.
  - For development, prefer SSH + VS Code Remote / terminal workflows.
  - For robot debugging, prefer running RViz locally on an Ubuntu machine that can see the ROS network, or use remote desktop only when full GUI interaction is necessary.

NoMachine uninstall attempt:

- User asked Codex to uninstall NoMachine from the Ubuntu host.
- Checked installed package and service:
  - package: `nomachine 9.5.7-2 amd64`
  - service: `nxserver.service`, loaded/active/running and enabled
- Attempted:

```bash
sudo -n systemctl stop nxserver.service && sudo -n systemctl disable nxserver.service && sudo -n apt-get purge -y nomachine
```

- It was blocked because `sudo` requires the user's password.
- Next step:
  - User can run `sudo -v` locally to cache sudo permission, then Codex can retry.
  - Or user can run the uninstall command directly.
- Follow-up:
  - User ran `sudo -v` in their own terminal, but Codex's execution session still could not reuse that sudo timestamp.
  - Retried with `sudo -n ...`; still failed with `sudo: 需要密码`.
  - User needs to execute the uninstall commands directly in their terminal, then Codex can verify with non-sudo checks.

Sim multi-floor map switching discussion:

- User wants to implement map switching in simulation first, but asked to discuss before changing code.
- Current sim launch starts:
  - `m20pro_navigation/sim_bridge`
  - synthetic lidar/fusion/dynamic obstacle simulator
  - `nav2_map_server/map_server`
  - `nav2_bringup/navigation_launch.py`
- Cleanest map switch entry is Nav2 map server's `/map_server/load_map` service, not restarting Nav2.
- Important detail:
  - switching the occupancy grid alone is not enough;
  - after loading a different floor map, the robot pose must be reset in that floor's map frame.
- The current `sim_bridge` already subscribes to `/initialpose` and resets simulated `x/y/yaw`, which is useful for sim floor switching.
- Recommended architecture:
  - add a higher-level floor/mission manager node, not put this logic directly into Nav2 BT;
  - config file describes floors, map YAML paths, stair portals, and floor-specific waypoints;
  - manager calls `/map_server/load_map`, publishes `/initialpose`, clears Nav2 costmaps, and publishes current floor/status.
- For stair logic:
  - current floor: navigate to stair entry pose;
  - switch gait to stair mode in real robot, mocked/logged in sim;
  - after traversal, load target floor map;
  - publish target floor stair-exit pose as `/initialpose`;
  - switch gait back to normal;
  - continue Nav2 navigation to inspection target.
- For simulation MVP:
  - use existing edited maps as placeholder floors;
  - implement manual floor switching first, e.g. publish `std_msgs/String` target floor ID to a topic;
  - then add automatic stair-portal task logic.

Sim multi-floor map switching implementation:

- Implemented the first MVP version for simulation.
- Added `src/m20pro_navigation/m20pro_navigation/floor_manager.py`.
- Registered console command:

```bash
ros2 run m20pro_navigation floor_manager
```

- Added dependencies in `src/m20pro_navigation/package.xml`:
  - `ament_index_python`
  - `nav2_msgs`
  - `python3-yaml`
- Added `floor_manager` entry point in `src/m20pro_navigation/setup.py`.
- Added optional floor manager launch support in `src/m20pro_bringup/launch/m20pro_sim.launch.py`:
  - `enable_floor_manager:=true`
  - `floor_config:=.../inspection_waypoints.yaml`
  - `initial_floor:=`
- Default `initial_floor` is empty on purpose so the existing sim startup does not unexpectedly reload maps. Use `initial_floor:=F1` only when explicitly desired.
- Extended `src/m20pro_bringup/config/inspection_waypoints.yaml`:
  - `F1` uses `working_1-20260429-162852_edited3`.
  - `F2` uses `working_1-20260429-162852_edited2` as a placeholder.
  - Both floors include `initial_pose`.
- Floor switching command:

```bash
ros2 topic pub --once /m20pro/switch_floor std_msgs/msg/String "{data: F2}"
```

- Common CLI typo:
  - wrong: `std_msgs/msgs/String`
  - correct: `std_msgs/msg/String`
  - If ROS 2 prints `The passed message type is invalid`, first check this `msg` vs `msgs` spelling.

- The manager does:
  1. Resolve floor map path, including `package://...`.
  2. Call `/map_server/load_map` using `nav2_msgs/srv/LoadMap`.
  3. Clear `/global_costmap/clear_entirely_global_costmap` and `/local_costmap/clear_entirely_local_costmap`.
  4. Publish `/initialpose` several times so `sim_bridge` resets pose.
  5. Publish `/m20pro/current_floor`.
- Verification:
  - `python3 -m py_compile src/m20pro_navigation/m20pro_navigation/floor_manager.py` passed.
  - YAML parse check for `inspection_waypoints.yaml` passed.
  - `colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install` passed.
  - Short launch test with `rviz:=false` and initial F1 confirmed:
    - map server loaded map;
    - floor manager loaded F1;
    - costmaps were cleared;
    - `sim_bridge` received `/initialpose` and reset pose.

Sim stair traversal MVP:

- User confirmed manual floor switching works and asked for a stair point where the robot can switch gait, climb/descend, and identify whether the transition is `F+1` or `F-1`.
- Extended `floor_manager.py`:
  - subscribes to `/m20pro/use_stairs` (`std_msgs/msg/String`);
  - accepts commands such as `F+1`, `F-1`, `up`, `down`, or a target floor like `F2`;
  - tracks current floor and robot pose via `/m20pro_tcp_bridge/map_pose`;
  - verifies the robot is near the configured stair entry before starting;
  - publishes gait labels on `/m20pro/gait_command`:
    - `stair_up`
    - `stair_down`
    - `flat`
  - publishes progress/status on `/m20pro/stair_status`;
  - after a simulated traversal delay, calls `/map_server/load_map`, clears costmaps, publishes `/initialpose`, and switches gait back to `flat`.
- Extended `inspection_waypoints.yaml`:
  - `F1.level: 1`
  - `F2.level: 2`
  - `F1.stairs.stair_A_up_to_F2` has an entry pose and target exit.
  - `F2.stairs.stair_A_down_to_F1` has an entry pose and target exit.
- `m20pro_sim.launch.py` now defaults `initial_floor:=F1`, but the manager assumes the current floor without reloading the initial map unless `load_initial_floor` is set true internally.
- Test commands:

```bash
ros2 topic pub --times 3 -r 2 /m20pro/use_stairs std_msgs/msg/String "{data: F+1}"
ros2 topic pub --times 3 -r 2 /m20pro/use_stairs std_msgs/msg/String "{data: F-1}"
```

- Use `--times 3 -r 2` instead of `--once` for these one-shot control topics if the CLI publisher exits before the subscriber matches.
- End-to-end tests passed:
  - `F+1` on F1 near stair entry published `stair_up`, waited, loaded F2, reset pose, then published `flat`.
  - `F-1` on F2 near stair entry published `stair_down`, waited, loaded F1, reset pose, then published `flat`.

Cross-floor goal navigation MVP:

- User wanted the RViz-visible behavior:
  1. Robot starts on F1.
  2. User publishes a target pose on F2.
  3. Robot plans/navigates on F1 to the stair entry.
  4. At the stair entry, the manager switches to stair gait.
  5. The map switches to F2.
  6. Robot pose is reset at the F2 stair exit.
  7. Robot navigates to the original F2 target pose.
- Implemented in `floor_manager.py`.
- New topic:

```bash
/m20pro/floor_goal
```

- Message type:

```bash
geometry_msgs/msg/PoseStamped
```

- The target floor is carried in `header.frame_id`; example:

```bash
ros2 topic pub --times 3 -r 2 /m20pro/floor_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: F2}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

- Added Nav2 `NavigateToPose` action client in `floor_manager.py`.
- Added dependency `action_msgs` in `m20pro_navigation/package.xml`.
- Internal behavior:
  - If target floor equals current floor, send the pose directly to Nav2.
  - If target floor differs, resolve stair route from current floor to target floor.
  - Send Nav2 goal to the stair entry first.
  - On stair-entry goal success, start the stair transition.
  - After map switch and `/initialpose`, wait briefly and send the original floor goal to Nav2.
  - Repeated floor goals are ignored while a cross-floor mission is active, so CLI `--times 3` does not repeatedly preempt Nav2.
- Verification:
  - `python3 -m py_compile src/m20pro_navigation/m20pro_navigation/floor_manager.py` passed.
  - `colcon build --packages-select m20pro_navigation --symlink-install` passed.
  - Runtime test showed:
    - received F2 floor goal;
    - sent Nav2 goal to F1 stair entry;
    - stair-entry navigation reached goal;
    - published `stair_up`;
    - loaded F2 map;
    - reset sim pose at F2 stair exit;
    - published `flat`;
    - sent Nav2 goal to the original F2 target.

How to edit stair points and publish cross-floor goals:

- Stair points are configured in `src/m20pro_bringup/config/inspection_waypoints.yaml`.
- In each floor:
  - `stairs.<name>.entry` is the stair entrance on the current floor. Nav2 will navigate to this pose before stair traversal.
  - `stairs.<name>.target_exit` is the pose where the robot is placed/relocalized after switching to the target floor map.
  - `direction` should be `up` or `down`.
  - `target_floor` is the destination floor.
- Example F1 to F2:
  - `F1.stairs.stair_A_up_to_F2.entry`
  - `F1.stairs.stair_A_up_to_F2.target_exit`
- Example F2 to F1:
  - `F2.stairs.stair_A_down_to_F1.entry`
  - `F2.stairs.stair_A_down_to_F1.target_exit`
- After editing YAML, restart `m20pro_sim.launch.py`; config is loaded at node startup.
- Publish a cross-floor target with:

```bash
ros2 topic pub --times 3 -r 2 /m20pro/floor_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: F2}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

- `header.frame_id` carries the target floor ID (`F1`, `F2`, etc.); the pose coordinates are in that floor's map frame.
- Clarification:
  - `entry` and `target_exit` are not the same field because they belong to different traversal directions and often different floor map frames.
  - For a paired stair:
    - `F1 -> F2 target_exit` should usually correspond to the F2 stair landing.
    - `F2 -> F1 entry` should also be on the F2 stair landing.
    - Their `x/y` may be the same or close, but `yaw` may differ depending on whether the robot is facing out of the stairs or into the stairs.
  - Similarly, `F2 -> F1 target_exit` corresponds to the F1 stair landing, and should be paired with `F1 -> F2 entry`.
  - The current placeholder values in `inspection_waypoints.yaml` are only demo coordinates and may not be physically consistent.

Getting map coordinates from RViz:

- User wanted a practical way to know the coordinates of a point on the map because RViz does not show mouse hover coordinates clearly.
- Added `rviz_default_plugins/PublishPoint` to `src/m20pro_bringup/rviz/m20pro_sim.rviz`.
- The tool publishes clicked map points to `/clicked_point`.
- To get x/y:

```bash
ros2 topic echo /clicked_point
```

- In RViz select the Publish Point tool and click on the map.
- To get x/y/yaw, use RViz `2D Goal Pose` and echo:

```bash
ros2 topic echo /goal_pose
```

- The clicked point is only x/y/z; yaw needs a pose tool because yaw comes from the arrow direction.

Stair point update:

- User provided the F1 upward stair-entry pose:

```yaml
position:
  x: -12.240381240844727
  y: 10.81514835357666
orientation:
  z: -0.023696212124303846
  w: 0.9997192053426602
```

- Converted quaternion to yaw:

```text
yaw = -0.047396860593401584
```

- Updated `src/m20pro_bringup/config/inspection_waypoints.yaml`:
  - `F1.stairs.stair_A_up_to_F2.entry`
  - `x: -12.240381240844727`
  - `y: 10.81514835357666`
  - `yaw: -0.047396860593401584`
- Still needed for a full F1 -> F2 transition:
  - F2 stair-exit / landing pose, i.e. `F1.stairs.stair_A_up_to_F2.target_exit`.
- User clarified that the down-stair point on the same landing should use the same x/y with yaw rotated by 180 degrees.
- For the F1 stair landing:
  - original yaw: `-0.047396860593401584`
  - reversed yaw: `3.0941957929963917`
- Updated `inspection_waypoints.yaml`:
  - `F1.stairs.stair_A_from_F2.exit`
  - `F2.stairs.stair_A_down_to_F1.target_exit`
  - both now use:
    - `x: -12.240381240844727`
    - `y: 10.81514835357666`
    - `yaw: 3.0941957929963917`
- Still needed:
  - actual F2 stair landing coordinate in the F2 map frame.

Stair map / realistic simulation discussion:

- User asked how to model the stair map because instant teleporting from one floor stair entry to another floor exit feels unrealistic.
- Recommendation:
  - Keep floor maps as separate 2D Nav2 maps.
  - Treat stairs as a special transition edge between floor maps.
  - Do not try to make the 2D occupancy grid itself represent a true 3D stair unless using a full physics simulator.
- Three possible fidelity levels:
  1. Current MVP:
     - navigate to stair entry;
     - wait simulated traversal duration;
     - switch map;
     - reset pose at target-floor exit.
     - Good for testing mission logic, but visually feels like teleporting.
  2. Better RViz simulation:
     - add an intermediate `STAIR_A` transition map or transition state;
     - publish a stair marker/path/progress in RViz;
     - optionally interpolate the robot pose along a fake stair corridor for several seconds;
     - then switch to the target floor map.
     - This is the recommended next step.
  3. True physical simulation:
     - use Gazebo/Ignition/Isaac Sim with stair geometry, wheel-leg contact, and gait controller;
     - needs much more accurate robot dynamics and possibly vendor gait/control details.
     - Not necessary for current Nav2/multi-floor logic.
- Best practical next implementation:
  - add a `stair_traversal_mode: visual_transition` or `transition_map` mode to `floor_manager`;
  - publish `/m20pro/stair_status`, `/m20pro/gait_command`, and RViz markers showing progress;
  - optionally switch to a simple artificial stair transition map before switching to F2.

Stair corridor traversal implementation:

- User clarified the desired sim flow:
  1. Robot starts on F1.
  2. User publishes an F2 goal.
  3. Robot navigates on F1 to the stair entry.
  4. Robot switches to stair gait.
  5. Robot continues navigating through a stair corridor drawn into the F1 map.
  6. At the end of that F1 stair corridor, the manager switches to the F2 map and resets pose.
  7. Robot switches back to flat gait and navigates to the F2 goal.
- Implemented optional stair corridor traversal in `floor_manager.py`.
- New optional stair config key:

```yaml
traverse_to:
  x: ...
  y: ...
  yaw: ...
```

- If `traverse_to` exists:
  - after `stair_entry` succeeds, the manager publishes stair gait;
  - sends a Nav2 goal labeled `stair_traverse` to `traverse_to`;
  - only after `stair_traverse` succeeds does it switch maps and reset to `target_exit`.
- If `traverse_to` is omitted:
  - behavior falls back to the previous timed stair traversal.
- Added commented templates for `traverse_to` in `inspection_waypoints.yaml` for both F1->F2 and F2->F1.
- Verification:
  - `python3 -m py_compile src/m20pro_navigation/m20pro_navigation/floor_manager.py` passed.
  - YAML parse check passed.
  - `colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install` passed.

Map editor default map removal:

- User asked to remove the default map from `map_editor`; they want to choose a map manually after opening.
- Updated `src/m20pro_navigation/m20pro_navigation/map_editor.py`:
  - `ros2 run m20pro_navigation map_editor` now opens a file chooser first.
  - If the user cancels the chooser, the app exits cleanly.
  - Passing a map path still works:

```bash
ros2 run m20pro_navigation map_editor /path/to/occ_grid.yaml
```

  - The file chooser starts in the package `maps` directory when available.
- Verification:
  - `python3 -m py_compile src/m20pro_navigation/m20pro_navigation/map_editor.py` passed.
  - `colcon build --packages-select m20pro_navigation --symlink-install` passed.

F1/F2 stair map clarification:

- User created two maps under `m20pro_bringup/maps`:
  - `F1`, with a manually drawn stair corridor from first floor to second floor.
  - `F2`, without a stair corridor to third floor.
- This is acceptable if the current simulation target is only F1 -> F2.
- F2 does not need a F2 -> F3 stair corridor unless testing navigation from F2 to F3.
- For returning F2 -> F1 with a visually continuous stair segment, F2 should eventually include the corresponding down-stair corridor or a configured `traverse_to` in the F2 map.
- For one-way F1 -> F2 testing:
  - F1 needs `entry` and `traverse_to`.
  - F2 needs at least `target_exit` and normal F2 goal poses.
  - `inspection_waypoints.yaml` must point `F1.map_yaml` and `F2.map_yaml` to the new map directories.

Docker deployment discussion:

- User asked whether the whole project can be packaged into Docker and run on the robot.
- Feasible, but it must be built for the robot environment:
  - architecture: `linux/arm64` / aarch64 because M20 Pro 104 is RK3588;
  - OS/ROS: Ubuntu 20.04 + ROS 2 Foxy for compatibility with the robot;
  - do not directly package the desktop Humble/x86_64 environment.
- Recommended runtime style:
  - use `--network host` so ROS 2 DDS can see robot topics and 103/106 network;
  - mount maps/models/logs as volumes;
  - use `--privileged` or specific `--device` mounts only if camera/NPU/GPU/hardware access is needed;
  - run RViz outside the robot container if possible.
- Main risk:
  - Nav2 dependencies differ between Foxy and Humble. Current project has a `nav2_params_foxy.yaml`, and real robot deployment should use Foxy-compatible params/nodes.
  - RK3588 NPU/YOLO/RKNN support inside Docker requires extra device/runtime handling, so the first Docker version should focus on navigation before adding inspection acceleration.
- Suggested rollout:
  1. Build an arm64 Foxy image on a machine with internet.
  2. `docker save` it to a tar file.
  3. Copy tar to 104.
  4. `docker load` on 104.
  5. Run with host network and mounted maps/config/models.

Docker environment clarification:

- User pushed back: the reason to use Docker is precisely that the robot environment is unknown, so "using the dog's environment" sounds backwards.
- Clarification:
  - Docker can and should carry the project's own user-space environment.
  - But it cannot ignore host hardware constraints:
    - CPU architecture must match, likely `arm64/aarch64` on RK3588.
    - The host kernel and hardware drivers still come from the robot.
    - ROS 2 DDS/networking still has to interoperate with robot nodes or vendor TCP interfaces.
  - Therefore the real decision is not "dog environment vs user environment"; it is:
    - build a self-contained image for the robot's architecture;
    - choose whether the image uses Foxy/20.04 for maximum robot compatibility or Humble/22.04 for maximum desktop parity.
- Practical recommendation:
  1. First collect robot facts with a small script:
     - architecture
     - Ubuntu version
     - Docker availability
     - ROS distro
     - DDS implementation
     - available topics/services
  2. Then choose between:
     - conservative image: `arm64 Ubuntu 20.04 + Foxy`;
     - portable own-env image: `arm64 Ubuntu 22.04 + Humble`.
  3. Test the Humble image only if DDS interop and vendor interfaces are confirmed.

Factory rosbag analysis:

- User recorded two rosbags under `bags/bags` while the robot was running factory autonomous navigation, not this project.
- Bags found:
  - `bags/bags/test_bag/rosbag2_2026_05_18-20_13_21`
  - `bags/bags/test_bag2/rosbag2_2026_05_18-20_15_31`
- `test_bag`:
  - size: ~143.7 MiB
  - duration: ~54.28 s
  - messages: 60,130
  - robot odom:
    - start: `x=-2.230 y=0.197 yaw=0.050`
    - end: `x=1.069 y=0.026 yaw=-0.086`
    - approximate traveled distance: ~7.56 m
  - target changes:
    - first target: `x=-0.049 y=0.303 yaw=0.006`
    - second target at +8.8 s: `x=4.432 y=0.178 yaw=0.043`
  - It did not reach the second target before the bag ended; it sat near x ~= 1.07 for much of the recording.
- `test_bag2`:
  - size: ~59.4 MiB
  - duration: ~23.63 s
  - messages: 24,059
  - robot odom:
    - start: `x=-1.879 y=0.315 yaw=-0.099`
    - end: `x=4.384 y=0.093 yaw=-0.022`
    - approximate traveled distance: ~7.87 m
  - target changes:
    - first target: `x=-0.049 y=0.303 yaw=0.006`
    - second target at +2.29 s: `x=4.432 y=0.178 yaw=0.043`
  - This bag reached the final target area.
- Important active factory navigation topics:
  - `/ODOM` (`nav_msgs/Odometry`) about 10 Hz, frame `map`.
  - `/tf` (`tf2_msgs/TFMessage`) about 10 Hz, sample edge `map -> base_link`.
  - `/target_goal` (`geometry_msgs/PoseStamped`) about 10 Hz, frame `map`.
  - `/local_goal` (`geometry_msgs/PoseStamped`) about 10 Hz, frame `map`.
  - `/local_goal_baselink` (`geometry_msgs/PoseStamped`) about 10 Hz, frame `base_link`.
  - `/path_Astar` (`nav_msgs/Path`) about 10 Hz, frame `map`.
  - `/track_path_baselink` (`nav_msgs/Path`) about 10 Hz, frame `base_link`.
  - `/local_path` (`nav_msgs/Path`) about 20 Hz, frame `base_link`.
  - `/local_map` (`nav_msgs/OccupancyGrid`) about 10 Hz.
  - `/grid_map_3d` (`sensor_msgs/PointCloud2`) about 10 Hz, frame `base_link`, fields `x,y,z`.
  - `/NAV_CMD`, `/PLANNER_STATUS`, `/REAL_STEER`, `/MOTION_INFO`, `/LOCATION_STATUS` exist but use custom `drdds` messages.
- `local_map` details:
  - `438 x 514`
  - resolution `0.1`
  - origin `(-17.9, -13.7)`
  - same dimensions/origin as current `occ_grid.yaml`.
  - Looks like a full occupancy map repeatedly published, not a cropped local costmap.
- Notably missing data:
  - `/LIDAR/POINTS`, `/LIDAR/POINTS2`, `/cloud_nav`, `/SEG_CLOUD`, `/GRID_MAP`, `/LIO_ODOM`, `/GAIT` all had 0 messages in these bags.
  - Therefore these bags cannot validate the custom pointcloud-to-LaserScan pipeline or gait switching.
- Local machine cannot deserialize custom messages because packages like `drdds` and `fibocom_msgs` are not installed in this workspace.
- Practical conclusions:
  - The factory navigation stack publishes useful map-frame pose and paths already.
  - On real robot, subscribing to `/ODOM` or `/tf map->base_link` may be a simpler localization source than rebuilding localization on 104.
  - Factory planner outputs are visible via `/path_Astar`, `/track_path_baselink`, `/local_path`, `/target_goal`, and `/local_goal`.
  - To deeply analyze `/NAV_CMD` and planner status, copy/install custom message packages from the robot.
  - To test this project's perception pipeline, re-record bags with actual `/LIDAR/POINTS` or camera/image topics containing data.

Factory rosbag context clarification:

- User clarified how the bags were recorded:
  - Robot was in factory remote-controller navigation mode.
  - User added two task points from the controller and started autonomous navigation.
  - `test_bag`: failed to complete obstacle avoidance and stopped midway.
  - `test_bag2`: lower difficulty route completed.
  - Recording command was `ros2 bag record -a`.
- Interpretation update:
  - Zero-message topics in metadata should not be interpreted as "the factory stack does not have these topics."
  - It means rosbag2 discovered those topics but recorded no messages during the recording window.
  - Possible reasons:
    - factory navigation mode did not publish those topics;
    - publisher existed but no sensor/module data was active;
    - QoS mismatch or bare DDS bridge behavior prevented rosbag2 from receiving messages;
    - raw pointcloud/image streams may be disabled while processed maps are published.
- Need direct real-robot checks:

```bash
ros2 topic hz /LIDAR/POINTS
ros2 topic hz /cloud_nav
ros2 topic hz /grid_map_3d
ros2 topic hz /local_map
ros2 topic hz /ODOM
ros2 topic info -v /LIDAR/POINTS
ros2 topic info -v /cloud_nav
ros2 topic info -v /grid_map_3d
```

- If topic list shows a publisher but `hz` receives nothing, test QoS variants such as best-effort echo/record.

True robot migration feasibility:

- Based on current code and factory bags, migrating the project to the real robot is feasible, but should be staged.
- Strong feasibility:
  - high-level mission/floor manager;
  - map switching logic;
  - inspection/YOLO pipeline once camera topic is confirmed;
  - using factory `/ODOM` or `/tf map->base_link` as localization input;
  - observing factory path/goal topics for diagnostics.
- Medium/risky feasibility:
  - running our own full Nav2 stack on 104 and commanding the robot directly;
  - replacing factory obstacle avoidance;
  - depending on raw `/LIDAR/POINTS` or `/cloud_nav`, because the bags did not contain actual messages.
- Main blockers to verify:
  - whether 104 has Nav2/Foxy dependencies or can install them;
  - whether our bridge can command the robot safely (`cmd_vel`, TCP, `/NAV_CMD`, or vendor API);
  - whether raw pointcloud/camera topics are accessible with compatible QoS;
  - whether `drdds`/`fibocom_msgs` custom message packages can be copied for introspection.
- Recommended real-machine deployment plan:
  1. Run `tools/collect_ros_snapshot.sh` on 104/106.
  2. Copy custom message packages if available.
  3. Build this workspace on 104 from source, not copying x86 `build/install`.
  4. First run read-only nodes: topic bridge/monitor, floor manager status, inspection subscriber.
  5. Then test command output in a controlled area.
  6. Only after that run our Nav2 or mission manager in command mode.

## 2026-05-19 F2 stair map drawing suggestion

- Added a visual reference:

```text
docs/f2_stair_layout_suggestion.svg
docs/f2_stair_pgm_style_cn.svg
```

- Core idea:
  - Do not draw F2 upstairs and downstairs as one merged free-space blob.
  - Nav2 only sees free/occupied cells; it does not understand "up" and "down" semantics from the occupancy grid.
  - F2 should use a small shared platform plus separated stair corridors.
  - Draw wall/keepout cells between the down-to-F1 lane and the up-to-F3 lane.
  - In YAML, keep routes separate:
    - `stair_A_down_to_F1`
    - future `stair_B_up_to_F3`
- Current sim assumption:
  - F1 and F2 can share the same coordinate system for quick testing.
  - For a more realistic F2 map, keep `entry`, `traverse_to`, and `target_exit` per route, instead of letting the global planner decide inside a large open stair area.

## 2026-05-19 Three-floor sim logic

- `inspection_waypoints.yaml` now defines `F1`, `F2`, and `F3`.
- Map paths:
  - `F1`: `package://m20pro_bringup/maps/F1/occ_grid.yaml`
  - `F2`: `package://m20pro_bringup/maps/F2/occ_grid.yaml`
  - `F3`: `package://m20pro_bringup/maps/F3/occ_grid.yaml`
- Current temporary sim assumption:
  - The three maps are copied from the same base map.
  - Therefore the same stair endpoints are reused for every adjacent floor transition.
- Stair routes:
  - `F1 -> F2`: `stair_A_up_to_F2`
  - `F2 -> F1`: `stair_A_down_to_F1`
  - `F2 -> F3`: `stair_A_up_to_F3`
  - `F3 -> F2`: `stair_A_down_to_F2`
- `floor_manager.py` was upgraded from direct one-step floor switching to adjacent-floor routing:
  - `F1 -> F3` is automatically executed as `F1 -> F2 -> F3`.
  - `F3 -> F1` is automatically executed as `F3 -> F2 -> F1`.
  - After each map switch, it checks whether the final target floor has been reached. If not, it starts the next stair leg instead of sending the final Nav2 goal too early.
- Build verified:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install
```

## 2026-05-20 Update: Factory lidar topic availability

- User checked the real robot 104 host with `ros2 topic list`.
- `/LIDAR/POINTS` and `/cloud_nav` do not appear on 104 at all in the current factory state.
- Earlier, 106 showed `/LIDAR/POINTS` only with `Publisher count: 0` and one subscriber, so that was not a usable point cloud data source.
- Current conclusion: these two names should not be treated as confirmed factory-published topics on the real robot.
- Next real-robot investigation should search all 104/106 topics by type/name and identify the actual point cloud or local perception topic, if one is exposed.

Update 2026-06-04:

- The above conclusion is outdated.
- User confirmed by direct testing that `/LIDAR/POINTS` is published normally.
- Handheld controller mode is not the key variable:
  - Factory normal mode does not stop `/LIDAR/POINTS`.
  - Factory navigation mode does not stop `/LIDAR/POINTS`.
- Future lidar checks should first use the known working sequence:

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
ros2 topic echo /LIDAR/POINTS --no-arr
```

- Do not treat handheld normal/navigation mode as the primary cause when `/LIDAR/POINTS` is missing. Prioritize ROS environment, DDS/multicast relay, host startup order, and whether the command sequence changed.

## 2026-06-04 Web access deployment decision

- Current stage:
  - Run the web dashboard on 104.
  - Notebook/handheld connects to the robot WiFi or robot internal network.
  - Browser opens `http://10.21.31.104:8080`.
- This keeps the on-site access path simple and avoids depending on public internet during debugging.
- Later stage:
  - Add an industrial router so the robot network has stable internet access.
  - Keep the same web dashboard running on 104.
  - Use VPN, intranet tunneling, or the client's server reverse proxy to access `104:8080` remotely.
  - Do not expose raw `8080` directly to the public internet.

## 2026-06-05 Factory performance/demo feature check

- Read-only scan was performed on 103/104/106 and local M20 manuals.
- No obvious factory "dance/show/performance/demo choreography" interface was found.
- `performance.service` on 104 is not a robot show feature; it only sets CPU governors to `performance` and forces HDMI status on.
- Factory ROS/DDS interfaces found around motion are normal control/status interfaces:
  - `/GAIT`
  - `/NAV_GAIT`
  - `/CHARGE_GAIT`
  - `/STEER`, `/REAL_STEER`, `/HANDLE_STEER`
  - `/NAV_CMD`, `/NAV_STATUS`
  - `/MOTION_INFO`, `/MOTION_STATE`, `/MOTION_STATUS`
  - LED services/topics
- M20 manuals mention motion mode, gait switching, navigation, and lights, but no dedicated show/dance/demo performance function.
- 103 has `rl_deploy`, `basic_server`, `height_map_nav`, and perception services; these look like locomotion/perception/runtime components, not user-facing performance scripts.
- Remaining uncertainty: a hidden feature could exist inside the handheld APK, but it is not exposed clearly in the host-side ROS topics/services/scripts checked here.

## 2026-05-19 RViz floor goal relay

- Added `m20pro_navigation/floor_goal_relay.py`.
- Registered console script:

```text
ros2 run m20pro_navigation floor_goal_relay
```

- `m20pro_sim.launch.py` now starts `m20pro_floor_goal_relay` when floor manager is enabled.
- RViz config now has three floor-aware goal topics:

```text
Goal F1 -> /m20pro/rviz_goal_f1
Goal F2 -> /m20pro/rviz_goal_f2
Goal F3 -> /m20pro/rviz_goal_f3
```

- Relay output:

```text
/m20pro/floor_goal
```

- Behavior:
  - Select the matching RViz goal tool for the destination floor.
  - Click the map with `2D Goal Pose`.
  - The relay sets `PoseStamped.header.frame_id` to `F1`, `F2`, or `F3`.
  - `floor_manager` receives it as a cross-floor target and handles stairs/map switching.
- Build verified:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select m20pro_navigation m20pro_bringup --symlink-install
```

## 2026-06-08 Task2 bag and Task3 real verification

- User recorded factory navigation bag on 104:

```text
/home/user/bags/rosbag2_2026_06_08_task2
```

- Bag summary:
  - duration: 34.198 s;
  - size: about 1.2 GiB;
  - total messages: 21569;
  - `/LIDAR/POINTS`: 347 messages, about 10 Hz;
  - `/ODOM`: 343 messages, about 10 Hz;
  - `/IMU`: 4172 messages, about 122 Hz;
  - `/LIDAR/IMU201`: 4060 messages;
  - `/LIDAR/IMU202`: 4059 messages;
  - `/tf`: 612 messages;
  - `/traversal_cost`, `/impassable_area`, `/accumulate_cloud/cloud_gravity`: 265 messages each.
- Interpretation:
  - factory navigation data chain is alive;
  - raw lidar, odom, IMU and factory navigation perception outputs are present;
  - this bag is useful as baseline reference for our real tests.

## 2026-06-08 Task3 fixes

- Frontend point saving problem was fixed in `web_dashboard_node.py`:
  - saving now validates map and XY;
  - if no archived map is selected but live `/map` exists, annotations are saved under `map_id=live_map`;
  - backend accepts `live_map` as a special temporary map id;
  - verified on 104 by POSTing and deleting an API test annotation.
- Real startup was wrapped by a new script:

```bash
ros2 run m20pro_bringup m20pro_real_full.sh shadow
ros2 run m20pro_bringup m20pro_real_full.sh move
```

- `shadow` keeps `enable_axis_command:=false`; `move` enables motion command output and should only be used after map/pose/costmap checks pass.
- `system_check` was relaxed for real mode:
  - it no longer requires direct raw cloud subscription in the check node;
  - it no longer blocks on per-message checks that can be misleading under factory DDS timing;
  - manual topic checks and rosbag review remain the authoritative field checks.
- Added project-local FastDDS profile:

```text
src/m20pro_bringup/config/m20pro_fastdds_udp.xml
```

- This profile keeps UDP + SHM but reduces SHM segment size from the factory 500 MB to 64 MB for our nodes.
- Reason:
  - factory `/opt/robot/fastdds.xml` can push `/dev/shm` to about 95% when the full real stack starts;
  - pure UDP avoids `/dev/shm` growth but failed to receive `/LIDAR/POINTS` samples reliably;
  - lightweight UDP+SHM received `/LIDAR/POINTS`, allowed `pointcloud_fusion` to publish `/scan`, and kept `/dev/shm` around 48% during verification.

## 2026-06-08 verified real shadow status

- 104 real shadow launch verified with:

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
cd /home/user/m20pro_ros2_ws
source install/setup.bash
ros2 run m20pro_bringup m20pro_real_full.sh shadow
```

- Verified results:
  - Web frontend reachable at `http://10.21.31.104:8080`;
  - frontend API returns live pose and floor;
  - `/LIDAR/POINTS` echoes live `PointCloud2` from `lidar_link`;
  - `m20pro_pointcloud_fusion` subscribes to `/LIDAR/POINTS` with reliable QoS;
  - `/scan` echoes live `LaserScan` in `m20pro_base_link`;
  - local/global costmaps subscribe to `/scan`;
  - Nav2 managed nodes become active.
- Current warning is meaningful, not a lidar failure:
  - logs show `Robot is out of bounds of the costmap` and sensor origin outside map bounds;
  - this means the current robot pose and the loaded F20 map do not match the physical test location;
  - do not run motion test until correct map/relocalization is confirmed.
- Desktop field script regenerated:

```text
/home/fabu/桌面/脚本.docx
```

- The script is intentionally simple: tasks, commands, when to Ctrl+C, and the frontend point/task flow.

## 2026-06-09 104 `/LIDAR/POINTS` visible but no echo data

- Symptom on 104:
  - `ros2 topic list` and `ros2 topic info -v /LIDAR/POINTS` could see two reliable `PointCloud2` publishers;
  - `ros2 topic echo /LIDAR/POINTS --no-arr` and `ros2 topic hz /LIDAR/POINTS` did not receive samples.
- Confirmed not caused by our M20Pro stack:
  - no `m20pro_real_full`, `m20pro.launch.py`, `m20pro_web_dashboard`, `m20pro_pointcloud_fusion`, Nav2, or system check processes were running on 104;
  - 106 could echo `/LIDAR/POINTS` locally with live frames.
- Key finding:
  - 106 lidar data was healthy locally;
  - 104 discovery worked but data forwarding did not;
  - 106 `/usr/bin/multicast.py` forwards multicast from `10.21.33.106` to `10.21.31.106`;
  - restarting `multicast-relay.service` on 106 restored 104 echo.
- Recovery command used on 106:

```bash
ssh user@10.21.31.106
source /opt/robot/scripts/setup_ros2.sh
su
systemctl restart multicast-relay.service
systemctl status multicast-relay.service --no-pager -l
```

- Recheck on 104:

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
ros2 daemon stop
ros2 daemon start
ros2 topic info -v /LIDAR/POINTS
ros2 topic echo /LIDAR/POINTS --no-arr
```

- Result after restart:
  - 104 echoed live `/LIDAR/POINTS` frames again;
  - sample frame had `frame_id: lidar_link`, `height: 1`, width around 45k to 66k points.
- Practical rule:
  - if 104 can see `/LIDAR/POINTS` publishers but echo/hz has no data while 106 echo is normal, first restart 106 `multicast-relay.service`;
  - do not change M20Pro project code or Nav2 parameters for this symptom.

## 2026-06-09 Task3 shadow test bag review

- Reviewed bag:

```text
/home/user/bags/m20_shadow_20260609_144525
```

- Bag summary:
  - duration: 117.612 s;
  - size: about 3.2 GiB;
  - total messages: 59624.
- Main live chains were healthy:
  - `/LIDAR/POINTS`: 1154 messages, about 10.00 Hz;
  - `/scan`: 1168 messages, about 10.00 Hz;
  - `/ODOM`: 1168 messages, about 10.00 Hz;
  - `/odom`: 583 messages, about 5.01 Hz;
  - `/IMU`: 14928 messages, about 127.71 Hz;
  - `/m20pro_tcp_bridge/map_pose`: 584 messages, about 5.00 Hz.
- Scan content:
  - first scan frame: `m20pro_base_link`, 361 bins, 203 finite bins, 67 bins within 2 m, min range about 1.15 m;
  - last scan frame: 191 finite bins, min range about 1.16 m.
- Costmap content:
  - `/local_costmap/costmap`: 1 message, 100 x 100 cells, 0.05 m resolution, 2134 marked/inflated cells, 63 lethal cells;
  - `/global_costmap/costmap`: 1 message, 436 x 515 cells, 0.10 m resolution, 34962 marked/inflated cells, 7564 lethal cells.
- Why `ros2 topic echo /local_costmap/costmap --no-arr` may show no continuous output:
  - real Nav2 config now has `always_send_full_costmap: false` for performance;
  - therefore full costmap may only publish initial/full snapshots, while continuous changes should be checked via costmap update topics or by temporarily enabling full costmap publishing for debug.
- Shadow-mode safety:
  - `/cmd_vel`: 0 messages, expected for shadow mode because motion command output was not enabled.
- Interpretation:
  - this Task3 run is useful and basically successful as a shadow validation;
  - raw point cloud, generated scan, odom, IMU, pose bridge and costmap content are all present;
  - frontend point marking was reported normal by field test;
  - next meaningful step is a controlled `move` test only after confirming correct map/relocalization and a safe route.

## 2026-06-09 Task4 preparation check

- Desktop script updated:

```text
/home/fabu/桌面/脚本.docx
```

- Main changes:
  - Task4 now explicitly treats `/local_costmap/costmap` as a low-frequency/full snapshot topic because real config has `always_send_full_costmap: false`;
  - Task4 startup checks focus on `/LIDAR/POINTS`, `/scan`, lifecycle active states, and `/cmd_vel` after a task starts;
  - the fallback `/m20pro/goal_command` command is documented as an absolute map-coordinate command, not a relative “move forward 1 m” command;
  - added the 106 `multicast-relay.service` restart recovery step for the “topic visible but echo has no samples” failure mode.
- Recorder script updated:

```text
src/m20pro_bringup/scripts/m20pro_record_real.sh
```

- It now records:
  - `/local_costmap/costmap_updates`;
  - `/global_costmap/costmap_updates`;
  - plus existing `/cmd_vel`, `/m20pro/floor_goal`, raw lidar, scan, odom, pose and status topics.
- Verified on 104:
  - `m20pro_bringup` builds successfully;
  - `m20pro_real_full.sh move` still maps to `enable_axis_command:=true`;
  - `m20pro_real_full.sh shadow` still maps to `enable_axis_command:=false`;
  - `/LIDAR/POINTS` can echo live frames from 104 using the known-good `source -> su -> source install` sequence.
- Task4 go/no-go:
  - proceed only if map and robot pose visually match;
  - do not start Task4 if `Robot is out of bounds of the costmap` appears;
  - use a web-clicked target 1 to 2 m away, not a hand-written coordinate guessed from memory;
  - keep the controller/operator beside the robot for immediate stop.

## 2026-06-09 Web task stop, field scripts, and test script update

- Web dashboard task execution was hardened:
  - added a visible `停止当前任务` button in the task panel;
  - added backend `POST /api/tasks/stop`;
  - stopping clears `active_task`, marks the task as `stopped`, publishes `/m20pro/stop_task`, and sends one zero `/cmd_vel`;
  - `floor_manager` subscribes `/m20pro/stop_task`, cancels the active Nav2 goal, clears floor mission state, and handles the race where a goal is accepted just after stop was requested;
  - starting a second task while one is running is rejected;
  - switching maps while a task is running is rejected;
  - deleting a point currently used by the active task is rejected;
  - starting a task checks missing point ids and map mismatch before publishing goals;
  - web startup clears stale `running` task state so a restarted web node cannot resume old goals silently.
- Field scripts were reorganized:
  - root `scripts/` is now the human-facing script folder;
  - `src/m20pro_bringup/scripts/` only keeps ROS package internal scripts (`m20pro_real_full.sh`, `m20pro_real_nav.sh`, `m20pro_real_web.sh`, `m20pro_record_real.sh`);
  - duplicated `104_*.sh` under `src/m20pro_bringup/scripts/` were removed to avoid confusion;
  - `.gitignore` no longer ignores root `/scripts/`, so the field scripts can be committed to GitHub.
- Root field scripts now include:

```text
scripts/104_check_lidar.sh
scripts/104_record_bag.sh
scripts/104_start_real_move.sh
scripts/104_start_real_shadow.sh
scripts/104_start_web.sh
scripts/104_status.sh
scripts/104_stop_real.sh
scripts/104_stop_web.sh
scripts/local_pull_bags.sh
scripts/README.md
```

- Desktop field script regenerated:

```text
/home/fabu/桌面/脚本.docx
```

  - changed old task 5/6 into:
    - task 5: long-distance + obstacle avoidance real test;
    - task 6: cross-floor real test;
  - bag pullback and preflight checks are now appendices;
  - multi-floor frontend behavior is documented: the web UI displays one floor map at a time, point annotations carry floor ids, and tasks publish `/m20pro/floor_goal` for `floor_manager` to handle floor transitions.
- Verification:
  - local `m20pro_bringup` build passed after script cleanup;
  - 104 `/home/user/m20pro_ros2_ws` synced with root `scripts/` and cleaned package scripts;
  - 104 `m20pro_bringup` build passed;
  - 104 now shows root `scripts/104_*.sh`, while package and install script paths only show the internal `m20pro_*.sh` scripts.

## 2026-06-09 Inspection waypoint semantics update

- Updated the web task model so inspection is no longer treated as only a sequence of x/y path points.
- Web annotations now store:
  - `pose.x/y/z/yaw`, with yaw in radians;
  - `manual_point_type`: `transition`, `task`, or `charge`;
  - `dwell_s`: stop duration after reaching the point;
  - `vendor_navigation`: M20 developer-manual Type=1003 fields `Value/MapID/PointInfo/Gait/Speed/Manner/ObsMode/NavMode`.
- Developer manual mapping used by the project:
  - transition point: `PointInfo=0`;
  - task point: `PointInfo=1`;
  - charge point: `PointInfo=3`;
  - default flat agile gait: `Gait=12`;
  - low speed: `Speed=1`;
  - forward walking: `Manner=0`;
  - obstacle stop/avoidance enabled: `ObsMode=0`;
  - autonomous navigation: `NavMode=1`; transition points default to straight navigation `NavMode=0`.
- Web task execution now publishes `/m20pro/active_waypoint` as JSON whenever a waypoint is being navigated to or dwelled at.
- Web task execution now honors `dwell_s` before advancing to the next point.
- Charge points are rejected unless they are the final waypoint in a task, because the manual says the robot enters and remains in charging after reaching a charge point.
- `inspection_waypoints.yaml` was updated to make sample waypoints explicit about point type, dwell time, yaw, and vendor navigation fields.
- Bag recording script now records `/m20pro/active_waypoint` for field-test replay and debugging.

## 2026-06-10 Web map selector and Chinese UI cleanup

- Web visible labels were cleaned up so the new waypoint fields no longer mix English UI text into the Chinese operation console:
  - `Yaw(rad)` -> `朝向角(rad)`;
  - `Gait/Speed/Manner/ObsMode/NavMode` -> `步态/速度/行走方式/停避障/导航方式`;
  - those manual fields now use dropdowns with the numeric values still sent unchanged to the backend.
- Web map selection now merges two map sources:
  - project built-in maps from `src/m20pro_bringup/config/map_manifest.yaml`;
  - maps pulled from 106 and stored under the web archive directory.
- Built-in map entries now appear as `F19/F20/F21` in the frontend map selector even when `~/.m20pro_web/maps.json` has no imported map records.
- Current built-in map truth:
  - repository has `src/m20pro_bringup/maps/F19`, `F20`, and `F21`;
  - `map_manifest.yaml` explicitly notes that F19 and F21 currently reuse the edited F20 map product and should be replaced by per-floor real maps before delivery.
- Frontend launch files now pass `map_manifest` into `web_dashboard`, including standalone, sim, and real launches.
- Clarified the 127/104 access difference:
  - `127.0.0.1:8080` is only local self-test on the machine running the web node;
  - field access to the 104-hosted dashboard should use `http://10.21.31.104:8080`.

## 2026-06-10 Current handoff snapshot

- Latest GitHub commit on `main`:

```text
988217d Improve web map selector and Chinese UI
```

- Local workspace status after push:
  - repository worktree was clean;
  - no local frontend process was left occupying port `8080`;
  - local validation had already passed for build, `/healthz`, `/api/maps`, and loading `builtin_F20`.
- Important frontend access rule:
  - if the web node is launched on the current development PC, `http://127.0.0.1:8080` is only for that PC itself;
  - if the web node is launched on 104, field devices should open `http://10.21.31.104:8080`;
  - root script `scripts/104_start_web.sh` now explicitly launches with `host:=0.0.0.0`.
- Current frontend map truth:
  - map dropdown should show project built-in `builtin_F19`, `builtin_F20`, and `builtin_F21`;
  - those come from `src/m20pro_bringup/config/map_manifest.yaml`;
  - `F19` and `F21` currently reuse the edited F20 map product and are placeholders until each floor has its own real 106 map.
- If asked to "拉起前端":
  - for local screenshot/testing on this PC:

```bash
source install/setup.bash
ros2 launch m20pro_bringup m20pro_web_dashboard.launch.py host:=127.0.0.1 port:=8080 enable_camera_proxy:=false
```

  - for field use on 104:

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
cd /home/user/m20pro_ros2_ws
source install/setup.bash
./scripts/104_start_web.sh
```

  - then open `http://10.21.31.104:8080` from a laptop/handheld connected to the robot network.

## 2026-06-10 Field script Word simplified for remaining real tests

- Desktop test script was rewritten:

```text
/home/fabu/桌面/脚本.docx
```

- Removed old task set from the Word script:
  - factory normal-mode bag recording;
  - factory navigation-mode bag recording/relocalization;
  - our real shadow-mode validation.
- New script keeps only the remaining real navigation work:
  - task 1: start real move system and record bag;
  - task 2: same-floor real navigation continuous test, combining short-distance verification, long-distance navigation, and obstacle avoidance;
  - task 3: cross-floor real navigation.
- Verified the regenerated docx text:
  - old titles `原厂常规模式录包`, `原厂导航模式录包`, and `我们的 real 影子导航测试` are no longer present;
  - new tasks are numbered 1/2/3 as above.

## 2026-06-10 104 cleanup before field test

- User confirmed they can manually SSH into 104 and see `/LIDAR/POINTS`; no DDS/FastDDS, multicast, or point-cloud configuration was changed.
- Cleaned coworker DDDMR deployment leftovers from 104:
  - `/home/user/m20_deploy/dddmr_code.tar.gz`;
  - `/home/user/m20_deploy/dddmr_arm64_rk3588.tar.gz`;
  - `/home/user/m20_deploy/start_dddmr_container.sh`;
  - `/home/user/m20_deploy/M20_DDDMR_实机部署完整操作指南.md`.
- 104 bag directory currently keeps only:
  - `/home/user/bags/m20_shadow_20260609_144525`.
- 104 free space after cleanup:
  - `/home/user` on `overlayroot`: `6.5G` available, `65%` used.

## 2026-06-11 Patrol point naming and payload check

- Meeting decision: patrol points must carry a human-readable name so inspection results can say which area/room they came from.
- Web point marking now saves explicit inspection semantics:
  - `label`: point name shown to operators and reports;
  - `area`: site area such as east zone, core tube, sample section;
  - `room`: room or inspected component/location;
  - `result_file_prefix`: result filename prefix for Onrol lidar, YOLO, and later report files.
- If the operator leaves `result_file_prefix` empty, the web backend generates one from `floor_area_room_label`.
- `/m20pro/active_waypoint` now publishes those fields inside `waypoint`, so the Onrol lidar detection process should name result files from:

```text
waypoint.result_file_prefix
```

- `inspection_waypoints.yaml` example task points were updated with Chinese labels plus `area`, `room`, and `result_file_prefix`.
- `config_audit_node` now warns if a task waypoint has no label/name, area/region, or room/place.
- Payload/负重 check:
  - official feedback: M20Pro payload handling is adaptive, so there is no expected manual payload value to configure in our stack;
  - this repository has no runtime payload/负重 compensation setting;
  - URDF has static link masses only, not a real-robot load setting;
  - 104 currently exposes `/EXT_LOAD/POWER` and `/EXT_LOAD/CUR` services with type `drdds/srv/StdSrvInt32`;
  - the M20 developer manual has `MotionStatus.Payload`, but marks it as an invalid parameter;
  - the manual also has `LoadPower`, which belongs to device enable/power status and should be treated as external load power, not robot weight compensation.
- Do not call `/EXT_LOAD/*` casually during navigation tests; it likely affects external load power rather than navigation payload tuning.
- Validation:
  - `python3 -m py_compile src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py src/m20pro_navigation/m20pro_navigation/config_audit_node.py`;
  - YAML safe-load and task waypoint semantic assertion for `inspection_waypoints.yaml`;
  - `colcon build --packages-select m20pro_cloud_bridge m20pro_navigation m20pro_bringup --symlink-install`.

## 2026-06-11 Real short-distance test no-motion diagnosis

- User reported: after setting points in the web dashboard and clicking start task during a short-distance real test, the robot did not move.
- 104 was full at first:
  - `/home/user` on `overlayroot` was `100%` used;
  - new bag `/home/user/bags/m20_real_nav_20260611_153008` was about `6.5G`;
  - old shadow bag `/home/user/bags/m20_shadow_20260609_144525` was about `3.2G`;
  - `/home/user/m20_deploy.zip` was about `2.2G`.
- Cleanup performed on 104:
  - deleted `/home/user/m20_deploy.zip`;
  - deleted old `/home/user/bags/m20_shadow_20260609_144525`;
  - kept today's `/home/user/bags/m20_real_nav_20260611_153008`;
  - free space recovered to about `5.2G`, `72%` used.
- Web task state after launch shutdown was stale:
  - `settings.json` still had `active_task.status=running`;
  - `tasks.json` still marked the task as `running`;
  - cleared `active_task` and marked that task `stopped`, without changing maps, points, DDS, or factory services.
- Bag and root ROS logs show the web frontend did send the task:
  - `/m20pro/floor_goal`: 2 messages;
  - `/m20pro/active_waypoint`: 2 messages;
  - `/plan`: 13 messages;
  - `/cmd_vel`: 244 messages.
- Concrete values from the bag:
  - target floor/pose: `F20`, `x=3.935`, `y=0.210`, `yaw=0.0`;
  - initial `/cmd_vel` was approximately `linear.x=0.55`, `angular.z=-0.041`.
- Root launch log proves the key cause:

```text
m20pro_tcp_bridge: M20 TCP bridge ready; target 103 host is 10.21.31.103:30001; shadow mode; axis command disabled
```

- Therefore this run was not actually sending `/cmd_vel` to the 103 body controller. Nav2 planned and generated velocity commands, but `tcp_bridge` was in shadow mode, so the robot would not move.
- Nav2 also reported controller progress failures:
  - `Failed to make progress`;
  - `Nav2 goal floor_goal finished with status 6`;
  - this is expected if the controller command is not being applied to the robot.
- The real launch was stopped by `Ctrl+C` around `2026-06-11 15:32:32`, so no real/web/Nav2 process was left running afterward.
- `scripts/104_status.sh` was enhanced and synced to 104:
  - prints 104 disk usage;
  - prints active web task if the web server is running;
  - prints the latest tcp bridge motion mode from logs;
  - run it after `su` to read `/root/.ros/log` and confirm whether the latest launch says `axis command enabled` or `shadow mode; axis command disabled`.
- Next real move test must confirm the root log contains:

```text
axis command enabled
```

- If it still says `shadow mode; axis command disabled`, stop and do not expect the robot to move.

## 2026-06-11 Web waypoint yaw marking improved

- User pointed out that manually entering waypoint yaw in the web dashboard is awkward during field marking.
- Web map marking now works closer to RViz `2D Goal Pose`:
  - press on the map to choose waypoint `x/y`;
  - drag toward the direction the robot should face at that waypoint;
  - release to fill `x/y/yaw` automatically.
- A plain click still updates `x/y` and keeps the current yaw value.
- Saved annotations are now drawn as small arrows instead of only circles, so waypoint heading can be checked visually before starting a task.
- Manual `x/y` and `yaw` inputs remain available for precise values copied from bags/RViz.

## 2026-06-11 Field script updated with 106 localization step

- `/home/fabu/桌面/脚本.docx` was regenerated as a small plain Word document.
- Added a mandatory pre-test 106/NOS localization check before 104 real navigation:
  - SSH to 106;
  - `su`;
  - `source /opt/ros/foxy/setup.bash`;
  - `export XAUTHORITY=/home/user/.Xauthority`;
  - `rviz2`;
  - open `/opt/robot/share/localization/conf/localization.rviz`;
  - use RViz `2D Pose Estimate` to align live pointcloud with the active map if needed.
- The script now explicitly says not to start real navigation if 106 localization, active map, or 104 web robot pose is not aligned.
- Updated web marking instructions in the script to use drag-arrow marking for `x/y/yaw`, matching the new web dashboard behavior.

## 2026-06-11 Field script localization policy simplified

- User pointed out that going to 106 for relocalization before every web task is too cumbersome.
- Correct field policy:
  - at the start of a test session, first check localization from the 104 web dashboard;
  - if the robot pose, map, pointcloud/scan, and costmap look aligned, do not open 106 RViz;
  - only go to 106 RViz and use `2D Pose Estimate` when localization is visibly wrong, the active map changed, the robot was moved manually, the robot restarted and pose is wrong, cross-floor pose is wrong, or the robot appears outside the map/costmap.
- `/home/fabu/桌面/脚本.docx` was regenerated with this lighter policy:
  - fast web-side localization check first;
  - 106 relocalization only as an exception path;
  - same-floor short/long/obstacle tests do not require relocalization between tasks when pose remains aligned.

## 2026-06-11 Web relocalization added

- User asked whether startup/relocalization can be done directly from the web dashboard instead of opening RViz on 106.
- Implemented a new `定位` tab in the web dashboard:
  - drag an arrow on the map at the robot's real position;
  - arrow direction is the robot's current heading;
  - click `执行重定位`.
- Web backend now publishes `geometry_msgs/msg/PoseWithCovarianceStamped` on `/initialpose`.
- Existing `m20pro_tcp_bridge` already subscribes to `/initialpose` and forwards it to the M20 Pro vendor localization reset API:
  - Type `2101`;
  - Command `1`;
  - fields `PosX`, `PosY`, `PosZ`, `Yaw`.
- The web dashboard also subscribes to `/m20pro_tcp_bridge/relocalization_result`, so the `定位` tab can show whether the vendor reset was accepted.
- Safety behavior:
  - web relocalization is rejected while a web task is running;
  - stop the task first, then relocalize.
- Launch files now expose and pass:
  - `initialpose_topic`;
  - `relocalization_result_topic`.
- `/home/fabu/桌面/脚本.docx` was regenerated:
  - fast web pose check first;
  - web relocalization as the normal correction path;
  - 106 RViz `2D Pose Estimate` only as fallback if web relocalization or the web frontend is unavailable.

## 2026-06-11 Web relocalization field failure diagnosis

- User reported after field testing:
  - the web page did not show the relocalization button;
  - the web page could show robot position/heading, but heading looked reversed;
  - 106 RViz fallback showed an `ODOM` error.
- Root cause of the missing web button:
  - local repo already had the `定位` tab and `执行重定位` button;
  - 104 actual runtime copy was still an older deployed copy from `2026-06-10`;
  - 104 port `8080` was not running at the first check.
- Fix applied:
  - synced updated `web_dashboard_node.py`, launch files, and `tcp_bridge_node.py` to `/home/user/m20pro_ros2_ws` on 104;
  - rebuilt on 104 with Foxy;
  - verified `http://10.21.31.104:8080/` served the new page containing:
    - `data-tab="localize"`;
    - `sendInitialPoseBtn`;
    - `执行重定位`.
- Web heading display:
  - added `robot_pose_display_yaw_offset_rad`;
  - earlier temporary handling set the real/web default to `pi` for the blue robot arrow and dashboard heading display;
  - this was later withdrawn in the next section because heading mismatch must be diagnosed from localization/map/source yaw/frontend drawing instead of guessed by a 180-degree display offset;
  - saved waypoint yaw, task yaw, and web relocalization yaw were not modified by this display offset.
- 106 RViz `ODOM` error / invalid localization finding:
  - during failure, 104 `/ODOM` contained invalid values such as `x=.inf`, `y=-.inf`;
  - direct 103 TCP `1007/2` map pose query returned truncated JSON while localization was lost;
  - 103 TCP `2002/1` reported `Location=1` at that time.
- After a relocalization/reset attempt:
  - 103 TCP `2002/1` returned `Location=0`;
  - 103 TCP `1007/2` returned valid `PosX/PosY/PosZ/Yaw`;
  - `m20pro_tcp_bridge` logged repeated vendor relocalization success messages for `2101/1`.
- Robustness added:
  - `tcp_bridge` now rejects `nan/inf` map poses and does not publish bad `/odom`, TF, or `/m20pro_tcp_bridge/map_pose`;
  - `tcp_bridge` publishes `localization_ok=false` when map pose query fails or returns no valid pose;
  - web dashboard ignores non-finite pose values instead of showing invalid robot state.
- Real shadow verification:
  - front-running 104 real shadow startup reached:

```text
M20PRO REAL OK: required topics, nodes, maps and Nav2 are active
```

- No real/web processes were left running after this diagnosis.
- Important field note:
  - if 106 RViz or 104 web shows ODOM/pose errors, first check whether localization is valid (`Location=0`) and whether 103 `1007/2` returns a full valid pose;
  - do not treat an `inf` ODOM state as a normal Nav2 problem.

## 2026-06-11 Web heading correction and full-stack test policy

- User correctly pointed out that the web robot arrow looking reversed should not be fixed by blindly rotating the display by 180 degrees.
- Correction applied:
  - reverted the default `robot_pose_display_yaw_offset_rad` from `pi` to `0.0`;
  - the parameter remains available only as an explicit debug override;
  - saved waypoint yaw, task yaw, and web relocalization yaw continue to use the raw map-frame yaw.
- Added web dashboard diagnostics:
  - `定位状态` from `/m20pro_tcp_bridge/localization_ok`;
  - `原厂导航` from `/m20pro_tcp_bridge/navigation_status`;
  - robot pose display now shows both displayed heading and raw heading, plus display offset only when a nonzero offset is explicitly configured.
- Field diagnosis rule:
  - if heading looks wrong, first compare web pose, `/m20pro_tcp_bridge/map_pose`, `/ODOM`, active map, and real robot orientation;
  - only change frontend drawing if localization and source yaw are correct and only the canvas display is wrong.
- Field startup policy tightened:
  - real tests must use one full-stack startup path on 104;
  - use `scripts/104_start_real_shadow.sh` for no-motion checks;
  - use `scripts/104_start_real_move.sh` only when motion control is allowed;
  - standalone `104_start_web.sh` / `m20pro_real_web.sh` are now documented as development preview only and are not valid for relocalization, marking, or task dispatch tests because they do not start tcp_bridge/Nav2/pointcloud fusion.

## 2026-06-12 Full-stack preflight script

- User clarified that the old Task 1 should not be a manual procedure for every customer-side startup.
- Added `scripts/104_preflight_check.sh`:
  - must be run on 104 after the known-good `source -> su -> source install` sequence;
  - does not restart original services, does not start/stop real, and does not change DDS/multicast settings;
  - checks full real startup nodes, topics, lidar data, scan data, finite map pose, finite ODOM, `/m20pro_tcp_bridge/localization_ok`, vendor navigation status, Nav2 lifecycle active states, web `/healthz`, and web `/api/state`;
  - default mode is `move`, so it also confirms motion command mode before field tasks;
  - `shadow` can be passed for no-motion diagnostics.
- Field policy:
  - start `scripts/104_start_real_move.sh`;
  - open another 104 terminal and run `scripts/104_preflight_check.sh move`;
  - if it prints `M20PRO PREFLIGHT OK`, proceed to web relocalization/marking/task dispatch;
  - if it prints `M20PRO PREFLIGHT FAIL`, do not start a task and fix the listed items first.
