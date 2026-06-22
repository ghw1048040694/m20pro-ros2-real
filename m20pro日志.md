# M20 Pro Project Notes

Last updated: 2026-06-22 18:35 CST

This file is maintained by Codex as the local M20 Pro project memory for future ChatGPT review. It records the current architecture, important decisions, recent changes, verification status, and next steps.

Naming note: this file replaced the previous local-only `codex.md`. Going forward, maintain this file, `m20pro日志.md`, after every meaningful project change or field diagnosis.

## 2026-06-22 field frontend failure postmortem and hardening

- Field report from the user:
  - on the development dog at the real site, web self-check timed out/failed;
  - web relocalization stayed at "sending /initialpose to 104 and 106" and did not complete;
  - marking points kept looking like "pending confirmation" and did not give trustworthy saved feedback;
  - after using the official 106 RViz `2D Pose Estimate`, the web frontend did not clearly reflect the localization recovery.
- Root causes identified:
  - `m20pro-real.service` runs as `root`, while the verified 104 -> 106 SSH setup was for `user`; the web process could therefore fail or hang on the remote 106 `/initialpose` path even though manual `user` SSH looked OK;
  - the one-shot remote `/initialpose` publisher could exit too quickly after creating a ROS 2 publisher; if DDS discovery had not found the 106 subscriber yet, the command could return success without the factory localization actually receiving the message;
  - preflight incorrectly allowed missing `/bt_navigator`/Nav2 node-name details to block the base self-check, even when map, lidar, `/scan`, localization, battery, and factory status were already OK;
  - annotation save feedback was written only to the map cursor/status text, so normal pointer movement could overwrite "saved" with "pending" text and make a successful save look unconfirmed.
- Code hardening:
  - web relocalization now explicitly uses `/home/user/.ssh/id_ed25519` and `/home/user/.ssh/known_hosts` for the 106 SSH hop, so it still works when the service process is `root`;
  - the remote 106 publisher waits up to 5 s for `/initialpose` subscribers, prints `subscriptions=N`, and reports `subscriptions=0` as a failed remote publish instead of pretending success;
  - the relocalization button now has a 45 s browser-side timeout, disables itself while running, and shows the full 106 SSH/initialpose/localization/map-pose/costmap verification result instead of staying in a vague "sending" state;
  - the localization tab now continuously displays live localization state, map pose, `/scan`, local/global costmap freshness, and factory navigation status, so official 106 RViz relocalization should also become visible in the web UI;
  - web preflight now splits base nodes from navigation nodes: `m20pro_tcp_bridge`, `m20pro_pointcloud_fusion`, `m20pro_web_dashboard`, `map_server`, `m20pro_nav2_startup_gate`, and `m20pro_floor_manager` remain base; `controller_server`, `planner_server`, `bt_navigator`, and `waypoint_follower` are navigation readiness warnings;
  - the marking tab now has persistent `markSaveStatus` feedback with saved id/label, floor, x/y/yaw, and current point count; delete operations also report that the list was refreshed.
- Guardrails:
  - `scripts/check_preflight_policy.py` now prevents future regressions where `/bt_navigator` blocks base preflight;
  - the same policy check verifies the root-compatible 106 SSH identity parameter and the subscriber wait in the remote `/initialpose` publisher.
- Verification on local real workspace:
  - Python compile passed for real launch, standalone web launch, web dashboard, and policy script;
  - shell syntax passed for `scripts/*.sh` and `src/m20pro_bringup/scripts/*.sh`;
  - `git diff --check` passed;
  - `python3 scripts/check_preflight_policy.py` passed;
  - `colcon build --symlink-install --packages-select m20pro_description m20pro_inspection m20pro_cloud_bridge m20pro_navigation m20pro_bringup` passed.
- Deployment verification on the development dog 104:
  - synced the real workspace to `/home/user/m20pro_real_ros2_ws`;
  - rebuilt the same five packages on 104 successfully;
  - restarted `m20pro-real.service`;
  - web `/healthz` returned `{"ok":true}`;
  - blocking web preflight returned `ok=true`, `navigation_ready=true`, `relocalization_ready=true`, `failures=0`, `warnings=0`;
  - `/api/state` showed `localization_ok=true`, fresh relay lidar, fresh `/scan`, fresh local/global costmaps, valid map pose, and battery around 55%;
  - Nav2 lifecycle states on 104 were active for `/controller_server`, `/planner_server`, `/bt_navigator`, and `/waypoint_follower`;
  - root-compatible SSH from 104 to 106 using `/home/user/.ssh/id_ed25519` returned the 106 hostname;
  - 106 `/initialpose` topic info reported type `geometry_msgs/msg/PoseWithCovarianceStamped` and `Subscription count: 1`;
  - the actual web relocalization API was not fired during this verification to avoid changing the robot's current field pose without operator intent.
- Important operational note:
  - being at the workstation can explain unlocalized/costmap-deferred navigation readiness, but it must not explain missing lidar or missing `/scan`;
  - future field debugging should first look at the relocalization result's `factory_initialpose.ssh_identity_file`, `factory_initialpose.subscriptions`, `verification.factory_pose_accepted`, live `localization_ok`, and `/scan` freshness.

## 2026-06-22 web relocalization changed to 106-first path

- User challenged the previous `/initialpose` explanation:
  - the factory manual workflow is effectively on 106/NOS through RViz `2D Pose Estimate`;
  - web relocalization had been tested several times and did not visibly work;
  - therefore the project must not claim that publishing `/initialpose` on 104 or sending the 103 TCP diagnostic request guarantees relocalization.
- Clarified meaning:
  - `/initialpose` is the ROS standard initial-pose topic used by RViz `2D Pose Estimate`;
  - it is only an input/guess for the localization system;
  - success must be verified by 106/factory localization state, not by the mere fact that 104 published a ROS message.
- Previous risk:
  - the web backend published `/initialpose` locally on 104;
  - full real startup also forced `tcp_bridge enable_initialpose_relocalization=true`, so the same message was intercepted and sent to 103 TCP `Type=2101 Command=1`;
  - if 106 factory localization is actually listening on 106's own ROS graph, then 104-side DDS delivery may be missed or blocked by DDS/network/participant issues;
  - 103 TCP `2101/1` is useful diagnostics but should not be treated as the main relocalization path.
- Code change:
  - web relocalization now still publishes local 104 `/initialpose`, but also SSHes to `user@10.21.31.106` and publishes the same `PoseWithCovarianceStamped` on 106 itself;
  - new web parameters:
    - `factory_initialpose_remote_publish` default `true`;
    - `factory_initialpose_topic` default `/initialpose`;
    - `factory_initialpose_source_command` default `source /opt/robot/scripts/setup_ros2.sh || source /opt/ros/foxy/setup.bash`;
    - `factory_initialpose_command_timeout_s` default `15.0`;
  - `m20pro_real_full.sh` now keeps `enable_initialpose_relocalization=false` in the full real runtime params, so 103 TCP `2101/1` no longer hijacks the normal `/initialpose` path.
- Runtime requirement:
  - 104 must have non-interactive SSH access to 106 as `user@10.21.31.106`;
  - current local check failed with `Permission denied (publickey,password)`, so this must be fixed on the robot before the new web relocalization can use the 106 path.
- Correct success criteria:
  - remote 106 `/initialpose` command returns OK;
  - `/m20pro_tcp_bridge/localization_ok` becomes true;
  - `/m20pro_tcp_bridge/map_pose` updates after the request;
  - updated pose is close to the requested pose;
  - `/scan` and costmaps recover after localization.
- Interview-ready explanation:
  - "I initially exposed relocalization through the ROS `/initialpose` convention, but field tests showed that the factory stack's reliable path is the 106/NOS RViz path. I then changed the web dashboard to publish the initial pose directly on 106 via SSH while keeping the 104 publish as a local mirror, and I stopped treating the 103 TCP `2101/1` response as the main success signal. The final acceptance is based on factory localization state and map pose update, not just API return."

## 2026-06-22 development dog costmap/Nav2 startup hardening

- User priority:
  - field tests repeatedly reached a state where base perception was OK but self-check still warned about costmaps;
  - fix the current development dog/runtime problem before continuing broader navigation work;
  - also evaluate the hand-controller "assist" mode, described by the user as an RL/all-terrain gait with manual drive and real-time avoidance.
- Diagnosis on the currently connected development dog 104:
  - `m20pro-real.service` was `active` and `enabled`;
  - web frontend was reachable at `http://10.21.31.104:8080`;
  - `/api/state` showed F20 map loaded, fresh relay lidar pointcloud, fresh `/scan`, valid pose, `localization_ok=true`, and `navigation_status=location=0 obstacle=0`;
  - therefore the current problem was not missing pointcloud, not missing `/scan`, and not just being at the workstation.
- Nav2/costmap failure evidence:
  - `/local_costmap/costmap` and `/global_costmap/costmap` existed as topic names but had no real costmap publishers;
  - lifecycle states for `/controller_server`, `/planner_server`, `/bt_navigator`, and `/waypoint_follower` were `unconfigured`;
  - service log stopped at `lifecycle_manager_navigation: Configuring controller_server` and never reached activation;
  - `system_check` repeatedly reported:
    `missing_topics=/local_costmap/costmap,/global_costmap/costmap inactive_lifecycle=/controller_server:unconfigured,...`;
  - this is the concrete reason field self-check kept showing costmap warnings after other basic items were fixed.
- DDS/SHM runtime issue found on development dog:
  - `/dev/shm` was about 95% full;
  - large `fastrtps_*` files were present, many around 477 MB;
  - logs repeatedly showed FastDDS SHM errors such as `Failed to create segment ... Unable to Register SHM Transport`;
  - ROS CLI graph output was therefore not fully trustworthy by itself, especially for bare DDS/factory publishers.
- Startup DDS hardening applied in code:
  - `m20pro_real_full.sh` now defaults our full real stack to the project UDP-only FastDDS profile `m20pro_fastdds_udp.xml`;
  - factory `/opt/robot/fastdds.xml` remains available by setting `M20PRO_FASTDDS_PROFILE=factory`;
  - autostart default files now write `M20PRO_FASTDDS_PROFILE=project_udp`;
  - added `m20pro_runtime_snapshot.sh`, installed by `m20pro_bringup`, to log active DDS profile, ROS/RMW env, `/dev/shm` usage, largest SHM entries, and selected topics at startup;
  - `m20pro_lidar_relay_guard.sh` now restarts only the project lidar relay when its inherited DDS profile differs from the new startup profile, avoiding an old relay continuing under the old SHM-heavy profile.
- Nav2 lifecycle hardening applied in code:
  - real launch now passes `autostart:=False` into Nav2 `navigation_launch.py`;
  - added `m20pro_navigation.nav2_startup_gate`;
  - the gate waits for `/map`, fresh `/scan`, `localization_ok=true`, valid `/m20pro_tcp_bridge/map_pose`, and a usable `map -> m20pro_base_link` TF before requesting `/lifecycle_manager_navigation/manage_nodes` STARTUP;
  - this prevents Nav2 from trying to configure costmaps while the robot is still unlocalized or while basic prerequisites are not ready;
  - once the robot is relocalized in the field, the gate should request lifecycle startup and costmap publishers should appear.
- Self-check behavior corrected:
  - frontend self-check now sends `site:"auto"` instead of always forcing `site:"workstation"`;
  - when unlocalized, auto mode still treats costmap/Nav2 as workstation/deferred warnings;
  - once localization is OK, auto mode treats the robot as field/navigation context, so missing costmaps/Nav2 lifecycle remains a real navigation readiness problem;
  - preflight now also expects `m20pro_nav2_startup_gate` as a core node.
- System check diagnostics improved:
  - `system_check_node.py` now reports high `/dev/shm` usage and the active FastDDS profile in waiting logs;
  - lifecycle waiting messages include remembered lifecycle query errors where available.
- Assist/RL gait integration first step:
  - existing control path is `/m20pro/gait_command` -> `tcp_bridge` -> vendor TCP request `Type=2 Command=23 GaitParam=...`;
  - existing mappings kept: `flat` -> `GaitParam=1`, `stair_*` -> `GaitParam=14`;
  - added configurable `gait_assist_param`, default `12`, with labels `assist`, `agile`, `rl`, `terrain`, and `all_terrain`;
  - real and sim config files now include `gait_assist_param: 12`;
  - because the hand-controller auxiliary/assist behavior has not been field-tested, `floor_manager` and real launch now default flat navigation gait back to `flat`; assist gait remains available only as an explicit test command.
- Assist/RL gait task integration follow-up:
  - `floor_manager` can actively publish the configured flat gait label before same-floor Nav2 goals, before post-switch floor goals, and before navigating to a stair entry;
  - with the current conservative default `flat_gait_label=flat`, normal patrol/floor goals do not automatically request the untested assist/RL gait;
  - stair traversal still switches to `stair_up` / `stair_down`, then returns to `flat` after exiting the stair/platform flow;
  - `publish_flat_gait_before_nav` remains available to control this automatic pre-nav flat gait request;
  - web dashboard now subscribes to `/m20pro_tcp_bridge/gait_result`, and the status area shows both requested gait and TCP bridge result such as `sent: label=assist GaitParam=12`.
- Important caveat about assist mode:
  - this code does not claim that `GaitParam=12` is definitely the exact hand-controller auxiliary/RL mode;
  - README/manual notes already record `Gait=12` as flat agile/navigation task gait and `ObsMode=0` as stop-obstacle avoidance;
  - field validation must confirm whether `gait_command=assist` produces the same behavior as the hand-controller auxiliary mode.
- Local verification:
  - Python compile passed for changed navigation/cloud bridge modules;
  - shell syntax check passed for changed startup scripts;
  - targeted builds passed:
    `colcon build --symlink-install --packages-select m20pro_navigation m20pro_bringup m20pro_cloud_bridge`.
  - after the assist-task follow-up, Python compile passed again for `floor_manager.py`, `tcp_bridge_node.py`, `web_dashboard_node.py`, `nav2_startup_gate.py`, and `system_check_node.py`;
  - extracted frontend JavaScript passed `node --check`;
  - shell syntax check passed for the changed startup/autostart scripts;
  - targeted build passed again for `m20pro_navigation`, `m20pro_cloud_bridge`, and `m20pro_bringup`.
- 104 deployment/current verification:
  - deployed the current workspace to the development dog 104 path `/home/user/m20pro_ros2_ws_20260529_173921`, which is not a git worktree;
  - `m20pro-real.service` is `enabled`, `active`, and running with `NRestarts=0`;
  - web health is OK at `http://10.21.31.104:8080/healthz`, and the listener is intentionally bound to `0.0.0.0:8080` so 104 can serve any connected host interface;
  - `/etc/default/m20pro-real` now uses `M20PRO_FASTDDS_PROFILE=project_udp` for the main project stack and `M20PRO_LIDAR_RELAY_FASTDDS_PROFILE=factory` for the raw lidar relay;
  - `/dev/shm` usage dropped from the earlier high-water mark around 95% to about 34% after the mixed-profile restart;
  - service logs show the relay reading factory lidar, `pointcloud_fusion` publishing `/scan`, `m20pro_nav2_startup_gate` requesting Nav2 lifecycle startup, and `lifecycle_manager_navigation` reporting managed nodes active;
  - dashboard `/api/state` shows `localization_ok=true`, `navigation_status=location=0 obstacle=0`, relay lidar around 44k-68k points, `/scan` about 211-219 finite ranges, and fresh local/global costmaps;
  - blocking preflight now returns `ok=true`, `navigation_ready=true`, and `relocalization_ready=true`; all core nodes, lidar, `/scan`, odom, map pose, map, both costmaps, battery, motion mode, and Nav2 lifecycle checks are OK.
- 104 post-assist deployment verification:
  - stopped `m20pro-real.service` before replacing the workspace, then rsynced/rebuilt the full workspace on 104 via `scripts/local_deploy_to_test_robot.sh`;
  - service stop hit a systemd stop-timeout while cleaning child processes, but all project ROS processes were gone afterward; reset the failed state before restart;
  - after restart, logs again show relay sample OK, `pointcloud_fusion` publishing `/scan`, startup gate requesting Nav2 startup, Nav2 lifecycle active, and `M20PRO REAL OK`;
  - blocking preflight after deployment returned `ok=true`, `navigation_ready=true`, `warnings=0`, and `failures=0`;
  - `/api/state` showed relay lidar around 69k points, `/scan` about 202 finite ranges, fresh local/global costmaps, `localization_ok=true`, and `navigation_status=location=0 obstacle=0`;
  - sent one non-motion gait test command `/m20pro/gait_command: assist`; `/api/state` and service logs confirmed `gait_result=sent: label=assist GaitParam=12`;
  - this proves the project command path to vendor gait parameter 12 is live, but field behavior still must be compared with the hand-controller auxiliary/RL mode.
- Runtime note after restart:
  - `/dev/shm` rose to about 58% after the post-deploy restart; this is below the 90% warning threshold but should be watched during long field runs;
  - do not clear `/dev/shm/fastrtps_*` while factory DDS participants are alive.
- Real costmap visibility fix:
  - changed real Nav2 local/global costmaps to `always_send_full_costmap: true`;
  - this makes `/local_costmap/costmap` and `/global_costmap/costmap` stay fresh for the web dashboard/self-check instead of relying only on update topics or one-shot full snapshots.
- Current caveat:
  - ROS CLI graph/echo remains unreliable for factory/raw DDS topics in this environment: topic names and subscribers may be visible even when `ros2 topic info` under a different FastDDS profile under-reports publishers;
  - judge the running system by web `/api/state`, service logs, relay sample logs, `/scan`, costmap freshness, and lifecycle states rather than by a single CLI `echo` of `/LIDAR/POINTS`.
- Field validation standard after deployment:
  - at workstation/unlocalized: base self-check may defer Nav2, but lidar, `/scan`, map, web, battery, and core nodes must be OK;
  - after field relocalization: `m20pro_nav2_startup_gate` should log that prerequisites are ready and lifecycle startup was requested/accepted;
  - `/controller_server`, `/planner_server`, `/bt_navigator`, and `/waypoint_follower` should become `active`;
  - `/local_costmap/costmap` and `/global_costmap/costmap` should have fresh data in `/api/state`;
  - if costmaps still do not appear, inspect startup gate logs and `system_check` for explicit DDS/SHM/lifecycle diagnostics rather than treating it as a generic workstation warning.

## 2026-06-18 test robot runtime, preflight, and RJ45 gateway update

- User reminder:
  - every project improvement must update `m20pro日志.md`;
  - this log had not been updated after several important 2026-06-18 fixes and field diagnostics.
- Latest pushed code state:
  - `f7daad7 Run dashboard preflight asynchronously`;
  - `8cffba7 Report relocalization readiness in preflight`;
  - `4f812b2 Clarify preflight and default fixed map flow`;
  - all three were pushed to GitHub `origin/main` and GitLab `gitlab/main`.
- Web preflight behavior is now asynchronous:
  - `/api/preflight/run` defaults to background execution unless the payload explicitly sends `wait:true`;
  - the web frontend sends `{mode:"move", site:"workstation", wait:false}`;
  - the POST timeout is now 10 s because the endpoint should return immediately;
  - the frontend then polls `/api/preflight` for the final result;
  - direct verification on 104 returned `running=true` in about 0.14-0.18 s.
- Interpretation of the old "30 seconds timeout" report:
  - if the browser still shows the old 30 s timeout, first suspect stale code, stale build, stale service, or browser cache;
  - verify with `curl -s http://127.0.0.1:8080/ | grep -E 'wait: false|10000|30000'`;
  - the new page should contain `wait: false` and `10000`, not the old synchronous `30000` path.
- Map/default relocalization behavior:
  - the frontend defaults to the manifest/default F20 fixed map when available;
  - changing the map dropdown loads the selected fixed 2D map immediately;
  - relocalization readiness is now separated from full navigation readiness;
  - a robot can be allowed to perform relocalization when map, lidar/scan, and core base services are OK even if Nav2/costmaps are still waiting because it is at the workstation or not localized yet.
- Test dog 104 status after setup:
  - workspace is a normal git checkout at `~/m20pro_ros2_ws`;
  - current HEAD verified as `f7daad7`;
  - `m20pro-real.service` is `active` and `enabled`;
  - full frontend is reachable at `http://10.21.31.104:8080`;
  - `/api/state` shows F20 map loaded, fresh relay lidar points, fresh `/scan`, fresh battery, and `navigation_status=location=1 obstacle=0`.
- Important root cause for the test dog self-check failure:
  - the failure was not low battery;
  - the failure was not because the robot was at the workstation;
  - the failure was not missing lidar or `/scan`;
  - 104 was missing Nav2 runtime packages, and the service log explicitly said:
    `Nav2 packages are not installed; starting M20Pro real bringup in observation mode without map_server/navigation.`
  - because of this, `/map_server`, `/controller_server`, `/planner_server`, `/bt_navigator`, `/m20pro_floor_manager`, and `/map` were absent, so preflight failed on map/Nav2 base items.
- Fix applied on the test dog 104:
  - installed `ros-foxy-navigation2` and `ros-foxy-nav2-bringup`;
  - restarted `m20pro-real.service`;
  - verified Nav2 packages via `ros2 pkg prefix`;
  - verified service process tree now includes `map_server`, `controller_server`, `planner_server`, `bt_navigator`, `waypoint_follower`, lifecycle managers, and `m20pro_floor_manager`;
  - verified `/map`, `/local_costmap/costmap`, `/global_costmap/costmap`, `/scan`, `/m20pro/lidar_points_relay`, `/LIDAR/POINTS`, `/LIDAR/POINTS2`, and `/ODOM` are visible.
- Current test dog preflight result after the Nav2 dependency fix:
  - `ok=true`;
  - `failures=0`;
  - `relocalization_ready=true`;
  - `navigation_ready=false` while at the workstation/unlocalized;
  - lidar points, `/scan`, `/map`, odom, battery, core nodes, and required topics are OK;
  - warnings are limited to map pose/localization/costmap/Nav2 lifecycle items that should be resolved after field relocalization.
- Workstation warning interpretation:
  - `Location=1`, `localization_ok=false`, missing valid `/m20pro_tcp_bridge/map_pose`, and costmap/TF lifecycle warnings can be expected at the workstation before relocalization;
  - missing lidar, missing `/scan`, missing `/map`, or missing core Nav2 nodes are not acceptable as "workstation" explanations.
- 103 RJ45-to-Wi-Fi gateway configuration for colleagues' wired-only Ubuntu laptops:
  - 103 Wi-Fi internet exit is `p2p0` on the workshop Wi-Fi;
  - 103 RJ45/workstation subnet is `eth2`, `10.21.31.103/24`;
  - kernel IPv4 forwarding is enabled and persisted by `/etc/sysctl.d/99-m20pro-internet-gateway.conf`;
  - NAT/forwarding from `10.21.31.0/24` on `eth2` to `p2p0` is active and saved in `/etc/iptables/rules.v4`;
  - `dnsmasq` is now `active` and `enabled`.
- 103 DHCP details:
  - `/etc/dnsmasq.conf` now serves DHCP on `eth2`;
  - wired laptops receive addresses in `10.21.31.150-10.21.31.199`;
  - gateway option is `10.21.31.103`;
  - DNS options are `223.5.5.5`, `114.114.114.114`, and `8.8.8.8`;
  - the pool deliberately avoids fixed robot addresses such as 104.
- Operator note for colleague laptops:
  - set Ubuntu wired IPv4 to automatic DHCP;
  - after the 103 change, disconnect/reconnect the wired network or renew DHCP;
  - expected result is an IP in `10.21.31.150-199`, default route via `10.21.31.103`, and working internet through 103's Wi-Fi.
- Do not confuse the two robots:
  - the development dog 104 seen earlier is not necessarily a git checkout and may require rsync deployment;
  - the current test dog 104 is git-enabled and was updated by `git pull`, build, and service restart;
  - always check `hostname`, `ip -br addr`, and `cd ~/m20pro_ros2_ws && git log --oneline -3` before assuming which robot is connected.

## 2026-06-18 update and build workflow notes

- On a git-enabled robot after pulling new code:
  - plain `colcon build` is allowed;
  - it is slower and may rebuild more than necessary, but it avoids typing `--symlink-install --packages-select ...`;
  - after build, restart the service with `sudo systemctl restart m20pro-real.service`.
- Faster targeted build remains useful when only web code changed:
  - `colcon build --symlink-install --packages-select m20pro_cloud_bridge`;
  - then restart `m20pro-real.service`.
- Pull failures previously seen on a second robot:
  - `.git/FETCH_HEAD: Permission denied` was caused by root-owned `.git/FETCH_HEAD`;
  - fix ownership with `sudo chown -R user:user .git`;
  - later `Could not resolve hostname github.com` was DNS/network, not git credentials.
- When diagnosing git/network on robots:
  - first verify default route and DNS;
  - then verify `ping 8.8.8.8`, `getent hosts github.com`, and `ping github.com`;
  - do not assume repository or SSH key failure until name resolution works.

## 2026-06-17 New session handoff summary

- Current preferred first read for any new assistant/session:
  - this file, especially this section and `2026-06-17 104 pointcloud recovery experience book`;
  - then inspect current `git status` before changing code.
- Current 104 runtime status after the last recovery:
  - full real frontend is running at `http://10.21.31.104:8080`;
  - `m20pro-real.service` is now `enabled` and `active`;
  - boot autostart is installed in `move` mode, but it still does not automatically start any inspection task;
  - web `/healthz` returned `{"ok":true}`;
  - service log shows the startup lidar guard received a real `/LIDAR/POINTS` sample before full real startup;
  - service log shows `PointCloud fusion ready: /LIDAR/POINTS -> /scan in m20pro_base_link`;
  - web `/api/state` reported fresh `/scan`, `/ODOM`, battery, navigation status, localization status, and current floor.
- Current full real startup choices:
  - `m20pro_real_full.sh move` uses the factory FastDDS profile by default;
  - set `M20PRO_USE_PROJECT_FASTDDS=1` only for deliberate DDS experiments;
  - full real runtime params must keep `enable_initialpose_relocalization: true`, `enable_axis_command: true`, and `send_idle_zero_commands: false`.
- Current navigation boundary:
  - the robot is still unlocalized (`Location=1`, `localization_ok=false`) in the current parking/charging state;
  - web/frontend/perception are up, but navigation readiness requires field relocalization before starting a task.
- Current important dirty changes are intentional unless a later human says otherwise:
  - `src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py`: web map/3D map/relocalization/task robustness work;
  - `src/m20pro_cloud_bridge/m20pro_cloud_bridge/pcd_derived.py`: PCD-derived lightweight 3D terrain, height grid, and stair zone generation;
  - `src/m20pro_navigation/m20pro_navigation/floor_manager.py`: stair-zone subscription and safer repeated task behavior;
  - `src/m20pro_bringup/scripts/m20pro_real_full.sh`: factory FastDDS by default, relocalization bridge enabled, move mode can send axis commands;
  - `src/m20pro_bringup/scripts/m20pro_record_real.sh`: factory FastDDS by default for field recording;
  - `scripts/local_deploy_to_test_robot.sh`: local-to-104 rsync/deploy path for the second/test robot and 104 rebuilds.
- Field test rule:
  - do not start navigation just because the web page opens;
  - first use the web relocalization page, enable the live scan overlay, drag the arrow until scan and map match, then execute relocalization;
  - after relocalization, run the web preflight/self-check and only start a task if localization and navigation are ready.
- Hard prohibition:
  - do not clear `/dev/shm/fastrtps_*`;
  - do not restart or edit factory multicast/lidar services;
  - do not start multiple real stacks at the same time;
  - do not “fake” localization when `Location=1`.

## 2026-06-18 104 workstation lidar/scan stabilization

- User correctly clarified that being at the workstation can explain unlocalized/Nav2 warnings, but must not explain missing `/LIDAR/POINTS` or `/scan`.
- Diagnosis from 104:
  - the startup guard could receive a real `/LIDAR/POINTS` sample;
  - the web node sometimes received fresh `/LIDAR/POINTS`;
  - `m20pro_pointcloud_fusion` had a publisher on `/scan` but its new diagnostics showed `cloud=0`, meaning its `/LIDAR/POINTS` subscription was discovered but received no callbacks;
  - this proved the failure was not TF, height filtering, source-age filtering, or workstation location. It was the fragile 104 DDS raw pointcloud subscription path.
- Added `m20pro_navigation.lidar_relay_node`:
  - executable: `ros2 run m20pro_navigation lidar_relay`;
  - subscribes to the factory `/LIDAR/POINTS`;
  - republishes the same real `PointCloud2` frames to `/m20pro/lidar_points_relay`;
  - publishes status on `/m20pro/lidar_relay/status`;
  - logs real sample confirmations such as `frame=lidar_link points=... messages=...`.
- Added `m20pro_lidar_relay_guard.sh`:
  - starts the relay as a minimal long-lived process before the full real launch;
  - waits for a real relay sample from the relay log;
  - detects an existing matching relay instead of starting duplicates;
  - stop mode cleans all matching relay processes.
- Real full startup now uses the relay path:
  - by default it no longer runs a short-lived raw `/LIDAR/POINTS` echo guard before full launch;
  - it starts/waits for the long-lived relay first;
  - it passes `cloud_topic:=/m20pro/lidar_points_relay` to the full real launch;
  - `m20pro_pointcloud_fusion` therefore subscribes to the project-internal relay topic instead of directly competing for the factory raw pointcloud.
- `pointcloud_fusion.py` was hardened:
  - added diagnostics on `/m20pro/pointcloud_fusion/status`;
  - logs cloud callback count, processed count, skipped/stale/drop reasons, finite scan bins, source topic, and scan publish count;
  - real launch now uses `max_source_age_s=1.0` and `publish_on_cloud_update=true`.
- Web dashboard now treats either raw or relay pointcloud as the real lidar state:
  - subscribes to `/m20pro/lidar_points_relay` for metadata;
  - marks `lidar_points.source` as `raw` or `relay`;
  - `/api/state` and workstation preflight no longer fail just because the web raw subscriber missed the factory topic while the relay is healthy.
- Verification on 104 at the workstation:
  - `m20pro-real.service` active;
  - exactly one long-lived `m20pro_lidar_relay` process remained after the duplicate-relay cleanup;
  - full launch command uses `cloud_topic:=/m20pro/lidar_points_relay`;
  - `/api/state` showed fresh `lidar_points` with `source="relay"` and fresh `/scan`;
  - `pointcloud_fusion` log showed `source=/m20pro/lidar_points_relay`, `processed>0`, `scan>0`, `finite_bins` around 200+, and `reason=published`;
  - workstation preflight returned `ok=true`, `navigation_ready=false`, with `/LIDAR/POINTS`/lidar and `/scan` OK and only localization/Nav2/costmap warnings left for field relocalization.
- Current operator interpretation:
  - lidar and `/scan` must be healthy at the workstation and in the test field;
  - localization/Nav2 readiness is still expected to remain warning at the workstation until field relocalization;
  - do not use missing `/scan` as a workstation explanation anymore.

## 2026-06-17 104 lidar startup hardening

- User asked to reduce the recurring state where 104 can no longer receive radar pointcloud data.
- Added a project-side guard script:
  - `src/m20pro_bringup/scripts/m20pro_lidar_guard.sh`;
  - installed by `m20pro_bringup`;
  - checks actual `/LIDAR/POINTS` `PointCloud2` sample reception, not only topic names;
  - defaults to the factory `/opt/robot/fastdds.xml` when no FastDDS profile is set;
  - prints the forbidden recovery actions directly: do not clear `/dev/shm/fastrtps_*`, do not restart factory multicast/lidar services from this project, and stop only the project real stack first.
- Full real startup is now stricter:
  - `m20pro_real_full.sh` refuses to start if another real `m20pro.launch.py mode:=real` is already running;
  - before starting Nav2/web/tcp_bridge/pointcloud_fusion it runs the lidar guard in `startup` mode;
  - if `/LIDAR/POINTS` is visible but no sample arrives, startup exits instead of creating more DDS participants.
- Project FastDDS experiment profile was made genuinely UDP-only:
  - `src/m20pro_bringup/config/m20pro_fastdds_udp.xml` no longer declares or uses SHM transport;
  - default runtime still prefers the factory `/opt/robot/fastdds.xml`;
  - `M20PRO_USE_PROJECT_FASTDDS=1` remains an explicit experiment switch.
- Systemd behavior was adjusted:
  - `m20pro-real.service` treats duplicate-stack exit `70` and lidar-not-ready exit `75` as non-restart statuses;
  - this prevents a boot-time pointcloud issue from becoming a 5-second restart loop that repeatedly creates ROS/DDS participants.
- Field scripts now share the same sample-level rule:
  - `m20pro_record_real.sh` uses the lidar guard before recording;
  - `scripts/104_check_lidar.sh` uses the lidar guard;
  - `scripts/104_status.sh` includes a quick sample check.
- This hardening intentionally does not change 106 services, factory multicast settings, or FastDDS shared-memory runtime files.

## 2026-06-17 104 autostart enabled after charging pause

- User paused because the robot battery was low; deployment was resumed after charging.
- Resumed state before deployment:
  - no local deploy/rsync/ssh residual process was running;
  - 104 was reachable;
  - `m20pro-real.service` was `inactive` and `disabled`;
  - 104 had about 5.2 GB free on `/`.
- Synchronized the current local workspace to `user@10.21.31.104:/home/user/m20pro_ros2_ws`.
- Full 104 build succeeded for all five packages:
  - `m20pro_cloud_bridge`;
  - `m20pro_description`;
  - `m20pro_inspection`;
  - `m20pro_navigation`;
  - `m20pro_bringup`.
- Installed and enabled autostart:
  - command: `./scripts/104_enable_autostart.sh move`;
  - `systemctl is-enabled m20pro-real.service` -> `enabled`;
  - `/etc/default/m20pro-real` contains:
    - `M20PRO_REAL_MODE=move`;
    - `M20PRO_WS=/home/user/m20pro_ros2_ws`;
    - `M20PRO_LIDAR_STARTUP_WAIT_S=45`.
- Started the service once for validation:
  - `systemctl is-active m20pro-real.service` -> `active`;
  - `systemctl show` reported `Restart=on-failure`, `RestartPreventExitStatus=70 75`, `NRestarts=0`;
  - `http://10.21.31.104:8080/healthz` returned `{"ok":true}`;
  - port `0.0.0.0:8080` is listening.
- Startup robustness verification:
  - lidar guard used factory `/opt/robot/fastdds.xml`;
  - lidar guard saw `/LIDAR/POINTS` and `/LIDAR/POINTS2`;
  - lidar guard received `/LIDAR/POINTS` sample `width=46432 height=1`;
  - `m20pro_tcp_bridge` started with `axis command enabled; idle zero disabled`;
  - `pointcloud_fusion` reported `/LIDAR/POINTS -> /scan`;
  - web dashboard reported listening on `http://0.0.0.0:8080`.
- Runtime web state from `/api/state`:
  - `scan available=True`, age about 0.015 s;
  - `odom available=True`, age about 0.011 s;
  - `battery available=True`, primary battery about 97%, about 82.1 V;
  - `navigation_status=location=1 obstacle=0`;
  - `localization_ok=False`.
- Interpretation:
  - boot autostart and full frontend availability are confirmed;
  - the frontend/perception chain is alive after startup;
  - the robot is still unlocalized, so the operator must relocalize in the field before considering Nav2/navigation ready or starting a task.
- Note:
  - opening a new CLI `ros2 topic echo /LIDAR/POINTS` while the full real stack is already running can still fail to receive raw samples in this fragile DDS state;
  - for startup safety, rely on the built-in startup lidar guard and web `/api/state` `/scan` freshness;
  - do not repeatedly start extra real stacks or manual raw pointcloud subscribers for routine checks.

## 2026-06-17 workstation-friendly base self-check

- User reported that the web base self-check often shows "15 seconds without response" while the robot is at the workstation, not in the mapped test field.
- Important operator context:
  - the robot is currently at the workstation/charging area;
  - `Location=1` and `localization_ok=false` are expected until field relocalization;
  - local/global costmaps and Nav2 lifecycle may remain waiting before relocalization and must not block the "开机基础自检" response.
- Frontend change:
  - `runPreflight()` now sends `{mode: "move", site: "workstation"}`;
  - request timeout was increased from 15 s to 30 s;
  - the pending text now says workstation/unlocalized state only checks base links;
  - timeout text now tells the operator to refresh/check service status instead of implying field navigation failure.
- Backend change:
  - preflight defaults to workstation mode;
  - topic freshness window is bounded to 2-8 s;
  - if raw `/LIDAR/POINTS` is not directly cached by the web node but `/scan` is fresh and has finite ranges, base perception is treated as usable with a warning instead of a hard failure;
  - localization warnings now explicitly say workstation/unlocalized state is expected and relocalization must be done in the test field;
  - Nav2 lifecycle queries are deferred in workstation/unlocalized state, so they no longer add blocking waits to the base self-check.
- Terminal fallback `scripts/104_preflight_check.sh` was aligned:
  - removed raw `/LIDAR/POINTS` CLI echo as a base hard gate;
  - reports web `/api/state` scan freshness;
  - explicitly labels missing pose as workstation/unlocalized and tells the operator to relocalize in the test field before starting tasks.
- Startup guard correction after 104 validation:
  - the previous strict lidar startup guard could keep the full frontend down at the workstation when `/LIDAR/POINTS` was temporarily not visible even though the operator still needed the web UI;
  - `m20pro_real_full.sh` now defaults to `M20PRO_LIDAR_GUARD_MODE=warn`, meaning lidar guard failures are logged but the full frontend still starts;
  - set `M20PRO_LIDAR_GUARD_MODE=strict` only for a deliberate field mode where no frontend should start unless raw lidar samples are confirmed before launch;
  - `systemd/m20pro-real.default` and `104_enable_autostart.sh` now write `M20PRO_LIDAR_GUARD_MODE=warn`.
- Local verification:
  - `python3 -m py_compile web_dashboard_node.py` passed;
  - `bash -n scripts/104_preflight_check.sh` passed;
  - extracted dashboard JavaScript passed `node --check`;
  - `git diff --check` passed.

## 2026-06-17 104 pointcloud recovery experience book

### What happened

- The user rebooted the robot and manually confirmed that 104 again exposed both factory pointcloud topics:
  - `/LIDAR/POINTS`;
  - `/LIDAR/POINTS2`.
- During later recovery/debug work, the assistant started and stopped the full real stack repeatedly and also previously touched FastDDS shared-memory runtime state while factory DDS participants were alive.
- That produced a misleading intermediate state on 104:
  - topic discovery could still show `/LIDAR/POINTS` and `/LIDAR/POINTS2`;
  - `ros2 topic info` could still show publishers;
  - but `ros2 topic echo` / `ros2 topic hz` could fail to receive samples.
- This was not evidence that the lidar hardware or 106 publisher was gone. 106 still had healthy lidar services and 104 could still see multicast UDP from 106. The failure was in the 104 DDS/subscriber side being destabilized.

### Why it was disturbed

- FastDDS shared-memory transport on 104 is fragile in this environment.
- Clearing or disturbing `/dev/shm/fastrtps_*` while factory DDS participants are running can break live DDS participants.
- The project FastDDS XML name says `udp`, but the file still contains SHM transport entries. Using it unintentionally can bring SHM warnings and unstable discovery/subscription behavior.
- Full real startup/stopping also creates many ROS participants. If mixed with manual DDS cleanup or multiple stacks, 104 can enter a bad state even though topic names remain visible.

### Correct baseline check

Use the exact known-good manual sequence when verifying factory lidar:

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
ros2 topic list | grep -E '^/LIDAR/POINTS$|^/LIDAR/POINTS2$'
timeout 8 ros2 topic echo /LIDAR/POINTS --no-arr
```

Interpretation:

- Seeing the topic name is not enough.
- Seeing publisher count is not enough.
- At least one `PointCloud2` sample or a positive `ros2 topic hz /LIDAR/POINTS` result is the useful confirmation.

### Correct recovery order

1. Stop our stack only:

```bash
systemctl stop m20pro-real.service 2>/dev/null || true
pkill -INT -f 'm20pro_real_full.sh|m20pro.launch.py|web_dashboard|pointcloud_fusion' 2>/dev/null || true
```

2. Do not touch factory services unless the user explicitly asks:

```text
Do not restart multicast-relay.service.
Do not restart rsdriver.service.
Do not clear /dev/shm/fastrtps_*.
```

3. Reboot the robot if 104 is in the bad topic-name-only state.

4. After reboot, first verify factory lidar with the baseline check above.

5. Only after pointcloud samples are readable, start the full real frontend.

### Current frontend recovery command pattern

For the current manual/systemd-hosted test run:

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
su
cd /home/user/m20pro_ros2_ws
source install/setup.bash
systemctl start m20pro-real.service
curl -fsS http://127.0.0.1:8080/healthz
```

Expected state:

```text
m20pro-real.service: active
m20pro-real.service enabled state: disabled
http://10.21.31.104:8080/healthz -> {"ok":true}
runtime params:
  enable_initialpose_relocalization: true
  enable_axis_command: true
  send_idle_zero_commands: false
```

### Lessons for future assistants

- Do not debug 104 pointcloud by repeatedly changing DDS profile, clearing SHM, or restarting factory services.
- Do not assume `/LIDAR/POINTS` is healthy just because the topic exists.
- Do not assume CLI `ros2 param get` or lifecycle commands are reliable when FastDDS SHM errors appear; use process args, logs, web health, and runtime parameter files as secondary checks.
- If the robot is not localized, Nav2 may be active and the web page may load, but real navigation is still not ready.
- If localization is invalid (`Location=1`), fix relocalization. Do not publish fake odom or bypass the localization check.

## 2026-06-17 Web 3D map integrated into the main map viewport

- User clarified that the frontend must not add another crowded right-side panel for 3D.
- The separate `3D地图` tab/panel was removed.
- The left main map viewport now has `2D地图` and `3D地图` mode buttons:
  - `2D地图` remains the default operation mode for relocalization, waypoint marking, paths, scan overlay, and normal navigation work;
  - `3D地图` renders the PCD-derived lightweight terrain and stair zones directly in the big map area;
  - in 3D mode, dragging pans the 3D terrain and wheel/buttons zoom the 3D terrain; waypoint marking and relocalization are intentionally done after switching back to 2D.
- This preserves the single main spatial view and keeps the right-side panels focused on operations.
- While consolidating the 104 runtime back to a single default `install`, the full real startup path was changed to prefer the factory FastDDS profile by default. The project `m20pro_fastdds_udp.xml` still contains SHM entries despite its name, so it must only be enabled deliberately with `M20PRO_USE_PROJECT_FASTDDS=1` after a separate DDS test.

## 2026-06-16 PCD lightweight 3D map and stair semantic zones

- Customer-facing goal:
  - show a 3D-looking map/stair area in the web frontend without loading the full raw PCD in the browser;
  - avoid asking the customer to run manual post-processing commands after mapping;
  - move toward automatic stair gait switching based on semantic stair zones instead of manually adding gait-switch points in every task.
- Implementation direction:
  - map import from 106 now runs PCD post-processing on 104 after the map is copied;
  - generated files live under the imported map directory `derived/`: `terrain_mesh.json`, `height_grid.json`, `stair_zones.json`, plus optional local stair pointcloud JSON files;
  - if no PCD is found or post-processing fails, the imported 2D map remains usable and the map record stores `derived.status` plus a readable message.
- Web dashboard:
  - added `/api/map_3d`, `/api/stair_zones`, and `/api/stair_pointcloud`;
  - 3D rendering uses the left main map viewport, not a separate right-side panel;
  - stair zones are shown as translucent overlays, and the robot pose is drawn on the 3D view when available.
- Stair automation:
  - the web dashboard publishes formal stair zones on `/m20pro/stair_zones`;
  - `floor_manager` subscribes to that topic and only trusts zones with `trigger_gait=true`;
  - PCD-only height candidates are displayed but do not trigger gait automatically until manually/semantically confirmed.
- Controller decision:
  - DWB/DWA is not replaced as part of this work;
  - PCD display cost is handled by offline downsampling/height-grid derivation, not by changing the local planner.
- Verification:
  - local temporary PCD derivation test used `Original_map/full_cloud.pcd` with 1,434,319 points and produced a 125x148 height grid in about 0.7s;
  - generated terrain mesh was about 98KB in the test, far below raw PCD browser cost.

## 2026-06-15 second M20Pro configured as test robot

- User asked to make the second M20Pro a test robot that can update after GitLab changes.
- Second 104 initial state:
  - `/home/user/m20pro_ros2_ws` did not exist;
  - no Git repo was present;
  - 104 had no default route;
  - after temporary routing through 103, `git.fabu.ai` resolved to `192.168.3.100` but both 103 and 104 could not reach SSH/HTTPS ports on that address from the current `YiFangDa` network.
- Important limitation:
  - direct `git pull` from 104 is not currently possible on this network;
  - this is a network reachability issue to company GitLab, not a ROS workspace issue.
- Added scripts:
  - `scripts/local_deploy_to_test_robot.sh`;
  - `scripts/104_update_from_gitlab.sh`.
- Current usable update path:
  - pull/update code on the upper computer;
  - run `./scripts/local_deploy_to_test_robot.sh`;
  - the script rsyncs the current workspace to `user@10.21.31.104:/home/user/m20pro_ros2_ws`, excluding `.git`, `build`, `install`, `log`, and bag/database files;
  - then it builds the full workspace on 104 with `colcon build --symlink-install`.
- Future direct-GitLab path:
  - once 104 can reach `git.fabu.ai`, run `/home/user/m20pro_ros2_ws/scripts/104_update_from_gitlab.sh` on 104 after `su`;
  - the script will clone/fetch/reset to `gitlab/main` and build the full workspace.
- GitLab deploy key prepared on the second 104:
  - private key path: `/home/user/.ssh/id_ed25519_m20pro_test_gitlab`;
  - public key path: `/home/user/.ssh/id_ed25519_m20pro_test_gitlab.pub`;
  - SSH config entry for `git.fabu.ai` uses this key and `StrictHostKeyChecking accept-new`;
  - public key should be added to the GitLab project as a deploy key or to the developer account as an SSH key before using direct pull.
- User later added the key to GitLab, but direct validation from the robot still failed because the network path was unavailable:
  - 104 resolved `git.fabu.ai` to `192.168.3.100`;
  - 104 timed out on ports 22 and 443;
  - 103 also timed out to `192.168.3.100` on ports 22 and 443;
  - the upper computer could reach GitLab port 22, so the current practical path remains upper-computer pull plus `local_deploy_to_test_robot.sh` until the robot network can reach company GitLab.
- Mirror-source clarification:
  - mirror sources can help with public internet GitHub/Gitee access, but cannot make the robot reach the company intranet GitLab address `192.168.3.100`;
  - second 104 can reach public `github.com` and `gitee.com` on ports 22 and 443 through the current 103/p2p0 route;
  - anonymous HTTPS access to the existing GitHub repo prompted for credentials, so the repo is not public-readable from the robot;
  - added `scripts/104_update_from_mirror.sh`, defaulting to `git@github.com:ghw1048040694/m20pro-ros2-navigation.git`;
  - to use GitHub/Gitee as a robot-side mirror, add the second 104 public key to that mirror repo and run `104_update_from_mirror.sh` on 104.
- User wants the second robot to stay outside for testing without staying connected to the upper computer:
  - this is compatible with a private GitHub/Gitee mirror;
  - the repository does not need to be public;
  - add the second 104 public key as a read-only deploy key to the private mirror repo;
  - 104 SSH config now includes `github.com` and `gitee.com`, both using `/home/user/.ssh/id_ed25519_m20pro_test_gitlab`;
  - 104 verified SSH network reachability to both `github.com:22` and `gitee.com:22`.
- Deploy key verification:
  - user added the second 104 public key to the GitHub private repo;
  - from second 104 user account, `ssh -T git@github.com` authenticated as `ghw1048040694/m20pro-ros2-navigation`;
  - `git ls-remote git@github.com:ghw1048040694/m20pro-ros2-navigation.git HEAD` succeeded and returned `803200e88a182fb425aaca8fa0defd451193eee1`;
  - running the mirror update as root initially failed because root did not use `/home/user/.ssh/id_ed25519_m20pro_test_gitlab`, so `104_update_from_gitlab.sh` was updated to explicitly use that key when present;
  - the first robot-side clone from GitHub succeeded, proving the private mirror read path works;
  - the current GitHub HEAD does not yet include the new `104_update_from_mirror.sh` script, so the local repo changes must be committed and pushed to GitHub before the robot can use the final one-command mirror update without manually copied scripts.
- Verification on the second 104:
  - deployed the current workspace to `/home/user/m20pro_ros2_ws`;
  - full build succeeded for all five packages:
    - `m20pro_bringup`;
    - `m20pro_cloud_bridge`;
    - `m20pro_navigation`;
    - `m20pro_description`;
    - `m20pro_inspection`;
  - `ros2 pkg prefix` resolved all five packages from `/home/user/m20pro_ros2_ws/install`.
- Installed 104 autostart service on the second test robot:
  - ran `./scripts/104_enable_autostart.sh move`;
  - `m20pro-real.service` is enabled;
  - current service state was left `inactive`;
  - next boot will start the full real stack and web dashboard, but it will not automatically execute any task.
- A temporary NAT route through 103 was tested:
  - 103 had `p2p0` connected to `YiFangDa`;
  - enabled in-memory forwarding/NAT from `10.21.31.0/24` to `p2p0`;
  - this allowed DNS resolution, but company GitLab remained unreachable because `git.fabu.ai` resolved to `192.168.3.100` and that address was not reachable from the current network.
  - NAT was not made persistent.

## 2026-06-15 second M20Pro factory DDS and WiFi setup

- User connected a second M20Pro. Initial symptom:
  - 104 could list `/LIDAR/POINTS` and see DDS publishers, but `ros2 topic echo /LIDAR/POINTS` had no samples;
  - 106 also listed `/LIDAR/POINTS`, but its ROS 2 side initially had `Publisher count: 0` and no echo/hz samples.
- Root cause was consistent with the first robot's earlier factory-side DDS/multicast fix not being applied to this second robot:
  - 106 had `boardresources 1.3.11`;
  - 106 `/opt/robot/fastdds.xml` only whitelisted `127.0.0.1` and `10.21.33.106`;
  - `10.21.31.106` was missing;
  - `multicast-relay.service` was disabled and inactive;
  - `multicast-relay.service` had no `Wants=` line.
- Applied the official package from the desktop:
  - local file: `/home/fabu/桌面/boardresources.v1.3.13+fcae53.arm64(1).deb`;
  - copied to 106 as `/tmp/boardresources.deb`;
  - installed with `dpkg -i /tmp/boardresources.deb`;
  - 106 `boardresources` upgraded from `1.3.11` to `1.3.13`.
- The package did not automatically add the `10.21.31.106` FastDDS interface or enable multicast relay, so applied the same practical fix used on the first robot:
  - backed up `/opt/robot/fastdds.xml`;
  - added `<address>10.21.31.106</address>` to the interface whitelist;
  - backed up `/lib/systemd/system/multicast-relay.service`;
  - added `Wants=network-online.target`;
  - ran `systemctl daemon-reload`;
  - enabled and restarted `multicast-relay.service`.
- Verification:
  - 106 `multicast-relay.service` became enabled and active;
  - 106 `/LIDAR/POINTS` publisher count recovered to 2;
  - 104 successfully echoed live `/LIDAR/POINTS` frames with `frame_id: lidar_link` and about 55k to 81k points per frame.
- Applied official WiFi script from desktop:
  - local file: `/home/fabu/桌面/Wifi_link(1)(2).sh`;
  - copied to 103 as `/tmp/wifi_link.sh`;
  - ran as root with `WIFI_IFACE=p2p0 bash /tmp/wifi_link.sh`;
  - selected `YiFangDa` with password provided by the user;
  - 103 `p2p0` connected to `YiFangDa` and received `192.168.105.42`;
  - 103 default route became `default via 192.168.107.254 dev p2p0`;
  - 103 `wlan0` remained connected to `myap24G`, preserving the robot hotspot;
  - `ping www.baidu.com` from 103 succeeded;
  - `ping 10.21.31.104` from 103 also succeeded.

## 2026-06-15 field script doc update

- Rewrote desktop `/home/fabu/桌面/脚本.docx` as a plain field-operation Word document.
- Current script keeps only three main tasks:
  - same-floor real navigation test;
  - cross-floor real navigation test;
  - mapping flow test.
- Each task now starts from web/opening/self-check/map selection/relocalization and ends with stop/reset/result recording, so tasks can be run consecutively without rebooting the robot when the system is healthy.
- Added the latest web map controls to the script:
  - `+` zoom in;
  - `-` zoom out;
  - `平移`;
  - `适配`;
  - `居中机器人`.
- The script explicitly says to zoom in before dragging relocalization and waypoint arrows; zoom/pan only changes display and does not change map coordinates.
- Verified `/home/fabu/桌面/脚本.docx` with `unzip -t` and LibreOffice headless text conversion.

## 2026-06-15 web map zoom and pan

- User issue:
  - the web map could not zoom, so fine relocalization arrows and inspection waypoint placement were not accurate enough.
- Web dashboard change:
  - added map zoom controls: `-`, `+`, `适配`, `居中机器人`;
  - added `平移` mode for touchscreen/handheld use, where dragging moves the map instead of marking a point;
  - mouse wheel or touchpad scroll now zooms around the cursor;
  - right button, middle button, Shift-drag, or Alt-drag can temporarily pan the map without switching modes.
- Coordinate behavior:
  - zoom and pan are display-only;
  - `canvasToWorld()` still converts screen position back to true map-frame x/y through the same view transform;
  - saved waypoints and web relocalization still publish the original map coordinates, not scaled screen coordinates.
- Verification:
  - local `py_compile` passed for `web_dashboard_node.py`;
  - extracted dashboard JavaScript passed `node --check`;
  - `git diff --check` passed for the dashboard file.

## 2026-06-15 web task stop and start safety

- Field issue:
  - after starting a web task, the robot rotated in place several circles;
  - the web `停止当前任务` button was grey and could not be clicked.
- Diagnosis:
  - spinning is consistent with starting a Nav2 task while localization/map alignment is not confirmed, or while the robot pose/goal yaw relation is wrong;
  - the stop button was tied to `active_task.status == running`, so if the web task state became empty/error while Nav2 was still active, the button could become unavailable at exactly the wrong time.
- Code changes:
  - web `停止当前任务` is now always clickable and sends `/api/tasks/stop` with `reason=web_manual_stop`;
  - backend stop now clears active task state and always sends a navigation reset even when no active task is recorded;
  - backend reset publishes `/m20pro/stop_task` repeatedly, publishes multiple zero `/cmd_vel` samples, clears costmaps, and publishes idle waypoint status;
  - `floor_manager` now publishes several zero velocity samples when cancelling a Nav2 goal, reducing residual command risk;
  - starting a web task now requires confirmed localization, fresh map pose, robot pose inside the current live map, first waypoint inside the task map, and for same-floor fixed-map tasks, matching live-map metadata between the web-selected map and Nav2 `/map`;
  - cross-floor tasks are not blocked only because the first waypoint floor differs from the current floor.
- Verification:
  - local `py_compile` passed for `web_dashboard_node.py` and `floor_manager.py`;
  - extracted dashboard JavaScript passed `node --check`;
  - `git diff --check` passed for the touched files.
- Next field use:
  - after updating/restarting the 104 full real system, if the robot rotates or acts wrong, first click web `停止当前任务`;
  - if web is unreachable or stop does not take effect immediately, use the handheld/manual emergency stop as the fallback;
  - before starting a task, make sure web localization is normal and the displayed map is the same environment as the robot.

## 2026-06-12 preflight split: base check vs navigation readiness

- User tested at the field and reported that web self-check kept failing; after returning to the desk, diagnosis showed two different states were being mixed together.
- Clarification from the user:
  - that field test happened before the base-check/navigation-readiness split;
  - therefore the old failure does not prove that web relocalization is still ineffective after the latest changes.
- Current desk/off-map factory state:
  - `/LIDAR/POINTS` is healthy on 104, with about 41k to 42k points per frame;
  - factory TCP `2002/1` returns valid navigation status such as `Location=1, ObsState=0`;
  - factory TCP `1007/2` can return truncated JSON when the robot is unlocalized, stopping at `{"Location":1,"PosX":`;
  - factory `/ODOM` can contain invalid values such as `.inf` or extremely large coordinates while unlocalized.
- Important conclusion:
  - web self-check is read-only and does not lock handheld control;
  - before relocalization, pose/ODOM/scan/costmap/Nav2 checks are navigation-readiness checks, not boot/basic-health checks;
  - treating those items as hard preflight failures made the normal desk/off-map state look like a system failure.
- Code change:
  - web preflight now reports `ok` for base health only: core nodes, base topics, raw lidar, factory navigation status, map, battery, and motion mode;
  - `/scan`, `/ODOM`, `/m20pro_tcp_bridge/map_pose`, localization, local/global costmaps, and Nav2 lifecycle are now grouped as `navigation` warnings until relocalization is confirmed;
  - UI button text changed to `开机基础自检`;
  - summary now distinguishes `基础自检通过，导航待重定位后确认` from full navigation readiness;
  - terminal `scripts/104_preflight_check.sh` uses the same policy.
- Verified on 104:
  - synced repo and rebuilt `m20pro_cloud_bridge` and `m20pro_bringup`;
  - short-started `m20pro-real.service`;
  - `/api/preflight/run` returned `ok=true`, `navigation_ready=false`, raw lidar OK, battery about 50%, and expected navigation warnings because the robot was back at the desk/off-map;
  - stopped `m20pro-real.service` afterward;
  - confirmed no real/web processes and no 8080 listener remained.
- Field use:
  - after boot/autostart, click web `开机基础自检`;
  - if it says `基础自检通过，导航待重定位后确认`, drive/bring the robot to the mapped test area, perform web relocalization, then confirm navigation items turn healthy before starting an inspection task.

## 2026-06-15 web relocalization failure root cause pass

- User reported that web relocalization has never succeeded.
- Screenshot result:
  - `/LIDAR/POINTS` raw pointcloud is healthy;
  - `/scan` may be missing before valid localization/TF;
  - web relocalization returned `failed: Expecting value: line 1 column 1 (char 0)`;
  - verification showed `vendor_request: fail`.
- 104/103 direct TCP findings:
  - `2002/1` returns valid JSON with `Location=1, ObsState=0` while unlocalized;
  - `1007/2` can return malformed/truncated JSON while unlocalized, stopping at `{"Location":1,"PosX":`;
  - `2101/1` returns valid JSON with `ErrorCode=1`, meaning factory initialization failed/rejected;
  - sending both scaled coordinates such as `PosX=1184.0` and meter coordinates such as `PosX=1.184` still returned `ErrorCode=1` in the current unlocalized state.
- Manual/document check:
  - the developer manual defines `2101/1` `PosX/PosY/PosZ` in map-frame meters;
  - `tcp_bridge` was corrected so relocalization requests no longer apply the 1007 pose scale before sending `2101/1`.
- 106 factory localization check:
  - `localization.service` is running `/opt/robot/share/localization/bin/localization_ddsnode`;
  - it loads `/var/opt/robot/data/maps/active/occ_grid.yaml`;
  - `localization.rviz` publishes `2D Pose Estimate` to `/initialpose`;
  - `drddsctl list` on 106 shows factory-side subscribers on `rt/initialpose`.
- Direction change:
  - field web relocalization should primarily reproduce the 106 RViz `/initialpose` path;
  - 103 TCP `2101/1` is kept as a diagnostic path, but should not be the only success criterion.
- Code changes:
  - `m20pro_real_full.sh` now passes `enable_initialpose_relocalization:=false` by default so `tcp_bridge` does not intercept `/initialpose` and turn it into 103 TCP `2101/1`;
  - web relocalization verification now accepts success when localization is true, map pose is fresh, and the map pose is close to the requested initial pose;
  - verification still reports `tcp_2101` separately if a TCP result exists;
  - `tcp_protocol.py` now wraps malformed factory JSON with a readable `M20ProtocolError` containing response length and a short raw payload.
- Verification:
  - local `py_compile` passed for `tcp_protocol.py`, `tcp_bridge_node.py`, `web_dashboard_node.py`, `m20pro_real.launch.py`, and `m20pro.launch.py`;
  - extracted dashboard JavaScript passed `node --check`.
- Important limitation:
  - this change has not yet been validated by restarting/deploying the running 104 full system during the field session;
  - next field test should restart the full real service from the updated code, go to the mapped test area, drag the red scan overlay to match the map, click `执行重定位`, and check whether `factory_pose_accepted=true`.

## 2026-06-16 relocalization field failure follow-up

- Field result:
  - web relocalization still failed;
  - `/scan` overlay had live points, but verification showed `tcp_2101` failure and no confirmed factory pose update;
  - first boot attempt showed web preflight stuck until the frontend was reopened.
- Current conclusion:
  - the failure is not simply "arrow not accurate";
  - normal web relocalization should not depend on 103 TCP `2101/1`, because 106 RViz succeeds through `/initialpose`.
- Code changes:
  - real/full startup now forces tcp_bridge `enable_initialpose_relocalization=false` and `enable_initialpose_3d_relocalization=false` in the runtime params;
  - `m20pro.launch.py`, `m20pro_real.launch.py`, `m20pro.yaml`, and `m20pro_real.yaml` now default TCP initialpose forwarding to false;
  - project FastDDS UDP profile now whitelists `10.21.31.103`, `10.21.31.104`, and `10.21.31.106`, so 104-published `/initialpose` is not constrained to a 104-only interface list;
  - web relocalization treats 103 TCP result as diagnostic only, and top-level success now requires actual factory map pose update near the requested pose;
  - `/initialpose` publish repeats increased to 10 with 0.15 s interval to reduce cross-host DDS miss probability;
  - web preflight now has a run lock and frontend 15 s request timeout, so the UI should not stay stuck at "自检中".
- Added script:
  - `scripts/104_check_initialpose_to_106.sh x y yaw`;
  - use it to verify whether a `/initialpose` published on 104 is visible on 106.

## 2026-06-12 web relocalization verification feedback

- User noted that 106 RViz `2D Pose Estimate` can be rough and still converge because the factory localization uses the click as an initial guess and then aligns live pointcloud to the 106 active map.
- Current web relocalization chain remains factory-based:
  - web map drag -> `/api/localization/initialpose`;
  - web dashboard publishes `/initialpose`;
  - `m20pro_tcp_bridge` forwards it to the factory TCP API as `Type=2101`, `Command=1`, with `PosX/PosY/PosZ/Yaw`;
  - factory localization should then converge if the map, frame, pose guess, and active map are correct.
- Added verification feedback after web relocalization:
  - new parameter `relocalization_verify_timeout_s`, default `8.0`;
  - after publishing `/initialpose`, the web backend waits for `/m20pro_tcp_bridge/relocalization_result`;
  - it also checks whether localization, `/m20pro_tcp_bridge/map_pose`, `/scan`, local costmap, and global costmap become fresh/healthy;
  - the API response now includes `verification.request_accepted`, `verification.navigation_ready`, `verification.result`, and per-item `checks`.
- Frontend behavior:
  - clicking `执行重定位` now shows `已发送重定位请求，正在等待原厂回执和导航链路恢复...`;
  - it then displays the full verification JSON instead of only saying the request was published;
  - live topic updates are temporarily prevented from overwriting the API verification result.
- Synced to 104 and rebuilt `m20pro_cloud_bridge` and `m20pro_bringup`.
- No real/web processes were left running after the update.

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

## 2026-06-12 web dashboard usability pass

- Stopped the standalone web preview before editing.
- Improved the web task panel:
  - tasks can now be renamed through `/api/tasks/update`;
  - tasks can now be deleted through `DELETE /api/tasks?id=...`;
  - a running task cannot be deleted until it is stopped;
  - deleting a task does not delete map annotations.
- Added robot battery display:
  - subscribes to factory `/BATTERY_DATA` when `drdds/msg/BatteryData` is available;
  - parses two battery packs, percentage, voltage, current, remaining capacity, cycles, and average temperature;
  - standalone/local environments without `drdds` still start, with battery display disabled.
- Improved map viewport behavior:
  - desktop layout now fits the map panel within the browser viewport;
  - the map canvas recalculates size after live map or fixed map loads;
  - mobile/narrow layouts still scroll normally.
- Verified on 104:
  - `m20pro_cloud_bridge` builds successfully;
  - standalone web preview starts on `0.0.0.0:8080`;
  - `/healthz` returns OK;
  - page HTML contains the new battery/task controls;
  - `/api/state` reports live battery data from two packs;
  - stopped the preview afterward and confirmed no web/real process remained.

## 2026-06-12 web preflight integration

- Integrated the 104 preflight workflow into the web dashboard.
- Added a new `自检` tab:
  - button: `开始自检`;
  - displays each check item as pass/warn/fail;
  - shows raw JSON for debugging.
- Added API:
  - `GET /api/preflight` returns the latest result;
  - `POST /api/preflight/run` runs the checks and always returns HTTP 200 with full details, even when the preflight itself fails.
- Added web-side status display:
  - `/api/tasks` reports `last_preflight_ok` for display only;
  - task start buttons do not depend on a preflight validity window;
  - `/api/tasks/start` starts the selected task directly and does not run preflight automatically.
- Checks currently include:
  - core nodes;
  - key topics;
  - `/LIDAR/POINTS` freshness and point count;
  - `/scan` freshness and valid range count;
  - `/ODOM` finite pose;
  - `/m20pro_tcp_bridge/map_pose` finite pose;
  - `localization_ok`;
  - navigation status;
  - `/map`;
  - local/global costmap;
  - battery level;
  - Nav2 lifecycle states;
  - confirmed `move` motion mode.
- The web node now lightly subscribes to `/LIDAR/POINTS`, `/scan`, `/ODOM`, `/local_costmap/costmap`, and `/global_costmap/costmap` for metadata only; it does not forward or render heavy raw arrays.
- Verified on 104 in standalone web preview:
  - page starts normally;
  - `/api/preflight` returns null before first run;
  - `/api/preflight/run` returns a full failed checklist in preview mode, as expected because full real is not running;
  - battery is still read correctly;
  - `/api/tasks/start` was later corrected to stop running preflight automatically;
  - preview was stopped and no web/real process remained.

## 2026-06-12 preflight policy adjustment

- Removed the strict “latest preflight must be within 120 seconds” task gate.
- Rationale:
  - the robot is expected to support remote autonomous operation;
  - requiring a human to click task start within a short time window is too strict and does not match unattended execution.
- Current behavior:
  - web preflight is a manual operator check;
  - the operator clicks “开始自检” once and uses the result to decide whether to continue;
  - starting a web task does not automatically run another preflight;
  - failed manual preflight still returns the complete checklist to the web UI and API caller.
- Verified on 104 standalone web preview:
  - task buttons are not disabled merely because the previous preflight is old or failed;
  - `POST /api/tasks/start` no longer calls `/api/preflight/run`;
  - in preview mode, `/api/preflight/run` still reports failures as expected because full real is not running;
  - no web/real process remained after the preview was stopped.

## 2026-06-12 real autostart service

- Added a controlled systemd autostart path for 104:
  - `systemd/m20pro-real.service`;
  - `systemd/m20pro-real.default`;
  - `scripts/104_autostart_entrypoint.sh`;
  - `scripts/104_enable_autostart.sh`;
  - `scripts/104_disable_autostart.sh`;
  - `scripts/104_autostart_status.sh`.
- Intended behavior:
  - boot starts the full real stack and web dashboard on 104;
  - no inspection task starts automatically;
  - operator opens `http://10.21.31.104:8080`, runs one manual web preflight, then relocalizes, maps/selects map, marks points, and starts the task manually.
- Default mode is `move`, because the final field workflow needs the web task button to be able to move the robot. This only enables the motion-control chain; it does not dispatch a goal by itself.
- The service only starts this project stack and does not modify or restart factory multicast/FastDDS services.
- During validation, the first service start reached the web dashboard but `tcp_bridge` still logged `shadow mode; axis command disabled`.
  - Root cause: `m20pro_real.yaml` contains node-specific `m20pro_tcp_bridge.enable_axis_command: false`, which overrode the launch-level `enable_axis_command:=true` on Foxy.
  - Fix: `src/m20pro_bringup/scripts/m20pro_real_full.sh` now generates a runtime params file under `/tmp` and explicitly sets `m20pro_tcp_bridge.enable_axis_command` to `true` for `move`, `false` for `shadow`.
  - Rebuilt `m20pro_bringup` on 104.
- Verified after the fix:
  - `m20pro-real.service` starts successfully;
  - `http://127.0.0.1:8080/healthz` returns `{"ok":true}`;
  - runtime params show `enable_axis_command: true`;
  - `tcp_bridge` log shows `axis command enabled`;
  - service is enabled for next boot but was stopped after validation, leaving no real/web processes running.

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

## 2026-06-12 idle axis command fix

- User reported that while still at the desk area, the robot could only move briefly from the handheld controller and then stopped.
- Finding:
  - web preflight is read-only and does not send commands; it was not the direct cause;
  - `m20pro-real.service` had started the full real `move` stack;
  - `tcp_bridge` in move mode was sending axis commands at 20 Hz even when no ROS `/cmd_vel` was active;
  - after `/cmd_vel` timeout it repeatedly sent zero velocity, which can override handheld control.
- Fix:
  - added `send_idle_zero_commands` parameter to `tcp_bridge`, default `false`;
  - `tcp_bridge` now sends axis commands only after receiving `/cmd_vel`;
  - after command timeout it sends at most one zero command to stop robot-side ROS navigation, then stops sending idle zero commands;
  - added `send_idle_zero_commands: false` to `m20pro.yaml` and `m20pro_real.yaml`.
- Synced to 104 and rebuilt `m20pro_navigation` and `m20pro_bringup`.
- Verification on 104:
  - `m20pro-real.service` starts;
  - web health returns `{"ok":true}`;
  - tcp bridge log shows `axis command enabled; idle zero disabled`;
  - service was stopped after validation and no real/web processes remained.
- Operational note:
  - failing self-check at the desk is expected if the robot is not on the loaded test-site map; `/scan`, global costmap, and localization can fail there;
  - this should not block handheld driving after the idle-zero fix.

## 2026-06-12 consecutive real task reliability pass

- User concern:
  - after finishing the short-distance task and then running the next real task, frontend/Nav2 state may get stuck and force a robot reboot.
- Web dashboard task-session changes:
  - task start now first resets the previous navigation session by publishing `/m20pro/stop_task`, sending several zero `/cmd_vel` samples, clearing local/global costmaps, and publishing an idle active-waypoint state;
  - task stop and task completion run the same cleanup path;
  - waypoint reached now cancels the old Nav2 goal and sends zero velocity before dwell/advance, reducing old-goal carryover between points;
  - current waypoint goal is resent at a low rate if it appears to have been dropped or ignored;
  - task list now treats only `active_task.status=running` as the real running state, and stale historical task `running` status is auto-normalized to `stopped`;
  - added a web button `复位导航状态` for field recovery without rebooting the robot.
- Floor manager robustness:
  - a new same-floor goal can replace an active same-floor `floor_goal` instead of being rejected by stale `floor_mission_active`;
  - stair/cross-floor missions are still protected and are not automatically interrupted by replacement goals;
  - stale Nav2 goal request/result callbacks after stop/replacement are ignored, so an old cancel/reject/result cannot clear the new active mission.
- Verification:
  - local `py_compile` passed for `web_dashboard_node.py`, `floor_manager.py`, and `tcp_bridge_node.py`;
  - extracted frontend JavaScript passed `node --check`;
  - synced to 104 and rebuilt `m20pro_cloud_bridge`, `m20pro_navigation`, and `m20pro_bringup`;
  - 104 post-build `py_compile` passed;
  - `m20pro-real.service` remained inactive after verification.
- Field use:
  - if task 1 finishes and task 2 looks stuck, first click webpage `复位导航状态`, then refresh task list and start the next task;
  - this recovery path does not restart original multicast/FastDDS services and does not change factory fallback behavior.

## 2026-06-15 web relocalization scan overlay

- Field issue:
  - user can navigate from the web map, but web relocalization is still harder to use than the original 106 RViz workflow;
  - in RViz, after dragging `2D Pose Estimate`, live scan/pointcloud feedback appears immediately, so it is easy to see whether the estimate matches the map.
- Web dashboard change:
  - added `显示实时激光轮廓` in the `定位` tab;
  - backend subscribes to `/scan`, downsamples finite laser ranges, and sends lightweight 2D points to the browser;
  - frontend overlays the scan on the map:
    - red means the scan is projected using the unsent relocalization draft pose;
    - blue means the scan is projected using the current robot pose.
- Field use:
  - open the web `定位` tab;
  - drag an arrow on the map and rotate it;
  - adjust until the red laser outline aligns with wall/map structure;
  - then click `执行重定位`.
- Implementation note:
  - this uses the fused 2D `/scan`, not raw `/LIDAR/POINTS`, to keep the web page responsive;
  - web dashboard now subscribes to `/scan` with Best Effort QoS to match the project `pointcloud_fusion` publisher.

## 2026-06-15 waypoint landing mismatch audit

- User concern:
  - the robot may not be stopping at the same physical point that was selected on the web map.
- Checks performed against the currently running 104 web service:
  - current saved waypoints are `F20_patrol_1` at `(4.572, -0.227)` and `F20_patrol_2` at `(12.347, 0.272)`;
  - current robot map pose was near `(0.005, 0.000)`, so `F20_patrol_1` is about `4.57m` from the robot and `F20_patrol_2` about `12.35m`;
  - the live `/map` and built-in `builtin_F20` map have the same width, height, origin, resolution, and occupancy hash, so there is no evidence of map scale or origin mismatch;
  - both saved waypoints are in free cells and not on occupied/unknown map cells;
  - frontend `canvasToWorld()` and `worldToCanvas()` use the standard occupancy-grid conversion with y-axis flip for display only;
  - saved annotation pose is published unchanged as `/m20pro/floor_goal`;
  - `floor_manager` passes the same x/y/yaw to Nav2 `NavigateToPose` without offset, scale, or rotation.
- Likely causes still open:
  - localization error or drift after relocalization;
  - operator comparing the selected map point with the robot body/front instead of the base-center reference point;
  - web task layer marking waypoint reached too early.
- Change:
  - reduced web task `goal_reached_tolerance_m` default from `0.6m` to `0.3m`, closer to Nav2's `0.25m` xy goal tolerance;
  - this should reduce early task completion/cancel before the robot has reached the selected point.
- Next field validation:
  - record a short bag while running one waypoint;
  - compare selected waypoint pose, `/m20pro_tcp_bridge/map_pose` final pose, and video/frame reference;
  - if final pose is close in `/map` but looks wrong physically, the issue is relocalization/map alignment; if final pose is far in `/map`, inspect Nav2/controller behavior.

## 2026-06-17 104 frontend recovery before field test

- 104 factory lidar baseline was restored by robot reboot; `/LIDAR/POINTS` and `/LIDAR/POINTS2` are visible again.
- Re-synced the current workspace to 104 and rebuilt all five packages.
- Restored full real web stack on 104 through `m20pro-real.service` for this test:
  - service is `active` but still `disabled`, so it is not configured for automatic boot start in this recovery pass;
  - web health returns `{"ok":true}` at `http://10.21.31.104:8080/healthz`;
  - system check log reports `M20PRO REAL OK`.
- Fixed a relocalization regression in `m20pro_real_full.sh`:
  - full real launch now passes `enable_initialpose_relocalization:=true`;
  - the generated runtime params now keep `enable_initialpose_relocalization: true`;
  - runtime params verified on 104 also keep `enable_axis_command: true` and `send_idle_zero_commands: false`.
- DDS note:
  - FastDDS SHM warnings still appear in logs during startup, but this run completed and web/Nav2 became active;
  - do not clear `/dev/shm/fastrtps_*` while factory ROS/DDS participants are running.

## 2026-06-22 104 current health and assist-mode boundary

- Current 104/development dog status:
  - `m20pro-real.service` is `active/running`, enabled, and `NRestarts=0`;
  - full frontend is reachable at `http://10.21.31.104:8080`;
  - blocking web preflight passed with `failures=0`, `warnings=0`, and `navigation_ready=true`;
  - live web state showed fresh relay pointcloud, `/scan`, odom, pose, local costmap, global costmap, localization, and navigation status.
- Costmap warning root cause and current fix status:
  - recent startup logs show the lidar relay sample became ready first, then `m20pro_nav2_startup_gate` waited for prerequisites and only then started Nav2 lifecycle;
  - after the gate requested lifecycle startup, local/global costmaps subscribed to `/scan`, activated cleanly, and `M20PRO REAL OK` was reported;
  - this is the intended fix for the previous field symptom where Nav2/costmap came up before scan/map/localization were stable and self-check kept reporting costmap warnings.
- Lidar/DDS observation:
  - 104 currently exposes `/LIDAR/POINTS`, `/m20pro/lidar_points_relay`, and `/scan`; `/LIDAR/POINTS2` was not present in this check;
  - despite ROS CLI graph output being incomplete for pointcloud publisher discovery, the reliable runtime evidence is that the web dashboard and `pointcloud_fusion` are receiving fresh relay pointcloud and generating fresh `/scan`;
  - future field diagnostics should prefer web `/api/state`, `pointcloud_fusion` logs, and relay guard logs over a plain `ros2 topic info` shell unless the shell environment/DDS profile is known to match the service.
- Assist mode boundary:
  - manual review indicates the hand-controller usage mode is `Type=1101 Command=5` with `Mode=2` for auxiliary/assist mode, and status fields include `ControlUsageMode` plus `OOA`;
  - this is different from the gait command `Type=2 Command=23` with `GaitParam=12`, which is only a flat/agile gait parameter already tested through `/m20pro/gait_command assist`;
  - because the manual says axis control `Type=2 Command=21` only supports normal mode, the project must not automatically switch to `Mode=2` during Nav2 tasks until the auxiliary mode is field-tested.
- Local code boundary after this check:
  - usage-mode command support exists in code for later testing but defaults to disabled;
  - frontend auxiliary/normal/navigation mode buttons were removed for now, so the web UI does not provide an accidental `Mode=2` control path;
  - local checks passed: `py_compile`, extracted dashboard JavaScript `node --check`, `git diff --check`, and targeted `colcon build --symlink-install --packages-select m20pro_navigation m20pro_cloud_bridge m20pro_bringup`.

## 2026-06-22 preflight costmap warning hardening

- User-facing issue:
  - field tests repeatedly saw self-check costmap warnings, and that blocked relocalization/navigation work;
  - at the workstation or immediately after arriving at the test site, Nav2/costmap can legitimately be delayed by the startup gate until map, scan, localization, pose, and TF are ready.
- Web preflight hardening:
  - added a short baseline wait before evaluating preflight so the dashboard has time to cache fresh map/perception/battery/navigation status instead of sampling too early;
  - if the robot is unlocalized or explicitly in workstation mode, local/global costmap and Nav2 lifecycle are now reported as informational deferred checks rather than warnings;
  - after localization is valid, local/global costmap and Nav2 lifecycle remain strict navigation checks, so real field failures are still visible before moving.
- Assist-mode safety boundary remains unchanged:
  - no web button or automatic behavior switches the robot into vendor auxiliary mode;
  - usage-mode command support stays disabled by default until auxiliary mode is tested deliberately.
- Verification:
  - local `py_compile` passed for web dashboard, TCP bridge, and Nav2 startup gate;
  - extracted dashboard JavaScript passed `node --check`;
  - `git diff --check` passed;
  - targeted build passed: `colcon build --symlink-install --packages-select m20pro_navigation m20pro_cloud_bridge m20pro_bringup`;
  - live 104 check still reports `m20pro-real.service` active/running with `NRestarts=0`, `/dev/shm` 58%, and blocking preflight `failures=0`, `warnings=0`, `navigation_ready=true`.

## 2026-06-22 deployed costmap preflight hardening to 104

- Deployment:
  - stopped `m20pro-real.service` on 104 before syncing code;
  - cleaned root-owned remote `build/install/log` artifacts left by the root systemd runtime;
  - ran `scripts/local_deploy_to_test_robot.sh user@10.21.31.104 /home/user/m20pro_ros2_ws`;
  - remote `colcon build --symlink-install` completed for all five packages.
- Restart verification:
  - reset the failed service state caused by the intentional SIGINT stop, then started `m20pro-real.service`;
  - service is `active/running`, `NRestarts=0`, `ExecMainPID=18966`;
  - `/dev/shm` usage is 58%.
- Runtime evidence:
  - relay guard reported `LIDAR relay sample OK`;
  - `m20pro_nav2_startup_gate` waited for `/map`, fresh `/scan`, and `localization_ok=true`, then requested lifecycle startup;
  - local/global costmaps subscribed to `/scan`, activated, and Nav2 lifecycle became active;
  - `m20pro_system_check` reported `M20PRO REAL OK`.
- Web preflight after deployment:
  - blocking `/api/preflight/run` returned `ok=true`, `navigation_ready=true`, `relocalization_ready=true`;
  - `failures=0`, `warnings=0`, `navigation_warnings=0`;
  - `lidar_points`, `/scan`, local costmap, and global costmap were all fresh within 1s.
- Assist-mode safety:
  - deployed code still has no `/api/usage_mode` route or frontend usage-mode buttons;
  - `enable_usage_mode_command` remains `false` in both config and TCP bridge defaults;
  - status-only parsing of `ControlUsageMode`/`OOA` is present for later observation.

## 2026-06-22 read-only field diagnosis script

- Added `scripts/104_diagnose_preflight.sh` as a field-friendly diagnosis entrypoint.
- The script is explicitly read-only:
  - calls web `/healthz`, `/api/state`, and blocking `/api/preflight/run`;
  - lists selected ROS topics/nodes in an isolated ROS shell;
  - prints systemd state, `/dev/shm` usage, usage-mode safety flags, and recent relevant logs;
  - does not publish `/cmd_vel`, gait commands, usage-mode commands, or any relocalization request.
- Updated `scripts/README.md`:
  - added `./scripts/104_diagnose_preflight.sh` to common 104 commands;
  - clarified that web preflight now treats pre-localization costmap/Nav2 delay as an informational deferred state instead of a WARN that blocks relocalization.
- Deployed the script to 104 without restarting the running service.
- Verification on 104:
  - script completed through `==== Done ====`;
  - service `active/running`, enabled, `NRestarts=0`;
  - web health OK;
  - web state showed fresh relay pointcloud, `/scan`, local/global costmaps, localization, and navigation status;
  - blocking preflight reported `failures=0`, `warnings=0`, `navigation_warnings=0`;
  - selected ROS graph included `/LIDAR/POINTS`, `/m20pro/lidar_points_relay`, `/scan`, local/global costmap topics, and `m20pro_nav2_startup_gate`;
  - usage-mode safety check reported `enable_usage_mode_command: false` and no web usage-mode control route/button.

## 2026-06-22 assist-mode read-only display polish

- Purpose:
  - keep the hand-controller auxiliary/assist mode strictly read-only in the web stack until it is field-tested;
  - make future observation understandable if the handler or vendor status reports `ControlUsageMode`/`OOA`.
- Web display change:
  - added frontend-only mapping for vendor usage mode:
    - `0` -> 常规;
    - `1` -> 导航;
    - `2` -> 辅助.
  - added frontend-only mapping for auxiliary obstacle avoidance `OOA`:
    - `0` -> 未启动;
    - `1` -> 空闲中;
    - `2` -> 未触发避障;
    - `3` -> 主动避障中.
- Safety boundary:
  - no `/api/usage_mode` route was added;
  - no frontend usage-mode button was added;
  - `enable_usage_mode_command` remains `false` in configs and TCP bridge defaults.
- Verification:
  - local `py_compile` passed for web dashboard, TCP bridge, and Nav2 startup gate;
  - extracted dashboard JavaScript passed `node --check`;
  - `git diff --check` passed;
  - targeted build passed for `m20pro_cloud_bridge`, `m20pro_navigation`, and `m20pro_bringup`;
  - deployed to 104, rebuilt all five packages, restarted `m20pro-real.service`;
  - 104 service is `active/running`, `NRestarts=0`, `/dev/shm` 58%;
  - blocking web preflight after deployment reported `failures=0`, `warnings=0`, `navigation_warnings=0`;
  - diagnostic script confirmed `enable_usage_mode_command: false` and no web usage-mode control route/button.

## 2026-06-22 development dog current issue sweep

- Current 104 runtime check:
  - connected robot is the development dog at `10.21.31.104`;
  - `m20pro-real.service` is `active/running`, enabled, and `NRestarts=0`;
  - web health is OK at `http://10.21.31.104:8080/healthz`;
  - blocking web preflight returned `ok=true`, `failures=0`, `warnings=0`, `navigation_warnings=0`;
  - relay pointcloud, `/scan`, odom, map pose, `/map`, local costmap, and global costmap were all fresh enough for navigation readiness.
- Runtime cleanup:
  - `_ros2_daemon` was consuming about one CPU core; stopped it with `ros2 daemon stop`;
  - project service and web preflight stayed healthy afterward.
- Development dog git/network findings:
  - `/home/user/m20pro_ros2_ws` is a symlink to `/home/user/m20pro_ros2_ws_20260529_173921`;
  - that deployed directory is not a git worktree, so direct `git pull` cannot work there even if internet is fixed;
  - current 104 network has only `10.21.31.104/24` on `eth0`, no default route, and no DNS;
  - a temporary test route through `10.21.31.103` was attempted because 103 is reachable, but 104 still could not reach `8.8.8.8` or resolve `github.com`;
  - the temporary route/DNS was removed afterward to restore the original robot LAN-only state;
  - SSH to 103 was not available from this session, so the 103 Wi-Fi/NAT/dnsmasq gateway could not be repaired here.
- Lidar topic finding:
  - current development dog ROS graph shows `/LIDAR/POINTS`, `/m20pro/lidar_points_relay`, and `/scan`;
  - `/LIDAR/POINTS2` is not present on this development dog at the time of the check;
  - the active navigation chain is `/LIDAR/POINTS -> /m20pro/lidar_points_relay -> /scan`;
  - plain `ros2 topic info /LIDAR/POINTS` can report publisher count `0` while relay/fusion and web state still receive live data, so use web `/api/state`, relay status, and pointcloud fusion status as stronger evidence.
- Map finding:
  - `/home/user/m20pro_maps` is empty, but project builtin maps are available through `/api/maps`;
  - current selected map is `builtin_F20`, and F20 PCD-derived terrain data is ready;
  - if the browser map panel appears empty while `/api/maps` is valid, suspect frontend rendering/cache/state before assuming backend map files are missing.
- New regression guard:
  - added `scripts/check_preflight_policy.py`;
  - it verifies that unlocalized/workstation costmap and Nav2 lifecycle checks remain informational, not navigation warnings;
  - it verifies that localized navigation readiness still counts `fail/warn`, not `info`;
  - it verifies that the web stack has no `/api/usage_mode`, no `data-usage-mode`, and no `setUsageMode` button/control path;
  - it verifies `enable_usage_mode_command: false` remains in real/sim config and TCP bridge defaults;
  - it verifies Nav2 startup gate installation, disabled Nav2 autostart, and `always_send_full_costmap: true`.
- Field diagnosis update:
  - enhanced `scripts/104_diagnose_preflight.sh` with read-only network/git readiness checks;
  - it now prints IPs, routes, DNS, git repo status, lidar topic info for `/LIDAR/POINTS` and `/LIDAR/POINTS2`, pointcloud fusion status, and relay status;
  - this should make future self-check failures easier to classify as network/git/deploy, lidar/DDS, startup gate, or real Nav2/costmap readiness.
- Terminal preflight update:
  - updated `scripts/104_preflight_check.sh` wording so workstation/unlocalized Nav2/costmap delay is treated as a deferred state;
  - `/scan` absence is now a real failure even at the workstation, matching the user's requirement that missing scan/lidar must not be explained away by being at the desk.
- Verification:
  - `python3 scripts/check_preflight_policy.py` passed;
  - `bash -n scripts/104_diagnose_preflight.sh scripts/104_preflight_check.sh` passed;
  - 104 service health was rechecked after the temporary network test and remained `active/running`, `NRestarts=0`, web health OK.

## 2026-06-22 optional second lidar relay/fusion hardening

- Reason:
  - user reported that factory documentation/expectation has front and rear lidar streams (`/LIDAR/POINTS` and `/LIDAR/POINTS2`);
  - development dog 104 currently publishes only `/LIDAR/POINTS`, but field/test dogs may expose `/LIDAR/POINTS2`;
  - navigation and obstacle avoidance should use the second lidar when it exists, while not blocking startup on robots where it is absent.
- Pointcloud fusion change:
  - `m20pro_pointcloud_fusion` now keeps primary and backup pointcloud ranges separately;
  - when both primary and backup clouds are fresh, `/scan` is produced by taking the per-angle nearest obstacle (`np.minimum`);
  - when only one side is fresh, `/scan` continues from that side instead of dropping scan output;
  - diagnostics now report backup topic, per-topic message counts, primary/backup freshness, and backup source age.
- Startup change:
  - full real startup still requires the primary relay `/LIDAR/POINTS -> /m20pro/lidar_points_relay`;
  - it now optionally tries `/LIDAR/POINTS2 -> /m20pro/lidar_points2_relay` with a short wait;
  - if the second relay receives a sample, `backup_cloud_topic` is passed into the real launch and fused into `/scan`;
  - if `/LIDAR/POINTS2` is absent or silent, startup logs a warning and continues with the primary lidar only.
- Launch/config change:
  - `m20pro.launch.py` and `m20pro_real.launch.py` now both declare and forward `backup_cloud_topic`;
  - autostart defaults include:
    - `M20PRO_ENABLE_LIDAR2_RELAY=1`;
    - `M20PRO_LIDAR2_TOPIC=/LIDAR/POINTS2`;
    - `M20PRO_LIDAR2_RELAY_TOPIC=/m20pro/lidar_points2_relay`;
    - `M20PRO_LIDAR2_RELAY_WAIT_S=8`.
- Diagnostics/docs:
  - `104_diagnose_preflight.sh` now lists `/m20pro/lidar_points2_relay` and `/m20pro/lidar_relay2/status`;
  - runtime snapshot includes the second relay topic when present;
  - scripts README explains that second lidar is optional and should not block boot/preflight if absent.
- Verification:
  - `python3 -m py_compile` passed for `pointcloud_fusion.py`, `m20pro_real.launch.py`, and `m20pro.launch.py`;
  - `bash -n` passed for changed startup/diagnosis/autostart scripts;
  - `python3 scripts/check_preflight_policy.py` passed and now guards backup lidar wiring;
  - `git diff --check` passed;
  - targeted build passed: `colcon build --symlink-install --packages-select m20pro_navigation m20pro_bringup`.
- 104 deployment/bugfix notes:
  - first deployment exposed a real startup bug when `/LIDAR/POINTS2` was absent: passing `backup_cloud_topic:=` with an empty value made ROS launch fail with `malformed launch argument`;
  - fixed by only appending `backup_cloud_topic:=...` when the optional second relay actually receives a sample;
  - second deployment exposed a relay cleanup bug: stopping the optional second relay could match relay command lines by substring, so `/LIDAR/POINTS` could be confused with `/LIDAR/POINTS2`;
  - fixed relay guard matching so start/stop locate relay processes by exact `input_topic:=...` and `output_topic:=...` argument tokens;
  - after the fix, development dog 104 recovered with only the primary relay running, `pointcloud_fusion` receiving primary clouds, `/scan` publishing, Nav2 startup gate requesting lifecycle startup, and local/global costmaps subscribing to `/scan`;
  - blocking preflight on 104 returned `ok=true`, `navigation_ready=true`, `relocalization_ready=true`, `failures=0`, `warnings=0`, `navigation_warnings=0`;
  - this verifies the current development dog behavior: missing `/LIDAR/POINTS2` is informational only and no longer breaks the primary lidar/costmap chain.

## 2026-06-22 developer-shell pointcloud visibility hardening

- Problem found on the development dog:
  - `m20pro-real.service` runs the real stack as `root`;
  - plain `user` ROS CLI could receive `/scan`, but could not receive `/LIDAR/POINTS` or `/m20pro/lidar_points_relay`;
  - the same probe as `root` could receive both raw lidar and relay pointclouds;
  - when the `user` shell exported the project UDP-only FastDDS profile, it could receive `/m20pro/lidar_points_relay` again.
- Interpretation:
  - current navigation and obstacle avoidance were healthy because `/scan` was fresh and web preflight was green;
  - the recurring "topic exists but echo has no pointcloud samples" symptom can be a root/user FastDDS SHM/profile split, not necessarily a lidar outage;
  - missing `/scan` remains a real fault and must not be explained away by being at the workstation.
- Script hardening:
  - `scripts/104_diagnose_preflight.sh` now auto-runs the current local script on `user@10.21.31.104` when launched from an upper computer, avoiding accidental local service/8080 checks;
  - the diagnose script now exports the project UDP-only FastDDS profile for ROS CLI probes and runs a small Python subscriber for relay pointcloud, `/scan`, fusion status, and relay status;
  - Foxy-incompatible and misleading `ros2 topic echo` sampling was removed from the diagnose script; status samples now use a Python subscriber too;
  - recent log output now summarizes camera RTSP failures separately and filters them out of the navigation/lidar log tail, so camera noise does not hide costmap or pointcloud evidence;
  - `scripts/104_preflight_check.sh` now also auto-runs on 104 from an upper computer and exports the project UDP-only FastDDS profile before terminal topic checks;
  - terminal preflight topic sampling now uses Python subscribers by message type instead of `ros2 topic echo`, covering relay pointcloud, `/scan`, pose, odom, localization, and navigation status;
  - terminal preflight now treats `/scan` with no data as a hard failure even at the workstation, while relay pointcloud CLI failure is a warning if the web `/scan` chain is fresh.
- Documentation/guardrails:
  - `scripts/README.md` documents the remote execution behavior and the UDP FastDDS observation profile;
  - `scripts/check_preflight_policy.py` now guards these diagnostics so future changes do not reintroduce unsupported Foxy echo flags or downgrade missing `/scan` to a workstation warning.

## 2026-06-22 pointcloud load reduction and assist default rollback

- Reason:
  - raw `/LIDAR/POINTS` clouds often contain tens of thousands of points at high rate;
  - relaying full clouds through DDS and then parsing them again in Python keeps the robot host under sustained pressure and contributes to `/dev/shm`/DDS stress;
  - the hand-controller auxiliary/assist mode has not been field-tested, so project task flow must not automatically request assist-like gait behavior.
- Lidar relay load reduction:
  - `m20pro_lidar_relay` now supports `max_output_points` and `min_publish_interval_s`;
  - the default relay output is capped to about 12000 points per cloud and 0.1 s minimum publish interval;
  - the relay status now reports input/output point counts, stride, published/skipped message counts, and input/output bytes;
  - raw input subscription still uses the factory DDS profile so the relay can see `/LIDAR/POINTS`, but downstream project consumers receive a lighter `/m20pro/lidar_points_relay`.
- Fusion load reduction:
  - real pointcloud fusion now processes at most 6000 points per relay cloud;
  - `/scan` remains at 10 Hz, but `publish_on_cloud_update` is disabled so cloud callbacks do not trigger extra scan publications beyond the timer;
  - this keeps obstacle information dense enough for the 1-degree scan while reducing Python/Numpy work.
- Autostart defaults:
  - `104_enable_autostart.sh` and `systemd/m20pro-real.default` now record:
    - `M20PRO_LIDAR_RELAY_MAX_OUTPUT_POINTS=12000`;
    - `M20PRO_LIDAR_RELAY_MIN_PUBLISH_INTERVAL_S=0.1`.
- Assist safety rollback:
  - `floor_manager` default `flat_gait_label` is back to `flat`;
  - real launch also passes `flat_gait_label=flat`;
  - `gait_assist_param=12` remains available for explicit future testing, but normal Nav2/floor tasks no longer automatically request it.

## 2026-06-22 local workspace cleanup

- Cleaned only low-risk generated files in the local development workspace:
  - removed ROS/colcon generated directories `build/`, `install/`, and `log/`;
  - removed Python `__pycache__`/`.pyc` cache files;
  - ran `git gc --prune=now` to pack loose Git objects.
- Size result on the upper-computer workspace:
  - before cleanup: about 180M;
  - after cleanup: about 46M;
  - `.git` was reduced from about 120M to about 5.6M.
- Files intentionally kept:
  - `src/m20pro_bringup/maps/Original_map/full_cloud.pcd`, because `m20pro.yaml`, `m20pro_real.yaml`, and `map_manifest.yaml` reference it;
  - `src/m20pro_inspection/models/playphone_bg_best_rk3588_int8.rknn`, because real/sim launch defaults reference it as the inspection model.
- Note:
  - after deleting `install/`, rebuild locally before running local launch commands:
    `colcon build --symlink-install`.

## 2026-06-22 real-only repository split

- This workspace is now the real-only project:
  - path: `/home/fabu/桌面/M20Pro/m20pro_real_ros2_ws`;
  - split commit: `8ed14ae chore: split real project`.
- Scope kept here:
  - 104/GOS field deployment scripts;
  - full real launch and web frontend;
  - lidar relay/downsampling and `/scan` fusion for the real robot;
  - preflight, autostart, systemd service, 103/106 integration, and inspection nodes.
- Scope removed from here:
  - laptop-only simulation launch/config/RViz/nodes;
  - the old unified `m20pro.launch.py mode:=...` entry.
- Validation at split time:
  - `git diff --check`;
  - Python compile for real launch, web dashboard, navigation, cloud bridge, and inspection modules;
  - shell syntax checks for root scripts and package scripts;
  - `python3 scripts/check_preflight_policy.py`.
- The sibling sim project is `/home/fabu/桌面/M20Pro/m20pro_sim_ros2_ws`.

## 2026-06-22 current real navigation and obstacle-avoidance architecture

This section is a stable architecture note for future AI assistants and for interview/project review. It summarizes the current real-robot navigation stack as it exists after the real/sim split.

### One-line architecture

The real stack converts factory lidar pointclouds into a lightweight 2D `/scan`, feeds `/scan` into Nav2 local/global costmaps, uses Navfn A* for global planning and DWB for local trajectory sampling/control, then sends `/cmd_vel` through the 103 TCP bridge.

### Runtime data flow

```text
/LIDAR/POINTS
  -> m20pro_lidar_relay
     - subscribes with factory FastDDS profile
     - down-samples by stride
     - limits publish rate
/m20pro/lidar_points_relay
  -> m20pro_pointcloud_fusion
     - optional backup input: /m20pro/lidar_points2_relay
     - converts pointcloud to 360-degree LaserScan
/scan
  -> Nav2 local_costmap and global_costmap obstacle layers
  -> Nav2 planner/controller
/cmd_vel
  -> m20pro_tcp_bridge
  -> 103 robot TCP motion interface
```

### Lidar relay and pointcloud reduction

- Current relay node: `src/m20pro_navigation/m20pro_navigation/lidar_relay_node.py`.
- Purpose:
  - avoid making every downstream node subscribe directly to heavy factory `/LIDAR/POINTS`;
  - reduce DDS traffic, memory copies, CPU pressure, and `/dev/shm` stress;
  - keep one long-lived relay as the controlled bridge between factory DDS and project DDS consumers.
- Current method:
  - this is not clustering;
  - it is stride sampling plus rate limiting;
  - example: if input cloud has about 48000 points and `max_output_points=12000`, stride is about 4, so roughly every fourth point is copied.
- Current default limits:
  - `M20PRO_LIDAR_RELAY_MAX_OUTPUT_POINTS=12000`;
  - `M20PRO_LIDAR_RELAY_MIN_PUBLISH_INTERVAL_S=0.1`;
  - relay output is therefore capped around 12000 points per message and about 10 Hz.
- Why not clustering here:
  - the navigation chain needs nearest obstacle distance per direction, not object identities;
  - DBSCAN/Euclidean clustering costs more CPU and has sensitive parameters;
  - clustering can accidentally remove thin poles, edges, legs, or stair-related geometry;
  - stride sampling is cheap, predictable, and mainly solves the current stability bottleneck.

### Pointcloud to LaserScan fusion

- Current fusion node: `src/m20pro_navigation/m20pro_navigation/pointcloud_fusion.py`.
- Current real config: `src/m20pro_bringup/config/m20pro_real.yaml`.
- Main parameters:
  - `input_cloud_topic: /m20pro/lidar_points_relay`;
  - `backup_cloud_topic: /m20pro/lidar_points2_relay`;
  - `output_scan_topic: /scan`;
  - `min_angle: -3.14159`;
  - `max_angle: 3.14159`;
  - `angle_increment: 0.0174533`, about 1 degree;
  - `min_range: 0.2`;
  - `max_range: 10.0`;
  - `height_min: -0.25`;
  - `height_max: 0.60`;
  - `robot_radius: 0.28`;
  - `publish_rate_hz: 10.0`;
  - `max_points_per_cloud: 6000`;
  - `min_cloud_interval_s: 0.05`;
  - `publish_on_cloud_update: false`.
- Actual point filtering meaning:
  - points below `height_min` or above `height_max` do not enter `/scan`;
  - points too near/far do not enter `/scan`;
  - because `robot_radius=0.28`, the effective near filter is about 0.28 m, not 0.2 m;
  - accepted points are projected into 2D polar bins;
  - each angle bin keeps the nearest obstacle distance.
- If both primary and second lidar relays are fresh:
  - fusion computes primary and backup scan ranges separately;
  - final `/scan` takes per-angle nearest obstacle by `np.minimum`;
  - if the second lidar is absent, startup continues with the primary lidar only.

### Nav2 global planning

- Config file: `src/m20pro_bringup/config/nav2_params_real.yaml`.
- Planner:
  - `planner_plugins: ["GridBased"]`;
  - `plugin: nav2_navfn_planner/NavfnPlanner`;
  - `use_astar: true`;
  - `allow_unknown: true`;
  - `tolerance: 0.5`.
- Meaning:
  - global planning is grid-based 2D planning over the map/global costmap;
  - A* computes a path from current pose to the goal;
  - this layer answers "where should the robot go globally?" rather than doing close-range collision control.

### Nav2 local controller

- Controller:
  - `plugin: dwb_core::DWBLocalPlanner`;
  - `controller_frequency: 20.0`;
  - `max_vel_x: 0.55`;
  - `max_vel_theta: 1.10`;
  - `vx_samples: 16`;
  - `vtheta_samples: 28`;
  - `sim_time: 1.2`.
- DWB behavior:
  - samples candidate velocity commands;
  - simulates short future trajectories;
  - scores them with critics;
  - chooses the best valid `/cmd_vel`.
- Current critics:
  - `RotateToGoal`;
  - `Oscillation`;
  - `BaseObstacle`;
  - `GoalAlign`;
  - `PathAlign`;
  - `PathDist`;
  - `GoalDist`.
- Important interpretation:
  - DWB is a local trajectory sampler/scorer;
  - it does not "understand" objects;
  - it uses local/global costmap costs to avoid obstacles while trying to stay near the global path and goal direction.

### Local costmap

- Local costmap parameters:
  - `rolling_window: true`;
  - `width: 5`;
  - `height: 5`;
  - `resolution: 0.05`;
  - `robot_radius: 0.25`;
  - `plugins: ["obstacle_layer", "inflation_layer"]`.
- Meaning:
  - rolling 5 m x 5 m window centered around the robot;
  - about 2.5 m in every direction from the robot;
  - 5 cm grid resolution;
  - roughly 100 x 100 cells.
- Obstacle layer:
  - source topic: `/scan`;
  - `data_type: LaserScan`;
  - `obstacle_range: 3.0`;
  - `raytrace_range: 3.5`;
  - `observation_persistence: 0.3`;
  - `clearing: true`;
  - `marking: true`.
- Inflation layer:
  - `inflation_radius: 0.40`;
  - `cost_scaling_factor: 6.0`.
- Practical meaning:
  - `/scan` itself can represent obstacles up to 10 m;
  - local costmap only keeps a 5 m x 5 m local window;
  - obstacle marking is mainly within 3 m;
  - obstacles are expanded by about 40 cm so DWB avoids driving too close.

### Global costmap

- Global costmap plugins:
  - `static_layer`;
  - `obstacle_layer`;
  - `inflation_layer`.
- Current global costmap also consumes `/scan`:
  - `obstacle_range: 3.0`;
  - `raytrace_range: 3.5`;
  - `observation_persistence: 0.3`;
  - `inflation_radius: 0.40`.
- Meaning:
  - static map gives the known building layout;
  - `/scan` can add short-lived dynamic obstacle marks;
  - Navfn can re-plan around nearby dynamic obstacles that enter the global costmap.

### Startup gate and readiness logic

- Nav2 is launched with `autostart: False`.
- `m20pro_nav2_startup_gate` waits for:
  - `/map`;
  - fresh `/scan`;
  - `localization_ok=true`;
  - valid `/m20pro_tcp_bridge/map_pose`;
  - usable TF between `map` and `m20pro_base_link`.
- Purpose:
  - avoid starting Nav2/costmaps while the robot is still unlocalized;
  - reduce misleading self-check failures;
  - make missing `/scan` a real fault, not something explained away by workstation testing.

### Interview-ready explanation

If asked "How does your robot navigate and avoid obstacles?", the concise answer is:

> We take the factory 3D lidar pointcloud, relay it through a long-lived project node, down-sample and rate-limit it to reduce DDS and CPU pressure, then convert it into a 360-degree LaserScan. Nav2 uses this `/scan` in local and global costmaps. Global planning is Navfn with A*, while local control uses DWB to sample and score candidate trajectories against the costmap, path alignment, goal alignment, oscillation, and obstacle costs. The local costmap is a 5 m by 5 m rolling window at 5 cm resolution, with 3 m obstacle marking and 40 cm inflation. We also added a startup gate so Nav2 only activates after map, scan, localization, pose, and TF are ready.

If asked "Why not cluster the pointcloud?", the answer is:

> For the Nav2 costmap path, clustering is not necessary because the controller mainly needs nearest obstacle distance by direction, not object identity. Clustering would add CPU cost and parameter sensitivity, and could accidentally remove thin obstacles. We therefore use cheap stride sampling and LaserScan binning first. If future perception needs object-level reasoning, clustering should be a separate perception branch, not the base safety chain.

If asked "What was the engineering difficulty?", the answer is:

> The hard part was not just running Nav2. The factory lidar topic was large and DDS-sensitive on 104, sometimes visible in the graph but not delivering samples to project subscribers. We stabilized the chain by introducing a long-lived lidar relay, reducing pointcloud load, adding diagnostics, making `/scan` the primary health signal, and gating Nav2 startup until perception and localization were actually ready.

If asked "What would you optimize next?", the answer is:

> The next optimization is not generic Python multi-threading. Better options are earlier reduction, smarter angle-bin compression before publishing, reducing unused far-range work for local avoidance, and eventually moving pointcloud-to-scan fusion to C++ if profiling shows Python/Numpy is still the bottleneck.
