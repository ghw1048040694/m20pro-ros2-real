# U360 雷达巡检技术操作文档

本文档说明 M20Pro 机器狗接入昂锐 U360 雷达后的本地测试、真机部署、前端操作、接口对接和常见问题处理流程。

## 1. 功能范围

当前雷达模块负责：

- 读取机器狗任务点 `/m20pro/active_waypoint`。
- 在任务点进入 `dwelling` 阶段后自动触发雷达扫描。
- 支持实测实量 `measuring` 和点云建模 `modeling` 两种模式。
- 保存原始结果、摘要结果、点云下载文件、人工点云登记和人工测量回填。
- 在 Web 前端展示雷达状态和任务结果。
- 提供 JSON/CSV 导出接口，便于后续接入甲方平台。

导航、路径规划、避障、重定位、Nav2 和运动控制仍由原系统负责，雷达模块不直接控制这些核心链路。

## 2. 系统架构

整体链路：

```text
前端创建任务点
  -> 机器狗导航到点
  -> Web/任务系统发布 /m20pro/active_waypoint
  -> m20pro_radar_inspection 触发 U360 扫描
  -> U360 HTTP 接口返回状态/结果
  -> 保存文件并发布 /m20pro/radar_inspection/*
  -> Web 前端展示和导出
```

关键 topic：

```text
/m20pro/active_waypoint
/m20pro/radar_inspection/status
/m20pro/radar_inspection/result
/m20pro/radar_inspection/events
```

关键 Web API：

```text
GET  /api/radar/status
GET  /api/radar/results?task_id=<task_id>
GET  /api/radar/result?radar_task_id=<radar_task_id>
GET  /api/radar/task?task_id=<task_id>
GET  /api/radar/task_export?task_id=<task_id>&format=json
GET  /api/radar/task_export?task_id=<task_id>&format=csv
POST /api/radar/artifact
POST /api/radar/manual_measurement
```

## 3. 任务点字段

任务点除了坐标外，还需要带业务字段：

| 字段 | 说明 |
| --- | --- |
| `building` | 楼栋 |
| `unit` | 单元 |
| `house` | 户号 |
| `floor` | 楼层 |
| `area` | 区域 |
| `room` | 房间或部位 |
| `scan_point` | 扫描点编号 |
| `result_file_prefix` | 结果文件名前缀 |
| `radar.enabled` | 是否启用雷达 |
| `radar.scans` | 当前点位要执行的扫描计划 |

示例：

```json
{
  "phase": "dwelling",
  "task_id": "task_001",
  "index": 0,
  "waypoint": {
    "id": "wp_living_p01",
    "label": "客厅P01",
    "building": "3栋",
    "unit": "1单元",
    "house": "2008户",
    "floor": "F20",
    "area": "东区",
    "room": "客厅",
    "scan_point": "P01",
    "result_file_prefix": "3栋_1单元_2008户_F20_客厅_P01",
    "manual_point_type": "task",
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

## 4. 本地 dry-run 测试

本地 dry-run 不需要真实雷达，用于验证任务触发、命名、保存、展示和导出。

终端 1，启动 Web：

```bash
cd ~/m20pro-ros2-real
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch m20pro_bringup m20pro_web_dashboard.launch.py \
  port:=8080 \
  wait_for_radar_inspection:=true \
  radar_results_dir:=/tmp/m20pro_radar_test
```

终端 2，启动雷达 dry-run：

```bash
cd ~/m20pro-ros2-real
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch m20pro_radar_inspection m20pro_radar_inspection.launch.py \
  backend:=dry_run \
  dry_run_duration_s:=2.0 \
  output_dir:=/tmp/m20pro_radar_test
```

终端 3，模拟机器狗到达任务点：

```bash
cd ~/m20pro-ros2-real
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /m20pro/active_waypoint std_msgs/msg/String "{data: '{\"phase\":\"dwelling\",\"task_id\":\"task_demo_001\",\"index\":0,\"waypoint\":{\"id\":\"wp_demo_001\",\"label\":\"客厅P01\",\"building\":\"3栋\",\"unit\":\"1单元\",\"house\":\"2008户\",\"floor\":\"F20\",\"area\":\"东区\",\"room\":\"客厅\",\"scan_point\":\"P01\",\"result_file_prefix\":\"3栋_1单元_2008户_F20_客厅_P01\",\"manual_point_type\":\"task\",\"radar\":{\"enabled\":true,\"scans\":[{\"mode\":\"measuring\",\"label\":\"实测实量\",\"result_suffix\":\"measure\"},{\"mode\":\"modeling\",\"label\":\"点云建模\",\"result_suffix\":\"cloud\",\"artifact_policy\":\"manual_import\",\"manual_measure_required\":true}]}}}'}"
```

检查输出：

```bash
find /tmp/m20pro_radar_test -maxdepth 3 -type f | sort
```

正常应看到：

```text
jobs/*.json
raw/*_measure_*.json
raw/*_cloud_*.json
summaries/*_measure_*.json
summaries/*_cloud_*.json
```

打开前端：

```text
http://127.0.0.1:8080
```

## 5. 真机部署前检查

真机测试前确认：

1. 代码已部署到机器狗工作区，例如：

```text
/home/user/m20pro_real_ros2_ws
```

2. 在机器狗上构建：

```bash
cd /home/user/m20pro_real_ros2_ws
source /opt/robot/scripts/setup_ros2.sh
source install/setup.bash
colcon build --packages-up-to m20pro_bringup --symlink-install
```

3. 雷达接入机器狗网络。

如果机器狗开机后只能提供自己的 WiFi，雷达需要连接机器狗 WiFi，或者通过有线、路由器、双网卡等方式保证机器狗能访问雷达。

4. 在机器狗上确认雷达 HTTP 可达：

```bash
curl -v --max-time 5 http://雷达IP:8080
```

这个不通时，ROS 和前端都无法控制雷达。

## 6. 真机启动配置

手动启动 real：

```bash
cd /home/user/m20pro_real_ros2_ws
source /opt/robot/scripts/setup_ros2.sh
source install/setup.bash

M20PRO_ENABLE_RADAR_INSPECTION=true \
M20PRO_RADAR_BACKEND=u360_http \
M20PRO_RADAR_SCAN_MODE=measuring \
M20PRO_RADAR_DEVICE_URL=http://雷达IP:8080 \
M20PRO_RADAR_OUTPUT_DIR=/home/user/m20pro_radar_results \
./scripts/104_start_real_move.sh
```

如果现场不允许运动，先用 shadow：

```bash
M20PRO_ENABLE_RADAR_INSPECTION=true \
M20PRO_RADAR_BACKEND=u360_http \
M20PRO_RADAR_DEVICE_URL=http://雷达IP:8080 \
./scripts/104_start_real_shadow.sh
```

## 7. 开机自启动配置

机器狗使用 `m20pro-real.service` 开机自启动时，需要把雷达参数写入：

```text
/etc/default/m20pro-real
```

示例：

```bash
sudo nano /etc/default/m20pro-real
```

配置：

```bash
M20PRO_ENABLE_RADAR_INSPECTION=true
M20PRO_RADAR_BACKEND=u360_http
M20PRO_RADAR_SCAN_MODE=measuring
M20PRO_RADAR_DEVICE_URL=http://雷达IP:8080
M20PRO_RADAR_OUTPUT_DIR=/home/user/m20pro_radar_results
M20PRO_RADAR_RELEASE_ON_ANALYSIS=true
M20PRO_RADAR_START_RETRY_TIMEOUT_S=120.0
M20PRO_RADAR_START_RETRY_INTERVAL_S=5.0
```

重启服务：

```bash
sudo systemctl restart m20pro-real.service
```

查看日志：

```bash
systemctl status m20pro-real.service --no-pager
journalctl -u m20pro-real.service -n 100 --no-pager
```

## 8. 前端操作流程

1. 打开 Web：

```text
http://机器狗IP:8080
```

2. 完成基础自检和定位。

3. 在地图上标点。

4. 为点位填写：

```text
楼栋、单元、户号、区域、房间、扫描点、结果名前缀
```

5. 选择雷达计划：

```text
不触发雷达
仅实测实量
仅点云建模
实测实量 + 点云建模
```

6. 创建任务并开始执行。

7. 机器狗到达任务点后，系统自动发布 `/m20pro/active_waypoint`，雷达节点自动扫描。

8. 任务卡片中查看雷达结果。

9. 如点云建模无法自动下载，在任务卡片点击“登记点云”，填入人工导出的点云工程路径。

10. 如需人工测量，在任务卡片点击“人工回填”，填写测量项和值。

11. 点击 `雷达JSON` 或 `雷达CSV` 导出结果。

## 9. 结果目录说明

默认结果目录：

```text
~/.m20pro_radar_results
```

真机推荐：

```text
/home/user/m20pro_radar_results
```

目录结构：

```text
jobs/        任务点扫描总记录
raw/         雷达原始返回
summaries/   摘要结果
downloads/   自动下载的建模文件
manual/      人工点云登记和人工测量回填
```

示例：

```text
jobs/task_001_0_wp_living_p01.json
raw/B03_U01_F20_R2008_P01_measure_20260701_154308.json
raw/B03_U01_F20_R2008_P01_cloud_20260701_154310_task_info.json
summaries/B03_U01_F20_R2008_P01_measure_20260701_154308.json
downloads/B03_U01_F20_R2008_P01_cloud_20260701_154310_01.zip
manual/task_001.json
```

## 10. 平台对接建议

后续接入甲方平台时，建议平台只对接稳定业务接口，不直接操作 Nav2 或雷达底层。

推荐职责：

```text
甲方平台：下发任务、查询结果、展示报告
机器狗本地系统：导航、到点、触发雷达、保存结果
雷达模块：控制 U360、解析结果、导出数据
```

当前 104 会持久保存 `jobs/raw/summaries/manual`，雷达返回下载 URL 时还会保存 `downloads` 下的点云工程文件；前端只是分页查看这些本地记录。现阶段尚无甲方服务器上传 ACK，因此不自动清理。平台对接完成后，应以“甲方已持久化确认”为本地清理前提，不能以“前端已经看过”作为删除依据。

平台下发任务点时应包含：

```json
{
  "building": "3栋",
  "unit": "1单元",
  "house": "2008户",
  "floor": "F20",
  "room": "客厅",
  "scan_point": "P01",
  "pose": {"x": 1.2, "y": 3.4, "yaw": 0.0},
  "radar": {
    "enabled": true,
    "scans": [
      {"mode": "measuring"},
      {"mode": "modeling"}
    ]
  }
}
```

平台回收结果：

```bash
curl "http://机器狗IP:8080/api/radar/status"

curl "http://机器狗IP:8080/api/radar/results?task_id=<task_id>"

curl "http://机器狗IP:8080/api/radar/task?task_id=<task_id>"

curl -o radar_task.json \
  "http://机器狗IP:8080/api/radar/task_export?task_id=<task_id>&format=json"

curl -o radar_task.csv \
  "http://机器狗IP:8080/api/radar/task_export?task_id=<task_id>&format=csv"
```

登记人工导出的点云：

```bash
curl -X POST http://机器狗IP:8080/api/radar/artifact \
  -H "Content-Type: application/json" \
  -d '{"task_id":"<task_id>","run_id":"<run_id>","artifact_path":"/path/to/u360/project"}'
```

人工测量回填：

```bash
curl -X POST http://机器狗IP:8080/api/radar/manual_measurement \
  -H "Content-Type: application/json" \
  -d '{"task_id":"<task_id>","run_id":"<run_id>","measurements":[{"name":"开关高度","value":"1.32m"}]}'
```

## 11. 常见问题

### 11.1 前端看不到“登记点云”

只有满足以下条件才显示：

- 任务是前端任务列表里的真实任务。
- 该任务点执行过 `modeling` 点云建模。
- 对应雷达结果已写入当前 `radar_results_dir`。

如果只是手动发布了一个不存在于前端任务列表的 `task_id`，结果文件会生成，但不会挂到前端任务卡片。

### 11.2 `/scan` 缺失能不能测雷达

可以测雷达业务闭环，但不能测完整导航闭环。

雷达 dry-run 和手动发布 `/m20pro/active_waypoint` 不依赖 `/scan`。

完整流程：

```text
前端设点 -> 机器狗导航到点 -> 到点触发雷达
```

需要真实机器狗或完整仿真环境。

### 11.3 本机仿真为什么缺 `/scan`

当前新仓库缺少仿真双雷达需要的：

```text
full_cloud.pcd
```

没有它时，仿真能启动 Web 和地图，但不能生成完整 `/cloud_nav -> /scan` 感知链路。

### 11.4 真机控制不了雷达

先在机器狗上执行：

```bash
curl -v --max-time 5 http://雷达IP:8080
```

如果不通，检查：

- 雷达是否接入机器狗 WiFi。
- 雷达 IP 是否变化。
- 8080 端口是否开放。
- 机器狗是否有多网卡路由问题。

### 11.5 结果文件有实测但没有点云文件

点云建模文件是否能自动下载取决于 U360 HTTP 接口是否返回可下载文件 URL。

如果设备只把点云保存在雷达本地，则结果会显示 `pending_import`，需要人工导出后在前端“登记点云”。

### 11.6 自启动后雷达没启用

检查：

```bash
cat /etc/default/m20pro-real
journalctl -u m20pro-real.service -n 100 --no-pager
```

确认包含：

```bash
M20PRO_ENABLE_RADAR_INSPECTION=true
M20PRO_RADAR_BACKEND=u360_http
M20PRO_RADAR_DEVICE_URL=http://雷达IP:8080
```

修改后需要重启：

```bash
sudo systemctl restart m20pro-real.service
```

## 12. 测试准入命令

提交或真机部署前建议运行：

```bash
cd ~/m20pro-ros2-real
source /opt/ros/humble/setup.bash

python3 -m py_compile \
  src/m20pro_radar_inspection/m20pro_radar_inspection/radar_inspection_node.py \
  src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py

python3 scripts/test_annotation_contract.py
python3 scripts/test_task_contract.py
python3 scripts/check_preflight_policy.py

colcon build --packages-up-to m20pro_bringup --symlink-install
```

说明：如果本机存在其它 ROS 工作区污染，`colcon` 可能打印其它目录的权限警告；只要当前 `m20pro_*` 相关包构建成功即可。
