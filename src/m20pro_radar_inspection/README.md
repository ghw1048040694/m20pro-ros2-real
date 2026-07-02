# M20Pro Radar Inspection

This package runs task-point radar inspection for U360RSE/UCL360.

Current responsibilities:

- Subscribe to `/m20pro/active_waypoint` and read the current task point metadata.
- Trigger a scan when a task waypoint enters the `dwelling` phase.
- Read `waypoint.radar.scans` so each point can run measuring, modeling, both,
  or no radar scan at all.
- Support `dry_run` simulation without a real radar device.
- Support U360 HTTP scan APIs for measuring and modeling modes.
- Save job records, raw results, summaries, optional modeling downloads, and
  manual artifact/measurement records.
- Publish scan status and result topics for the web dashboard.

Topics:

```text
sub  /m20pro/active_waypoint
pub  /m20pro/radar_inspection/status
pub  /m20pro/radar_inspection/result
pub  /m20pro/radar_inspection/events
```

Expected active-waypoint metadata:

```json
{
  "phase": "dwelling",
  "task_id": "task_001",
  "index": 0,
  "waypoint": {
    "building": "3栋",
    "unit": "1单元",
    "house": "2008户",
    "floor": "F20",
    "area": "东区",
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

If `radar.enabled` is false, this package does nothing for that waypoint. If
`radar.scans` is missing, it falls back to the launch-level `radar_scan_mode`.

Simulation dry-run:

```bash
ros2 launch m20pro_bringup m20pro.launch.py \
  mode:=sim \
  enable_web_dashboard:=true \
  enable_radar_inspection:=true \
  radar_backend:=dry_run \
  radar_dry_run_duration_s:=2.0
```

Open the dashboard and start a web task that contains task points. When a task
point enters `dwelling`, the node publishes status/result and writes files under
`~/.m20pro_radar_results/`.

Simulation navigation with the real U360:

```bash
ros2 launch m20pro_bringup m20pro.launch.py \
  mode:=sim \
  enable_web_dashboard:=true \
  enable_radar_inspection:=true \
  radar_backend:=u360_http \
  radar_scan_mode:=measuring \
  radar_device_url:=http://192.168.107.72:8080
```

This keeps the M20 robot/navigation in simulation, but calls the real radar
HTTP API when a task point enters `dwelling`.

In measuring mode, the robot does not have to wait for the U360 result to reach
100% before leaving the point. By default the radar node releases the waypoint
only when `/nuc/queryState` enters `analyzing`, while it continues polling
`/nuc/getResult` in the background. You can disable this early release if needed:

```bash
ros2 launch m20pro_bringup m20pro.launch.py \
  mode:=sim \
  enable_web_dashboard:=true \
  enable_radar_inspection:=true \
  radar_backend:=u360_http \
  radar_scan_mode:=measuring \
  radar_device_url:=http://192.168.107.72:8080 \
  radar_release_on_analysis:=true
```

If the next waypoint is reached while the device is still busy, the node retries
starting the next scan for `radar_start_retry_timeout_s` seconds, with
`radar_start_retry_interval_s` between attempts.

Use modeling/point-cloud mode instead of measuring mode:

```bash
ros2 launch m20pro_bringup m20pro.launch.py \
  mode:=sim \
  enable_web_dashboard:=true \
  enable_radar_inspection:=true \
  radar_backend:=u360_http \
  radar_scan_mode:=modeling \
  radar_device_url:=http://192.168.107.72:8080
```

Before using the real radar from simulation, check that this computer can reach
the device:

```bash
curl -v --max-time 5 http://192.168.107.72:8080
```

Real U360 measuring mode:

```bash
ros2 launch m20pro_bringup m20pro.launch.py \
  mode:=real \
  enable_web_dashboard:=true \
  enable_radar_inspection:=true \
  radar_backend:=u360_http \
  radar_scan_mode:=measuring \
  radar_device_url:=http://192.168.107.72:8080
```

Real U360 modeling mode:

```bash
ros2 launch m20pro_bringup m20pro.launch.py \
  mode:=real \
  enable_web_dashboard:=true \
  enable_radar_inspection:=true \
  radar_backend:=u360_http \
  radar_scan_mode:=modeling \
  radar_device_url:=http://192.168.107.72:8080
```

Real shadow mode on the robot:

```bash
M20PRO_ENABLE_RADAR_INSPECTION=true \
M20PRO_RADAR_BACKEND=u360_http \
M20PRO_RADAR_SCAN_MODE=measuring \
M20PRO_RADAR_DEVICE_URL=http://192.168.107.72:8080 \
ros2 run m20pro_bringup m20pro_real_full.sh shadow
```

Use `M20PRO_RADAR_SCAN_MODE=modeling` for point-cloud/modeling mode.

Saved results default to:

```text
~/.m20pro_radar_results/
  jobs/
  raw/
  summaries/
  downloads/
  manual/
```

Modeling/point-cloud handling:

- If the U360 HTTP API returns downloadable file URLs, downloaded files are
  stored under `downloads/`.
- If the device keeps the point-cloud project locally and an operator copies it
  out manually, the web task card `登记点云` action records the exported project
  path in `manual/<task_id>.json`.
- Manual measurements made later in third-party point-cloud software can be
  entered from `人工回填`; these records are also exported in JSON/CSV.

Task result export:

After a web task runs through one or more task points, open the dashboard task
panel and use:

- `雷达JSON`: exports every radar job for that task, including waypoint metadata,
  raw/summary paths, scan status, and parsed summaries.
- `雷达CSV`: exports a flat table of every parsed measurement item for Excel or
  other result management tools.

The exported records include both `scan_released_at` and `finished_at`.
`scan_released_at` is the time the robot was allowed to move to the next point;
`finished_at` is when U360 analysis/result collection finally completed.

The same export endpoints can be used directly:

```bash
curl -o radar_task.json \
  "http://127.0.0.1:8080/api/radar/task_export?task_id=<task_id>&format=json"

curl -o radar_task.csv \
  "http://127.0.0.1:8080/api/radar/task_export?task_id=<task_id>&format=csv"
```

Manual artifact and measurement APIs:

```bash
curl -X POST http://127.0.0.1:8080/api/radar/artifact \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"<task_id>","run_id":"<run_id>","artifact_path":"/path/to/u360/project"}'

curl -X POST http://127.0.0.1:8080/api/radar/manual_measurement \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"<task_id>","run_id":"<run_id>","measurements":[{"name":"开关高度","value":"1.32m"}]}'
```

The web dashboard waits for radar completion only when
`enable_radar_inspection:=true`. The wait timeout is controlled by
`radar_inspection_timeout_s`. For measuring mode, "completion" for task flow
means scan release; final analysis results are still collected and exported
afterward.
