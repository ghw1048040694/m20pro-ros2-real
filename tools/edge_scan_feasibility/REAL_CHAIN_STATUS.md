# Edge Scan Real Chain Status

Date: 2026-07-08

## Current Installed Chain

The real robot is currently configured for the new edge scan chain:

```text
106 DrDDS /LIDAR/POINTS
  -> m20pro-edge-scan-106.service
  -> /scan
  -> 104 Nav2/Web
```

104 no longer starts:

- `m20pro_lidar_relay`;
- `m20pro_pointcloud_fusion`;
- Web PointCloud2 subscriptions for lidar relay/raw pointcloud.

104 currently uses:

```text
M20PRO_SCAN_SOURCE=edge_scan
M20PRO_EDGE_SCAN_TOPIC=/scan
M20PRO_FASTDDS_PROFILE=project_udp
```

106 currently runs:

```text
m20pro-edge-scan-106.service
```

The service is manual and disabled for boot autostart.

## Verified On 2026-07-08

104:

```text
/api/state perception_status.code=perception_ready
perception_status.mode=edge_scan
scan.frame_id=m20pro_base_link
scan.age_sec≈0.25
scan.finite_ranges≈180-190
/dev/shm≈1.1G/7.7G, 15%
```

Processes absent on 104:

```text
m20pro_lidar_relay
m20pro_pointcloud_fusion
```

106:

```text
m20pro-edge-scan-106.service active
drdds_edge_scan_demo /LIDAR/POINTS /scan ...
/dev/shm≈2.4G/7.7G, 31%
```

Web preflight:

```text
edge_scan ok: edge scan 已输出 /scan
scan ok
```

## Rollback

On 104:

```bash
source /opt/robot/scripts/setup_ros2.sh
su
python3 - <<'PY'
from pathlib import Path
path = Path('/etc/default/m20pro-real')
text = path.read_text(encoding='utf-8')
updates = {
    'M20PRO_SCAN_SOURCE': 'local_fusion',
    'M20PRO_FASTDDS_PROFILE': 'factory',
}
lines = []
seen = set()
for line in text.splitlines():
    if '=' in line and not line.lstrip().startswith('#'):
        key = line.split('=', 1)[0]
        if key in updates:
            lines.append(f'{key}={updates[key]}')
            seen.add(key)
            continue
    lines.append(line)
for key, value in updates.items():
    if key not in seen:
        lines.append(f'{key}={value}')
path.write_text('\n'.join(lines).rstrip() + '\n', encoding='utf-8')
PY
systemctl restart m20pro-real.service
```

On 106:

```bash
source /opt/robot/scripts/setup_ros2.sh
su
systemctl stop m20pro-edge-scan-106.service
```

## Next Real Test

Before running a long task, do a short operator test:

1. Open `http://10.21.31.104:8080`.
2. Confirm the top/status area shows perception ready.
3. Run relocalization on the current map.
4. Verify the laser outline matches the map.
5. Run one short single-floor task.
6. If navigation behaves oddly, stop the task first, then rollback to
   `local_fusion`.
