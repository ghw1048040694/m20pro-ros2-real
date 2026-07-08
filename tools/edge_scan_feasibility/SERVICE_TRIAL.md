# Reversible 106 Edge Scan Service Trial

Date: 2026-07-07

This is a template-only plan. Do not install or start this service until the
short battery test in `NEXT_BATTERY_TEST_PLAN.md` has passed.

## Purpose

Run the balanced 106 edge scan demo as a reversible service:

```text
106: /LIDAR/POINTS -> /m20pro/scan_edge_exp
104: compare or optionally test against /m20pro/scan_edge_exp
```

The service must not publish `/scan`. Production fallback on 104 remains:

```text
104: /LIDAR/POINTS -> /m20pro/lidar_points_relay -> /scan
```

## Files

Templates:

```text
tools/edge_scan_feasibility/service/m20pro-edge-scan-106.env.example
tools/edge_scan_feasibility/service/m20pro-edge-scan-106.service.example
```

The env file uses `DURATION_S=0`, which means the demo runs until stopped.

## Install During An Approved Trial

On 106, after the repository is present at `/home/user/m20pro_real_ros2_ws`:

```bash
ssh user@10.21.31.106
source /opt/robot/scripts/setup_ros2.sh
su
cd /home/user/m20pro_real_ros2_ws
cp tools/edge_scan_feasibility/service/m20pro-edge-scan-106.env.example \
  /etc/m20pro-edge-scan-106.env
cp tools/edge_scan_feasibility/service/m20pro-edge-scan-106.service.example \
  /etc/systemd/system/m20pro-edge-scan-106.service
systemctl daemon-reload
systemctl start m20pro-edge-scan-106.service
```

Do not run `systemctl enable` during the first trial. Keep it manual.

## Verify

On 106:

```bash
systemctl status --no-pager m20pro-edge-scan-106.service
journalctl -u m20pro-edge-scan-106.service -n 80 --no-pager
```

On 104:

```bash
source /opt/robot/scripts/setup_ros2.sh
cd /home/user/m20pro_real_ros2_ws
python3 tools/edge_scan_feasibility/compare_scan_topics.py \
  --duration 300 /scan /m20pro/scan_edge_exp
```

Pass criteria are the same as `NEXT_BATTERY_TEST_PLAN.md`.

## Stop And Roll Back

On 106:

```bash
systemctl stop m20pro-edge-scan-106.service
systemctl disable m20pro-edge-scan-106.service || true
rm -f /etc/systemd/system/m20pro-edge-scan-106.service
rm -f /etc/m20pro-edge-scan-106.env
systemctl daemon-reload
pkill -f drdds_edge_scan_demo || true
```

On 104, verify the production fallback still reports `perception_ready`.

## Production Gate

Only after a successful service trial and a reviewed rosbag should we consider
a Nav2 input switch. Disabling 104 raw `/LIDAR/POINTS`, 104 `lidar_relay`, or
104 `pointcloud_fusion` is a separate production change and must remain
reversible.
