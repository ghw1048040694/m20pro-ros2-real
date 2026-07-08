# Next Battery Test Plan

Date: 2026-07-07

This plan resumes the 106 edge scan feasibility work after the robot battery is
replaced. It is intentionally time-boxed and does not switch production Nav2
input in the first pass.

## Goal

Validate that the balanced 106 edge scan demo can run long enough to justify a
reversible service-level trial:

```text
106: DrDDS /LIDAR/POINTS -> /m20pro/scan_edge_exp
104: compare /scan and /m20pro/scan_edge_exp
```

## Battery-Safe First Pass

Target duration: about 10 minutes.

1. Confirm the production fallback is healthy on 104.

```bash
curl -s http://10.21.31.104:8080/api/state | python3 -m json.tool
```

Required before continuing:

- `perception_status.code` is `perception_ready`;
- production `/scan` is fresh;
- no active task is running.

2. Start the balanced demo on 106.

```bash
ssh user@10.21.31.106
source /opt/robot/scripts/setup_ros2.sh
su
cd /home/user/m20pro_real_ros2_ws
DURATION_S=420 tools/edge_scan_feasibility/run_balanced_demo_on_106.sh \
  > /tmp/m20pro_edge_scan_balanced.log 2>&1
```

The demo publishes only `/m20pro/scan_edge_exp`. It must not publish `/scan`.

3. Compare scan topics from 104 for 5 minutes.

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
cd /home/user/m20pro_real_ros2_ws
python3 tools/edge_scan_feasibility/compare_scan_topics.py \
  --duration 300 /scan /m20pro/scan_edge_exp
```

Pass criteria:

- `/m20pro/scan_edge_exp` frame is `m20pro_base_link`;
- rate is roughly `3.0Hz` or higher;
- finite-bin mean is within about 15% of production `/scan`;
- mean age is close to production `/scan` and stays below about `0.30s`;
- production `/scan` remains healthy throughout.

Stop criteria:

- production `/scan` becomes stale;
- 104 web state stops reporting `perception_ready`;
- `/m20pro/scan_edge_exp` disappears or drops below about `2Hz`;
- the robot battery becomes questionable.

4. Record one short bag only if the 5-minute comparison passes.

```bash
ros2 bag record -o /home/user/bags/edge_scan_compare_$(date +%Y%m%d_%H%M%S) \
  /scan /m20pro/scan_edge_exp /tf /tf_static \
  /local_costmap/costmap /global_costmap/costmap
```

Keep the bag short, about 2 to 3 minutes.

5. Analyze the bag offline.

```bash
python3 tools/edge_scan_feasibility/analyze_scan_bag.py \
  /home/user/bags/edge_scan_compare_YYYYMMDD_HHMMSS \
  --topics /scan /m20pro/scan_edge_exp
```

The input may be the bag directory or the copied `.db3` file.

The offline result should agree with the live comparison: frame,
frequency, finite-bin mean, age, and interval jitter should remain close.

## Second Pass Only After Review

Do not do this during the first battery-constrained pass unless explicitly
approved.

- Wrap the 106 demo as a reversible service.
- Add an operator switch to choose production `/scan` fallback or edge scan.
- Run Nav2 with the edge scan input in a controlled short-task test.
- Only after successful navigation testing should 104 raw `/LIDAR/POINTS`,
  `lidar_relay`, and `pointcloud_fusion` be disabled.

## Rollback

The current production fallback is still:

```text
104: /LIDAR/POINTS -> /m20pro/lidar_points_relay -> /scan
```

If anything looks wrong, stop the 106 demo:

```bash
pkill -f drdds_edge_scan_demo
```

Then verify 104 still reports `perception_ready`.
