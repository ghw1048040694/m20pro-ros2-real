# 106 Edge Pointcloud Feasibility

This directory keeps a manual, non-production probe for the 106-side
pointcloud path. It does not start Nav2, does not publish `/scan`, and is not
installed by any launch file.

For the Chinese decision summary, start from:

```text
tools/edge_scan_feasibility/DECISION_REPORT_CN.md
```

For the current real-chain deployment and rollback state, see:

```text
tools/edge_scan_feasibility/REAL_CHAIN_STATUS.md
```

## What Was Verified

On 2026-07-07, the normal ROS2/rclpy path on 106 could see
`/LIDAR/POINTS` publishers but received no samples. The native DrDDS path did
receive samples:

```bash
/tmp/m20pro_drdds_lidar_probe /LIDAR/POINTS 0 0 rt 8
```

Observed result:

```text
sample ... frame=lidar_link ...
samples=51 rate_hz=9.99 ... frame=lidar_link
```

Important details:

- The topic argument must be `/LIDAR/POINTS` with the leading slash.
- `LIDAR/POINTS` without the leading slash received `0` samples.
- Domain `0`, prefix `rt`, and `use_shm=0` worked.
- This proves 106 can consume the factory pointcloud through native DrDDS
  without relying on FastDDS SHM for the probe process.

## How To Run On 106

Use the known-good robot shell sequence:

```bash
ssh user@10.21.31.106
source /opt/robot/scripts/setup_ros2.sh
su
```

Then from a copy of this repository on 106:

```bash
cd /home/user/m20pro_real_ros2_ws
tools/edge_scan_feasibility/build_on_106.sh
/tmp/m20pro_drdds_lidar_probe /LIDAR/POINTS 0 0 rt 8
```

Arguments:

```text
m20pro_drdds_lidar_probe <topic> <domain> <use_shm:0|1> <prefix> <duration_s>
```

Recommended smoke test:

```bash
/tmp/m20pro_drdds_lidar_probe /LIDAR/POINTS 0 0 rt 8
```

## Why This Exists

The current production chain on 104 is:

```text
/LIDAR/POINTS -> /m20pro/lidar_points_relay -> /scan -> Nav2/Web
```

That means large raw pointclouds still reach 104 before being reduced. If a
future 106-side edge node converts the factory pointcloud into a lightweight
relay or scan topic, 104 could stop subscribing to the raw pointcloud directly.

This probe is only the first proof: "106 can read raw pointcloud through
DrDDS." A production edge node still needs a separate design, fallback plan,
and parallel `/scan_edge` validation before it can replace the 104 chain.

## Edge Scan Demo

The demo can also convert DrDDS pointcloud samples into an experimental
LaserScan topic:

```bash
/tmp/m20pro_edge_scan_feasibility/drdds_edge_scan_demo \
  /LIDAR/POINTS /m20pro/scan_edge 90 0 0 rt -1.0 1.0 10 20000
```

Arguments:

```text
drdds_edge_scan_demo <input_topic> <output_topic> <duration_s> <domain>
  <use_shm:0|1> <prefix> <height_min> <height_max> <max_publish_hz>
  <max_points> [frame_id] [angle_increment] [range_max] [range_min]
```

Observed on 2026-07-07:

```text
clouds=900 scans=558 rate_hz=6.21106
```

104 received `/m20pro/scan_edge` in parallel while the production `/scan`
remained healthy:

```text
/scan                 rate_hz=3.835 frame=m20pro_base_link finite_mean=201.0 age_mean=0.045
/m20pro/scan_edge     rate_hz=6.177 frame=lidar_link        finite_mean=879.1 age_mean=0.175
```

Do not wire this demo directly into Nav2. The experimental scan still needs
frame/TF alignment and height-filter tuning before it can replace production
`/scan`.

To compare more fairly with the current production `/scan`, run the demo with
`frame_id=m20pro_base_link` and `angle_increment=0.0174533`:

```bash
/tmp/m20pro_edge_scan_feasibility/drdds_edge_scan_demo \
  /LIDAR/POINTS /m20pro/scan_edge 60 0 0 rt -1.0 1.0 10 20000 \
  m20pro_base_link 0.0174533 10.0 0.2
```

The most balanced 2026-07-07 tuning result was:

```bash
/tmp/m20pro_edge_scan_feasibility/drdds_edge_scan_demo \
  /LIDAR/POINTS /m20pro/scan_edge_exp 90 0 0 rt -0.05 0.55 4 12000 \
  m20pro_base_link 0.0174533 10.0 0.2
```

Or run the same balanced configuration with:

```bash
tools/edge_scan_feasibility/run_balanced_demo_on_106.sh
```

The script builds into `/tmp/m20pro_edge_scan_feasibility` if needed. Override
parameters with environment variables, for example:

```bash
DURATION_S=300 OUTPUT_TOPIC=/m20pro/scan_edge_exp \
  tools/edge_scan_feasibility/run_balanced_demo_on_106.sh
```

Observed 104-side comparison:

```text
/scan                  rate_hz=3.653 frame=m20pro_base_link finite_mean=190.00 age_mean=0.145
/m20pro/scan_edge_exp  rate_hz=3.357 frame=m20pro_base_link finite_mean=182.49 age_mean=0.155
```

The closest semantic match used `max_publish_hz=10` and `max_points=20000`:

```text
/scan                  finite_mean=191.09 rate_hz=3.723
/m20pro/scan_edge_exp  finite_mean=193.20 rate_hz=6.343
```

For a lightweight Nav2 candidate, start from the balanced 4Hz/12000-point
configuration, not the denser 10Hz/20000-point configuration.

## 104 Comparison Tool

Copy and run the helper on 104 after sourcing the robot ROS environment:

```bash
python3 tools/edge_scan_feasibility/compare_scan_topics.py \
  --duration 45 /scan /m20pro/scan_edge_exp
```

The helper is read-only and only subscribes to LaserScan topics.

## Bag Analysis

After recording a comparison bag, run:

```bash
python3 tools/edge_scan_feasibility/analyze_scan_bag.py \
  /home/user/bags/edge_scan_compare_YYYYMMDD_HHMMSS \
  --topics /scan /m20pro/scan_edge_exp
```

The input can be either a rosbag2 directory or a raw `.db3` file copied from a
bag directory.

For a single-topic smoke test on an older bag:

```bash
python3 tools/edge_scan_feasibility/analyze_scan_bag.py \
  /path/to/bag --topics /scan --allow-missing
```

## Next Battery Test

When the robot battery is replaced, resume with:

```text
tools/edge_scan_feasibility/NEXT_BATTERY_TEST_PLAN.md
```

That plan keeps the first pass to a short comparison and does not switch Nav2
or production `/scan`.

If that first pass succeeds, the reversible service trial is documented in:

```text
tools/edge_scan_feasibility/SERVICE_TRIAL.md
```

The service files under `tools/edge_scan_feasibility/service/` are examples
only. They are not installed by this repository.

The production cutover plan is documented in:

```text
tools/edge_scan_feasibility/PRODUCTION_MIGRATION_PLAN.md
```

It records the current 104 relay/fusion dependencies and the future switch
points. It is not an implementation patch.

## Offline Audit

Before archiving or syncing this experiment, run:

```bash
tools/edge_scan_feasibility/audit_artifacts.py
```

The audit checks that required reports and demo scripts exist, balanced demo
defaults are intact, service trial files are examples only, and experimental
edge scan strings have not entered `src/`, `scripts/`, or `systemd`.

## Cleanup

After a manual test on 106:

```bash
pkill -f '/tmp/m20pro_edge_scan_feasibility.*/drdds_edge_scan_demo'
rm -rf /tmp/m20pro_edge_scan_feasibility /tmp/m20pro_edge_scan_feasibility_test
rm -f /tmp/m20pro_edge_scan_demo*.log /tmp/m20pro_edge_scan_demo*.pid
```

This directory is intentionally not installed by launch files or systemd.
