# 106 Edge Scan Feasibility Report

Date: 2026-07-07

Chinese decision summary:

```text
tools/edge_scan_feasibility/DECISION_REPORT_CN.md
```

Current real-chain deployment status:

```text
tools/edge_scan_feasibility/REAL_CHAIN_STATUS.md
```

## Goal

Reduce the 104-side DDS/SHM load by moving raw lidar consumption away from
104. The desired future chain is:

```text
106: /LIDAR/POINTS -> lightweight scan or relay
104: subscribe lightweight topic -> Nav2/Web
```

The current production fallback remains:

```text
104: /LIDAR/POINTS -> /m20pro/lidar_points_relay -> /scan -> Nav2/Web
```

## Tested Routes

1. 106 ROS2/rclpy subscriber to `/LIDAR/POINTS`

Result: failed in the current field state. ROS2 topic discovery could see
publishers in some profiles, but subscribers received `0` samples.

2. 106 recovery using the old 104 playbook

Result: insufficient for this case. The old 104 recovery path was correct root
ROS environment, FastDDS whitelist, and `multicast-relay.service`. On 106,
these were checked, and `multicast-relay.service` was restarted, but rclpy
still received `0` samples.

3. 106 native DrDDS pointcloud probe

Result: succeeded. Command shape:

```bash
drdds_lidar_probe /LIDAR/POINTS 0 0 rt 8
```

Observed roughly 10Hz raw pointcloud samples, frame `lidar_link`, with
`use_shm=0`.

4. 106 native DrDDS pointcloud to experimental LaserScan

Result: succeeded as a parallel demo. Command shape:

```bash
drdds_edge_scan_demo /LIDAR/POINTS /m20pro/scan_edge 90 0 0 rt -1.0 1.0 10 20000
```

Observed on 106:

```text
clouds=900 scans=558 rate_hz=6.21106
```

Observed on 104 in parallel:

```text
/scan                 rate_hz=3.835 frame=m20pro_base_link finite_mean=201.0 age_mean=0.045
/m20pro/scan_edge     rate_hz=6.177 frame=lidar_link        finite_mean=879.1 age_mean=0.175
```

5. Tuned 106 edge LaserScan with production-like frame and scan geometry

Result: succeeded as a stronger demo candidate. The demo was updated to accept
`frame_id`, `angle_increment`, `range_max`, and `range_min`, then tuned against
the production `/scan`.

Best semantic match:

```text
height=[-0.05,0.55] max_publish_hz=10 max_points=20000
/scan                  finite_mean=191.09 rate_hz=3.723
/m20pro/scan_edge_exp  finite_mean=193.20 rate_hz=6.343
```

Best lightweight candidate:

```text
height=[-0.05,0.55] max_publish_hz=4 max_points=12000
/scan                  finite_mean=190.00 rate_hz=3.653 age_mean=0.145
/m20pro/scan_edge_exp  finite_mean=182.49 rate_hz=3.357 age_mean=0.155
```

106-side result for the lightweight candidate:

```text
clouds=882 scans=294 rate_hz=3.3341
```

The demo process was observed around 27% CPU shortly after startup and around
7.5% later in the run. This is still a demo binary, not a production service.

6. Short live retest after the report was drafted

Result: confirmed the same split again. On 106, ordinary ROS2 could see
`Publisher count: 2` for `/LIDAR/POINTS`, but a 6-second echo received no
samples. The DrDDS probe still worked:

```text
samples=62 rate_hz=10.1844 frame=lidar_link
```

A 45-second edge scan demo published only `/m20pro/scan_edge_exp`; it did not
publish `/scan`:

```text
clouds=435 scans=143 rate_hz=3.32899
```

104 received both scans in a 25-second read-only comparison:

```text
/scan                  rate_hz=3.680 finite_mean=202.46 age_mean=0.144 frame=m20pro_base_link
/m20pro/scan_edge_exp  rate_hz=3.375 finite_mean=190.18 age_mean=0.157 frame=m20pro_base_link
```

After cleanup, 106 had no demo process, temporary files were removed, 106
`/dev/shm` returned to about 27%, 104 only listed the production `/scan`, and
104 Web/API still reported `perception_ready`.

## Conclusion

The best next route is not ordinary 106 rclpy subscription. The best route is a
106-side native DrDDS edge node that consumes `/LIDAR/POINTS` and publishes a
lightweight scan for 104.

This is high-feasibility but not production-ready yet. The recommended demo
candidate is:

```bash
drdds_edge_scan_demo /LIDAR/POINTS /m20pro/scan_edge_exp 90 0 0 rt \
  -0.05 0.55 4 12000 m20pro_base_link 0.0174533 10.0 0.2
```

The same candidate is wrapped by:

```bash
tools/edge_scan_feasibility/run_balanced_demo_on_106.sh
```

## Blockers Before Production

- Output frame must continue to match Nav2 expectations. Production `/scan`
  uses `m20pro_base_link`; the tuned demo now supports this explicitly.
- Height filtering and obstacle semantics are close but not fully proven. The
  best lightweight candidate was within roughly 4% of the production finite-bin
  mean in one 45-second comparison.
- A formal service must be reversible:
  - default fallback: 104 local relay;
  - optimized path: 106 edge scan/relay;
  - operator rollback: one command or one env switch.
- Long-run rosbag comparison and one short single-floor navigation test are
  required before switching Nav2 input.
- The production change must also remove or disable 104 raw pointcloud
  subscription, 104 `lidar_relay`, and 104 `pointcloud_fusion`; that has not
  been done yet.

## Safety Status

The 2026-07-07 demo was cleaned up after testing. It did not modify launch
files, systemd units, `/etc/default/m20pro-real`, Nav2, or the production
`/scan` topic.

## Battery-Aware Stop Point

The robot battery was reported as insufficient for more long-running tests.
This feasibility phase is therefore archived here instead of continuing into
service-level or navigation-motion tests.

Current stage is complete for feasibility:

- multiple routes tested;
- 106 DrDDS route identified as highest feasibility;
- balanced demo preserved;
- production blockers documented.

Deferred until the battery is replaced:

- 5-minute or longer `/scan` versus `/m20pro/scan_edge_exp` comparison;
- rosbag recording for offline review;
- offline bag analysis with `tools/edge_scan_feasibility/analyze_scan_bag.py`;
- reversible service wrapper on 106;
- any Nav2 input switch or short navigation task.

Resume from:

```text
tools/edge_scan_feasibility/NEXT_BATTERY_TEST_PLAN.md
```

If the battery-safe pass succeeds, continue with:

```text
tools/edge_scan_feasibility/SERVICE_TRIAL.md
```

The future production cutover is tracked in:

```text
tools/edge_scan_feasibility/PRODUCTION_MIGRATION_PLAN.md
```

Offline artifact safety can be checked with:

```text
tools/edge_scan_feasibility/audit_artifacts.py
```

Small local preparation already done:

- `perception_status_payload(..., perception_mode="edge_scan")` can treat a
  fresh scan as the hard readiness condition without requiring 104 relay
  freshness;
- default behavior remains `local_fusion`;
- `web_dashboard_node.py` and `m20pro_real.launch.py` expose the parameter with
  default `local_fusion`;
- `m20pro_real.launch.py` exposes `scan_topic` with default `/scan` for fusion
  output and web scan subscription;
- systemd defaults and production launch arguments still do not switch to
  `edge_scan`.
