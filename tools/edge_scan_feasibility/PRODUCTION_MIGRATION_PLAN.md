# Production Migration Plan

Date: 2026-07-07

Update 2026-07-08:

The first real edge scan chain has been implemented and deployed for field
testing. See:

```text
tools/edge_scan_feasibility/REAL_CHAIN_STATUS.md
```

104 now supports `M20PRO_SCAN_SOURCE=edge_scan`; in that mode it skips local
lidar relay and pointcloud fusion, disables Web PointCloud2 lidar
subscriptions, uses `/scan`, and sets Web/API perception mode to `edge_scan`.
106 runs a manual `m20pro-edge-scan-106.service` that publishes `/scan` from
DrDDS `/LIDAR/POINTS`.

This document started as an offline migration plan. As of 2026-07-08, the
first reversible real-chain trial is deployed on the robot for field testing.
Keep this document as the migration record and use `REAL_CHAIN_STATUS.md` for
the exact installed state and rollback commands.

## Current Production Chain

On 104 today:

```text
/LIDAR/POINTS
  -> m20pro_lidar_relay_guard.sh
  -> m20pro_navigation/lidar_relay
  -> /m20pro/lidar_points_relay
  -> m20pro_pointcloud_fusion
  -> /scan
  -> Nav2 costmaps, web scan overlay, readiness checks
```

Important current entry points:

- `src/m20pro_bringup/scripts/m20pro_real_full.sh`
  - starts `m20pro_lidar_relay_guard.sh`;
  - passes `cloud_topic:="${M20PRO_LIDAR_RELAY_TOPIC}"` into launch.
- `src/m20pro_bringup/launch/m20pro_real.launch.py`
  - launches `m20pro_pointcloud_fusion`;
  - publishes fusion output to `/scan`;
  - passes `cloud_topic` into the web dashboard and system check.
- `src/m20pro_bringup/config/nav2_params_real.yaml`
  - AMCL and both costmaps consume `/scan`.
- `src/m20pro_cloud_bridge/m20pro_cloud_bridge/perception_contract.py`
  - currently treats relay freshness as part of `perception_ready`.
- `src/m20pro_cloud_bridge/m20pro_cloud_bridge/web_dashboard_node.py`
  - subscribes to relay status and relay pointcloud;
  - preflight currently expects `m20pro_pointcloud_fusion` as a required node.

## Target Chain

The desired lightweight chain is:

```text
106 DrDDS /LIDAR/POINTS
  -> 106 edge scan service
  -> /m20pro/scan_edge
  -> 104 Nav2/Web
```

After production cutover, 104 should not need:

- cross-host raw `/LIDAR/POINTS` subscription;
- `m20pro_lidar_relay`;
- `m20pro_pointcloud_fusion`;
- relay pointcloud status as a hard readiness condition.

## Proposed Switch Design

Add one explicit mode variable, defaulting to current behavior:

```text
M20PRO_SCAN_SOURCE=local_fusion
```

Supported values:

- `local_fusion`: current production fallback.
- `edge_scan`: use 106 edge scan topic as the scan source.

Proposed environment variables:

```text
M20PRO_SCAN_SOURCE=local_fusion
M20PRO_EDGE_SCAN_TOPIC=/m20pro/scan_edge_exp
M20PRO_EDGE_SCAN_REQUIRED=0
```

Use `/m20pro/scan_edge_exp` until the trial is complete. Only rename to
`/m20pro/scan_edge` or `/scan` after final acceptance.

## Code Changes For A Future Trial

Do not apply these until the service trial passes.

1. `m20pro_real_full.sh`

When `M20PRO_SCAN_SOURCE=edge_scan`:

- skip `m20pro_lidar_relay_guard.sh start`;
- skip optional LIDAR2 relay;
- pass `fusion:=false` or an equivalent launch flag;
- pass `scan_topic:="${M20PRO_EDGE_SCAN_TOPIC}"` if launch supports it;
- keep the web dashboard and Nav2 running.

2. `m20pro_real.launch.py`

Add launch arguments:

```text
scan_topic
enable_pointcloud_fusion
perception_mode
```

When `enable_pointcloud_fusion=false`:

- do not launch `m20pro_pointcloud_fusion`;
- configure web dashboard `scan_topic` to the edge scan topic;
- configure system check to not require a cloud topic;
- keep Nav2 expecting a scan topic.

3. `nav2_params_real.yaml`

Nav2 currently hardcodes `/scan` in AMCL and costmaps. There are two possible
paths:

- safer first trial: remap `/m20pro/scan_edge_exp` to `/scan` in a controlled
  launch namespace or relay;
- cleaner production: template the Nav2 params file so `scan_topic` can become
  `/m20pro/scan_edge`.

The first trial should avoid changing Nav2 params if possible.

4. Web/API perception readiness

When `perception_mode=edge_scan`:

- `perception_status_payload(...)` must not require relay freshness;
- `/scan` or edge scan freshness becomes the hard condition;
- relay status can be shown as `not_used` instead of `fail`;
- messages should stop telling the operator to check `lidar_relay` when the
  configured source is edge scan.

Current local status:

- `perception_status_payload(..., perception_mode="edge_scan")` has pure
  contract support;
- default remains `local_fusion`, so current production behavior is unchanged;
- `web_dashboard_node.py` declares `perception_mode`;
- `m20pro_real.launch.py` passes launch argument `perception_mode` to the web
  dashboard, defaulting to `local_fusion`.
- `m20pro_real.launch.py` also exposes launch argument `scan_topic`, defaulting
  to `/scan`, and passes it to pointcloud fusion output and the web dashboard.

5. Web dashboard preflight

When `perception_mode=edge_scan`:

- `m20pro_pointcloud_fusion` must not be a required node;
- raw `/LIDAR/POINTS` must not be a required base topic on 104;
- the scan topic remains required;
- UI should display the source as `edge_scan`.

## Trial Gates

Before switching any production input:

1. `NEXT_BATTERY_TEST_PLAN.md` passes.
2. `SERVICE_TRIAL.md` passes.
3. A short rosbag is reviewed and shows:
   - edge scan frame is `m20pro_base_link`;
   - edge scan rate is stable;
   - costmaps remain populated;
   - production fallback `/scan` remains healthy during parallel comparison.
4. A rollback command is tested.

## Rollback

Rollback must be one environment change and one service restart:

```text
M20PRO_SCAN_SOURCE=local_fusion
systemctl restart m20pro-real.service
```

On 106:

```bash
systemctl stop m20pro-edge-scan-106.service
```

The old 104 chain must remain available until edge scan has passed real
navigation tests.

## Acceptance Criteria

The migration is not complete until all are true:

- 104 no longer subscribes to raw `/LIDAR/POINTS`;
- 104 no longer runs `m20pro_lidar_relay`;
- 104 no longer runs `m20pro_pointcloud_fusion`;
- Nav2 receives a fresh scan directly from the edge scan path;
- Web/API `perception_ready` is true in edge scan mode;
- SHM usage on 104 is materially lower after a clean restart;
- a short single-floor navigation task succeeds;
- fallback to `local_fusion` has been tested.
