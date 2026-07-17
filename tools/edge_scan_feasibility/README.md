# M20Pro Production Edge Scan

The only production perception chain is:

```text
106 DrDDS /LIDAR/POINTS -> m20pro_edge_scan -> /scan -> 104 Nav2/Web
```

104 must not subscribe to raw point clouds or run a lidar relay/fusion node.
Install the 106 service as root with:

```bash
./scripts/106_enable_edge_scan_service.sh
```

The installer builds and installs `/usr/local/lib/m20pro/m20pro_edge_scan`,
installs `/etc/m20pro-edge-scan-106.env`, and enables and starts
`m20pro-edge-scan-106.service`.

Production acceptance requires:

- the 106 service is `active` and `enabled`;
- 104 `/api/state.perception_status` is `perception_ready`;
- `/scan` frame is `m20pro_base_link`, age is below 2 seconds, and at least 20
  ranges are finite;
- 104 has no relay/fusion process and does not subscribe to raw point clouds;
- `/dev/shm` remains below the system warning threshold.

The production height band is `-0.25..0.60 m` in the vendor point coordinate
interpretation. It matches the previously field-tested real pointcloud
converter and includes obstacles below the body center while rejecting the
floor. Production uses the full cloud (`MAX_POINTS=0`); stride sampling is not
allowed because it can discard the few returns from a small, low obstacle.
The converter ingests every vendor cloud and publishes a 0.75-second rolling
angular aggregate. This is required because individual `/LIDAR/POINTS` messages
periodically omit the rear hemisphere; publish-rate throttling must happen only
after those fragments have updated the aggregate.
After a lidar mounting or posture change, verify both a low obstacle and a tall
obstacle before navigation. Do not lower the minimum height further without a
ground-return test, since floor points can otherwise make free space occupied.

The input cloud reports `lidar_link`, while the vendor point coordinates used by
the previous deployed converter were already consumed as body coordinates with
TF conversion disabled. The edge node preserves that field interpretation and
publishes the resulting scan as `m20pro_base_link`. Any future lidar firmware or
mounting change requires a static alignment test before navigation.
