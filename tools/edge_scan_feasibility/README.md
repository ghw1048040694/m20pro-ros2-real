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

The input cloud reports `lidar_link`, while the vendor point coordinates used by
the previous deployed converter were already consumed as body coordinates with
TF conversion disabled. The edge node preserves that field interpretation and
publishes the resulting scan as `m20pro_base_link`. Any future lidar firmware or
mounting change requires a static alignment test before navigation.
