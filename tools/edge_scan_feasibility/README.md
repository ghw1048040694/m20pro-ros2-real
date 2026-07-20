# M20Pro Production Edge Scan

The only production perception chain is:

```text
106 DrDDS /LIDAR/POINTS -> m20pro_edge_scan -> /scan -> 104 AMCL/Web/recording
                                      \-> stair 3D envelope -> stair obstacle scan
                                                               -> 104 Nav2 selector
```

104 must not subscribe to raw point clouds or run a lidar relay/fusion node.
Install the 106 service as root with:

```bash
./scripts/106_enable_edge_scan_service.sh
```

The installer validates the repository's canonical
`src/m20pro_bringup/config/m20pro_field_profile.yaml`, generates and atomically
installs `/etc/m20pro-edge-scan-106.env`, builds
`/usr/local/lib/m20pro/m20pro_edge_scan`, and enables and restarts
`m20pro-edge-scan-106.service`. The `/etc` file is a generated artifact and is
never an editable configuration source.

Production acceptance requires:

- the 106 service is `active` and `enabled`;
- 104 `/api/state.perception_status` is `perception_ready`;
- `/scan` frame is `m20pro_base_link`, age is below 2 seconds, and at least 20
  ranges are finite;
- 104 has no relay/fusion process and does not subscribe to raw point clouds;
- `/dev/shm` remains below the system warning threshold.

The default production height band is `-0.25..0.60 m` in the vendor point coordinate
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

## Stair-safe output

The normal `/scan` contract never changes. During a leased stair session the
same 106 process additionally merges 0.50 seconds of forward-corridor 3D points,
estimates the local tread envelope, and publishes:

- `/m20pro/stair_obstacle_scan`: only residual obstacle geometry;
- `/m20pro/stair_clearance`: `clear`, `blocked`, or `unknown` plus evidence;
- `/m20pro/stair_perception_mode`: consumed as the 104 session heartbeat.

With the default field profile, regular adjacent steps up to `0.24 m` are terrain. Stable
geometry at least `0.26 m` above the local tread, or a larger
height discontinuity, blocks motion. Sparse or malformed profiles are `unknown`
and must never be treated as clear. The mode lease expires after 1.50 seconds so
a dead 104 controller cannot leave 106 in stair mode indefinitely.

The mode request and clearance result carry the canonical profile name and
SHA-256. 106 rejects a stair request from a 104 running a different profile;
104 also rejects a clearance result with a different hash. There is no legacy
env fallback and the edge binary cannot be started with a partial argument set.

The geometry threshold has a physical limit: a low object that is no taller
than a valid step and is continuous with the stair profile cannot be reliably
distinguished from the stair by this lidar alone. Stairways must still be
cleared before a real run, and mounting/posture changes require a new recorded
calibration test.
