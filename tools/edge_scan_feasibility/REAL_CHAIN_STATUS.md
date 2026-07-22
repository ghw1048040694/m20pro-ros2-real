# Edge Scan Production Status

Since 2026-07-10, edge scan is the only supported real-robot perception chain.
The retired 104 raw-pointcloud relay/fusion implementation is archived outside
the Git repository under the workstation `M20Pro运行数据` directory.

106 owns raw pointcloud ingestion and publishes lightweight `/scan`. Its systemd
service is enabled at boot and restarts unconditionally after failures. 104 uses
the project UDP FastDDS profile. AMCL, Nav2 costmaps, the web dashboard,
self-check and recording all consume the same `/scan` contract directly.

The edge converter consumes every pointcloud fragment and keeps each angular
bin for at most 0.75 seconds before publishing at 4 Hz. This prevents periodic
rear-hemisphere omissions in the vendor cloud from becoming rear blind frames
in `/scan`, without retaining stale obstacles indefinitely.

There is no production fallback to local fusion. If `/scan` is stale or empty,
navigation must remain blocked until the 106 service or DDS link is restored.

All field-tunable scan and Nav2 values originate in
`src/m20pro_bringup/config/m20pro_field_profile.yaml`. Deployment renders the
106 systemd environment and 104 loads the same installed file. The old 106 env
template and partial-argument demo launcher have been removed. A profile hash
mismatch blocks deployment, and profiles are never hot-reloaded during a task.

The stair-envelope classifier, stair scan/status topics and the 104 scan
selector introduced on 2026-07-19 are retired. Cross-floor stair execution is
fail-closed until the replacement climbing design is integrated.
