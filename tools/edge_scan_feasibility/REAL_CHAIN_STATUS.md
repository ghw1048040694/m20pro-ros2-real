# Edge Scan Production Status

Since 2026-07-10, edge scan is the only supported real-robot perception chain.
The retired 104 raw-pointcloud relay/fusion implementation is archived outside
the Git repository under the workstation `M20Pro运行数据` directory.

106 owns raw pointcloud ingestion and publishes lightweight `/scan`. Its systemd
service is enabled at boot and restarts unconditionally after failures. 104 uses
the project UDP FastDDS profile and consumes only `/scan` for Nav2, the web
dashboard, preflight checks, and runtime task protection.

There is no production fallback to local fusion. If `/scan` is stale or empty,
navigation must remain blocked until the 106 service or DDS link is restored.
