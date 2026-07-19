# Edge Scan Production Status

Since 2026-07-10, edge scan is the only supported real-robot perception chain.
The retired 104 raw-pointcloud relay/fusion implementation is archived outside
the Git repository under the workstation `M20Pro运行数据` directory.

106 owns raw pointcloud ingestion and publishes lightweight `/scan`. Its systemd
service is enabled at boot and restarts unconditionally after failures. 104 uses
the project UDP FastDDS profile. AMCL, the web dashboard, recording and normal
navigation retain `/scan`; the Nav2 costmaps consume the selected
`/m20pro/navigation_scan` contract.

The edge converter consumes every pointcloud fragment and keeps each angular
bin for at most 0.75 seconds before publishing at 4 Hz. This prevents periodic
rear-hemisphere omissions in the vendor cloud from becoming rear blind frames
in `/scan`, without retaining stale obstacles indefinitely.

There is no production fallback to local fusion. If `/scan` is stale or empty,
navigation must remain blocked until the 106 service or DDS link is restored.

Stair sessions add a bounded 0.50-second 3D tread-envelope classifier on 106.
Normal steps remain terrain, while residual objects are published on
`/m20pro/stair_obstacle_scan` and the fail-closed state is published on
`/m20pro/stair_clearance`. 104 selects that obstacle-only scan only while the
floor manager holds a live stair lease. Startup needs three consecutive clear
samples; blocked, unknown, missing or stale data stops the stair action. The
stair behavior tree has no reverse, spin, costmap clear, or motion recovery.
