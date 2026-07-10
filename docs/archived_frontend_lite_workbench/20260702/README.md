# Archived Lite Workbench

This folder keeps the 2026-07-02 frontend layout experiment for reference only.
Do not deploy these files to 104 as the active dashboard.

Current status:

- It is a layout draft, not the maintained field UI.
- It does not include the U360/Unre radar controls, task export buttons, manual artifact registration, or `/api/radar/*` integration.
- It has been updated with the 2026-07-09 old-panel behavior fixes: final relocalization verdict display, no green pose-history trail, always-call stop/reset task controls, and no task-readiness/watcher/field-snapshot UI.
- The active field frontend remains `src/m20pro_cloud_bridge/m20pro_cloud_bridge/static/`.

Before this layout can replace the field UI, port the current old-panel behavior first:

- full field testing for relocalization and single-floor task dispatch;
- active waypoint/task status;
- map selection and current-floor display;
- YOLO display;
- U360 radar metadata, status, result export, artifact registration, and manual measurement APIs.
