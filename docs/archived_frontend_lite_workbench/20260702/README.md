# Archived Lite Workbench

This folder keeps the 2026-07-02 frontend layout experiment for reference only.
Do not deploy these files to 104 as the active dashboard.

Current status:

- It is a layout draft, not the maintained field UI.
- It does not include the U360/Unre radar controls, task export buttons, manual artifact registration, or `/api/radar/*` integration.
- It still contains old task readiness, watcher, and field-snapshot references that have been removed from the active workflow.
- The active field frontend remains `src/m20pro_cloud_bridge/m20pro_cloud_bridge/static/`.

Before this layout can replace the field UI, port the current old-panel behavior first:

- relocalization and single-floor task dispatch;
- active waypoint/task status;
- map selection and current-floor display;
- YOLO display;
- U360 radar metadata, status, result export, artifact registration, and manual measurement APIs.
