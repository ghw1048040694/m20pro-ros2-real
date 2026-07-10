# Archived Lite Workbench

This folder contains the compact frontend layout. It is served as the parallel
`/lite` workbench and shares the maintained `/static/dashboard.js` controller
with the field dashboard.

Current status:

- The default field frontend remains `/` until `/lite` completes field acceptance.
- Business behavior is not duplicated here. Maps, localization, annotation editing,
  tasks, YOLO, U360 radar and recording use the maintained shared controller.
- The compact layout uses the current H.264/WebRTC camera pages on port 8888.
- U360 task export, artifact registration and manual measurement are rendered
  from the same `/api/radar/*` integration used by the default dashboard.

Before making `/lite` the default `/`, complete one field pass covering map
selection, relocalization, point create/edit, a short navigation task, recording,
video, YOLO and U360 dry-run/real-device status.
