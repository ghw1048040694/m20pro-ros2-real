#!/usr/bin/env bash
set -euo pipefail

WEB_URL="${M20PRO_WEB_URL:-http://127.0.0.1:8080}"

payload="$(curl -fsS "${WEB_URL}/api/state")"
python3 - "${payload}" <<'PY'
import json
import sys

state = json.loads(sys.argv[1])
status = state.get("perception_status") or {}
scan = status.get("scan") or {}
print("perception=%s mode=%s frame=%s finite=%s age=%s" % (
    status.get("code"), status.get("mode"), scan.get("frame_id"),
    scan.get("finite_ranges"), scan.get("age_sec"),
))
if not status.get("ready") or status.get("mode") != "edge_scan":
    raise SystemExit(1)
if scan.get("frame_id") != "m20pro_base_link" or int(scan.get("finite_ranges") or 0) < 20:
    raise SystemExit(1)
PY
