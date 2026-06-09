#!/usr/bin/env bash
set -euo pipefail

echo "[104_status] web/real processes:"
ps -eo pid,args | awk '/web_dashboard|m20pro_real_full|m20pro.launch.py/ && !/awk/ {print}'

echo
echo "[104_status] listening on 8080:"
ss -ltnp 2>/dev/null | grep ':8080' || true

echo
echo "[104_status] quick health:"
curl -fsS http://127.0.0.1:8080/healthz 2>/dev/null || true
echo
