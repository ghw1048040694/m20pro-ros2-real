#!/usr/bin/env bash
set -euo pipefail

echo "[104_autostart_status] unit file:"
systemctl list-unit-files m20pro-real.service --no-pager 2>/dev/null || true

echo
echo "[104_autostart_status] service state:"
systemctl --no-pager --full status m20pro-real.service 2>/dev/null || true

echo
echo "[104_autostart_status] config:"
if [[ -f /etc/default/m20pro-real ]]; then
  cat /etc/default/m20pro-real
else
  echo "/etc/default/m20pro-real not found"
fi

echo
echo "[104_autostart_status] web health:"
curl -fsS http://127.0.0.1:8080/healthz 2>/dev/null || true
echo

echo
echo "[104_autostart_status] listening on 8080:"
ss -ltnp 2>/dev/null | grep ':8080' || true

echo
echo "[104_autostart_status] recent logs:"
journalctl -u m20pro-real.service -n 80 --no-pager 2>/dev/null || true
