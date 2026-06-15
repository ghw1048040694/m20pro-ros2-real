#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root on 104 after su." >&2
  exit 2
fi

systemctl stop m20pro-real.service 2>/dev/null || true
systemctl disable m20pro-real.service 2>/dev/null || true
rm -f /etc/systemd/system/m20pro-real.service /etc/default/m20pro-real
systemctl daemon-reload

echo "[104_disable_autostart] stopped, disabled, and removed m20pro-real.service"
