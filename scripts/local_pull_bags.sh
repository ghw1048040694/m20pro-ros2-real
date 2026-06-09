#!/usr/bin/env bash
set -euo pipefail

DEST="${1:-${HOME}/bags/m20pro}"
mkdir -p "${DEST}"
rsync -avz user@10.21.31.104:/home/user/bags/ "${DEST}/"
