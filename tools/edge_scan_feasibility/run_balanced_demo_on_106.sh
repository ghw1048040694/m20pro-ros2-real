#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${1:-/tmp/m20pro_edge_scan_feasibility}"

INPUT_TOPIC="${INPUT_TOPIC:-/LIDAR/POINTS}"
OUTPUT_TOPIC="${OUTPUT_TOPIC:-/m20pro/scan_edge_exp}"
DURATION_S="${DURATION_S:-90}"
DOMAIN="${DOMAIN:-0}"
USE_SHM="${USE_SHM:-0}"
PREFIX="${PREFIX:-rt}"
HEIGHT_MIN="${HEIGHT_MIN:--0.05}"
HEIGHT_MAX="${HEIGHT_MAX:-0.55}"
MAX_PUBLISH_HZ="${MAX_PUBLISH_HZ:-4}"
MAX_POINTS="${MAX_POINTS:-12000}"
FRAME_ID="${FRAME_ID:-m20pro_base_link}"
ANGLE_INCREMENT="${ANGLE_INCREMENT:-0.0174533}"
RANGE_MAX="${RANGE_MAX:-10.0}"
RANGE_MIN="${RANGE_MIN:-0.2}"
BIN_HOLD_S="${BIN_HOLD_S:-0.75}"

if [[ ! -x "${OUT_DIR}/m20pro_edge_scan" ]]; then
  "${SCRIPT_DIR}/build_on_106.sh" "${OUT_DIR}"
fi

exec "${OUT_DIR}/m20pro_edge_scan" \
  "${INPUT_TOPIC}" \
  "${OUTPUT_TOPIC}" \
  "${DURATION_S}" \
  "${DOMAIN}" \
  "${USE_SHM}" \
  "${PREFIX}" \
  "${HEIGHT_MIN}" \
  "${HEIGHT_MAX}" \
  "${MAX_PUBLISH_HZ}" \
  "${MAX_POINTS}" \
  "${FRAME_ID}" \
  "${ANGLE_INCREMENT}" \
  "${RANGE_MAX}" \
  "${RANGE_MIN}" \
  "${BIN_HOLD_S}"
