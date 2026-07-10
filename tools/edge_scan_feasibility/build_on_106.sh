#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${1:-/tmp/m20pro_edge_scan_feasibility}"

mkdir -p "${OUT_DIR}"

cxx_flags=(
  -std=c++17
  -DGEN_API_VER=2
  -I/usr/local/include
  -I/usr/local/include/dridl
)

link_flags=(
  -L/usr/local/lib
  -Wl,-rpath,/usr/local/lib
  -ldrdds
  -lfastcdr
  -lfastrtps
  -lfoonathan_memory-0.7.3
  -lpthread
)

g++ "${cxx_flags[@]}" \
  "${SCRIPT_DIR}/drdds_lidar_probe.cpp" \
  -o "${OUT_DIR}/drdds_lidar_probe" \
  "${link_flags[@]}"

g++ "${cxx_flags[@]}" \
  "${SCRIPT_DIR}/drdds_edge_scan_demo.cpp" \
  -o "${OUT_DIR}/drdds_edge_scan_demo" \
  "${link_flags[@]}"

echo "Built ${OUT_DIR}/drdds_lidar_probe"
echo "Built ${OUT_DIR}/drdds_edge_scan_demo"
echo "Try: ${OUT_DIR}/drdds_lidar_probe /LIDAR/POINTS 0 0 rt 8"
echo "Try: ${OUT_DIR}/drdds_edge_scan_demo /LIDAR/POINTS /m20pro/scan_edge 20"
