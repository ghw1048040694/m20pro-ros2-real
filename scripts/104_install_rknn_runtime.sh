#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${M20PRO_WS:-/home/user/m20pro_real_ros2_ws}"
WHEEL="${M20PRO_RKNNLITE_WHEEL:-/tmp/rknn_toolkit_lite2-2.3.2-cp38-cp38-manylinux_2_17_aarch64.manylinux2014_aarch64.whl}"
RUNTIME_LIB="${M20PRO_RKNN_RUNTIME_LIB:-/tmp/librknnrt.so}"
MODEL="${M20PRO_RKNN_MODEL:-/tmp/best_rk3588_fp16.rknn}"
PYDEPS="/home/user/m20pro_rknn_pydeps"
MODEL_DST="${WS_DIR}/src/m20pro_inspection/models/best_rk3588_fp16.rknn"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root on 104" >&2
  exit 2
fi
if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "RKNN runtime installer requires aarch64" >&2
  exit 1
fi
for path in "${WHEEL}" "${RUNTIME_LIB}" "${MODEL}"; do
  [[ -f "${path}" ]] || { echo "missing required file: ${path}" >&2; exit 1; }
done

rm -rf "${PYDEPS}"
install -d -o user -g user -m 0755 "${PYDEPS}" "$(dirname "${MODEL_DST}")"
sudo -u user python3 -m pip install --no-deps --target "${PYDEPS}" "${WHEEL}"
install -m 0755 "${RUNTIME_LIB}" /usr/lib/librknnrt.so
install -o user -g user -m 0644 "${MODEL}" "${MODEL_DST}"
ldconfig

PYTHONPATH="${PYDEPS}" python3 - <<PY
from rknnlite.api import RKNNLite

runtime = RKNNLite()
assert runtime.load_rknn("${MODEL_DST}") == 0
assert runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO) == 0
runtime.release()
print("RKNNLite runtime and model initialized successfully")
PY
