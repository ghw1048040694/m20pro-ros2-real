#!/usr/bin/env bash
set -eo pipefail

STAGE="${1:?staged workspace is required}"
TARGET="${2:?target workspace is required}"
BACKUP="${3:?backup workspace is required}"
REVISION="${4:-unknown}"
FAILED="${STAGE}.failed"
SYSTEMD_BACKUP="${BACKUP}.systemd"

had_target=0
had_default=0
had_unit=0
cutover_started=0

rollback() {
  local status="$?"
  trap - ERR
  set +e
  if [[ "${cutover_started}" -eq 1 ]]; then
    sudo -n systemctl stop m20pro-real.service >/dev/null 2>&1 || true
    if [[ -d "${TARGET}" ]]; then
      mv "${TARGET}" "${FAILED}" 2>/dev/null || true
    fi
    if [[ "${had_target}" -eq 1 && -d "${BACKUP}" ]]; then
      mv "${BACKUP}" "${TARGET}"
    fi
    if [[ "${had_default}" -eq 1 ]]; then
      sudo -n cp -a "${SYSTEMD_BACKUP}/m20pro-real.default" /etc/default/m20pro-real
    else
      sudo -n rm -f /etc/default/m20pro-real
    fi
    if [[ "${had_unit}" -eq 1 ]]; then
      sudo -n cp -a "${SYSTEMD_BACKUP}/m20pro-real.service" \
        /etc/systemd/system/m20pro-real.service
    else
      sudo -n rm -f /etc/systemd/system/m20pro-real.service
    fi
    sudo -n systemctl daemon-reload
    sudo -n systemctl reset-failed m20pro-real.service || true
    if [[ "${had_target}" -eq 1 ]]; then
      sudo -n systemctl start m20pro-real.service || true
    fi
  fi
  echo "[104_install_staged_workspace] deployment failed; previous runtime restored" >&2
  exit "${status}"
}
trap rollback ERR

if [[ ! -d "${STAGE}/src" || ! -x "${STAGE}/scripts/104_enable_autostart.sh" ]]; then
  echo "invalid staged workspace: ${STAGE}" >&2
  exit 20
fi
if [[ -e "${BACKUP}" || -e "${FAILED}" || -e "${SYSTEMD_BACKUP}" ]]; then
  echo "deployment path already exists; choose a new timestamp" >&2
  exit 21
fi

mkdir -p "$(dirname "${BACKUP}")" "${SYSTEMD_BACKUP}"
sudo -v
if [[ -f /etc/default/m20pro-real ]]; then
  sudo -n cp -a /etc/default/m20pro-real "${SYSTEMD_BACKUP}/m20pro-real.default"
  had_default=1
fi
if [[ -f /etc/systemd/system/m20pro-real.service ]]; then
  sudo -n cp -a /etc/systemd/system/m20pro-real.service \
    "${SYSTEMD_BACKUP}/m20pro-real.service"
  had_unit=1
fi

sudo -n systemctl stop m20pro-real.service >/dev/null 2>&1 || true
cutover_started=1
if [[ -d "${TARGET}" ]]; then
  mv "${TARGET}" "${BACKUP}"
  had_target=1
fi
mv "${STAGE}" "${TARGET}"

# A symlink install embeds absolute build paths, so it must be built only after
# the staged source has reached its final target directory.
rm -rf "${TARGET}/build" "${TARGET}/install" "${TARGET}/log"
set +u
source /opt/robot/scripts/setup_ros2.sh
cd "${TARGET}"
colcon build --symlink-install
chmod +x scripts/*.sh src/m20pro_bringup/scripts/*.sh 2>/dev/null || true
test -f install/m20pro_bringup/share/m20pro_bringup/local_setup.bash
printf '%s\n' "${REVISION}" > .m20pro_deploy_revision
date --iso-8601=seconds > .m20pro_deploy_time

sudo -n env M20PRO_WS="${TARGET}" \
  bash "${TARGET}/scripts/104_enable_autostart.sh" move
sudo -n systemctl reset-failed m20pro-real.service
sudo -n systemctl start m20pro-real.service

web_ready=0
for _ in $(seq 1 90); do
  state="$(sudo -n systemctl is-active m20pro-real.service 2>/dev/null || true)"
  if [[ "${state}" == "active" ]] && \
      curl --connect-timeout 1 --max-time 2 -fsS \
        http://127.0.0.1:8080/healthz >/dev/null 2>&1; then
    web_ready=1
    break
  fi
  [[ "${state}" != "failed" ]]
  sleep 1
done
[[ "${web_ready}" -eq 1 ]]

trap - ERR
echo "[104_install_staged_workspace] target=${TARGET}"
echo "[104_install_staged_workspace] backup=${BACKUP}"
echo "[104_install_staged_workspace] revision=${REVISION}"
echo "[104_install_staged_workspace] web ready; restart 106 edge scan before final acceptance"
