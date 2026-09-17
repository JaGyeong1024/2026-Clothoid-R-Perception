#!/bin/bash
# Snapshot the host root filesystem into a tarball for `docker import`.
#
# Designed to run on the vehicle (cnu PC) where perception is installed
# natively on the host. Result is written to the directory this script is
# invoked from ($PWD), not the script's own location.
#
# Usage:
#   cd /path/to/output_dir
#   sudo /path/to/snapshot_host_rootfs.sh           # default name
#   sudo /path/to/snapshot_host_rootfs.sh my.tar.gz # custom name
set -e

OUT_NAME="${1:-cnu-rootfs-$(hostname)-$(date +%Y%m%d).tar.gz}"
INVOKE_DIR="$(pwd)"
OUT_PATH="${INVOKE_DIR}/${OUT_NAME}"

if [ "$(id -u)" -ne 0 ]; then
    echo "[err] root 권한 필요. sudo $0 [out_name] 으로 실행." >&2
    exit 1
fi

# 자기 자신을 tar에 넣지 않도록 미리 0바이트로 만들고 절대경로 확정
touch "${OUT_PATH}"
OUT_PATH="$(readlink -f "${OUT_PATH}")"

# 출력 위치의 여유 공간 체크 (대충 10 GiB 미만이면 경고)
AVAIL_KB=$(df -Pk "${INVOKE_DIR}" | awk 'NR==2 {print $4}')
if [ "${AVAIL_KB}" -lt $((10 * 1024 * 1024)) ]; then
    echo "[warn] ${INVOKE_DIR} 여유 공간이 10 GiB 미만 ($((AVAIL_KB / 1024)) MiB). 그래도 진행하려면 5초 안에 Ctrl-C 안 누르면 계속." >&2
    sleep 5
fi

COMP="gzip"
if command -v pigz >/dev/null 2>&1; then
    COMP="pigz"
fi

echo "[info] output : ${OUT_PATH}"
echo "[info] comp   : ${COMP}"
echo "[info] start  : $(date)"

tar --numeric-owner --one-file-system \
    --warning=no-file-changed --warning=no-file-removed \
    --exclude=/proc --exclude=/sys --exclude=/dev \
    --exclude=/tmp --exclude=/run --exclude=/mnt --exclude=/media \
    --exclude=/var/cache/apt --exclude=/var/lib/apt/lists \
    --exclude=/var/lib/docker --exclude=/var/lib/containerd \
    --exclude=/var/log --exclude=/var/tmp \
    --exclude=/var/crash --exclude=/var/lib/snapd \
    --exclude=/snap \
    --exclude=/boot --exclude=/lib/modules --exclude=/lib/firmware \
    --exclude=/swapfile --exclude=/swap.img --exclude=/lost+found \
    --exclude=/root/.cache --exclude=/home/*/.cache \
    --exclude=/home/*/.ssh --exclude=/home/*/.aws \
    --exclude=/home/*/.docker --exclude=/home/*/.git-credentials \
    --exclude=/home/*/.bash_history \
    --exclude=/home/*/Downloads --exclude=/home/*/Videos \
    --exclude='*.bag' --exclude='*.bag.active' \
    --exclude="${OUT_PATH}" \
    -cf - / | ${COMP} > "${OUT_PATH}"

echo "[info] end    : $(date)"
echo
echo "=========================================="
ls -lh "${OUT_PATH}"
echo "sha256: $(sha256sum "${OUT_PATH}" | awk '{print $1}')"
echo "=========================================="
echo "[done] 이 파일 하나만 옮기면 됨: ${OUT_PATH}"
