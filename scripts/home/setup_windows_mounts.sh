#!/usr/bin/env bash
# 持久挂载本机 Windows C/D 盘到 /mnt/windows 与 /mnt/win_data
# 用法：sudo bash scripts/home/setup_windows_mounts.sh
#
# 使用 FUSE ntfs-3g（比内核 ntfs3 更适合双系统；也不吃 utf8 这种错误参数）。
# Windows「快速启动」仍可能导致失败：请正常关机或关掉快速启动。

set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  echo "请用 sudo 运行：sudo bash $0"
  exit 1
fi

UID_NUM="${SUDO_UID:-1000}"
GID_NUM="${SUDO_GID:-1000}"

WIN_C_UUID="5282B7F582B7DBA3"
WIN_D_UUID="FA60BAE960BAABAF"

MOUNT_C="/mnt/windows"
MOUNT_D="/mnt/win_data"
# ntfs-3g：utf8/locale 可用；nofail / x-systemd.* 由 systemd/util-linux 处理
FSTYPE="ntfs-3g"
OPTS="uid=${UID_NUM},gid=${GID_NUM},umask=022,windows_names,locale=zh_CN.UTF-8,nofail,x-systemd.device-timeout=8"

mkdir -p "${MOUNT_C}" "${MOUNT_D}"

FSTAB="/etc/fstab"
BACKUP="/etc/fstab.bak.windows-mounts.$(date +%Y%m%d%H%M%S)"
cp -a "${FSTAB}" "${BACKUP}"
echo "已备份 fstab → ${BACKUP}"

# 清掉旧的重复 Windows 注释/UUID 行，再重写
tmp="$(mktemp)"
grep -vE "^[[:space:]]*UUID=(5282B7F582B7DBA3|FA60BAE960BAABAF)[[:space:]]" "${FSTAB}" \
  | grep -vE "^# Windows C:" \
  | grep -vE "^# Windows D:" \
  > "${tmp}"
mv "${tmp}" "${FSTAB}"

{
  echo ""
  echo "# Windows C: (nvme0n1p3) — persistent mount"
  echo "UUID=${WIN_C_UUID}  ${MOUNT_C}  ${FSTYPE}  ${OPTS}  0  0"
  echo ""
  echo "# Windows D: (nvme1n1p2 / 新加卷) — persistent mount"
  echo "UUID=${WIN_D_UUID}  ${MOUNT_D}  ${FSTYPE}  ${OPTS}  0  0"
} >> "${FSTAB}"
echo "已写入 ntfs-3g fstab 条目"

systemctl daemon-reload 2>/dev/null || true

# 先修 dirty 位（双系统常见）
ntfsfix "/dev/disk/by-uuid/${WIN_D_UUID}" || true
ntfsfix "/dev/disk/by-uuid/${WIN_C_UUID}" || true

echo "挂载 D 盘..."
if mount "${MOUNT_D}"; then
  echo "D OK → ${MOUNT_D}"
  ls "${MOUNT_D}" | head -20
else
  echo "D 仍失败，尝试 remove_hiberfile（会清 Windows 休眠文件）..."
  mount -t ntfs-3g -o "uid=${UID_NUM},gid=${GID_NUM},umask=022,remove_hiberfile" \
    "/dev/disk/by-uuid/${WIN_D_UUID}" "${MOUNT_D}"
  ls "${MOUNT_D}" | head -20
fi

echo "挂载 C 盘..."
mount "${MOUNT_C}" || mount -t ntfs-3g -o "uid=${UID_NUM},gid=${GID_NUM},umask=022,remove_hiberfile" \
  "/dev/disk/by-uuid/${WIN_C_UUID}" "${MOUNT_C}" || echo "C 盘暂挂不上（可稍后再试）"

echo "---"
findmnt "${MOUNT_C}" "${MOUNT_D}" || true
