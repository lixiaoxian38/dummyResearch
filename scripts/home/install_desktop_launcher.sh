#!/usr/bin/env bash
# 把 Tracking Hub 装到桌面 + 应用菜单（可双击）。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
HUB="${SCRIPT_DIR}/start_tracking_hub.sh"
chmod +x "${HUB}" "${SCRIPT_DIR}/install_desktop_launcher.sh"

DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
if [[ -z "${DESKTOP_DIR}" || ! -d "${DESKTOP_DIR}" ]]; then
  DESKTOP_DIR="${HOME}/Desktop"
fi
APPDIR="${HOME}/.local/share/applications"
mkdir -p "${APPDIR}"

write_desktop() {
  local dest="$1"
  cat >"${dest}" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Dummy Tracking Hub
Name[zh_CN]=Dummy 跟随控制台
Comment=Start ROS Servo, D415, CDC bridge, and the tracking HUD
Comment[zh_CN]=启动 ROS Servo、相机、CDC 与跟随窗口（不自动跟随）
Exec=${HUB}
Path=${REPO_ROOT}
Icon=camera-web
Terminal=true
Categories=Utility;Science;
StartupNotify=true
EOF
  chmod +x "${dest}"
}

write_desktop "${APPDIR}/dummy-tracking-hub.desktop"
echo "已写入 ${APPDIR}/dummy-tracking-hub.desktop"

install_one() {
  local dir="$1"
  if [[ ! -d "${dir}" ]]; then
    return 1
  fi
  if [[ ! -w "${dir}" ]]; then
    echo "WARN: ${dir} 不可写（可能是 root 属主），跳过桌面图标"
    return 1
  fi
  write_desktop "${dir}/Dummy跟随控制台.desktop"
  if command -v gio >/dev/null 2>&1; then
    gio set "${dir}/Dummy跟随控制台.desktop" metadata::trusted true 2>/dev/null || true
  fi
  echo "已放到 ${dir}/Dummy跟随控制台.desktop"
  return 0
}

ok=0
install_one "${DESKTOP_DIR}" && ok=1 || true
if [[ "${DESKTOP_DIR}" != "${HOME}/桌面" ]]; then
  install_one "${HOME}/桌面" && ok=1 || true
fi

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "${APPDIR}" 2>/dev/null || true
fi

echo ""
echo "应用菜单名称: Dummy Tracking Hub / Dummy 跟随控制台"
if [[ "${ok}" -eq 0 ]]; then
  echo "桌面目录写不进去时：把下面文件复制到桌面，右键「允许启动」："
  echo "  ${APPDIR}/dummy-tracking-hub.desktop"
fi
echo "脚本也可直接: bash ${HUB}"
