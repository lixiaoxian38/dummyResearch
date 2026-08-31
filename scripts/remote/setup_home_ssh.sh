#!/usr/bin/env bash
# 家里 Linux 一次性：SSH 服务 + 授权单位 Windows 公钥
# 用法: bash scripts/remote/setup_home_ssh.sh

set -euo pipefail

LINUX_USER="${USER}"
AUTH_KEYS="${HOME}/.ssh/authorized_keys"
WINDOWS_PUBKEY="ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIMqXLoV96rp2agKlynZ79LeN6mq59cTMR8kSsN7/GHYt lxx"

echo "=========================================="
echo " 家里 Linux — SSH 远程开发配置"
echo " 用户: ${LINUX_USER}"
echo "=========================================="

# 1. openssh-server
if ! dpkg -s openssh-server >/dev/null 2>&1; then
  echo "[1/4] 安装 openssh-server ..."
  sudo apt update
  sudo apt install -y openssh-server
else
  echo "[1/4] openssh-server 已安装"
fi

sudo systemctl enable --now ssh
echo "✅ SSH 服务已启用"

# 2. authorized_keys
echo "[2/4] 配置公钥授权 ..."
mkdir -p "${HOME}/.ssh"
chmod 700 "${HOME}/.ssh"
touch "${AUTH_KEYS}"
chmod 600 "${AUTH_KEYS}"

if grep -qF "${WINDOWS_PUBKEY}" "${AUTH_KEYS}" 2>/dev/null; then
  echo "ℹ️  单位公钥已存在"
else
  echo "${WINDOWS_PUBKEY}" >> "${AUTH_KEYS}"
  echo "✅ 已添加单位 Windows 公钥"
fi

# 3. Tailscale
echo "[3/4] Tailscale 状态 ..."
if command -v tailscale >/dev/null 2>&1; then
  TS_IP="$(tailscale ip -4 2>/dev/null || true)"
  TS_NAME="$(tailscale status --json 2>/dev/null | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('Self',{}).get('HostName',''))" 2>/dev/null || hostname)"
  if [[ -n "${TS_IP}" ]]; then
    echo "✅ Tailscale IP:   ${TS_IP}"
    echo "✅ Tailscale 主机: ${TS_NAME}"
  else
    echo "⚠️  Tailscale 未连接，请运行: sudo tailscale up"
  fi
else
  echo "⚠️  未安装 Tailscale"
  echo "   curl -fsSL https://tailscale.com/install.sh | sh && sudo tailscale up"
fi

# 4. 防火墙提示
echo "[4/4] 防火墙 ..."
if command -v ufw >/dev/null 2>&1 && sudo ufw status 2>/dev/null | grep -q "Status: active"; then
  sudo ufw allow OpenSSH
  sudo ufw allow in on tailscale0 2>/dev/null || true
  echo "✅ ufw 已放行 SSH / tailscale0"
else
  echo "ℹ️  ufw 未启用或未安装，跳过"
fi

echo ""
echo "=========================================="
echo " ✅ 家里 SSH 配置完成"
echo ""
echo " 在单位 Windows 测试:"
echo "   ssh ${LINUX_USER}@<TailscaleIP> hostname"
echo ""
echo " 单位 Cursor 连接 Host: dummy-home"
echo " 打开目录: /home/lxx/Projects/dummyResearch"
echo " 详见: docs/CURSOR_REMOTE.md"
echo "=========================================="
