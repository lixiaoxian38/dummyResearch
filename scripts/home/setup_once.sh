#!/usr/bin/env bash
# 家里 Linux 一次性初始化（只需运行一次）

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "=========================================="
echo " Dummy 家里 Server — 一次性初始化"
echo "=========================================="
echo "仓库目录: ${REPO_ROOT}"
echo ""

# 1. 配置文件
if [[ ! -f "${SCRIPT_DIR}/config.env" ]]; then
  cp "${SCRIPT_DIR}/config.env.example" "${SCRIPT_DIR}/config.env"
  echo "✅ 已创建 scripts/home/config.env（可按需修改）"
else
  echo "ℹ️  config.env 已存在，跳过"
fi

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

# 2. 系统依赖提示
echo ""
echo "[1/4] 检查系统工具..."
for cmd in git tmux curl python3 pip3; do
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "❌ 缺少 ${cmd}，请先安装: sudo apt install git tmux curl python3 python3-pip"
    exit 1
  fi
done
echo "✅ 基础工具 OK"

# 3. 编译 ROS 工作空间
echo ""
echo "[2/4] 编译 dummy_moveit_ws（可能需要几分钟）..."
cd "${WS_DIR}"
rosdep install -y -r -i --rosdistro "${ROS_DISTRO}" --from-paths src 2>/dev/null || {
  echo "⚠️  rosdep 有警告，继续编译..."
}
colcon build --symlink-install
echo "✅ 编译完成"

# 4. Python 依赖（stream API）
echo ""
echo "[3/4] 安装 stream API Python 依赖..."
pip3 install --user fastapi uvicorn pydantic 2>/dev/null || pip3 install fastapi uvicorn pydantic
echo "✅ Python 依赖 OK"

# 5. SSH + Tailscale 提示
echo ""
echo "[4/4] 网络服务（需手动确认）..."
echo ""
echo "请确认以下两项已配置（详见 AGENTS.md）："
echo "  • Tailscale: curl -fsSL https://tailscale.com/install.sh | sh && sudo tailscale up"
echo "  • SSH:       sudo apt install openssh-server && sudo systemctl enable --now ssh"
echo ""
echo "=========================================="
echo " ✅ 初始化完成！"
echo ""
echo " 以后回家只需三条命令："
echo "   cd ~/projects/dummy          # 或你的仓库路径"
echo "   git pull"
echo "   bash scripts/home/start.sh"
echo "=========================================="
