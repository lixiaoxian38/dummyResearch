#!/usr/bin/env bash
# 回家傻瓜式一键：pull 代码 + 启动服务

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

echo "=========================================="
echo " 🏠 回家一键启动"
echo "=========================================="

# 1. 拉代码
echo ""
echo "[1/3] git pull ..."
git pull

# 2. 如有新代码则增量编译
echo ""
echo "[2/3] 检查是否需要重新编译..."
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/env.sh"

cd "${WS_DIR}"
NEED_BUILD=false
if [[ ! -d install ]]; then
  NEED_BUILD=true
elif find src -newer install -print -quit 2>/dev/null | grep -q .; then
  NEED_BUILD=true
fi

if [[ "${NEED_BUILD}" == true ]]; then
  echo "🔨 检测到变更，重新编译..."
  colcon build --symlink-install
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/setup.bash"
else
  echo "✅ 无需编译"
fi

# 3. 启动
echo ""
echo "[3/3] 启动机械臂服务..."
bash "${SCRIPT_DIR}/start.sh"
