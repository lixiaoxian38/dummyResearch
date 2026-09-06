#!/usr/bin/env bash
# 启动标定板光轴跟随：停 tracker，经 bridge 回 handeye（不杀 bridge），再开 LIVE。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec python3 "${REPO_ROOT}/scripts/home/track_control.py" start
