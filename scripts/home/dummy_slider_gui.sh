#!/usr/bin/env bash
# 启动 Dummy 六轴拖动 GUI（CDC ASCII）
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec python3 "${ROOT}/scripts/home/dummy_slider_gui.py" "$@"
