#!/usr/bin/env bash
# Install LocateAnything-3B runtime for Dummy VLM locator (RTX 3060).
set -euo pipefail

ROOT="${HOME}/models"
EAGLE="${ROOT}/eagle"
VENV="${ROOT}/locateanything/venv"
MODEL="${ROOT}/LocateAnything-3B"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"

mkdir -p "${ROOT}"

echo "==> 0) NVIDIA driver check"
if ! nvidia-smi >/dev/null 2>&1; then
  echo "ERROR: nvidia-smi failed."
  echo "This machine is on kernel $(uname -r) but nvidia modules were built for 6.14."
  echo "Fix (needs sudo), pick ONE:"
  echo "  A) sudo apt install -y linux-modules-nvidia-580-open-generic-7.0 \\"
  echo "       linux-modules-nvidia-580-open-\$(uname -r) && sudo modprobe nvidia"
  echo "  B) reboot and select kernel 6.14.0-36-generic (modules already installed)"
  exit 1
fi
nvidia-smi --query-gpu=name,memory.total --format=csv

echo "==> 1) Clone NVlabs/Eagle Embodied"
if [[ ! -d "${EAGLE}/.git" ]]; then
  git clone --depth 1 https://github.com/NVlabs/Eagle.git "${EAGLE}"
fi

echo "==> 2) Python venv + deps"
python3 -m venv "${VENV}"
# shellcheck disable=SC1091
source "${VENV}/bin/activate"
pip install -U pip wheel
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
pip install flask pillow numpy
pip install -e "${EAGLE}/Embodied"

echo "==> 3) Download weights (HF)"
if [[ ! -f "${MODEL}/config.json" ]]; then
  pip install -U "huggingface_hub[cli]"
  huggingface-cli download nvidia/LocateAnything-3B --local-dir "${MODEL}"
fi

echo "==> DONE"
echo "Start server:  bash ${REPO}/scripts/vision/start_locateanything_server.sh"
echo "Then Hub → source VLM → 开始跟随"
