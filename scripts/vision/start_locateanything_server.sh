#!/usr/bin/env bash
set -euo pipefail
ROOT="${HOME}/models"
VENV="${ROOT}/locateanything/venv"
EAGLE="${ROOT}/eagle2/Embodied"
MODEL="${ROOT}/LocateAnything-3B"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"

if [[ ! -x "${VENV}/bin/python" ]]; then
  echo "Missing venv. Run: bash ${REPO}/scripts/vision/install_locateanything.sh"
  exit 1
fi
if ! nvidia-smi >/dev/null 2>&1; then
  echo "nvidia-smi failed — load GPU modules first (see install script)."
  exit 1
fi

# Clean env: ROS PYTHONPATH breaks transformers' cv2/decord import check.
exec env -i \
  HOME="${HOME}" \
  PATH="${VENV}/bin:/usr/bin:/bin" \
  EAGLE_EMBODIED="${EAGLE}" \
  LOCATEANYTHING_MODEL="nvidia/LocateAnything-3B" \
  LOCATEANYTHING_LOCAL="${MODEL}" \
  PYTHONPATH="${EAGLE}:${MODEL}" \
  HF_HOME="${ROOT}/hf" \
  LA_ATTN="${LA_ATTN:-sdpa}" \
  LA_USE_BATCH="${LA_USE_BATCH:-1}" \
  "${VENV}/bin/python" -u "${REPO}/scripts/vision/locateanything_server.py" --host 127.0.0.1 --port 8765
