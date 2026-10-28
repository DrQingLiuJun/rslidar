#!/bin/bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export PYTHONNOUSERSITE=1 CUDA_HOME=/usr/local/cuda-12.1 TORCH_CUDA_ARCH_LIST=8.9 MAX_JOBS=6
export PATH="$CUDA_HOME/bin:$PATH"
.venv/bin/python -c 'import torch,spconv; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name())'
export CPATH="$(find "$PWD/.venv/lib/python3.10/site-packages/nvidia" -type d -name include | paste -sd:):${CPATH:-}"
cd vendor/OpenPCDet
../../.venv/bin/python -m pip install -v --no-build-isolation --no-deps -e .
