#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.9}" MAX_JOBS="${MAX_JOBS:-6}"
export CPATH="$(find "$RSLIDAR_WORKSPACE/.venv/lib/python3.10/site-packages/nvidia" -type d -name include | paste -sd:):${CPATH:-}"
cd "$RSLIDAR_WORKSPACE/vendor/OpenPCDet"
exec "$RSLIDAR_WORKSPACE/.venv/bin/python" -m pip install -v --no-build-isolation --no-deps -e .
