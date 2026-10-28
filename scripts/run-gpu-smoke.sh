#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
source /opt/ros/humble/setup.bash
cd vendor/OpenPCDet
../../.venv/bin/python ../../scripts/gpu-smoke.py
