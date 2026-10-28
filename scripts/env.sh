#!/usr/bin/env bash
# Source from other scripts; all paths are derived from this checkout.
RSLIDAR_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RSLIDAR_WORKSPACE PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-82}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}" OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.1}"
source /opt/ros/humble/setup.bash
if [[ -f "$RSLIDAR_WORKSPACE/.cache/colcon/install/setup.bash" ]]; then
    source "$RSLIDAR_WORKSPACE/.cache/colcon/install/setup.bash"
fi
if [[ -f "$RSLIDAR_WORKSPACE/.venv/bin/activate" ]]; then
    source "$RSLIDAR_WORKSPACE/.venv/bin/activate"
fi
export PATH="$CUDA_HOME/bin:$PATH"
