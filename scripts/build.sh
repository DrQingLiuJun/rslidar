#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$RSLIDAR_WORKSPACE"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-6}"
exec colcon --log-base .cache/colcon/log build     --build-base .cache/colcon/build --install-base .cache/colcon/install     --paths src/rslidar_msg src/rslidar_sdk src/lidar_perception     --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
