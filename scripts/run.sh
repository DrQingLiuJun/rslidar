#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$RSLIDAR_WORKSPACE/vendor/OpenPCDet"
exec ros2 launch lidar_perception perception.launch.py "$@"
