#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$RSLIDAR_WORKSPACE/vendor/OpenPCDet"
exec "$RSLIDAR_WORKSPACE/.venv/bin/python" "$RSLIDAR_WORKSPACE/scripts/smoke_test.py"
