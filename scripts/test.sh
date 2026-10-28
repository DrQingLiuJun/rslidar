#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$RSLIDAR_WORKSPACE"
exec "$RSLIDAR_WORKSPACE/.venv/bin/python" -m unittest discover -s tests -v
