#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$HOME/.config/systemd/user"
sed "s|@WORKSPACE@|$workspace|g" "$workspace/deploy/rslidar.service.in" > "$HOME/.config/systemd/user/rslidar.service"
systemctl --user daemon-reload
systemctl --user restart rslidar.service
