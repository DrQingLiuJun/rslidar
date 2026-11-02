#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
# Match the running driver even if this desktop terminal exports different ROS settings.
service_pid=$(systemctl --user show rslidar.service -p MainPID --value 2>/dev/null || true)
if [[ "$service_pid" =~ ^[1-9][0-9]*$ && -r "/proc/$service_pid/environ" ]]; then
    while IFS= read -r -d '' setting; do
        case "$setting" in ROS_DOMAIN_ID=*|ROS_LOCALHOST_ONLY=*|RMW_IMPLEMENTATION=*|CYCLONEDDS_URI=*) export "$setting" ;; esac
    done < "/proc/$service_pid/environ"
else
    echo "未发现运行中的雷达服务；如需查看实时检测，请先执行 systemctl --user start rslidar.service。" >&2
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then echo "请在服务器图形桌面的终端运行，或使用已配置图形转发的SSH。" >&2; exit 1; fi
echo "RViz: ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
exec rviz2 -d "$RSLIDAR_WORKSPACE/config/three_lidars.rviz" "$@"
