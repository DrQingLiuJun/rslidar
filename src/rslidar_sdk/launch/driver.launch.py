"""Start the three configured RoboSense receivers without detection."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("rslidar_sdk"))
    return LaunchDescription(
        [
            Node(
                package="rslidar_sdk",
                executable="rslidar_sdk_node",
                output="screen",
                parameters=[{"config_path": str(share / "config/radars.yaml")}],
            )
        ]
    )
