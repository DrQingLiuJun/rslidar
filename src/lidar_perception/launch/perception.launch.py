"""Start the driver, static transforms, and one detector per configured lidar."""

import os
import sys
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from lidar_perception.geometry import calc_tf_args


def generate_launch_description():
    share = Path(get_package_share_directory("lidar_perception"))
    driver_share = Path(get_package_share_directory("rslidar_sdk"))
    config = yaml.safe_load((share / "config/sensors.yaml").read_text())
    environment = os.environ.get("VIRTUAL_ENV")
    python = Path(environment) / "bin/python" if environment else Path(sys.executable)
    actions = [
        Node(
            package="rslidar_sdk",
            executable="rslidar_sdk_node",
            name="rslidar_sdk_node",
            output="screen",
            parameters=[{"config_path": str(driver_share / "config/radars.yaml")}],
        )
    ]
    for sensor in config["sensors"]:
        index = sensor["id"]
        parameters = dict(sensor["parameters"])
        parameters["config_file"] = str(
            share / "config/models" / sensor["model_config"]
        )
        parameters["ckpt_file"] = str(share / "models" / sensor["checkpoint"])
        tf = calc_tf_args(
            parameters["rsu_lat"],
            parameters["rsu_lon"],
            parameters["install_offset_x"],
            parameters["install_offset_y"],
            parameters["install_rotation"],
            **config["reference"],
        )
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"tf_lidar_{index}",
                arguments=tf + ["map", sensor["frame_id"]],
            )
        )
        actions.append(
            Node(
                package="lidar_perception",
                executable="detector_node",
                name=f"detector_node_{index}",
                output="screen",
                prefix=[str(python) + " "],
                parameters=[parameters],
            )
        )
    return LaunchDescription(actions)
