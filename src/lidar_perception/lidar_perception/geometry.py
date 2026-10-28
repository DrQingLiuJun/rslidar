"""Coordinate conversion shared by launch and visualization."""

import math
import numpy as np


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.
    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


def calc_tf_args(
    rsu_lat,
    rsu_lon,
    offset_x,
    offset_y,
    rotation_deg,
    ref_lat=34.5878275,
    ref_lon=113.6800472,
    z=0.0,
):
    R_EARTH = 6378137.0
    DEG2RAD = math.pi / 180.0
    ref_lat_rad = ref_lat * DEG2RAD
    x_base = (rsu_lon - ref_lon) * DEG2RAD * R_EARTH * math.cos(ref_lat_rad)
    y_base = (rsu_lat - ref_lat) * DEG2RAD * R_EARTH
    final_x = x_base + offset_x
    final_y = y_base + offset_y
    yaw_rad = rotation_deg * DEG2RAD
    return [str(final_x), str(final_y), str(z), str(yaw_rad), "0", "0"]
