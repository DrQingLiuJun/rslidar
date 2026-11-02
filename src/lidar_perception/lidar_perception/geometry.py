"""Shared right-handed ENU extrinsics; angles are degrees in configuration."""
import math
import numpy as np

EARTH_RADIUS = 6378137.0

def get_quaternion_from_euler(roll, pitch, yaw):
    cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2);cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]

def rotation_matrix(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    roll,pitch,yaw=map(math.radians,(roll_deg,pitch_deg,yaw_deg));cr,sr=math.cos(roll),math.sin(roll);cp,sp=math.cos(pitch),math.sin(pitch);cy,sy=math.cos(yaw),math.sin(yaw)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr], [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr], [-sp,cp*sr,cp*cr]])

def sensor_pose(rsu_lat, rsu_lon, offset_x, offset_y, rotation_deg, ref_lat=34.5878275, ref_lon=113.6800472, z=0.0, roll_deg=0.0, pitch_deg=0.0):
    values=[rsu_lat,rsu_lon,offset_x,offset_y,rotation_deg,ref_lat,ref_lon,z,roll_deg,pitch_deg]
    if not all(math.isfinite(v) for v in values): raise ValueError("Extrinsic parameters must be finite")
    x=math.radians(rsu_lon-ref_lon)*EARTH_RADIUS*math.cos(math.radians(ref_lat))+offset_x;y=math.radians(rsu_lat-ref_lat)*EARTH_RADIUS+offset_y
    return rotation_matrix(roll_deg,pitch_deg,rotation_deg),np.array([x,y,z])

def calc_tf_args(rsu_lat, rsu_lon, offset_x, offset_y, rotation_deg, ref_lat=34.5878275, ref_lon=113.6800472, z=0.0, roll_deg=0.0, pitch_deg=0.0):
    _,translation=sensor_pose(rsu_lat,rsu_lon,offset_x,offset_y,rotation_deg,ref_lat,ref_lon,z,roll_deg,pitch_deg)
    return [str(v) for v in [*translation,math.radians(rotation_deg),math.radians(pitch_deg),math.radians(roll_deg)]]

def enu_to_lonlat(x, y, ref_lat=34.5878275, ref_lon=113.6800472):
    return ref_lon+math.degrees(x/(EARTH_RADIUS*math.cos(math.radians(ref_lat)))),ref_lat+math.degrees(y/EARTH_RADIUS)

def heading_in_map(local_yaw, rotation):
    direction=rotation@np.array([math.cos(local_yaw),math.sin(local_yaw),0.0])
    return math.degrees(math.atan2(direction[1],direction[0]))%360
