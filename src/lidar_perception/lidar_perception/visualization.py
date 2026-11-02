"""Publish tracked boxes and geographic labels as ROS markers."""

import math
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration as BuiltinDuration
from .geometry import get_quaternion_from_euler, sensor_pose, enu_to_lonlat, heading_in_map


class DetectionVisualizer:
    def __init__(self, class_names, publisher, tracker, latitude, longitude, offset_x, offset_y, rotation, roll=0.0, pitch=0.0, height=0.0, ref_lat=34.5878275, ref_lon=113.6800472):
        self.class_names = class_names
        self.marker_pub = publisher
        self.tracker = tracker
        self.last_pos_cache = {}
        self.rotation, self.translation = sensor_pose(latitude,longitude,offset_x,offset_y,rotation,ref_lat,ref_lon,height,roll,pitch)
        self.ref_lat, self.ref_lon = ref_lat, ref_lon

    def publish(self, boxes, labels, raw_scores, header):
        f_boxes, f_labels, f_scores, f_ids = self.tracker.update(
            boxes, labels, raw_scores
        )
        marker_array = MarkerArray()
        ros_duration = BuiltinDuration(sec=0, nanosec=600000000)
        current_frame_cache = {}
        for i in range(len(f_boxes)):
            box, label, score, t_id = (f_boxes[i], f_labels[i], f_scores[i], f_ids[i])
            lidar_x, lidar_y = (float(box[0]), float(box[1]))
            world = self.rotation @ np.asarray(box[:3], dtype=float) + self.translation
            global_lon, global_lat = enu_to_lonlat(world[0],world[1],self.ref_lat,self.ref_lon)
            heading_deg = heading_in_map(float(box[6]),self.rotation)
            if t_id in self.last_pos_cache:
                prev_x, prev_y, prev_heading = self.last_pos_cache[t_id]
                dx = lidar_x - prev_x
                dy = lidar_y - prev_y
                dist = math.hypot(dx, dy)
                if dist > 0.1:
                    move_rad = math.atan2(dy, dx)
                    move_deg = heading_in_map(move_rad,self.rotation)
                    heading_deg = move_deg
                else:
                    heading_deg = prev_heading
            heading_deg = (heading_deg + 360) % 360
            current_frame_cache[t_id] = (lidar_x, lidar_y, heading_deg)
            color = (
                (1.0, 0.0, 0.0, 1.0)
                if label == 1
                else (0.0, 0.3, 0.9, 1.0) if label == 2 else (1.0, 1.0, 0.0, 1.0)
            )
            m = Marker()
            m.header = header
            m.ns = "boxes"
            m.id = t_id
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (
                lidar_x,
                lidar_y,
                float(box[2]),
            )
            q = get_quaternion_from_euler(0, 0, float(box[6]))
            (
                m.pose.orientation.x,
                m.pose.orientation.y,
                m.pose.orientation.z,
                m.pose.orientation.w,
            ) = (q[0], q[1], q[2], q[3])
            m.scale.x = 0.08
            m.color.r, m.color.g, m.color.b, m.color.a = color
            l, w, h = (float(box[3]), float(box[4]), float(box[5]))
            x_c = [l / 2, l / 2, l / 2, l / 2, -l / 2, -l / 2, -l / 2, -l / 2]
            y_c = [w / 2, w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2]
            z_c = [h / 2, -h / 2, h / 2, -h / 2, h / 2, -h / 2, h / 2, -h / 2]
            conn = [
                (0, 1),
                (2, 3),
                (4, 5),
                (6, 7),
                (0, 2),
                (2, 6),
                (6, 4),
                (4, 0),
                (1, 3),
                (3, 7),
                (7, 5),
                (5, 1),
            ]
            for s, e in conn:
                m.points.append(Point(x=x_c[s], y=y_c[s], z=z_c[s]))
                m.points.append(Point(x=x_c[e], y=y_c[e], z=z_c[e]))
            m.lifetime = ros_duration
            marker_array.markers.append(m)
            class_name = self.class_names[label - 1].capitalize()
            t = Marker()
            t.header = header
            t.ns = "labels"
            t.id = t_id
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = lidar_x
            t.pose.position.y = lidar_y
            t.pose.position.z = float(box[2] + h / 2 + 0.8)
            t.text = f"ID:{t_id} {class_name}\nLat: {global_lat:.7f}\nLon: {global_lon:.7f}\nAng: {heading_deg:.1f}°"
            t.scale.z = 0.8
            t.color.r, t.color.g, t.color.b, t.color.a = (1.0, 1.0, 1.0, 1.0)
            t.lifetime = ros_duration
            marker_array.markers.append(t)
        self.last_pos_cache = current_frame_cache
        self.marker_pub.publish(marker_array)
