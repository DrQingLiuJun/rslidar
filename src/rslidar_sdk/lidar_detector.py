#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from pyproj import CRS, Transformer
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
import torch
import time
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration as BuiltinDuration

# OpenPCDet imports
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.datasets import DatasetTemplate
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils

# rsu 
from rsu_interfaces.msg import DetectedObject, DetectedObjectArray

# --- 辅助函数：替代 tf.transformations ---
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
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class RosDemoDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=False, logger=None):
        super().__init__(dataset_cfg=dataset_cfg, class_names=class_names, training=training, logger=logger)

    def prepare_ros_data(self, points):
        input_dict = {'points': points, 'frame_id': 0}
        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict

class PVRCNN_ROS(Node):
    def __init__(self, cfg_file, ckpt):
        super().__init__('pv_rcnn_tracker_node')
        
        self.logger = common_utils.create_logger()
        cfg_from_yaml_file(cfg_file, cfg)
        self.class_names = cfg.CLASS_NAMES
        self.dataset = RosDemoDataset(dataset_cfg=cfg.DATA_CONFIG, class_names=self.class_names, training=False, logger=self.logger)
        
        self.model = build_network(model_cfg=cfg.MODEL, num_class=len(self.class_names), dataset=self.dataset)
        self.model.load_params_from_file(filename=ckpt, logger=self.logger, to_cpu=True)
        self.model.cuda()
        self.model.eval()


        # ======== 新增：坐标转换初始化 (从参考程序迁移) ========
        # 1. 设置基站(RSU)参数
        self.RSU_LAT = 34.5865343
        self.RSU_LON = 113.6796418
        self.CALIB_ROTATION_DEG = 127.7422  
        self.CALIB_OFFSET_X = 0.9268
        self.CALIB_OFFSET_Y = 0.7425

        # 2. 初始化转换器 (ENU -> WGS84)
        # 定义投影字符串，以 RSU 位置为原点建立局部坐标系
        proj_str = f"+proj=aeqd +lat_0={self.RSU_LAT} +lon_0={self.RSU_LON} +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"
        # always_xy=True 确保输出格式为 (经度, 纬度)
        self.to_wgs84 = Transformer.from_crs(CRS.from_proj4(proj_str), CRS.from_epsg(4326), always_xy=True)

        # 3. 预计算旋转矩阵参数 (避免在循环中重复计算)
        theta_rad = np.radians(self.CALIB_ROTATION_DEG)
        self.cos_t = np.cos(theta_rad)
        self.sin_t = np.sin(theta_rad)
        
        self.get_logger().info("✅ 坐标转换器初始化完成")
        
        # ======== 【新增】用于记录目标上一帧位置的缓存 ========
        self.last_pos_cache = {} 
        # ====================================================


        # --- 追踪与平滑参数 ---
        self.track_cache = {}      
        self.next_track_id = 0
        self.alpha_score = 0.15    
        self.alpha_geo = 0.10      
        self.alpha_yaw = 0.08      
        
        self.dist_limit = 1.2      
        self.move_threshold = 0.12  
        self.high_thresh = 0.55    
        self.low_thresh = 0.35     
        self.max_age_static = 8    
        
        self.max_points = 80000    

        # ROS 2 Publisher & Subscriber
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        # 定义 QoS 策略 (匹配常见的雷达驱动设置)
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. 创建订阅者：接收原始点云
        # 请确认你的雷达话题名称是否为 '/rslidar_points'
        self.sub = self.create_subscription(
            PointCloud2, 
            '/rslidar_points',      # <--- ⚠️ 如果你的话题名不同，请修改这里
            self.callback, 
            qos_policy
        )

        # 2. 创建发布者：发布检测结果 (3D框和文字)
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            'detection_result',     # 发布的话题名
            10
        )

        # 发布给 C++ 融合节点的话题
        self.lidar_pub = self.create_publisher(
            DetectedObjectArray, 
            '/perception/lidar/objects',  # 话题名
            10
        )
        
        self.last_log_time = self.get_clock().now()
        self.get_logger().info("🚀 ROS 2 PVRCNN ID追踪模式已启动")

    def filter_static_obstacles(self, points):
        # 你的自定义过滤逻辑保持不变
        mask1 = (points[:, 0] > 16.5) & (points[:, 0] < 26.5) & \
                (points[:, 1] > -2.8) & (points[:, 1] < 7.2) & \
                (points[:, 2] > -5.0) & (points[:, 2] < -3.0)
        mask2 = (points[:, 0] > 14.5) & (points[:, 0] < 15.5) & \
                (points[:, 1] > 4.0) & (points[:, 1] < 5.5)
        mask3 = (points[:, 0] > 20.0) & (points[:, 0] < 25.0) & \
                (points[:, 1] > -24.0) & (points[:, 1] < -21.0) & \
                (points[:, 2] > -6.0) & (points[:, 2] < -4.5)
        return points[~(mask1 | mask2 | mask3)]

    def process_tracking(self, cur_boxes, cur_labels, cur_scores):
        # 追踪逻辑
        updated_tracks = {}
        matched_cur_indices = set()

        for t_id, (last_score, last_box, last_label, age, was_moving) in self.track_cache.items():
            best_dist = self.dist_limit
            best_idx = -1
            for i in range(len(cur_boxes)):
                if i in matched_cur_indices: continue
                if cur_labels[i] != last_label: continue
                
                dist = np.linalg.norm(cur_boxes[i][0:3] - last_box[0:3])
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            
            if best_idx != -1:
                matched_cur_indices.add(best_idx)
                s_new, b_new = cur_scores[best_idx], cur_boxes[best_idx]
                
                dist_moved = np.linalg.norm(b_new[0:3] - last_box[0:3])
                is_moving = dist_moved > self.move_threshold

                if is_moving:
                    updated_tracks[t_id] = [s_new, b_new, last_label, 0, True]
                else:
                    smooth_s = self.alpha_score * s_new + (1 - self.alpha_score) * last_score
                    smooth_b = self.alpha_geo * b_new + (1 - self.alpha_geo) * last_box
                    diff = np.arctan2(np.sin(b_new[6] - last_box[6]), np.cos(b_new[6] - last_box[6]))
                    smooth_b[6] = last_box[6] + self.alpha_yaw * diff
                    updated_tracks[t_id] = [smooth_s, smooth_b, last_label, 0, False]
            else:
                if not was_moving and age < self.max_age_static:
                    updated_tracks[t_id] = [last_score * 0.9, last_box, last_label, age + 1, False]

        for i in range(len(cur_boxes)):
            if i not in matched_cur_indices and cur_scores[i] > self.high_thresh:
                updated_tracks[self.next_track_id] = [cur_scores[i], cur_boxes[i], cur_labels[i], 0, False]
                self.next_track_id += 1

        self.track_cache = updated_tracks
        
        f_boxes, f_labels, f_scores, f_ids = [], [], [], []
        for t_id, (s, b, l, age, m) in updated_tracks.items():
            if s > self.low_thresh:
                f_boxes.append(b); f_labels.append(l); f_scores.append(s); f_ids.append(t_id)
        return f_boxes, f_labels, f_scores, f_ids

    def publish_results(self, boxes, labels, raw_scores, header):
        f_boxes, f_labels, f_scores, f_ids = self.process_tracking(boxes, labels, raw_scores)
        marker_array = MarkerArray()
        
        # 0.2秒生存时间
        ros_duration = BuiltinDuration(sec=0, nanosec=200000000)

        # 准备一个临时字典，用于更新下一帧的缓存
        current_frame_cache = {}

        # ================= 🔴 新增：填充结构化数据发送给 C++ =================
        lidar_msg = DetectedObjectArray()
        lidar_msg.header = header
        # lidar_msg.header.frame_id = "map"
        lidar_msg.sensor_ip = "lidar_main" # 给雷达一个固定的 ID 标识

        for i in range(len(f_boxes)):
            box, label, score, t_id = f_boxes[i], f_labels[i], f_scores[i], f_ids[i]
            
            # --- 1. 获取当前坐标 ---
            lidar_x, lidar_y = float(box[0]), float(box[1])
            
            # --- 2. 经纬度转换 (即使旋转角为0，也能算出相对经纬度) ---
            rot_x = lidar_x * self.cos_t - lidar_y * self.sin_t
            rot_y = lidar_x * self.sin_t + lidar_y * self.cos_t
            final_enu_x = rot_x + self.CALIB_OFFSET_X
            final_enu_y = rot_y + self.CALIB_OFFSET_Y
            global_lon, global_lat = self.to_wgs84.transform(final_enu_x, final_enu_y)

            # --- 3. 【核心】根据轨迹计算航向角 ---
            # 默认使用模型检测的朝向 (兜底策略)
            # box[6] 是弧度，转为度
            heading_deg = math.degrees(float(box[6])) + self.CALIB_ROTATION_DEG 

            # 如果这个 ID 在上一帧出现过，我们用“移动方向”来覆盖“车头朝向”
            if t_id in self.last_pos_cache:
                prev_x, prev_y, prev_heading = self.last_pos_cache[t_id]
                
                dx, dy = lidar_x - prev_x, lidar_y - prev_y
                dist = math.hypot(dx, dy)

                # 只有当移动距离足够大时(例如0.1米)，才更新航向，避免静止时乱跳
                if dist > 0.1:
                    # 计算移动向量的角度 (atan2 返回的是弧度)
                    move_rad = math.atan2(dy, dx)
                    # 加上雷达安装角(如果有)，转成全局角度
                    move_deg = math.degrees(move_rad) + self.CALIB_ROTATION_DEG
                    heading_deg = move_deg
                else:
                    # 如果静止不动，保持上一帧的航向 (比模型单帧跳动更稳)
                    heading_deg = prev_heading

            # 归一化到 0~360 度
            heading_deg = (heading_deg + 360) % 360

            
            # 更新缓存：记录当前位置和计算出的航向
            current_frame_cache[t_id] = (lidar_x, lidar_y, heading_deg)

            # --- 4. 绘制 Marker (保持原有逻辑) ---
            color = (1.0, 0.0, 0.0, 1.0) if label == 1 else ((0.0, 0.3, 0.9, 1.0) if label == 2 else (1.0, 1.0, 0.0, 1.0))
            
            # 画框
            m = Marker()
            m.header = header; m.ns = "boxes"; m.id = t_id
            m.type = Marker.LINE_LIST; m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = lidar_x, lidar_y, float(box[2])
            q = get_quaternion_from_euler(0, 0, float(box[6])) # 框的物理显示还是保留模型输出，更贴合点云
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q[0], q[1], q[2], q[3]
            m.scale.x = 0.08; m.color.r, m.color.g, m.color.b, m.color.a = color
            # ... (画框的顶点计算代码省略，与之前一致，请保留) ...
            l, w, h = float(box[3]), float(box[4]), float(box[5])
            x_c = [l/2, l/2, l/2, l/2, -l/2, -l/2, -l/2, -l/2]
            y_c = [w/2, w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2]
            z_c = [h/2, -h/2, h/2, -h/2, h/2, -h/2, h/2, -h/2]
            conn = [(0,1), (2,3), (4,5), (6,7), (0,2), (2,6), (6,4), (4,0), (1,3), (3,7), (7,5), (5,1)]
            for s, e in conn:
                m.points.append(Point(x=x_c[s], y=y_c[s], z=z_c[s]))
                m.points.append(Point(x=x_c[e], y=y_c[e], z=z_c[e]))
            m.lifetime = ros_duration
            marker_array.markers.append(m)

            # 画文字
            class_name = self.class_names[label-1].capitalize()
            t = Marker()
            t.header = header; t.ns = "labels"; t.id = t_id
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = lidar_x
            t.pose.position.y = lidar_y
            t.pose.position.z = float(box[2] + h/2 + 0.8)
            
            # 显示航向角 Ang
            t.text = f"ID:{t_id} {class_name}\nLat: {global_lat:.7f}\nLon: {global_lon:.7f}\nAng: {heading_deg:.1f}°"
            
            t.scale.z = 0.8; t.color.r, t.color.g, t.color.b, t.color.a = (1.0, 1.0, 1.0, 1.0)
            t.lifetime = ros_duration
            marker_array.markers.append(t)

            # 填充发送给 RSU_C++ 雷达的单个目标
            obj = DetectedObject()
            obj.object_id = int(t_id)
            # 类别映射：OpenPCDet输出 1:Car, 2:Pedestrian, 3:Cyclist
            if label == 1: obj.type = "car"
            elif label == 2: obj.type = "person"
            elif label == 3: obj.type = "bike"
            else: obj.type = "unknown"
            
            obj.x = final_enu_x # 发送转换后的 ENU 坐标
            obj.y = final_enu_y
            obj.z = float(box[2])
            
            # 重要：发送经纬度给 C++，因为你的 C++ 需要用经纬度来反算或做其他逻辑
            obj.longitude = global_lon
            obj.latitude = global_lat
            
            # 角度转弧度发给 ROS (ROS通常用弧度) 或者直接发度数，取决于你C++怎么解析
            # 这里建议发度数，然后在 C++ 里处理，或者保持一致
            obj.yaw = float(heading_deg) 
            
            lidar_msg.objects.append(obj)

        # 更新全局缓存，供下一帧使用
        self.last_pos_cache = current_frame_cache

        self.lidar_pub.publish(lidar_msg)
        self.marker_pub.publish(marker_array)


    def callback(self, msg):
        t_start = time.time()
        
        # 解析点云 (ROS 2 中通常需要更严谨的解析，但这里沿用你的 numpy 方式)
        points_raw = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)
        points = self.filter_static_obstacles(points_raw[:, :4].copy())
        
        if points.shape[0] > self.max_points:
            points = points[np.random.choice(points.shape[0], self.max_points, replace=False)]

        with torch.no_grad():
            data_dict = self.dataset.prepare_ros_data(points)
            data_dict = self.dataset.collate_batch([data_dict])
            load_data_to_gpu(data_dict)
            pred_dicts, _ = self.model.forward(data_dict)
            self.publish_results(pred_dicts[0]['pred_boxes'].cpu().numpy(), 
                                 pred_dicts[0]['pred_labels'].cpu().numpy(), 
                                 pred_dicts[0]['pred_scores'].cpu().numpy(), msg.header)

        t_total = (time.time() - t_start) * 1000
        
        # 日志节流实现 (Throttle)
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 1.5 * 1e9: # 1.5 seconds
            self.get_logger().info(f"[Tracking] Time: {t_total:.1f}ms | Objects: {len(self.track_cache)}")
            self.last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    
    # 路径保持原样
    cfg_file = '/home/user/rslidar_ws/src/rslidar_sdk/config/pv_rcnn6636.yaml'
    ckpt = '/home/user/rslidar_ws/src/rslidar_sdk/model/checkpoint_epoch_90_6636.pth'
    
    tracker_node = PVRCNN_ROS(cfg_file, ckpt)
    
    try:
        rclpy.spin(tracker_node)
    except KeyboardInterrupt:
        pass
    finally:
        tracker_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()