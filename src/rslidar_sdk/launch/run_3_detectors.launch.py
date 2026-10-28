import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# ========================================================================
# 🟢 辅助函数：经纬度 -> TF 变换参数 (x, y, z, yaw, pitch, roll)
# 算法逻辑与 C++ geo_utils1.hpp 保持绝对一致
# ========================================================================
def calc_tf_args(rsu_lat, rsu_lon, offset_x, offset_y, rotation_deg, 
                 ref_lat=34.5878275, ref_lon=113.6800472, z=0.0):
    
    R_EARTH = 6378137.0
    DEG2RAD = math.pi / 180.0

    # 1. 计算相对于原点的 ENU 坐标
    ref_lat_rad = ref_lat * DEG2RAD
    
    # x = (lon - ref_lon) * deg2rad * R * cos(ref_lat)
    x_base = (rsu_lon - ref_lon) * DEG2RAD * R_EARTH * math.cos(ref_lat_rad)
    
    # y = (lat - ref_lat) * deg2rad * R
    y_base = (rsu_lat - ref_lat) * DEG2RAD * R_EARTH

    # 2. 加上安装偏移量 (Install Offset)
    # 假设 offset 是在雷达局部坐标系下的，直接叠加到 ENU 坐标上
    # (如果 offset 是相对于杆子的偏移，通常可以直接加)
    final_x = x_base + offset_x
    final_y = y_base + offset_y

    # 3. 角度转弧度 (TF 发布需要弧度)
    yaw_rad = rotation_deg * DEG2RAD

    # 4. 返回 static_transform_publisher 需要的参数列表 (字符串格式)
    # 格式: x y z yaw pitch roll frame_id child_frame_id
    # 注意：tf2_ros 的参数顺序是 x y z yaw pitch roll
    return [str(final_x), str(final_y), str(z), str(yaw_rad), "0", "0"]


def generate_launch_description():
    pkg_share = get_package_share_directory('rslidar_sdk')
    multi_lidar_config = os.path.join(pkg_share, 'config', 'config_multi.yaml')

    # ========================================================================
    # 1. 启动雷达驱动
    # ========================================================================
    rslidar_sdk_driver = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        parameters=[{'config_path': multi_lidar_config}]
    )

    # ========================================================================
    # 2. 定义检测参数 (保持原样)
    # ========================================================================
    config_1 = os.path.join(pkg_share, 'config', 'pv_rcnn6636.yaml')
    ckpt_1   = os.path.join(pkg_share, 'model', 'checkpoint_epoch_90_6636.pth')
    
    config_2 = os.path.join(pkg_share, 'config', 'pv_rcnn6639.yaml')         
    ckpt_2   = os.path.join(pkg_share, 'model', 'checkpoint_epoch_90_6639.pth') 
    
    config_3 = os.path.join(pkg_share, 'config', 'pv_rcnn6641.yaml')      
    ckpt_3   = os.path.join(pkg_share, 'model', 'checkpoint_epoch_90_6641.pth')

    # ========================================================================
    # 3. 计算 TF 参数 (自动计算，无需手动填)
    # ========================================================================
    
    # --- 雷达 1 ---
    # rsu_lat=34.5865343, rsu_lon=113.6796418, offset_x=0.9268, offset_y=0.7425, rot=127.7422
    tf_args_1 = calc_tf_args(34.5865343, 113.6796418, 0.9268, 0.7425, 127.7422)
    
    # --- 雷达 2 ---
    # rsu_lat=34.58681631, rsu_lon=113.679107467, offset_x=0.8673, offset_y=0.7335, rot=129.3601
    tf_args_2 = calc_tf_args(34.58681631, 113.679107467, 0.86731080, 0.73356548, 129.360102)

    # --- 雷达 3 ---
    # rsu_lat=34.58656254, rsu_lon=113.6788296, offset=0, rot=0
    tf_args_3 = calc_tf_args(34.58656254, 113.6788296, 0.0, 0.0, 0.0)

    # ========================================================================
    # 4. 定义节点列表
    # ========================================================================
    return LaunchDescription([
        rslidar_sdk_driver,

        # --- 静态 TF 发布节点 ---
        # 格式: map -> rslidar_1
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_1',
            arguments=tf_args_1 + ["map", "rslidar_1"]
        ),
        # 格式: map -> rslidar_2
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_2',
            arguments=tf_args_2 + ["map", "rslidar_2"]
        ),
        # 格式: map -> rslidar_3
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_3',
            arguments=tf_args_3 + ["map", "rslidar_3"]
        ),

        # --- 检测节点 ---
        Node(
            package='rslidar_sdk', 
            executable='3D_detector.py',
            prefix=['/home/user/carla-projects/rslidar_ws/.venv/bin/python', ' '], 
            name='detector_node_1', 
            output='screen',
            parameters=[{
                'lidar_topic': '/rslidar_points_1', 'output_topic': 'detection_result_1',    
                'config_file': config_1, 'ckpt_file': ckpt_1,
                'rsu_lat': 34.5865343, 'rsu_lon': 113.6796418,
                'install_offset_x': 0.9268, 'install_offset_y': 0.7425, 'install_rotation': 127.7422
            }]
        ),
        Node(
            package='rslidar_sdk', 
            executable='3D_detector.py',
            prefix=['/home/user/carla-projects/rslidar_ws/.venv/bin/python', ' '], 
            name='detector_node_2', 
            output='screen',
            parameters=[{
                'lidar_topic': '/rslidar_points_2', 'output_topic': 'detection_result_2',
                'config_file': config_2, 'ckpt_file': ckpt_2,
                'rsu_lat': 34.58681631, 'rsu_lon': 113.679107467,
                'install_offset_x': 0.86731080, 'install_offset_y': 0.73356548, 'install_rotation': 129.360102
            }]
        ),
        Node(
            package='rslidar_sdk', 
            executable='3D_detector.py',
            prefix=['/home/user/carla-projects/rslidar_ws/.venv/bin/python', ' '], 
            name='detector_node_3', 
            output='screen',
            parameters=[{
                'lidar_topic': '/rslidar_points_3', 'output_topic': 'detection_result_3',
                'config_file': config_3, 'ckpt_file': ckpt_3,
                'rsu_lat': 34.58656254, 'rsu_lon': 113.6788296,
                'install_offset_x': 0.0, 'install_offset_y': 0.0, 'install_rotation': 0.0
            }]
        ),
    ])