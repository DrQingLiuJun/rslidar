"""ROS 2 entry point for one PV-RCNN detector."""

from pathlib import Path
import time
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
from .config import load_model_config
from .dataset import PointCloudDataset
from .preprocessing import filter_static_obstacles
from .tracking import TrackManager
from .visualization import DetectionVisualizer


class DetectorNode(Node):
    def __init__(self):
        super().__init__("pv_rcnn_detector")
        share = Path(get_package_share_directory("lidar_perception"))
        defaults = {
            "config_file": str(share / "config/models/pv_rcnn_lidar_1.yaml"),
            "ckpt_file": str(share / "models/pv_rcnn_lidar_1.pth"),
            "lidar_topic": "/rslidar_points_1",
            "output_topic": "/detection_result_1",
            "rsu_lat": 34.5865343,
            "rsu_lon": 113.6796418,
            "install_offset_x": 0.9268,
            "install_offset_y": 0.7425,
            "install_rotation": 127.7422,
            "max_points": 80000,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        params = {key: self.get_parameter(key).value for key in defaults}
        self.max_points = params["max_points"]
        logger = common_utils.create_logger()
        config = load_model_config(params["config_file"])
        self.dataset = PointCloudDataset(
            config.DATA_CONFIG, config.CLASS_NAMES, False, logger
        )
        self.model = build_network(config.MODEL, len(config.CLASS_NAMES), self.dataset)
        self.model.load_params_from_file(params["ckpt_file"], logger, to_cpu=True)
        self.model.cuda().eval()
        self.tracker = TrackManager()
        self.marker_pub = self.create_publisher(MarkerArray, params["output_topic"], 10)
        self.visualizer = DetectionVisualizer(
            config.CLASS_NAMES,
            self.marker_pub,
            self.tracker,
            params["rsu_lat"],
            params["rsu_lon"],
            params["install_offset_x"],
            params["install_offset_y"],
            params["install_rotation"],
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscription = self.create_subscription(
            PointCloud2, params["lidar_topic"], self.callback, qos
        )
        self.last_log_time = self.get_clock().now()
        self.get_logger().info(f"Ready; waiting for {params['lidar_topic']}")

    def callback(self, msg):
        start = time.time()
        points_raw = np.frombuffer(msg.data, dtype=np.float32).reshape(
            -1, msg.point_step // 4
        )
        points = filter_static_obstacles(points_raw[:, :4].copy())
        if len(points) > self.max_points:
            points = points[
                np.random.choice(len(points), self.max_points, replace=False)
            ]
        with torch.no_grad():
            batch = self.dataset.collate_batch([self.dataset.prepare_ros_data(points)])
            load_data_to_gpu(batch)
            predictions, _ = self.model(batch)
            prediction = predictions[0]
            self.visualizer.publish(
                prediction["pred_boxes"].cpu().numpy(),
                prediction["pred_labels"].cpu().numpy(),
                prediction["pred_scores"].cpu().numpy(),
                msg.header,
            )
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 1.5e9:
            elapsed = (time.time() - start) * 1000
            self.get_logger().info(
                f"[Tracking] Time: {elapsed:.1f}ms | Objects: {len(self.tracker.track_cache)}"
            )
            self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
