"""Check relocatable model configs and driver/detector channel agreement."""

from pathlib import Path
import tempfile
import unittest
import yaml
from lidar_perception.config import load_model_config

ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT / "src/lidar_perception"


class ConfigTests(unittest.TestCase):
    def test_model_paths_work_outside_workspace(self):
        import os

        previous = Path.cwd()
        try:
            with tempfile.TemporaryDirectory() as directory:
                os.chdir(directory)
                for index in (1, 2, 3):
                    cfg = load_model_config(
                        SHARE / f"config/models/pv_rcnn_lidar_{index}.yaml"
                    )
                    self.assertEqual(cfg.MODEL.NAME, "PVRCNN")
                    self.assertEqual(
                        cfg.CLASS_NAMES, ["Vehicle", "Pedestrian", "Cyclist"]
                    )
                    self.assertEqual(len(cfg.DATA_CONFIG.POINT_CLOUD_RANGE), 6)
        finally:
            os.chdir(previous)

    def test_channels_and_weights_match(self):
        sensors = yaml.safe_load((SHARE / "config/sensors.yaml").read_text())["sensors"]
        drivers = yaml.safe_load(
            (ROOT / "src/rslidar_sdk/config/radars.yaml").read_text()
        )["lidar"]
        self.assertEqual(len(sensors), 3)
        for sensor, driver in zip(sensors, drivers):
            self.assertEqual(
                sensor["parameters"]["lidar_topic"],
                driver["ros"]["ros_send_point_cloud_topic"],
            )
            self.assertEqual(sensor["frame_id"], driver["ros"]["ros_frame_id"])
            self.assertTrue((SHARE / "models" / sensor["checkpoint"]).is_file())


if __name__ == "__main__":
    unittest.main()
