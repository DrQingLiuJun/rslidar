"""Exercise all checkpoints with synthetic points; this does not measure accuracy."""

from pathlib import Path
import json
import os
import numpy as np
import torch
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
from lidar_perception.config import load_model_config
from lidar_perception.dataset import PointCloudDataset


def main():
    root = Path(os.environ["RSLIDAR_WORKSPACE"]) / "src/lidar_perception"
    logger = common_utils.create_logger()
    for index in (1, 2, 3):
        cfg = load_model_config(root / f"config/models/pv_rcnn_lidar_{index}.yaml")
        dataset = PointCloudDataset(cfg.DATA_CONFIG, cfg.CLASS_NAMES, False, logger)
        model = build_network(cfg.MODEL, len(cfg.CLASS_NAMES), dataset)
        model.load_params_from_file(
            str(root / f"models/pv_rcnn_lidar_{index}.pth"), logger, to_cpu=True
        )
        model.cuda().eval()
        rng = np.random.default_rng(82)
        extent = np.array(cfg.DATA_CONFIG.POINT_CLOUD_RANGE)
        points = np.empty((20000, 4), dtype=np.float32)
        points[:, :3] = rng.uniform(
            extent[:3] + 0.2, extent[3:] - 0.2, size=(len(points), 3)
        )
        points[:, 3] = rng.uniform(0, 255, len(points))
        batch = dataset.collate_batch([dataset.prepare_ros_data(points)])
        load_data_to_gpu(batch)
        with torch.no_grad():
            predictions, _ = model(batch)
        torch.cuda.synchronize()
        print(
            json.dumps(
                {
                    "synthetic": True,
                    "lidar": index,
                    "points": len(points),
                    "predictions": len(predictions[0]["pred_boxes"]),
                }
            ),
            flush=True,
        )
        del model, batch, predictions, dataset
        torch.cuda.empty_cache()


if __name__ == "__main__":
    main()
