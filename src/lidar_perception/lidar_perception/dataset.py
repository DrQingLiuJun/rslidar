"""Inference adapter for in-memory point clouds; no training dataset required."""

from pcdet.datasets import DatasetTemplate


class PointCloudDataset(DatasetTemplate):

    def __init__(self, dataset_cfg, class_names, training=False, logger=None):
        super().__init__(
            dataset_cfg=dataset_cfg,
            class_names=class_names,
            training=training,
            logger=logger,
        )

    def prepare_ros_data(self, points):
        input_dict = {"points": points, "frame_id": 0}
        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict
