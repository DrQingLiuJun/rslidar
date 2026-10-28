"""Regression cases for association, angle wrapping, and track expiry."""

import unittest
import numpy as np
from lidar_perception.tracking import TrackManager


class TrackingTests(unittest.TestCase):
    def test_association_keeps_id_and_smooths_wrapped_yaw(self):
        tracker = TrackManager()
        first = np.array([[1.0, 2.0, 0.0, 4.0, 2.0, 1.5, np.pi - 0.02]])
        _, _, _, ids = tracker.update(first, np.array([1]), np.array([0.9]))
        self.assertEqual(ids, [0])
        second = first.copy()
        second[0, 0] += 0.05
        second[0, 6] = -np.pi + 0.02
        boxes, _, _, ids = tracker.update(second, np.array([1]), np.array([0.8]))
        self.assertEqual(ids, [0])
        self.assertAlmostEqual(boxes[0][0], 1.005)
        self.assertAlmostEqual(boxes[0][6], np.pi - 0.02 + 0.08 * 0.04)

    def test_missing_static_track_eventually_expires(self):
        tracker = TrackManager()
        box = np.array([[1.0, 2.0, 0.0, 4.0, 2.0, 1.5, 0.0]])
        tracker.update(box, np.array([1]), np.array([0.9]))
        for _ in range(9):
            tracker.update(np.empty((0, 7)), np.array([], dtype=int), np.array([]))
        self.assertEqual(tracker.track_cache, {})

    def test_different_class_never_reuses_track_id(self):
        tracker = TrackManager()
        box = np.array([[1.0, 2.0, 0.0, 4.0, 2.0, 1.5, 0.0]])
        tracker.update(box, np.array([1]), np.array([0.9]))
        _, _, _, ids = tracker.update(box, np.array([2]), np.array([0.9]))
        self.assertEqual(ids, [0, 1])


if __name__ == "__main__":
    unittest.main()
