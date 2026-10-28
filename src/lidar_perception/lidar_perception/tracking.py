"""Nearest-neighbor tracking with the original smoothing and retention rules."""

import numpy as np


class TrackManager:
    def __init__(self):
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

    def update(self, cur_boxes, cur_labels, cur_scores):
        updated_tracks = {}
        matched_cur_indices = set()
        for t_id, (
            last_score,
            last_box,
            last_label,
            age,
            was_moving,
        ) in self.track_cache.items():
            best_dist = self.dist_limit
            best_idx = -1
            for i in range(len(cur_boxes)):
                if i in matched_cur_indices:
                    continue
                if cur_labels[i] != last_label:
                    continue
                dist = np.linalg.norm(cur_boxes[i][0:3] - last_box[0:3])
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            if best_idx != -1:
                matched_cur_indices.add(best_idx)
                s_new, b_new = (cur_scores[best_idx], cur_boxes[best_idx])
                dist_moved = np.linalg.norm(b_new[0:3] - last_box[0:3])
                is_moving = dist_moved > self.move_threshold
                if is_moving:
                    updated_tracks[t_id] = [s_new, b_new, last_label, 0, True]
                else:
                    smooth_s = (
                        self.alpha_score * s_new + (1 - self.alpha_score) * last_score
                    )
                    smooth_b = self.alpha_geo * b_new + (1 - self.alpha_geo) * last_box
                    diff = np.arctan2(
                        np.sin(b_new[6] - last_box[6]), np.cos(b_new[6] - last_box[6])
                    )
                    smooth_b[6] = last_box[6] + self.alpha_yaw * diff
                    updated_tracks[t_id] = [smooth_s, smooth_b, last_label, 0, False]
            elif not was_moving and age < self.max_age_static:
                updated_tracks[t_id] = [
                    last_score * 0.9,
                    last_box,
                    last_label,
                    age + 1,
                    False,
                ]
        for i in range(len(cur_boxes)):
            if i not in matched_cur_indices and cur_scores[i] > self.high_thresh:
                updated_tracks[self.next_track_id] = [
                    cur_scores[i],
                    cur_boxes[i],
                    cur_labels[i],
                    0,
                    False,
                ]
                self.next_track_id += 1
        self.track_cache = updated_tracks
        f_boxes, f_labels, f_scores, f_ids = ([], [], [], [])
        for t_id, (s, b, l, age, m) in updated_tracks.items():
            if s > self.low_thresh:
                f_boxes.append(b)
                f_labels.append(l)
                f_scores.append(s)
                f_ids.append(t_id)
        return (f_boxes, f_labels, f_scores, f_ids)
