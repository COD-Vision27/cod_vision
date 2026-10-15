#!/usr/bin/env python3
"""Regression tests for the offline telemetry evaluator."""

from __future__ import annotations

import json
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from metrics_summary import pair_visual_targets, read_events, summarize  # noqa: E402


class PairingTests(unittest.TestCase):
    def test_state_estimate_schema_is_counted_and_validated(self) -> None:
        base = {"schema": "daedalus.simulation_metrics", "version": 1}
        path = Path(self.id().replace(".", "_") + ".jsonl")
        try:
            path.write_text(
                json.dumps({
                    **base,
                    "event": "state_estimate",
                    "frame_seq": 3,
                    "target_index": 0,
                    "target_index_scope": "frame",
                    "tracker_state": "tracking",
                    "state_valid": True,
                    "ekf_x": [0.0, 1.0],
                    "armor_xyza_list": [[1.0, 2.0, 3.0, 0.0]],
                }) + "\n",
                encoding="utf-8",
            )
            events = read_events(path)
            self.assertEqual(summarize(events)["state_estimates"], 1)
        finally:
            path.unlink(missing_ok=True)

    def test_assignment_is_not_event_order_dependent(self) -> None:
        base = {"schema": "daedalus.simulation_metrics", "version": 1}
        ground_truth = [
            {**base, "event": "ground_truth_armor", "frame_seq": 1,
             "yaw_deg": 0.0, "pitch_deg": 0.0, "visible": True},
            {**base, "event": "ground_truth_armor", "frame_seq": 1,
             "yaw_deg": 2.0, "pitch_deg": 0.0, "visible": True},
        ]
        estimates = [
            {**base, "event": "visual_estimate", "frame_seq": 1,
             "yaw_deg": 0.95, "pitch_deg": 0.0},
            {**base, "event": "visual_estimate", "frame_seq": 1,
             "yaw_deg": -1.0, "pitch_deg": 0.0},
        ]

        result = pair_visual_targets(ground_truth + estimates)

        self.assertEqual(result["true_positives"], 2)
        self.assertEqual(result["false_positives"], 0)
        self.assertEqual(result["false_negatives"], 0)
        self.assertEqual(result["f1"], 1.0)

    def test_wraps_yaw_and_excludes_invisible_truth(self) -> None:
        base = {"schema": "daedalus.simulation_metrics", "version": 1}
        events = [
            {**base, "event": "ground_truth_armor", "frame_seq": 4,
             "yaw_deg": 179.0, "pitch_deg": 1.0, "visible": True},
            {**base, "event": "ground_truth_armor", "frame_seq": 4,
             "yaw_deg": 20.0, "pitch_deg": 1.0, "visible": False},
            {**base, "event": "visual_estimate", "frame_seq": 4,
             "yaw_deg": -179.0, "pitch_deg": 1.0},
        ]

        result = pair_visual_targets(events, angle_threshold_deg=3.0)

        self.assertEqual(result["ground_truth_with_angles"], 1)
        self.assertEqual(result["estimates_with_angles"], 1)
        self.assertEqual(result["true_positives"], 1)
        self.assertAlmostEqual(result["angular_error_mae_deg"], 2.0)

    def test_unmatched_estimate_is_false_positive(self) -> None:
        base = {"schema": "daedalus.simulation_metrics", "version": 1}
        events = [
            {**base, "event": "ground_truth_armor", "frame_seq": 7,
             "target_id": "tracked", "yaw_deg": 5.0, "pitch_deg": 0.0},
            {**base, "event": "visual_estimate", "frame_seq": 7,
             "target_id": "tracked", "yaw_deg": 5.2, "pitch_deg": 0.0},
            {**base, "event": "visual_estimate", "frame_seq": 7,
             "target_id": "spurious", "yaw_deg": 80.0, "pitch_deg": 0.0},
        ]

        result = pair_visual_targets(events)

        self.assertEqual(result["matched_pairs"], 1)
        self.assertEqual(result["true_positives"], 1)
        self.assertEqual(result["false_positives"], 1)
        self.assertEqual(result["false_negatives"], 0)

    def test_missing_angles_are_not_counted_as_scored_records(self) -> None:
        base = {"schema": "daedalus.simulation_metrics", "version": 1}
        events = [
            {**base, "event": "ground_truth_target", "frame_seq": 2,
             "yaw_deg": 1.0, "pitch_deg": 0.0},
            {**base, "event": "visual_estimate", "frame_seq": 2,
             "yaw_deg": 1.0},
        ]

        result = pair_visual_targets(events)

        self.assertEqual(result["ground_truth_with_angles"], 1)
        self.assertEqual(result["estimates_with_angles"], 0)
        self.assertEqual(result["matched_pairs"], 0)
        self.assertEqual(result["false_negatives"], 1)

    def test_large_frame_assignment_is_fast_and_complete(self) -> None:
        base = {"schema": "daedalus.simulation_metrics", "version": 1}
        count = 32
        ground_truth = [
            {**base, "event": "ground_truth_armor", "frame_seq": 8,
             "target_id": index, "yaw_deg": float(index),
             "pitch_deg": 0.0, "visible": True}
            for index in range(count)
        ]
        estimates = [
            {**base, "event": "visual_estimate", "frame_seq": 8,
             "target_id": 1000 + index, "yaw_deg": float(index) + 0.1,
             "pitch_deg": 0.0}
            for index in range(count)
        ]

        started = time.perf_counter()
        result = pair_visual_targets(ground_truth + estimates)
        elapsed = time.perf_counter() - started

        self.assertLess(elapsed, 1.0)
        self.assertEqual(result["matched_pairs"], count)
        self.assertEqual(result["true_positives"], count)
        self.assertEqual(result["false_positives"], 0)
        self.assertEqual(result["false_negatives"], 0)


if __name__ == "__main__":
    unittest.main()
