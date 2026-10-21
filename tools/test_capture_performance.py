#!/usr/bin/env python3
"""Pure-Python tests for image observation statistics; no ROS or simulator."""
import unittest
import contextlib
import io
import os
import tempfile
from pathlib import Path
from unittest.mock import patch

import capture_performance
from capture_performance import image_summary


def frame(stamp, system_receive, monotonic_receive):
    return {'stamp_ns': stamp, 'received_ros_ns': system_receive,
            'received_monotonic_ns': monotonic_receive,
            'width': 640, 'height': 480, 'encoding': 'rgb8'}


class ImageStatisticsTests(unittest.TestCase):
    def test_application_ros_domain_is_not_inherited(self):
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '25'}):
            capture_performance.isolate_ros_domain()
            self.assertEqual(os.environ['ROS_DOMAIN_ID'], '90')

    def test_non_worker_is_rejected_before_gpu_or_simulator_start(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(capture_performance, 'ROOT', Path(folder)), \
                patch('sys.argv', ['capture_performance.py']), \
                patch.object(capture_performance, 'gpu_snapshot') as gpu, \
                patch.object(capture_performance.subprocess, 'Popen') as start, \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as exit:
                capture_performance.main()
            self.assertEqual(exit.exception.code, 2)
            gpu.assert_not_called()
            start.assert_not_called()

    def test_empty_and_single_image_have_no_measurable_rate(self):
        empty = image_summary([])
        self.assertEqual(empty['images'], 0)
        self.assertIsNone(empty['receive_fps'])
        self.assertIsNone(empty['frame_age_ms']['p95'])
        single = image_summary([frame(1_000_000_000, 1_005_000_000, 100)])
        self.assertEqual(single['frame_age_ms']['p50'], 5)
        self.assertEqual(single['capture_interval_ms']['count'], 0)
        self.assertIsNone(single['receive_fps'])

    def test_steady_window_counts_only_intervals_inside_window(self):
        frames = [frame(1_000_000_000 + t, 1_002_000_000 + t, 1_000 + t)
                  for t in [0, 4_999_000_000, 5_000_000_000, 5_010_000_000, 5_020_000_000]]
        cutoff = frames[0]['received_monotonic_ns'] + 5_000_000_000
        steady = image_summary([f for f in frames if f['received_monotonic_ns'] >= cutoff])
        self.assertEqual(steady['images'], 3)
        self.assertEqual(steady['receive_interval_ms']['count'], 2)
        self.assertEqual(steady['capture_interval_ms']['p95'], 10)
        self.assertAlmostEqual(steady['receive_fps'], 100)

    def test_out_of_order_and_negative_ages_are_retained(self):
        result = image_summary([frame(20_000_000, 19_000_000, 100_000_000),
                                frame(10_000_000, 22_000_000, 110_000_000),
                                frame(10_000_000, 23_000_000, 120_000_000)])
        self.assertEqual(result['images'], 3)
        self.assertEqual(result['nonpositive_capture_intervals'], 2)
        self.assertEqual(result['negative_frame_ages'], 1)
        self.assertEqual(result['capture_interval_ms']['p50'], -5)
        self.assertEqual(result['frame_age_ms']['p50'], 12)
        # Arrival/header observations alone do not establish network packet loss.
        self.assertNotIn('network_packet_loss', result)

    def test_epoch_age_never_subtracts_monotonic_clock(self):
        epoch = 1_790_000_000_000_000_000
        result = image_summary([frame(epoch, epoch + 2_000_000, 500_000_000),
                                frame(epoch + 10_000_000, epoch + 13_000_000, 510_000_000)])
        self.assertEqual(result['frame_age_ms']['p50'], 2.5)
        self.assertAlmostEqual(result['frame_age_ms']['p95'], 2.95)
        self.assertEqual(result['receive_interval_ms']['p50'], 10)
        self.assertEqual(result['capture_interval_ms']['p50'], 10)


if __name__ == '__main__':
    unittest.main()
