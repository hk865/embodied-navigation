"""Offline native spline evaluation and active-goal association."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from embodied_navigation.simulation.dynanav.pct_scan_worker import active_request, sample_bspline


class PctScanWorkerTests(unittest.TestCase):
    def test_native_spline_sampling_uses_curve_not_control_polygon(self):
        points = [[0, 0, 0], [1, 2, .3], [2, 0, .6], [3, 2, .9]]
        sampled, times = sample_bspline(3, [-3, -2, -1, 0, 1, 2, 3, 4], points)
        self.assertTrue(np.allclose(sampled[0], [1, 4 / 3, .3]))
        self.assertTrue(np.allclose(sampled[-1], [2, 2 / 3, .6]))
        self.assertEqual(times[0], 0)
        self.assertEqual(times[-1], 1)
        self.assertTrue(np.all(np.diff(np.asarray(sampled)[:, 2]) > 0))

    def test_invalid_spline_rejected_without_polyline_fallback(self):
        with self.assertRaises(ValueError):
            sample_bspline(3, [0, 0, 0], [[0, 0, 0]] * 4)

    def test_active_observation_owns_cancel_and_generation(self):
        with tempfile.TemporaryDirectory() as directory:
            mailbox = Path(directory)
            request = dict(version=1, run_id='test', seq=7, goal_xyz_m=[1, 2, 3],
                frame_id='map', child_frame_id='imu', pose_source='livo_post_vio')
            (mailbox / 'planner_request.json').write_text(json.dumps(request))
            observation = dict(run_id='test', waypoints=dict(active_seq=7))
            (mailbox / 'observation.json').write_text(json.dumps(observation))
            self.assertEqual(active_request(mailbox, 'test'), request)
            observation['waypoints']['active_seq'] = None
            (mailbox / 'observation.json').write_text(json.dumps(observation))
            self.assertIsNone(active_request(mailbox, 'test'))
            self.assertIsNone(active_request(mailbox, 'other-generation'))

    def test_malformed_goal_never_crashes_or_starts_planner(self):
        with tempfile.TemporaryDirectory() as directory:
            mailbox = Path(directory)
            request = dict(version=1, run_id='test', seq=7, goal_xyz_m='invalid',
                frame_id='map', child_frame_id='imu', pose_source='livo_post_vio')
            (mailbox / 'planner_request.json').write_text(json.dumps(request))
            (mailbox / 'observation.json').write_text(json.dumps(dict(run_id='test', waypoints=dict(active_seq=7))))
            self.assertIsNone(active_request(mailbox, 'test'))


if __name__ == '__main__':
    unittest.main()
