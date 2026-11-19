import unittest
import json
import tempfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from embodied_navigation.localization_mapping.evaluation.metrics import (
    pose, interpolate, initial_alignment, absolute_error, relative_error,
    optimization_changes, summarize, evaluate_run, BASE_TO_IMU)


class SlamEvaluationMetricsTests(unittest.TestCase):
    def test_alignment_recovers_full_rigid_transform(self):
        x = np.array([pose([i, i * .1, 0], Rotation.from_euler('xyz', [.1, .2, i * .1]).as_quat()) for i in range(6)])
        A = pose([2, -3, 4], Rotation.from_euler('xyz', [.3, -.2, 1.2]).as_quat())
        g = A @ x
        np.testing.assert_allclose(initial_alignment(x, g), A, atol=1e-12)
        error = absolute_error(x[-1], g[-1], A)
        self.assertLess(error['translation_m'], 1e-12)
        self.assertLess(error['rotation_rad'], 1e-12)

    def test_position_subtraction_is_not_pose_correction(self):
        x = pose([0, 0, 0], [0, 0, 0, 1])
        g = pose([0, 0, 0], Rotation.from_euler('y', .4).as_quat())
        result = absolute_error(x, g, np.eye(4))
        self.assertEqual(result['translation_m'], 0.)
        self.assertAlmostEqual(result['rotation_rad'], .4)

    def test_slerp_crosses_pi_and_never_extrapolates(self):
        poses = [pose([0, 0, 0], Rotation.from_euler('z', a, degrees=True).as_quat()) for a in (179, -179)]
        values, valid = interpolate([0, 2], poses, [-1, 0, 1, 2, 3])
        self.assertEqual(valid.tolist(), [False, True, True, True, False])
        self.assertTrue(np.isnan(values[0]).all())
        self.assertAlmostEqual(Rotation.from_matrix(values[2, :3, :3]).magnitude(), np.pi)

    def test_relative_error_is_invariant_to_world_left_multiplication(self):
        xi = pose([1, 2, 3], Rotation.from_euler('x', .1).as_quat())
        xj = pose([3, 1, 4], Rotation.from_euler('y', .2).as_quat())
        gi = pose([0, 0, 0], [0, 0, 0, 1])
        gj = pose([2, 0, 1], Rotation.from_euler('z', .3).as_quat())
        A = pose([4, -3, 2], Rotation.from_euler('xyz', [.4, .5, .6]).as_quat())
        expected = relative_error(xi, xj, gi, gj)
        actual = relative_error(A @ xi, A @ xj, A @ gi, A @ gj)
        for key in expected:
            self.assertAlmostEqual(expected[key], actual[key])

    def test_same_node_optimization_retains_full_transform_and_edges(self):
        old = pose([1, 0, 0], [0, 0, 0, 1])
        D = pose([0, 0, 2], Rotation.from_euler('z', np.pi / 2).as_quat())
        result = optimization_changes({'keys': [8], 'poses': [old], 'edges': [{'a': 8}]},
                                      {'keys': [8], 'poses': [D @ old], 'edges': [{'a': 8, 'effective_weight': .5}]})
        np.testing.assert_allclose(result['nodes'][0]['T_after_inverse_before'], D, atol=1e-12)
        np.testing.assert_allclose(result['nodes'][0]['world_displacement_xyz_m'], [-1, 1, 2], atol=1e-12)
        self.assertAlmostEqual(result['nodes'][0]['rotation_rad'], np.pi / 2)
        self.assertEqual(result['edges_after'][0]['effective_weight'], .5)

    def test_end_to_end_run_files(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run / 'capture').mkdir()
            (run / 'trajectories').mkdir()
            (run / 'map').mkdir()
            (run / 'stages').mkdir()
            truth, estimates, keys = [], [], []
            for t in range(121):
                position = [t * .1, 0, 0]
                T = pose(np.array(position) + BASE_TO_IMU, [0, 0, 0, 1])
                truth.append({'time': t, 'position': position, 'quat_wxyz': [1, 0, 0, 0]})
                estimates.append({'stamp_ns': t * 10**9, 'T_parent_child': T.tolist()})
                if t % 10 == 0:
                    keys.append({'key': [0, t], 'stamp_ns': t * 10**9, 'T_odom_body': T.tolist(), 'T_world_body': T.tolist()})
            def save(path, rows):
                path.write_text(''.join(json.dumps(row) + '\n' for row in rows))
            save(run / 'capture/truth_scoring_stream.jsonl', truth)
            for name in ('frontend_post_vio', 'frontend_lio', 'backend_online'):
                save(run / f'trajectories/{name}.jsonl', estimates)
            (run / 'map/map.json').write_text(json.dumps({'keyframes': keys}))
            save(run / 'stages/events.jsonl', [{'stage': 'downstairs_loop_1', 'event': event, 'stamp_ns': t * 10**9} for event, t in [('start', 0), ('end', 120)]])
            result = evaluate_run(run)
            self.assertTrue((run / 'evaluation/metrics.json').exists())
            stream = result['streams']['frontend_post_vio']
            self.assertEqual(stream['summary']['samples'], 121)
            self.assertEqual(stream['summary']['rpe_100s']['samples'], 21)
            self.assertLess(stream['summary']['ate_translation_rmse_m'], 1e-12)
            self.assertEqual(result['streams']['final_keyframes']['summary']['samples'], 13)
            self.assertEqual(stream['stages'][0]['stage'], 'downstairs_loop_1')

    def test_missing_samples_are_null(self):
        summary = summarize([])
        self.assertIsNone(summary['ate_translation_rmse_m'])
        self.assertIsNone(summary['rpe_100s']['rte_translation_rmse_m'])


if __name__ == '__main__':
    unittest.main()
