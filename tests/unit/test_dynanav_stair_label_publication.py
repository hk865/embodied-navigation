"""Execute the worker's real nested publication callback without a ROS runtime."""
import ast
from concurrent.futures import Future
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from embodied_navigation.simulation.dynanav import pct_scan_worker


class StairLabelPublicationTests(unittest.TestCase):
    def setup_callback(self, *, stamp=1_000_000_000, latest=1_100_000_000,
                       identity=(4, 'owned'), count=2, current=True):
        tree = ast.parse(Path(pct_scan_worker.__file__).read_text())
        run = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == 'run')
        callback = next(x for x in run.body if isinstance(x, ast.FunctionDef)
                        and x.name == 'collect_stair_labels')
        future = Future()
        future.set_result(dict(voxel_indices=np.zeros((count, 3), dtype=int),
                               models=[], observed_point_count=count, source_stamp_ns=stamp, source_frames=('map', 'imu')))
        sent, written, warnings = [], [], []
        namespace = dict(stair_job=dict(future=future, identity=identity, stamp=stamp, frames=('map', 'imu'), model_first_stamp=900_000_000),
            state=dict(request=dict(seq=4, frame_id='map', child_frame_id='imu'),
                       child=SimpleNamespace(ack_nonce='owned'), terminal_failed=False),
            current=lambda: current, source_interval=[0, latest], np=np, json=json,
            run_id='this-run', String=lambda **kwargs: SimpleNamespace(**kwargs),
            stair_pub=SimpleNamespace(publish=sent.append), mailbox=Path('not-written'),
            atomic_json=lambda path, value: written.append(value),
            node=SimpleNamespace(get_logger=lambda: SimpleNamespace(warning=warnings.append)))
        exec(compile(ast.Module(body=[callback], type_ignores=[]), '<worker-callback>', 'exec'), namespace)
        return namespace, sent, written, warnings

    def test_current_result_keeps_source_time_and_identity(self):
        env, sent, written, _ = self.setup_callback()
        env['collect_stair_labels']()
        payload = json.loads(sent[0].data)
        self.assertEqual(payload['source_stamp_ns'], 1_000_000_000)
        self.assertEqual(payload['startup_nonce'], 'owned')
        self.assertEqual(payload['localization_generation'], 0)
        self.assertEqual(payload['run_id'], 'this-run')
        self.assertEqual(payload['voxel_indices'], [[0, 0, 0], [0, 0, 0]])
        self.assertFalse(written[0]['unseen_surface_completion'])
        self.assertIsNone(env['stair_job']['future'])

    def test_changed_sequence_or_child_never_inherits_labels(self):
        for identity in ((3, 'owned'), (4, 'old-process')):
            env, sent, _, _ = self.setup_callback(identity=identity)
            env['collect_stair_labels']()
            self.assertFalse(sent)

    def test_stale_or_future_source_is_not_retimestamped(self):
        for latest in (1_300_000_001, 999_999_999):
            env, sent, _, _ = self.setup_callback(latest=latest)
            env['collect_stair_labels']()
            self.assertFalse(sent)

    def test_cancelled_request_cannot_publish(self):
        env, sent, _, _ = self.setup_callback(current=False)
        env['collect_stair_labels']()
        self.assertFalse(sent)

    def test_over_budget_clears_labels_instead_of_truncating(self):
        env, sent, _, _ = self.setup_callback(count=4097)
        env['collect_stair_labels']()
        self.assertEqual(json.loads(sent[0].data)['voxel_indices'], [])

    def test_pending_future_does_not_block(self):
        env, sent, _, _ = self.setup_callback()
        pending = Future()
        env['stair_job']['future'] = pending
        env['collect_stair_labels']()
        self.assertIs(env['stair_job']['future'], pending)
        self.assertFalse(sent)

    def test_classifier_exception_leaves_native_expiry_in_charge(self):
        env, sent, _, warnings = self.setup_callback()
        failed = Future()
        failed.set_exception(ValueError('bad cloud'))
        env['stair_job']['future'] = failed
        env['collect_stair_labels']()
        self.assertFalse(sent)
        self.assertIn('bad cloud', warnings[0])

    def test_source_frames_cannot_be_renamed_as_request(self):
        env, sent, _, _ = self.setup_callback()
        env['stair_job']['frames'] = ('different-map', 'imu')
        env['collect_stair_labels']()
        self.assertFalse(sent)

    def test_terminal_failure_drops_completed_result(self):
        env, sent, _, _ = self.setup_callback()
        env['state']['terminal_failed'] = True
        env['collect_stair_labels']()
        self.assertFalse(sent)

    def test_native_status_is_observation_and_rejects_malformed_or_wrong_child(self):
        import io
        tree = ast.parse(Path(pct_scan_worker.__file__).read_text())
        run = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == 'run')
        callback = next(x for x in run.body if isinstance(x, ast.FunctionDef)
                        and x.name == 'stair_status_callback')
        written = []
        stream = io.StringIO()
        env = dict(feedback_json=lambda message: message, json=json, run_id='this-run',
            state=dict(request=dict(seq=4), child=SimpleNamespace(ack_nonce='owned')),
            stair_status_log=stream, mailbox=Path('not-written'),
            atomic_json=lambda path, status: written.append(status))
        exec(compile(ast.Module(body=[callback], type_ignores=[]), '<status-callback>', 'exec'), env)
        env['stair_status_callback'](None)
        env['stair_status_callback'](dict(run_id='this-run', seq=4, startup_nonce='other'))
        self.assertFalse(written)
        accepted = dict(run_id='this-run', seq=4, startup_nonce='owned', status='accepted', accepted_count=3)
        env['stair_status_callback'](accepted)
        self.assertEqual(written, [accepted])
        self.assertEqual(json.loads(stream.getvalue()), accepted)
        self.assertEqual(env['state']['request'], dict(seq=4))

    def test_slow_model_fit_rechecks_new_current_points_not_old_labels(self):
        from unittest.mock import patch
        old = np.array([[0., 0., 0.]])
        new = np.array([[1., 2., 3.]])
        latest = [(100, old, [0., 0., 0.], ('map', 'imu'))]
        def fit(points, body):
            latest[0] = (200, new, [1., 0., 0.], ('map', 'imu'))
            return {'models': [{'measured': True}]}
        def match(points, body, models):
            self.assertIs(points, new)
            self.assertEqual(body, [1., 0., 0.])
            self.assertEqual(models, [{'measured': True}])
            return {'voxel_indices': np.array([[12, 25, 37]])}
        with patch('embodied_navigation.planning.stair_geometry.detect_stair_voxels', fit), \
             patch('embodied_navigation.planning.stair_geometry.match_stair_voxels', match):
            result = pct_scan_worker.fit_current_stair_geometry(old, [0., 0., 0.], lambda: latest[0])
        self.assertEqual(result['source_stamp_ns'], 200)
        self.assertEqual(result['source_frames'], ('map', 'imu'))
