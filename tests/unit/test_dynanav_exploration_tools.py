"""Offline contracts for the reusable project entrypoint and snapshot review.

No ROS discovery, simulator launch, request submission, or private data dependency.
"""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

REPO = Path(__file__).resolve().parents[2]
TOOLS = REPO / 'scripts/dynanav_exploration'


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


class ExplorationToolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cache = tempfile.TemporaryDirectory(prefix='dynanav-tools-tests-')
        cls.environment = mock.patch.dict(os.environ, MPLCONFIGDIR=cls.cache.name)
        cls.environment.start()
        cls.entry = module('dynanav_project_entry', REPO / 'scripts/dynanav_exploration.py')
        cls.core = module('dynanav_project_review', TOOLS / 'review_generation.py')

    @classmethod
    def tearDownClass(cls):
        cls.environment.stop()
        cls.cache.cleanup()

    def binding(self, root, output='outputs'):
        return {'run_root': str(root / 'run'), 'evidence_root': str(root / 'evidence'),
                'allowed_input_roots': [str(root / 'run'), str(root / 'evidence')],
                'output_root': str(root / output)}

    def test_dispatch_uses_only_checkout_code(self):
        for name in self.entry.COMMANDS:
            command = self.entry.command_line([name, '--help'])
            self.assertEqual(command[:2], [sys.executable, '-B'])
            self.assertTrue(Path(command[2]).is_relative_to(REPO))
            self.assertNotIn('archive_20261111', command[2])
            self.assertNotIn('Documents', command[2])
            self.assertTrue(Path(command[2]).is_file())

    def test_sop_defaults_to_current_checkout(self):
        command = self.entry.command_line(['sop', '--run-dir', 'run', '--evidence-dir', 'evidence', 'status'])
        self.assertEqual(command[3:5], ['--repo', str(REPO)])
        explicit = self.entry.command_line(['sop', '--repo=/explicit/repo', '--help'])
        self.assertEqual(explicit.count('--repo'), 0)
        self.assertIn('--repo=/explicit/repo', explicit)

    def test_unknown_command_is_not_executed(self):
        with mock.patch.object(self.entry.subprocess, 'call') as call:
            self.assertEqual(self.entry.main(['shell', 'rm']), 2)
        call.assert_not_called()

    def test_cpu_help_is_safe_outside_repository(self):
        # collect imports ROS message types, so only CPU commands belong here.
        with tempfile.TemporaryDirectory() as temporary:
            for name in ('sop', 'view', 'review', 'roi'):
                with self.subTest(command=name):
                    result = subprocess.run([sys.executable, '-B', str(REPO / 'scripts/dynanav_exploration.py'), name, '--help'],
                                            cwd=temporary, text=True, capture_output=True, timeout=30)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertIn('usage:', result.stdout)
            self.assertEqual(list(Path(temporary).iterdir()), [])

    def test_output_requires_new_external_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binding = self.binding(root)
            output = self.core.output_directory(binding, 's01')
            self.assertEqual(output, root / 'outputs/s01')
            marker = output / 'existing.json'
            marker.write_text('preserve')
            with self.assertRaises(FileExistsError):
                self.core.output_directory(binding, 's01')
            self.assertEqual(marker.read_text(), 'preserve')

    def test_output_cannot_overlap_input_or_checkout(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binding = self.binding(root)
            for target in (root / 'run', root / 'run/review', root, REPO / 'scripts/review'):
                with self.subTest(target=target), self.assertRaises(ValueError):
                    self.core.output_directory(dict(binding, output_root=str(target)), 's01')

    def test_output_symlink_to_input_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'run').mkdir()
            (root / 'alias').symlink_to(root / 'run', target_is_directory=True)
            with self.assertRaises(ValueError):
                self.core.output_directory(self.binding(root, 'alias'), 's01')

    def test_snapshot_and_roi_labels_cannot_escape_output(self):
        for value in ('../s01', '/tmp/s01', '.', '..', 's01/child', ''):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.core.validate_snapshot(value)
        self.core.validate_snapshot('s01-inspect_2')

    def test_both_snapshot_cloud_contracts_supported(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / 'evidence'
            source.mkdir()
            binding = self.binding(root)
            for name in ('measured_cloud.npz', 'measured_partial_cloud.npz'):
                path = source / name
                path.write_bytes(b'fixture')
                self.assertEqual(self.core.snapshot_cloud(source, binding), path)
                path.unlink()
            with self.assertRaises(ValueError):
                self.core.snapshot_cloud(source, binding)
            for name in ('measured_cloud.npz', 'measured_partial_cloud.npz'):
                (source / name).touch()
            with self.assertRaises(ValueError):
                self.core.snapshot_cloud(source, binding)
            self.assertEqual(self.core.snapshot_cloud(source, binding, collector=True).name,
                             'measured_partial_cloud.npz')

    def test_grid_contracts_both_discovered(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            expected = ['height_chart_000.json', 'level_+0.01_operator_map_partial.json']
            for name in [*expected, 'observation.json', 'map.json']:
                (root / name).write_text('{}')
            self.assertEqual([p.name for p in self.core.grid_manifests(root)], expected)

    def test_trace_preserves_measured_sources_and_gaps(self):
        base = dict(pose_source='livo_post_vio', no_gt_pose_used=True,
                    frame_id='dynanav_lio_map', child_frame_id='go2_imu',
                    odom_stamp_s=1, aligned_position_xyz_m=[0, 0, 0], aligned_yaw_rad=0)
        rows = [base, dict(base, odom_stamp_s=1.2, aligned_position_xyz_m=[1, 0, 0]),
                dict(base, odom_stamp_s=3, aligned_position_xyz_m=[99, 0, 0]),
                dict(base, odom_stamp_s=1.1, pose_source='truth')]
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'navigation.jsonl'
            path.write_text('\n'.join(json.dumps(row) for row in rows))
            trace = self.core.load_trace(path, dict(pose_source='livo_post_vio',
                map_frame='dynanav_lio_map', pose_child_frame='go2_imu'), cutoff=2)
        np.testing.assert_array_equal(trace[:, 0], [1, 1.2])
        self.assertEqual(self.core.measured_length(trace), 1)

    def test_receipt_audit_does_not_include_future_reach(self):
        accepted = dict(seq=1, event='waypoint_accepted', goal_xyz_m=[1, 0, 0])
        reached = dict(seq=1, event='waypoint_reached')
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'receipts.jsonl'
            path.write_text('\n'.join(json.dumps(row) for row in (accepted, reached)))
            audit = self.core.receipts(path, {'receipts': [accepted]})
        self.assertEqual(audit['goto_accepted'], 1)
        self.assertEqual(audit['goto_reached'], 0)
        self.assertEqual(audit['active_seq'], 1)


if __name__ == '__main__':
    unittest.main()
