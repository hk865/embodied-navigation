"""Independent stair-result transport tests through the real worker callbacks."""
from concurrent.futures import Future
import json
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from scan_worker_acceptance_fixtures import ImmediatePool, WorkerRig


class StairWorkerAcceptance(unittest.TestCase):
    def exercise(self, scenario):
        jobs = []
        original = ImmediatePool.submit
        def submit(pool, function, *args, **kwargs):
            if function.__name__ != 'fit_current_stair_geometry':
                return original(pool, function, *args, **kwargs)
            future = Future()
            jobs.append((future, args))
            return future  # Deliberately unfinished; pair/poll must not await it.
        with tempfile.TemporaryDirectory() as directory:
            rig = WorkerRig(directory)
            with patch.object(ImmediatePool, 'submit', submit):
                rig.run_scenario(lambda r: scenario(r, jobs))

    @staticmethod
    def complete(job, count=1, source_stamp=10_600_000_000):
        job[0].set_result(dict(voxel_indices=np.tile([1, 2, 3], (count, 1)),
                               models=[], observed_point_count=2,
                               source_stamp_ns=source_stamp,
                               source_frames=('camera_init', 'body')))

    @staticmethod
    def labels(rig):
        return [json.loads(m.data) for m in rig.node.publisher('stair_labels').messages]

    def test_unfinished_classification_does_not_block_pair_and_keeps_one_job(self):
        def scenario(rig, jobs):
            rig.prepare()
            self.assertEqual(len(jobs), 1)
            rig.observe(10.7)
            rig.observe(10.8)
            self.assertEqual(len(jobs), 1)
            self.assertGreaterEqual(len(rig.node.publisher('cloud').messages), 3)
            self.assertEqual(self.labels(rig), [])
        self.exercise(scenario)

    def test_exact_source_timestamp_preserved(self):
        def scenario(rig, jobs):
            rig.prepare()
            self.complete(jobs[0])
            rig.poll()
            messages = self.labels(rig)
            self.assertEqual(len(messages), 1)
            self.assertEqual(messages[0]['source_stamp_ns'], 10_600_000_000)
            self.assertEqual(messages[0]['voxel_indices'], [[1, 2, 3]])
            self.assertEqual(messages[0]['frame_id'], 'camera_init')
            self.assertEqual(messages[0]['model_source_interval_ns'], [10_200_000_000, 10_600_000_000])
            self.assertEqual(jobs[0][1][0].shape, (10, 3))
            self.assertEqual(jobs[0][1][2]()[1].shape, (2, 3))
        self.exercise(scenario)

    def test_source_more_than_point_three_seconds_old_is_discarded(self):
        def scenario(rig, jobs):
            rig.prepare()
            rig.observe(11.0)
            self.complete(jobs[0])
            rig.poll()
            self.assertEqual(self.labels(rig), [])
        self.exercise(scenario)

    def test_oversized_classification_publishes_empty_not_truncated_mask(self):
        def scenario(rig, jobs):
            rig.prepare()
            self.complete(jobs[0], 4097)
            rig.poll()
            self.assertEqual(self.labels(rig)[0]['voxel_indices'], [])
        self.exercise(scenario)

    def test_changed_request_does_not_publish_old_job(self):
        def scenario(rig, jobs):
            rig.prepare()
            request_path = rig.mailbox / 'planner_request.json'
            request = json.loads(request_path.read_text())
            request['seq'] += 1
            request_path.write_text(json.dumps(request))
            self.complete(jobs[0])
            rig.poll()
            self.assertEqual(self.labels(rig), [])
        self.exercise(scenario)

    def test_new_actual_snapshot_stamp_not_old_submission_stamp_is_published(self):
        def scenario(rig, jobs):
            rig.prepare()
            rig.observe(11.0, (.1, .2, .4))
            snapshot = jobs[0][1][2]()
            self.assertEqual(snapshot[0], 11_000_000_000)
            self.assertEqual(snapshot[2], [.1, .2, .4])
            self.assertEqual(snapshot[3], ('camera_init', 'body'))
            self.complete(jobs[0], source_stamp=snapshot[0])
            rig.poll()
            message = self.labels(rig)[0]
            self.assertEqual(message['source_stamp_ns'], 11_000_000_000)
            self.assertEqual(message['model_source_interval_ns'], [10_200_000_000, 10_600_000_000])
        self.exercise(scenario)
