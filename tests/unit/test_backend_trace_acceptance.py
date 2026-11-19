"""Independent tests against agreed trace constructor; do not inspect implementation."""
import copy
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path.cwd() / 'tests' / 'unit'))
import slam_backend_fixtures as F
from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe

class MemoryTrace:
    def __init__(self):
        self.rows = []
    def emit(self, event_name, **fields):
        self.rows.append((event_name, copy.deepcopy(fields)))

class BrokenTrace:
    def emit(self, event_name, **fields):
        raise OSError('acceptance simulated full disk')

def snapshot(backend):
    return dict(C=backend.T_world_odom, count=backend.loop_count,
                keys=backend.graph.keys, poses=backend.graph.poses,
                edges=backend.graph.edges, rejection=backend.admission.rejections,
                map_poses=backend.map.poses, tracking=backend._tracking,
                failures=backend._failures)

def assert_identical(test, a, b):
    if isinstance(a, np.ndarray):
        np.testing.assert_array_equal(a, b)
    elif isinstance(a, dict):
        test.assertEqual(a.keys(), b.keys())
        for k in a: assert_identical(test, a[k], b[k])
    elif isinstance(a, (list, tuple)):
        test.assertEqual(len(a), len(b))
        for x, y in zip(a, b): assert_identical(test, x, y)
    else:
        test.assertEqual(a, b)

class BackendTraceAcceptance(unittest.TestCase):
    def test_trace_and_failing_sink_preserve_all_decisions(self):
        trace = MemoryTrace()
        backends = [SlamBackend(BackendConfig(), trace=t) for t in (None, trace, BrokenTrace())]
        trajectory = F.closed_loop_with_drift(drift_m=1.5)
        rows = F.keyframe_stream(trajectory['scene'], trajectory['odom'])
        saw_loop = False
        for row in rows:
            outputs = [b.ingest(Keyframe(**copy.deepcopy(row))) for b in backends]
            saw_loop |= bool(outputs[0].get('loop'))
            assert_identical(self, outputs[0], outputs[1])
            assert_identical(self, outputs[0], outputs[2])
            assert_identical(self, snapshot(backends[0]), snapshot(backends[1]))
            assert_identical(self, snapshot(backends[0]), snapshot(backends[2]))
        self.assertTrue(saw_loop, 'fixture must exercise optimization and correction')
        names = [name for name, _ in trace.rows]
        for required in ('search_candidates', 'graph_before', 'graph_after', 'loop_decision', 'correction'):
            self.assertIn(required, names)
        self.assertTrue(backends[2].trace_error, 'recording failure must be independently visible')
        self.assertIn('full disk', str(backends[2].trace_error))

if __name__ == '__main__':
    unittest.main()
