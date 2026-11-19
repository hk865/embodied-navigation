import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from embodied_navigation.localization_mapping.backend.trace import BackendTrace, graph_snapshot
from embodied_navigation.localization_mapping.backend.backend import BackendConfig
from embodied_navigation.localization_mapping.backend.pose_graph import PoseGraph, invert, se3_log

class TraceStorageAcceptance(unittest.TestCase):
    def test_real_writer_and_graph_snapshot_direction_weight_and_copy(self):
        graph = PoseGraph(cauchy_scale=3.)
        A = np.eye(4)
        A[:3,:3] = Rotation.from_euler('xyz', [.2,-.3,.7]).as_matrix()
        A[:3,3] = [1,2,3]
        B = np.eye(4)
        B[:3,:3] = Rotation.from_euler('xyz', [-.1,.4,-.8]).as_matrix()
        B[:3,3] = [-2,1,.5]
        Z = invert(A) @ B
        Z[:3,3] += [.1,.2,-.3]
        info = np.diag([2,3,4,5,6,7])
        graph.add_node((1,1), A)
        graph.add_node((1,2), B)
        graph.add_edge((1,1),(1,2),Z,info,robust=True)
        snap = graph_snapshot(graph)
        edge = snap['edges'][0]
        self.assertEqual(edge['a'], (1,1))
        self.assertEqual(edge['b'], (1,2))
        np.testing.assert_allclose(edge['T_a_b'], Z, atol=1e-12)
        np.testing.assert_array_equal(edge['information'], info)
        residual = se3_log(invert(Z) @ invert(A) @ B)
        np.testing.assert_allclose(edge['residual'], residual, atol=1e-12)
        self.assertAlmostEqual(edge['effective_weight'], 1/(1+residual@info@residual/9))
        graph.poses[1][:3,3] += 100
        np.testing.assert_array_equal(snap['poses'][1], B)
        with tempfile.TemporaryDirectory() as tmp:
            trace = BackendTrace(Path(tmp)/'trace', BackendConfig())
            trace.emit('graph_before', graph=snap, stamp_ns=1234567890123456789)
            trace.emit('icp', fine={'rmse_m':float('inf'), 'converged':False})
            summary = trace.close()
            rows = [json.loads(line, parse_constant=lambda v: self.fail('nonstandard JSON '+v))
                    for line in (Path(tmp)/'trace/events.jsonl').read_text().splitlines()]
            self.assertEqual([r['seq'] for r in rows], [1,2,3])
            self.assertEqual(rows[1]['stamp_ns'], 1234567890123456789)
            self.assertEqual(rows[2]['fine']['rmse_m'], 'inf')
            self.assertEqual(summary['submitted'], 3)
            self.assertEqual(summary['written'], 3)
            self.assertIsNone(summary['error'])
            np.testing.assert_allclose(rows[1]['graph']['edges'][0]['T_a_b'], Z)

if __name__ == '__main__':
    unittest.main()
