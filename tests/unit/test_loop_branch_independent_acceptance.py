"""Independent production-chain checks; only Scan Context nomination is mocked."""
import unittest
from dataclasses import replace
from unittest.mock import Mock

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.visual import VisualObservation
from loop_branch_acceptance_fixtures import Trace, room, frame


class SurfaceProductionAcceptance(unittest.TestCase):
    def run_loop(self, voxel_m, gate, floor_height=.05):
        trace = Trace()
        backend = SlamBackend(BackendConfig(map_voxel_m=voxel_m,
            use_surface_verification=gate, gravity_aligned=gate,
            exclude_recent=1, min_loop_gap_s=0), trace=trace)
        backend.index.query = Mock(return_value=[])
        backend.ingest(frame(0, room()))
        backend.ingest(frame(1, room()))
        backend.index.query.return_value = [((1, 0), 0., 0.)]
        event = backend.ingest(frame(2, room(floor_height)))
        self.assertNotIn('error', event)
        return backend, event, trace.rows

    def test_default_map_density_is_insufficient_for_surface_gate(self):
        backend, event, rows = self.run_loop(.2, True)
        self.assertFalse(event['loop'])
        icp = [row for kind, row in rows if kind == 'icp'][-1]
        self.assertTrue(icp['fine']['converged'])
        surface = [row for kind, row in rows if kind == 'surface_verification'][-1]
        self.assertEqual(surface['reason'], 'insufficient_surface_observations')
        self.assertEqual(len(backend.graph.edges), 2)

    def test_dense_map_gate_accepts_real_icp_without_extra_graph_information(self):
        control, control_event, _ = self.run_loop(.05, False)
        backend, event, rows = self.run_loop(.05, True)
        self.assertTrue(control_event['loop'])
        self.assertTrue(event['loop'])
        self.assertEqual(event['loop_key'], (1, 0))
        self.assertEqual(backend.loop_count, 1)
        self.assertEqual(len(backend.graph.edges), 3)
        surface = [row for kind, row in rows if kind == 'surface_verification'][-1]
        self.assertEqual(surface['reason'], 'consistent')
        np.testing.assert_allclose(backend.graph.edges[-1][2], control.graph.edges[-1][2])
        np.testing.assert_array_equal(backend.graph.edges[-1][3], control.graph.edges[-1][3])

    def test_dense_map_gate_rejects_inconsistent_floor_that_real_icp_accepts(self):
        control, control_event, _ = self.run_loop(.05, False, .2)
        backend, event, rows = self.run_loop(.05, True, .2)
        self.assertTrue(control_event['loop'])
        self.assertFalse(event['loop'])
        icp = [row for kind, row in rows if kind == 'icp'][-1]
        self.assertTrue(icp['fine']['converged'])
        self.assertLess(icp['fine']['rmse_m'], backend.config.max_rmse_m)
        surface = [row for kind, row in rows if kind == 'surface_verification'][-1]
        self.assertEqual(surface['reason'], 'surface_mismatch')
        self.assertEqual(len(backend.graph.edges), 2)
        self.assertEqual(len(control.graph.edges), 3)


class VisualDefaultDensityAcceptance(unittest.TestCase):
    def test_default_config_accepts_real_visual_geometry_icp_and_single_loop(self):
        # Synthetic pixels/returns share actual projections; no metric pose is mocked.
        rng = np.random.default_rng(3003)
        gray = rng.integers(0, 256, (160, 160), dtype=np.uint8)
        cloud = rng.uniform([-1.1, -1.1, 2.], [1.1, 1.1, 4.4], (2500, 3))
        intrinsic = np.array([[100., 0., 80.], [0., 100., 80.], [0., 0., 1.]])
        trace = Trace()
        backend = SlamBackend(BackendConfig(use_visual_retrieval=True), trace=trace)
        backend.index.query = Mock(return_value=[])
        for i in range(12):
            observation = frame(i, cloud)
            if i in (0, 11):
                observation = replace(observation,
                    visual=VisualObservation(gray, intrinsic, np.eye(4), observation.stamp_ns))
            event = backend.ingest(observation)
            self.assertNotIn('error', event)
            self.assertEqual(event['loop'], i == 11)
        self.assertEqual(backend.loop_count, 1)
        self.assertEqual(len(backend.graph.edges), 12)
        visual = [row for kind, row in trace.rows if kind == 'visual_verification'][-1]
        self.assertEqual(visual['status'], 'verified')
        icp = [row for kind, row in trace.rows if kind == 'icp'][-1]
        self.assertTrue(icp['fine']['converged'])
        self.assertLess(icp['fine']['rmse_m'], backend.config.max_rmse_m)


if __name__ == '__main__':
    unittest.main()
