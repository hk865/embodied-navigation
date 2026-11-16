"""Run the image-only proposal through measured geometry, real ICP and graph."""
import unittest
from dataclasses import replace
from unittest.mock import Mock

from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from visual_retrieval_fixtures import dense_visual_frame


class VisualRetrievalFlow(unittest.TestCase):
    def test_image_only_proposal_passes_real_icp_and_adds_one_loop(self):
        backend = SlamBackend(BackendConfig(use_visual_retrieval=True, exclude_recent=1,
                                             min_loop_gap_s=0, map_voxel_m=.001,
                                             icp_voxel_m=.1))
        # Disable only the SC proposal path: visual matching/3D verification,
        # ICP, motion gate and graph admission remain the production functions.
        backend.index.query = Mock(return_value=[])
        backend.ingest(dense_visual_frame(0))
        backend.ingest(replace(dense_visual_frame(1), visual=None))
        event = backend.ingest(dense_visual_frame(2))
        self.assertNotIn('error', event)
        self.assertTrue(event['loop'])
        self.assertEqual(event['loop_key'], (1, 0))
        self.assertEqual(backend.loop_count, 1)
        self.assertEqual(len(backend.graph.edges), 3)
