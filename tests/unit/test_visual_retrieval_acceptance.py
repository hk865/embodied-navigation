"""Independent image-retrieval acceptance, authored without reading implementation."""
import unittest
from dataclasses import replace
from unittest.mock import Mock

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import BackendConfig,SlamBackend
from embodied_navigation.localization_mapping.backend.registration import IcpResult
from embodied_navigation.localization_mapping.backend.visual import verify_visual
from embodied_navigation.localization_mapping.backend.visual_retrieval import VisualIndex,VisualRetrievalConfig,image_descriptor
from visual_retrieval_fixtures import visual_frame


class VisualRetrievalContractAcceptance(unittest.TestCase):
    def test_descriptor_depends_on_image_not_lidar(self):
        frame = visual_frame()
        moved_cloud = replace(frame,points_lidar=np.array([[100.,100.,100.]]))
        np.testing.assert_array_equal(image_descriptor(frame),image_descriptor(moved_cloud))

    def test_missing_stale_and_blank_pixels_are_unavailable(self):
        frame = visual_frame()
        cases = [replace(frame,visual=None),
                 replace(frame,visual=replace(frame.visual,stamp_ns=frame.stamp_ns-100_000_000)),
                 replace(frame,visual=replace(frame.visual,gray=np.zeros_like(frame.visual.gray)))]
        for candidate in cases:
            with self.subTest(visual=candidate.visual):
                self.assertIsNone(image_descriptor(candidate))

    def test_index_finds_identical_image_and_respects_exclusions(self):
        index = VisualIndex()
        old = visual_frame(0)
        index.add(old)
        query = visual_frame(5)
        result = index.query(query)
        self.assertEqual(result[0][0],old.key)
        self.assertAlmostEqual(result[0][1],0.,places=10)
        self.assertEqual(index.query(query,exclude=lambda key:key==old.key),[])

    def test_index_capacity_is_bounded_by_recent_insertions(self):
        index = VisualIndex(VisualRetrievalConfig(max_entries=2))
        for i in range(3):
            index.add(visual_frame(i))
        self.assertNotIn((1,0),[key for key,_ in index.query(visual_frame(4))])


class VisualRetrievalBackendAcceptance(unittest.TestCase):
    def backend(self, *, enabled=True, epoch=1, **options):
        config = BackendConfig(use_visual_retrieval=enabled,exclude_recent=1,min_loop_gap_s=0,**options)
        backend = SlamBackend(config)
        backend.ingest(visual_frame(0,epoch=epoch))
        backend.ingest(visual_frame(1,epoch=epoch,visual=False))
        backend.index.query = Mock(return_value=[])
        backend._register = Mock(return_value=IcpResult(np.eye(4),1.,0.,True))
        return backend

    def test_fixture_has_measured_geometric_support(self):
        self.assertEqual(verify_visual(visual_frame(2),visual_frame(0)).status,'verified')

    def test_visual_retrieves_with_no_scan_context_candidates(self):
        backend = self.backend()
        event = backend.ingest(visual_frame(2))
        self.assertNotIn('error',event)
        self.assertTrue(event['loop'])
        backend._register.assert_called_once()
        self.assertEqual(len(backend.graph.edges),3)

    def test_high_scan_context_distance_cannot_hide_visual_candidate(self):
        backend = self.backend()
        backend.index.query.return_value = [((1,0),.99,0.)]
        event = backend.ingest(visual_frame(2))
        self.assertNotIn('error',event)
        self.assertTrue(event['loop'])
        backend._register.assert_called_once()

    def test_visual_candidate_has_independent_registration_budget(self):
        backend = self.backend(icp_candidates=0)
        event = backend.ingest(visual_frame(2))
        self.assertNotIn('error',event)
        self.assertTrue(event['loop'])
        backend._register.assert_called_once()

    def test_overlapping_scan_and_visual_candidate_is_registered_once(self):
        backend = self.backend()
        backend.index.query.return_value = [((1,0),0.,0.)]
        event = backend.ingest(visual_frame(2))
        self.assertNotIn('error',event)
        self.assertTrue(event['loop'])
        backend._register.assert_called_once()

    def test_recent_candidate_is_excluded_from_visual_retrieval(self):
        backend = SlamBackend(BackendConfig(use_visual_retrieval=True,exclude_recent=1,min_loop_gap_s=0))
        backend.ingest(visual_frame(0))
        backend.index.query = Mock(return_value=[])
        backend._register = Mock(side_effect=AssertionError('recent candidate reached ICP'))
        event = backend.ingest(visual_frame(1))
        self.assertNotIn('error',event)
        self.assertFalse(event['loop'])
        backend._register.assert_not_called()

    def test_mapping_excludes_other_epoch_candidates(self):
        backend = self.backend(epoch=2)
        event = backend.ingest(visual_frame(2,epoch=3))
        self.assertNotIn('error',event)
        self.assertFalse(event['loop'])
        backend._register.assert_not_called()

    def test_missing_query_image_does_not_nominate_visual_loop(self):
        backend = self.backend()
        event = backend.ingest(visual_frame(2,visual=False))
        self.assertNotIn('error',event)
        self.assertFalse(event['loop'])
        backend._register.assert_not_called()

    def test_visual_appearance_without_lidar_geometry_cannot_add_edge(self):
        backend = self.backend()
        query = replace(visual_frame(2),points_lidar=np.array([[0.,0.,2.]]))
        event = backend.ingest(query)
        self.assertNotIn('error',event)
        self.assertFalse(event['loop'])
        backend._register.assert_not_called()
        self.assertEqual(len(backend.graph.edges),2)

    def test_lidar_candidate_without_pixels_retains_original_icp_path(self):
        backend = self.backend()
        backend.index.query.return_value = [((1,0),0.,0.)]
        event = backend.ingest(visual_frame(2,visual=False))
        self.assertNotIn('error',event)
        self.assertTrue(event['loop'])
        backend._register.assert_called_once()

    def test_visual_geometry_still_requires_icp_verification(self):
        backend = self.backend()
        backend._register.return_value = None
        event = backend.ingest(visual_frame(2))
        self.assertNotIn('error',event)
        self.assertFalse(event['loop'])
        backend._register.assert_called_once()
        self.assertEqual(len(backend.graph.edges),2)

    def test_prior_relocalization_can_retrieve_across_epoch(self):
        from embodied_navigation.localization_mapping.backend.map_store import MapStore
        prior = MapStore()
        old = visual_frame(0,epoch=2)
        prior.add(old,np.eye(4),np.zeros((20,60)))
        backend = SlamBackend(BackendConfig(use_visual_retrieval=True),prior_map=prior)
        backend.index.query = Mock(return_value=[])
        backend._register = Mock(return_value=IcpResult(np.eye(4),1.,0.,True))
        event = backend.ingest(visual_frame(5,epoch=3))
        self.assertNotIn('error',event)
        self.assertTrue(event['relocalized'])
        backend._register.assert_called_once()

    def test_visual_match_still_obeys_motion_consistency_gate(self):
        backend = self.backend()
        proposed = np.eye(4); proposed[2,3] = 20.
        backend._register.return_value = IcpResult(proposed,1.,0.,True)
        event = backend.ingest(visual_frame(2))
        self.assertNotIn('error',event)
        self.assertFalse(event['loop'])
        self.assertEqual(len(backend.graph.edges),2)

    def test_default_bypass_does_not_nominate_visual_candidate(self):
        self.assertFalse(BackendConfig().use_visual_retrieval)
        backend = self.backend(enabled=False)
        event = backend.ingest(visual_frame(2))
        self.assertNotIn('error',event)
        self.assertFalse(event['loop'])
        backend._register.assert_not_called()


if __name__=='__main__':
    unittest.main()
