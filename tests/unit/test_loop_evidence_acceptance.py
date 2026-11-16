"""Independent acceptance cases for measured loop evidence (synthetic observations)."""
import unittest

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.submaps import SubmapConfig, build_submap
from slam_backend_fixtures import pose


def frame(sequence, *, epoch=1, stamp=None, body=None, lidar=None, points=None):
    return Keyframe(
        (epoch, sequence), sequence * 1_000_000_000 if stamp is None else stamp,
        np.eye(4) if body is None else body,
        np.eye(4) if lidar is None else lidar,
        np.array([[.03, .03, .03]]) if points is None else np.asarray(points, dtype=float),
    )


class MeasuredSubmapAcceptance(unittest.TestCase):
    def test_anchor_coordinates_include_body_and_lidar_extrinsics(self):
        old = frame(0, body=pose(x=1), lidar=pose(z=.2), points=[[.02, .03, .04]])
        anchor = frame(1, body=pose(x=2), points=[[.4, .4, .4]])
        result = build_submap([old, anchor], anchor, SubmapConfig())
        self.assertEqual(set(result.source_keys), {old.key, anchor.key})
        expected = np.array([[-.98, .03, .24], [.4, .4, .4]])
        for point in expected:
            self.assertLess(np.linalg.norm(result.points_body - point, axis=1).min(), 1e-10)

    def test_repeated_observations_share_voxel_and_retain_provenance(self):
        frames = [frame(i, points=[[.03, .03, .03], [.04, .04, .04]]) for i in range(4)]
        result = build_submap(frames, frames[-1], SubmapConfig())
        self.assertEqual(len(result.points_body), 1)
        self.assertEqual(len(result.point_sources), 1)
        self.assertEqual(set(result.point_sources[0]), {f.key for f in frames})

    def test_window_stops_at_first_discontinuous_epoch(self):
        frames = [frame(0), frame(1, epoch=2), frame(2)]
        result = build_submap(frames, frames[-1], SubmapConfig())
        self.assertEqual(set(result.source_keys), {frames[-1].key})

    def test_time_gap_stops_even_if_older_pose_is_close(self):
        frames = [frame(0), frame(1), frame(5)]
        result = build_submap(frames, frames[-1], SubmapConfig(max_gap_s=3))
        self.assertEqual(set(result.source_keys), {frames[-1].key})

    def test_translation_height_and_rotation_each_bound_window(self):
        for boundary in [pose(x=2.1), pose(z=.51), pose(roll=.36)]:
            with self.subTest(boundary=boundary.tolist()):
                # An older close frame must not be reached across a failed boundary.
                frames = [frame(0), frame(1, body=boundary), frame(2)]
                result = build_submap(frames, frames[-1], SubmapConfig())
                self.assertEqual(set(result.source_keys), {frames[-1].key})

    def test_frame_count_is_bounded(self):
        frames = [frame(i) for i in range(8)]
        result = build_submap(frames, frames[-1], SubmapConfig(max_frames=3))
        self.assertEqual(set(result.source_keys), {f.key for f in frames[-3:]})

    def test_query_does_not_import_future_observations(self):
        frames = [frame(i) for i in range(4)]
        result = build_submap(frames, frames[1], SubmapConfig())
        self.assertEqual(set(result.source_keys), {f.key for f in frames[:2]})


class EvidenceBypassAcceptance(unittest.TestCase):
    def test_modules_are_disabled_by_default(self):
        config = BackendConfig()
        for name in ['use_measured_submaps', 'use_visual_verification', 'use_surface_verification']:
            self.assertFalse(getattr(config, name))

    def test_default_and_explicit_bypass_preserve_graph_and_correction(self):
        default = SlamBackend(BackendConfig(exclude_recent=20))
        explicit = SlamBackend(BackendConfig(
            exclude_recent=20, use_measured_submaps=False,
            use_visual_verification=False, use_surface_verification=False,
        ))
        rng = np.random.default_rng(317)
        cloud = rng.uniform(-2, 2, (250, 3))
        for i in range(4):
            keyframe = frame(i, body=pose(x=i*.25, z=i*.12, pitch=.1), points=cloud)
            event = default.ingest(keyframe)
            self.assertNotIn('error', event)
            self.assertEqual(event, explicit.ingest(keyframe))
        self.assertEqual(default.graph.keys, explicit.graph.keys)
        for key in default.graph.keys:
            np.testing.assert_array_equal(default.graph.pose(key), explicit.graph.pose(key))
        np.testing.assert_array_equal(default.T_world_odom, explicit.T_world_odom)
        self.assertEqual(default.loop_count, 0)

class BackendEvidenceGateAcceptance(unittest.TestCase):
    def _candidate_backend(self, **options):
        from unittest.mock import Mock
        backend = SlamBackend(BackendConfig(exclude_recent=1, min_loop_gap_s=0, **options))
        old = frame(0, points=[[.1,.1,.1],[.2,.2,.2]])
        backend.ingest(old)
        backend.ingest(frame(1, points=[[.8,.8,.8]]))
        backend.index.query = Mock(return_value=[(old.key, 0., 0.)])
        return backend

    def test_missing_visual_stops_before_icp_and_does_not_add_loop(self):
        from unittest.mock import Mock
        backend = self._candidate_backend(use_visual_verification=True)
        backend._register = Mock(side_effect=AssertionError('missing visual reached ICP'))
        event = backend.ingest(frame(2))
        self.assertNotIn('error', event)
        self.assertFalse(event['loop'])
        backend._register.assert_not_called()
        self.assertEqual(len(backend.graph.edges), 2)

    def test_surface_requires_explicit_gravity_alignment(self):
        with self.assertRaises(ValueError):
            SlamBackend(BackendConfig(use_surface_verification=True, gravity_aligned=False))

    def test_unobservable_surface_rejects_even_successful_icp(self):
        from unittest.mock import Mock
        from embodied_navigation.localization_mapping.backend.registration import IcpResult
        backend = self._candidate_backend(use_surface_verification=True, gravity_aligned=True)
        backend._register = Mock(return_value=IcpResult(np.eye(4), 1., 0., True))
        event = backend.ingest(frame(2))
        self.assertNotIn('error', event)
        backend._register.assert_called_once()
        self.assertFalse(event['loop'])
        self.assertEqual(len(backend.graph.edges), 2)

    def test_measured_source_contains_historical_returns_without_information_inflation(self):
        from unittest.mock import Mock
        from embodied_navigation.localization_mapping.backend.registration import IcpResult
        matrices = []
        for enabled in [False, True]:
            backend = self._candidate_backend(use_measured_submaps=enabled)
            backend._register = Mock(return_value=IcpResult(np.eye(4), 1., 0., True))
            event = backend.ingest(frame(2, points=[[1.,1.,1.]]))
            self.assertNotIn('error', event)
            self.assertTrue(event['loop'])
            source = backend._register.call_args.args[0]
            if enabled:
                self.assertLess(np.linalg.norm(source - [.1,.1,.1],axis=1).min(), 1e-10)
                self.assertLess(np.linalg.norm(source - [1.,1.,1.],axis=1).min(), 1e-10)
            matrices.append(backend.graph.edges[-1][3])
        np.testing.assert_array_equal(matrices[0],matrices[1])


    def test_production_loop_registration_receives_measured_historical_target(self):
        from unittest.mock import Mock
        from embodied_navigation.localization_mapping.backend.registration import IcpResult
        backend = SlamBackend(BackendConfig(use_measured_submaps=True, exclude_recent=1, min_loop_gap_s=0))
        for i, point in enumerate([.1, .8, 1.4]):
            backend.ingest(frame(i, points=[[point, point, point]]))
        backend.index.query = Mock(return_value=[((1, 1), 0., 0.)])
        backend._register = Mock(return_value=IcpResult(np.eye(4), 1., 0., True))
        event = backend.ingest(frame(3, points=[[1.8, 1.8, 1.8]]))
        self.assertNotIn('error', event)
        self.assertTrue(event['loop'])
        source, target, _ = backend._register.call_args.args
        for point in [.1, .8]:
            self.assertLess(np.linalg.norm(target - [point]*3, axis=1).min(), 1e-10)
        self.assertGreater(np.linalg.norm(target - [1.4]*3, axis=1).min(), .1)
        self.assertLess(np.linalg.norm(source - [1.8]*3, axis=1).min(), 1e-10)

    def test_observed_surface_agreement_allows_original_icp_graph_edge(self):
        from unittest.mock import Mock
        from embodied_navigation.localization_mapping.backend.registration import IcpResult
        backend = SlamBackend(BackendConfig(use_surface_verification=True, gravity_aligned=True,
                                             exclude_recent=1, min_loop_gap_s=0, map_voxel_m=.02))
        x, y = np.meshgrid(np.arange(.015, .59, .025), np.arange(.015, .59, .025))
        cloud = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, .05)])
        backend.ingest(frame(0, points=cloud))
        backend.ingest(frame(1, points=cloud))
        backend.index.query = Mock(return_value=[((1, 0), 0., 0.)])
        backend._register = Mock(return_value=IcpResult(np.eye(4), 1., 0., True))
        event = backend.ingest(frame(2, points=cloud))
        self.assertNotIn('error', event)
        self.assertTrue(event['loop'])
        self.assertTrue(event['surfaces']['patches'])
        self.assertEqual(len(backend.graph.edges), 3)


if __name__ == '__main__':
    unittest.main()
