"""Independent visual evidence contract; no rendered/GT depth is supplied."""
import unittest
from dataclasses import replace
import numpy as np
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.visual import VisualObservation, verify_visual


class VisualEvidenceAcceptance(unittest.TestCase):
    def make_pair(self):
        rng = np.random.default_rng(104)
        gray = rng.integers(0, 256, (160, 160), dtype=np.uint8)
        intrinsic = np.array([[100., 0, 80], [0, 100, 80], [0, 0, 1]])
        pixels = np.array([(x,y) for y in [25,50,75,100,125] for x in [25,50,75,100,125]])
        depth = rng.uniform(2, 4, len(pixels))
        xyz = np.column_stack(((pixels[:,0]-80)*depth/100, (pixels[:,1]-80)*depth/100, depth))
        ext = np.eye(4); ext[0,3] = .2
        source = Keyframe((1,0), 1_000_000_000, np.eye(4), np.eye(4), xyz + [.2,0,0],
                          visual=VisualObservation(gray, intrinsic, ext, 1_000_000_000))
        target = Keyframe((1,1), 2_000_000_000, np.eye(4), np.eye(4), xyz,
                          visual=VisualObservation(gray.copy(), intrinsic, np.eye(4), 2_000_000_000))
        return source, target

    def test_map_roundtrip_preserves_visual_measurement_and_calibration(self):
        import tempfile
        from pathlib import Path
        from embodied_navigation.localization_mapping.backend.map_store import MapStore
        source, _ = self.make_pair()
        store = MapStore()
        store.add(source, source.T_odom_body, np.zeros((20,60)))
        with tempfile.TemporaryDirectory() as directory:
            store.save(Path(directory))
            loaded = MapStore.load(Path(directory)).keyframes[source.key]
        self.assertEqual(loaded.visual.stamp_ns,source.visual.stamp_ns)
        for name in ['gray','K','T_body_camera']:
            np.testing.assert_array_equal(getattr(loaded.visual,name),getattr(source.visual,name))

    def test_missing_pixels_cannot_produce_a_pose(self):
        source, target = self.make_pair()
        missing = Keyframe(source.key, source.stamp_ns, source.T_odom_body,
                           source.T_body_lidar, source.points_lidar)
        result = verify_visual(missing, target)
        self.assertEqual(result.status, 'missing_observation')
        self.assertIsNone(result.T_target_source)

    def test_stale_image_is_rejected(self):
        source, target = self.make_pair()
        source = replace(source, visual=replace(source.visual, stamp_ns=900_000_000))
        result = verify_visual(source, target)
        self.assertEqual(result.status, 'time_mismatch')
        self.assertIsNone(result.T_target_source)

    def test_blank_images_are_insufficient(self):
        source, target = self.make_pair()
        source.visual.gray[:] = 80
        target.visual.gray[:] = 80
        result = verify_visual(source, target)
        self.assertEqual(result.status, 'insufficient_texture')
        self.assertIsNone(result.T_target_source)

    def test_different_images_do_not_become_se3_constraints(self):
        source, target = self.make_pair()
        target.visual.gray[:] = np.random.default_rng(981).integers(0,256,target.visual.gray.shape,dtype=np.uint8)
        result = verify_visual(source,target)
        self.assertNotEqual(result.status, 'verified')
        self.assertIsNone(result.T_target_source)

    def test_real_lidar_correspondences_recover_body_frame_transform(self):
        source, target = self.make_pair()
        result = verify_visual(source, target)
        self.assertEqual(result.status, 'verified')
        expected = np.eye(4); expected[0,3] = -.2
        np.testing.assert_allclose(result.T_target_source, expected, atol=1e-9)
        self.assertGreaterEqual(result.inliers, 8)


if __name__ == '__main__':
    unittest.main()
