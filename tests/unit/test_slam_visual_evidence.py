"""Synthetic measured image/LiDAR pairs; no ground truth used by matcher."""
import unittest
import json
import tempfile
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe, read_keyframe
from embodied_navigation.localization_mapping.backend.visual import VisualObservation, verify_visual, decode_image


def pair():
    rng = np.random.default_rng(7)
    K = np.array([[100., 0., 100.], [0., 100., 100.], [0., 0., 1.]])
    pixels = np.array([(x, y) for y in range(30, 180, 20) for x in range(30, 180, 20)])
    z = rng.uniform(2, 4, len(pixels))
    points = np.column_stack(((pixels[:, 0]-100)*z/100, (pixels[:, 1]-100)*z/100, z))
    image = np.zeros((200, 200))
    for x, y in pixels:
        image[y-3:y+4, x-3:x+4] = rng.integers(0, 256, (7, 7))
    visual = VisualObservation(image, K, np.eye(4), 0)
    a = Keyframe((0, 0), 0, np.eye(4), np.eye(4), points, visual)
    # Camera and LiDAR body coordinates both change by this measured transform.
    T = np.eye(4)
    T[:3, 3] = [.3, -.2, .1]
    target = replace(a, key=(0, 1), points_lidar=points + T[:3, 3], visual=replace(visual, T_body_camera=T))
    return a, target, T


class VisualTests(unittest.TestCase):
    def test_metric_geometry_and_extrinsic(self):
        a, b, T = pair()
        result = verify_visual(a, b)
        self.assertEqual(result.status, 'verified')
        np.testing.assert_allclose(result.T_target_source, T, atol=1e-10)
        self.assertGreater(result.inliers, 20)

    def test_missing_stale_and_untextured(self):
        a, b, _ = pair()
        self.assertEqual(verify_visual(replace(a, visual=None), b).status, 'missing_observation')
        self.assertEqual(verify_visual(replace(a, stamp_ns=100_000_000), b).status, 'time_mismatch')
        a = replace(a, visual=replace(a.visual, gray=np.zeros_like(a.visual.gray)))
        self.assertEqual(verify_visual(a, b).status, 'insufficient_texture')

    def test_same_appearance_wrong_geometry(self):
        a, b, _ = pair()
        # Same projected pixels and image, independently wrong depths.
        rng = np.random.default_rng(9)
        points = a.points_lidar * rng.uniform(.5, 2., (len(a.points_lidar), 1))
        b = replace(a, points_lidar=points)
        self.assertNotEqual(verify_visual(a, b).status, 'verified')

    def test_rgb_stride(self):
        msg = SimpleNamespace(encoding='rgb8', width=1, height=2, step=4, data=bytes([255, 0, 0, 0, 0, 255, 0, 0]))
        np.testing.assert_allclose(decode_image(msg), [[255*.299], [255*.587]])
        msg.encoding = '16UC1'
        with self.assertRaises(ValueError):
            decode_image(msg)

    def test_invalid_calibration(self):
        a, _, _ = pair()
        with self.assertRaises(ValueError):
            replace(a.visual, K=np.zeros((3, 3)))
        T = np.eye(4)
        T[0, 0] = 2
        with self.assertRaises(ValueError):
            replace(a.visual, T_body_camera=T)

    def test_spool_visual_observation_roundtrip(self):
        a, _, _ = pair()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            row = dict(schema='livo_keyframe/v1', epoch=0, sequence=0, stamp_ns=0,
                       T_odom_body=np.eye(4).tolist(), T_body_lidar=np.eye(4).tolist())
            (path/'metadata.json').write_text(json.dumps(row))
            (path/'points_lidar.pcd').write_text('DATA ascii\n0 0 1\n')
            self.assertIsNone(read_keyframe(path).visual)
            np.savez(path/'visual.npz', gray=a.visual.gray, K=a.visual.K,
                     T_body_camera=a.visual.T_body_camera, stamp_ns=0)
            restored = read_keyframe(path)
            np.testing.assert_array_equal(restored.visual.gray, a.visual.gray)
            np.savez(path/'visual.npz', gray=a.visual.gray)
            with self.assertRaisesRegex(ValueError, 'invalid visual observation'):
                read_keyframe(path)
