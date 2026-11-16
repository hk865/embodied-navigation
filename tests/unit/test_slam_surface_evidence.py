import unittest
import numpy as np
from embodied_navigation.localization_mapping.backend.surfaces import SurfaceConfig, model_surfaces, compare_surfaces


def plane(z, start=0):
    x, y = np.meshgrid(np.arange(start+.015, start+.59, .025), np.arange(.015, .59, .025))
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))


class SurfaceEvidenceTests(unittest.TestCase):
    def test_steps_keep_observed_levels_and_provenance(self):
        points = np.vstack([plane(.05), plane(.25, .6), plane(.45, 1.2)])
        keys = [(0, i//576) for i in range(len(points))]
        model = model_surfaces(points, keys)
        heights = sorted({round(p.height_m, 2) for p in model.patches if p.kind == 'horizontal'})
        self.assertEqual(heights, [.05, .25, .45])
        self.assertEqual(sum(p.support_count for p in model.patches)+model.unknown_points, len(points))
        self.assertTrue(all(p.source_keys for p in model.patches))
        for p in model.patches:
            self.assertTrue(np.isin(p.points[:, 2], [.05, .25, .45]).all())

    def test_vertical_is_not_ground_and_slopes_unknown(self):
        wall = plane(.05)[:, [2, 0, 1]]
        model = model_surfaces(wall)
        self.assertTrue(model.patches)
        self.assertTrue(all(p.kind == 'vertical' and p.height_m is None for p in model.patches))
        slope = plane(.05)
        slope[:, 2] = slope[:, 0]
        self.assertEqual(len(model_surfaces(slope).patches), 0)

    def test_z_mismatch_rejected_without_locking_z(self):
        target = model_surfaces(plane(.05))
        query = model_surfaces(plane(.15))
        self.assertFalse(compare_surfaces(query, target).accepted)
        self.assertTrue(compare_surfaces(target, target).accepted)
        self.assertAlmostEqual(query.patches[0].height_m, .15)

    def test_sparse_and_empty_observations_reject(self):
        empty = model_surfaces(np.empty((0, 3)))
        self.assertFalse(compare_surfaces(empty, empty).accepted)
        sparse = model_surfaces(np.array([[0., 0., 0.], [1., 1., 1.]]))
        self.assertEqual(sparse.unknown_points, 2)

    def test_walls_do_not_hide_horizontal_disagreement(self):
        wall = plane(.05)[:, [2, 0, 1]]
        target = model_surfaces(np.vstack([plane(.05), wall]))
        query = model_surfaces(np.vstack([plane(.15), wall]))
        result = compare_surfaces(query, target)
        self.assertFalse(result.accepted)
        self.assertLess(result.coverage_by_kind['horizontal'], .6)

    def test_disconnected_platforms_remain_separate(self):
        points = np.vstack([plane(-.15), plane(-.15, 1.8)])
        model = model_surfaces(points)
        self.assertEqual(len(model.patches), 2)
        self.assertTrue(all(p.height_m < 0 for p in model.patches))
        self.assertTrue(all(np.ptp(p.points[:, 0]) < .6 for p in model.patches))

    def test_gravity_alignment_is_explicit_and_not_body_height(self):
        from embodied_navigation.localization_mapping.backend.keyframes import transform_points
        from scipy.spatial.transform import Rotation
        world = plane(.05)
        T_world_body = np.eye(4)
        T_world_body[:3, :3] = Rotation.from_euler('y', .4).as_matrix()
        T_world_body[:3, 3] = [.4, -.2, 1.3]
        T_body_world = np.linalg.inv(T_world_body)
        body_points = transform_points(T_body_world, world)
        self.assertEqual(len(model_surfaces(body_points).patches), 0)
        aligned = model_surfaces(transform_points(T_world_body, body_points))
        self.assertTrue(aligned.patches)
        self.assertTrue(all(abs(p.height_m-.05) < 1e-10 for p in aligned.patches))

    def test_external_input_validation(self):
        with self.assertRaises(ValueError):
            model_surfaces([[float('nan'), 0, 0]])
        with self.assertRaises(ValueError):
            model_surfaces(plane(.05), ['missing'])
