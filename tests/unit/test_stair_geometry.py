import unittest
import numpy as np
from embodied_navigation.planning.stair_geometry import detect_stair_voxels, match_stair_voxels


def staircase(risers=True):
    chunks = []
    for i in range(3):
        x, y = np.meshgrid(np.arange(i * .4 + .02, (i + 1) * .4 - .01, .025), np.arange(-.6, .601, .025))
        chunks.append(np.column_stack([x.ravel(), y.ravel(), np.full(x.size, i * .15)]))
        if risers and i:
            z, y = np.meshgrid(np.arange((i - 1) * .15 + .01, i * .15, .02), np.arange(-.6, .601, .025))
            chunks.append(np.column_stack([np.full(z.size, i * .4), y.ravel(), z.ravel()]))
    return np.concatenate(chunks)


class StairGeometryTest(unittest.TestCase):
    def test_stair_requires_observed_risers(self):
        good = detect_stair_voxels(staircase(), [0, 0, .3])
        self.assertGreater(len(good['voxel_indices']), 50)
        self.assertTrue(good['models'])
        self.assertEqual(len(detect_stair_voxels(staircase(False), [0, 0, .3])['voxel_indices']), 0)

    def test_wall_only_and_single_table(self):
        y, z = np.meshgrid(np.arange(-1, 1, .03), np.arange(0, 1, .03))
        wall = np.column_stack([np.zeros(y.size), y.ravel(), z.ravel()])
        self.assertEqual(len(detect_stair_voxels(wall, [0, 0, .3])['voxel_indices']), 0)
        table = staircase(False)[:735]
        self.assertEqual(len(detect_stair_voxels(table, [0, 0, .3])['voxel_indices']), 0)

    def test_vertical_faces_without_treads(self):
        faces = []
        for i in range(3):
            y, z = np.meshgrid(np.arange(-.6, .61, .025), np.arange(i * .15, (i + 1) * .15, .02))
            faces.append(np.column_stack([np.full(y.size, i * .4), y.ravel(), z.ravel()]))
        self.assertEqual(len(detect_stair_voxels(np.concatenate(faces), [0, 0, .3])['voxel_indices']), 0)

    def test_inconsistent_height_is_not_a_stair(self):
        p = staircase()
        p[p[:, 2] > .2, 2] += .2
        self.assertEqual(len(detect_stair_voxels(p, [0, 0, .3])['voxel_indices']), 0)

    def test_yaw_translation_independent(self):
        p = staircase(); angle = 1.16
        r = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        shift = np.array([8, -4, 2])
        result = detect_stair_voxels(p @ r.T + shift, np.array([0, 0, .3]) + shift)
        self.assertGreater(len(result['voxel_indices']), 50)
        np.testing.assert_allclose(result['models'][0]['rise_m'], [.15, .15], atol=.01)

    def test_mixed_cell_unknown(self):
        p = staircase(); original = detect_stair_voxels(p, [0, 0, .3]); cell = original['voxel_indices'][10]
        # An off-surface point in the same target cell must veto that cell.
        point = (cell + .5) * .08
        point[2] = cell[2] * .08 + .071
        mixed = detect_stair_voxels(np.vstack([p, point]), [0, 0, .3])
        self.assertNotIn(tuple(cell), set(map(tuple, mixed['voxel_indices'])))
        self.assertFalse(mixed['point_stair_mask'][-1])

    def test_missing_and_outside_evidence(self):
        p = staircase()
        self.assertEqual(len(detect_stair_voxels(p, [20, 20, .3])['voxel_indices']), 0)
        self.assertEqual(len(detect_stair_voxels(p[::100], [0, 0, .3])['voxel_indices']), 0)

    def test_crop_boundary_cells_remain_unknown(self):
        body = np.array([.6, 2.5, .3])
        result = detect_stair_voxels(staircase(), body)
        self.assertGreater(len(result['voxel_indices']), 10)
        lower = result['voxel_indices'] * .08
        upper = lower + .08
        far = np.maximum(abs(lower[:, :2] - body[:2]), abs(upper[:, :2] - body[:2]))
        self.assertTrue(np.all(np.linalg.norm(far, axis=1) <= 3.))
        self.assertTrue(np.all(lower[:, 2] >= body[2] - 2.))
        self.assertTrue(np.all(upper[:, 2] <= body[2] + 2.))

    def test_models_cannot_create_current_observations(self):
        models = detect_stair_voxels(staircase(), [0, 0, .3])['models']
        empty = match_stair_voxels(np.empty((0, 3)), [0, 0, .3], models)
        self.assertEqual(len(empty['voxel_indices']), 0)
        p = staircase()
        current = p[(p[:, 0] < .3) & (p[:, 2] == 0)]
        matched = match_stair_voxels(current, [0, 0, .3], models)
        self.assertGreater(len(matched['voxel_indices']), 10)
        observed = set(map(tuple, np.floor(current / .08).astype(int)))
        self.assertTrue(set(map(tuple, matched['voxel_indices'])).issubset(observed))

    def test_current_obstacle_does_not_match_old_model(self):
        models = detect_stair_voxels(staircase(), [0, 0, .3])['models']
        y, z = np.meshgrid(np.arange(-.6, .61, .025), np.arange(0, .31, .025))
        wall = np.column_stack([np.full(y.size, .9), y.ravel(), z.ravel()])
        self.assertEqual(len(match_stair_voxels(wall, [0, 0, .3], models)['voxel_indices']), 0)
        moved = staircase(False) + [0, 0, .06]
        self.assertEqual(len(match_stair_voxels(moved, [0, 0, .3], models)['voxel_indices']), 0)

    def test_current_mixed_cell_veto_with_frozen_model(self):
        p = staircase()
        model = detect_stair_voxels(p, [0, 0, .3])
        cell = model['voxel_indices'][10]
        point = (cell + .5) * .08
        point[2] = cell[2] * .08 + .071
        mixed = match_stair_voxels(np.vstack([p, point]), [0, 0, .3], model['models'])
        self.assertNotIn(tuple(cell), set(map(tuple, mixed['voxel_indices'])))

    def test_invalid_external_data(self):
        with self.assertRaises(ValueError): detect_stair_voxels([[np.nan, 0, 0]], [0, 0, 0])
        with self.assertRaises(ValueError): detect_stair_voxels([], [0, 0, 0])
        with self.assertRaises(ValueError): detect_stair_voxels(np.empty((0, 3)), [0, 0, 0], resolution=0)
