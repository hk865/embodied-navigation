import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from embodied_navigation.planning.stair_collision_shadow import BodyBox, compare_stair_collision, body_voxel_hits, swept_body_collision

class StairShadowTests(unittest.TestCase):
    def evaluate(self, voxels, labels, box=None):
        return compare_stair_collision(
            voxel_indices=np.array(voxels,dtype=int).reshape(-1,3),
            stair_mask=np.array(labels,dtype=bool),
            query_indices=np.array([[0,0,2]]),
            boxes=(box or BodyBox(np.array([.04,.04,.24]),np.array([.10,.04,.04]),np.eye(3)),),
            resolution=.08,radius=.25,inflation_up=.12,inflation_down=.12)
    def test_stair_keeps_raw_geometry_without_inflation(self):
        r=self.evaluate([[0,0,0]],[True])
        self.assertTrue(r['baseline_observed_collision'])
        self.assertFalse(r['candidate_observed_collision'])
        self.assertFalse(r['runtime_permit'])
    def test_wall_keeps_original_inflation(self):
        r=self.evaluate([[0,0,0]],[False])
        self.assertTrue(r['candidate_observed_collision'])
        self.assertEqual(r['retained_non_stair_inflation_hits'],[[0,0,0]])
    def test_stair_body_collision_not_removed(self):
        r=self.evaluate([[0,0,2]],[True])
        self.assertTrue(r['candidate_observed_collision'])
        self.assertEqual(r['raw_stair_body_hits'],[[0,0,2]])
    def test_pitch_changes_body_intersection(self):
        b=BodyBox(np.array([.04,.04,.24]),np.array([.20,.04,.04]),Rotation.from_euler('y',60,degrees=True).as_matrix())
        self.assertTrue(self.evaluate([[0,0,1]],[True],b)['candidate_observed_collision'])
    def test_finite_voxel_boundary_not_only_center(self):
        b=BodyBox(np.array([.12,.04,.24]),np.array([.05,.04,.04]),np.eye(3))
        self.assertTrue(self.evaluate([[0,0,2]],[True],b)['candidate_observed_collision'])
    def test_mixed_labels_keep_wall(self):
        r=self.evaluate([[0,0,0],[1,0,0]],[True,False])
        self.assertTrue(r['candidate_observed_collision'])
        self.assertEqual(r['retained_non_stair_inflation_hits'],[[1,0,0]])
    def test_empty_observations_are_not_runtime_permit(self):
        self.assertFalse(self.evaluate([],[])['runtime_permit'])
    def test_missing_body_rejected(self):
        with self.assertRaises(ValueError):
            compare_stair_collision(voxel_indices=np.array([[0,0,0]]),stair_mask=np.array([True]),query_indices=np.array([[0,0,2]]),boxes=(),resolution=.08,radius=.25,inflation_up=.12,inflation_down=.12)

class SweptBodyTests(unittest.TestCase):
    def sweep(self, voxels, box, positions, rotations=None, subdivisions=8):
        if rotations is None:
            rotations = np.repeat(np.eye(3)[None], len(positions), axis=0)
        return swept_body_collision(
            voxel_indices=np.array(voxels, dtype=int).reshape(-1, 3), resolution=.1,
            body_boxes=(box,), reference_positions=np.array(positions),
            reference_rotations=rotations, subdivisions=subdivisions)

    def test_translation_endpoints_clear_but_middle_blocked(self):
        box = BodyBox(np.zeros(3), np.full(3, .04), np.eye(3))
        voxels = np.array([[10, 0, 0]])
        for p in ([0, .05, .05], [2, .05, .05]):
            self.assertFalse(body_voxel_hits(voxels, .1, (BodyBox(np.array(p), box.half_extents, box.rotation),)).any())
        r = self.sweep(voxels, box, [[0, .05, .05], [2, .05, .05]])
        self.assertEqual(r['hit_indices'], [[10, 0, 0]])
        self.assertFalse(r['runtime_permit'])

    def test_rotating_long_body_endpoints_clear_middle_blocked(self):
        box = BodyBox(np.zeros(3), np.array([1., .03, .03]), np.eye(3))
        rotations = Rotation.from_euler('z', [0, 90], degrees=True).as_matrix()
        voxels = np.array([[6, 6, 0]])
        for rotation in rotations:
            self.assertFalse(body_voxel_hits(voxels, .1, (BodyBox(np.zeros(3), box.half_extents, rotation),)).any())
        self.assertTrue(self.sweep(voxels, box, [[0, 0, 0]] * 2, rotations)['conservative_observed_collision'])

    def test_offset_component_rotation_covered_with_one_interval(self):
        box = BodyBox(np.array([1., 0, 0]), np.full(3, .02), np.eye(3))
        rotations = Rotation.from_euler('z', [0, 90], degrees=True).as_matrix()
        # Obstacle near a quarter-interval position, not either endpoint or midpoint.
        voxels = np.array([[9, 3, 0]])
        r = self.sweep(voxels, box, [[0, 0, 0]] * 2, rotations, subdivisions=1)
        self.assertTrue(r['conservative_observed_collision'])
        self.assertGreater(r['max_envelope_padding_m'], .7)

    def test_far_obstacle_clear_and_all_voxels_counted(self):
        box = BodyBox(np.zeros(3), np.full(3, .04), np.eye(3))
        r = self.sweep([[100, 100, 100], [-100, 0, 0]], box, [[0, 0, 0], [1, 0, 0]])
        self.assertFalse(r['conservative_observed_collision'])
        self.assertEqual(r['occupied_voxel_count'], 2)

    def test_empty_map_never_permission(self):
        box = BodyBox(np.zeros(3), np.full(3, .04), np.eye(3))
        self.assertFalse(self.sweep([], box, [[0, 0, 0], [1, 0, 0]])['runtime_permit'])

    def test_external_invalid_pose_and_subdivisions_rejected(self):
        box = BodyBox(np.zeros(3), np.full(3, .04), np.eye(3))
        for subdivisions in (0, 1.5, True):
            with self.assertRaises(ValueError):
                self.sweep([], box, [[0, 0, 0], [1, 0, 0]], subdivisions=subdivisions)
        with self.assertRaises(ValueError):
            self.sweep([], box, [[0, 0, 0], [np.nan, 0, 0]])

if __name__=='__main__':unittest.main()

