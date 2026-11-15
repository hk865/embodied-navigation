"""SLAM-BE ScanContext (R-SC-*) against the frozen scan_context.py interface.

Yaw convention used here (the one R-SC-2 needs for ICP seeding):
``d, s = distance(a, b)`` with ``a`` the query scan and ``b`` the stored scan;
``shift_to_yaw(s)`` is the yaw of ``T_b_a`` (rotates the query cloud into the
stored scan's frame), i.e. the ICP initial value with source=query and
target=stored.  ``ScanContextIndex.query(desc)`` returns that same yaw for each
stored key.
"""
import math
import unittest

import numpy as np

import slam_backend_fixtures as F
from embodied_navigation.localization_mapping.backend.registration import icp_point_to_plane
from embodied_navigation.localization_mapping.backend.scan_context import (
    ScanContextConfig, ScanContextIndex, descriptor, distance, ring_key, shift_to_yaw)

CFG = ScanContextConfig()
SECTOR_RAD = 2.0 * math.pi / CFG.sectors


class ScanContextTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = F.make_world()
        cls.A = F.loop_pose_at(14.0)                      # scene (19, 4), heading +x
        _, cls.body_A = cls.world.scan_body(cls.A, 101)

    def test_config_defaults_and_shapes(self):
        # Catches: changed frozen defaults or wrong descriptor / ring-key shapes.
        self.assertEqual((CFG.rings, CFG.sectors, CFG.max_range_m), (20, 60, 20.0))
        desc = descriptor(self.body_A, CFG)
        self.assertEqual(desc.shape, (20, 60))
        self.assertEqual(ring_key(desc).shape, (20,))

    def test_r_sc_1_rotation_0p7_distance_and_yaw(self):
        # Catches: wrong yaw sign or shift->angle conversion (e.g. +shift vs -shift, degrees, sector width).
        yaw_true = 0.7
        rotated = F.transform_points(F.pose(yaw=yaw_true), self.body_A)  # T_rot_orig = Rz(+0.7)
        query, stored = descriptor(self.body_A, CFG), descriptor(rotated, CFG)
        d, shift = distance(query, stored)
        self.assertLess(d, 0.15)
        self.assertGreaterEqual(d, 0.0)
        yaw = shift_to_yaw(shift, CFG)
        self.assertLessEqual(abs(F.wrap_angle(yaw - yaw_true)), SECTOR_RAD)
        # Same through the index: the stored scan is the rotated one, so T_stored_query = Rz(+0.7).
        index = ScanContextIndex(CFG)
        index.add("stored", stored)
        key, d_index, yaw_index = index.query(query)[0]
        self.assertEqual(key, "stored")
        self.assertLess(d_index, 0.15)
        self.assertLessEqual(abs(F.wrap_angle(yaw_index - yaw_true)), SECTOR_RAD)

    def test_r_sc_2_yaw_from_shift_seeds_icp_to_truth(self):
        # Catches: a yaw sign convention that disagrees with ICP's T_target_source (ICP then starts 1.4 rad off).
        B = F.loop_pose_at(14.5, 0.7) @ F.pose(0.0, 0.3)  # 0.5 m along, 0.3 m aside, heading +0.7 rad
        _, body_B = self.world.scan_body(B, 202)
        T_A_B = F.inverse(self.A) @ B                     # truth: query(B) -> stored(A)
        desc_A, desc_B = descriptor(self.body_A, CFG), descriptor(body_B, CFG)
        _, shift = distance(desc_B, desc_A)
        index = ScanContextIndex(CFG)
        index.add("A", desc_A)
        _, _, yaw_index = index.query(desc_B)[0]
        for label, yaw in (("distance", shift_to_yaw(shift, CFG)), ("index", yaw_index)):
            with self.subTest(label):
                result = icp_point_to_plane(body_B, self.body_A, F.pose(yaw=yaw))
                self.assertTrue(result.converged)
                t_err, r_err = F.pose_error(result.T_target_source, T_A_B)
                self.assertLess(t_err, 0.1)
                self.assertLess(r_err, 0.03)

    def test_r_sc_3_distinct_places_farther_than_same_place(self):
        # Catches: a degenerate descriptor (e.g. constant, or range/height ignored) that cannot tell places apart.
        _, same_body = self.world.scan_body(self.A @ F.pose(yaw=0.7), 303)  # same place, new scan, rotated
        desc_A = descriptor(self.body_A, CFG)
        d_same, _ = distance(desc_A, descriptor(same_body, CFG))
        index = ScanContextIndex(CFG)
        index.add("A", desc_A)
        for arc in (40.0, 30.0, 52.0):                    # (15,14), (25,14), (5,12): all >= 10 m from (19,4)
            far = F.loop_pose_at(arc)
            self.assertGreater(np.linalg.norm(far[:2, 3] - self.A[:2, 3]), 10.0)
            _, far_body = self.world.scan_body(far, 400 + int(arc))
            desc_far = descriptor(far_body, CFG)
            with self.subTest(arc=arc):
                self.assertGreater(distance(desc_A, desc_far)[0], d_same)
            index.add(("far", arc), desc_far)
        results = index.query(descriptor(same_body, CFG))
        self.assertEqual(results[0][0], "A")
        dists = [r[1] for r in results]
        self.assertEqual(dists, sorted(dists))

    def test_contract_index_exclude(self):
        # Catches: query ignoring the exclude predicate (recent keyframes would close trivial loops).
        index = ScanContextIndex(CFG)
        desc_A = descriptor(self.body_A, CFG)
        index.add((1, 1), desc_A)
        index.add((1, 2), desc_A.copy())
        keys = [r[0] for r in index.query(desc_A, exclude=lambda key: key == (1, 2))]
        self.assertIn((1, 1), keys)
        self.assertNotIn((1, 2), keys)


if __name__ == "__main__":
    unittest.main()
