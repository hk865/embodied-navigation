"""SLAM-BE map->odom admission (R-COR-*) against the frozen correction.py interface.

Every candidate C = T_world_odom is judged at the body pose (C @ T_odom_body).
The body here is 30 m from the odom origin, so a small yaw about the body
moves C's translation a lot (lever arm, B-5 / A-1) without moving the body.
"""
import unittest

import numpy as np

import slam_backend_fixtures as F
from embodied_navigation.localization_mapping.backend.correction import CorrectionAdmission

SEC = 1_000_000_000
T_OB = F.pose(30.0, 0.0, 0.4)          # body 30 m from the odom origin
C0 = F.pose(1.0, -2.0, 0.2)


def about_body(C, dyaw, T_ob=T_OB):
    """C rotated by dyaw about the body: the body position stays, only its yaw changes."""
    return C @ T_ob @ F.pose(yaw=dyaw) @ F.inverse(T_ob)


def at_body(C, ref, T_ob=T_OB):
    return F.pose_error(C @ T_ob, ref @ T_ob)


class CorrectionAdmissionTests(unittest.TestCase):
    def _adopt(self, admission, C, t0_s=1):
        self.assertIsNone(admission.propose(C, T_OB, t0_s * SEC))
        adopted = admission.propose(C, T_OB, (t0_s + 1) * SEC)
        self.assertIsNotNone(adopted)
        return adopted

    def test_r_cor_1_lever_arm_consistent_candidates_adopted(self):
        # Catches: comparing the translation of C directly (B-5): 0.003 rad at 30 m looks like 0.09-0.18 m.
        cases = [
            # (thresholds, candidate a, candidate b)
            (dict(agree_translation_m=0.05), C0, about_body(C0, 0.003)),
            (dict(), about_body(C0, 0.003), about_body(C0, -0.003)),   # default 0.15 m gate
        ]
        for kwargs, Ca, Cb in cases:
            with self.subTest(kwargs=kwargs):
                gate = kwargs.get("agree_translation_m", 0.15)
                # fixture sanity: a raw C-translation comparison would call them inconsistent
                self.assertGreater(np.linalg.norm(Ca[:3, 3] - Cb[:3, 3]), gate)
                self.assertLess(at_body(Ca, Cb)[0], 1e-9)
                admission = CorrectionAdmission(**kwargs)
                self.assertIsNone(admission.propose(Ca, T_OB, 1 * SEC))
                adopted = admission.propose(Cb, T_OB, 2 * SEC)
                self.assertIsNotNone(adopted)
                t_err, r_err = at_body(adopted, Ca)
                self.assertLess(t_err, 0.01)
                self.assertLess(r_err, 0.01)
                np.testing.assert_allclose(admission.current, adopted, atol=1e-12)

    def test_r_cor_2_body_jump_rejected_and_current_kept(self):
        # Catches: no jump gate after initialisation, or a rejected jump clearing the adopted C.
        admission = CorrectionAdmission()
        adopted = self._adopt(admission, C0)
        kept = np.array(admission.current, copy=True)
        np.testing.assert_allclose(kept, adopted, atol=1e-12)
        jump = F.pose(3.0, 0.0, 0.0) @ C0                     # body moves 3 m in the world
        self.assertAlmostEqual(at_body(jump, C0)[0], 3.0, places=9)
        self.assertIsNone(admission.propose(jump, T_OB, 20 * SEC))
        self.assertIsNone(admission.propose(jump, T_OB, 21 * SEC))   # consistent with each other, still a jump
        self.assertIsNotNone(admission.current)
        np.testing.assert_array_equal(admission.current, kept)

    def test_r_cor_2_jump_is_measured_at_body_not_odom_origin(self):
        # Catches: the jump gate comparing C translations (0.04 rad about a body 30 m out = 1.2 m "jump").
        admission = CorrectionAdmission()
        self._adopt(admission, C0)
        rotated = about_body(C0, 0.04)                      # body: 0 m, 0.04 rad < 0.35 rad gate
        self.assertGreater(np.linalg.norm(rotated[:3, 3] - C0[:3, 3]), 1.0)
        outputs = [admission.propose(rotated, T_OB, 20 * SEC), admission.propose(rotated, T_OB, 21 * SEC)]
        self.assertTrue(any(out is not None for out in outputs))
        t_err, r_err = at_body(admission.current, rotated)
        self.assertLess(t_err, 0.01)
        self.assertLess(r_err, 0.01)

    def test_r_cor_3_single_candidate_and_window(self):
        # Catches: adopting on the first candidate, or confirming with a candidate older than window_s.
        admission = CorrectionAdmission()                    # window_s = 10 s, confirmations = 2
        self.assertIsNone(admission.propose(C0, T_OB, 0))
        self.assertIsNone(admission.current)
        self.assertIsNone(admission.propose(C0, T_OB, 11 * SEC))   # 11 s later: outside the window
        self.assertIsNone(admission.current)
        adopted = admission.propose(C0, T_OB, 12 * SEC)              # pairs with the 11 s candidate
        self.assertIsNotNone(adopted)
        self.assertLess(at_body(adopted, C0)[0], 1e-6)


if __name__ == "__main__":
    unittest.main()
