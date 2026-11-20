"""SLAM-BE SE(3) pose graph (R-PG-*) against the frozen pose_graph.py interface.

Information matrices are isotropic (scalar * I6) so the tests do not depend on
whether the implementation orders the 6-vector as (rotation, translation) or
(translation, rotation).
"""
import math
import unittest

import numpy as np

import slam_backend_fixtures as F
from embodied_navigation.localization_mapping.backend.pose_graph import PoseGraph

N = 40
INFO = 100.0 * np.eye(6)


def square_truth(start=(2.0, 3.0, 0.3)):
    """40 nodes, 1 m apart, on a 10 m x 10 m square (closed: node 40 would be node 0)."""
    corners = np.array([[0, 0], [10, 0], [10, 10], [0, 10], [0, 0]], dtype=float)
    poses = []
    for k in range(N):
        side, s = divmod(k, 10)
        a, b = corners[side], corners[side + 1]
        p = a + (b - a) * (s / 10.0)
        poses.append(F.pose(p[0], p[1], math.atan2(b[1] - a[1], b[0] - a[0])))
    offset = F.pose(*start[:2], start[2])
    return [offset @ T for T in poses]


def translation_error(T, truth):
    return float(np.linalg.norm(T[:3, 3] - truth[:3, 3]))


class PoseGraphTests(unittest.TestCase):
    def test_r_pg_1_square_loop_with_yaw_drift(self):
        # Catches: an update applied in the wrong direction / wrong error sign (optimisation makes it worse).
        truth = square_truth()
        drift = math.radians(0.5)  # 0.5 deg per metre, every edge is 1 m
        measured = [F.inverse(truth[k]) @ truth[k + 1] @ F.pose(yaw=drift) for k in range(N - 1)]
        initial = [truth[0]]
        for rel in measured:
            initial.append(initial[-1] @ rel)
        graph = PoseGraph()
        for k in range(N):
            graph.add_node(k, initial[k])
        for k, rel in enumerate(measured):
            graph.add_edge(k, k + 1, rel, INFO)
        graph.add_edge(N - 1, 0, F.inverse(truth[N - 1]) @ truth[0], INFO)  # exact loop closure
        before = translation_error(initial[-1], truth[-1])
        self.assertGreater(before, 1.5)  # fixture sanity: 20 deg accumulated drift (~2.1 m at the end)
        result = graph.optimize()
        for key in ("converged", "iterations", "error_before", "error_after"):
            self.assertIn(key, result)
        self.assertLess(result["error_after"], result["error_before"])
        after = translation_error(graph.pose(N - 1), truth[-1])
        self.assertLessEqual(after, before / 10.0)
        np.testing.assert_allclose(graph.pose(0), truth[0], atol=1e-9)  # first node is fixed

    def test_r_pg_2_cauchy_rejects_3m_wrong_loop(self):
        # Catches: robust=True being ignored (a 3 m outlier then bends the whole loop by ~2.7 m).
        # sigma = 1 cm / 0.01 rad on every edge.  Hand check: least squares bends the loop by ~2.2 m;
        # Cauchy on the whitened residual (chi2 = 9e4 for the outlier) keeps nodes within cm for any
        # kernel scale up to ~10 sigma.
        info = 1e4 * np.eye(6)
        truth = square_truth()
        rng = np.random.default_rng(F.SEED)
        graph = PoseGraph()
        for k in range(N):
            noise = np.eye(4) if k == 0 else F.pose(*rng.normal(0, 0.05, 2), rng.normal(0, 0.01))
            graph.add_node((1, k), truth[k] @ noise)
        for k in range(N - 1):
            graph.add_edge((1, k), (1, k + 1), F.inverse(truth[k]) @ truth[k + 1], info)
        graph.add_edge((1, N - 1), (1, 0), F.inverse(truth[N - 1]) @ truth[0], info, robust=True)
        wrong = F.inverse(truth[10]) @ truth[30]
        wrong[:3, 3] += [3.0, 0.0, 0.0]  # 3 m off
        graph.add_edge((1, 10), (1, 30), wrong, info, robust=True)
        graph.optimize()
        errors = [translation_error(graph.pose((1, k)), truth[k]) for k in range(N)]
        self.assertLessEqual(max(errors), 0.3, f"worst node error {max(errors):.3f} m")


if __name__ == "__main__":
    unittest.main()
