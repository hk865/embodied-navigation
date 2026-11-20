import unittest
import numpy as np
from embodied_navigation.perception.depth import depth_to_cloud


class DepthTests(unittest.TestCase):
    def test_actual_projection_and_self_filter_formula(self):
        rng = np.random.default_rng(42)
        depth = rng.uniform(0, 22, (64, 64))
        depth[0, 0] = np.nan
        mount = np.array([0.1, 0.2, 0.3])
        imu = np.array([0.02, 0, -0.005])
        R = np.eye(3)
        row, col = np.indices(depth.shape)
        keep = np.isfinite(depth) & (depth > 0.05) & (depth < 20)
        old = np.column_stack(
            [
                depth[keep],
                -(col[keep] - 256) / (512 * 24 / 42) * depth[keep],
                -(row[keep] - 256) / (512 * 24 / 42) * depth[keep],
            ]
        ) @ R.T + (mount - imu)
        body = old + imu
        mask = (
            (np.abs(body[:, 0]) < 0.5)
            & (np.abs(body[:, 1]) < 0.3)
            & (body[:, 2] > -0.45)
            & (body[:, 2] < 0.3)
        )
        old = old[~mask]
        old = old[:: max(1, (len(old) + 7999) // 8000)]
        new, valid, self_mask = depth_to_cloud(depth, R, mount, imu)
        np.testing.assert_array_equal(old, new)
        np.testing.assert_array_equal(keep, valid)
        np.testing.assert_array_equal(mask, self_mask)


if __name__ == "__main__":
    unittest.main()
