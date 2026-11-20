"""多帧实测 LiDAR 子图分支（exp/slam-measured-submaps）单元测试。"""

from pathlib import Path
import unittest

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.submaps import SubmapConfig, build_submap
import slam_backend_fixtures as F

REPO_ROOT = Path(__file__).resolve().parents[2]


class MeasuredSubmapsTests(unittest.TestCase):
    def _make_kf(self, seq: int, stamp_s: float, x: float = 0.0, z: float = 0.0, epoch: int = 1) -> Keyframe:
        T = np.eye(4)
        T[0, 3] = x
        T[2, 3] = z
        pts = np.array([[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.5]], dtype=np.float64)
        return Keyframe(
            key=(epoch, seq),
            stamp_ns=int(round(stamp_s * 1e9)),
            T_odom_body=T,
            T_body_lidar=np.eye(4),
            points_lidar=pts,
        )

    def test_build_submap_stops_at_continuity_budget(self):
        # 第 1 帧高度跳变超出 0.5 m 预算，应只累积第 2、3 帧。
        frames = [
            self._make_kf(1, 1.0, x=0.0, z=-1.0),
            self._make_kf(2, 2.0, x=0.5, z=0.0),
            self._make_kf(3, 3.0, x=1.0, z=0.1),
        ]
        submap = build_submap(frames, frames[-1], SubmapConfig(max_frames=5, max_height_m=0.5))
        self.assertEqual(submap.source_keys, ((1, 3), (1, 2)))
        self.assertGreater(len(submap.points_body), 0)
        self.assertEqual(len(submap.point_sources), len(submap.points_body))

    def test_submap_config_rejects_invalid_values(self):
        with self.assertRaises(ValueError):
            SubmapConfig(max_frames=0)
        with self.assertRaises(ValueError):
            SubmapConfig(voxel_m=0.0)
        with self.assertRaises(ValueError):
            SubmapConfig(anchor_mode="invalid")
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"measured_submap": {"unknown_key": 1}})

    def test_config_json_loads_cleanly_and_closes_loop_in_ablation_modes(self):
        cfg = BackendConfig.from_json(REPO_ROOT / "configs/localization_mapping/slam_measured_submaps.json")
        self.assertTrue(cfg.use_measured_submaps)
        self.assertEqual(cfg.measured_submap.anchor_mode, "nearest")
        self.assertAlmostEqual(cfg.loop_sigma_rad, 0.03)
        self.assertAlmostEqual(cfg.loop_sigma_yaw_rad, 0.01)

        loop = F.closed_loop_with_drift(drift_m=1.0, step_m=3.0)
        rows = F.keyframe_stream(loop["scene"], loop["odom"], epoch=11)
        for anchor_mode in ("nearest", "candidate"):
            with self.subTest(anchor_mode=anchor_mode):
                sub_cfg = BackendConfig(
                    use_measured_submaps=True,
                    measured_submap=SubmapConfig(anchor_mode=anchor_mode),
                    loop_sigma_rad=0.03,
                    loop_sigma_yaw_rad=0.01,
                    cauchy_scale=3.0,
                )
                backend = SlamBackend(sub_cfg)
                events = [backend.ingest(Keyframe(**row)) for row in rows]
                self.assertTrue(any(e.get("loop") for e in events))
                self.assertGreater(backend.measured_query_submaps, 0)
                self.assertGreater(backend.measured_target_submaps, 0)
                self.assertFalse([e for e in events if "error" in e])


if __name__ == "__main__":
    unittest.main()
