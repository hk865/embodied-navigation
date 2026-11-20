"""重力对齐实测局部平面建模与回环表面一致性门控单元测试。"""

from __future__ import annotations

from pathlib import Path
import unittest

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.surfaces import (
    SurfaceConfig,
    compare_surfaces,
    model_surfaces,
)

REPO_ROOT = Path(__file__).resolve().parents[2]


def _grid_plane(
    xs: np.ndarray,
    ys: np.ndarray,
    z: float = 0.0,
) -> np.ndarray:
    """在 z=const 上生成规则水平地面采样点。"""
    gx, gy = np.meshgrid(xs, ys)
    return np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, z, dtype=float)])


def _vertical_wall(
    x: float,
    ys: np.ndarray,
    zs: np.ndarray,
) -> np.ndarray:
    """在 x=const 上生成规则竖直墙面采样点。"""
    gy, gz = np.meshgrid(ys, zs)
    return np.column_stack([np.full(gy.size, x, dtype=float), gy.ravel(), gz.ravel()])


def _room_patch(z_offset: float = 0.0) -> np.ndarray:
    """构造含水平地面与两面正交竖直墙的实测局部场景点云。"""
    floor = _grid_plane(np.linspace(-2.0, 2.0, 41), np.linspace(-2.0, 2.0, 41), z=z_offset)
    wall_x = _vertical_wall(2.0, np.linspace(-2.0, 2.0, 41), np.linspace(0.05, 1.75, 18) + z_offset)
    wall_y_gx, wall_y_gz = np.meshgrid(np.linspace(-2.0, 2.0, 41), np.linspace(0.05, 1.75, 18) + z_offset)
    wall_y = np.column_stack([wall_y_gx.ravel(), np.full(wall_y_gx.size, 2.0), wall_y_gz.ravel()])
    return np.vstack([floor, wall_x, wall_y])


class SurfaceGateUnitTest(unittest.TestCase):
    def test_model_surfaces_separates_horizontal_vertical_and_unknown(self) -> None:
        """水平地面、竖直墙面应正确分类，稀疏噪声点保持未知而不插值。"""
        room = _room_patch(z_offset=0.0)
        # 少量远离平面的孤立点，达不到 min_points 应归为 unknown_points
        sparse_noise = np.array([[10.0, 10.0, 5.0], [10.05, 10.0, 5.02], [10.0, 10.05, 5.04]])
        points = np.vstack([room, sparse_noise])

        model = model_surfaces(points, config=SurfaceConfig())
        kinds = {patch.kind for patch in model.patches}
        self.assertEqual(kinds, {"horizontal", "vertical"})
        self.assertGreaterEqual(model.unknown_points, len(sparse_noise))
        self.assertEqual(model.observed_points, len(points))

        horizontal = [patch for patch in model.patches if patch.kind == "horizontal"]
        self.assertTrue(horizontal)
        for patch in horizontal:
            self.assertIsNotNone(patch.height_m)
            self.assertAlmostEqual(patch.height_m, 0.0, places=3)

    def test_compare_surfaces_accepts_consistent_and_rejects_shifted_floor(self) -> None:
        """同高度平面一致应放行，跨高度偏移（模拟错层或 Z 漂移）应被拒绝。"""
        cfg = SurfaceConfig()
        ref_model = model_surfaces(_room_patch(z_offset=0.0), config=cfg)
        same_model = model_surfaces(_room_patch(z_offset=0.01), config=cfg)
        shifted_model = model_surfaces(_room_patch(z_offset=0.25), config=cfg)

        ok = compare_surfaces(same_model, ref_model, cfg)
        self.assertTrue(ok.accepted)
        self.assertEqual(ok.reason, "consistent")
        self.assertGreaterEqual(ok.matched_patches, cfg.min_patches)
        self.assertLessEqual(ok.rmse_m, cfg.max_residual_m)

        bad = compare_surfaces(shifted_model, ref_model, cfg)
        self.assertFalse(bad.accepted)
        self.assertEqual(bad.reason, "surface_mismatch")

    def test_compare_surfaces_rejects_insufficient_surfaces(self) -> None:
        """当任一侧未能提取出足够平面片时，必须返回 insufficient_surface_observations 拒绝。"""
        cfg = SurfaceConfig(min_patches=1)
        empty_model = model_surfaces(np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]), config=cfg)
        ref_model = model_surfaces(_room_patch(), config=cfg)

        result = compare_surfaces(empty_model, ref_model, cfg)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "insufficient_surface_observations")

    def test_backend_config_strict_validation_and_json_loading(self) -> None:
        """配置加载必须强制 gravity_aligned=True 且拒绝未知键或非法范围。"""
        with self.assertRaises(ValueError):
            BackendConfig(use_surface_verification=True, gravity_aligned=False)
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"use_surface_verification": True, "gravity_aligned": False})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"use_surface_verification": "true", "gravity_aligned": True})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"surface": {"unknown_key": 1.0}})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"surface": {"min_coverage": 1.5}})

        cfg_path = REPO_ROOT / "configs" / "localization_mapping" / "slam_surface_gate.json"
        cfg = BackendConfig.from_json(cfg_path)
        self.assertTrue(cfg.use_surface_verification)
        self.assertTrue(cfg.gravity_aligned)
        self.assertAlmostEqual(cfg.map_voxel_m, 0.05)
        self.assertAlmostEqual(cfg.loop_sigma_rad, 0.03)
        self.assertAlmostEqual(cfg.loop_sigma_yaw_rad, 0.01)
        self.assertAlmostEqual(cfg.cauchy_scale, 3.0)

    def test_slam_backend_surface_gate_accepts_true_loop_and_records_rejection(self) -> None:
        """SlamBackend 开启表面门控时，真回环放行且可记录拒绝原因。"""
        pts = _room_patch(z_offset=0.0)
        cfg = BackendConfig(
            use_surface_verification=True,
            gravity_aligned=True,
            map_voxel_m=0.05,
            icp_voxel_m=0.1,
            min_loop_gap_s=1.0,
            exclude_recent=1,
            loop_submap_neighbors=0,
            min_fitness=0.5,
            max_rmse_m=0.15,
        )
        backend = SlamBackend(cfg)

        def _kf(idx: int, stamp_s: float, x: float, cloud: np.ndarray) -> Keyframe:
            T = np.eye(4)
            T[0, 3] = x
            return Keyframe(
                key=f"kf_{idx}",
                stamp_ns=int(stamp_s * 1e9),
                T_odom_body=T,
                points_lidar=cloud,
                T_body_lidar=np.eye(4),
            )

        # 关键帧 0 与 1 拉开时间窗，关键帧 2 回到原点触发真回环。
        backend.ingest(_kf(0, 0.0, 0.0, pts))
        backend.ingest(_kf(1, 2.0, 1.0, pts - np.array([1.0, 0.0, 0.0])))
        ev = backend.ingest(_kf(2, 4.0, 0.02, pts))

        self.assertTrue(ev["loop"])
        self.assertGreaterEqual(backend.surface_checks, 1)
        self.assertGreaterEqual(backend.surface_accepted, 1)
        self.assertEqual(backend.surface_rejected_by_reason, {})

        # 构造一个把 min_patches 抬到不可能满足的后端，验证表面门控会拒绝该候选并计数。
        strict_cfg = BackendConfig(
            use_surface_verification=True,
            gravity_aligned=True,
            map_voxel_m=0.05,
            icp_voxel_m=0.1,
            min_loop_gap_s=1.0,
            exclude_recent=1,
            loop_submap_neighbors=0,
            min_fitness=0.5,
            max_rmse_m=0.15,
            surface=SurfaceConfig(min_patches=99),
        )
        strict_backend = SlamBackend(strict_cfg)
        strict_backend.ingest(_kf(0, 0.0, 0.0, pts))
        strict_backend.ingest(_kf(1, 2.0, 1.0, pts - np.array([1.0, 0.0, 0.0])))
        ev_rej = strict_backend.ingest(_kf(2, 4.0, 0.02, pts))

        self.assertFalse(ev_rej["loop"])
        self.assertGreaterEqual(strict_backend.surface_checks, 1)
        self.assertEqual(strict_backend.surface_accepted, 0)
        self.assertGreaterEqual(
            strict_backend.surface_rejected_by_reason.get("insufficient_surface_observations", 0), 1
        )


if __name__ == "__main__":
    unittest.main()
