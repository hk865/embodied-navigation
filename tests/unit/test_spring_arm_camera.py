"""游戏风格 Spring-Arm 防穿墙相机、楼层切片与室外 V4 配置单元测试。"""

from __future__ import annotations

from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from embodied_navigation.localization_mapping.backend.backend import BackendConfig
from embodied_navigation.visualization.spring_arm_camera import (
    SpringArmCameraConfig,
    SpringArmCameraController,
    VoxelObstacleGrid,
    build_look_at_pose,
    filter_floor_cutaway,
)


class SpringArmCameraTests(unittest.TestCase):
    def test_sphere_cast_retracts_before_wall_and_catches_side_pillar(self) -> None:
        grid = VoxelObstacleGrid(voxel_m=0.25)
        # 在 x = -2.0 m 处立一面墙（垂直于机器人后方）
        ys, zs = np.meshgrid(np.linspace(-2.0, 2.0, 25), np.linspace(0.0, 3.0, 20))
        wall = np.column_stack([np.full(ys.size, -2.0), ys.ravel(), zs.ravel()])
        grid.replace_points(wall)

        origin = np.array([0.0, 0.0, 0.7])
        target = np.array([-4.5, 0.0, 1.5])
        safe_dist, clipped = grid.sphere_cast(
            origin,
            target,
            probe_radius_m=0.35,
            collision_margin_m=0.35,
            min_dist_m=0.8,
            step_m=0.1,
        )
        self.assertTrue(clipped)
        # 相机眼点必须严格停在墙前（x > -2.0 m），绝不穿墙
        direction = (target - origin) / np.linalg.norm(target - origin)
        eye = origin + direction * safe_dist
        self.assertGreater(eye[0], -1.95)

        # 旁侧柱子（中心射线未命中，但在球体探头半径 0.35 m 内）也应触发收缩
        grid.clear()
        pillar_z = np.linspace(0.0, 3.0, 25)
        pillar = np.column_stack([np.full_like(pillar_z, -2.2), np.full_like(pillar_z, 0.25), pillar_z])
        grid.replace_points(pillar)
        safe_dist_p, clipped_p = grid.sphere_cast(origin, target, probe_radius_m=0.35, collision_margin_m=0.3)
        self.assertTrue(clipped_p)
        self.assertLess(safe_dist_p, 2.3)

    def test_ceiling_clamp_and_fast_retract_smooth_extend(self) -> None:
        cfg = SpringArmCameraConfig(
            desired_arm_length_m=4.5,
            min_arm_length_m=0.9,
            pitch_deg=30.0,
            ceiling_margin_m=0.35,
            retract_tau_s=0.05,
            extend_tau_s=0.80,
        )
        ctrl = SpringArmCameraController(cfg)
        # 在 z = 2.6 m 处铺一层天花板
        xs, ys = np.meshgrid(np.linspace(-4.0, 4.0, 35), np.linspace(-4.0, 4.0, 35))
        ceil = np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, 2.6)])
        ctrl.grid.replace_points(ceil)

        cams = ctrl.update(np.array([0.0, 0.0, 0.0]), 0.0, 1.0)
        spring = cams["spring_follow"]
        self.assertTrue(spring.clipped)
        # 相机高度不得穿入 z=2.6 m 的天花板
        self.assertLessEqual(spring.position_xyz[2], 2.6 - cfg.ceiling_margin_m + 0.05)

        # 在机器人后方 x=-1.8 突然出现墙壁：1 步（0.1 s）内必须立刻收缩到墙前
        ys_w, zs_w = np.meshgrid(np.linspace(-2.0, 2.0, 20), np.linspace(0.0, 2.6, 15))
        wall = np.column_stack([np.full(ys_w.size, -1.8), ys_w.ravel(), zs_w.ravel()])
        ctrl.grid.add_points(wall)
        cams_hit = ctrl.update(np.array([0.0, 0.0, 0.0]), 0.0, 1.1)
        self.assertLess(cams_hit["spring_follow"].arm_length_m, 1.9)

        # 障碍消失后应按 extend_tau_s 柔和回弹，而非 0.1 s 内瞬间弹满
        ctrl.grid.clear()
        cams_rel = ctrl.update(np.array([0.0, 0.0, 0.0]), 0.0, 1.2)
        self.assertGreater(cams_rel["spring_follow"].arm_length_m, cams_hit["spring_follow"].arm_length_m)
        self.assertLess(cams_rel["spring_follow"].arm_length_m, cfg.desired_arm_length_m - 0.5)

    def test_look_at_pose_and_floor_cutaway(self) -> None:
        eye = np.array([-3.0, 0.0, 2.0])
        tgt = np.array([1.0, 0.0, 0.5])
        cam = build_look_at_pose(eye, tgt, frame_id="rviz_cam_spring")
        R = Rotation.from_quat(cam.quaternion_xyzw).as_matrix()
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-6)
        self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=6)
        # RViz FrameAligned (Yaw=0, Pitch=0) 沿 +X 轴看向前方，+Z 轴朝上
        fwd_expected = (tgt - eye) / np.linalg.norm(tgt - eye)
        np.testing.assert_allclose(R[:, 0], fwd_expected, atol=1e-6)
        self.assertGreater(R[2, 2], 0.5)

        pts = np.array([[0, 0, -1.5], [0, 0, 0.0], [0, 0, 1.2], [0, 0, 3.3]], dtype=np.float64)
        sliced = filter_floor_cutaway(pts, robot_z=0.0, above_m=1.85, below_m=0.90)
        self.assertEqual(len(sliced), 2)
        np.testing.assert_allclose(sliced[:, 2], [0.0, 1.2])

    def test_outdoor_backend_config_loads_cleanly(self) -> None:
        repo_root = Path(__file__).resolve().parents[2]
        cfg_path = repo_root / "configs" / "localization_mapping" / "slam_combined_lidar_ba_outdoor.json"
        cfg = BackendConfig.from_json(cfg_path)
        self.assertFalse(cfg.floor_plane_enabled)
        self.assertFalse(cfg.floor_normal_gravity_enabled)
        self.assertFalse(cfg.floor_gravity_world_calibration)
        self.assertIsNone(cfg.floor_normal_graph_sigma_rad)
        self.assertEqual(cfg.gravity_optimize_every, 0)
        self.assertTrue(cfg.planar_ba_enabled)
        self.assertEqual(cfg.planar_ba_global_every, 50)


if __name__ == "__main__":
    unittest.main()
