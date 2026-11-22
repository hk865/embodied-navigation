"""有界图像地点召回与实测 3D 支撑视觉几何校验单元测试。"""

from __future__ import annotations

from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe, read_keyframe, transform_points
from embodied_navigation.localization_mapping.backend.pose_graph import invert, pose_difference
from embodied_navigation.localization_mapping.backend.visual import (
    VisualObservation,
    decode_image,
    verify_visual,
)
from embodied_navigation.localization_mapping.backend.visual_retrieval import (
    VisualIndex,
    VisualRetrievalConfig,
    image_descriptor,
)

REPO_ROOT = Path(__file__).resolve().parents[2]


def _camera_extrinsics() -> tuple[np.ndarray, np.ndarray]:
    """返回测试用针孔相机内参 K 与机体到光学系外参 T_body_camera（x 前、y 左、z 上 -> x 右、y 下、z 前）。"""
    K = np.array([[120.0, 0.0, 64.0], [0.0, 120.0, 48.0], [0.0, 0.0, 1.0]], dtype=float)
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=float,
    )
    return K, T


def _textured_image(seed: int = 7, shape: tuple[int, int] = (96, 128)) -> np.ndarray:
    """生成高对比度、局部 patch 可区分的确定性灰度图。"""
    rng = np.random.default_rng(seed)
    h, w = shape
    yy, xx = np.meshgrid(np.linspace(0.0, 4.0 * np.pi, h), np.linspace(0.0, 5.0 * np.pi, w), indexing="ij")
    base = 128.0 + 45.0 * np.sin(xx) * np.cos(yy) + 35.0 * np.sin(0.5 * xx + 0.7 * yy)
    noise = rng.normal(0.0, 28.0, size=shape)
    return np.clip(base + noise, 0.0, 255.0)


def _scene_points(seed: int = 11) -> np.ndarray:
    """生成分布在机体前方不同深度与高度的非共面 3D 点云，附带平面墙体以支撑 ICP。"""
    rng = np.random.default_rng(seed)
    fy, fz = np.meshgrid(np.linspace(-2.0, 2.0, 25), np.linspace(-1.2, 1.2, 17))
    front_wall = np.column_stack([np.full(fy.size, 4.0), fy.ravel(), fz.ravel()])
    gx, gy = np.meshgrid(np.linspace(1.5, 4.5, 20), np.linspace(-2.0, 2.0, 25))
    floor = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, -1.2)])
    sx, sz = np.meshgrid(np.linspace(1.5, 4.5, 20), np.linspace(-1.2, 1.2, 17))
    side_wall = np.column_stack([sx.ravel(), np.full(sx.size, -2.0), sz.ravel()])
    scatter = np.column_stack(
        [
            rng.uniform(2.2, 5.5, 220),
            rng.uniform(-1.8, 1.8, 220),
            rng.uniform(-1.0, 1.0, 220),
        ]
    )
    return np.vstack([front_wall, floor, side_wall, scatter])


def _render_projected_image(
    points_body: np.ndarray,
    T_body_camera: np.ndarray,
    K: np.ndarray,
    world_texture: np.ndarray,
    T_world_body: np.ndarray,
) -> np.ndarray:
    """用固定世界坐标到像素的映射构造多视角一致的灰度图（仅对小位姿扰动做精确投影渲染）。"""
    h, w = world_texture.shape
    if np.allclose(T_world_body, np.eye(4)):
        return world_texture.copy()
    T_cam_ref_cur = invert(T_body_camera) @ T_world_body @ T_body_camera
    K_inv = np.linalg.inv(K)
    uu, vv = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
    rays = np.stack([uu, vv, np.ones_like(uu)], axis=-1).reshape(-1, 3) @ K_inv.T
    pts_cur = rays * (4.0 / np.maximum(rays[:, 2:3], 1e-6))
    pts_ref = transform_points(T_cam_ref_cur, pts_cur)
    proj = pts_ref @ K.T
    u_ref = np.rint(proj[:, 0] / np.maximum(proj[:, 2], 1e-6)).astype(int)
    v_ref = np.rint(proj[:, 1] / np.maximum(proj[:, 2], 1e-6)).astype(int)
    valid = (u_ref >= 0) & (u_ref < w) & (v_ref >= 0) & (v_ref < h)
    out = np.full(h * w, 128.0, dtype=float)
    out[valid] = world_texture[v_ref[valid], u_ref[valid]]
    return out.reshape(h, w)


class VisualRetrievalUnitTest(unittest.TestCase):
    def test_descriptor_filters_low_contrast_stale_and_evicts_fifo(self) -> None:
        """描述子应拒绝低对比度与时间超限帧，VisualIndex 按最老条目淘汰。"""
        K, T_bc = _camera_extrinsics()
        img = _textured_image()
        pts = _scene_points()

        def _kf(seq: int, gray: np.ndarray, vis_offset_ns: int = 0) -> Keyframe:
            stamp = seq * 1_000_000_000
            return Keyframe(
                key=(0, seq),
                stamp_ns=stamp,
                T_odom_body=np.eye(4),
                T_body_lidar=np.eye(4),
                points_lidar=pts,
                visual=VisualObservation(gray=gray, K=K, T_body_camera=T_bc, stamp_ns=stamp + vis_offset_ns),
            )

        self.assertIsNone(image_descriptor(_kf(0, np.full((64, 64), 100.0))))
        self.assertIsNone(image_descriptor(_kf(1, img, vis_offset_ns=100_000_000)))

        d1 = image_descriptor(_kf(2, img))
        d2 = image_descriptor(_kf(3, img * 0.5 + 30.0))
        self.assertIsNotNone(d1)
        self.assertIsNotNone(d2)
        self.assertAlmostEqual(float(d1 @ d2), 1.0, places=6)

        idx = VisualIndex(VisualRetrievalConfig(max_entries=2, candidates=3, max_distance=0.25))
        idx.add(_kf(1, img))
        idx.add(_kf(2, _textured_image(seed=19)))
        idx.add(_kf(3, _textured_image(seed=23)))
        self.assertEqual(list(idx.descriptors.keys()), [(0, 2), (0, 3)])

    def test_decode_image_and_observation_validation(self) -> None:
        """decode_image 应支持带行填充的 mono8/rgb8/bgr8，并拒绝非法 stride/编码。"""
        row0 = bytes([10, 20, 30, 40, 50, 60, 70, 80, 90, 0, 0, 0])
        row1 = bytes([90, 80, 70, 60, 50, 40, 30, 20, 10, 0, 0, 0])
        msg = SimpleNamespace(encoding="bgr8", width=3, height=2, step=12, data=row0 + row1)
        gray = decode_image(msg)
        self.assertEqual(gray.shape, (2, 3))
        self.assertAlmostEqual(gray[0, 0], 30 * 0.299 + 20 * 0.587 + 10 * 0.114, places=5)

        with self.assertRaises(ValueError):
            decode_image(SimpleNamespace(encoding="16UC1", width=2, height=2, step=4, data=b"\x00" * 8))
        with self.assertRaises(ValueError):
            decode_image(SimpleNamespace(encoding="mono8", width=4, height=2, step=2, data=b"\x00" * 4))

        K, T_bc = _camera_extrinsics()
        bad_K = K.copy()
        bad_K[0, 0] = -1.0
        with self.assertRaises(ValueError):
            VisualObservation(gray=_textured_image(), K=bad_K, T_body_camera=T_bc, stamp_ns=0)

    def test_verify_visual_recovers_rigid_transform_and_rejects_bad_inputs(self) -> None:
        """对前方实测 3D 支撑点做小刚体变换后，verify_visual 应精确恢复相对位姿。"""
        K, T_bc = _camera_extrinsics()
        img_ref = _textured_image(seed=42)
        pts_ref = _scene_points(seed=13)
        T_target_source = np.eye(4)
        T_target_source[:3, 3] = [0.0, 0.08, 0.0]
        pts_src = transform_points(invert(T_target_source), pts_ref)
        img_src = _render_projected_image(pts_src, T_bc, K, img_ref, T_target_source)

        kf_target = Keyframe(
            key=(0, 0),
            stamp_ns=1_000_000_000,
            T_odom_body=np.eye(4),
            T_body_lidar=np.eye(4),
            points_lidar=pts_ref,
            visual=VisualObservation(gray=img_ref, K=K, T_body_camera=T_bc, stamp_ns=1_000_000_000),
        )
        kf_source = Keyframe(
            key=(0, 1),
            stamp_ns=2_000_000_000,
            T_odom_body=T_target_source,
            T_body_lidar=np.eye(4),
            points_lidar=pts_src,
            visual=VisualObservation(gray=img_src, K=K, T_body_camera=T_bc, stamp_ns=2_000_000_000),
        )

        res = verify_visual(kf_source, kf_target)
        self.assertEqual(res.status, "verified")
        self.assertIsNotNone(res.T_target_source)
        dt, dr = pose_difference(res.T_target_source, T_target_source)
        self.assertLess(dt, 0.05)
        self.assertLess(dr, 0.02)

        kf_no_vis = Keyframe(
            key=(0, 2),
            stamp_ns=3_000_000_000,
            T_odom_body=np.eye(4),
            T_body_lidar=np.eye(4),
            points_lidar=pts_ref,
            visual=None,
        )
        self.assertEqual(verify_visual(kf_no_vis, kf_target).status, "missing_observation")
        kf_stale = Keyframe(
            key=(0, 3),
            stamp_ns=3_000_000_000,
            T_odom_body=np.eye(4),
            T_body_lidar=np.eye(4),
            points_lidar=pts_ref,
            visual=VisualObservation(gray=img_ref, K=K, T_body_camera=T_bc, stamp_ns=3_500_000_000),
        )
        self.assertEqual(verify_visual(kf_stale, kf_target).status, "time_mismatch")

    def test_backend_config_strict_validation_and_json_loading(self) -> None:
        """配置加载应严格校验 visual_retrieval 嵌套键与布尔开关。"""
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"use_visual_retrieval": 1})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"visual_retrieval": {"unknown_key": 3}})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"visual_retrieval": {"max_distance": 3.0}})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"visual_agree_translation_m": -0.1})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"visual_retrieval": {"feature_type": "invalid_feat"}})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"visual_retrieval": {"verify_ratio_threshold": 1.0}})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"visual_retrieval": {"lidar_px_radius": 0.0}})

        cfg_path = REPO_ROOT / "configs" / "localization_mapping" / "slam_v3_hybrid_color_visual.json"
        cfg = BackendConfig.from_json(cfg_path)
        self.assertTrue(cfg.use_visual_retrieval)
        self.assertFalse(cfg.use_visual_verification)
        self.assertEqual(cfg.visual_retrieval.feature_type, "akaze")
        self.assertAlmostEqual(cfg.loop_sigma_rad, 0.03)
        self.assertAlmostEqual(cfg.loop_sigma_yaw_rad, 0.01)
        self.assertAlmostEqual(cfg.cauchy_scale, 3.0)

    def test_slam_backend_visual_additive_proposal_and_fallback(self) -> None:
        """验证视觉召回独立增补能力以及可选验证失败时回退到合格 ScanContext 候选。"""
        K, T_bc = _camera_extrinsics()
        img_ref = _textured_image(seed=99)
        pts_ref = _scene_points(seed=17)

        cfg_visual_only = BackendConfig(
            use_visual_retrieval=True,
            use_visual_verification=False,
            max_scan_context_distance=1e-6,
            map_voxel_m=0.05,
            icp_voxel_m=0.1,
            min_loop_gap_s=1.0,
            exclude_recent=1,
            loop_submap_neighbors=0,
            min_fitness=0.5,
            max_rmse_m=0.15,
        )
        backend_a = SlamBackend(cfg_visual_only)
        T_mid = np.eye(4)
        T_mid[0, 3] = 1.0
        T_ret = np.eye(4)
        T_ret[1, 3] = 0.04
        pts_ret = transform_points(invert(T_ret), pts_ref)
        img_ret = _render_projected_image(pts_ret, T_bc, K, img_ref, T_ret)

        backend_a.ingest(
            Keyframe(
                key=(0, 0),
                stamp_ns=0,
                T_odom_body=np.eye(4),
                T_body_lidar=np.eye(4),
                points_lidar=pts_ref,
                visual=VisualObservation(gray=img_ref, K=K, T_body_camera=T_bc, stamp_ns=0),
            )
        )
        backend_a.ingest(
            Keyframe(
                key=(0, 1),
                stamp_ns=2_000_000_000,
                T_odom_body=T_mid,
                T_body_lidar=np.eye(4),
                points_lidar=transform_points(invert(T_mid), pts_ref),
                visual=VisualObservation(gray=_textured_image(seed=101), K=K, T_body_camera=T_bc, stamp_ns=2_000_000_000),
            )
        )
        ev_a = backend_a.ingest(
            Keyframe(
                key=(0, 2),
                stamp_ns=4_000_000_000,
                T_odom_body=T_ret,
                T_body_lidar=np.eye(4),
                points_lidar=pts_ret,
                visual=VisualObservation(gray=img_ret, K=K, T_body_camera=T_bc, stamp_ns=4_000_000_000),
            )
        )
        self.assertTrue(ev_a["loop"])
        self.assertGreaterEqual(backend_a.visual_candidates_proposed, 1)
        self.assertGreaterEqual(backend_a.visual_verified_count, 1)
        self.assertEqual(backend_a.visual_loops_accepted, 1)

        cfg_fallback = BackendConfig(
            use_visual_retrieval=True,
            use_visual_verification=False,
            map_voxel_m=0.05,
            icp_voxel_m=0.1,
            min_loop_gap_s=1.0,
            exclude_recent=1,
            loop_submap_neighbors=0,
            min_fitness=0.5,
            max_rmse_m=0.15,
        )
        backend_b = SlamBackend(cfg_fallback)
        img_blur = np.roll(img_ref, shift=20, axis=1)
        h, w = 96, 128
        grad = np.zeros((h, w), dtype=float)
        grad[:, w // 2 :] = 200.0
        backend_b.ingest(
            Keyframe(
                key=(0, 0),
                stamp_ns=0,
                T_odom_body=np.eye(4),
                T_body_lidar=np.eye(4),
                points_lidar=pts_ref,
                visual=VisualObservation(gray=grad, K=K, T_body_camera=T_bc, stamp_ns=0),
            )
        )
        backend_b.ingest(
            Keyframe(
                key=(0, 1),
                stamp_ns=2_000_000_000,
                T_odom_body=T_mid,
                T_body_lidar=np.eye(4),
                points_lidar=transform_points(invert(T_mid), pts_ref),
                visual=VisualObservation(gray=img_blur, K=K, T_body_camera=T_bc, stamp_ns=2_000_000_000),
            )
        )
        ev_b = backend_b.ingest(
            Keyframe(
                key=(0, 2),
                stamp_ns=4_000_000_000,
                T_odom_body=np.eye(4),
                T_body_lidar=np.eye(4),
                points_lidar=pts_ref,
                visual=VisualObservation(gray=grad, K=K, T_body_camera=T_bc, stamp_ns=4_000_000_000),
            )
        )
        self.assertTrue(ev_b["loop"])
        self.assertGreaterEqual(backend_b.visual_fallback_count, 1)
        self.assertEqual(backend_b.visual_loops_accepted, 0)

    def test_visual_disagreement_does_not_drop_eligible_lidar_candidate(self) -> None:
        """双通道命中同一键且视觉验证成功但与 ICP 不一致时，绝不能丢弃原合格 ScanContext 候选。"""
        K, T_bc = _camera_extrinsics()
        img_ref = _textured_image(seed=99)
        pts_ref = _scene_points(seed=17)
        T_mid = np.eye(4)
        T_mid[0, 3] = 1.0
        T_ret = np.eye(4)
        T_ret[1, 3] = 0.04
        pts_ret = transform_points(invert(T_ret), pts_ref)
        img_ret = _render_projected_image(pts_ret, T_bc, K, img_ref, T_ret)

        cfg = BackendConfig(
            use_visual_retrieval=True,
            use_visual_verification=False,
            visual_agree_translation_m=1e-9,
            visual_agree_rotation_rad=1e-9,
            map_voxel_m=0.05,
            icp_voxel_m=0.1,
            min_loop_gap_s=1.0,
            exclude_recent=1,
            loop_submap_neighbors=0,
            min_fitness=0.5,
            max_rmse_m=0.15,
        )
        backend = SlamBackend(cfg)
        backend.ingest(
            Keyframe(
                key=(0, 0),
                stamp_ns=0,
                T_odom_body=np.eye(4),
                T_body_lidar=np.eye(4),
                points_lidar=pts_ref,
                visual=VisualObservation(gray=img_ref, K=K, T_body_camera=T_bc, stamp_ns=0),
            )
        )
        backend.ingest(
            Keyframe(
                key=(0, 1),
                stamp_ns=2_000_000_000,
                T_odom_body=T_mid,
                T_body_lidar=np.eye(4),
                points_lidar=transform_points(invert(T_mid), pts_ref),
                visual=VisualObservation(gray=_textured_image(seed=101), K=K, T_body_camera=T_bc, stamp_ns=2_000_000_000),
            )
        )
        ev = backend.ingest(
            Keyframe(
                key=(0, 2),
                stamp_ns=4_000_000_000,
                T_odom_body=T_ret,
                T_body_lidar=np.eye(4),
                points_lidar=pts_ret,
                visual=VisualObservation(gray=img_ret, K=K, T_body_camera=T_bc, stamp_ns=4_000_000_000),
            )
        )
        self.assertTrue(ev["loop"])
        self.assertGreaterEqual(backend.visual_disagreement_count, 1)
        self.assertEqual(backend.visual_loops_accepted, 0)

    def test_read_keyframe_ignores_corrupt_visual_npz_by_default_and_checks_K_row(self) -> None:
        """read_keyframe 默认不触碰 visual.npz（零侵入纯 LiDAR 读取），开启 load_visual 时才校验。"""
        K, T_bc = _camera_extrinsics()
        bad_K = K.copy()
        bad_K[2] = [0.0, 1.0, 1.0]
        with self.assertRaises(ValueError):
            VisualObservation(gray=_textured_image(), K=bad_K, T_body_camera=T_bc, stamp_ns=0)

        with tempfile.TemporaryDirectory() as tmp:
            kf_dir = Path(tmp) / "kf_1_1"
            kf_dir.mkdir()
            (kf_dir / "metadata.json").write_text(
                '{"schema":"livo_keyframe/v1","epoch":1,"sequence":1,"stamp_ns":1000,'
                '"T_odom_body":[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],'
                '"T_body_lidar":[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]}',
                encoding="utf-8",
            )
            (kf_dir / "points_lidar.pcd").write_text("VERSION .7\nDATA ascii\n1.0 0.0 0.0\n", encoding="utf-8")
            (kf_dir / "visual.npz").write_bytes(b"corrupted-not-a-zip")

            kf_default = read_keyframe(kf_dir)
            self.assertIsNone(kf_default.visual)
            with self.assertRaises(ValueError):
                read_keyframe(kf_dir, load_visual=True)

    def test_viewpoint_invariant_features_verify_rotated_viewpoint(self) -> None:
        """ORB/AKAZE/BRISK/SIFT 视角不变特征应在偏航旋转视角下成功召回并恢复精确 SE(3) 位姿。"""
        try:
            import cv2  # noqa: F401
        except ImportError:
            self.skipTest("当前环境未安装 OpenCV (cv2)")

        h, w = 256, 256
        K = np.array([[220.0, 0.0, 128.0], [0.0, 220.0, 128.0], [0.0, 0.0, 1.0]], dtype=float)
        _, T_bc = _camera_extrinsics()
        rng = np.random.default_rng(2026)
        img_ref = np.full((h, w), 120.0, dtype=float)
        for _ in range(180):
            cx, cy = int(rng.integers(16, w - 16)), int(rng.integers(16, h - 16))
            rx, ry = int(rng.integers(4, 14)), int(rng.integers(4, 14))
            val = float(rng.choice([15.0, 240.0]))
            img_ref[cy - ry : cy + ry, cx - rx : cx + rx] = val

        fy, fz = np.meshgrid(np.linspace(-2.5, 2.5, 65), np.linspace(-2.5, 2.5, 65))
        pts_ref = np.column_stack([np.full(fy.size, 4.0), fy.ravel(), fz.ravel()])

        yaw = float(np.deg2rad(12.0))
        T_target_source = np.eye(4, dtype=float)
        T_target_source[:3, :3] = [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
        T_target_source[:3, 3] = [0.05, 0.12, 0.0]
        pts_src = transform_points(invert(T_target_source), pts_ref)
        img_src = _render_projected_image(pts_src, T_bc, K, img_ref, T_target_source)

        kf_target = Keyframe(
            key=(0, 0),
            stamp_ns=1_000_000_000,
            T_odom_body=np.eye(4),
            T_body_lidar=np.eye(4),
            points_lidar=pts_ref,
            visual=VisualObservation(gray=img_ref, K=K, T_body_camera=T_bc, stamp_ns=1_000_000_000),
        )
        kf_source = Keyframe(
            key=(0, 1),
            stamp_ns=2_000_000_000,
            T_odom_body=T_target_source,
            T_body_lidar=np.eye(4),
            points_lidar=pts_src,
            visual=VisualObservation(gray=img_src, K=K, T_body_camera=T_bc, stamp_ns=2_000_000_000),
        )

        for feat in ("orb", "akaze", "brisk", "sift"):
            with self.subTest(feature_type=feat):
                vcfg = VisualRetrievalConfig(feature_type=feat, max_distance=0.6)
                idx = VisualIndex(vcfg)
                idx.add(kf_target)
                hits = idx.query(kf_source)
                self.assertTrue(hits and hits[0][0] == (0, 0))
                res = verify_visual(
                    kf_source,
                    kf_target,
                    feature_type=feat,
                    inlier_m=0.15,
                    min_inliers=8,
                    ratio_threshold=0.78,
                    target_features=idx.local_features.get((0, 0)),
                )
                self.assertEqual(res.status, "verified")
                dt, dr = pose_difference(res.T_target_source, T_target_source)
                self.assertLess(dt, 0.08)
                self.assertLess(dr, 0.03)


if __name__ == "__main__":
    unittest.main()
