"""v3 混合视觉+Color-ScanContext+航向门控单元测试。"""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.color_scan_context import (
    ColorScanContextIndex,
    color_descriptor,
    color_sc_distance,
)
from embodied_navigation.localization_mapping.backend.evaluation import (
    load_input_json_keyframes,
    load_map_keyframes,
)
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe, read_keyframe, transform_points
from embodied_navigation.localization_mapping.backend.map_store import MapStore
from embodied_navigation.localization_mapping.backend.pose_graph import invert
from embodied_navigation.localization_mapping.backend.scan_context import ScanContextConfig, descriptor
from embodied_navigation.localization_mapping.backend.visual import VisualObservation
from embodied_navigation.localization_mapping.backend.visual_retrieval import (
    VisualIndex,
    VisualRetrievalConfig,
)

REPO_ROOT = Path(__file__).resolve().parents[2]


def _camera_extrinsics() -> tuple[np.ndarray, np.ndarray]:
    """机体系 (x前, y左, z上) -> 光学系 (x右, y下, z前) 相机参数。"""
    K = np.array([[160.0, 0.0, 64.0], [0.0, 160.0, 48.0], [0.0, 0.0, 1.0]], dtype=float)
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


def _yaw_transform(yaw_rad: float, xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4, dtype=float)
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = xyz
    return T


def _textured_image(seed: int = 7, shape: tuple[int, int] = (96, 128)) -> np.ndarray:
    rng = np.random.default_rng(seed)
    h, w = shape
    yy, xx = np.meshgrid(np.linspace(0.0, 4.0 * np.pi, h), np.linspace(0.0, 5.0 * np.pi, w), indexing="ij")
    base = 128.0 + 45.0 * np.sin(xx) * np.cos(yy) + 35.0 * np.sin(0.5 * xx + 0.7 * yy)
    noise = rng.normal(0.0, 26.0, size=shape)
    return np.clip(base + noise, 0.0, 255.0)


def _asymmetric_room_points(seed: int = 11) -> np.ndarray:
    """生成前/后/左/右非对称的 3D 房间点云，保证 360° ScanContext 无 180° 歧义且支撑 ICP。"""
    rng = np.random.default_rng(seed)
    # 近前方墙 (x=4.0)、远后方墙 (x=-7.5)、近左墙 (y=2.2)、远右墙 (y=-4.5) + 地面
    fy, fz = np.meshgrid(np.linspace(-2.5, 2.0, 24), np.linspace(-1.2, 1.2, 16))
    front_wall = np.column_stack([np.full(fy.size, 4.0), fy.ravel(), fz.ravel()])
    by, bz = np.meshgrid(np.linspace(-3.0, 2.0, 24), np.linspace(-1.2, 1.2, 16))
    back_wall = np.column_stack([np.full(by.size, -7.5), by.ravel(), bz.ravel()])
    lx, lz = np.meshgrid(np.linspace(-6.0, 4.0, 30), np.linspace(-1.2, 1.2, 16))
    left_wall = np.column_stack([lx.ravel(), np.full(lx.size, 2.2), lz.ravel()])
    rx, rz = np.meshgrid(np.linspace(-6.0, 4.0, 30), np.linspace(-1.2, 1.2, 16))
    right_wall = np.column_stack([rx.ravel(), np.full(rx.size, -4.5), rz.ravel()])
    gx, gy = np.meshgrid(np.linspace(-6.0, 4.0, 30), np.linspace(-4.0, 2.0, 24))
    floor = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, -1.2)])
    # 前方特征立柱与后方不同高度结构，增强 3D 非退化性与正反航向可分性
    pillar = np.column_stack(
        [
            rng.uniform(2.2, 3.6, 180),
            rng.uniform(-1.2, 1.2, 180),
            rng.uniform(-1.0, 1.0, 180),
        ]
    )
    return np.vstack([front_wall, back_wall, left_wall, right_wall, floor, pillar])


class HybridColorVisualBackendTests(unittest.TestCase):
    def test_color_scan_context_distinguishes_rgb_and_handles_non_overlapping_fov(self) -> None:
        """ColorScanContext 应区分几何完全一致但 RGB 不同的场景，且视野不重叠时返回 None。"""
        K, T_bc = _camera_extrinsics()
        sc_cfg = ScanContextConfig(rings=20, sectors=60, max_range_m=20.0)
        # 在前方多个距离与方位角布置丰富 LiDAR 点以填充 >=12 个极坐标栅格
        xs, ys, zs = np.meshgrid(
            np.linspace(2.0, 8.0, 12),
            np.linspace(-2.5, 2.5, 16),
            np.linspace(-0.8, 0.8, 4),
        )
        pts = np.column_stack([xs.ravel(), ys.ravel(), zs.ravel()])

        # 构造亮度几乎相等（~85）但色相截然相反的两张 RGB 图像（红 vs 蓝绿）
        rgb_red = np.zeros((96, 128, 3), dtype=np.uint8)
        rgb_red[:, :] = [210, 30, 30]
        rgb_blue = np.zeros((96, 128, 3), dtype=np.uint8)
        rgb_blue[:, :] = [30, 110, 220]
        gray_same = np.full((96, 128), 85.0, dtype=float)

        obs_red = VisualObservation(gray=gray_same, K=K, T_body_camera=T_bc, stamp_ns=0, rgb=rgb_red)
        obs_red2 = VisualObservation(gray=gray_same, K=K, T_body_camera=T_bc, stamp_ns=0, rgb=rgb_red.copy())
        obs_blue = VisualObservation(gray=gray_same, K=K, T_body_camera=T_bc, stamp_ns=0, rgb=rgb_blue)

        d_red1 = color_descriptor(pts, obs_red, sc_cfg)
        d_red2 = color_descriptor(pts, obs_red2, sc_cfg)
        d_blue = color_descriptor(pts, obs_blue, sc_cfg)
        self.assertIsNotNone(d_red1)
        self.assertIsNotNone(d_blue)
        self.assertGreaterEqual(int(d_red1.mask.sum()), 12)

        dist_same = color_sc_distance(d_red1, d_red2, shift=0, min_color_bins=12)
        dist_diff = color_sc_distance(d_red1, d_blue, shift=0, min_color_bins=12)
        self.assertIsNotNone(dist_same)
        self.assertIsNotNone(dist_diff)
        self.assertAlmostEqual(dist_same, 0.0, places=6)
        self.assertGreater(dist_diff, 0.5)

        # 反向航向（180° = shift=30）：前向相机视野完全无重叠，必须返回 None 不误罚
        dist_reverse = color_sc_distance(d_red1, d_red2, shift=30, min_color_bins=12)
        self.assertIsNone(dist_reverse)

        # ColorScanContextIndex 重排测试：几何距离相同，Color-SC 应把同色候选排在最前并拒绝异色候选
        cidx = ColorScanContextIndex(sc_cfg, min_color_bins=12, color_weight=0.25, max_color_distance=0.4)
        cidx.add((0, 0), d_blue)
        cidx.add((0, 1), d_red2)
        ranked, rescored, rejected = cidx.rescore(
            d_red1,
            [((0, 0), 0.10, 0.0), ((0, 1), 0.12, 0.0), ((0, 2), 0.15, np.pi)],
        )
        self.assertEqual(rescored, 2)
        self.assertEqual(rejected, 1)
        self.assertEqual([row[0] for row in ranked], [(0, 1), (0, 2)])

    def test_visual_observation_rgb_serialization_and_gray_backward_compat(self) -> None:
        """VisualObservation 与各磁盘加载器应自动加载可选 rgb，且兼容纯灰度缓存。"""
        K, T_bc = _camera_extrinsics()
        gray = _textured_image(seed=5)
        rgb = np.stack([np.clip(gray, 0, 255), np.full_like(gray, 100), np.full_like(gray, 50)], axis=-1).astype(
            np.uint8
        )
        pts = _asymmetric_room_points(seed=3)

        with self.assertRaises(ValueError):
            VisualObservation(gray=gray, K=K, T_body_camera=T_bc, stamp_ns=0, rgb=rgb.astype(np.float32))
        with self.assertRaises(ValueError):
            VisualObservation(gray=gray, K=K, T_body_camera=T_bc, stamp_ns=0, rgb=rgb[:, :10, :])

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            # 1) MapStore 往返（一帧带 rgb，一帧仅灰度）
            store = MapStore(voxel_m=0.2)
            kf0 = Keyframe(
                key=(0, 0),
                stamp_ns=100,
                T_odom_body=np.eye(4),
                T_body_lidar=np.eye(4),
                points_lidar=pts,
                visual=VisualObservation(gray=gray, K=K, T_body_camera=T_bc, stamp_ns=100, rgb=rgb),
            )
            kf1 = Keyframe(
                key=(0, 1),
                stamp_ns=200,
                T_odom_body=np.eye(4),
                T_body_lidar=np.eye(4),
                points_lidar=pts,
                visual=VisualObservation(gray=gray, K=K, T_body_camera=T_bc, stamp_ns=200),
            )
            store.add(kf0, np.eye(4), np.zeros((20, 60)))
            store.add(kf1, np.eye(4), np.zeros((20, 60)))
            map_dir = root / "map"
            store.save(map_dir)
            loaded = load_map_keyframes(map_dir)
            self.assertIsNotNone(loaded[0].visual.rgb)
            np.testing.assert_array_equal(loaded[0].visual.rgb, rgb)
            self.assertIsNone(loaded[1].visual.rgb)

            # 2) evaluation visual_dir (.npz 带 rgb 与纯灰度)
            vdir = root / "visual_cache"
            vdir.mkdir()
            np.savez(vdir / "100.npz", gray=gray, rgb=rgb, K=K, T_body_camera=T_bc, stamp_ns=np.int64(100))
            np.savez(vdir / "200.npz", gray=gray, K=K, T_body_camera=T_bc, stamp_ns=np.int64(200))
            loaded_vdir = load_map_keyframes(map_dir, visual_dir=vdir)
            self.assertIsNotNone(loaded_vdir[0].visual.rgb)
            self.assertIsNone(loaded_vdir[1].visual.rgb)

            # 3) input.json manifest 加载
            pcd_path = root / "pts.pcd"
            pcd_lines = ["VERSION .7", "FIELDS x y z", "DATA ascii"] + [
                f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}" for p in pts
            ]
            pcd_path.write_text("\n".join(pcd_lines) + "\n", encoding="utf-8")
            manifest = root / "input.json"
            manifest.write_text(
                json.dumps(
                    [
                        {
                            "key": [0, 0],
                            "stamp_ns": 100,
                            "T_odom_body": np.eye(4).tolist(),
                            "T_body_lidar": np.eye(4).tolist(),
                            "points": "pts.pcd",
                        }
                    ]
                ),
                encoding="utf-8",
            )
            kfs_json = load_input_json_keyframes(manifest, visual_dir=vdir)
            self.assertIsNotNone(kfs_json[0].visual.rgb)
            np.testing.assert_array_equal(kfs_json[0].visual.rgb, rgb)

            # 4) read_keyframe spool 旁文件
            spool_kf = root / "kf_0_0"
            spool_kf.mkdir()
            (spool_kf / "metadata.json").write_text(
                json.dumps(
                    {
                        "schema": "livo_keyframe/v1",
                        "epoch": 0,
                        "sequence": 0,
                        "stamp_ns": 100,
                        "T_odom_body": np.eye(4).tolist(),
                        "T_body_lidar": np.eye(4).tolist(),
                    }
                ),
                encoding="utf-8",
            )
            (spool_kf / "points_lidar.pcd").write_text(pcd_path.read_text(encoding="utf-8"), encoding="utf-8")
            np.savez(spool_kf / "visual.npz", gray=gray, rgb=rgb, K=K, T_body_camera=T_bc, stamp_ns=np.int64(100))
            kf_spool = read_keyframe(spool_kf, load_visual=True)
            self.assertIsNotNone(kf_spool.visual.rgb)

    def test_multiview_window_builds_denser_cloud_and_picks_best_view(self) -> None:
        """visual_multiview_radius>0 应聚合邻域点云用于深度绑定，并从局部窗口挑选最佳视野。"""
        K, T_bc = _camera_extrinsics()
        pts_full = _asymmetric_room_points(seed=21)
        img_match = _textured_image(seed=77)
        img_other = _textured_image(seed=88)

        # 拆成左右两部分点云：k1 仅有左侧前方点（不足以单独支撑右侧大量视觉特征），
        # k0/k2 补全右侧与近景结构，只有多帧融合才能满足 min_inliers。
        pts_left = pts_full[pts_full[:, 1] >= 0.3]
        pts_right = pts_full[pts_full[:, 1] < 0.3]
        # 保证每帧自身仍够 ICP 下限 50 点，但视觉视野内 (x>0, 7x7栅格) 仅单帧很稀疏
        pts_k1 = np.vstack([pts_full[pts_full[:, 0] < 0.0], pts_left[:25]])
        pts_k0 = np.vstack([pts_full[pts_full[:, 0] < 0.0], pts_right])
        pts_k2 = np.vstack([pts_full[pts_full[:, 0] < 0.0], pts_left[25:]])

        def _make_backend(mv_radius: int) -> SlamBackend:
            cfg = BackendConfig(
                use_visual_retrieval=True,
                use_visual_verification=True,
                visual_multiview_radius=mv_radius,
                visual_retrieval=VisualRetrievalConfig(
                    feature_type="patch7x7",
                    max_distance=0.35,
                    verify_min_inliers=12,
                ),
                max_scan_context_distance=1e-6,  # 仅通过视觉检索触发，直接检验多视图视野挑选与稠密深度
                map_voxel_m=0.05,
                icp_voxel_m=0.1,
                min_loop_gap_s=1.0,
                exclude_recent=1,
                loop_submap_neighbors=1,
                min_fitness=0.5,
                max_rmse_m=0.15,
            )
            b = SlamBackend(cfg)
            T0 = _yaw_transform(0.0, (0.0, 0.0, 0.0))
            T1 = _yaw_transform(0.0, (0.01, 0.0, 0.0))
            T2 = _yaw_transform(0.0, (0.02, 0.0, 0.0))
            T_away = _yaw_transform(0.0, (4.0, 0.0, 0.0))
            T_ret = _yaw_transform(0.0, (0.01, 0.0, 0.0))

            b.ingest(
                Keyframe(
                    (0, 0),
                    0,
                    T0,
                    np.eye(4),
                    transform_points(invert(T0), pts_k0),
                    visual=VisualObservation(img_other, K, T_bc, 0),
                )
            )
            b.ingest(
                Keyframe(
                    (0, 1),
                    1_000_000_000,
                    T1,
                    np.eye(4),
                    transform_points(invert(T1), pts_k1),
                    visual=VisualObservation(img_match, K, T_bc, 1_000_000_000),
                )
            )
            b.ingest(
                Keyframe(
                    (0, 2),
                    2_000_000_000,
                    T2,
                    np.eye(4),
                    transform_points(invert(T2), pts_k2),
                    visual=VisualObservation(img_other, K, T_bc, 2_000_000_000),
                )
            )
            b.ingest(
                Keyframe(
                    (0, 3),
                    3_000_000_000,
                    T_away,
                    np.eye(4),
                    transform_points(invert(T_away), pts_full),
                    visual=VisualObservation(_textured_image(seed=105), K, T_bc, 3_000_000_000),
                )
            )
            ev = b.ingest(
                Keyframe(
                    (0, 4),
                    5_000_000_000,
                    T_ret,
                    np.eye(4),
                    transform_points(invert(T_ret), pts_full),
                    visual=VisualObservation(img_match, K, T_bc, 5_000_000_000),
                )
            )
            return b, ev

        b_single, ev_single = _make_backend(mv_radius=0)
        b_multi, ev_multi = _make_backend(mv_radius=1)
        self.assertFalse(ev_single["loop"])
        self.assertTrue(ev_multi["loop"])
        self.assertEqual(b_multi.visual_loops_accepted, 1)

    def test_seq_consistency_rejects_isolated_hit_and_confirms_sequence(self) -> None:
        """visual_seq_consistency>=2 应拦截孤立单帧匹配，并在相邻帧一致支持时放行与加权。"""
        K, T_bc = _camera_extrinsics()
        pts = _asymmetric_room_points(seed=31)
        vcfg = VisualRetrievalConfig(feature_type="patch7x7", candidates=3, max_distance=0.35)
        idx = VisualIndex(vcfg, seq_consistency=2)

        rng = np.random.default_rng(31)
        imgs = [rng.uniform(0.0, 255.0, size=(96, 128)) for _ in range(10)]
        for i, im in enumerate(imgs):
            idx.add(
                Keyframe(
                    (0, i),
                    i * 1_000_000_000,
                    np.eye(4),
                    np.eye(4),
                    pts,
                    visual=VisualObservation(im, K, T_bc, i * 1_000_000_000),
                )
            )

        # 首帧查询历史不足，返回空
        q0 = Keyframe((0, 10), 10_000_000_000, np.eye(4), np.eye(4), pts, VisualObservation(imgs[1], K, T_bc, 10_000_000_000))
        self.assertEqual(idx.query(q0), [])

        # 第2帧查询命中远离第1帧的 (0, 8)，应被时序一致性门拒绝
        q1_jump = Keyframe((0, 11), 11_000_000_000, np.eye(4), np.eye(4), pts, VisualObservation(imgs[8], K, T_bc, 11_000_000_000))
        self.assertEqual(idx.query(q1_jump), [])
        self.assertGreaterEqual(idx.seq_rejected_count, 1)

        # 第3帧查询命中与上一帧 (0, 8) 相邻的 (0, 9)，应确认通过且带 0.75 分数增益
        q2_seq = Keyframe((0, 12), 12_000_000_000, np.eye(4), np.eye(4), pts, VisualObservation(imgs[9], K, T_bc, 12_000_000_000))
        hits = idx.query(q2_seq)
        self.assertEqual(len(hits), 1)
        self.assertEqual(hits[0][0], (0, 9))
        self.assertAlmostEqual(hits[0][1], 0.0, places=6)

    def test_heading_gated_hybrid_admission_same_vs_reverse_heading(self) -> None:
        """<65° 同向重访强制视觉几何校验；>=65° 反向重访走 360° LiDAR ScanContext + ICP。"""
        K, T_bc = _camera_extrinsics()
        pts_world = _asymmetric_room_points(seed=41)
        img_fwd = _textured_image(seed=301)
        img_rev = _textured_image(seed=302)  # 反向相机看到完全不同的画面

        def _make_cfg() -> BackendConfig:
            return BackendConfig(
                use_visual_retrieval=True,
                use_visual_verification=False,
                hybrid_heading_gate_deg=65.0,
                color_sc_enabled=True,
                color_sc_weight=0.25,
                color_sc_min_bins=12,
                fine_icp_voxel_m=0.12,
                visual_retrieval=VisualRetrievalConfig(feature_type="patch7x7", max_distance=0.35),
                map_voxel_m=0.1,
                icp_voxel_m=0.15,
                min_loop_gap_s=1.0,
                exclude_recent=1,
                loop_submap_neighbors=0,
                min_fitness=0.55,
                max_rmse_m=0.12,
            )

        # 场景 A：同向重访（|Δyaw|=0° < 65°）若视觉无纹理，必须拒绝而非放行纯 LiDAR SC
        b_same_reject = SlamBackend(_make_cfg())
        T0 = np.eye(4)
        T1 = _yaw_transform(0.0, (1.5, 0.0, 0.0))
        T_ret_same = _yaw_transform(0.0, (0.05, 0.02, 0.0))
        flat_img = np.full((96, 128), 120.0, dtype=float)
        b_same_reject.ingest(
            Keyframe((0, 0), 0, T0, np.eye(4), pts_world, VisualObservation(img_fwd, K, T_bc, 0))
        )
        b_same_reject.ingest(
            Keyframe((0, 1), 2_000_000_000, T1, np.eye(4), transform_points(invert(T1), pts_world), VisualObservation(img_rev, K, T_bc, 2_000_000_000))
        )
        ev_rej = b_same_reject.ingest(
            Keyframe((0, 2), 4_000_000_000, T_ret_same, np.eye(4), transform_points(invert(T_ret_same), pts_world), VisualObservation(flat_img, K, T_bc, 4_000_000_000))
        )
        self.assertFalse(ev_rej["loop"])
        self.assertGreaterEqual(b_same_reject.hybrid_same_heading_visual_rejected, 1)

        # 场景 B：同向重访带匹配纹理 -> 走视觉校验放行；之后继续反向重访 (180° >= 65°) -> 走 360° LiDAR ICP 放行
        b_hybrid = SlamBackend(_make_cfg())
        b_hybrid.ingest(
            Keyframe((0, 0), 0, T0, np.eye(4), pts_world, VisualObservation(img_fwd, K, T_bc, 0))
        )
        b_hybrid.ingest(
            Keyframe((0, 1), 2_000_000_000, T1, np.eye(4), transform_points(invert(T1), pts_world), VisualObservation(img_rev, K, T_bc, 2_000_000_000))
        )
        # 同向返回 (0, 2)
        ev_same = b_hybrid.ingest(
            Keyframe((0, 2), 4_000_000_000, T0, np.eye(4), pts_world, VisualObservation(img_fwd, K, T_bc, 4_000_000_000))
        )
        self.assertTrue(ev_same["loop"])
        self.assertEqual(b_hybrid.hybrid_same_heading_visual_accepted, 1)

        # 反向掉头 180° 重访 (0, 3)（此时前向相机看到完全不同的 img_rev）
        T_rev = _yaw_transform(np.pi, (0.05, 0.0, 0.0))
        ev_rev = b_hybrid.ingest(
            Keyframe((0, 3), 6_000_000_000, T_rev, np.eye(4), transform_points(invert(T_rev), pts_world), VisualObservation(img_rev, K, T_bc, 6_000_000_000))
        )
        self.assertTrue(ev_rev["loop"])
        self.assertEqual(b_hybrid.hybrid_reverse_lidar_accepted, 1)
        stats = b_hybrid.observability_stats()
        self.assertEqual(stats["hybrid_same_heading_visual_accepted"], 1)
        self.assertEqual(stats["hybrid_reverse_lidar_accepted"], 1)
        self.assertIn("color_sc_rescored", stats)
        self.assertIn("visual_seq_rejected", stats)

    def test_v3_presets_load_and_strict_validation_rejects_bad_inputs(self) -> None:
        """两个 v3 预设 JSON 应正常加载，非法键或越界字段必须抛出 ValueError。"""
        p1 = REPO_ROOT / "configs" / "localization_mapping" / "slam_v3_hybrid_color_visual.json"
        p2 = REPO_ROOT / "configs" / "localization_mapping" / "slam_v3_color_visual_multiview.json"
        cfg1 = BackendConfig.from_json(p1)
        self.assertTrue(cfg1.use_visual_retrieval)
        self.assertTrue(cfg1.color_sc_enabled)
        self.assertEqual(cfg1.visual_retrieval.feature_type, "akaze")
        self.assertEqual(cfg1.visual_multiview_radius, 1)
        self.assertAlmostEqual(cfg1.hybrid_heading_gate_deg, 65.0)
        self.assertAlmostEqual(cfg1.color_sc_weight, 0.25)
        self.assertAlmostEqual(cfg1.fine_icp_voxel_m, 0.12)

        cfg2 = BackendConfig.from_json(p2)
        self.assertEqual(cfg2.visual_multiview_radius, 2)
        self.assertEqual(cfg2.visual_seq_consistency, 2)
        self.assertTrue(cfg2.color_sc_enabled)
        self.assertAlmostEqual(cfg2.hybrid_heading_gate_deg, 75.0)

        for bad in (
            {"unknown_v3_key": True},
            {"color_sc_enabled": 1},
            {"color_sc_weight": -0.1},
            {"color_sc_min_bins": 0},
            {"color_sc_max_distance": 1.5},
            {"visual_multiview_radius": -1},
            {"visual_seq_consistency": 0},
            {"hybrid_heading_gate_deg": 0.0},
            {"hybrid_heading_gate_deg": 65.0},
            {"hybrid_heading_gate_deg": 181.0},
            {"reverse_icp_fitness_min": 0.6},
            {"reverse_icp_fitness_min": 1.2},
            {"reverse_icp_rmse_max": 0.1},
            {"reverse_icp_rmse_max": 0.0},
            {"use_visual_retrieval": True, "hybrid_heading_gate_deg": 0.0},
            {"use_visual_retrieval": True, "hybrid_heading_gate_deg": 181.0},
            {"use_visual_retrieval": True, "reverse_icp_fitness_min": 1.2},
            {"use_visual_retrieval": True, "reverse_icp_rmse_max": 0.0},
            {"fine_icp_voxel_m": -0.05},
        ):
            with self.assertRaises(ValueError, msg=repr(bad)):
                BackendConfig.from_dict(bad)

        with self.assertRaises(ValueError):
            VisualRetrievalConfig(verify_min_inliers=2)
        with self.assertRaises(ValueError):
            VisualRetrievalConfig(verify_min_inliers=10, max_features=8)

    def test_grayscale_01_normalization_and_color_sc_stale_stamp_and_badzip(self) -> None:
        K, T_bc = _camera_extrinsics()
        pts = _asymmetric_room_points(seed=17)
        gray_255 = _textured_image(seed=19)
        gray_01 = gray_255 / 255.0
        obs_255 = VisualObservation(gray=gray_255, K=K, T_body_camera=T_bc, stamp_ns=1_000_000_000)
        obs_01 = VisualObservation(gray=gray_01, K=K, T_body_camera=T_bc, stamp_ns=1_000_000_000)
        np.testing.assert_allclose(obs_01.gray, obs_255.gray, atol=1e-9)

        sc_cfg = ScanContextConfig(rings=20, sectors=60, max_range_m=20.0)
        cidx = ColorScanContextIndex(sc_cfg, max_time_delta_s=0.05)
        kf_fresh = Keyframe((0, 0), 1_000_000_000, np.eye(4), np.eye(4), pts, obs_01)
        kf_stale = Keyframe(
            (0, 1),
            1_000_000_000,
            np.eye(4),
            np.eye(4),
            pts,
            VisualObservation(gray=gray_01, K=K, T_body_camera=T_bc, stamp_ns=1_200_000_000),
        )
        self.assertIsNotNone(cidx.compute(kf_fresh))
        self.assertIsNone(cidx.compute(kf_stale))

        with tempfile.TemporaryDirectory() as tmp:
            kf_dir = Path(tmp) / "kf_0_0"
            kf_dir.mkdir()
            (kf_dir / "metadata.json").write_text(
                json.dumps(
                    {
                        "schema": "livo_keyframe/v1",
                        "epoch": 0,
                        "sequence": 0,
                        "stamp_ns": 100,
                        "T_odom_body": np.eye(4).tolist(),
                        "T_body_lidar": np.eye(4).tolist(),
                    }
                ),
                encoding="utf-8",
            )
            (kf_dir / "points_lidar.pcd").write_text("VERSION .7\nFIELDS x y z\nDATA ascii\n1 0 0\n", encoding="utf-8")
            # Truncated ZIP header triggers zipfile.BadZipFile inside np.load
            (kf_dir / "visual.npz").write_bytes(b"PK\x03\x04truncated_corrupt_zip")
            with self.assertRaises(ValueError) as ctx:
                read_keyframe(kf_dir, load_visual=True)
            self.assertIn(str(kf_dir), str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
