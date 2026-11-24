"""物体级 3D Surface Nets Mesh 重建与关键帧锚定记忆库单元测试。"""

from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping.object_mesh_memory import (
    ObjectMemoryEntry,
    ObjectMeshConfig,
    ObjectMeshMemory,
    cluster_foreground_objects,
    mesh_object_surface_nets,
    separate_foreground_objects,
    write_ply_mesh,
)


def _sample_box_surface(
    center: tuple[float, float, float],
    extent: tuple[float, float, float],
    step: float = 0.05,
) -> tuple[np.ndarray, np.ndarray]:
    """生成带外法向的立方体六面点云。"""
    cx, cy, cz = center
    lx, ly, lz = extent
    xs = np.arange(-lx / 2, lx / 2 + 1e-6, step)
    ys = np.arange(-ly / 2, ly / 2 + 1e-6, step)
    zs = np.arange(-lz / 2, lz / 2 + 1e-6, step)
    pts_list, nrm_list = [], []
    # +/- X
    yy, zz = np.meshgrid(ys, zs)
    for sgn in (-1.0, 1.0):
        pts_list.append(np.column_stack([np.full(yy.size, cx + sgn * lx / 2), cy + yy.ravel(), cz + zz.ravel()]))
        nrm_list.append(np.tile([sgn, 0.0, 0.0], (yy.size, 1)))
    # +/- Y
    xx, zz = np.meshgrid(xs, zs)
    for sgn in (-1.0, 1.0):
        pts_list.append(np.column_stack([cx + xx.ravel(), np.full(xx.size, cy + sgn * ly / 2), cz + zz.ravel()]))
        nrm_list.append(np.tile([0.0, sgn, 0.0], (xx.size, 1)))
    # +/- Z
    xx, yy = np.meshgrid(xs, ys)
    for sgn in (-1.0, 1.0):
        pts_list.append(np.column_stack([cx + xx.ravel(), cy + yy.ravel(), np.full(xx.size, cz + sgn * lz / 2)]))
        nrm_list.append(np.tile([0.0, 0.0, sgn], (xx.size, 1)))
    return np.vstack(pts_list), np.vstack(nrm_list)


class ObjectMeshMemoryTests(unittest.TestCase):
    def test_separate_and_cluster_room_objects(self) -> None:
        # 构造 6m x 6m x 2.8m 房间（地板 + 天花板 + 两面大墙）+ 1 张桌子 + 1 把椅子
        g = np.arange(-3.0, 3.01, 0.10)
        xx, yy = np.meshgrid(g, g)
        floor = np.column_stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)])
        floor_n = np.tile([0.0, 0.0, 1.0], (xx.size, 1))
        ceil = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, 2.8)])
        ceil_n = np.tile([0.0, 0.0, -1.0], (xx.size, 1))

        zz = np.arange(0.15, 2.65, 0.10)
        wx, wz = np.meshgrid(g, zz)
        wall = np.column_stack([wx.ravel(), np.full(wx.size, 3.0), wz.ravel()])
        wall_n = np.tile([0.0, -1.0, 0.0], (wx.size, 1))

        desk_pts, desk_n = _sample_box_surface((0.0, 0.0, 0.45), (1.2, 0.7, 0.70), step=0.08)
        chair_pts, chair_n = _sample_box_surface((1.8, -1.0, 0.45), (0.5, 0.5, 0.75), step=0.08)

        pts = np.vstack([floor, ceil, wall, desk_pts, chair_pts])
        nrm = np.vstack([floor_n, ceil_n, wall_n, desk_n, chair_n])
        cfg = ObjectMeshConfig()
        fg_mask, _, stats = separate_foreground_objects(pts, nrm, cfg)
        self.assertGreater(stats["floor_voxels"], 1000)
        self.assertGreater(stats["ceiling_voxels"], 1000)
        self.assertGreater(stats["wall_voxels"], 500)

        clusters = cluster_foreground_objects(pts[fg_mask], cfg)
        self.assertEqual(len(clusters), 2)

    def test_surface_nets_mesh_and_ply_export(self) -> None:
        pts, nrm = _sample_box_surface((1.0, -0.5, 0.6), (0.9, 0.6, 0.7), step=0.05)
        rgb = np.tile(np.array([180, 120, 60], dtype=np.uint8), (len(pts), 1))
        mesh = mesh_object_surface_nets(pts, nrm, rgb, voxel_m=0.06, smooth_iters=1)
        self.assertGreater(mesh.num_vertices, 40)
        self.assertGreater(mesh.num_faces, 60)
        self.assertTrue(np.all((mesh.faces >= 0) & (mesh.faces < mesh.num_vertices)))
        self.assertEqual(mesh.colors_rgb.shape, (mesh.num_vertices, 3))
        with tempfile.TemporaryDirectory() as tmp:
            ply = Path(tmp) / "desk.ply"
            write_ply_mesh(ply, mesh)
            txt = ply.read_text(encoding="utf-8")
            self.assertIn(f"element vertex {mesh.num_vertices}", txt)
            self.assertIn(f"element face {mesh.num_faces}", txt)

    def test_memory_query_and_keyframe_reprojection(self) -> None:
        e1 = ObjectMemoryEntry(
            object_id="obj_0001",
            floor_label="floor_1_upper",
            geometric_category="desk_or_table",
            dominant_color="brown_wood",
            anchor_kf_key=[1, 42],
            center_world_xyz=[10.0, 20.0, 3.6],
            center_anchor_xyz=[1.0, 0.0, 0.4],
            obb_extent_lwh_m=[1.4, 0.8, 0.75],
            obb_yaw_rad=0.0,
            height_above_floor_m=0.0,
            point_count=300,
            num_vertices=120,
            num_triangles=220,
            mean_rgb=[160, 110, 65],
            mesh_ply="meshes/obj_0001.ply",
            best_rgb_crop="crops/obj_0001_crop.png",
            best_view_frame=None,
            best_view_stamp_s=100.0,
        )
        mem = ObjectMeshMemory([e1], Path("/tmp"))
        hits = mem.query(floor_label="floor_1_upper", category="desk", color="brown")
        self.assertEqual(len(hits), 1)
        self.assertEqual(hits[0].object_id, "obj_0001")

        # 模拟回环更新 anchor_kf 位姿：平移 (+0.5, -0.3, +0.1)
        T_new = np.eye(4, dtype=np.float64)
        T_new[:3, 3] = [9.5, 19.7, 3.3]
        moved = mem.reproject_from_keyframe_poses({(1, 42): T_new})
        self.assertEqual(moved, 1)
        np.testing.assert_allclose(e1.center_world_xyz, [10.5, 19.7, 3.7], atol=1e-4)


if __name__ == "__main__":
    unittest.main()
