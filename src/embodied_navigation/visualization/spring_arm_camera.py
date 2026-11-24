"""游戏风格 Spring-Arm 防穿墙/防穿模跟随相机与多视角相机运动学核心（纯 NumPy）。

设计思路对标游戏引擎（Unreal ``USpringArmComponent`` / Unity Cinemachine）：
1. 从机器人上方锚点向理想第三人称机位发射多射线球体探测（Sphere-Cast）；
2. 探测射线与实时点云体素障碍栅格求交，遇墙、门框、栏杆或楼板立即快速收缩悬臂；
3. 驶出狭窄走廊/门洞后按较慢时间常数平滑回弹，同时施加天花板高度软限幅与姿态低通滤波，
   彻底消除 RViz 固定大半径 Orbit 视角在室内多层建筑中的穿墙、穿楼板与画面剧烈跳动。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation

from embodied_navigation.common.geometry import wrap_angle


@dataclass(frozen=True)
class CameraPose:
    """相机在固定世界坐标系（如 ``map``）下的 6-DoF 位姿。"""

    frame_id: str
    position_xyz: np.ndarray  # (3,) float64
    quaternion_xyzw: np.ndarray  # (4,) float64
    arm_length_m: float
    clipped: bool


@dataclass(frozen=True)
class SpringArmCameraConfig:
    """Spring-Arm 防穿模相机与多机位参数。"""

    # 障碍物体素栅格分辨率与容量上限
    voxel_m: float = 0.25
    max_voxels: int = 350_000

    # 第三人称防穿墙主相机（spring_follow）
    anchor_height_m: float = 0.65
    look_ahead_m: float = 1.25
    look_height_m: float = 0.45
    desired_arm_length_m: float = 4.2
    min_arm_length_m: float = 0.95
    pitch_deg: float = 20.0
    probe_radius_m: float = 0.35
    collision_margin_m: float = 0.35
    ray_step_m: float = 0.15
    ceiling_margin_m: float = 0.35
    ceiling_search_min_dz_m: float = 1.25
    ceiling_search_max_dz_m: float = 4.5
    self_ignore_radius_m: float = 0.65

    # 指数平滑时间常数（秒）：遇障快收，离障慢弹
    pos_tau_s: float = 0.22
    yaw_tau_s: float = 0.65
    retract_tau_s: float = 0.06
    extend_tau_s: float = 0.65
    # 沿速度方向平滑转头的最小水平速度阈值（避免原地步态噪声）
    velocity_yaw_min_speed_mps: float = 0.25

    # 俯视切片相机（overhead_cutaway）
    overhead_height_m: float = 13.5
    overhead_pos_tau_s: float = 0.45
    cutaway_above_robot_m: float = 1.85
    cutaway_below_robot_m: float = 0.90

    # 等轴 45° 双层立体相机（isometric_follow）
    iso_distance_m: float = 11.0
    iso_pitch_deg: float = 42.0
    iso_yaw_deg: float = 45.0
    iso_pos_tau_s: float = 0.45


class VoxelObstacleGrid:
    """轻量级 3D 空间体素障碍栅格，用于 Spring-Arm 实时球体防穿墙射线检测。"""

    def __init__(self, voxel_m: float = 0.25, max_voxels: int = 350_000) -> None:
        if voxel_m <= 0.0 or not math.isfinite(voxel_m):
            raise ValueError(f"voxel_m must be finite and positive, got {voxel_m!r}")
        self.voxel_m = float(voxel_m)
        self.inv_voxel = 1.0 / self.voxel_m
        self.max_voxels = int(max_voxels)
        self._occupied: set[tuple[int, int, int]] = set()

    def __len__(self) -> int:
        return len(self._occupied)

    def clear(self) -> None:
        self._occupied.clear()

    def replace_points(self, points_xyz: np.ndarray) -> int:
        """用最新完整全局地图点云整体刷新障碍体素（零累计残留）。"""
        pts = np.asarray(points_xyz, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) == 0:
            return len(self._occupied)
        xyz = pts[:, :3]
        finite = np.all(np.isfinite(xyz), axis=1)
        xyz = xyz[finite]
        if len(xyz) == 0:
            return len(self._occupied)
        keys = np.floor(xyz * self.inv_voxel).astype(np.int64)
        uniq = np.unique(keys, axis=0)
        if len(uniq) > self.max_voxels:
            step = max(1, len(uniq) // self.max_voxels)
            uniq = uniq[::step]
        self._occupied = set(map(tuple, uniq.tolist()))
        return len(self._occupied)

    def add_points(self, points_xyz: np.ndarray) -> int:
        """增量并入局部扫描点云体素。"""
        pts = np.asarray(points_xyz, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) == 0:
            return len(self._occupied)
        xyz = pts[:, :3]
        finite = np.all(np.isfinite(xyz), axis=1)
        xyz = xyz[finite]
        if len(xyz) == 0:
            return len(self._occupied)
        keys = np.floor(xyz * self.inv_voxel).astype(np.int64)
        uniq = np.unique(keys, axis=0)
        for row in uniq.tolist():
            if len(self._occupied) >= self.max_voxels:
                break
            self._occupied.add((row[0], row[1], row[2]))
        return len(self._occupied)

    def is_occupied(self, point_xyz: np.ndarray) -> bool:
        ix = math.floor(float(point_xyz[0]) * self.inv_voxel)
        iy = math.floor(float(point_xyz[1]) * self.inv_voxel)
        iz = math.floor(float(point_xyz[2]) * self.inv_voxel)
        return (ix, iy, iz) in self._occupied

    def estimate_ceiling_z(
        self,
        xy: np.ndarray,
        robot_z: float,
        *,
        min_dz_m: float = 1.25,
        max_dz_m: float = 4.5,
        xy_radius_cells: int = 2,
    ) -> float | None:
        """探测机器人上方最近楼板/天花板高度，防止相机穿入上层空间或楼板内部。"""
        if not self._occupied:
            return None
        cx = math.floor(float(xy[0]) * self.inv_voxel)
        cy = math.floor(float(xy[1]) * self.inv_voxel)
        iz_min = math.floor((robot_z + min_dz_m) * self.inv_voxel)
        iz_max = math.ceil((robot_z + max_dz_m) * self.inv_voxel)
        for iz in range(iz_min, iz_max + 1):
            hits = 0
            for dx in range(-xy_radius_cells, xy_radius_cells + 1):
                for dy in range(-xy_radius_cells, xy_radius_cells + 1):
                    if (cx + dx, cy + dy, iz) in self._occupied:
                        hits += 1
            # 至少 3 个相邻水平体素命中才判定为真实楼板/天花板，过滤偶发噪点
            if hits >= 3:
                return iz * self.voxel_m
        return None

    def sphere_cast(
        self,
        origin: np.ndarray,
        target: np.ndarray,
        *,
        probe_radius_m: float = 0.35,
        collision_margin_m: float = 0.35,
        min_dist_m: float = 0.95,
        step_m: float = 0.15,
        self_ignore_radius_m: float = 0.65,
    ) -> tuple[float, bool]:
        """沿 ``origin -> target`` 执行多射线球体碰撞探测，返回 ``(safe_dist_m, clipped)``。"""
        vec = np.asarray(target, dtype=np.float64) - np.asarray(origin, dtype=np.float64)
        total_dist = float(np.linalg.norm(vec))
        if total_dist <= 1e-6 or not self._occupied:
            return total_dist, False
        direction = vec / total_dist

        # 构造垂直于射线的正交基 (u, v)，围绕主射线布置 6 条包络偏置射线模拟球体扫掠
        ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        if abs(float(np.dot(direction, ref))) > 0.95:
            ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        u = np.cross(direction, ref)
        u /= float(np.linalg.norm(u))
        v = np.cross(direction, u)
        offsets = [
            np.zeros(3, dtype=np.float64),
            probe_radius_m * u,
            -probe_radius_m * u,
            probe_radius_m * v,
            -probe_radius_m * v,
            0.7071 * probe_radius_m * (u + v),
            0.7071 * probe_radius_m * (u - v),
            0.7071 * probe_radius_m * (-u + v),
            0.7071 * probe_radius_m * (-u - v),
        ]

        d = max(step_m, self_ignore_radius_m)
        while d <= total_dist + 1e-9:
            s = min(d, total_dist)
            center = origin + direction * s
            for off in offsets:
                if self.is_occupied(center + off):
                    safe = max(min_dist_m, s - collision_margin_m)
                    return min(safe, total_dist), True
            d += step_m
        return total_dist, False


def build_look_at_pose(
    eye: np.ndarray,
    target: np.ndarray,
    *,
    up_hint: np.ndarray | None = None,
    frame_id: str = "rviz_cam",
    arm_length_m: float = 0.0,
    clipped: bool = False,
) -> CameraPose:
    """构造 RViz2 ``FrameAligned``（``Yaw=0, Pitch=0, Point towards=+x axis``）兼容的相机 TF 姿态。

    在 RViz2 ``FPSViewController / FrameViewController`` 默认 ``Yaw=0, Pitch=0`` 下，相机沿目标坐标系
    **+X 轴前视、+Y 轴朝左、+Z 轴朝上**（标准 ROS FLU 右手系）。
    构造正交旋转矩阵 ``R = [forward, left, up]``，使相机镜头精准对准 ``target`` 且无滚转倾斜。
    """
    eye_v = np.asarray(eye, dtype=np.float64).reshape(3)
    tgt_v = np.asarray(target, dtype=np.float64).reshape(3)
    fwd = tgt_v - eye_v
    norm = float(np.linalg.norm(fwd))
    if norm <= 1e-6:
        fwd = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        fwd = fwd / norm

    up_ref = np.array([0.0, 0.0, 1.0], dtype=np.float64) if up_hint is None else np.asarray(up_hint, dtype=np.float64)
    if abs(float(np.dot(fwd, up_ref))) > 0.995:
        up_ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    left = np.cross(up_ref, fwd)
    left /= float(np.linalg.norm(left))
    up = np.cross(fwd, left)
    up /= float(np.linalg.norm(up))

    R = np.column_stack([fwd, left, up])
    quat_xyzw = Rotation.from_matrix(R).as_quat()
    return CameraPose(
        frame_id=frame_id,
        position_xyz=eye_v.copy(),
        quaternion_xyzw=quat_xyzw.astype(np.float64),
        arm_length_m=float(arm_length_m),
        clipped=bool(clipped),
    )


def filter_floor_cutaway(
    points_xyz: np.ndarray,
    robot_z: float,
    *,
    above_m: float = 1.85,
    below_m: float = 0.90,
) -> np.ndarray:
    """按当前机器人高度切除上层楼板/天花板点云，使俯视机位无遮挡透视当前楼层内部。"""
    pts = np.asarray(points_xyz)
    if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) == 0:
        return pts[:0]
    z = pts[:, 2]
    mask = (z >= (robot_z - below_m)) & (z <= (robot_z + above_m))
    return pts[mask]


class SpringArmCameraController:
    """维护平滑状态，输出三路同步机位（第三人称防穿墙、俯视切片、等轴立体）。"""

    def __init__(self, config: SpringArmCameraConfig | None = None) -> None:
        self.config = config or SpringArmCameraConfig()
        self.grid = VoxelObstacleGrid(
            voxel_m=self.config.voxel_m,
            max_voxels=self.config.max_voxels,
        )
        self._last_stamp_s: float | None = None
        self._last_robot_pos: np.ndarray | None = None
        self._smooth_pos: np.ndarray | None = None
        self._smooth_yaw: float | None = None
        self._current_arm: float = self.config.desired_arm_length_m
        self._smooth_overhead_pos: np.ndarray | None = None
        self._smooth_iso_pos: np.ndarray | None = None

    @staticmethod
    def _exp_alpha(dt_s: float, tau_s: float) -> float:
        if tau_s <= 1e-6 or dt_s <= 0.0:
            return 1.0
        return 1.0 - math.exp(-dt_s / tau_s)

    def update(self, robot_xyz: np.ndarray, robot_yaw_rad: float, stamp_s: float) -> dict[str, CameraPose]:
        """根据最新机器人位姿与时间戳更新全部机位（带球体避障与快收慢弹阻尼）。"""
        c = self.config
        pos = np.asarray(robot_xyz, dtype=np.float64).reshape(3)
        yaw = float(robot_yaw_rad)

        if self._last_stamp_s is None or self._smooth_pos is None or self._smooth_yaw is None:
            self._last_stamp_s = float(stamp_s)
            self._last_robot_pos = pos.copy()
            self._smooth_pos = pos.copy()
            self._smooth_yaw = yaw
            self._current_arm = c.desired_arm_length_m
            self._smooth_overhead_pos = pos.copy()
            self._smooth_iso_pos = pos.copy()
            dt = 0.0
        else:
            dt = max(0.0, min(1.0, float(stamp_s) - self._last_stamp_s))
            self._last_stamp_s = float(stamp_s)
            # 当机器人显著直线前进时，优先向速度航向微调融合，减少原地步态/倒退镜头乱甩
            target_yaw = yaw
            if dt > 1e-3 and self._last_robot_pos is not None:
                dxy = pos[:2] - self._last_robot_pos[:2]
                speed = float(np.linalg.norm(dxy)) / dt
                if speed >= c.velocity_yaw_min_speed_mps:
                    vel_yaw = math.atan2(float(dxy[1]), float(dxy[0]))
                    # 仅在前进方向与机头夹角 < 75° 时混合速度航向（倒车脱困时保持机头朝向）
                    if abs(wrap_angle(vel_yaw - yaw)) < math.radians(75.0):
                        target_yaw = wrap_angle(yaw + 0.35 * wrap_angle(vel_yaw - yaw))
            self._last_robot_pos = pos.copy()

            a_pos = self._exp_alpha(dt, c.pos_tau_s)
            a_yaw = self._exp_alpha(dt, c.yaw_tau_s)
            self._smooth_pos = self._smooth_pos + a_pos * (pos - self._smooth_pos)
            self._smooth_yaw = wrap_angle(self._smooth_yaw + a_yaw * wrap_angle(target_yaw - self._smooth_yaw))

            a_ov = self._exp_alpha(dt, c.overhead_pos_tau_s)
            self._smooth_overhead_pos = self._smooth_overhead_pos + a_ov * (pos - self._smooth_overhead_pos)
            a_iso = self._exp_alpha(dt, c.iso_pos_tau_s)
            self._smooth_iso_pos = self._smooth_iso_pos + a_iso * (pos - self._smooth_iso_pos)

        # 1. 游戏风格第三人称防穿墙跟随相机（spring_follow）
        fwd_xy = np.array([math.cos(self._smooth_yaw), math.sin(self._smooth_yaw), 0.0], dtype=np.float64)
        anchor = self._smooth_pos + np.array([0.0, 0.0, c.anchor_height_m], dtype=np.float64)
        pitch_rad = math.radians(c.pitch_deg)
        # 悬臂方向指向机器人斜后上方
        arm_dir = -math.cos(pitch_rad) * fwd_xy + np.array([0.0, 0.0, math.sin(pitch_rad)], dtype=np.float64)
        arm_dir /= float(np.linalg.norm(arm_dir))

        desired_eye = anchor + arm_dir * c.desired_arm_length_m
        # 天花板高度硬保护：防止第三人称相机穿入上层楼板
        ceil_z = self.grid.estimate_ceiling_z(
            self._smooth_pos[:2],
            float(self._smooth_pos[2]),
            min_dz_m=c.ceiling_search_min_dz_m,
            max_dz_m=c.ceiling_search_max_dz_m,
        )
        ceiling_clamped = False
        if ceil_z is not None:
            max_eye_z = ceil_z - c.ceiling_margin_m
            if desired_eye[2] > max_eye_z and desired_eye[2] > anchor[2] + 0.15:
                desired_eye[2] = max(anchor[2] + 0.15, max_eye_z)
                ceiling_clamped = True

        # 球体扫掠避障检测（墙、门框、柱子、扶手、楼板）
        safe_arm, hit_wall = self.grid.sphere_cast(
            anchor,
            desired_eye,
            probe_radius_m=c.probe_radius_m,
            collision_margin_m=c.collision_margin_m,
            min_dist_m=c.min_arm_length_m,
            step_m=c.ray_step_m,
            self_ignore_radius_m=c.self_ignore_radius_m,
        )
        # 快收慢弹（Fast Retract / Smooth Extend）
        if safe_arm < self._current_arm:
            a_arm = self._exp_alpha(dt, c.retract_tau_s) if dt > 0.0 else 1.0
        else:
            a_arm = self._exp_alpha(dt, c.extend_tau_s) if dt > 0.0 else 1.0
        self._current_arm = self._current_arm + a_arm * (safe_arm - self._current_arm)
        # 保证即时碰撞硬约束，绝不允许当前悬臂超过刚刚测出的无碰撞安全距离
        if hit_wall and self._current_arm > safe_arm:
            self._current_arm = safe_arm

        vec_arm = desired_eye - anchor
        vec_norm = float(np.linalg.norm(vec_arm))
        unit_arm = (vec_arm / vec_norm) if vec_norm > 1e-6 else arm_dir
        eye_spring = anchor + unit_arm * self._current_arm
        look_spring = self._smooth_pos + fwd_xy * c.look_ahead_m + np.array([0.0, 0.0, c.look_height_m])
        cam_spring = build_look_at_pose(
            eye_spring,
            look_spring,
            frame_id="rviz_cam_spring",
            arm_length_m=self._current_arm,
            clipped=(hit_wall or ceiling_clamped),
        )

        # 2. 本层俯视平面图追踪锚点（供 RViz TopDownOrtho 平滑锁定）
        cam_overhead = build_look_at_pose(
            self._smooth_overhead_pos,
            self._smooth_overhead_pos + np.array([1.0, 0.0, 0.0], dtype=np.float64),
            frame_id="rviz_cam_overhead",
            arm_length_m=c.overhead_height_m,
            clipped=False,
        )

        # 3. 等轴 45° 双层立体追踪锚点（供 RViz Orbit 平滑锁定）
        cam_iso = build_look_at_pose(
            self._smooth_iso_pos,
            self._smooth_iso_pos + np.array([1.0, 0.0, 0.0], dtype=np.float64),
            frame_id="rviz_cam_isometric",
            arm_length_m=c.iso_distance_m,
            clipped=False,
        )

        return {
            "spring_follow": cam_spring,
            "overhead_cutaway": cam_overhead,
            "isometric_follow": cam_iso,
        }
