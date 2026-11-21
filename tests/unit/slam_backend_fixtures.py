"""Synthetic world, trajectories and spool writer shared by test_slam_backend_*.

Only numpy and the standard library; nothing here imports the backend under
test, so the fixtures can be checked on their own.

World: a 30 m x 20 m room (scene frame, origin at a corner, z up) built from
axis-aligned boxes -- outer walls, an inner partial wall, pillars and crates,
plus the floor.  ``SyntheticWorld.surface_points`` samples every exposed box
face uniformly; ``SyntheticWorld.scan`` simulates a 360 deg, 16-channel
spinning lidar (range 20 m, 0.01 m Gaussian noise, fixed seeds) by casting
rays against the same box surfaces, so occlusion is exact.

Pose convention follows the frozen rulings: 4x4 float64, ``T_a_b`` maps points
in frame b into frame a.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np

SEED = 20261115
ROOM_X_M = 30.0
ROOM_Y_M = 20.0
WALL_HEIGHT_M = 3.0
LIDAR_RANGE_M = 20.0
LIDAR_MIN_RANGE_M = 0.3
LIDAR_NOISE_M = 0.01
LIDAR_AZIMUTH_STEP_DEG = 1.0
LIDAR_ELEVATIONS_DEG = np.linspace(-15.0, 15.0, 16)

# Lidar sits 0.4 m above the body origin (body origin on the floor), 0.1 m forward.
T_BODY_LIDAR = np.eye(4)
T_BODY_LIDAR[:3, 3] = [0.10, 0.0, 0.40]

# Loop used for mapping (scene frame): rectangle, 20 m x 10 m, perimeter 60 m.
LOOP_CORNERS = np.array([[5.0, 4.0], [25.0, 4.0], [25.0, 14.0], [5.0, 14.0], [5.0, 4.0]])
LOOP_PERIMETER_M = 60.0


# --------------------------------------------------------------------------- poses
def pose(x=0.0, y=0.0, yaw=0.0, z=0.0, roll=0.0, pitch=0.0):
    """4x4 pose from translation and ZYX Euler angles (rad)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    rx = np.array([[1.0, 0, 0], [0, cr, -sr], [0, sr, cr]])
    T = np.eye(4)
    T[:3, :3] = rz @ ry @ rx
    T[:3, 3] = [x, y, z]
    return T


def pose_deg(xyz, rpy_deg=(0.0, 0.0, 0.0)):
    """4x4 pose from ``(x, y, z)`` and ``(roll, pitch, yaw)`` in degrees."""
    r, p, y = np.radians(rpy_deg)
    return pose(xyz[0], xyz[1], yaw=y, z=xyz[2], roll=r, pitch=p)


def inverse(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def transform_points(T, points):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return points @ T[:3, :3].T + T[:3, 3]


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rotation_angle(R):
    """Geodesic angle of a 3x3 rotation (rad)."""
    c = (np.trace(R) - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, c)))


def pose_error(T_est, T_true):
    """(translation error m, rotation error rad) between two 4x4 poses."""
    d = inverse(T_true) @ T_est
    return float(np.linalg.norm(T_est[:3, 3] - T_true[:3, 3])), rotation_angle(d[:3, :3])


# --------------------------------------------------------------------------- world
def _box(cx, cy, sx, sy, h, z0=0.0):
    return np.array([[cx - sx / 2, cy - sy / 2, z0], [cx + sx / 2, cy + sy / 2, z0 + h]])


class SyntheticWorld:
    """Axis-aligned boxes. ``boxes`` has shape (M, 2, 3): [min_xyz, max_xyz]."""

    def __init__(self):
        t = 0.2
        boxes = [
            # floor (top face at z = 0) -- slightly larger than the room
            np.array([[-t, -t, -0.1], [ROOM_X_M + t, ROOM_Y_M + t, 0.0]]),
            # outer walls (inner faces at x=0, x=30, y=0, y=20)
            np.array([[-t, -t, 0.0], [0.0, ROOM_Y_M + t, WALL_HEIGHT_M]]),
            np.array([[ROOM_X_M, -t, 0.0], [ROOM_X_M + t, ROOM_Y_M + t, WALL_HEIGHT_M]]),
            np.array([[-t, -t, 0.0], [ROOM_X_M + t, 0.0, WALL_HEIGHT_M]]),
            np.array([[-t, ROOM_Y_M, 0.0], [ROOM_X_M + t, ROOM_Y_M + t, WALL_HEIGHT_M]]),
            # inner partial wall, 2 m high
            _box(15.0, 12.5, 6.0, 0.2, 2.0),
            # pillars 0.4 x 0.4 x 3 m
            _box(10.0, 8.0, 0.4, 0.4, WALL_HEIGHT_M),
            _box(15.0, 10.0, 0.4, 0.4, WALL_HEIGHT_M),
            _box(20.0, 7.0, 0.4, 0.4, WALL_HEIGHT_M),
            _box(8.0, 17.0, 0.4, 0.4, WALL_HEIGHT_M),
            _box(27.0, 2.0, 0.4, 0.4, WALL_HEIGHT_M),
            _box(2.0, 11.0, 0.4, 0.4, WALL_HEIGHT_M),
            _box(18.0, 17.5, 0.4, 0.4, WALL_HEIGHT_M),
            # crates of different sizes and heights
            _box(12.0, 6.5, 2.0, 1.0, 1.0),
            _box(19.0, 9.5, 1.5, 1.5, 1.5),
            _box(22.0, 1.5, 3.0, 1.0, 0.8),
            _box(3.0, 17.0, 1.5, 2.0, 1.2),
            _box(28.0, 16.0, 1.0, 3.0, 2.0),
            _box(14.0, 17.0, 2.0, 1.0, 0.6),
            _box(2.0, 2.0, 1.0, 1.0, 1.0),
        ]
        self.boxes = np.stack(boxes)

    # -- uniform surface sampling (world description as a point set) --------
    def surface_points(self, spacing_m=0.1, seed=SEED):
        """Uniformly sample every box face inside the room (scene frame)."""
        rng = np.random.default_rng(seed)
        chunks = []
        for lo, hi in self.boxes:
            size = hi - lo
            for axis in range(3):
                others = [a for a in range(3) if a != axis]
                area = size[others[0]] * size[others[1]]
                n = max(1, int(round(area / spacing_m ** 2)))
                for value in (lo[axis], hi[axis]):
                    pts = np.empty((n, 3))
                    pts[:, axis] = value
                    for a in others:
                        pts[:, a] = rng.uniform(lo[a], hi[a], n)
                    chunks.append(pts)
        pts = np.concatenate(chunks)
        inside = ((pts[:, 0] >= 0) & (pts[:, 0] <= ROOM_X_M) & (pts[:, 1] >= 0)
                  & (pts[:, 1] <= ROOM_Y_M) & (pts[:, 2] >= 0) & (pts[:, 2] <= WALL_HEIGHT_M))
        return pts[inside]

    # -- lidar ---------------------------------------------------------------
    @staticmethod
    def ray_directions():
        az = np.deg2rad(np.arange(0.0, 360.0, LIDAR_AZIMUTH_STEP_DEG))
        el = np.deg2rad(LIDAR_ELEVATIONS_DEG)
        A, E = np.meshgrid(az, el, indexing="ij")
        return np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)

    def scan(self, T_scene_lidar, seed):
        """Simulated scan in the lidar frame: (N, 3) float64."""
        dirs_lidar = self.ray_directions()
        dirs = dirs_lidar @ T_scene_lidar[:3, :3].T
        origin = T_scene_lidar[:3, 3]
        with np.errstate(divide="ignore", invalid="ignore"):
            inv = 1.0 / dirs                                         # (R,3)
            t0 = (self.boxes[None, :, 0, :] - origin) * inv[:, None, :]  # (R,M,3)
            t1 = (self.boxes[None, :, 1, :] - origin) * inv[:, None, :]
        tmin = np.nanmax(np.minimum(t0, t1), axis=2)
        tmax = np.nanmin(np.maximum(t0, t1), axis=2)
        hit = (tmax >= tmin) & (tmin > 1e-6)
        t = np.where(hit, tmin, np.inf).min(axis=1)
        keep = (t >= LIDAR_MIN_RANGE_M) & (t <= LIDAR_RANGE_M)
        points = dirs_lidar[keep] * t[keep, None]
        rng = np.random.default_rng(seed)
        return points + rng.normal(0.0, LIDAR_NOISE_M, points.shape)

    def scan_body(self, T_scene_body, seed, T_body_lidar=T_BODY_LIDAR):
        """(points_lidar, points_body) for a body pose in the scene frame."""
        lidar = self.scan(T_scene_body @ T_body_lidar, seed)
        return lidar, transform_points(T_body_lidar, lidar)


_WORLD = None


def make_world():
    global _WORLD
    if _WORLD is None:
        _WORLD = SyntheticWorld()
    return _WORLD


# ---------------------------------------------------------------------- trajectories
def loop_pose_at(arc_m, yaw_offset=0.0):
    """Scene pose on the rectangular loop at arc length ``arc_m`` (wraps)."""
    s = arc_m % LOOP_PERIMETER_M
    for a, b in zip(LOOP_CORNERS[:-1], LOOP_CORNERS[1:]):
        length = float(np.linalg.norm(b - a))
        if s <= length + 1e-9:
            p = a + (b - a) * (s / length)
            heading = math.atan2(b[1] - a[1], b[0] - a[0])
            return pose(p[0], p[1], wrap_angle(heading + yaw_offset))
        s -= length
    raise AssertionError("unreachable")


def _apply_yaw_bias(truth, bias_rad_per_m):
    """Integrate true relative motions with an extra yaw of bias*distance per step."""
    odom = [np.eye(4)]
    for k in range(1, len(truth)):
        rel = inverse(truth[k - 1]) @ truth[k]
        dist = float(np.linalg.norm(rel[:3, 3]))
        odom.append(odom[-1] @ rel @ pose(yaw=bias_rad_per_m * dist))
    return odom


def closed_loop_with_drift(drift_m=1.5, step_m=1.2, overlap_m=3.6):
    """Closed loop (60 m rectangle + ``overlap_m`` past the start) with odom drift.

    Returns a dict:
      scene:  list of T_scene_body (ground truth in the scene frame)
      truth:  list of T_world_body, world := body frame at the first keyframe
              (= the odom frame, so truth[0] == odom[0] == I)
      odom:   list of T_odom_body, drifted by a constant yaw bias per metre,
              calibrated so the position error at the loop-return keyframe
              (arc = 60 m) equals ``drift_m``
      return_index: index of the loop-return keyframe
      bias_rad_per_m, T_scene_world
    """
    n_loop = int(round(LOOP_PERIMETER_M / step_m))
    n = n_loop + int(round(overlap_m / step_m)) + 1
    scene = [loop_pose_at(k * step_m) for k in range(n)]
    T_scene_world = scene[0].copy()
    T_world_scene = inverse(T_scene_world)
    truth = [T_world_scene @ T for T in scene]

    def err(bias):
        odom = _apply_yaw_bias(truth, bias)
        return float(np.linalg.norm(odom[n_loop][:3, 3] - truth[n_loop][:3, 3]))

    lo, hi = 0.0, 0.01
    while err(hi) < drift_m:
        hi *= 2.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if err(mid) < drift_m else (lo, mid)
    bias = 0.5 * (lo + hi)
    return dict(scene=scene, truth=truth, odom=_apply_yaw_bias(truth, bias), return_index=n_loop,
                bias_rad_per_m=bias, T_scene_world=T_scene_world)


def relocalization_run(start_arc_m=5.0, yaw_offset=1.2, step_m=0.6, frames=10):
    """Poses for a relocalization run along the mapped loop.

    The body walks along the loop starting at ``start_arc_m`` with its heading
    rotated by ``yaw_offset`` from the mapping run; odom origin is the start
    point.  With the default mapping loop (world origin at arc 0) the true
    T_world_odom has a 5 m translation and a 1.2 rad yaw.
    Returns dict(scene, truth (T_world_body in the *mapping* world), odom,
    T_world_odom_true).
    """
    T_scene_world = loop_pose_at(0.0)
    T_world_scene = inverse(T_scene_world)
    scene = [loop_pose_at(start_arc_m + k * step_m, yaw_offset) for k in range(frames)]
    truth = [T_world_scene @ T for T in scene]
    odom = [inverse(truth[0]) @ T for T in truth]
    return dict(scene=scene, truth=truth, odom=odom, T_world_odom_true=truth[0].copy())


# --------------------------------------------------------------------------- spool
def _matrix_json(T):
    """Exactly what snapshot_bridge.cpp ``matrix()`` emits at setprecision(17):
    row-major nested rows ``[[r0c0,r0c1,...],[r1c0,...],...]``."""
    T = np.asarray(T, dtype=float).reshape(4, 4)
    return "[" + ",".join("[" + ",".join(format(float(v), ".17g") for v in row) + "]" for row in T) + "]"


def keyframe_files(epoch, sequence, stamp_ns, T_odom_body, T_body_lidar, points_lidar,
                   final_stage="lio"):
    """(pcd_text, metadata_text) byte-for-byte in the snapshot_bridge.cpp layout."""
    pts = np.asarray(points_lidar, dtype=np.float32).reshape(-1, 3)  # C++ PointXYZ is float
    n = len(pts)
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
              f"{n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n")
    body = "".join(f"{float(x):.9g} {float(y):.9g} {float(z):.9g}\n" for x, y, z in pts)
    meta = ('{"schema":"livo_keyframe/v1","epoch":' + str(int(epoch)) + ',"sequence":' + str(int(sequence))
            + ',"stamp_ns":' + str(int(stamp_ns)) + ',"T_odom_body":' + _matrix_json(T_odom_body)
            + ',"T_body_lidar":' + _matrix_json(T_body_lidar) + ',"final_stage":"' + final_stage
            + '","point_count":' + str(n) + ',"source_point_count":' + str(n)
            + ',"copy_ns":1234,"cloud_frame":"lidar"}\n')
    return header + body, meta


def write_spool_keyframe(spool, epoch, sequence, stamp_ns, T_odom_body, T_body_lidar, points_lidar,
                         *, ready=True, write_metadata=True):
    """Write one keyframe like SnapshotBridge::write_snapshot.

    Files go into ``tmp_<epoch>_<seq>`` first (cloud, then metadata), then the
    directory is renamed to ``kf_<epoch>_<seq>`` -- the rename is the "ready"
    signal.  ``ready=False`` leaves the temporary directory (a write in
    progress); ``write_metadata=False`` omits metadata.json.
    """
    spool = Path(spool)
    spool.mkdir(parents=True, exist_ok=True)
    suffix = f"{int(epoch)}_{int(sequence)}"
    temporary, ready_path = spool / ("tmp_" + suffix), spool / ("kf_" + suffix)
    temporary.mkdir()
    pcd, meta = keyframe_files(epoch, sequence, stamp_ns, T_odom_body, T_body_lidar, points_lidar)
    (temporary / "points_lidar.pcd").write_text(pcd, newline="\n")
    if write_metadata:
        (temporary / "metadata.json").write_text(meta, newline="\n")
    if not ready:
        return temporary
    os.rename(temporary, ready_path)
    return ready_path


def write_bridge_status(spool, epoch=0):
    """bridge_status.json (+ a stray .tmp) as the bridge leaves them in the spool root."""
    spool = Path(spool)
    row = {"schema": "livo_snapshot_bridge_status/v1", "epoch": epoch, "owner_stopped": False}
    (spool / "bridge_status.tmp").write_text(json.dumps(row) + "\n")
    (spool / "bridge_status.json").write_text(json.dumps(row) + "\n")


def write_spool(spool, frames, *, epoch=0, first_sequence=1, T_body_lidar=T_BODY_LIDAR):
    """frames: iterable of (stamp_ns, T_odom_body, points_lidar). Returns ready dirs."""
    return [write_spool_keyframe(spool, epoch, first_sequence + i, stamp, T, T_body_lidar, pts)
            for i, (stamp, T, pts) in enumerate(frames)]


# ----------------------------------------------------------------- keyframe streams
def keyframe_stream(scene, odom, *, seed=SEED, epoch=1, stamp0_ns=1_000_000_000, dt_ns=1_000_000_000,
                    T_body_lidar=T_BODY_LIDAR):
    """Plain dicts with every field of the frozen ``Keyframe`` (scans are simulated)."""
    world = make_world()
    rows = []
    for k, (T_scene_body, T_odom_body) in enumerate(zip(scene, odom)):
        lidar, _ = world.scan_body(T_scene_body, seed + k, T_body_lidar)
        rows.append(dict(key=(epoch, k + 1), stamp_ns=stamp0_ns + k * dt_ns,
                         T_odom_body=np.array(T_odom_body, dtype=float),
                         T_body_lidar=np.array(T_body_lidar, dtype=float), points_lidar=lidar))
    return rows
