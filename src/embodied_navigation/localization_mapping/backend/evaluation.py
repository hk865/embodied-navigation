"""Three-layer offline SLAM backend trajectory evaluator (ADR-0004).

Evaluates both the online causal trajectory (as emitted while navigating) and
the final graph-optimized trajectory against a ground-truth stream using pure
NumPy / SciPy vectorization:

1. Layer 1 (Scalar summary): ATE (3D/XY/Z), AOE (geodesic + gravity tilt +
   roll/pitch/yaw), RTE/ROE across distance (5/10/20 m) and time (1/5/10/100 s)
   windows, under both initial-window SE(3) alignment and global Umeyama SE(3).
2. Layer 2 (Per-keyframe evolution curves): prefix ATE/AOE/Z/tilt curves at
   every optimization event and pre/post-loop jump deltas.
3. Layer 3 (Per-second pose revision distribution): how each historical pose
   shifts across subsequent graph optimizations, aggregated in 1 s buckets
   (median, IQR, P5-P95, max, cumulative revision jitter).
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .backend import SlamBackend
from .keyframes import Keyframe, _read_ascii_pcd
from .map_store import MapStore
from .pose_graph import invert, project_so3
from .visual import VisualObservation


@dataclass(frozen=True)
class TruthTrajectory:
    """Ground-truth body trajectory sampled in world coordinates."""

    stamps_s: np.ndarray  # (M,) strictly increasing float64
    positions: np.ndarray  # (M, 3) float64
    rotations: Rotation  # length M

    def interpolate(self, query_stamps_s: np.ndarray) -> np.ndarray:
        """Interpolate world->body 4x4 SE(3) poses at ``query_stamps_s``."""
        q = np.asarray(query_stamps_s, dtype=np.float64)
        if q.ndim != 1 or len(q) == 0 or not np.isfinite(q).all():
            raise ValueError("query_stamps_s must be a non-empty finite 1D array")
        q_min = float(np.min(q))
        q_max = float(np.max(q))
        if q_min < self.stamps_s[0] - 1e-3 or q_max > self.stamps_s[-1] + 1e-3:
            raise ValueError(
                f"query timestamps [{q_min:.3f}, {q_max:.3f}] lie outside truth "
                f"range [{self.stamps_s[0]:.3f}, {self.stamps_s[-1]:.3f}]"
            )
        clamped = np.clip(q, self.stamps_s[0], self.stamps_s[-1])
        pos = np.column_stack(
            [np.interp(clamped, self.stamps_s, self.positions[:, axis]) for axis in range(3)]
        )
        slerp = Slerp(self.stamps_s, self.rotations)
        rots = slerp(clamped).as_matrix()
        poses = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], len(clamped), axis=0)
        poses[:, :3, :3] = rots
        poses[:, :3, 3] = pos
        return poses


def load_truth_stream(path: Path) -> TruthTrajectory:
    """Read ``capture/truth_scoring_stream.jsonl`` into a ``TruthTrajectory``."""
    stamps: list[float] = []
    positions: list[list[float]] = []
    quats_xyzw: list[list[float]] = []
    last_t = -float("inf")
    with Path(path).open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            t = float(row["time"])
            if not np.isfinite(t):
                raise ValueError(f"{path}:{line_no}: non-finite timestamp {t!r}")
            if t < last_t:
                raise ValueError(f"{path}:{line_no}: non-monotonic truth timestamp {t} < {last_t}")
            if t == last_t:
                continue
            last_t = t
            stamps.append(t)
            positions.append(row["position"])
            w, x, y, z = row["quat_wxyz"]
            quats_xyzw.append([x, y, z, w])
    if len(stamps) < 2:
        raise ValueError(f"{path}: need at least 2 truth samples, got {len(stamps)}")
    return TruthTrajectory(
        stamps_s=np.asarray(stamps, dtype=np.float64),
        positions=np.asarray(positions, dtype=np.float64),
        rotations=Rotation.from_quat(np.asarray(quats_xyzw, dtype=np.float64)),
    )


def _load_visual_npz(path: Path) -> VisualObservation | None:
    if not path.is_file():
        return None
    with np.load(path, allow_pickle=False) as data:
        raw_stamp = np.asarray(data["stamp_ns"]).item()
        if isinstance(raw_stamp, (bool, float)):
            raise ValueError(f"{path}: stamp_ns must be an integer, got {type(raw_stamp).__name__}")
        return VisualObservation(
            gray=np.asarray(data["gray"]),
            K=np.asarray(data["K"], dtype=np.float64),
            T_body_camera=np.asarray(data["T_body_camera"], dtype=np.float64),
            stamp_ns=int(raw_stamp),
        )


def load_imu_in_base_from_preflight(path: Path) -> np.ndarray:
    """Read ``imu_translation_in_base`` (3,) from a run's ``observer/preflight.json``."""
    doc = json.loads(Path(path).read_text(encoding="utf-8"))
    selected = doc.get("camera_calibration", {}).get("selected_fields", {})
    vec = selected.get("imu_translation_in_base")
    if vec is None:
        raise ValueError(f"{path}: missing camera_calibration.selected_fields.imu_translation_in_base")
    arr = np.asarray(vec, dtype=np.float64).reshape(3)
    if not np.isfinite(arr).all():
        raise ValueError(f"{path}: non-finite imu_translation_in_base {vec!r}")
    return arr


def _build_imu_to_base_transform(imu_in_base_xyz: Iterable[float] | None) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(T_imu_base, imu_in_base_vec)`` so ``T_world_base = T_world_imu @ T_imu_base``."""
    if imu_in_base_xyz is None:
        return np.eye(4, dtype=np.float64), np.zeros(3, dtype=np.float64)
    vec = np.asarray(list(imu_in_base_xyz), dtype=np.float64).reshape(3)
    if not np.isfinite(vec).all():
        raise ValueError(f"imu_in_base_xyz must contain 3 finite numbers, got {imu_in_base_xyz!r}")
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = -vec
    return T, vec


def apply_right_transform(poses: np.ndarray, T_right: np.ndarray) -> np.ndarray:
    """Apply a single 4x4 right transform ``T_right`` to a batch ``(N, 4, 4)`` of poses."""
    return np.einsum("nij,jk->nik", poses, T_right)


def load_map_keyframes(map_dir: Path, *, visual_dir: Path | None = None) -> list[Keyframe]:
    """Load keyframes in insertion order from a saved ``MapStore`` directory."""
    store = MapStore.load(Path(map_dir))
    kfs = list(store.keyframes.values())
    if visual_dir is not None:
        vdir = Path(visual_dir)
        kfs = [
            Keyframe(
                key=kf.key,
                stamp_ns=kf.stamp_ns,
                T_odom_body=kf.T_odom_body,
                T_body_lidar=kf.T_body_lidar,
                points_lidar=kf.points_lidar,
                visual=_load_visual_npz(vdir / f"{kf.stamp_ns}.npz"),
            )
            for kf in kfs
        ]
    return kfs


def load_input_json_keyframes(input_json: Path, *, visual_dir: Path | None = None) -> list[Keyframe]:
    """Load keyframes from an offline comparison ``input.json`` manifest."""
    path = Path(input_json)
    document = json.loads(path.read_text(encoding="utf-8"))
    frames = document["frames"] if isinstance(document, dict) else document
    keyframes: list[Keyframe] = []
    for row in frames:
        pts_path = Path(row["points"])
        if not pts_path.is_absolute():
            pts_path = path.parent / pts_path
        points = _read_ascii_pcd(pts_path)
        visual = None
        if row.get("visual_file"):
            vpath = Path(row["visual_file"])
            if not vpath.is_absolute():
                vpath = path.parent / vpath
            visual = _load_visual_npz(vpath)
        elif visual_dir is not None:
            visual = _load_visual_npz(Path(visual_dir) / f"{int(row['stamp_ns'])}.npz")
        keyframes.append(
            Keyframe(
                key=(int(row["key"][0]), int(row["key"][1])),
                stamp_ns=int(row["stamp_ns"]),
                T_odom_body=np.asarray(row["T_odom_body"], dtype=np.float64),
                T_body_lidar=np.asarray(row["T_body_lidar"], dtype=np.float64),
                points_lidar=points,
                visual=visual,
            )
        )
    return keyframes


@dataclass
class ReplayRecord:
    """Raw output of replaying a ``SlamBackend`` over a keyframe sequence."""

    stamps_s: np.ndarray  # (N,)
    keys: list[tuple[int, int]]
    frontend_poses: np.ndarray  # (N, 4, 4) T_odom_body
    online_adopted_poses: np.ndarray  # (N, 4, 4) C_adopted @ T_odom_body
    online_graph_poses: np.ndarray  # (N, 4, 4) graph.pose(k) at step k
    final_poses: np.ndarray  # (N, 4, 4) graph.pose(i) after final step
    snapshot_kf_indices: np.ndarray  # (S,) keyframe index at each snapshot
    snapshot_is_loop: np.ndarray  # (S,) bool whether snapshot followed a loop
    snapshot_pos: np.ndarray  # (S, N, 3) float32, NaN for future frames
    snapshot_quat_xyzw: np.ndarray  # (S, N, 4) float32, NaN for future frames
    step_wall_s: np.ndarray  # (N,) wall seconds per keyframe
    loop_events: list[dict] = field(default_factory=list)
    errors: int = 0
    missing_graph_nodes: int = 0
    total_wall_s: float = 0.0
    extra_stats: dict = field(default_factory=dict)
    snapshot_pre_pos: np.ndarray | None = None  # (S, N, 3) float32 pre-loop poses for same-prefix delta
    snapshot_pre_quat_xyzw: np.ndarray | None = None  # (S, N, 4) float32 pre-loop quats for same-prefix delta


def _poses_to_pos_quat(poses: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pos = poses[:, :3, 3].astype(np.float32)
    quat = Rotation.from_matrix(poses[:, :3, :3]).as_quat().astype(np.float32)
    return pos, quat


def _pos_quat_to_poses(pos: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    n = len(pos)
    out = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], n, axis=0)
    out[:, :3, :3] = Rotation.from_quat(quat_xyzw.astype(np.float64)).as_matrix()
    out[:, :3, 3] = pos.astype(np.float64)
    return out


def replay_backend(
    backend: SlamBackend,
    keyframes: list[Keyframe],
    *,
    snapshot_on_every_loop: bool = True,
    snapshot_stride: int = 50,
    on_progress: Callable[[int, int, dict], None] | None = None,
) -> ReplayRecord:
    """Replay ``keyframes`` sequentially through ``backend`` and record all 3 layers."""
    n = len(keyframes)
    if n == 0:
        raise ValueError("keyframes must be non-empty")

    stamps_s = np.array([kf.stamp_ns * 1e-9 for kf in keyframes], dtype=np.float64)
    keys = [kf.key for kf in keyframes]
    frontend_poses = np.stack([kf.T_odom_body for kf in keyframes], axis=0).astype(np.float64)
    online_adopted = np.zeros((n, 4, 4), dtype=np.float64)
    online_graph = np.zeros((n, 4, 4), dtype=np.float64)
    step_wall = np.zeros(n, dtype=np.float64)

    snap_indices: list[int] = []
    snap_is_loop: list[bool] = []
    snap_pos_list: list[np.ndarray] = []
    snap_quat_list: list[np.ndarray] = []
    snap_pre_pos_list: list[np.ndarray] = []
    snap_pre_quat_list: list[np.ndarray] = []
    loop_events: list[dict] = []
    errors = 0

    def _capture_snapshot(kf_idx: int, is_loop: bool, pre_poses: list[np.ndarray] | None) -> None:
        prefix = np.stack(backend.graph.poses[: kf_idx + 1], axis=0)
        pos_f32, quat_f32 = _poses_to_pos_quat(prefix)
        pos_row = np.full((n, 3), np.nan, dtype=np.float32)
        quat_row = np.full((n, 4), np.nan, dtype=np.float32)
        pos_row[: kf_idx + 1] = pos_f32
        quat_row[: kf_idx + 1] = quat_f32
        pre_pos_row = np.full((n, 3), np.nan, dtype=np.float32)
        pre_quat_row = np.full((n, 4), np.nan, dtype=np.float32)
        if is_loop and pre_poses is not None and len(pre_poses) > 0:
            k_pre = min(len(pre_poses), kf_idx)
            pre_stack = np.stack(pre_poses[:k_pre], axis=0)
            p_pre, q_pre = _poses_to_pos_quat(pre_stack)
            pre_pos_row[:k_pre] = p_pre
            pre_quat_row[:k_pre] = q_pre
        snap_indices.append(kf_idx)
        snap_is_loop.append(is_loop)
        snap_pos_list.append(pos_row)
        snap_quat_list.append(quat_row)
        snap_pre_pos_list.append(pre_pos_row)
        snap_pre_quat_list.append(pre_quat_row)

    t_total_0 = time.perf_counter()
    for idx, kf in enumerate(keyframes):
        # Shallow copy of pose references before ingest: PoseGraph never mutates
        # pose arrays in place, so this captures exact pre-optimization poses in O(N).
        pre_poses_ref = list(backend.graph.poses)
        t0 = time.perf_counter()
        event = backend.ingest(kf)
        step_wall[idx] = time.perf_counter() - t0
        if "error" in event:
            errors += 1

        C = backend.T_world_odom
        online_adopted[idx] = kf.T_odom_body if C is None else (C @ kf.T_odom_body)
        if kf.key in backend.graph.index:
            online_graph[idx] = backend.graph.pose(kf.key)
        else:
            online_graph[idx] = online_adopted[idx]

        is_loop = bool(event.get("loop", False))
        if is_loop or "loop_rejected" in event:
            loop_events.append(
                {
                    "kf_idx": idx,
                    "key": list(kf.key),
                    "stamp_s": float(stamps_s[idx]),
                    "loop": is_loop,
                    "loop_key": list(event["loop_key"]) if event.get("loop_key") is not None else None,
                    "adopted": bool(event.get("adopted", False)),
                    "loop_rejected": event.get("loop_rejected"),
                    "loop_rejected_kind": event.get("loop_rejected_kind"),
                }
            )

        need_snap = (
            idx == 0
            or idx == n - 1
            or (snapshot_on_every_loop and is_loop)
            or (snapshot_stride > 0 and idx % snapshot_stride == 0)
        )
        if need_snap and len(backend.graph.poses) == idx + 1:
            _capture_snapshot(idx, is_loop, pre_poses_ref if is_loop else None)

        if on_progress is not None:
            on_progress(idx, n, event)

    total_wall = time.perf_counter() - t_total_0
    missing_nodes = sum(1 for k in keys if k not in backend.graph.index)
    final_poses = np.stack(
        [
            backend.graph.pose(k) if k in backend.graph.index else online_graph[i]
            for i, k in enumerate(keys)
        ],
        axis=0,
    ).astype(np.float64)

    extra = {}
    if hasattr(backend, "observability_stats") and callable(backend.observability_stats):
        extra = dict(backend.observability_stats())
    elif hasattr(backend, "stats") and isinstance(backend.stats, dict):
        extra = dict(backend.stats)

    return ReplayRecord(
        stamps_s=stamps_s,
        keys=keys,
        frontend_poses=frontend_poses,
        online_adopted_poses=online_adopted,
        online_graph_poses=online_graph,
        final_poses=final_poses,
        snapshot_kf_indices=np.asarray(snap_indices, dtype=np.int32),
        snapshot_is_loop=np.asarray(snap_is_loop, dtype=bool),
        snapshot_pos=np.stack(snap_pos_list, axis=0),
        snapshot_quat_xyzw=np.stack(snap_quat_list, axis=0),
        step_wall_s=step_wall,
        loop_events=loop_events,
        errors=errors,
        missing_graph_nodes=missing_nodes,
        total_wall_s=float(total_wall),
        extra_stats=extra,
        snapshot_pre_pos=np.stack(snap_pre_pos_list, axis=0),
        snapshot_pre_quat_xyzw=np.stack(snap_pre_quat_list, axis=0),
    )


# Alignment helpers -----------------------------------------------------------


def compute_initial_se3_alignment(
    est_poses: np.ndarray,
    truth_poses: np.ndarray,
    stamps_s: np.ndarray,
    *,
    init_window_s: float = 5.0,
) -> np.ndarray:
    """Compute a fixed left SE(3) alignment ``A`` so ``A @ T_est ≈ T_truth`` on the initial window.

    When ``init_window_s <= 0`` or only one keyframe falls inside the window,
    falls back to exact first-keyframe alignment ``T_truth[0] @ inv(T_est[0])``.
    """
    if init_window_s <= 0.0:
        mask = np.zeros(len(stamps_s), dtype=bool)
        mask[0] = True
    else:
        mask = (stamps_s - stamps_s[0]) <= init_window_s
        if not np.any(mask):
            mask[0] = True

    idx = np.flatnonzero(mask)
    if len(idx) == 1:
        return truth_poses[idx[0]] @ invert(est_poses[idx[0]])

    # Left transforms A_i = T_truth_i @ inv(T_est_i); average rotations via SVD, translations by mean.
    R_sum = np.zeros((3, 3), dtype=np.float64)
    t_sum = np.zeros(3, dtype=np.float64)
    for i in idx:
        R_i = truth_poses[i, :3, :3] @ est_poses[i, :3, :3].T
        t_i = truth_poses[i, :3, 3] - R_i @ est_poses[i, :3, 3]
        R_sum += R_i
        t_sum += t_i
    R_avg = project_so3(R_sum)
    # Recompute translation using R_avg for exact geometric consistency.
    t_avg = np.mean(
        truth_poses[idx, :3, 3] - np.einsum("ij,nj->ni", R_avg, est_poses[idx, :3, 3]),
        axis=0,
    )
    A = np.eye(4, dtype=np.float64)
    A[:3, :3] = R_avg
    A[:3, 3] = t_avg
    return A


def compute_umeyama_se3_alignment(est_poses: np.ndarray, truth_poses: np.ndarray) -> np.ndarray:
    """Rigid SE(3) Umeyama alignment (scale=1) minimizing position sum of squared errors."""
    P = est_poses[:, :3, 3]
    Q = truth_poses[:, :3, 3]
    mu_p = P.mean(axis=0)
    mu_q = Q.mean(axis=0)
    Pc = P - mu_p
    Qc = Q - mu_q
    cov = Qc.T @ Pc / len(P)
    U, _, Vt = np.linalg.svd(cov)
    S = np.eye(3, dtype=np.float64)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    R = U @ S @ Vt
    t = mu_q - R @ mu_p
    A = np.eye(4, dtype=np.float64)
    A[:3, :3] = R
    A[:3, 3] = t
    return A


def apply_left_transform(A: np.ndarray, poses: np.ndarray) -> np.ndarray:
    """Apply a single 4x4 left transform ``A`` to a batch ``(N, 4, 4)`` of poses."""
    return np.einsum("ij,njk->nik", A, poses)


# Layer 1: Scalar metrics (ATE / AOE / RTE / ROE) ----------------------------


def _summary_stats(values: np.ndarray) -> dict[str, float | None]:
    arr = np.asarray(values, dtype=np.float64)
    if len(arr) == 0:
        return {"rmse": None, "mean": None, "median": None, "p95": None, "max": None}
    return {
        "rmse": float(np.sqrt(np.mean(arr * arr))),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(np.max(arr)),
    }


def _rotation_errors_deg(R_est: np.ndarray, R_truth: np.ndarray) -> dict[str, np.ndarray]:
    """Compute geodesic angle, yaw-invariant gravity tilt, and body-frame roll/pitch/yaw (deg)."""
    dR = np.einsum("nji,njk->nik", R_truth, R_est)  # R_truth^T @ R_est
    rot_err = Rotation.from_matrix(dR)
    angle_deg = np.degrees(rot_err.magnitude())

    # Yaw-invariant gravity tilt: angle between body-expressed world +Z vectors.
    # For R = R_world_body, world +Z in body frame is R^T @ [0,0,1] = R[2, :].
    g_est = R_est[:, 2, :]
    g_truth = R_truth[:, 2, :]
    sin_t = np.linalg.norm(np.cross(g_est, g_truth), axis=1)
    cos_t = np.einsum("ni,ni->n", g_est, g_truth)
    tilt_deg = np.degrees(np.arctan2(sin_t, cos_t))

    # Intrinsic XYZ Euler of R_truth^T @ R_est
    rpy_deg = np.abs(rot_err.as_euler("xyz", degrees=True))
    return {
        "angle_deg": angle_deg,
        "tilt_deg": tilt_deg,
        "roll_deg": rpy_deg[:, 0],
        "pitch_deg": rpy_deg[:, 1],
        "yaw_deg": rpy_deg[:, 2],
    }


def _relative_errors(
    est_poses: np.ndarray,
    truth_poses: np.ndarray,
    idx_a: np.ndarray,
    idx_b: np.ndarray,
) -> dict[str, float | int | None]:
    """Vectorized RTE (m) and ROE (deg) over index pairs ``(idx_a, idx_b)``."""
    if len(idx_a) == 0:
        return {
            "pairs": 0,
            "rte_rmse_m": None,
            "rte_mean_m": None,
            "rte_max_m": None,
            "roe_rmse_deg": None,
            "roe_mean_deg": None,
            "roe_max_deg": None,
        }

    Ra_e, ta_e = est_poses[idx_a, :3, :3], est_poses[idx_a, :3, 3]
    Rb_e, tb_e = est_poses[idx_b, :3, :3], est_poses[idx_b, :3, 3]
    Ra_t, ta_t = truth_poses[idx_a, :3, :3], truth_poses[idx_a, :3, 3]
    Rb_t, tb_t = truth_poses[idx_b, :3, :3], truth_poses[idx_b, :3, 3]

    # Relative poses: inv(Ta) @ Tb
    dR_e = np.einsum("nji,njk->nik", Ra_e, Rb_e)
    dt_e = np.einsum("nji,nj->ni", Ra_e, tb_e - ta_e)
    dR_t = np.einsum("nji,njk->nik", Ra_t, Rb_t)
    dt_t = np.einsum("nji,nj->ni", Ra_t, tb_t - ta_t)

    # Error transform: inv(dT_t) @ dT_e
    err_R = np.einsum("nji,njk->nik", dR_t, dR_e)
    err_t = np.einsum("nji,nj->ni", dR_t, dt_e - dt_t)

    rte = np.linalg.norm(err_t, axis=1)
    roe = np.degrees(Rotation.from_matrix(err_R).magnitude())
    return {
        "pairs": int(len(idx_a)),
        "rte_rmse_m": float(np.sqrt(np.mean(rte * rte))),
        "rte_mean_m": float(np.mean(rte)),
        "rte_max_m": float(np.max(rte)),
        "roe_rmse_deg": float(np.sqrt(np.mean(roe * roe))),
        "roe_mean_deg": float(np.mean(roe)),
        "roe_max_deg": float(np.max(roe)),
    }


def _window_pairs(axis_values: np.ndarray, delta: float, tolerance_ratio: float = 0.25) -> tuple[np.ndarray, np.ndarray]:
    """Find earliest index ``b > a`` where ``axis[b] - axis[a] >= delta`` within tolerance."""
    targets = axis_values + delta
    b = np.searchsorted(axis_values, targets, side="left")
    a = np.arange(len(axis_values))
    valid = b < len(axis_values)
    a, b = a[valid], b[valid]
    if len(a) == 0:
        return a, b
    actual = axis_values[b] - axis_values[a]
    ok = (b > a) & (actual <= delta * (1.0 + tolerance_ratio))
    return a[ok], b[ok]


def compute_trajectory_metrics(
    aligned_poses: np.ndarray,
    truth_poses: np.ndarray,
    stamps_s: np.ndarray,
    *,
    distance_windows_m: Iterable[float] = (5.0, 10.0, 20.0),
    time_windows_s: Iterable[float] = (1.0, 5.0, 10.0, 100.0),
) -> tuple[dict, dict[str, np.ndarray]]:
    """Compute Layer 1 ATE/AOE/RTE/ROE metrics and per-frame error series."""
    diff = aligned_poses[:, :3, 3] - truth_poses[:, :3, 3]
    err_3d = np.linalg.norm(diff, axis=1)
    err_xy = np.linalg.norm(diff[:, :2], axis=1)
    err_z = np.abs(diff[:, 2])
    signed_z = diff[:, 2]

    rot_errs = _rotation_errors_deg(aligned_poses[:, :3, :3], truth_poses[:, :3, :3])

    # Floor-height stratified Z RMSE (relative to truth start Z, consistent with 2026-11-17/20 reports)
    rel_z = truth_poses[:, 2, 3] - truth_poses[0, 2, 3]
    z_bands = {}
    for band_name, mask in (
        ("low_lt_0.4m", rel_z < 0.4),
        ("mid_0.4_2.9m", (rel_z >= 0.4) & (rel_z <= 2.9)),
        ("high_gt_2.9m", rel_z > 2.9),
    ):
        if np.any(mask):
            z_bands[band_name] = {
                "count": int(np.sum(mask)),
                "z_rmse_m": float(np.sqrt(np.mean(err_z[mask] ** 2))),
                "z_mean_signed_m": float(np.mean(signed_z[mask])),
                "z_max_abs_m": float(np.max(err_z[mask])),
            }

    # Cumulative truth path length for distance-window RTE/ROE
    step_dist = np.linalg.norm(np.diff(truth_poses[:, :3, 3], axis=0), axis=1)
    cum_dist = np.concatenate([[0.0], np.cumsum(step_dist)])

    rpe_dist = {}
    for d in distance_windows_m:
        ia, ib = _window_pairs(cum_dist, float(d))
        rpe_dist[f"{d:g}m"] = _relative_errors(aligned_poses, truth_poses, ia, ib)

    rpe_time = {}
    for dt in time_windows_s:
        ia, ib = _window_pairs(stamps_s, float(dt))
        rpe_time[f"{dt:g}s"] = _relative_errors(aligned_poses, truth_poses, ia, ib)

    summary = {
        "samples": int(len(stamps_s)),
        "path_length_m": float(cum_dist[-1]),
        "duration_s": float(stamps_s[-1] - stamps_s[0]),
        "ate_3d_m": _summary_stats(err_3d),
        "ate_xy_m": _summary_stats(err_xy),
        "ate_z_m": _summary_stats(err_z),
        "aoe_deg": _summary_stats(rot_errs["angle_deg"]),
        "tilt_deg": _summary_stats(rot_errs["tilt_deg"]),
        "roll_deg": _summary_stats(rot_errs["roll_deg"]),
        "pitch_deg": _summary_stats(rot_errs["pitch_deg"]),
        "yaw_deg": _summary_stats(rot_errs["yaw_deg"]),
        "z_bands": z_bands,
        "endpoint_translation_err_m": float(err_3d[-1]),
        "endpoint_rotation_err_deg": float(rot_errs["angle_deg"][-1]),
        "rpe_distance": rpe_dist,
        "rpe_time": rpe_time,
    }
    series = {
        "err_3d_m": err_3d,
        "err_xy_m": err_xy,
        "err_z_signed_m": signed_z,
        "aoe_deg": rot_errs["angle_deg"],
        "tilt_deg": rot_errs["tilt_deg"],
        "roll_deg": rot_errs["roll_deg"],
        "pitch_deg": rot_errs["pitch_deg"],
        "yaw_deg": rot_errs["yaw_deg"],
    }
    return summary, series


# Layer 2 & Layer 3: Evolution curves & per-second revision distribution ------


def compute_evolution_and_revisions(
    record: ReplayRecord,
    truth_poses: np.ndarray,
    A_align: np.ndarray,
    *,
    bucket_s: float = 1.0,
    imu_in_base_xyz: Iterable[float] | None = None,
) -> tuple[dict, dict[str, np.ndarray]]:
    """Compute Layer 2 per-snapshot evolution curves and Layer 3 per-second revision distributions."""
    if bucket_s <= 0.0 or not np.isfinite(bucket_s):
        raise ValueError(f"bucket_s must be finite and positive, got {bucket_s!r}")
    T_imu_base, _ = _build_imu_to_base_transform(imu_in_base_xyz)
    t_offset = T_imu_base[:3, 3]  # -imu_in_base_xyz
    has_offset = bool(np.any(t_offset != 0.0))

    s_count, n = record.snapshot_pos.shape[:2]
    R_A = A_align[:3, :3]
    t_A = A_align[:3, 3]
    truth_pos = truth_poses[:, :3, 3]
    truth_R = truth_poses[:, :3, :3]
    truth_g = truth_R[:, 2, :]  # (N, 3)

    # Allocate (S, N) error matrices filled with NaN for unobserved future frames
    err_3d_mat = np.full((s_count, n), np.nan, dtype=np.float32)
    err_z_mat = np.full((s_count, n), np.nan, dtype=np.float32)
    aoe_mat = np.full((s_count, n), np.nan, dtype=np.float32)
    tilt_mat = np.full((s_count, n), np.nan, dtype=np.float32)
    aligned_pos_mat = np.full((s_count, n, 3), np.nan, dtype=np.float32)

    evo_ate_rmse = np.zeros(s_count, dtype=np.float64)
    evo_ate_mean = np.zeros(s_count, dtype=np.float64)
    evo_ate_max = np.zeros(s_count, dtype=np.float64)
    evo_z_rmse = np.zeros(s_count, dtype=np.float64)
    evo_z_max = np.zeros(s_count, dtype=np.float64)
    evo_aoe_mean = np.zeros(s_count, dtype=np.float64)
    evo_aoe_max = np.zeros(s_count, dtype=np.float64)
    evo_tilt_max = np.zeros(s_count, dtype=np.float64)

    for s in range(s_count):
        k = int(record.snapshot_kf_indices[s]) + 1
        pos_raw = record.snapshot_pos[s, :k].astype(np.float64)
        quat_raw = record.snapshot_quat_xyzw[s, :k].astype(np.float64)
        R_raw = Rotation.from_quat(quat_raw).as_matrix()
        if has_offset:
            pos_raw = pos_raw + np.einsum("nij,j->ni", R_raw, t_offset)

        pos_aligned = pos_raw @ R_A.T + t_A
        R_aligned = np.einsum("ij,njk->nik", R_A, R_raw)

        diff = pos_aligned - truth_pos[:k]
        e3d = np.linalg.norm(diff, axis=1)
        ez = np.abs(diff[:, 2])
        dR = np.einsum("nji,njk->nik", truth_R[:k], R_aligned)
        aoe = np.degrees(Rotation.from_matrix(dR).magnitude())
        g_al = R_aligned[:, 2, :]
        tilt = np.degrees(
            np.arctan2(
                np.linalg.norm(np.cross(g_al, truth_g[:k]), axis=1),
                np.einsum("ni,ni->n", g_al, truth_g[:k]),
            )
        )

        aligned_pos_mat[s, :k] = pos_aligned.astype(np.float32)
        err_3d_mat[s, :k] = e3d.astype(np.float32)
        err_z_mat[s, :k] = ez.astype(np.float32)
        aoe_mat[s, :k] = aoe.astype(np.float32)
        tilt_mat[s, :k] = tilt.astype(np.float32)

        evo_ate_rmse[s] = float(np.sqrt(np.mean(e3d * e3d)))
        evo_ate_mean[s] = float(np.mean(e3d))
        evo_ate_max[s] = float(np.max(e3d))
        evo_z_rmse[s] = float(np.sqrt(np.mean(ez * ez)))
        evo_z_max[s] = float(np.max(ez))
        evo_aoe_mean[s] = float(np.mean(aoe))
        evo_aoe_max[s] = float(np.max(aoe))
        evo_tilt_max[s] = float(np.max(tilt))

    # Layer 2 same-prefix per-loop deltas: compare post-opt vs pre-opt on the EXACT same node set.
    loop_delta_ate: list[float] = []
    loop_delta_z: list[float] = []
    loop_delta_by_kf: dict[int, tuple[float, float]] = {}
    for s in range(s_count):
        if not record.snapshot_is_loop[s]:
            continue
        k_curr = int(record.snapshot_kf_indices[s]) + 1
        d_ate: float | None = None
        d_z: float | None = None
        if record.snapshot_pre_pos is not None:
            valid_pre = np.isfinite(record.snapshot_pre_pos[s, :, 0])
            k_pre = int(np.sum(valid_pre))
            if k_pre > 0:
                pre_pos = record.snapshot_pre_pos[s, :k_pre].astype(np.float64)
                if has_offset and record.snapshot_pre_quat_xyzw is not None:
                    pre_R = Rotation.from_quat(record.snapshot_pre_quat_xyzw[s, :k_pre].astype(np.float64)).as_matrix()
                    pre_pos = pre_pos + np.einsum("nij,j->ni", pre_R, t_offset)
                pre_aligned = pre_pos @ R_A.T + t_A
                pre_diff = pre_aligned - truth_pos[:k_pre]
                pre_e3d = np.linalg.norm(pre_diff, axis=1)
                pre_ez = np.abs(pre_diff[:, 2])
                post_e3d = err_3d_mat[s, :k_pre].astype(np.float64)
                post_ez = err_z_mat[s, :k_pre].astype(np.float64)
                d_ate = float(np.sqrt(np.mean(post_e3d * post_e3d)) - np.sqrt(np.mean(pre_e3d * pre_e3d)))
                d_z = float(np.sqrt(np.mean(post_ez * post_ez)) - np.sqrt(np.mean(pre_ez * pre_ez)))
        elif s > 0:
            k_common = min(int(record.snapshot_kf_indices[s - 1]) + 1, k_curr)
            if k_common > 0:
                pre_e3d = err_3d_mat[s - 1, :k_common].astype(np.float64)
                pre_ez = err_z_mat[s - 1, :k_common].astype(np.float64)
                post_e3d = err_3d_mat[s, :k_common].astype(np.float64)
                post_ez = err_z_mat[s, :k_common].astype(np.float64)
                d_ate = float(np.sqrt(np.mean(post_e3d * post_e3d)) - np.sqrt(np.mean(pre_e3d * pre_e3d)))
                d_z = float(np.sqrt(np.mean(post_ez * post_ez)) - np.sqrt(np.mean(pre_ez * pre_ez)))
        if d_ate is not None and d_z is not None:
            loop_delta_ate.append(d_ate)
            loop_delta_z.append(d_z)
            loop_delta_by_kf[int(record.snapshot_kf_indices[s])] = (d_ate, d_z)

    # Layer 3: Per-keyframe historical revision statistics starting from insertion pose
    # so the first revision between insertion and the next snapshot is never missed.
    online_base = apply_right_transform(record.online_graph_poses, T_imu_base) if has_offset else record.online_graph_poses
    init_aligned_pos = apply_left_transform(A_align, online_base)[:, :3, 3]
    init_err_3d = np.linalg.norm(init_aligned_pos - truth_pos, axis=1)

    kf_err_p05 = np.full(n, np.nan, dtype=np.float64)
    kf_err_p25 = np.full(n, np.nan, dtype=np.float64)
    kf_err_p50 = np.full(n, np.nan, dtype=np.float64)
    kf_err_p75 = np.full(n, np.nan, dtype=np.float64)
    kf_err_p95 = np.full(n, np.nan, dtype=np.float64)
    kf_err_max = np.full(n, np.nan, dtype=np.float64)
    kf_rev_max_shift = np.zeros(n, dtype=np.float64)
    kf_rev_max_step = np.zeros(n, dtype=np.float64)
    kf_rev_cum_jitter = np.zeros(n, dtype=np.float64)

    for i in range(n):
        valid_idx = np.flatnonzero(np.isfinite(err_3d_mat[:, i]))
        if len(valid_idx) == 0:
            continue
        vals = err_3d_mat[valid_idx, i].astype(np.float64)
        pts = aligned_pos_mat[valid_idx, i, :].astype(np.float64)
        if int(record.snapshot_kf_indices[valid_idx[0]]) > i:
            vals = np.concatenate(([init_err_3d[i]], vals))
            pts = np.vstack((init_aligned_pos[i : i + 1], pts))
        p05, p25, p50, p75, p95 = np.percentile(vals, [5, 25, 50, 75, 95])
        kf_err_p05[i] = p05
        kf_err_p25[i] = p25
        kf_err_p50[i] = p50
        kf_err_p75[i] = p75
        kf_err_p95[i] = p95
        kf_err_max[i] = float(np.max(vals))

        if len(pts) >= 2:
            shifts = np.linalg.norm(pts - pts[0], axis=1)
            step_norms = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            kf_rev_max_shift[i] = float(np.max(shifts))
            kf_rev_max_step[i] = float(np.max(step_norms))
            kf_rev_cum_jitter[i] = float(np.sum(step_norms))

    # Aggregate into 1-second buckets
    rel_t = record.stamps_s - record.stamps_s[0]
    bucket_ids = np.floor(rel_t / bucket_s).astype(np.int64)
    unique_buckets = np.unique(bucket_ids)
    b_t = np.zeros(len(unique_buckets), dtype=np.float64)
    b_p05 = np.zeros(len(unique_buckets), dtype=np.float64)
    b_p25 = np.zeros(len(unique_buckets), dtype=np.float64)
    b_p50 = np.zeros(len(unique_buckets), dtype=np.float64)
    b_p75 = np.zeros(len(unique_buckets), dtype=np.float64)
    b_p95 = np.zeros(len(unique_buckets), dtype=np.float64)
    b_max = np.zeros(len(unique_buckets), dtype=np.float64)
    b_jitter = np.zeros(len(unique_buckets), dtype=np.float64)
    b_max_shift = np.zeros(len(unique_buckets), dtype=np.float64)

    for bi, bid in enumerate(unique_buckets):
        cols = np.flatnonzero(bucket_ids == bid)
        sub = err_3d_mat[:, cols]
        flat = sub[np.isfinite(sub)].astype(np.float64)
        b_t[bi] = float(bid) * bucket_s
        if len(flat) > 0:
            b_p05[bi], b_p25[bi], b_p50[bi], b_p75[bi], b_p95[bi] = np.percentile(flat, [5, 25, 50, 75, 95])
            b_max[bi] = float(np.max(flat))
        b_jitter[bi] = float(np.mean(kf_rev_cum_jitter[cols]))
        b_max_shift[bi] = float(np.max(kf_rev_max_shift[cols]))

    summary = {
        "layer2_evolution": {
            "snapshots": int(s_count),
            "loop_snapshots": int(np.sum(record.snapshot_is_loop)),
            "same_prefix_loop_evals": int(len(loop_delta_ate)),
            "peak_prefix_ate_rmse_m": float(np.max(evo_ate_rmse)),
            "peak_prefix_ate_max_m": float(np.max(evo_ate_max)),
            "peak_prefix_z_rmse_m": float(np.max(evo_z_rmse)),
            "peak_prefix_tilt_max_deg": float(np.max(evo_tilt_max)),
            "final_prefix_ate_rmse_m": float(evo_ate_rmse[-1]),
            "worst_loop_ate_jump_m": float(max(loop_delta_ate)) if loop_delta_ate else 0.0,
            "best_loop_ate_drop_m": float(min(loop_delta_ate)) if loop_delta_ate else 0.0,
            "negative_loop_steps": int(sum(1 for d in loop_delta_ate if d > 0.01)),
            "positive_loop_steps": int(sum(1 for d in loop_delta_ate if d < -0.01)),
        },
        "layer3_revisions": {
            "bucket_s": float(bucket_s),
            "buckets": int(len(unique_buckets)),
            "median_iqr_m": float(np.nanmedian(kf_err_p75 - kf_err_p25)),
            "p95_iqr_m": float(np.nanpercentile(kf_err_p75 - kf_err_p25, 95)),
            "mean_max_shift_m": float(np.mean(kf_rev_max_shift)),
            "max_shift_m": float(np.max(kf_rev_max_shift)),
            "max_single_step_shift_m": float(np.max(kf_rev_max_step)),
            "mean_cum_jitter_m": float(np.mean(kf_rev_cum_jitter)),
            "max_cum_jitter_m": float(np.max(kf_rev_cum_jitter)),
        },
    }
    arrays = {
        "snapshot_kf_indices": record.snapshot_kf_indices,
        "snapshot_stamps_s": record.stamps_s[record.snapshot_kf_indices],
        "snapshot_is_loop": record.snapshot_is_loop,
        "evo_ate_rmse_m": evo_ate_rmse,
        "evo_ate_mean_m": evo_ate_mean,
        "evo_ate_max_m": evo_ate_max,
        "evo_z_rmse_m": evo_z_rmse,
        "evo_z_max_m": evo_z_max,
        "evo_aoe_mean_deg": evo_aoe_mean,
        "evo_aoe_max_deg": evo_aoe_max,
        "evo_tilt_max_deg": evo_tilt_max,
        "same_prefix_loop_delta_ate_m": np.asarray(loop_delta_ate, dtype=np.float64),
        "same_prefix_loop_delta_z_m": np.asarray(loop_delta_z, dtype=np.float64),
        "bucket_t_s": b_t,
        "bucket_err_p05_m": b_p05,
        "bucket_err_p25_m": b_p25,
        "bucket_err_p50_m": b_p50,
        "bucket_err_p75_m": b_p75,
        "bucket_err_p95_m": b_p95,
        "bucket_err_max_m": b_max,
        "bucket_mean_jitter_m": b_jitter,
        "bucket_max_shift_m": b_max_shift,
        "kf_rev_max_shift_m": kf_rev_max_shift,
        "kf_rev_max_step_m": kf_rev_max_step,
        "kf_rev_cum_jitter_m": kf_rev_cum_jitter,
    }
    for ev in record.loop_events:
        if ev.get("loop") and ev.get("kf_idx") in loop_delta_by_kf:
            d_ate, d_z = loop_delta_by_kf[ev["kf_idx"]]
            ev["same_prefix_delta_ate_m"] = d_ate
            ev["same_prefix_delta_z_m"] = d_z
    return summary, arrays


def evaluate_replay(
    record: ReplayRecord,
    truth: TruthTrajectory,
    *,
    init_window_s: float = 5.0,
    bucket_s: float = 1.0,
    imu_in_base_xyz: Iterable[float] | None = None,
) -> tuple[dict, dict[str, np.ndarray]]:
    """Run full ADR-0004 3-layer evaluation on a ``ReplayRecord``."""
    truth_poses = truth.interpolate(record.stamps_s)
    T_imu_base, imu_vec = _build_imu_to_base_transform(imu_in_base_xyz)
    has_offset = bool(np.any(imu_vec != 0.0))

    raw_trajectories = {
        "frontend": record.frontend_poses,
        "online_adopted": record.online_adopted_poses,
        "online_graph": record.online_graph_poses,
        "final": record.final_poses,
    }
    trajectories = {
        name: (apply_right_transform(poses, T_imu_base) if has_offset else poses)
        for name, poses in raw_trajectories.items()
    }

    # Fixed initial-window alignment computed ONCE on the raw frontend trajectory
    # (already transformed to the truth body origin) so every backend variant shares
    # the exact same world alignment.
    A_init = compute_initial_se3_alignment(
        trajectories["frontend"], truth_poses, record.stamps_s, init_window_s=init_window_s
    )

    layer1_init: dict[str, dict] = {}
    layer1_umeyama: dict[str, dict] = {}
    arrays: dict[str, np.ndarray] = {
        "stamps_s": record.stamps_s,
        "truth_poses": truth_poses,
        "A_init": A_init,
        "imu_in_base_xyz": imu_vec,
    }

    for name, raw_poses in trajectories.items():
        aligned_init = apply_left_transform(A_init, raw_poses)
        m_init, s_init = compute_trajectory_metrics(aligned_init, truth_poses, record.stamps_s)
        layer1_init[name] = m_init
        arrays[f"{name}_aligned_init_poses"] = aligned_init
        for k, v in s_init.items():
            arrays[f"{name}_init_{k}"] = v

        A_um = compute_umeyama_se3_alignment(raw_poses, truth_poses)
        aligned_um = apply_left_transform(A_um, raw_poses)
        m_um, _ = compute_trajectory_metrics(aligned_um, truth_poses, record.stamps_s)
        layer1_umeyama[name] = m_um

    l23_summary, l23_arrays = compute_evolution_and_revisions(
        record, truth_poses, A_init, bucket_s=bucket_s, imu_in_base_xyz=imu_in_base_xyz
    )
    arrays.update(l23_arrays)

    loops_accepted = sum(1 for e in record.loop_events if e["loop"])
    loops_rejected_tilt = sum(
        1 for e in record.loop_events if e.get("loop_rejected") is not None and e.get("loop_rejected_kind") == "tilt"
    )
    loops_rejected_odom = sum(
        1 for e in record.loop_events if e.get("loop_rejected") is not None and e.get("loop_rejected_kind") != "tilt"
    )
    summary = {
        "runtime": {
            "keyframes": int(len(record.stamps_s)),
            "loops_accepted": int(loops_accepted),
            "loops_rejected_odom": int(loops_rejected_odom),
            "loops_rejected_tilt": int(loops_rejected_tilt),
            "errors": int(record.errors),
            "missing_graph_nodes": int(record.missing_graph_nodes),
            "total_wall_s": float(record.total_wall_s),
            "step_wall_mean_ms": float(np.mean(record.step_wall_s) * 1000.0),
            "step_wall_p95_ms": float(np.percentile(record.step_wall_s, 95) * 1000.0),
            "step_wall_max_ms": float(np.max(record.step_wall_s) * 1000.0),
            "extra_stats": record.extra_stats,
        },
        "alignment": {
            "init_window_s": float(init_window_s),
            "eval_origin": "base_via_imu_in_base" if has_offset else "estimated_origin",
            "imu_in_base_xyz": imu_vec.tolist(),
            "A_init": A_init.tolist(),
        },
        "layer1_init_se3": layer1_init,
        "layer1_umeyama_se3": layer1_umeyama,
        **l23_summary,
    }
    return summary, arrays


def write_tum_trajectory(path: Path, stamps_s: np.ndarray, poses: np.ndarray) -> None:
    """Export poses in standard TUM format: ``timestamp tx ty tz qx qy qz qw``."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    quats = Rotation.from_matrix(poses[:, :3, :3]).as_quat()  # xyzw
    lines = [
        f"{stamps_s[i]:.6f} {poses[i, 0, 3]:.6f} {poses[i, 1, 3]:.6f} {poses[i, 2, 3]:.6f} "
        f"{quats[i, 0]:.6f} {quats[i, 1]:.6f} {quats[i, 2]:.6f} {quats[i, 3]:.6f}\n"
        for i in range(len(stamps_s))
    ]
    path.write_text("".join(lines), encoding="utf-8")


def save_evaluation_bundle(
    output_dir: Path,
    summary: dict,
    arrays: dict[str, np.ndarray],
    *,
    export_tum: bool = True,
    loop_events: list[dict] | None = None,
) -> Path:
    """Persist ``summary.json``, compressed ``metrics.npz``, optional ``loop_events.json`` and TUM files."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8"
    )
    if loop_events is not None:
        (output_dir / "loop_events.json").write_text(
            json.dumps(loop_events, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8"
        )
    np.savez_compressed(output_dir / "metrics.npz", **arrays)
    if export_tum:
        tum_dir = output_dir / "tum"
        stamps = arrays["stamps_s"]
        write_tum_trajectory(tum_dir / "truth.tum", stamps, arrays["truth_poses"])
        for name in ("frontend", "online_adopted", "online_graph", "final"):
            key = f"{name}_aligned_init_poses"
            if key in arrays:
                write_tum_trajectory(tum_dir / f"{name}.tum", stamps, arrays[key])
    return output_dir
