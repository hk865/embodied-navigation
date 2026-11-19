"""200 Hz simulation-truth pose buffer and vectorized RTX LiDAR deskewing.

Pure numpy / stdlib module (no Isaac or ROS imports at import time) so all
extrinsic, interpolation, and deskew contracts are directly testable on CPU.

Quaternion Convention (CRITICAL — never mix wxyz vs xyzw):
- ``common/geometry.py::quaternion_matrix(w, x, y, z)`` takes ``w, x, y, z``.
- ``robot/unitree/policy/native/observation.py::quaternion_rotation(quat_wxyz)``
  takes ``[w, x, y, z]`` and returns ``R_wb`` (world-from-body, i.e.
  ``p_world = R_wb @ p_body + p_wb``).
- ``capture/physics.py`` reads ``art.get_world_poses()`` as ``wq, xq, yq, zq = quat``
  and passes it directly to ``quaternion_rotation(quat)`` — i.e., Isaac
  articulation world poses are ``wxyz``.
- Every quaternion stored or returned by this module is strictly ``wxyz``
  (``[w, x, y, z]``), representing world-from-body / world-from-IMU rotation.
  When serializing to ROS ``geometry_msgs/Quaternion``, components are assigned
  explicitly by field name (``w, x, y, z``).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from numbers import Integral

import numpy as np

from embodied_navigation.common.geometry import quaternion_matrix

TRUTH_DESKEWED_CLOUD_TOPIC = "/dynanav/truth/cloud_deskewed"
TRUTH_LIO_ODOM_TOPIC = "/dynanav/truth/imu_odometry"
TRUTH_VIO_ODOM_TOPIC = "/dynanav/truth/control_odometry"
TRUTH_FRAME_ID = "camera_init"
TRUTH_CHILD_FRAME_ID = "go2_imu"
TRUTH_POSE_SOURCE = "simulation_truth"

TRUTH_DESKEWED_FIELD_NAMES = ("x", "y", "z", "intensity", "curvature")
TRUTH_DESKEWED_DTYPE = np.dtype([(name, "<f4") for name in TRUTH_DESKEWED_FIELD_NAMES])


def _require_int_ns(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{name} must be an integer nanosecond timestamp")
    return int(value)


def _normalize_quat_wxyz(quat_wxyz) -> np.ndarray:
    q = np.asarray(quat_wxyz, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("quat_wxyz must contain 4 finite numbers")
    norm = float(np.linalg.norm(q))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("quat_wxyz must be unit length within 1e-3")
    return q / norm


def _rotate_vectors_wxyz(quats_wxyz: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    """Rotate N 3D vectors by N unit ``wxyz`` quaternions (vectorized)."""
    w = quats_wxyz[:, 0:1]
    u = quats_wxyz[:, 1:4]
    t_cross = 2.0 * np.cross(u, vectors)
    return vectors + w * t_cross + np.cross(u, t_cross)


def _slerp_wxyz_batch(
    q0: np.ndarray, q1: np.ndarray, alpha: np.ndarray
) -> np.ndarray:
    """Vectorized shortest-arc SLERP on unit ``wxyz`` quaternions."""
    q1_aligned = q1.copy()
    dot = np.sum(q0 * q1_aligned, axis=1)
    flip = dot < 0.0
    if np.any(flip):
        q1_aligned[flip] = -q1_aligned[flip]
        dot[flip] = -dot[flip]
    dot = np.clip(dot, 0.0, 1.0)
    small = dot > (1.0 - 1e-7)
    safe_dot = np.where(small, 1.0, dot)
    theta = np.arccos(safe_dot)
    sin_theta = np.sin(theta)
    safe_sin = np.where(small, 1.0, sin_theta)
    w0 = np.where(small, 1.0 - alpha, np.sin((1.0 - alpha) * theta) / safe_sin)
    w1 = np.where(small, alpha, np.sin(alpha * theta) / safe_sin)
    out = w0[:, None] * q0 + w1[:, None] * q1_aligned
    out /= np.linalg.norm(out, axis=1, keepdims=True)
    return out


class TruthPoseBuffer:
    """Fixed-capacity ring buffer storing 200 Hz ``go2_imu`` world poses.

    Each physics step records ``go2_base`` world pose ``(pos_wb, quat_wb_wxyz)``
    and converts it once using the fixed IMU mount in the base frame:

        R_wi = R_wb,   p_wi = p_wb + R_wb @ imu_mount

    ``go2_imu`` shares the same axes orientation as ``go2_base`` (`R_wi = R_wb`).
    """

    def __init__(
        self,
        imu_mount,
        *,
        capacity: int = 400,
        max_clamp_ns: int = 2_000_000,
    ) -> None:
        mount = np.asarray(imu_mount, dtype=np.float64)
        if mount.shape != (3,) or not np.isfinite(mount).all():
            raise ValueError("imu_mount must contain 3 finite numbers")
        if isinstance(capacity, bool) or not isinstance(capacity, Integral) or int(capacity) < 1:
            raise ValueError("capacity must be a positive integer")
        if isinstance(max_clamp_ns, bool) or not isinstance(max_clamp_ns, Integral) or int(max_clamp_ns) < 0:
            raise ValueError("max_clamp_ns must be a nonnegative integer")
        self.imu_mount = mount.copy()
        self.capacity = int(capacity)
        self.max_clamp_ns = int(max_clamp_ns)
        self._stamps_ns: deque[int] = deque(maxlen=self.capacity)
        self._positions_wi: deque[np.ndarray] = deque(maxlen=self.capacity)
        self._quats_wi_wxyz: deque[np.ndarray] = deque(maxlen=self.capacity)

    def __len__(self) -> int:
        return len(self._stamps_ns)

    @property
    def first_stamp_ns(self) -> int | None:
        return self._stamps_ns[0] if self._stamps_ns else None

    @property
    def last_stamp_ns(self) -> int | None:
        return self._stamps_ns[-1] if self._stamps_ns else None

    def covers(self, start_ns: int, end_ns: int) -> bool:
        """Return True iff the buffer brackets ``[start_ns, end_ns]`` on both sides."""
        if not self._stamps_ns:
            return False
        start = _require_int_ns(start_ns, "start_ns")
        end = _require_int_ns(end_ns, "end_ns")
        return self._stamps_ns[0] <= start <= end <= self._stamps_ns[-1]

    def record(
        self, stamp_ns: int, pos_wb: np.ndarray, quat_wb_wxyz: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Validate monotonic timestamp & finite pose, store ``go2_imu`` world pose."""
        stamp = _require_int_ns(stamp_ns, "stamp_ns")
        if stamp < 0:
            raise ValueError("stamp_ns must be nonnegative")
        if self._stamps_ns and stamp <= self._stamps_ns[-1]:
            raise ValueError(
                f"stamp_ns must be strictly increasing (got {stamp} <= {self._stamps_ns[-1]})"
            )
        pos = np.asarray(pos_wb, dtype=np.float64)
        if pos.shape != (3,) or not np.isfinite(pos).all():
            raise ValueError("pos_wb must contain 3 finite numbers")
        quat_unit = _normalize_quat_wxyz(quat_wb_wxyz)
        w, x, y, z = quat_unit
        r_wb = quaternion_matrix(w, x, y, z)
        pos_wi = pos + r_wb @ self.imu_mount
        quat_wi_wxyz = quat_unit.copy()
        self._stamps_ns.append(stamp)
        self._positions_wi.append(pos_wi)
        self._quats_wi_wxyz.append(quat_wi_wxyz)
        return pos_wi.copy(), quat_wi_wxyz.copy()

    def interpolate_imu_poses(
        self, query_ns_array
    ) -> tuple[np.ndarray, np.ndarray]:
        """Vectorized linear-position + shortest-arc-SLERP query for ``(N,)`` timestamps."""
        if not self._stamps_ns:
            raise ValueError("TruthPoseBuffer is empty")
        queries = np.asarray(query_ns_array, dtype=np.int64)
        if queries.ndim != 1:
            raise ValueError("query_ns_array must be 1D")
        if queries.size == 0:
            return (
                np.empty((0, 3), dtype=np.float64),
                np.empty((0, 4), dtype=np.float64),
            )
        stamps = np.asarray(self._stamps_ns, dtype=np.int64)
        t_min, t_max = int(stamps[0]), int(stamps[-1])
        q_min, q_max = int(queries.min()), int(queries.max())
        if q_min < t_min - self.max_clamp_ns or q_max > t_max + self.max_clamp_ns:
            raise ValueError(
                f"Query range [{q_min}, {q_max}] exceeds buffer [{t_min}, {t_max}] "
                f"by more than {self.max_clamp_ns} ns"
            )
        clamped = np.clip(queries, t_min, t_max)
        positions = np.asarray(self._positions_wi, dtype=np.float64)
        quats = np.asarray(self._quats_wi_wxyz, dtype=np.float64)
        if len(stamps) == 1:
            n = len(clamped)
            return np.repeat(positions, n, axis=0), np.repeat(quats, n, axis=0)
        idx1 = np.clip(
            np.searchsorted(stamps, clamped, side="right"), 1, len(stamps) - 1
        )
        idx0 = idx1 - 1
        t0 = stamps[idx0]
        t1 = stamps[idx1]
        alpha = (clamped - t0).astype(np.float64) / (t1 - t0).astype(np.float64)
        pos_out = (1.0 - alpha)[:, None] * positions[idx0] + alpha[:, None] * positions[idx1]
        quat_out = _slerp_wxyz_batch(quats[idx0], quats[idx1], alpha)
        return pos_out, quat_out

    def interpolate_imu_pose(self, query_ns: int) -> tuple[np.ndarray, np.ndarray]:
        """Interpolate a single ``go2_imu`` world pose ``(pos_wi, quat_wi_wxyz)``."""
        stamp = _require_int_ns(query_ns, "query_ns")
        pos, quat = self.interpolate_imu_poses(np.array([stamp], dtype=np.int64))
        return pos[0], quat[0]


@dataclass(frozen=True)
class TruthDeskewedScan:
    """World-frame truth-deskewed RTX scan paired with its ``go2_imu`` pose."""

    records: np.ndarray
    points_world: np.ndarray
    curvature_ms: np.ndarray
    cloud_stamp_ns: int
    pair_pos_wi: np.ndarray
    pair_quat_wi_wxyz: np.ndarray


def deskew_rtx_packet_to_world(
    packet, pose_buffer: TruthPoseBuffer, now_ns: int
) -> TruthDeskewedScan:
    """Deskew an RTX ``LidarPacket`` (in ``go2_imu``) into ``camera_init`` world frame.

    Vectorized across all scan points using ``TruthPoseBuffer.interpolate_imu_poses``:
    ``t_abs_ns = int(packet.stamp_ns) + packet.records["t"].astype(np.int64)``
    ``p_world_k = R_wi(t_k) @ p_imu_k + p_wi(t_k)``

    Strict stamp & coverage contract (no silent truncation or boundary clamping):
    - ``cloud_stamp_ns == last_retained_ns`` (exact firing end of the retained scan)
    - ``last_retained_ns <= now_ns`` (published ``/clock`` covers the full scan)
    - ``pose_buffer.covers(first_retained_ns, last_retained_ns)`` (two-sided
      200 Hz pose bracketing for every point in the packet)
    """
    clock_limit_ns = _require_int_ns(now_ns, "now_ns")
    if len(packet.records) == 0:
        raise ValueError("Cannot deskew empty RTX packet")
    t_abs_ns = int(packet.stamp_ns) + packet.records["t"].astype(np.int64)
    first_retained_ns = int(t_abs_ns.min())
    last_retained_ns = int(t_abs_ns.max())
    receipt_last = packet.receipt.get("last_retained_ns")
    if receipt_last is None:
        raise ValueError("RTX packet receipt lacks last_retained_ns")
    if int(receipt_last) != last_retained_ns:
        raise ValueError(
            f"RTX packet last_retained_ns {int(receipt_last)} disagrees with "
            f"record max timestamp {last_retained_ns}"
        )
    if last_retained_ns > clock_limit_ns:
        raise ValueError(
            f"RTX scan last_retained_ns {last_retained_ns} exceeds published /clock {clock_limit_ns}"
        )
    if not pose_buffer.covers(first_retained_ns, last_retained_ns):
        raise ValueError(
            f"TruthPoseBuffer [{pose_buffer.first_stamp_ns}, {pose_buffer.last_stamp_ns}] "
            f"does not cover scan interval [{first_retained_ns}, {last_retained_ns}]"
        )

    pos_pts, quat_pts = pose_buffer.interpolate_imu_poses(t_abs_ns)
    p_imu = np.column_stack(
        [packet.records["x"], packet.records["y"], packet.records["z"]]
    ).astype(np.float64)
    points_world = _rotate_vectors_wxyz(quat_pts, p_imu) + pos_pts

    cloud_stamp_ns = last_retained_ns
    pair_pos_wi, pair_quat_wi_wxyz = pose_buffer.interpolate_imu_pose(cloud_stamp_ns)

    curvature_ms = packet.records["t"].astype(np.float32) * np.float32(1e-6)
    records = np.empty(len(points_world), dtype=TRUTH_DESKEWED_DTYPE)
    records["x"] = points_world[:, 0].astype(np.float32)
    records["y"] = points_world[:, 1].astype(np.float32)
    records["z"] = points_world[:, 2].astype(np.float32)
    records["intensity"] = packet.records["intensity"].astype(np.float32)
    records["curvature"] = curvature_ms

    return TruthDeskewedScan(
        records=records,
        points_world=points_world,
        curvature_ms=curvature_ms,
        cloud_stamp_ns=cloud_stamp_ns,
        pair_pos_wi=pair_pos_wi,
        pair_quat_wi_wxyz=pair_quat_wi_wxyz,
    )


def truth_odometry_message(
    stamp_ns: int,
    pos_wi: np.ndarray,
    quat_wi_wxyz: np.ndarray,
    *,
    frame_id: str = TRUTH_FRAME_ID,
    child_frame_id: str = TRUTH_CHILD_FRAME_ID,
):
    """Build a ROS ``nav_msgs/Odometry`` message from ``go2_imu`` world pose (wxyz)."""
    from nav_msgs.msg import Odometry

    stamp = _require_int_ns(stamp_ns, "stamp_ns")
    msg = Odometry()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.pose.pose.position.x = float(pos_wi[0])
    msg.pose.pose.position.y = float(pos_wi[1])
    msg.pose.pose.position.z = float(pos_wi[2])
    msg.pose.pose.orientation.w = float(quat_wi_wxyz[0])
    msg.pose.pose.orientation.x = float(quat_wi_wxyz[1])
    msg.pose.pose.orientation.y = float(quat_wi_wxyz[2])
    msg.pose.pose.orientation.z = float(quat_wi_wxyz[3])
    return msg


def truth_deskewed_cloud_message(
    scan: TruthDeskewedScan, *, frame_id: str = TRUTH_FRAME_ID
):
    """Build a ROS ``sensor_msgs/PointCloud2`` (5 x float32, point_step=20)."""
    from sensor_msgs.msg import PointCloud2, PointField

    msg = PointCloud2()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
        scan.cloud_stamp_ns, 1_000_000_000
    )
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(scan.records)
    msg.fields = [
        PointField(
            name=name,
            offset=TRUTH_DESKEWED_DTYPE.fields[name][1],
            datatype=7,
            count=1,
        )
        for name in TRUTH_DESKEWED_FIELD_NAMES
    ]
    msg.point_step = TRUTH_DESKEWED_DTYPE.itemsize
    msg.row_step = msg.width * msg.point_step
    msg.is_dense = True
    msg.is_bigendian = False
    msg.data = scan.records.tobytes()
    return msg
