"""Keyframes exported by the FAST-LIVO2 ``snapshot_bridge`` spool.

Frames: ``odom`` is the frontend's fixed odometry frame (``camera_init``),
``body`` is the IMU/body frame whose pose the frontend estimates, and
``lidar`` is the sensor frame of the stored cloud. ``T_a_b`` maps points from
frame ``b`` into frame ``a`` (4x4 homogeneous, float64).

Spool layout (``snapshot_bridge.cpp``): each keyframe is written into
``tmp_<epoch>_<sequence>/`` and then renamed to ``kf_<epoch>_<sequence>/``, so
only ``kf_*`` directories are complete. Each holds ``metadata.json``
(schema ``livo_keyframe/v1``, matrices as row-major nested lists) and an ASCII
``points_lidar.pcd`` with ``x y z`` rows.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .visual import VisualObservation

SCHEMA = "livo_keyframe/v1"
READY_NAME = re.compile(r"kf_(\d+)_(\d+)")
MAX_VISUAL_IMAGE_DIM = 4096


@dataclass(frozen=True)
class Keyframe:
    key: tuple[int, int]  # (epoch, sequence)
    stamp_ns: int
    T_odom_body: np.ndarray  # 4x4
    T_body_lidar: np.ndarray  # 4x4
    points_lidar: np.ndarray  # (N,3) float64, lidar frame
    visual: VisualObservation | None = None

    def points_body(self) -> np.ndarray:
        """Cloud expressed in the body frame."""
        return transform_points(self.T_body_lidar, self.points_lidar)


def transform_points(T_a_b: np.ndarray, points_b: np.ndarray) -> np.ndarray:
    """Map (N,3) points from frame b into frame a."""
    return points_b @ T_a_b[:3, :3].T + T_a_b[:3, 3]


def ready_directories(directory: Path) -> list[Path]:
    """Complete keyframe directories, ordered by numeric (epoch, sequence).

    A ``kf_*`` name is only produced by the writer's final rename, but a
    directory without both files is still skipped so a foreign or partial
    copy is never decoded.
    """
    found = []
    for path in Path(directory).iterdir():
        match = READY_NAME.fullmatch(path.name)
        if match and (path / "metadata.json").is_file() and (path / "points_lidar.pcd").is_file():
            found.append(((int(match[1]), int(match[2])), path))
    return [path for _, path in sorted(found)]


def read_spool(directory: Path, *, load_visual: bool = False) -> list[Keyframe]:
    """Read every ready keyframe in ``directory`` in numeric key order."""
    return [read_keyframe(path, load_visual=load_visual) for path in ready_directories(directory)]


def read_keyframe(path: Path, *, load_visual: bool = False) -> Keyframe:
    """Decode one ``kf_<epoch>_<sequence>`` directory; ValueError names the path.

    默认 ``load_visual=False`` 严格保持纯 LiDAR 基线读取路径，不探测也不解码 ``visual.npz``。
    """
    path = Path(path)
    try:
        row = json.loads((path / "metadata.json").read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"{path}: invalid metadata.json ({error})") from error
    if not isinstance(row, dict):
        raise ValueError(f"{path}: metadata.json root must be a JSON object")
    if row.get("schema") != SCHEMA:
        raise ValueError(f"{path}: schema is not {SCHEMA}")
    try:
        key = (int(row["epoch"]), int(row["sequence"]))
        stamp_ns = int(row["stamp_ns"])
        T_odom_body = _matrix(row["T_odom_body"], path)
        T_body_lidar = _matrix(row["T_body_lidar"], path)
        points_lidar = _read_ascii_pcd(path / "points_lidar.pcd")
    except (KeyError, TypeError, ValueError, OSError) as error:
        if isinstance(error, ValueError) and str(error).startswith(f"{path}:"):
            raise
        raise ValueError(f"{path}: malformed keyframe ({error})") from error
    visual = None
    if load_visual and (path / "visual.npz").is_file():
        # 仅当显式启用视觉读取时才解码可选旁文件，并对图像尺寸和时间戳标量做边界防护。
        try:
            with np.load(path / "visual.npz", allow_pickle=False) as data:
                gray = data["gray"]
                if gray.ndim != 2 or max(gray.shape) > MAX_VISUAL_IMAGE_DIM:
                    raise ValueError(f"灰度图尺寸非法或超过上限 {MAX_VISUAL_IMAGE_DIM}: {gray.shape}")
                rgb = data["rgb"] if "rgb" in data.files else None
                if rgb is not None and (rgb.ndim != 3 or max(rgb.shape[:2]) > MAX_VISUAL_IMAGE_DIM):
                    raise ValueError(f"RGB 图尺寸非法或超过上限 {MAX_VISUAL_IMAGE_DIM}: {rgb.shape}")
                stamp_arr = np.asarray(data["stamp_ns"])
                if stamp_arr.ndim != 0 or not np.issubdtype(stamp_arr.dtype, np.integer):
                    raise ValueError(f"stamp_ns 必须为整型标量，当前 dtype={stamp_arr.dtype}, shape={stamp_arr.shape}")
                visual = VisualObservation(gray, data["K"], data["T_body_camera"], int(stamp_arr.item()), rgb=rgb)
        except (KeyError, OSError, TypeError, ValueError) as error:
            raise ValueError(f"{path}: 视觉观测 visual.npz 非法: {error}") from error
    return Keyframe(
        key=key,
        stamp_ns=stamp_ns,
        T_odom_body=T_odom_body,
        T_body_lidar=T_body_lidar,
        points_lidar=points_lidar,
        visual=visual,
    )


def _matrix(rows, path: Path) -> np.ndarray:
    matrix = np.asarray(rows, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{path}: transform is not a finite 4x4 matrix")
    return matrix


def _read_ascii_pcd(path: Path) -> np.ndarray:
    _, marker, body = path.read_text().partition("DATA ascii\n")
    if not marker:
        raise ValueError(f"{path}: not an ASCII PCD")
    points = np.asarray(body.split(), dtype=np.float64).reshape(-1, 3)
    if not np.isfinite(points).all():
        raise ValueError(f"{path}: non-finite point coordinates")
    return points


def voxel_downsample(points: np.ndarray, voxel_m: float, *, use_cpp: bool = True) -> np.ndarray:
    """Replace the points of each ``voxel_m`` cube by their centroid.

    The grid is anchored at the frame origin, so the result is independent of
    the input order and stays in the input frame.
    """
    if len(points) == 0:
        return np.empty((0, 3))
    if use_cpp:
        from .accel import cpp_voxel_downsample

        fast = cpp_voxel_downsample(points, voxel_m)
        if fast is not None:
            return fast
    cells = np.floor(points / voxel_m).astype(np.int64)
    _, inverse, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.reshape(-1)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]
