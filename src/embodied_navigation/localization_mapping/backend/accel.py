"""Optional C++17 / Eigen3 hot-path accelerator for the SLAM backend.

Compiles ``_cpp/slam_backend_accel.cpp`` once on demand into the user cache
directory keyed by the source SHA-256 (never writes binaries inside the git
repository) and loads it via ``ctypes``. If ``g++`` or Eigen3 headers are not
installed, every helper returns ``None`` and callers fall back seamlessly to
the pure-Python reference implementation.
"""

from __future__ import annotations

import ctypes
import hashlib
import math
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import tempfile
import threading

import numpy as np

_CPP_SOURCE = Path(__file__).resolve().parent / "_cpp" / "slam_backend_accel.cpp"
_LOCK = threading.Lock()
_LOADED = False
_LIB: ctypes.CDLL | None = None

_DOUBLE_PTR = ctypes.POINTER(ctypes.c_double)
_INT_PTR = ctypes.POINTER(ctypes.c_int)
_MAX_CPP_COUNT = 2_000_000_000


def _cache_dir() -> Path:
    base = Path(os.environ.get("XDG_CACHE_HOME") or (Path.home() / ".cache"))
    return base / "embodied_navigation" / "slam_backend_accel"


def _find_eigen_include() -> Path | None:
    for candidate in (Path("/usr/include/eigen3"), Path("/usr/local/include/eigen3")):
        if (candidate / "Eigen" / "Dense").is_file():
            return candidate
    return None


def _compile_and_load() -> ctypes.CDLL | None:
    if not _CPP_SOURCE.is_file():
        return None
    cxx = shutil.which("g++")
    eigen_inc = _find_eigen_include()
    if cxx is None or eigen_inc is None:
        return None
    digest = hashlib.sha256(_CPP_SOURCE.read_bytes()).hexdigest()[:16]
    arch = re.sub(r"[^A-Za-z0-9_.-]", "_", platform.machine() or "unknown")
    out_dir = _cache_dir()
    so_path = out_dir / f"libslam_backend_accel_{arch}_{digest}.so"
    if not so_path.is_file():
        tmp_path: Path | None = None
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(
                dir=out_dir,
                prefix=f"libslam_backend_accel_{arch}_{digest}.",
                suffix=".tmp.so",
                delete=False,
            ) as fh:
                tmp_path = Path(fh.name)
            cmd = [
                cxx,
                "-O3",
                "-std=c++17",
                "-shared",
                "-fPIC",
                "-DNDEBUG",
                f"-I{eigen_inc}",
                str(_CPP_SOURCE),
                "-o",
                str(tmp_path),
            ]
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            if proc.returncode != 0:
                return None
            tmp_path.replace(so_path)
        except (OSError, subprocess.SubprocessError):
            return None
        finally:
            if tmp_path is not None:
                tmp_path.unlink(missing_ok=True)
    try:
        lib = ctypes.CDLL(str(so_path))
    except OSError:
        return None
    _bind_signatures(lib)
    return lib


def _bind_signatures(lib: ctypes.CDLL) -> None:
    lib.cpp_voxel_downsample.argtypes = [_DOUBLE_PTR, ctypes.c_int, ctypes.c_double, _DOUBLE_PTR]
    lib.cpp_voxel_downsample.restype = ctypes.c_int

    lib.cpp_icp_point_to_plane.argtypes = [
        _DOUBLE_PTR, ctypes.c_int,
        _DOUBLE_PTR, ctypes.c_int,
        _DOUBLE_PTR,
        ctypes.c_double, ctypes.c_int, ctypes.c_double,
        ctypes.c_int, _DOUBLE_PTR, _DOUBLE_PTR, ctypes.c_double, ctypes.c_double,
        _DOUBLE_PTR, _DOUBLE_PTR, _DOUBLE_PTR, _INT_PTR, _DOUBLE_PTR,
    ]
    lib.cpp_icp_point_to_plane.restype = None

    lib.cpp_optimize_pose_graph.argtypes = [
        _DOUBLE_PTR, ctypes.c_int,
        _INT_PTR, _INT_PTR, _DOUBLE_PTR, _DOUBLE_PTR, _INT_PTR, ctypes.c_int,
        _INT_PTR, _DOUBLE_PTR, _DOUBLE_PTR, _INT_PTR, ctypes.c_int, _DOUBLE_PTR,
        _INT_PTR, _DOUBLE_PTR, _DOUBLE_PTR, _INT_PTR, ctypes.c_int,
        ctypes.c_double, ctypes.c_int,
        _INT_PTR, _INT_PTR, _DOUBLE_PTR, _DOUBLE_PTR,
    ]
    lib.cpp_optimize_pose_graph.restype = None

    lib.cpp_run_oriented_planar_ba.argtypes = [
        _DOUBLE_PTR, ctypes.c_int, ctypes.c_int,
        _DOUBLE_PTR, _INT_PTR,
        _INT_PTR, _INT_PTR, _DOUBLE_PTR, _DOUBLE_PTR, _INT_PTR, ctypes.c_int,
        _INT_PTR, _DOUBLE_PTR, _DOUBLE_PTR, _INT_PTR, ctypes.c_int,
        _DOUBLE_PTR,
        ctypes.c_double, ctypes.c_int, ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double, ctypes.c_int,
        ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_int,
        _INT_PTR, _INT_PTR, _INT_PTR, _DOUBLE_PTR, _DOUBLE_PTR,
    ]
    lib.cpp_run_oriented_planar_ba.restype = None


def get_accel_lib() -> ctypes.CDLL | None:
    """Return the loaded C++ accelerator library, or ``None`` if unavailable."""
    global _LOADED, _LIB
    if _LOADED:
        return _LIB
    with _LOCK:
        if not _LOADED:
            _LIB = _compile_and_load()
            _LOADED = True
    return _LIB


def _validate_points_array(points: np.ndarray, name: str) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3), got {pts.shape}")
    if len(pts) > _MAX_CPP_COUNT:
        raise ValueError(f"{name} exceeds maximum count {_MAX_CPP_COUNT}: {len(pts)}")
    if pts.size > 0 and not np.isfinite(pts).all():
        raise ValueError(f"{name} must contain only finite coordinates")
    return np.ascontiguousarray(pts, dtype=np.float64)


def _validate_matrix_array(arr: np.ndarray, shape: tuple[int, ...], name: str) -> np.ndarray:
    mat = np.asarray(arr, dtype=np.float64)
    if mat.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {mat.shape}")
    if mat.size > 0 and not np.isfinite(mat).all():
        raise ValueError(f"{name} must contain only finite values")
    return np.ascontiguousarray(mat, dtype=np.float64)


def _validate_node_index(idx, n_nodes: int, name: str) -> int:
    if isinstance(idx, bool) or not isinstance(idx, (int, np.integer)):
        raise ValueError(f"{name} must be an integer index, got {idx!r}")
    val = int(idx)
    if val < 0 or val >= n_nodes:
        raise ValueError(f"{name} index {val} out of range [0, {n_nodes})")
    return val


def cpp_voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray | None:
    if isinstance(voxel_m, bool):
        raise ValueError(f"voxel_m must be a finite positive float, got {voxel_m!r}")
    voxel = float(voxel_m)
    if not math.isfinite(voxel) or voxel <= 0.0:
        raise ValueError(f"voxel_m must be finite and > 0, got {voxel_m!r}")
    pts = _validate_points_array(points, "points")
    n = len(pts)
    lib = get_accel_lib()
    if lib is None:
        return None
    if n == 0:
        return np.empty((0, 3), dtype=np.float64)
    out = np.empty((n, 3), dtype=np.float64)
    m = lib.cpp_voxel_downsample(
        pts.ctypes.data_as(_DOUBLE_PTR),
        n,
        voxel,
        out.ctypes.data_as(_DOUBLE_PTR),
    )
    if m < 0 or m > n:
        raise ValueError(f"cpp_voxel_downsample failed (returned count {m})")
    return out[:m].copy()


def cpp_icp_point_to_plane(
    source: np.ndarray,
    target: np.ndarray,
    T_init: np.ndarray,
    *,
    max_correspondence_m: float,
    iterations: int,
    voxel_m: float,
    source_up_body: np.ndarray | None,
    gravity_sigma_rad: float | None,
    gravity_ref_point_sigma_m: float,
    target_up_world: np.ndarray | None = None,
    return_hessian: bool = False,
) -> tuple[np.ndarray, float, float, bool, np.ndarray | None] | None:
    src = _validate_points_array(source, "source")
    tgt = _validate_points_array(target, "target")
    T0 = _validate_matrix_array(T_init, (4, 4), "T_init")
    max_corr = float(max_correspondence_m)
    vox = float(voxel_m)
    if not math.isfinite(max_corr) or max_corr <= 0.0:
        raise ValueError(f"max_correspondence_m must be finite and > 0, got {max_correspondence_m!r}")
    if not math.isfinite(vox) or vox < 0.0:
        raise ValueError(f"voxel_m must be finite and >= 0, got {voxel_m!r}")
    if isinstance(iterations, bool) or not isinstance(iterations, (int, np.integer)) or int(iterations) < 0:
        raise ValueError(f"iterations must be a non-negative integer, got {iterations!r}")
    has_grav = 1 if (source_up_body is not None and gravity_sigma_rad is not None) else 0
    if has_grav:
        up_arr = _validate_matrix_array(source_up_body, (3,), "source_up_body")
        target_up_arr = (
            _validate_matrix_array(target_up_world, (3,), "target_up_world")
            if target_up_world is not None
            else np.array([0.0, 0.0, 1.0], dtype=np.float64)
        )
        grav_sig = float(gravity_sigma_rad)
        grav_ref = float(gravity_ref_point_sigma_m)
        if not math.isfinite(grav_sig) or grav_sig <= 0.0:
            raise ValueError(f"gravity_sigma_rad must be finite and > 0, got {gravity_sigma_rad!r}")
        if not math.isfinite(grav_ref) or grav_ref < 0.0:
            raise ValueError(
                f"gravity_ref_point_sigma_m must be finite and >= 0, got {gravity_ref_point_sigma_m!r}"
            )
    else:
        up_arr = np.zeros(3, dtype=np.float64)
        target_up_arr = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        grav_sig = 0.0
        grav_ref = float(gravity_ref_point_sigma_m)
        if not math.isfinite(grav_ref):
            raise ValueError(f"gravity_ref_point_sigma_m must be finite, got {gravity_ref_point_sigma_m!r}")

    lib = get_accel_lib()
    if lib is None:
        return None
    T_out = np.empty((4, 4), dtype=np.float64)
    fitness = ctypes.c_double(0.0)
    rmse = ctypes.c_double(float("inf"))
    converged = ctypes.c_int(0)
    hess = np.empty((6, 6), dtype=np.float64) if return_hessian else None
    hess_ptr = hess.ctypes.data_as(_DOUBLE_PTR) if hess is not None else ctypes.cast(0, _DOUBLE_PTR)

    lib.cpp_icp_point_to_plane(
        src.ctypes.data_as(_DOUBLE_PTR), len(src),
        tgt.ctypes.data_as(_DOUBLE_PTR), len(tgt),
        T0.ctypes.data_as(_DOUBLE_PTR),
        max_corr, int(iterations), vox,
        has_grav, up_arr.ctypes.data_as(_DOUBLE_PTR), target_up_arr.ctypes.data_as(_DOUBLE_PTR), grav_sig, grav_ref,
        T_out.ctypes.data_as(_DOUBLE_PTR),
        ctypes.byref(fitness), ctypes.byref(rmse), ctypes.byref(converged),
        hess_ptr,
    )
    return T_out, float(fitness.value), float(rmse.value), bool(converged.value), hess


def cpp_optimize_pose_graph(
    poses: list[np.ndarray],
    edges: list[tuple[int, int, np.ndarray, np.ndarray, bool]],
    gravity_priors: list[tuple[int, np.ndarray, float, bool]],
    z_priors: list[tuple[int, float, float, bool]],
    *,
    cauchy_scale: float,
    iterations: int,
    gravity_world: np.ndarray | None = None,
) -> tuple[list[np.ndarray], dict] | None:
    n_nodes = len(poses)
    if n_nodes > _MAX_CPP_COUNT:
        raise ValueError(f"poses exceeds maximum count {_MAX_CPP_COUNT}: {n_nodes}")
    c_scale = float(cauchy_scale)
    if not math.isfinite(c_scale) or c_scale <= 0.0:
        raise ValueError(f"cauchy_scale must be finite and > 0, got {cauchy_scale!r}")
    if isinstance(iterations, bool) or not isinstance(iterations, (int, np.integer)) or int(iterations) < 0:
        raise ValueError(f"iterations must be a non-negative integer, got {iterations!r}")
    validated_poses = [_validate_matrix_array(p, (4, 4), f"poses[{i}]") for i, p in enumerate(poses)]

    n_edges = len(edges)
    if n_edges > _MAX_CPP_COUNT:
        raise ValueError(f"edges exceeds maximum count {_MAX_CPP_COUNT}: {n_edges}")
    if n_edges > 0:
        ea_list, eb_list, eZ_list, eI_list, eR_list = [], [], [], [], []
        for idx, (a, b, Z_inv, info, robust) in enumerate(edges):
            ea_list.append(_validate_node_index(a, n_nodes, f"edges[{idx}].a"))
            eb_list.append(_validate_node_index(b, n_nodes, f"edges[{idx}].b"))
            eZ_list.append(_validate_matrix_array(Z_inv, (4, 4), f"edges[{idx}].Z_inv"))
            eI_list.append(_validate_matrix_array(info, (6, 6), f"edges[{idx}].info"))
            eR_list.append(1 if robust else 0)
        ea = np.ascontiguousarray(ea_list, dtype=np.int32)
        eb = np.ascontiguousarray(eb_list, dtype=np.int32)
        eZ = np.ascontiguousarray(np.stack(eZ_list, axis=0), dtype=np.float64)
        eI = np.ascontiguousarray(np.stack(eI_list, axis=0), dtype=np.float64)
        eR = np.ascontiguousarray(eR_list, dtype=np.int32)
    else:
        ea = eb = eR = np.zeros(1, dtype=np.int32)
        eZ = np.zeros((1, 4, 4), dtype=np.float64)
        eI = np.zeros((1, 6, 6), dtype=np.float64)

    n_grav = len(gravity_priors)
    if n_grav > _MAX_CPP_COUNT:
        raise ValueError(f"gravity_priors exceeds maximum count {_MAX_CPP_COUNT}: {n_grav}")
    if n_grav > 0:
        gn_list, gb_list, gw_list, gr_list = [], [], [], []
        for idx, (ni, g_body, weight, robust) in enumerate(gravity_priors):
            gn_list.append(_validate_node_index(ni, n_nodes, f"gravity_priors[{idx}].node"))
            gb_list.append(_validate_matrix_array(g_body, (3,), f"gravity_priors[{idx}].g_body"))
            w = float(weight)
            if not math.isfinite(w) or w < 0.0:
                raise ValueError(f"gravity_priors[{idx}].weight must be finite and >= 0, got {weight!r}")
            gw_list.append(w)
            gr_list.append(1 if robust else 0)
        gn = np.ascontiguousarray(gn_list, dtype=np.int32)
        gb = np.ascontiguousarray(np.stack(gb_list, axis=0), dtype=np.float64)
        gw = np.ascontiguousarray(gw_list, dtype=np.float64)
        gr = np.ascontiguousarray(gr_list, dtype=np.int32)
    else:
        gn = gr = np.zeros(1, dtype=np.int32)
        gb = np.zeros((1, 3), dtype=np.float64)
        gw = np.zeros(1, dtype=np.float64)

    n_zp = len(z_priors)
    if n_zp > _MAX_CPP_COUNT:
        raise ValueError(f"z_priors exceeds maximum count {_MAX_CPP_COUNT}: {n_zp}")
    if n_zp > 0:
        zn_list, zt_list, zw_list, zr_list = [], [], [], []
        for idx, (ni, target_z, weight, robust) in enumerate(z_priors):
            zn_list.append(_validate_node_index(ni, n_nodes, f"z_priors[{idx}].node"))
            tz = float(target_z)
            w = float(weight)
            if not math.isfinite(tz):
                raise ValueError(f"z_priors[{idx}].target_z must be finite, got {target_z!r}")
            if not math.isfinite(w) or w < 0.0:
                raise ValueError(f"z_priors[{idx}].weight must be finite and >= 0, got {weight!r}")
            zt_list.append(tz)
            zw_list.append(w)
            zr_list.append(1 if robust else 0)
        zn = np.ascontiguousarray(zn_list, dtype=np.int32)
        zt = np.ascontiguousarray(zt_list, dtype=np.float64)
        zw = np.ascontiguousarray(zw_list, dtype=np.float64)
        zr = np.ascontiguousarray(zr_list, dtype=np.int32)
    else:
        zn = zr = np.zeros(1, dtype=np.int32)
        zt = zw = np.zeros(1, dtype=np.float64)

    lib = get_accel_lib()
    if lib is None:
        return None
    if n_nodes == 0:
        return [], {"converged": True, "iterations": 0, "error_before": 0.0, "error_after": 0.0}
    poses_arr = np.ascontiguousarray(np.stack(validated_poses, axis=0), dtype=np.float64)

    out_conv = ctypes.c_int(0)
    out_iter = ctypes.c_int(0)
    out_before = ctypes.c_double(0.0)
    out_after = ctypes.c_double(0.0)

    gw_vec = (
        _validate_matrix_array(gravity_world, (3,), "gravity_world")
        if gravity_world is not None
        else np.array([0.0, 0.0, 1.0], dtype=np.float64)
    )
    lib.cpp_optimize_pose_graph(
        poses_arr.ctypes.data_as(_DOUBLE_PTR), n_nodes,
        ea.ctypes.data_as(_INT_PTR), eb.ctypes.data_as(_INT_PTR),
        eZ.ctypes.data_as(_DOUBLE_PTR), eI.ctypes.data_as(_DOUBLE_PTR),
        eR.ctypes.data_as(_INT_PTR), n_edges,
        gn.ctypes.data_as(_INT_PTR), gb.ctypes.data_as(_DOUBLE_PTR),
        gw.ctypes.data_as(_DOUBLE_PTR), gr.ctypes.data_as(_INT_PTR), n_grav,
        gw_vec.ctypes.data_as(_DOUBLE_PTR),
        zn.ctypes.data_as(_INT_PTR), zt.ctypes.data_as(_DOUBLE_PTR),
        zw.ctypes.data_as(_DOUBLE_PTR), zr.ctypes.data_as(_INT_PTR), n_zp,
        c_scale, int(iterations),
        ctypes.byref(out_conv), ctypes.byref(out_iter),
        ctypes.byref(out_before), ctypes.byref(out_after),
    )
    updated = [poses_arr[i].copy() for i in range(n_nodes)]
    return updated, {
        "converged": bool(out_conv.value),
        "iterations": int(out_iter.value),
        "error_before": float(out_before.value),
        "error_after": float(out_after.value),
    }


def cpp_run_oriented_planar_ba(
    poses: list[np.ndarray],
    points_body_list: list[np.ndarray],
    edges: list[tuple[int, int, np.ndarray, np.ndarray, bool]],
    gravity_priors: list[tuple[int, np.ndarray, float, bool]],
    *,
    n_fixed: int = 1,
    gravity_world: np.ndarray | None = None,
    ba_voxel_m: float = 1.0,
    min_frame_pts: int = 8,
    plane_ratio_max: float = 0.08,
    plane_thickness_max_m: float = 0.08,
    min_normal_cos: float = 0.85,
    max_plane_offset_m: float = 0.15,
    max_frames_per_cluster: int = 20,
    plane_sigma_m: float = 0.03,
    cauchy_scale: float = 10.0,
    outer_rounds: int = 2,
    inner_iterations: int = 8,
) -> tuple[list[np.ndarray], dict] | None:
    n_nodes = len(poses)
    if n_nodes != len(points_body_list):
        raise ValueError(
            f"points_body_list length ({len(points_body_list)}) must match poses length ({n_nodes})"
        )
    if n_nodes > _MAX_CPP_COUNT:
        raise ValueError(f"poses exceeds maximum count {_MAX_CPP_COUNT}: {n_nodes}")
    for name, val in (
        ("ba_voxel_m", ba_voxel_m),
        ("plane_ratio_max", plane_ratio_max),
        ("plane_thickness_max_m", plane_thickness_max_m),
        ("max_plane_offset_m", max_plane_offset_m),
        ("plane_sigma_m", plane_sigma_m),
        ("cauchy_scale", cauchy_scale),
    ):
        if isinstance(val, bool) or not math.isfinite(float(val)) or float(val) <= 0.0:
            raise ValueError(f"{name} must be finite and > 0, got {val!r}")
    if isinstance(min_normal_cos, bool) or not math.isfinite(float(min_normal_cos)) or not (-1.0 < float(min_normal_cos) <= 1.0):
        raise ValueError(f"min_normal_cos must lie in (-1, 1], got {min_normal_cos!r}")
    for name, val, lo in (
        ("n_fixed", n_fixed, 1),
        ("min_frame_pts", min_frame_pts, 3),
        ("max_frames_per_cluster", max_frames_per_cluster, 2),
        ("outer_rounds", outer_rounds, 0),
        ("inner_iterations", inner_iterations, 0),
    ):
        if isinstance(val, bool) or not isinstance(val, (int, np.integer)) or int(val) < lo:
            raise ValueError(f"{name} must be an integer >= {lo}, got {val!r}")

    validated_poses = [_validate_matrix_array(p, (4, 4), f"poses[{i}]") for i, p in enumerate(poses)]
    validated_pts = [_validate_points_array(pts, f"points_body_list[{i}]") for i, pts in enumerate(points_body_list)]
    total_pts = sum(len(pts) for pts in validated_pts)
    if total_pts > _MAX_CPP_COUNT:
        raise ValueError(f"total points exceeds maximum count {_MAX_CPP_COUNT}: {total_pts}")

    n_edges = len(edges)
    if n_edges > _MAX_CPP_COUNT:
        raise ValueError(f"edges exceeds maximum count {_MAX_CPP_COUNT}: {n_edges}")
    if n_edges > 0:
        ea_list, eb_list, eZ_list, eI_list, eR_list = [], [], [], [], []
        for idx, (a, b, Z_inv, info, robust) in enumerate(edges):
            ea_list.append(_validate_node_index(a, n_nodes, f"edges[{idx}].a"))
            eb_list.append(_validate_node_index(b, n_nodes, f"edges[{idx}].b"))
            eZ_list.append(_validate_matrix_array(Z_inv, (4, 4), f"edges[{idx}].Z_inv"))
            eI_list.append(_validate_matrix_array(info, (6, 6), f"edges[{idx}].info"))
            eR_list.append(1 if robust else 0)
        ea = np.ascontiguousarray(ea_list, dtype=np.int32)
        eb = np.ascontiguousarray(eb_list, dtype=np.int32)
        eZ = np.ascontiguousarray(np.stack(eZ_list, axis=0), dtype=np.float64)
        eI = np.ascontiguousarray(np.stack(eI_list, axis=0), dtype=np.float64)
        eR = np.ascontiguousarray(eR_list, dtype=np.int32)
    else:
        ea = eb = eR = np.zeros(1, dtype=np.int32)
        eZ = np.zeros((1, 4, 4), dtype=np.float64)
        eI = np.zeros((1, 6, 6), dtype=np.float64)

    n_grav = len(gravity_priors)
    if n_grav > _MAX_CPP_COUNT:
        raise ValueError(f"gravity_priors exceeds maximum count {_MAX_CPP_COUNT}: {n_grav}")
    if n_grav > 0:
        gn_list, gb_list, gw_list, gr_list = [], [], [], []
        for idx, (ni, g_body, weight, robust) in enumerate(gravity_priors):
            gn_list.append(_validate_node_index(ni, n_nodes, f"gravity_priors[{idx}].node"))
            gb_list.append(_validate_matrix_array(g_body, (3,), f"gravity_priors[{idx}].g_body"))
            w = float(weight)
            if not math.isfinite(w) or w < 0.0:
                raise ValueError(f"gravity_priors[{idx}].weight must be finite and >= 0, got {weight!r}")
            gw_list.append(w)
            gr_list.append(1 if robust else 0)
        gn = np.ascontiguousarray(gn_list, dtype=np.int32)
        gb = np.ascontiguousarray(np.stack(gb_list, axis=0), dtype=np.float64)
        gw = np.ascontiguousarray(gw_list, dtype=np.float64)
        gr = np.ascontiguousarray(gr_list, dtype=np.int32)
    else:
        gn = gr = np.zeros(1, dtype=np.int32)
        gb = np.zeros((1, 3), dtype=np.float64)
        gw = np.zeros(1, dtype=np.float64)

    gw_vec = (
        _validate_matrix_array(gravity_world, (3,), "gravity_world")
        if gravity_world is not None
        else np.array([0.0, 0.0, 1.0], dtype=np.float64)
    )

    lib = get_accel_lib()
    if lib is None:
        return None
    if n_nodes <= int(n_fixed):
        return [p.copy() for p in validated_poses], {
            "converged": True,
            "clusters_used": 0,
            "obs_used": 0,
            "error_before": 0.0,
            "error_after": 0.0,
        }

    poses_arr = np.ascontiguousarray(np.stack(validated_poses, axis=0), dtype=np.float64)
    offsets = np.zeros(n_nodes + 1, dtype=np.int32)
    for i, pts in enumerate(validated_pts):
        offsets[i + 1] = offsets[i] + len(pts)
    all_pts = (
        np.ascontiguousarray(np.vstack([pts for pts in validated_pts if len(pts) > 0]), dtype=np.float64)
        if total_pts > 0
        else np.zeros((1, 3), dtype=np.float64)
    )

    out_conv = ctypes.c_int(0)
    out_clusters = ctypes.c_int(0)
    out_obs = ctypes.c_int(0)
    out_before = ctypes.c_double(0.0)
    out_after = ctypes.c_double(0.0)

    lib.cpp_run_oriented_planar_ba(
        poses_arr.ctypes.data_as(_DOUBLE_PTR), n_nodes, int(n_fixed),
        all_pts.ctypes.data_as(_DOUBLE_PTR), offsets.ctypes.data_as(_INT_PTR),
        ea.ctypes.data_as(_INT_PTR), eb.ctypes.data_as(_INT_PTR),
        eZ.ctypes.data_as(_DOUBLE_PTR), eI.ctypes.data_as(_DOUBLE_PTR),
        eR.ctypes.data_as(_INT_PTR), n_edges,
        gn.ctypes.data_as(_INT_PTR), gb.ctypes.data_as(_DOUBLE_PTR),
        gw.ctypes.data_as(_DOUBLE_PTR), gr.ctypes.data_as(_INT_PTR), n_grav,
        gw_vec.ctypes.data_as(_DOUBLE_PTR),
        float(ba_voxel_m), int(min_frame_pts), float(plane_ratio_max), float(plane_thickness_max_m),
        float(min_normal_cos), float(max_plane_offset_m), int(max_frames_per_cluster),
        float(plane_sigma_m), float(cauchy_scale), int(outer_rounds), int(inner_iterations),
        ctypes.byref(out_conv), ctypes.byref(out_clusters), ctypes.byref(out_obs),
        ctypes.byref(out_before), ctypes.byref(out_after),
    )
    updated = [poses_arr[i].copy() for i in range(n_nodes)]
    return updated, {
        "converged": bool(out_conv.value),
        "clusters_used": int(out_clusters.value),
        "obs_used": int(out_obs.value),
        "error_before": float(out_before.value),
        "error_after": float(out_after.value),
    }
