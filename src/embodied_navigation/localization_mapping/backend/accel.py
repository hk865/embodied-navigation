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
import os
from pathlib import Path
import shutil
import subprocess
import threading

import numpy as np

_CPP_SOURCE = Path(__file__).resolve().parent / "_cpp" / "slam_backend_accel.cpp"
_LOCK = threading.Lock()
_LOADED = False
_LIB: ctypes.CDLL | None = None

_DOUBLE_PTR = ctypes.POINTER(ctypes.c_double)
_INT_PTR = ctypes.POINTER(ctypes.c_int)


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
    out_dir = _cache_dir()
    so_path = out_dir / f"libslam_backend_accel_{digest}.so"
    if not so_path.is_file():
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
            tmp_path = out_dir / f"libslam_backend_accel_{digest}.{os.getpid()}.tmp.so"
            cmd = [
                cxx,
                "-O3",
                "-march=native",
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
                tmp_path.unlink(missing_ok=True)
                return None
            tmp_path.replace(so_path)
        except (OSError, subprocess.SubprocessError):
            return None
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
        ctypes.c_int, _DOUBLE_PTR, ctypes.c_double, ctypes.c_double,
        _DOUBLE_PTR, _DOUBLE_PTR, _DOUBLE_PTR, _INT_PTR, _DOUBLE_PTR,
    ]
    lib.cpp_icp_point_to_plane.restype = None

    lib.cpp_optimize_pose_graph.argtypes = [
        _DOUBLE_PTR, ctypes.c_int,
        _INT_PTR, _INT_PTR, _DOUBLE_PTR, _DOUBLE_PTR, _INT_PTR, ctypes.c_int,
        _INT_PTR, _DOUBLE_PTR, _DOUBLE_PTR, _INT_PTR, ctypes.c_int,
        _INT_PTR, _DOUBLE_PTR, _DOUBLE_PTR, _INT_PTR, ctypes.c_int,
        ctypes.c_double, ctypes.c_int,
        _INT_PTR, _INT_PTR, _DOUBLE_PTR, _DOUBLE_PTR,
    ]
    lib.cpp_optimize_pose_graph.restype = None


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


def cpp_voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray | None:
    lib = get_accel_lib()
    if lib is None:
        return None
    pts = np.ascontiguousarray(points, dtype=np.float64)
    n = len(pts)
    if n == 0:
        return np.empty((0, 3), dtype=np.float64)
    out = np.empty((n, 3), dtype=np.float64)
    m = lib.cpp_voxel_downsample(
        pts.ctypes.data_as(_DOUBLE_PTR),
        n,
        float(voxel_m),
        out.ctypes.data_as(_DOUBLE_PTR),
    )
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
    return_hessian: bool = False,
) -> tuple[np.ndarray, float, float, bool, np.ndarray | None] | None:
    lib = get_accel_lib()
    if lib is None:
        return None
    src = np.ascontiguousarray(source, dtype=np.float64).reshape(-1, 3)
    tgt = np.ascontiguousarray(target, dtype=np.float64).reshape(-1, 3)
    T0 = np.ascontiguousarray(T_init, dtype=np.float64).reshape(4, 4)
    T_out = np.empty((4, 4), dtype=np.float64)
    fitness = ctypes.c_double(0.0)
    rmse = ctypes.c_double(float("inf"))
    converged = ctypes.c_int(0)
    has_grav = 1 if (source_up_body is not None and gravity_sigma_rad is not None) else 0
    up_arr = np.ascontiguousarray(source_up_body, dtype=np.float64).reshape(3) if has_grav else np.zeros(3, dtype=np.float64)
    grav_sig = float(gravity_sigma_rad) if has_grav else 0.0
    hess = np.empty((6, 6), dtype=np.float64) if return_hessian else None
    hess_ptr = hess.ctypes.data_as(_DOUBLE_PTR) if hess is not None else ctypes.cast(0, _DOUBLE_PTR)

    lib.cpp_icp_point_to_plane(
        src.ctypes.data_as(_DOUBLE_PTR), len(src),
        tgt.ctypes.data_as(_DOUBLE_PTR), len(tgt),
        T0.ctypes.data_as(_DOUBLE_PTR),
        float(max_correspondence_m), int(iterations), float(voxel_m),
        has_grav, up_arr.ctypes.data_as(_DOUBLE_PTR), grav_sig, float(gravity_ref_point_sigma_m),
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
) -> tuple[list[np.ndarray], dict] | None:
    lib = get_accel_lib()
    if lib is None:
        return None
    n_nodes = len(poses)
    if n_nodes == 0:
        return [], {"converged": True, "iterations": 0, "error_before": 0.0, "error_after": 0.0}
    poses_arr = np.ascontiguousarray(np.stack(poses, axis=0), dtype=np.float64)

    n_edges = len(edges)
    if n_edges > 0:
        ea = np.ascontiguousarray([e[0] for e in edges], dtype=np.int32)
        eb = np.ascontiguousarray([e[1] for e in edges], dtype=np.int32)
        eZ = np.ascontiguousarray(np.stack([e[2] for e in edges], axis=0), dtype=np.float64)
        eI = np.ascontiguousarray(np.stack([e[3] for e in edges], axis=0), dtype=np.float64)
        eR = np.ascontiguousarray([1 if e[4] else 0 for e in edges], dtype=np.int32)
    else:
        ea = eb = eR = np.zeros(1, dtype=np.int32)
        eZ = np.zeros((1, 4, 4), dtype=np.float64)
        eI = np.zeros((1, 6, 6), dtype=np.float64)

    n_grav = len(gravity_priors)
    if n_grav > 0:
        gn = np.ascontiguousarray([g[0] for g in gravity_priors], dtype=np.int32)
        gb = np.ascontiguousarray(np.stack([g[1] for g in gravity_priors], axis=0), dtype=np.float64)
        gw = np.ascontiguousarray([g[2] for g in gravity_priors], dtype=np.float64)
        gr = np.ascontiguousarray([1 if g[3] else 0 for g in gravity_priors], dtype=np.int32)
    else:
        gn = gr = np.zeros(1, dtype=np.int32)
        gb = np.zeros((1, 3), dtype=np.float64)
        gw = np.zeros(1, dtype=np.float64)

    n_zp = len(z_priors)
    if n_zp > 0:
        zn = np.ascontiguousarray([z[0] for z in z_priors], dtype=np.int32)
        zt = np.ascontiguousarray([z[1] for z in z_priors], dtype=np.float64)
        zw = np.ascontiguousarray([z[2] for z in z_priors], dtype=np.float64)
        zr = np.ascontiguousarray([1 if z[3] else 0 for z in z_priors], dtype=np.int32)
    else:
        zn = zr = np.zeros(1, dtype=np.int32)
        zt = zw = np.zeros(1, dtype=np.float64)

    out_conv = ctypes.c_int(0)
    out_iter = ctypes.c_int(0)
    out_before = ctypes.c_double(0.0)
    out_after = ctypes.c_double(0.0)

    lib.cpp_optimize_pose_graph(
        poses_arr.ctypes.data_as(_DOUBLE_PTR), n_nodes,
        ea.ctypes.data_as(_INT_PTR), eb.ctypes.data_as(_INT_PTR),
        eZ.ctypes.data_as(_DOUBLE_PTR), eI.ctypes.data_as(_DOUBLE_PTR),
        eR.ctypes.data_as(_INT_PTR), n_edges,
        gn.ctypes.data_as(_INT_PTR), gb.ctypes.data_as(_DOUBLE_PTR),
        gw.ctypes.data_as(_DOUBLE_PTR), gr.ctypes.data_as(_INT_PTR), n_grav,
        zn.ctypes.data_as(_INT_PTR), zt.ctypes.data_as(_DOUBLE_PTR),
        zw.ctypes.data_as(_DOUBLE_PTR), zr.ctypes.data_as(_INT_PTR), n_zp,
        float(cauchy_scale), int(iterations),
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
