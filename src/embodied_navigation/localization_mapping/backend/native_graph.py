"""Bounded native SE3 graph solve. Called only by the background history owner."""

from embodied_navigation.project import project_root as _project_root
import ctypes
from pathlib import Path
import numpy as np

LIBRARY = _project_root() / "install/backend/lib/liblivo_graph.so"
_library = None


def optimize_graph(poses, edges, iterations=8):
    """Solve one bounded SE3 pose graph through the native library.

    Converts poses/edges into contiguous float64/int64/uint8 arrays, calls
    ``livo_optimize_graph`` with at most 80 active nodes, and accepts the result
    only if the solver returns 0, the output stays finite and the final cost does
    not exceed the initial cost. Returns ``(poses_or_None, report)``; a missing
    library degrades to a diagnostic report instead of raising.
    """
    global _library
    if not LIBRARY.is_file():
        return None, dict(optimized=False, reason="native_graph_solver_unavailable")
    if _library is None:
        _library = ctypes.CDLL(str(LIBRARY))
        _library.livo_optimize_graph.argtypes = [
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_void_p,
        ]
        _library.livo_optimize_graph.restype = ctypes.c_int
    values = np.ascontiguousarray(poses, dtype=np.float64)
    endpoints = np.ascontiguousarray([(e.target, e.source) for e in edges], dtype=np.int64)
    measurements = np.ascontiguousarray([e.T_target_source for e in edges], dtype=np.float64)
    sigmas = np.ascontiguousarray(
        [(e.translation_sigma, e.rotation_sigma) for e in edges], dtype=np.float64
    )
    loops = np.ascontiguousarray([e.kind == "loop" for e in edges], dtype=np.uint8)
    report = np.zeros(5, dtype=np.float64)
    pointer = lambda a: ctypes.c_void_p(a.ctypes.data)
    status = _library.livo_optimize_graph(
        len(values),
        pointer(values),
        len(edges),
        pointer(endpoints),
        pointer(measurements),
        pointer(sigmas),
        pointer(loops),
        80,
        min(8, max(1, iterations)),
        pointer(report),
    )
    accepted = status == 0 and np.isfinite(values).all() and report[1] <= report[0]
    result = dict(
        optimized=bool(accepted),
        reason="verified_loop_graph" if accepted else "native_solver_rejected",
        initial_cost=float(report[0]),
        final_cost=float(report[1]),
        iterations=int(report[2]),
        solve_ms=float(report[3]),
        converged=bool(report[4]),
        active_nodes=min(80, len(values) - 1),
        method="bounded_native_SE3_sparse_graph",
        solver_status=status,
    )
    return (values if accepted else None), result
