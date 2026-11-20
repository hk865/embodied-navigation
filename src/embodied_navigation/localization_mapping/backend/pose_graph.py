"""SE(3) pose graph solved by Levenberg-Marquardt on sparse normal equations.

Node poses are ``T_world_body`` (4x4). An edge ``(a, b, T_a_b)`` measures the
pose of node b in node a's frame. Tangent vectors are ordered
``(translation rho, rotation phi)``; 6x6 information matrices use the same
order. Updates are right-multiplied: ``T <- T @ exp(delta)``. The first node
added is held fixed (gauge).

Edge residual: ``e = log(inv(T_a_b) @ inv(T_a) @ T_b)``. Robust edges are
weighted with a Cauchy kernel on the Mahalanobis norm ``s = e^T I e``:
cost ``c^2 log(1 + s / c^2)``, IRLS weight ``1 / (1 + s / c^2)``.
"""

from __future__ import annotations

import numpy as np
import scipy.sparse as sparse
from scipy.sparse.linalg import spsolve
from scipy.spatial.transform import Rotation


def hat(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _left_jacobian(phi: np.ndarray) -> np.ndarray:
    theta = np.linalg.norm(phi)
    K = hat(phi)
    if theta < 1e-6:
        return np.eye(3) + K / 2 + K @ K / 6
    return (
        np.eye(3)
        + (1 - np.cos(theta)) / theta**2 * K
        + (theta - np.sin(theta)) / theta**3 * K @ K
    )


def se3_exp(xi: np.ndarray) -> np.ndarray:
    """Exponential map of a ``(rho, phi)`` twist."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(xi[3:]).as_matrix()
    T[:3, 3] = _left_jacobian(xi[3:]) @ xi[:3]
    return T


def se3_log(T: np.ndarray) -> np.ndarray:
    """Inverse of ``se3_exp``."""
    phi = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    return np.r_[np.linalg.solve(_left_jacobian(phi), T[:3, 3]), phi]


def adjoint(T: np.ndarray) -> np.ndarray:
    """Adjoint for ``(rho, phi)`` twists: ``T exp(x) inv(T) = exp(Ad(T) x)``."""
    R, t = T[:3, :3], T[:3, 3]
    A = np.zeros((6, 6))
    A[:3, :3] = A[3:, 3:] = R
    A[:3, 3:] = hat(t) @ R
    return A


def invert(T: np.ndarray) -> np.ndarray:
    inverse = np.eye(4)
    inverse[:3, :3] = T[:3, :3].T
    inverse[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return inverse


def pose_difference(A: np.ndarray, B: np.ndarray) -> tuple[float, float]:
    """(translation m, rotation rad) of ``inv(A) @ B``."""
    delta = invert(A) @ B
    angle = np.arccos(np.clip((np.trace(delta[:3, :3]) - 1) / 2, -1.0, 1.0))
    return float(np.linalg.norm(delta[:3, 3])), float(angle)


class PoseGraph:
    def __init__(self, *, cauchy_scale: float = 1.0):
        self.cauchy_scale = cauchy_scale
        self.keys: list = []
        self.index: dict = {}
        self.poses: list[np.ndarray] = []
        self.edges: list[tuple[int, int, np.ndarray, np.ndarray, bool]] = []

    def add_node(self, key, T_world_body: np.ndarray) -> None:
        self.index[key] = len(self.keys)
        self.keys.append(key)
        self.poses.append(np.array(T_world_body, dtype=np.float64))

    def add_edge(self, a, b, T_a_b: np.ndarray, information: np.ndarray, *, robust: bool = False) -> None:
        self.edges.append((self.index[a], self.index[b], invert(T_a_b), information, robust))

    def pose(self, key) -> np.ndarray:
        return self.poses[self.index[key]].copy()

    def optimize(self, iterations: int = 20) -> dict:
        """LM iterations; returns robust costs before/after and convergence."""
        before = cost = self._cost(self.poses)
        damping = 1e-4
        converged = False
        done = 0
        for done in range(1, iterations + 1):
            H, g = self._normal_equations(self.poses)
            step = spsolve((H + damping * sparse.diags(H.diagonal())).tocsc(), -g)
            trial = self._retract(step)
            trial_cost = self._cost(trial)
            if trial_cost < cost:
                self.poses, damping = trial, damping / 10
                converged = cost - trial_cost <= 1e-9 * max(cost, 1e-12) or np.abs(step).max() < 1e-8
                cost = trial_cost
            else:
                damping *= 10
                converged = np.abs(step).max() < 1e-8
            if converged:
                break
        return {"converged": bool(converged), "iterations": done, "error_before": before, "error_after": cost}

    def _residual(self, poses, edge) -> np.ndarray:
        a, b, Z_inv, _, _ = edge
        return se3_log(Z_inv @ invert(poses[a]) @ poses[b])

    def _weight(self, e, information, robust) -> float:
        if not robust:
            return 1.0
        return 1.0 / (1.0 + e @ information @ e / self.cauchy_scale**2)

    def _cost(self, poses) -> float:
        total = 0.0
        for edge in self.edges:
            e = self._residual(poses, edge)
            s = e @ edge[3] @ e
            c2 = self.cauchy_scale**2
            total += c2 * np.log1p(s / c2) if edge[4] else s
        return float(total)

    def _normal_equations(self, poses):
        """H and g over the free nodes (all but the first)."""
        size = 6 * (len(poses) - 1)
        rows, cols, values = [], [], []
        g = np.zeros(size)
        for edge in self.edges:
            a, b, _, information, robust = edge
            e = self._residual(poses, edge)
            weighted = self._weight(e, information, robust) * information
            jacobians = {a: -adjoint(invert(poses[b]) @ poses[a]), b: np.eye(6)}
            for i, Ji in jacobians.items():
                if i == 0:
                    continue
                g[6 * (i - 1) : 6 * i] += Ji.T @ weighted @ e
                for j, Jj in jacobians.items():
                    if j == 0:
                        continue
                    block_rows, block_cols = np.meshgrid(
                        np.arange(6 * (i - 1), 6 * i), np.arange(6 * (j - 1), 6 * j), indexing="ij"
                    )
                    rows.append(block_rows.ravel())
                    cols.append(block_cols.ravel())
                    values.append((Ji.T @ weighted @ Jj).ravel())
        H = sparse.coo_matrix(
            (np.concatenate(values), (np.concatenate(rows), np.concatenate(cols))), shape=(size, size)
        )
        return H.tocsr(), g

    def _retract(self, step) -> list[np.ndarray]:
        moved = [self.poses[0]]
        for i, T in enumerate(self.poses[1:]):
            moved.append(T @ se3_exp(step[6 * i : 6 * i + 6]))
        return moved
