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
    theta = float(np.linalg.norm(phi))
    K = hat(phi)
    K2 = K @ K
    t2 = theta * theta
    if theta < 1e-3:
        return np.eye(3) + (0.5 - t2 / 24.0) * K + (1.0 / 6.0 - t2 / 120.0) * K2
    return (
        np.eye(3)
        + (1.0 - np.cos(theta)) / t2 * K
        + (theta - np.sin(theta)) / (t2 * theta) * K2
    )


def _so3_left_jacobian_inv(phi: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(phi))
    K = hat(phi)
    K2 = K @ K
    t2 = theta * theta
    if theta < 1e-3:
        return np.eye(3) - 0.5 * K + (1.0 / 12.0 + t2 / 720.0) * K2
    coef = (1.0 / t2) - (1.0 + np.cos(theta)) / (2.0 * theta * np.sin(theta))
    return np.eye(3) - 0.5 * K + coef * K2


def _se3_Q_left(rho: np.ndarray, phi: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(phi))
    P = hat(rho)
    K = hat(phi)
    K2 = K @ K
    t2 = theta * theta
    if theta < 1e-2:
        t4 = t2 * t2
        c1 = 1.0 / 6.0 - t2 / 120.0 + t4 / 5040.0
        c2 = 1.0 / 24.0 - t2 / 720.0 + t4 / 40320.0
        c3 = 1.0 / 120.0 - t2 / 2520.0 + t4 / 120960.0
    else:
        t3 = t2 * theta
        t4 = t2 * t2
        t5 = t4 * theta
        s = np.sin(theta)
        c = np.cos(theta)
        c1 = (theta - s) / t3
        c2 = (t2 + 2.0 * c - 2.0) / (2.0 * t4)
        c3 = (2.0 * theta - 3.0 * s + theta * c) / (2.0 * t5)
    return (
        0.5 * P
        + c1 * (K @ P + P @ K + K @ P @ K)
        + c2 * (K2 @ P + P @ K2 - 3.0 * K @ P @ K)
        + c3 * (K @ P @ K2 + K2 @ P @ K)
    )


def se3_right_jacobian_inv(xi: np.ndarray) -> np.ndarray:
    """Exact SE(3) inverse right Jacobian ``J_r^{-1}(xi)`` for ``(rho, phi)`` twists."""
    Jr_phi_inv = _so3_left_jacobian_inv(-xi[3:])
    Qr = _se3_Q_left(-xi[:3], -xi[3:])
    J = np.zeros((6, 6), dtype=np.float64)
    J[:3, :3] = Jr_phi_inv
    J[3:, 3:] = Jr_phi_inv
    J[:3, 3:] = -(Jr_phi_inv @ Qr @ Jr_phi_inv)
    return J


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


def project_so3(R: np.ndarray) -> np.ndarray:
    """Nearest proper rotation matrix in SO(3) via SVD."""
    U, _, Vt = np.linalg.svd(R)
    R_proj = U @ Vt
    if np.linalg.det(R_proj) < 0.0:
        U[:, -1] *= -1.0
        R_proj = U @ Vt
    return R_proj


def rotation_angle(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Geodesic angle (rad) between two 3x3 rotation matrices."""
    return float(np.arccos(np.clip((np.trace(R_a.T @ R_b) - 1.0) * 0.5, -1.0, 1.0)))


def pose_difference(A: np.ndarray, B: np.ndarray) -> tuple[float, float]:
    """(translation m, rotation rad) of ``inv(A) @ B``."""
    delta = invert(A) @ B
    return float(np.linalg.norm(delta[:3, 3])), rotation_angle(A[:3, :3], B[:3, :3])


class PoseGraph:
    def __init__(self, *, cauchy_scale: float = 1.0):
        self.cauchy_scale = cauchy_scale
        self.keys: list = []
        self.index: dict = {}
        self.poses: list[np.ndarray] = []
        self.edges: list[tuple[int, int, np.ndarray, np.ndarray, bool]] = []
        self.gravity_priors: list[tuple[int, np.ndarray, float, bool]] = []
        self.z_priors: list[tuple[int, float, float, bool]] = []

    def add_node(self, key, T_world_body: np.ndarray) -> None:
        if key in self.index:
            raise ValueError(f"duplicate pose-graph node key: {key!r}")
        self.index[key] = len(self.keys)
        self.keys.append(key)
        self.poses.append(np.array(T_world_body, dtype=np.float64))

    def add_edge(self, a, b, T_a_b: np.ndarray, information: np.ndarray, *, robust: bool = False) -> None:
        self.edges.append((self.index[a], self.index[b], invert(T_a_b), information, robust))

    def add_gravity_prior(self, key, up_body: np.ndarray, sigma_rad: float, *, robust: bool = False) -> None:
        """Constrain node's world-frame up vector ``R_world_body @ up_body`` toward ``+e_z``.

        Leaves translation and world-Z yaw completely unconstrained.
        """
        vec = np.asarray(up_body, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(vec))
        if norm <= 1e-6 or not np.isfinite(norm):
            raise ValueError("up_body must be a finite non-zero 3-vector")
        if sigma_rad <= 0.0 or not np.isfinite(sigma_rad):
            raise ValueError(f"sigma_rad must be finite and positive, got {sigma_rad!r}")
        self.gravity_priors.append((self.index[key], vec / norm, 1.0 / (sigma_rad * sigma_rad), robust))

    def add_z_prior(self, key, target_z_m: float, sigma_m: float, *, robust: bool = True) -> None:
        """Softly constrain node's world-frame Z coordinate toward a shared floor slab elevation."""
        target = float(target_z_m)
        sig = float(sigma_m)
        if not np.isfinite(target):
            raise ValueError(f"target_z_m must be finite, got {target_z_m!r}")
        if not np.isfinite(sig) or sig <= 0.0:
            raise ValueError(f"sigma_m must be finite and positive, got {sigma_m!r}")
        self.z_priors.append((self.index[key], target, 1.0 / (sig * sig), robust))

    def pose(self, key) -> np.ndarray:
        return self.poses[self.index[key]].copy()

    def optimize(self, iterations: int = 20, *, use_cpp: bool = True) -> dict:
        """LM iterations; returns robust costs before/after and convergence."""
        if use_cpp:
            from .accel import cpp_optimize_pose_graph

            fast = cpp_optimize_pose_graph(
                self.poses,
                self.edges,
                self.gravity_priors,
                self.z_priors,
                cauchy_scale=self.cauchy_scale,
                iterations=iterations,
            )
            if fast is not None:
                self.poses, summary = fast
                return summary
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
        c2 = self.cauchy_scale**2
        for edge in self.edges:
            e = self._residual(poses, edge)
            s = e @ edge[3] @ e
            total += c2 * np.log1p(s / c2) if edge[4] else s
        for node_idx, g_body, weight, robust in self.gravity_priors:
            u = poses[node_idx][:3, :3] @ g_body
            e_g = np.array([u[0], u[1], u[2] - 1.0], dtype=np.float64)
            s = weight * float(e_g @ e_g)
            total += c2 * np.log1p(s / c2) if robust else s
        for node_idx, target_z, weight, robust in self.z_priors:
            e_z = float(poses[node_idx][2, 3]) - target_z
            s = weight * (e_z * e_z)
            total += c2 * np.log1p(s / c2) if robust else s
        return float(total)

    def _normal_equations(self, poses):
        """H and g over the free nodes (all but the first)."""
        size = 6 * (len(poses) - 1)
        rows, cols, values = [], [], []
        g = np.zeros(size)
        c2 = self.cauchy_scale**2
        for edge in self.edges:
            a, b, _, information, robust = edge
            e = self._residual(poses, edge)
            weighted = self._weight(e, information, robust) * information
            J_r_inv = se3_right_jacobian_inv(e)
            jacobians = {
                a: -(J_r_inv @ adjoint(invert(poses[b]) @ poses[a])),
                b: J_r_inv,
            }
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
        for node_idx, g_body, weight, robust in self.gravity_priors:
            if node_idx == 0:
                continue
            R = poses[node_idx][:3, :3]
            u = R @ g_body
            e_g = np.array([u[0], u[1], u[2] - 1.0], dtype=np.float64)
            s = weight * float(e_g @ e_g)
            w_eff = (weight / (1.0 + s / c2)) if robust else weight
            # Right perturbation: R exp(phi^) g_b ≈ R (g_b - [g_b]_x phi)
            J_rot = -(R @ hat(g_body))  # (3, 3)
            slot = 6 * (node_idx - 1) + 3
            g[slot : slot + 3] += w_eff * (J_rot.T @ e_g)
            H_rot = w_eff * (J_rot.T @ J_rot)
            br, bc = np.meshgrid(np.arange(slot, slot + 3), np.arange(slot, slot + 3), indexing="ij")
            rows.append(br.ravel())
            cols.append(bc.ravel())
            values.append(H_rot.ravel())
        for node_idx, target_z, weight, robust in self.z_priors:
            if node_idx == 0:
                continue
            e_z = float(poses[node_idx][2, 3]) - target_z
            s = weight * (e_z * e_z)
            w_eff = (weight / (1.0 + s / c2)) if robust else weight
            J_t = poses[node_idx][2, :3]
            slot = 6 * (node_idx - 1)
            g[slot : slot + 3] += (w_eff * e_z) * J_t
            H_t = w_eff * np.outer(J_t, J_t)
            br, bc = np.meshgrid(np.arange(slot, slot + 3), np.arange(slot, slot + 3), indexing="ij")
            rows.append(br.ravel())
            cols.append(bc.ravel())
            values.append(H_t.ravel())
        H = sparse.coo_matrix(
            (np.concatenate(values), (np.concatenate(rows), np.concatenate(cols))), shape=(size, size)
        )
        return H.tocsr(), g

    def _retract(self, step) -> list[np.ndarray]:
        moved = [self.poses[0]]
        for i, T in enumerate(self.poses[1:]):
            moved.append(T @ se3_exp(step[6 * i : 6 * i + 6]))
        return moved
