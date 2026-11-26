"""Incremental 3D surface-visibility texture keyframe selection and Set-Cover pruning.

Pure-CPU (NumPy + stdlib) implementation usable by both Habitat RGB-D and DynaNav/SLAM
streams. Separates visual texture coverage from SLAM odometry keyframes:

1. Online incremental admission: projects depth (or SLAM 3D points) to a 3D surface
   voxel ledger where each surface voxel tracks its best visual observation quality
   ``q(s) = max(0, n_s . v) / (0.35 + d^2)``. A frame is admitted into the candidate
   texture pool only when it covers enough new surface voxels or significantly upgrades
   viewing quality (e.g. oblique/far glimpse -> fronto-parallel close-up).
2. Delayed sliding-window / region-exit Submodular Set-Cover pruning: selects a minimal
   subset of keyframes covering >= ``target_coverage`` (default 92%) of the weighted
   3D surface voxels in a region, eliminating redundant transition frames while
   preserving full visual texture reconstruction coverage.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Mapping, Sequence

import numpy as np


@dataclass
class TextureKeyframeRecord:
    """Compact surface-visibility footprint of one admitted texture candidate frame."""

    keyframe_id: str
    region_hint: str
    pose_xyz: tuple[float, float, float]
    voxel_quality: dict[tuple[int, int, int], float] = field(default_factory=dict)
    total_weight: float = 0.0
    retained: bool = True
    pruned_by_set_cover: bool = False


@dataclass(frozen=True)
class TextureAdmissionResult:
    """Online marginal-coverage evaluation for an incoming sensor frame."""

    admit_texture: bool
    reason: str
    observed_voxels: int
    new_voxels: int
    upgraded_voxels: int
    marginal_gain_ratio: float
    mean_quality: float


@dataclass(frozen=True)
class SetCoverPruneResult:
    """Outcome of delayed greedy submodular Set-Cover pruning over a region."""

    region_id: str
    candidate_count: int
    selected_keyframe_ids: tuple[str, ...]
    pruned_keyframe_ids: tuple[str, ...]
    total_voxels: int
    covered_voxels: int
    weighted_coverage_ratio: float


def extract_rgbd_surface_voxels(
    depth_m: np.ndarray,
    T_world_body: np.ndarray,
    *,
    voxel_m: float = 0.14,
    hfov_deg: float = 90.0,
    camera_height_m: float = 0.88,
    min_range_m: float = 0.30,
    max_range_m: float = 6.0,
    stride: int = 8,
) -> dict[tuple[int, int, int], float]:
    """Project an RGB-D depth frame into world surface voxels with viewing-quality weights.

    Quality per surface sample is ``cos(incidence) / (0.35 + d^2)``, rewarding close,
    fronto-parallel views and penalizing grazing oblique angles or distant views.
    """
    arr = depth_m[:, :, 0] if depth_m.ndim == 3 else depth_m
    h, w = arr.shape[:2]
    if h < 4 or w < 4:
        return {}

    st = max(1, int(stride))
    sub = np.asarray(arr[::st, ::st], dtype=np.float64)
    sh, sw = sub.shape
    if sh < 3 or sw < 3:
        return {}

    fx = (0.5 * float(w)) / math.tan(math.radians(float(hfov_deg)) * 0.5)
    fy = fx
    cx = 0.5 * float(w)
    cy = 0.5 * float(h)

    us = np.arange(0, w, st, dtype=np.float64)[None, :]
    vs = np.arange(0, h, st, dtype=np.float64)[:, None]

    # Robotics Z-up camera/body frame: +x forward, +y left, +z up
    valid = np.isfinite(sub) & (sub >= float(min_range_m)) & (sub <= float(max_range_m))
    if not np.any(valid):
        return {}

    x_c = np.where(valid, sub, 0.0)
    y_c = np.where(valid, -(us - cx) * sub / fx, 0.0)
    z_c = np.where(valid, -(vs - cy) * sub / fy + float(camera_height_m), 0.0)

    # Estimate surface normals via central finite differences on the subsampled 3D grid
    pts_grid = np.stack([x_c, y_c, z_c], axis=-1)  # (sh, sw, 3)
    du = np.zeros_like(pts_grid)
    dv = np.zeros_like(pts_grid)
    du[:, 1:-1, :] = pts_grid[:, 2:, :] - pts_grid[:, :-2, :]
    dv[1:-1, :, :] = pts_grid[2:, :, :] - pts_grid[:-2, :, :]

    # Require neighbors to be valid and not depth discontinuities (> 0.45 m jump)
    valid_inner = np.zeros_like(valid)
    valid_inner[1:-1, 1:-1] = (
        valid[1:-1, 1:-1]
        & valid[1:-1, :-2]
        & valid[1:-1, 2:]
        & valid[:-2, 1:-1]
        & valid[2:, 1:-1]
        & (np.abs(sub[1:-1, 2:] - sub[1:-1, :-2]) <= 0.45)
        & (np.abs(sub[2:, 1:-1] - sub[:-2, 1:-1]) <= 0.45)
    )

    normals = np.cross(du, dv)
    n_norm = np.linalg.norm(normals, axis=-1, keepdims=True)
    good_n = valid_inner & (n_norm[..., 0] > 1e-6)
    normals_unit = np.zeros_like(normals)
    normals_unit[good_n] = normals[good_n] / n_norm[good_n]

    # Viewing direction from surface point back to camera center (0, 0, camera_height_m)
    view_vec = np.stack(
        [-x_c, -y_c, float(camera_height_m) - z_c],
        axis=-1,
    )
    v_norm = np.linalg.norm(view_vec, axis=-1, keepdims=True)
    view_unit = np.divide(view_vec, np.maximum(v_norm, 1e-6))

    # Absolute cosine because cross-product sign depends on u/v handedness
    cos_inc = np.abs(np.sum(normals_unit * view_unit, axis=-1))
    # For valid boundary/edge points lacking smooth neighbor gradients, assign conservative 0.45
    cos_eff = np.where(good_n, np.clip(cos_inc, 0.05, 1.0), 0.45)

    qual = cos_eff / (0.35 + sub * sub)

    # Filter out floor-only grazing points? Keep all surfaces (floors + walls + furniture)
    # because floors/carpets also carry visual texture cues, just naturally down-weighted by cos_inc.
    mask = valid & (qual > 0.01)
    if not np.any(mask):
        return {}

    pts_local = pts_grid[mask]  # (N, 3)
    q_vals = qual[mask]  # (N,)

    T = np.asarray(T_world_body, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    pts_world = pts_local @ R.T + t[None, :]

    inv_vox = 1.0 / max(0.02, float(voxel_m))
    v_idx = np.floor(pts_world * inv_vox).astype(np.int64)

    voxel_q: dict[tuple[int, int, int], float] = {}
    for vx, vy, vz, qv in zip(v_idx[:, 0], v_idx[:, 1], v_idx[:, 2], q_vals):
        key = (int(vx), int(vy), int(vz))
        qf = float(qv)
        prev = voxel_q.get(key)
        if prev is None or qf > prev:
            voxel_q[key] = qf
    return voxel_q


def extract_pointcloud_surface_voxels(
    points_world: np.ndarray,
    camera_xyz_world: Sequence[float],
    *,
    normals_world: np.ndarray | None = None,
    voxel_m: float = 0.14,
    min_range_m: float = 0.30,
    max_range_m: float = 6.0,
) -> dict[tuple[int, int, int], float]:
    """Generic SLAM point-cloud surface voxel quality extractor (for Isaac/DynaNav reuse)."""
    pts = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    if pts.size == 0:
        return {}
    cam = np.asarray(camera_xyz_world, dtype=np.float64).reshape(3)
    diff = cam[None, :] - pts
    dist = np.linalg.norm(diff, axis=1)
    valid = np.all(np.isfinite(pts), axis=1) & (dist >= float(min_range_m)) & (dist <= float(max_range_m))
    if not np.any(valid):
        return {}
    pts_v = pts[valid]
    dist_v = dist[valid]
    if normals_world is not None:
        nrm = np.asarray(normals_world, dtype=np.float64).reshape(-1, 3)[valid]
        n_norm = np.linalg.norm(nrm, axis=1, keepdims=True)
        n_unit = np.divide(nrm, np.maximum(n_norm, 1e-6))
        v_unit = diff[valid] / np.maximum(dist_v[:, None], 1e-6)
        cos_eff = np.clip(np.abs(np.sum(n_unit * v_unit, axis=1)), 0.05, 1.0)
    else:
        cos_eff = np.full(dist_v.shape, 0.70, dtype=np.float64)
    q_vals = cos_eff / (0.35 + dist_v * dist_v)

    inv_vox = 1.0 / max(0.02, float(voxel_m))
    v_idx = np.floor(pts_v * inv_vox).astype(np.int64)
    voxel_q: dict[tuple[int, int, int], float] = {}
    for vx, vy, vz, qv in zip(v_idx[:, 0], v_idx[:, 1], v_idx[:, 2], q_vals):
        key = (int(vx), int(vy), int(vz))
        qf = float(qv)
        prev = voxel_q.get(key)
        if prev is None or qf > prev:
            voxel_q[key] = qf
    return voxel_q


class TextureKeyframeSelector:
    """Maintains global 3D textured-surface ledger and minimal Set-Cover keyframes per region."""

    def __init__(
        self,
        *,
        voxel_m: float = 0.14,
        min_marginal_gain: float = 0.16,
        min_new_voxels: int = 20,
        quality_upgrade_ratio: float = 1.45,
        set_cover_target: float = 0.92,
        max_keyframes_per_region: int = 6,
    ) -> None:
        self.voxel_m = float(voxel_m)
        self.min_marginal_gain = float(min_marginal_gain)
        self.min_new_voxels = max(1, int(min_new_voxels))
        self.quality_upgrade_ratio = max(1.05, float(quality_upgrade_ratio))
        self.set_cover_target = min(0.99, max(0.50, float(set_cover_target)))
        self.max_keyframes_per_region = max(1, int(max_keyframes_per_region))

        self.global_voxel_quality: dict[tuple[int, int, int], float] = {}
        self.records: dict[str, TextureKeyframeRecord] = {}
        self.prune_history: list[SetCoverPruneResult] = []
        self.total_frames_evaluated: int = 0
        self.total_candidates_admitted: int = 0

    def evaluate_voxels(
        self,
        voxel_quality: Mapping[tuple[int, int, int], float],
        *,
        force: bool = False,
    ) -> TextureAdmissionResult:
        """Compute marginal surface-texture gain of a candidate frame without mutating state."""
        self.total_frames_evaluated += 1
        n_obs = len(voxel_quality)
        if n_obs == 0:
            return TextureAdmissionResult(
                admit_texture=bool(force),
                reason=("forced_empty" if force else "no_valid_surface_voxels"),
                observed_voxels=0,
                new_voxels=0,
                upgraded_voxels=0,
                marginal_gain_ratio=0.0,
                mean_quality=0.0,
            )

        n_new = 0
        n_upg = 0
        q_sum = 0.0
        upg_ratio = self.quality_upgrade_ratio
        g_map = self.global_voxel_quality
        face_nb = (
            (0, 0, 0),
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1),
        )
        for key, qv in voxel_quality.items():
            q_sum += qv
            vx, vy, vz = key
            prev = 0.0
            found = False
            for dx, dy, dz in face_nb:
                v_nb = g_map.get((vx + dx, vy + dy, vz + dz))
                if v_nb is not None:
                    found = True
                    if v_nb > prev:
                        prev = v_nb
            if not found:
                n_new += 1
            elif qv >= prev * upg_ratio:
                n_upg += 1

        eff_gain = float(n_new) + 0.70 * float(n_upg)
        ratio = eff_gain / float(n_obs)
        mean_q = q_sum / float(n_obs)

        if force:
            return TextureAdmissionResult(
                admit_texture=True,
                reason="forced_anchor",
                observed_voxels=n_obs,
                new_voxels=n_new,
                upgraded_voxels=n_upg,
                marginal_gain_ratio=round(ratio, 4),
                mean_quality=round(mean_q, 4),
            )

        if not self.records:
            return TextureAdmissionResult(
                admit_texture=True,
                reason="initial_texture_seed",
                observed_voxels=n_obs,
                new_voxels=n_new,
                upgraded_voxels=n_upg,
                marginal_gain_ratio=round(ratio, 4),
                mean_quality=round(mean_q, 4),
            )

        admit = (ratio >= self.min_marginal_gain) and ((n_new + n_upg) >= self.min_new_voxels)
        reason = (
            "new_surface_coverage"
            if (admit and n_new >= n_upg)
            else ("closeup_quality_upgrade" if admit else "redundant_surface_view")
        )
        return TextureAdmissionResult(
            admit_texture=admit,
            reason=reason,
            observed_voxels=n_obs,
            new_voxels=n_new,
            upgraded_voxels=n_upg,
            marginal_gain_ratio=round(ratio, 4),
            mean_quality=round(mean_q, 4),
        )

    def commit_candidate(
        self,
        keyframe_id: str,
        voxel_quality: Mapping[tuple[int, int, int], float],
        *,
        pose_xyz: Sequence[float],
        region_hint: str = "",
    ) -> TextureKeyframeRecord:
        """Register an admitted texture keyframe and update the global 3D surface ledger."""
        vq_copy = dict(voxel_quality)
        for key, qv in vq_copy.items():
            prev = self.global_voxel_quality.get(key)
            if prev is None or qv > prev:
                self.global_voxel_quality[key] = qv
        rec = TextureKeyframeRecord(
            keyframe_id=str(keyframe_id),
            region_hint=str(region_hint or ""),
            pose_xyz=(
                round(float(pose_xyz[0]), 3),
                round(float(pose_xyz[1]), 3),
                round(float(pose_xyz[2]), 3),
            ),
            voxel_quality=vq_copy,
            total_weight=round(sum(vq_copy.values()), 4),
            retained=True,
            pruned_by_set_cover=False,
        )
        self.records[rec.keyframe_id] = rec
        self.total_candidates_admitted += 1
        return rec

    def prune_region_set_cover(
        self,
        region_id: str,
        candidate_kf_ids: Sequence[str],
        *,
        target_coverage: float | None = None,
        max_keyframes: int | None = None,
        min_marginal_fraction: float = 0.015,
    ) -> SetCoverPruneResult:
        """Select a minimal Set-Cover subset of texture keyframes for one region.

        Uses greedy weighted submodular maximization over the union of 3D surface voxels
        observed by ``candidate_kf_ids``. Provides the standard (1 - 1/e) approximation
        guarantee for weighted surface coverage.
        """
        cands = [
            self.records[kid]
            for kid in dict.fromkeys(str(k) for k in candidate_kf_ids)
            if kid in self.records and self.records[kid].voxel_quality
        ]
        if not cands:
            res = SetCoverPruneResult(
                region_id=str(region_id),
                candidate_count=0,
                selected_keyframe_ids=(),
                pruned_keyframe_ids=(),
                total_voxels=0,
                covered_voxels=0,
                weighted_coverage_ratio=0.0,
            )
            return res

        # Build the union best-achievable surface quality map for this region
        target_q: dict[tuple[int, int, int], float] = {}
        for rec in cands:
            for key, qv in rec.voxel_quality.items():
                prev = target_q.get(key)
                if prev is None or qv > prev:
                    target_q[key] = qv

        total_target_weight = sum(target_q.values())
        if total_target_weight <= 1e-9:
            first_id = cands[0].keyframe_id
            return SetCoverPruneResult(
                region_id=str(region_id),
                candidate_count=len(cands),
                selected_keyframe_ids=(first_id,),
                pruned_keyframe_ids=tuple(r.keyframe_id for r in cands[1:]),
                total_voxels=len(target_q),
                covered_voxels=len(cands[0].voxel_quality),
                weighted_coverage_ratio=1.0,
            )

        cov_goal = self.set_cover_target if target_coverage is None else float(target_coverage)
        k_cap = self.max_keyframes_per_region if max_keyframes is None else max(1, int(max_keyframes))

        covered_q: dict[tuple[int, int, int], float] = {}
        covered_weight = 0.0
        selected: list[str] = []
        remaining = list(cands)

        while remaining and len(selected) < k_cap:
            best_rec: TextureKeyframeRecord | None = None
            best_gain = -1.0
            for rec in remaining:
                gain = 0.0
                for key, qv in rec.voxel_quality.items():
                    cur = covered_q.get(key, 0.0)
                    if qv > cur:
                        gain += qv - cur
                if gain > best_gain:
                    best_gain = gain
                    best_rec = rec

            if best_rec is None or best_gain <= 1e-9:
                break
            if selected and (best_gain / total_target_weight) < float(min_marginal_fraction):
                break

            selected.append(best_rec.keyframe_id)
            remaining.remove(best_rec)
            for key, qv in best_rec.voxel_quality.items():
                cur = covered_q.get(key, 0.0)
                if qv > cur:
                    covered_q[key] = qv
                    covered_weight += qv - cur

            if (covered_weight / total_target_weight) >= cov_goal:
                break

        selected_set = set(selected)
        pruned: list[str] = []
        for rec in cands:
            if rec.keyframe_id in selected_set:
                rec.retained = True
                rec.pruned_by_set_cover = False
            else:
                rec.retained = False
                rec.pruned_by_set_cover = True
                pruned.append(rec.keyframe_id)

        result = SetCoverPruneResult(
            region_id=str(region_id),
            candidate_count=len(cands),
            selected_keyframe_ids=tuple(selected),
            pruned_keyframe_ids=tuple(pruned),
            total_voxels=len(target_q),
            covered_voxels=len(covered_q),
            weighted_coverage_ratio=round(min(1.0, covered_weight / total_target_weight), 4),
        )
        self.prune_history.append(result)
        return result

    def region_surface_stats(self, kf_ids: Sequence[str]) -> tuple[int, float]:
        """Return ``(unique_voxels, retained_weighted_coverage)`` for a set of keyframes."""
        all_q: dict[tuple[int, int, int], float] = {}
        ret_q: dict[tuple[int, int, int], float] = {}
        for kid in kf_ids:
            rec = self.records.get(str(kid))
            if rec is None:
                continue
            for key, qv in rec.voxel_quality.items():
                if qv > all_q.get(key, 0.0):
                    all_q[key] = qv
                if rec.retained and qv > ret_q.get(key, 0.0):
                    ret_q[key] = qv
        if not all_q:
            return 0, 0.0
        tot = sum(all_q.values())
        cov = sum(ret_q.values())
        return len(all_q), round(min(1.0, cov / max(1e-9, tot)), 4)

    def diagnostic_metrics(self) -> dict[str, Any]:
        retained = sum(1 for r in self.records.values() if r.retained)
        pruned = sum(1 for r in self.records.values() if r.pruned_by_set_cover)
        return {
            "voxel_m": self.voxel_m,
            "total_frames_evaluated": self.total_frames_evaluated,
            "total_candidates_admitted": self.total_candidates_admitted,
            "retained_texture_keyframes": retained,
            "pruned_redundant_keyframes": pruned,
            "global_textured_voxels": len(self.global_voxel_quality),
            "prune_runs": len(self.prune_history),
        }
