"""Small CPU localization against an explicitly supplied static scene map.

No ROS, simulator pose, IMU orientation, frontend covariance, or route answer is
used by the objective. This is known-scene map assisted localization, not an
independently built sensor submap. Numerical gates are engineering limits, not
a calibrated posterior probability. A failed gate produces no correction.
"""

from __future__ import annotations
from embodied_navigation.common.integration_limits import (
    MATCH_CORRECTION_MAX_AGE_NS,
    RAY_ABS_P95_LIMIT_M,
    LIDAR_BODY_TRANSLATION_M,
)

from dataclasses import dataclass, asdict
from pathlib import Path
import hashlib
import json
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation

STATIC_SUPPORT_QUALITY_MODEL = "static_support_nonforeground/v1"


def checked_transform(value):
    """Validate and return a copy of a finite rigid 4x4 transform.

    Rejects wrong shapes, non-finite entries, a non-bottom-row [0,0,0,1], a
    non-orthonormal rotation block (``atol=1e-6``) and a determinant far from 1
    (``1e-6``), so downstream code never silently accepts a scaled/sheared pose.
    """
    t = np.asarray(value, dtype=float)
    if (
        t.shape != (4, 4)
        or not np.isfinite(t).all()
        or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-10)
        or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6)
        or abs(np.linalg.det(t[:3, :3]) - 1) > 1e-6
    ):
        raise ValueError("invalid rigid transform")
    return t.copy()


def transform_distance(a, b):
    """Return (translation_metres, rotation_radians) between two rigid transforms."""
    return (
        float(np.linalg.norm(a[:3, 3] - b[:3, 3])),
        float(Rotation.from_matrix(a[:3, :3] @ b[:3, :3].T).magnitude()),
    )


@dataclass(frozen=True)
class MatchPolicy:
    """Engineering gates for one match; defaults are limits, not a posterior.

    Distances are metres, angles radians, times seconds/nanoseconds. A candidate
    must pass every gate in ``KnownSceneMatcher.score``; changing a default here
    changes admission behaviour, so the legacy safety bounds are re-checked in
    ``KnownSceneMatcher.__init__``.
    """

    fit_points: int = 900
    score_points: int = 4000
    minimum_points: int = 400
    iterations: int = 40
    compute_budget_s: float = 1.6
    correspondence_gate_m: float = 0.5
    huber_m: float = 0.06
    rms_max_m: float = 0.060
    foreground_max_fraction: float | None = 0.05
    foreground_negative_first_hit_threshold_m: float = 0.12
    foreground_minimum_surface_distance_m: float = 0.1
    surface_p95_max_m: float = 0.12
    inlier_fraction_min: float = 0.90
    ray_hit_fraction_min: float = 0.90
    ray_behind_max_fraction: float = 0.12
    ray_abs_median_max_m: float = 0.045
    ray_abs_p95_max_m: float = RAY_ABS_P95_LIMIT_M
    ray_quality_population: str = "all"
    quality_model: str = "legacy"
    geometry_ratio_min: float = 1e-5
    max_initial_correction_m: float = 0.65
    max_initial_rotation_rad: float = 0.20
    ambiguity_translation_m: float = 0.10
    ambiguity_rotation_rad: float = 0.10
    ambiguity_score_margin_m: float = 0.012
    nominal_body_clearance_seed_m: float = 0.30
    independent_confirmations: int = 2
    confirmation_min_dt_ns: int = 150_000_000
    confirmation_max_dt_ns: int = 10_000_000_000
    confirmation_c_translation_m: float = 0.15
    confirmation_c_rotation_rad: float = 0.08


def policy_for_profile(profile):
    model = profile.get("known_scene_quality_model", "legacy")
    if type(model) is not str or model not in ("legacy", STATIC_SUPPORT_QUALITY_MODEL):
        raise ValueError("Explicit known-scene quality model differs")
    if model == STATIC_SUPPORT_QUALITY_MODEL:
        # This replaces, rather than claims to retain, the legacy 5% budget.
        return MatchPolicy(
            quality_model=model,
            foreground_max_fraction=None,
            ray_quality_population="nonforeground",
        )
    # New explicit quality population, not an equivalent interpretation of all-ray.
    return MatchPolicy(
        ray_quality_population=profile.get("known_scene_ray_quality_population", "all")
    )


class SceneReference:
    """Static analytic box map loaded from one explicitly supplied JSON file.

    Boxes are axis-aligned in their own frame with a rigid ``T_world_box`` and
    positive ``size_m``; ``nearest``/``rays`` answer surface-distance and
    origin-to-point first-hit queries against the same packed geometry, and
    ``seed_heights`` derives floor/ramp top and bottom candidate hypotheses.
    """

    def __init__(self, data, sha256=None):
        if data.get("schema") != "given_scene_analytic_box_map/v1":
            raise ValueError("unsupported reference schema")
        self.sha256 = sha256
        self.source = data["source"]
        self.boxes = data["boxes"]
        if not 1 <= len(self.boxes) <= 10000:
            raise ValueError("Scene requires a bounded nonempty box set")
        self.packed = []
        for b in self.boxes:
            t = checked_transform(b["T_world_box"])
            h = np.asarray(b["size_m"], dtype=float) / 2
            if h.shape != (3,) or not np.isfinite(h).all() or np.any(h <= 0):
                raise ValueError("invalid box")
            self.packed.append((t[:3, :3], t[:3, 3], h))

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text()))

    def nearest(self, points):
        best = np.full(len(points), np.inf)
        nearest = np.zeros_like(points)
        normal = np.zeros_like(points)
        owners = np.zeros(len(points), dtype=np.int16)
        faces = owners.copy()
        for i, (r, origin, half) in enumerate(self.packed):
            p = (points - origin) @ r
            q = np.clip(p, -half, half)
            inside = np.all(np.abs(p) <= half, axis=1)
            axis = np.argmin(half - np.abs(p), axis=1)
            sign = np.where(p[np.arange(len(p)), axis] >= 0, 1.0, -1.0)
            ix = np.flatnonzero(inside)
            q[ix, axis[inside]] = sign[inside] * half[axis[inside]]
            delta = p - q
            d = np.linalg.norm(delta, axis=1)
            n = delta / np.maximum(d[:, None], 1e-12)
            n[ix] = 0
            n[ix, axis[inside]] = sign[inside]
            face_axis = np.where(inside, axis, np.argmax(np.abs(p) - half, axis=1))
            label = 2 * face_axis + (p[np.arange(len(p)), face_axis] >= 0)
            take = d < best
            best[take] = d[take]
            nearest[take] = q[take] @ r.T + origin
            normal[take] = n[take] @ r.T
            owners[take] = i
            faces[take] = label[take]
        return best, nearest, normal, owners, faces

    def rays(self, points, origin, *, return_arrays=False):
        delta = points - origin
        observed = np.linalg.norm(delta, axis=1)
        direction = delta / np.maximum(observed[:, None], 1e-12)
        first = np.full(len(points), np.inf)
        for r, center, half in self.packed:
            o = (origin - center) @ r
            d = direction @ r
            parallel = np.abs(d) < 1e-12
            safe = np.where(parallel, 1.0, d)
            a, b = (-half - o) / safe, (half - o) / safe
            lo = np.where(parallel, -np.inf, np.minimum(a, b)).max(1)
            hi = np.where(parallel, np.inf, np.maximum(a, b)).min(1)
            valid = (hi >= np.maximum(lo, 0)) & ~np.any(parallel & (np.abs(o) > half), axis=1)
            first = np.minimum(first, np.where(valid, np.where(lo >= 0, lo, hi), np.inf))
        valid = np.isfinite(first)
        error = observed[valid] - first[valid]
        if not len(error):
            summary = dict(hit_fraction=0.0, behind_fraction=1.0, abs_median_m=1e6, abs_p95_m=1e6)
        else:
            summary = dict(
                hit_fraction=float(valid.mean()),
                behind_fraction=float(np.mean(error > 0.1)),
                abs_median_m=float(np.median(np.abs(error))),
                abs_p95_m=float(np.quantile(np.abs(error), 0.95)),
            )
        # The original summary and its denominators never exclude foreground.
        return (summary, observed, first) if return_arrays else summary

    def seed_heights(self, xy, clearance):
        # Map geometry determines hypotheses only. No expected route floor enters
        # the cost or gate. Both top AND bottom are retained to expose aliases.
        heights = []
        for b, (r, center, half) in zip(self.boxes, self.packed):
            if not b["model"].startswith(("floor_", "ramp_")):
                continue
            for sign in (-1.0, 1.0):
                n = r[:, 2]
                if abs(n[2]) < 0.5:
                    continue
                face = center + sign * half[2] * n
                z = face[2] - np.dot(n[:2], xy - face[:2]) / n[2]
                local = (np.r_[xy, z] - center) @ r
                if np.all(np.abs(local[:2]) <= half[:2] + 0.5):
                    heights.append(float(z + clearance))
        return sorted(set(round(z, 5) for z in heights))


class KnownSceneMatcher:
    """Fit, score, gate and confirm one known-scene pose correction.

    ``match`` returns a data-only result dict (never a published transform):
    candidates are ranked, ambiguity/jump checks reject unsafe winners, and a
    correction is accepted after independent confirmations within policy windows.
    No-quality and ambiguous-map outcomes clear pending state; other rejection/wait
    branches may preserve it, while inconsistent confirmations restart it.
    """

    def __init__(self, reference, policy=None):
        self.reference = reference
        self.policy = policy or MatchPolicy()
        if type(
            self.policy.ray_quality_population
        ) is not str or self.policy.ray_quality_population not in ("all", "nonforeground"):
            raise ValueError("Explicit all/nonforeground ray quality population required")
        if self.policy.quality_model not in ("legacy", STATIC_SUPPORT_QUALITY_MODEL):
            raise ValueError("Explicit known-scene quality model differs")
        if self.policy.quality_model == STATIC_SUPPORT_QUALITY_MODEL:
            if (
                self.policy.foreground_max_fraction is not None
                or self.policy.ray_quality_population != "nonforeground"
                or self.policy.inlier_fraction_min != 0.90
            ):
                raise ValueError(
                    "Static-support model requires explicit no-FG-cap and .90 all-point support"
                )
        elif (
            not math.isfinite(self.policy.foreground_max_fraction)
            or not 0 <= self.policy.foreground_max_fraction <= 0.05
        ):
            raise ValueError("Legacy foreground policy must retain the .05 safety bound")
        if (
            not math.isfinite(self.policy.foreground_negative_first_hit_threshold_m)
            or self.policy.foreground_negative_first_hit_threshold_m < 0.12
        ):
            raise ValueError("foreground policy must retain the .05/.12 safety bounds")
        self.pending = None
        self.confirmations = 0
        self.query_capture = None  # Optional data-only observer, no new worker.

    def _fit(self, local, initial, deadline):
        p = self.policy
        t = initial.copy()
        converged = False
        iteration = 0
        for iteration in range(p.iterations):
            if time.monotonic() > deadline:
                break
            moved = local @ t[:3, :3].T + t[:3, 3]
            d, near, n, _, _ = self.reference.nearest(moved)
            valid = d < p.correspondence_gate_m
            if valid.sum() < min(100, len(local) // 2):
                break
            lever = moved[valid] - t[:3, 3]
            j = np.column_stack((n[valid], np.cross(lever, n[valid])))
            residual = np.sum(n[valid] * (moved[valid] - near[valid]), axis=1)
            weight = np.sqrt(np.minimum(1.0, p.huber_m / np.maximum(np.abs(residual), 1e-12)))
            step = np.linalg.lstsq(j * weight[:, None], -residual * weight, rcond=1e-8)[0]
            step[:3] *= min(1.0, 0.2 / max(np.linalg.norm(step[:3]), 1e-12))
            step[3:] *= min(1.0, 0.05 / max(np.linalg.norm(step[3:]), 1e-12))
            t[:3, 3] += step[:3]
            t[:3, :3] = Rotation.from_rotvec(step[3:]).as_matrix() @ t[:3, :3]
            if np.linalg.norm(step) < 1e-5:
                converged = True
                break
        return t, converged, iteration + 1

    def score(self, body_points, pose, mount, converged=True):
        p = self.policy
        moved = body_points @ pose[:3, :3].T + pose[:3, 3]
        d, _, n, owners, faces = self.reference.nearest(moved)
        valid = d < 0.1
        rays, observed, first = self.reference.rays(
            moved, pose[:3, 3] + pose[:3, :3] @ mount, return_arrays=True
        )
        foreground = (
            np.isfinite(observed)
            & (observed > 0)
            & np.isfinite(first)
            & (first > 0)
            & (observed - first < -p.foreground_negative_first_hit_threshold_m)
            & (d >= p.foreground_minimum_surface_distance_m)
        )
        static = ~foreground
        static_valid = valid & static
        j = np.column_stack(
            (n[static_valid], np.cross(moved[static_valid] - pose[:3, 3], n[static_valid]))
        )
        eigen = np.linalg.eigvalsh(j.T @ j / max(len(j), 1))
        counts = {}
        for owner, face in zip(owners[valid], faces[valid]):
            label = self.reference.boxes[owner]["surface_ids"][face]
            counts[label] = counts.get(label, 0) + 1
        static_counts = {}
        for owner, face in zip(owners[static_valid], faces[static_valid]):
            label = self.reference.boxes[owner]["surface_ids"][face]
            static_counts[label] = static_counts.get(label, 0) + 1
        indices = np.flatnonzero(foreground)
        sample_indices = (
            indices[np.linspace(0, len(indices) - 1, min(32, len(indices)), dtype=int)]
            if len(indices)
            else []
        )
        samples = [
            dict(
                point_index_in_score_cloud=int(i),
                point_world_xyz=moved[i].tolist(),
                point_body_xyz=body_points[i].tolist(),
                observed_range_m=float(observed[i]),
                expected_first_hit_range_m=float(first[i]),
                range_error_m=float(observed[i] - first[i]),
                surface_distance_m=float(d[i]),
            )
            for i in sample_indices
        ]
        ray_valid = np.isfinite(first)
        error = observed - first
        ray_static = ray_valid & static

        def ray_summary(mask):
            values = error[mask]
            return dict(
                count=int(mask.sum()),
                abs_p95_m=float(np.quantile(np.abs(values), 0.95)) if len(values) else None,
                negative_beyond_original_p95_limit=int((values < -p.ray_abs_p95_max_m).sum()),
                positive_beyond_original_p95_limit=int((values > p.ray_abs_p95_max_m).sum()),
            )

        tails = np.flatnonzero(ray_static & (observed > 0) & (np.abs(error) > p.ray_abs_p95_max_m))
        tails = (
            tails[np.linspace(0, len(tails) - 1, min(32, len(tails)), dtype=int)]
            if len(tails)
            else []
        )
        origin = pose[:3, 3] + pose[:3, :3] @ mount
        tail_samples = [
            dict(
                point_body_xyz=body_points[i].tolist(),
                point_world_xyz=moved[i].tolist(),
                observed_range_m=float(observed[i]),
                expected_first_hit_range_m=float(first[i]),
                range_error_m=float(error[i]),
                surface_distance_m=float(d[i]),
                surface_id=self.reference.boxes[owners[i]]["surface_ids"][faces[i]],
                nearest_surface_normal_world=n[i].tolist(),
                ray_direction_world=((moved[i] - origin) / observed[i]).tolist(),
            )
            for i in tails
        ]
        ray_diagnostic = dict(
            schema="known_scene_signed_ray_tail_diagnostic/v1",
            quality_gates_changed=False,
            world_lidar_origin=origin.tolist(),
            all_valid=ray_summary(ray_valid),
            nonforeground=ray_summary(ray_static),
            foreground=ray_summary(ray_valid & foreground),
            nonforeground_tail_sample_count=len(tail_samples),
            nonforeground_tail_samples=tail_samples,
        )
        quality_rays = rays
        if p.ray_quality_population == "nonforeground":
            # Reuse original first-hit residuals and original static mask. No new
            # geometry query, foreground rule, point removal or Actor input.
            values = error[ray_static]
            quality_rays = (
                dict(
                    hit_fraction=float(ray_static.sum() / max(int(static.sum()), 1)),
                    behind_fraction=float(np.mean(values > 0.1)),
                    abs_median_m=float(np.median(np.abs(values))),
                    abs_p95_m=ray_diagnostic["nonforeground"]["abs_p95_m"],
                )
                if len(values)
                else dict(hit_fraction=0.0, behind_fraction=1.0, abs_median_m=1e6, abs_p95_m=1e6)
            )
        foreground_fraction = float(foreground.mean())
        result = dict(
            T_world_body=pose.tolist(),
            converged=bool(converged),
            rms_capped_m=float(np.sqrt(np.mean(np.minimum(d, 0.5) ** 2))),
            static_surface_rms_capped_m=(
                float(np.sqrt(np.mean(np.minimum(d[static], 0.5) ** 2))) if np.any(static) else 1e6
            ),
            surface_p95_m=float(np.quantile(d, 0.95)),
            inlier_fraction=float(valid.mean()),
            rays=rays,
            ray_quality_population=p.ray_quality_population,
            quality_rays=quality_rays,
            geometry_ratio=float(eigen[0] / max(eigen[-1], 1e-12)),
            geometry_eigenvalues=eigen.tolist(),
            surface_counts=counts,
            static_support_points=int(static_valid.sum()),
            nonforeground_points=int(static.sum()),
            static_surface_counts=static_counts,
            foreground=dict(
                schema="bounded_unmapped_negative_static_first_hit_foreground/v2",
                count=int(foreground.sum()),
                total_scored_points=len(d),
                fraction=foreground_fraction,
                max_fraction=p.foreground_max_fraction,
                negative_first_hit_threshold_m=p.foreground_negative_first_hit_threshold_m,
                minimum_surface_distance_m=p.foreground_minimum_surface_distance_m,
                classification_is_dynamic_object_proof=False,
                static_support_required_points=p.minimum_points,
                sample_count=len(samples),
                samples=samples,
            ),
        )
        new_model = p.quality_model == STATIC_SUPPORT_QUALITY_MODEL
        quality_surface_p95 = result["surface_p95_m"]
        coverage = result["inlier_fraction"]
        if new_model:
            # All nonforeground distances, including its non-inliers: not a
            # self-validating P95 over only d<.1 correspondence points.
            quality_surface_p95 = float(np.quantile(d[static], 0.95)) if np.any(static) else 1e6
            coverage = float(static_valid.sum() / max(len(d), 1))
            result.update(
                quality_model=p.quality_model,
                quality_surface_population="all_nonforeground_points",
                quality_surface_p95_m=quality_surface_p95,
                static_valid_fraction=coverage,
                quality_budget=dict(
                    population="static_valid / all_score_points",
                    minimum_fraction=p.inlier_fraction_min,
                    maximum_nonstatic_or_invalid_fraction=0.10,
                    foreground_max_fraction=None,
                    legacy_five_percent_guarantee_retained=False,
                ),
            )
        reasons = []
        gates = [(converged, "fit_not_converged")]
        if not new_model:
            gates.append((foreground_fraction <= p.foreground_max_fraction, "foreground_budget"))
        gates += [
            (result["static_support_points"] >= p.minimum_points, "static_support"),
            (result["static_surface_rms_capped_m"] <= p.rms_max_m, "surface_rms"),
            (
                quality_surface_p95 <= p.surface_p95_max_m,
                "nonforeground_surface_p95" if new_model else "surface_p95",
            ),
            (
                coverage >= p.inlier_fraction_min,
                "static_valid_fraction" if new_model else "surface_coverage",
            ),
            (result["geometry_ratio"] >= p.geometry_ratio_min, "geometry_degenerate"),
            (quality_rays["hit_fraction"] >= p.ray_hit_fraction_min, "ray_coverage"),
            (quality_rays["behind_fraction"] <= p.ray_behind_max_fraction, "ray_occlusion"),
            (quality_rays["abs_median_m"] <= p.ray_abs_median_max_m, "ray_median"),
            (quality_rays["abs_p95_m"] <= p.ray_abs_p95_max_m, "ray_p95"),
        ]
        reasons.extend(name for passed, name in gates if not passed)
        result["ray_tail_diagnostic"] = ray_diagnostic
        result["quality_passed"] = not reasons
        result["rejections"] = reasons
        # Rays dominate aliases, but never replace the individual gates.
        result["ranking_score_m"] = result["rms_capped_m"] + 0.2 * rays["behind_fraction"]
        return result

    def match(
        self,
        source_ns,
        body_points,
        T_odom_body,
        current_c=None,
        mount=LIDAR_BODY_TRANSLATION_M,
        provenance=None,
    ):
        if type(source_ns) is not int or source_ns < 0:
            raise ValueError("source_ns must be an integer")
        odom = checked_transform(T_odom_body)
        c = checked_transform(np.eye(4) if current_c is None else current_c)
        mount = np.asarray(mount, dtype=float)
        local = np.asarray(body_points, dtype=float)
        if local.ndim != 2 or local.shape[1] != 3:
            raise ValueError("invalid query cloud")
        local = local[np.isfinite(local).all(1)]
        base = dict(
            schema="known_scene_match/v1",
            source_ns=source_ns,
            reference_sha256=self.reference.sha256,
            reference_world_sha256=self.reference.source["sha256"],
            reference_kind="provided_simulation_scene_geometry",
            independent_sensor_map=False,
            robot_truth_pose_used=False,
            IMU_world_orientation_used=False,
            global_search_complete=False,
            calibrated_covariance=False,
            policy=asdict(self.policy),
            input_C_world_odom=c.tolist(),
            control_allowed=False,
            accepted=False,
        )
        if len(local) < self.policy.minimum_points:
            return dict(base, reason="insufficient_query", candidates=[])
        started = time.monotonic()
        deadline = started + self.policy.compute_budget_s
        fit_cloud = local[:: max(1, math.ceil(len(local) / self.policy.fit_points))]
        score_cloud = local[:: max(1, math.ceil(len(local) / self.policy.score_points))]
        predicted = c @ odom
        initial = []
        for z in self.reference.seed_heights(
            predicted[:2, 3], self.policy.nominal_body_clearance_seed_m
        ):
            t = predicted.copy()
            t[2, 3] = z
            initial.append(t)
        if not initial:
            return dict(base, reason="no_map_support_hypotheses", candidates=[])
        candidates = []
        completed = 0
        for seed in initial:
            if time.monotonic() > deadline:
                break
            t, converged, iterations = self._fit(fit_cloud, seed, deadline)
            completed += 1
            if any(
                transform_distance(t, checked_transform(q["T_world_body"]))[0] < 0.025
                and transform_distance(t, checked_transform(q["T_world_body"]))[1] < 0.025
                for q in candidates
            ):
                continue
            result = self.score(score_cloud, t, mount, converged)
            result["iterations"] = iterations
            result["initial_position"] = seed[:3, 3].tolist()
            candidates.append(result)
        candidates.sort(key=lambda q: q["ranking_score_m"])
        base.update(
            candidates=candidates,
            requested_seeds=len(initial),
            completed_seeds=completed,
            fit_points=len(fit_cloud),
            scored_points=len(score_cloud),
            query_points=len(local),
            duration_s=time.monotonic() - started,
            local_seed_domain="current pose XY/orientation; height exclusively all map top/bottom support hypotheses",
            frontend_height_used_as_hypothesis=False,
        )
        if self.query_capture is not None:
            # Original deadline/fit/score and duration sampling are complete.
            # Pending confirmation state has not been changed by this call yet.
            try:
                self.query_capture.prepared(
                    source_ns,
                    body_points,
                    odom,
                    c,
                    mount,
                    predicted,
                    initial,
                    provenance,
                    len(local),
                    self.pending,
                    self.confirmations,
                )
            except Exception:
                pass  # Observer failures cannot become matcher decisions.
        good = [q for q in candidates if q["quality_passed"]]
        if completed != len(initial):
            return dict(base, reason="finite_hypotheses_budget_exceeded")
        if not good:
            self.pending, self.confirmations = None, 0
            return dict(base, reason="no_quality_candidate")
        best = good[0]
        pose = checked_transform(best["T_world_body"])
        for other in good[1:]:
            dist, angle = transform_distance(pose, checked_transform(other["T_world_body"]))
            if (
                dist > self.policy.ambiguity_translation_m
                or angle > self.policy.ambiguity_rotation_rad
            ) and other["ranking_score_m"] - best[
                "ranking_score_m"
            ] < self.policy.ambiguity_score_margin_m:
                self.pending, self.confirmations = None, 0
                return dict(base, reason="ambiguous_map_pose")
        target_c = pose @ np.linalg.inv(odom)
        dist, angle = transform_distance(target_c, c)
        if (
            dist > self.policy.max_initial_correction_m
            or angle > self.policy.max_initial_rotation_rad
        ):
            return dict(base, reason="correction_jump_rejected")
        base.update(
            best=best,
            T_world_body=pose.tolist(),
            C_world_odom=target_c.tolist(),
            provenance=provenance or {},
        )
        if self.pending is None:
            self.pending = (source_ns, target_c)
            self.confirmations = 1
        else:
            ns, previous_c = self.pending
            delta_ns = source_ns - ns
            if delta_ns <= 0:
                return dict(base, reason="non_increasing_confirmation_stamp")
            if delta_ns < self.policy.confirmation_min_dt_ns:
                return dict(base, reason="confirmation_not_independent_yet")
            dist, angle = transform_distance(target_c, previous_c)
            if (
                delta_ns > self.policy.confirmation_max_dt_ns
                or dist > self.policy.confirmation_c_translation_m
                or angle > self.policy.confirmation_c_rotation_rad
            ):
                self.pending, self.confirmations = (source_ns, target_c), 1
                return dict(base, reason="confirmation_inconsistent", confirmations=1)
            self.pending = (source_ns, target_c)
            self.confirmations += 1
        if self.confirmations < self.policy.independent_confirmations:
            return dict(base, reason="await_independent_frame", confirmations=self.confirmations)
        return dict(
            base, reason="confirmed_shadow_match", accepted=True, confirmations=self.confirmations
        )


class CorrectionState:
    """A bounded map->odom transform; never changes frontend velocity or bias.

    Integration applies one returned snapshot atomically to pose AND cloud and
    invalidates an existing path when generation changes. A proposal alone has
    no navigation authority. Rotation acts on physical velocity; C derivatives
    are deliberately absent. Expired corrections are flagged, never snapped off.
    """

    def __init__(
        self,
        translation_rate_mps=0.04,
        rotation_rate_radps=0.02,
        max_match_age_ns=MATCH_CORRECTION_MAX_AGE_NS,
        initial_c=None,
    ):
        self.applied = checked_transform(np.eye(4) if initial_c is None else initial_c)
        self.target = self.applied.copy()
        self.generation = 0
        self.last_tick_ns = None
        self.last_match_ns = None
        self.translation_rate = translation_rate_mps
        self.rotation_rate = rotation_rate_radps
        self.max_match_age_ns = max_match_age_ns

    def propose(self, result):
        if result.get("accepted") is not True:
            return False
        ns = result["source_ns"]
        if self.last_match_ns is not None and ns <= self.last_match_ns:
            return False
        self.target = checked_transform(result["C_world_odom"])
        self.last_match_ns = ns
        return True

    def tick(self, source_ns):
        if type(source_ns) is not int or source_ns < 0:
            raise ValueError("invalid correction tick")
        if self.last_tick_ns is not None and source_ns <= self.last_tick_ns:
            raise ValueError("correction clock must increase")
        dt = 0 if self.last_tick_ns is None else min(0.2, (source_ns - self.last_tick_ns) / 1e9)
        self.last_tick_ns = source_ns
        stale = (
            self.last_match_ns is None
            or source_ns < self.last_match_ns
            or source_ns - self.last_match_ns > self.max_match_age_ns
        )
        if not stale and dt > 0:
            delta = self.target[:3, 3] - self.applied[:3, 3]
            step = delta * min(1.0, self.translation_rate * dt / max(np.linalg.norm(delta), 1e-12))
            rot = Rotation.from_matrix(self.target[:3, :3] @ self.applied[:3, :3].T).as_rotvec()
            rstep = rot * min(1.0, self.rotation_rate * dt / max(np.linalg.norm(rot), 1e-12))
            if np.linalg.norm(step) > 1e-10 or np.linalg.norm(rstep) > 1e-10:
                self.applied[:3, 3] += step
                self.applied[:3, :3] = (
                    Rotation.from_rotvec(rstep).as_matrix() @ self.applied[:3, :3]
                )
                self.generation += 1
        return dict(
            source_ns=source_ns,
            generation=self.generation,
            C_world_odom=self.applied.tolist(),
            match_source_ns=self.last_match_ns,
            correction_stale=stale,
            frontend_velocity_or_bias_repaired=False,
        )

    @staticmethod
    def apply(
        snapshot, odom_pose, registered_odom_xyz, velocity_odom=None, T_navigation_world=None
    ):
        c = checked_transform(snapshot["C_world_odom"])
        if T_navigation_world is not None:
            c = checked_transform(T_navigation_world) @ c
        pose = c @ checked_transform(odom_pose)
        cloud = np.asarray(registered_odom_xyz, dtype=float) @ c[:3, :3].T + c[:3, 3]
        velocity = None if velocity_odom is None else c[:3, :3] @ np.asarray(velocity_odom)
        return pose, cloud, velocity
