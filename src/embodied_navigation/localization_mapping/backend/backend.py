"""Point-cloud SLAM backend core: loop closure, prior-map relocalization, map->odom.

Frames: ``odom`` is the frontend odometry frame, ``body`` the estimated body
frame, ``world`` the map frame. The output is the correction
``C = T_world_odom`` so that ``T_world_body = C @ T_odom_body``.

Mapping mode (no prior map): world equals the first keyframe's odom frame and
C starts as identity. Every keyframe becomes a pose-graph node linked to its
predecessor by the odometry delta. ScanContext proposes loop candidates outside
the recent window, each with a yaw guess; coarse-then-fine ICP against the
candidate's world submap verifies them. The ICP body pose must then agree with
the graph's current estimate of the query within an odometry budget that grows
with the odom path ``d`` travelled since the candidate (rejects symmetric
false matches). An accepted loop adds a robust edge and re-optimises the
graph; ``C = T_world_body(latest) @ inv(T_odom_body(latest))`` is then offered
to ``CorrectionAdmission`` on every keyframe, always behind its jump gate.

Relocalization mode (prior map): ScanContext global search over the prior
keyframes, ICP against the prior submap, and two confirmations adopt C. After
that, every ``relocalize_every`` keyframes a local ICP starting from the
current C keeps tracking; ``lost_after`` consecutive failures (no ICP match or
a jump rejection) return to global search with a fresh admission, while the
last adopted C stays published. New keyframes are stored in a session map in
the prior map's world frame, which ``save_map`` writes.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, fields
from numbers import Real
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .correction import CorrectionAdmission
from .keyframes import Keyframe, voxel_downsample
from .map_store import MapStore
from .pose_graph import PoseGraph, invert, pose_difference, project_so3, rotation_angle
from .registration import IcpResult, _correspondences, _normals, icp_point_to_plane
from .scan_context import ScanContextConfig, ScanContextIndex, descriptor


@dataclass
class BackendConfig:
    scan_context: ScanContextConfig = field(default_factory=ScanContextConfig)
    map_voxel_m: float = 0.2  # stored keyframe clouds and submaps
    icp_voxel_m: float = 0.25
    coarse_correspondence_m: float = 2.5
    fine_correspondence_m: float = 0.6
    icp_iterations: int = 30
    min_fitness: float = 0.7
    max_rmse_m: float = 0.1
    submap_radius_m: float = 15.0
    loop_submap_neighbors: int = 5
    max_scan_context_distance: float = 0.6
    scan_context_prefilter: int = 10
    icp_candidates: int = 3
    exclude_recent: int = 10
    min_loop_gap_s: float = 20.0
    # Loop odometry-consistency budget at the body: base + per metre of odom path.
    loop_budget_translation_m: float = 0.5
    loop_budget_translation_per_m: float = 0.05
    loop_budget_rotation_rad: float = 0.2
    loop_budget_rotation_per_m: float = 0.01
    odometry_sigma_m: float = 0.2
    odometry_sigma_rad: float = 0.02
    loop_sigma_m: float = 0.05
    loop_sigma_rad: float = 0.01
    cauchy_scale: float = 10.0
    optimize_iterations: int = 20
    relocalize_every: int = 1
    lost_after: int = 3
    confirmations: int = 2
    window_s: float = 10.0
    agree_translation_m: float = 0.15
    agree_rotation_rad: float = 0.05
    # Jump gates. A loop closure legitimately moves C by the accumulated drift;
    # in relocalization a large move is only accepted via lost -> global search.
    loop_max_jump_translation_m: float = 3.0
    loop_max_jump_rotation_rad: float = 0.6
    max_jump_translation_m: float = 1.0
    max_jump_rotation_rad: float = 0.35
    # Optional per-axis sigmas (appended last to preserve positional compatibility).
    # None keeps the isotropic scalar above; axes follow PoseGraph's local tangent frame.
    odometry_sigma_z_m: float | None = None
    odometry_sigma_yaw_rad: float | None = None
    loop_sigma_z_m: float | None = None
    loop_sigma_yaw_rad: float | None = None
    # Phase 4 stabilization options (all disabled by default to preserve baseline behavior).
    min_loop_travel_m: float = 0.0
    min_loop_spacing_m: float = 0.0
    min_loop_interval_s: float = 0.0
    icp_gravity_sigma_rad: float | None = None
    graph_gravity_sigma_rad: float | None = None
    graph_gravity_robust: bool = True
    max_loop_tilt_rad: float | None = None
    zupt_enabled: bool = False
    zupt_max_odom_xy_m: float = 0.06
    zupt_max_median_dist_m: float = 0.08
    zupt_inlier_dist_m: float = 0.18
    zupt_min_fitness: float = 0.85
    zupt_max_measured_shift_m: float = 0.035
    zupt_full_stop_rot_rad: float = 0.026
    zupt_sigma_m: float = 0.015
    zupt_sigma_rad: float = 0.005
    zupt_skip_loop_search: bool = True

    def __post_init__(self) -> None:
        self.odometry_information()
        self.loop_information()

    def odometry_information(self) -> np.ndarray:
        """6x6 information of one odometry edge; isotropic unless overridden."""
        sigma_z = self.odometry_sigma_m if self.odometry_sigma_z_m is None else self.odometry_sigma_z_m
        sigma_yaw = self.odometry_sigma_rad if self.odometry_sigma_yaw_rad is None else self.odometry_sigma_yaw_rad
        return anisotropic_information(self.odometry_sigma_m, sigma_z, self.odometry_sigma_rad, sigma_yaw)

    def loop_information(self) -> np.ndarray:
        """6x6 information of one loop edge; isotropic unless overridden."""
        sigma_z = self.loop_sigma_m if self.loop_sigma_z_m is None else self.loop_sigma_z_m
        sigma_yaw = self.loop_sigma_rad if self.loop_sigma_yaw_rad is None else self.loop_sigma_yaw_rad
        return anisotropic_information(self.loop_sigma_m, sigma_z, self.loop_sigma_rad, sigma_yaw)

    @classmethod
    def from_dict(cls, data: dict) -> BackendConfig:
        """Strict config from a JSON-style mapping; unknown keys are rejected."""
        if not isinstance(data, dict):
            raise ValueError(f"BackendConfig must be a mapping, got {type(data).__name__}")
        known = {option.name for option in fields(cls)}
        unknown = sorted(set(data) - known, key=str)
        if unknown:
            raise ValueError(f"unknown BackendConfig keys: {unknown}")
        values = dict(data)
        if "scan_context" in values:
            values["scan_context"] = _scan_context_from_dict(values["scan_context"])
        config = cls(**values)
        config._validate()
        return config

    @classmethod
    def from_json(cls, path: Path) -> BackendConfig:
        """Read a JSON config file through the same strict validation."""
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def _validate(self) -> None:
        """Reject non-positive/non-finite sigmas, steps and thresholds from JSON."""
        for name in ("odometry_sigma_m", "odometry_sigma_rad", "loop_sigma_m", "loop_sigma_rad"):
            _positive(name, getattr(self, name))
        for name in ("odometry_sigma_z_m", "odometry_sigma_yaw_rad", "loop_sigma_z_m", "loop_sigma_yaw_rad"):
            value = getattr(self, name)
            if value is not None:
                _positive(name, value)
        self.odometry_information()
        self.loop_information()
        for name in (
            "map_voxel_m",
            "icp_voxel_m",
            "coarse_correspondence_m",
            "fine_correspondence_m",
            "max_rmse_m",
            "submap_radius_m",
            "max_scan_context_distance",
            "cauchy_scale",
            "window_s",
            "agree_translation_m",
            "agree_rotation_rad",
        ):
            _positive(name, getattr(self, name))
        if not 0.0 < _positive("min_fitness", self.min_fitness) <= 1.0:
            raise ValueError(f"min_fitness must lie in (0, 1], got {self.min_fitness!r}")
        for name in (
            "min_loop_gap_s",
            "loop_budget_translation_m",
            "loop_budget_translation_per_m",
            "loop_budget_rotation_rad",
            "loop_budget_rotation_per_m",
            "loop_max_jump_translation_m",
            "loop_max_jump_rotation_rad",
            "max_jump_translation_m",
            "max_jump_rotation_rad",
        ):
            _nonnegative(name, getattr(self, name))
        for name in ("loop_submap_neighbors", "exclude_recent"):
            _nonnegative_int(name, getattr(self, name))
        for name in (
            "icp_iterations",
            "scan_context_prefilter",
            "icp_candidates",
            "optimize_iterations",
            "relocalize_every",
            "lost_after",
            "confirmations",
        ):
            _count(name, getattr(self, name))
        for name in ("min_loop_travel_m", "min_loop_spacing_m", "min_loop_interval_s", "zupt_full_stop_rot_rad"):
            _nonnegative(name, getattr(self, name))
        for name in ("icp_gravity_sigma_rad", "graph_gravity_sigma_rad"):
            value = getattr(self, name)
            if value is not None:
                _inv_variance(name, value)
        if self.max_loop_tilt_rad is not None:
            _positive("max_loop_tilt_rad", self.max_loop_tilt_rad)
        for name in ("zupt_sigma_m", "zupt_sigma_rad"):
            _inv_variance(name, getattr(self, name))
        for name in ("zupt_max_odom_xy_m", "zupt_max_median_dist_m", "zupt_inlier_dist_m", "zupt_max_measured_shift_m"):
            _positive(name, getattr(self, name))
        if not 0.0 < _positive("zupt_min_fitness", self.zupt_min_fitness) <= 1.0:
            raise ValueError(f"zupt_min_fitness must lie in (0, 1], got {self.zupt_min_fitness!r}")
        for name in ("graph_gravity_robust", "zupt_enabled", "zupt_skip_loop_search"):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"{name} must be a bool, got {getattr(self, name)!r}")


def _scan_context_from_dict(data: dict) -> ScanContextConfig:
    """Strict nested ScanContextConfig; unknown keys and invalid bounds are rejected."""
    if not isinstance(data, dict):
        raise ValueError(f"scan_context must be a mapping, got {type(data).__name__}")
    unknown = sorted(set(data) - {option.name for option in fields(ScanContextConfig)}, key=str)
    if unknown:
        raise ValueError(f"unknown scan_context keys: {unknown}")
    config = ScanContextConfig(**data)
    _count("scan_context.rings", config.rings)
    _count("scan_context.sectors", config.sectors)
    _positive("scan_context.max_range_m", config.max_range_m)
    return config


def _positive(name: str, value) -> float:
    """Finite positive real; rejects bools and non-numeric input from JSON."""
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a finite positive number, got {value!r}")
    return float(value)


def _nonnegative(name: str, value) -> float:
    """Finite nonnegative real; rejects bools and non-numeric input from JSON."""
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be a finite nonnegative number, got {value!r}")
    return float(value)


def _count(name: str, value) -> int:
    """Positive integer count; rejects bools, floats and non-numeric input from JSON."""
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{name} must be a positive integer, got {value!r}")
    return value


def _nonnegative_int(name: str, value) -> int:
    """Nonnegative integer count; rejects bools, floats and non-numeric input from JSON."""
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer, got {value!r}")
    return value


def _inv_variance(name: str, value) -> float:
    sigma = _positive(name, value)
    try:
        weight = 1.0 / (sigma * sigma)
    except (OverflowError, ZeroDivisionError) as error:
        raise ValueError(f"{name}={value!r} cannot produce a finite positive weight") from error
    if not np.isfinite(weight) or weight <= 0.0:
        raise ValueError(f"{name}={value!r} produced non-finite or zero weight {weight!r}")
    return float(weight)


def anisotropic_information(
    sigma_xy_m: float,
    sigma_z_m: float,
    sigma_rp_rad: float,
    sigma_yaw_rad: float,
) -> np.ndarray:
    """Diagonal 6x6 information for (rho_x, rho_y, rho_z, phi_x, phi_y, phi_z).

    Horizontal translation shares ``sigma_xy_m`` and roll/pitch share
    ``sigma_rp_rad``; height and yaw carry their own sigma in PoseGraph's
    local edge tangent frame.
    """
    w_xy = _inv_variance("sigma_xy_m", sigma_xy_m)
    w_z = _inv_variance("sigma_z_m", sigma_z_m)
    w_rp = _inv_variance("sigma_rp_rad", sigma_rp_rad)
    w_yaw = _inv_variance("sigma_yaw_rad", sigma_yaw_rad)
    return np.diag([w_xy, w_xy, w_z, w_rp, w_rp, w_yaw])


def information(sigma_m: float, sigma_rad: float) -> np.ndarray:
    """Diagonal 6x6 information in the pose graph's (translation, rotation) order."""
    return anisotropic_information(sigma_m, sigma_m, sigma_rad, sigma_rad)


def yaw_pose(yaw: float) -> np.ndarray:
    T = np.eye(4)
    T[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    return T


class SlamBackend:
    def __init__(self, config: BackendConfig, *, prior_map: MapStore | None = None):
        self.config = config
        self.prior = prior_map
        self.map = MapStore(config.map_voxel_m)
        self.graph = PoseGraph(cauchy_scale=config.cauchy_scale)
        self.index = ScanContextIndex(config.scan_context)
        self.admission = self._new_admission()
        self.loop_count = 0
        self.last_error: str | None = None
        self._previous: Keyframe | None = None
        self._up_body: dict = {}
        self._zupt_anchor_kf: Keyframe | None = None
        self._zupt_recovery_anchor: Keyframe | None = None
        self._path_m: dict = {}  # key -> cumulative odom path length
        self._last_loop_path_m: float | None = None
        self._last_loop_stamp_ns: int | None = None
        self._stats = {
            "loops_skipped_spacing": 0,
            "loops_skipped_zupt": 0,
            "loops_rejected_tilt": 0,
            "zupt_full_stop_count": 0,
            "zupt_pivot_count": 0,
        }
        self._correction: np.ndarray | None = None
        self._tracking = False
        self._failures = 0
        self._since_check = 0
        if prior_map is not None:
            for key, desc in prior_map.descriptors.items():
                self.index.add(key, desc)

    @property
    def T_world_odom(self) -> np.ndarray | None:
        return self._correction

    def observability_stats(self) -> dict:
        return dict(self._stats)

    def save_map(self, directory: Path) -> None:
        self.map.save(directory)

    def ingest(self, keyframe: Keyframe) -> dict:
        """Process one keyframe; failures are reported in ``event["error"]``."""
        event = {"key": keyframe.key, "stamp_ns": keyframe.stamp_ns}
        try:
            if len(keyframe.points_lidar) == 0:
                raise ValueError(f"keyframe {keyframe.key} has no points")
            desc = descriptor(keyframe.points_body(), self.config.scan_context)
            if self.prior is None:
                event.update(self._map_step(keyframe, desc))
            else:
                event.update(self._relocalize_step(keyframe, desc))
        except Exception as error:  # one bad keyframe must not stop the backend
            self.last_error = f"{type(error).__name__}: {error}"
            event["error"] = self.last_error
        event["T_world_odom"] = None if self._correction is None else self._correction.tolist()
        return event

    # Mapping mode -------------------------------------------------------

    def _map_step(self, keyframe: Keyframe, desc: np.ndarray) -> dict:
        zupt_mode, _ = self._add_node(keyframe)
        loop_key, rejected, rejected_kind = self._find_loop(keyframe, desc, is_zupt=zupt_mode is not None)
        self.map.add(keyframe, self.graph.pose(keyframe.key), desc)
        self.index.add(keyframe.key, desc)
        if loop_key is not None:
            self.loop_count += 1
            self.graph.optimize(self.config.optimize_iterations)
            self.map.update_poses({key: self.graph.pose(key) for key in self.graph.keys})
        if self._correction is None:
            self._correction = np.eye(4)
        C_graph = self.graph.pose(keyframe.key) @ invert(keyframe.T_odom_body)
        if zupt_mode is not None and loop_key is None:
            status = self._propose_anchored(C_graph, keyframe)
        else:
            status = self._propose(C_graph, keyframe)
            if loop_key is not None and status == "pending":
                self.admission.seed_anchored(C_graph, keyframe.T_odom_body, keyframe.stamp_ns)
        event = {"loop": loop_key is not None, "loop_key": loop_key, "adopted": status == "adopted"}
        if zupt_mode is not None:
            event["zupt"] = zupt_mode
        if rejected is not None:
            event["loop_rejected"] = rejected
            event["loop_rejected_kind"] = rejected_kind
        return event

    @staticmethod
    def _estimate_zero_translation_shift(
        p_rot: np.ndarray,
        prev_pts: np.ndarray,
        tree: cKDTree,
        inlier_dist_m: float,
        *,
        init_shift: np.ndarray | None = None,
        iterations: int = 3,
    ) -> tuple[np.ndarray, float] | None:
        """Estimate signed 3D point-to-plane translation and residual RMS from ``p_rot`` to ``prev_pts``.

        Returns None when correspondences are insufficient or any translation
        axis is geometrically unobservable (e.g. vertical walls without floor/ceiling).
        """
        if len(prev_pts) < 50:
            return None
        normals = _normals(prev_pts, tree)
        shift = np.zeros(3, dtype=np.float64) if init_shift is None else np.array(init_shift, dtype=np.float64)
        res = np.empty(0, dtype=np.float64)
        for _ in range(iterations):
            moved = p_rot + shift
            _, res, n = _correspondences(moved, prev_pts, normals, tree, inlier_dist_m)
            if len(res) < 50:
                return None
            H = n.T @ n
            eig = np.linalg.eigvalsh(H)
            if eig[-1] <= 0.0 or eig[0] <= 0.02 * eig[-1]:
                return None
            step = np.linalg.solve(H, -(n.T @ res))
            shift += step
            if float(np.linalg.norm(step)) < 1e-4:
                break
        rms = float(np.sqrt(np.mean(res * res)))
        return shift, rms

    def _shift_is_zero(self, est: tuple[np.ndarray, float] | None) -> bool:
        """True iff estimated 3D translation is within both absolute cap and residual noise floor."""
        if est is None:
            return False
        shift, rms = est
        norm = float(np.linalg.norm(shift))
        return norm <= self.config.zupt_max_measured_shift_m and norm <= 2.5 * rms + 0.005

    def _detect_zupt(
        self, previous: Keyframe, keyframe: Keyframe, delta_odom: np.ndarray
    ) -> tuple[str | None, np.ndarray | None]:
        """Sensor-only geometric check for zero-translation (full stop or in-place pivot).

        Compares ``keyframe`` against the stationary streak's anchor keyframe
        (``self._zupt_anchor_kf`` when already stationary, else ``previous``) so
        slow continuous creep or slow rotation accumulates past the threshold
        and breaks ZUPT rather than being repeatedly zeroed out. Returns
        ``(mode, pivot_rotation)`` where ``pivot_rotation`` is the cloud-verified
        rotation from ``anchor`` to ``keyframe`` when ``mode == "pivot"``.
        """
        c = self.config
        if not c.zupt_enabled:
            return None, None
        anchor = self._zupt_anchor_kf if self._zupt_anchor_kf is not None else previous
        # Check horizontal motion in the gravity-aligned odom frame against BOTH
        # the previous frame and the streak anchor so body tilt never leaks Z drift into XY.
        step_dxy = float(
            np.hypot(
                keyframe.T_odom_body[0, 3] - previous.T_odom_body[0, 3],
                keyframe.T_odom_body[1, 3] - previous.T_odom_body[1, 3],
            )
        )
        anchor_dxy = float(
            np.hypot(
                keyframe.T_odom_body[0, 3] - anchor.T_odom_body[0, 3],
                keyframe.T_odom_body[1, 3] - anchor.T_odom_body[1, 3],
            )
        )
        if step_dxy > c.zupt_max_odom_xy_m or anchor_dxy > c.zupt_max_odom_xy_m:
            return None, None
        anchor_pts = voxel_downsample(anchor.points_body(), c.map_voxel_m)
        curr_pts = voxel_downsample(keyframe.points_body(), c.map_voxel_m)
        if len(anchor_pts) < 50 or len(curr_pts) < 50:
            return None, None
        delta_anchor = invert(anchor.T_odom_body) @ keyframe.T_odom_body
        R_odom_anchor = project_so3(delta_anchor[:3, :3])
        R_anchor_prev = project_so3((invert(self.graph.pose(anchor.key)) @ self.graph.pose(previous.key))[:3, :3])
        R_prev_step = project_so3(R_anchor_prev @ delta_odom[:3, :3])
        tree = cKDTree(anchor_pts)
        p_rot = curr_pts @ R_prev_step.T
        d_zero, _ = tree.query(p_rot)
        R_pivot = R_prev_step
        if rotation_angle(R_prev_step, R_odom_anchor) > 1e-4:
            p_odom = curr_pts @ R_odom_anchor.T
            d_odom, _ = tree.query(p_odom)
            if float(np.mean(d_odom)) + 1e-4 < float(np.mean(d_zero)):
                p_rot, d_zero, R_pivot = p_odom, d_odom, R_odom_anchor
        rot_anchor = rotation_angle(np.eye(3), R_pivot)
        # Check unrotated stationary full_stop against the streak anchor (and reject
        # bogus yaw drift when unrotated curr_pts aligns strictly better than p_rot).
        if c.zupt_full_stop_rot_rad > 0.0:
            d_id_anchor, _ = tree.query(curr_pts)
            id_fit = float(np.mean(d_id_anchor <= c.zupt_inlier_dist_m))
            id_med = float(np.median(d_id_anchor))
            id_better_than_rot = float(np.mean(d_id_anchor)) + 0.002 < float(np.mean(d_zero))
            id_as_good_small_rot = (
                rot_anchor <= c.zupt_full_stop_rot_rad
                and float(np.mean(d_id_anchor)) <= float(np.mean(d_zero)) + 0.002
                and float(np.percentile(d_id_anchor, 90)) <= float(np.percentile(d_zero, 90)) + 0.005
            )
            if id_fit >= c.zupt_min_fitness and id_med <= c.zupt_max_median_dist_m and (id_better_than_rot or id_as_good_small_rot):
                est_id = self._estimate_zero_translation_shift(curr_pts, anchor_pts, tree, c.zupt_inlier_dist_m)
                if self._shift_is_zero(est_id):
                    return "full_stop", None

        med_zero = float(np.median(d_zero))
        fit_zero = float(np.mean(d_zero <= c.zupt_inlier_dist_m))
        if fit_zero < c.zupt_min_fitness or med_zero > c.zupt_max_median_dist_m:
            return None, None
        est_rot = self._estimate_zero_translation_shift(p_rot, anchor_pts, tree, c.zupt_inlier_dist_m)
        if not self._shift_is_zero(est_rot):
            return None, None
        return "pivot", R_pivot

    def _estimate_anchor_exit_delta(
        self, anchor: Keyframe, previous: Keyframe, keyframe: Keyframe, delta_step: np.ndarray
    ) -> np.ndarray | None:
        """Estimate cloud-supported translation from ``anchor`` to ``keyframe`` when exiting ZUPT.

        Uses the graph's already-corrected transform ``T_anchor_prev`` composed with
        the step delta so eliminated stationary yaw drift is never re-introduced
        and ``init_shift`` is expressed in ``anchor``'s frame (including any
        intermediate sparse frames before recovery succeeded).
        """
        c = self.config
        anchor_pts = voxel_downsample(anchor.points_body(), c.map_voxel_m)
        curr_pts = voxel_downsample(keyframe.points_body(), c.map_voxel_m)
        if len(anchor_pts) < 50 or len(curr_pts) < 50:
            return None
        T_anchor_prev = invert(self.graph.pose(anchor.key)) @ self.graph.pose(previous.key)
        R_anchor_prev = project_so3(T_anchor_prev[:3, :3])
        R_anchor = project_so3(R_anchor_prev @ delta_step[:3, :3])
        init_shift = T_anchor_prev[:3, 3] + R_anchor_prev @ delta_step[:3, 3]
        if float(np.linalg.norm(init_shift)) > 2.0:
            return None
        p_rot = curr_pts @ R_anchor.T
        tree = cKDTree(anchor_pts)
        search_r = max(c.zupt_inlier_dist_m, 0.25)
        est = self._estimate_zero_translation_shift(
            p_rot, anchor_pts, tree, search_r, init_shift=init_shift, iterations=5
        )
        if est is None:
            return None
        shift, _ = est
        if float(np.linalg.norm(shift - init_shift)) > 0.5:
            return None
        dists, _ = tree.query(p_rot + shift)
        if float(np.mean(dists <= c.zupt_inlier_dist_m)) < c.zupt_min_fitness:
            return None
        if float(np.median(dists)) > c.zupt_max_median_dist_m:
            return None
        T_measured = np.eye(4, dtype=np.float64)
        T_measured[:3, :3] = R_anchor
        T_measured[:3, 3] = shift
        return T_measured

    def _add_node(self, keyframe: Keyframe) -> tuple[str | None, np.ndarray]:
        """Node at the previous optimised pose composed with the odometry (or ZUPT) delta."""
        c = self.config
        previous = self._previous
        zupt_mode: str | None = None
        delta_use = np.eye(4, dtype=np.float64)
        if previous is None:
            self.graph.add_node(keyframe.key, keyframe.T_odom_body)
            self._path_m[keyframe.key] = 0.0
            up_body = keyframe.T_odom_body[2, :3].copy()
        else:
            delta = invert(previous.T_odom_body) @ keyframe.T_odom_body
            zupt_mode, pivot_rot = self._detect_zupt(previous, keyframe, delta)
            parent_key = previous.key
            exited_zupt = False
            if zupt_mode == "full_stop":
                self._stats["zupt_full_stop_count"] += 1
                if self._zupt_anchor_kf is not None:
                    parent_key = self._zupt_anchor_kf.key
                delta_use = np.eye(4, dtype=np.float64)
                odometry = anisotropic_information(c.zupt_sigma_m, c.zupt_sigma_m, c.zupt_sigma_rad, c.zupt_sigma_rad)
                step_dist = 0.0
            elif zupt_mode == "pivot":
                self._stats["zupt_pivot_count"] += 1
                delta_use = np.eye(4, dtype=np.float64)
                if self._zupt_anchor_kf is not None:
                    parent_key = self._zupt_anchor_kf.key
                delta_use[:3, :3] = pivot_rot if pivot_rot is not None else delta[:3, :3]
                sigma_yaw = c.odometry_sigma_rad if c.odometry_sigma_yaw_rad is None else c.odometry_sigma_yaw_rad
                odometry = anisotropic_information(c.zupt_sigma_m, c.zupt_sigma_m, c.odometry_sigma_rad, sigma_yaw)
                step_dist = 0.0
            else:
                delta_use = delta
                odometry = c.odometry_information()
                step_dist = float(np.linalg.norm(delta[:3, 3]))
                exited_zupt = self._zupt_anchor_kf is not None
                # Exiting a ZUPT streak (or retrying after a sparse exit frame): recover
                # point-cloud-supported motion since the streak anchor so genuine slow
                # motion after a drifted stop is not lost.
                recovery_anchor = self._zupt_anchor_kf if self._zupt_anchor_kf is not None else self._zupt_recovery_anchor
                if recovery_anchor is not None:
                    T_anchor_prev = invert(self.graph.pose(recovery_anchor.key)) @ self.graph.pose(previous.key)
                    est_shift = T_anchor_prev[:3, 3] + T_anchor_prev[:3, :3] @ delta[:3, 3]
                    if float(np.linalg.norm(est_shift)) > 2.0:
                        self._zupt_recovery_anchor = None
                    else:
                        measured_exit = self._estimate_anchor_exit_delta(recovery_anchor, previous, keyframe, delta)
                        if measured_exit is not None:
                            parent_key = recovery_anchor.key
                            delta_use = measured_exit
                            step_dist = float(np.linalg.norm(measured_exit[:3, 3]))
                            self._zupt_recovery_anchor = None
                        else:
                            self._zupt_recovery_anchor = recovery_anchor
            if zupt_mode is not None:
                self._zupt_recovery_anchor = None
                if self._zupt_anchor_kf is None:
                    self._zupt_anchor_kf = previous
            else:
                self._zupt_anchor_kf = None
            new_pose = self.graph.pose(parent_key) @ delta_use
            self.graph.add_node(keyframe.key, new_pose)
            self.graph.add_edge(parent_key, keyframe.key, delta_use, odometry)
            if zupt_mode is not None:
                path_step = 0.0
            elif parent_key == previous.key or exited_zupt:
                path_step = step_dist
            else:
                path_step = min(
                    float(np.linalg.norm(delta[:3, 3])),
                    float(np.linalg.norm(new_pose[:3, 3] - self.graph.pose(previous.key)[:3, 3])),
                )
            self._path_m[keyframe.key] = self._path_m[previous.key] + path_step
            up_body = project_so3(delta_use[:3, :3]).T @ self._up_body[parent_key]
            up_norm = float(np.linalg.norm(up_body))
            if up_norm > 1e-9:
                up_body = up_body / up_norm
        if c.graph_gravity_sigma_rad is not None:
            self.graph.add_gravity_prior(
                keyframe.key,
                up_body,
                c.graph_gravity_sigma_rad,
                robust=c.graph_gravity_robust,
            )
        self._previous = keyframe
        self._up_body[keyframe.key] = up_body
        return zupt_mode, delta_use

    def _odometry_inconsistency(self, key, keyframe: Keyframe, T_world_body: np.ndarray) -> str | None:
        """Reason why a loop match disagrees with the graph estimate, or None.

        Compares the ICP body pose with the graph's current pose of the query
        at the body; the budget grows with the odom path ``d`` since ``key``.
        """
        c = self.config
        d = self._path_m[keyframe.key] - self._path_m[key]
        translation, rotation = pose_difference(self.graph.pose(keyframe.key), T_world_body)
        max_translation = c.loop_budget_translation_m + c.loop_budget_translation_per_m * d
        max_rotation = c.loop_budget_rotation_rad + c.loop_budget_rotation_per_m * d
        if translation <= max_translation and rotation <= max_rotation:
            return None
        return (
            f"odometry inconsistent with {key}: {translation:.2f} m / {rotation:.2f} rad "
            f"exceeds {max_translation:.2f} m / {max_rotation:.2f} rad over {d:.1f} m"
        )

    def _find_loop(self, keyframe: Keyframe, desc: np.ndarray, *, is_zupt: bool = False):
        """Add one verified loop edge to the newest keyframe.

        Returns ``(partner key, None, None)`` for an added loop,
        ``(None, reason, kind)`` when a geometric match failed the tilt or
        odometry gate (``kind`` is ``"tilt"`` or ``"odom"``), else ``(None, None, None)``.
        """
        c = self.config
        if is_zupt and c.zupt_skip_loop_search:
            self._stats["loops_skipped_zupt"] += 1
            return None, None, None
        curr_path = self._path_m[keyframe.key]
        if self._last_loop_path_m is not None:
            if c.min_loop_spacing_m > 0.0 and (curr_path - self._last_loop_path_m) < c.min_loop_spacing_m:
                self._stats["loops_skipped_spacing"] += 1
                return None, None, None
            if (
                c.min_loop_interval_s > 0.0
                and self._last_loop_stamp_ns is not None
                and (keyframe.stamp_ns - self._last_loop_stamp_ns) < c.min_loop_interval_s * 1e9
            ):
                self._stats["loops_skipped_spacing"] += 1
                return None, None, None

        recent = set(list(self.map.keyframes)[-c.exclude_recent :])
        gap_ns = c.min_loop_gap_s * 1e9
        excluded = {
            key
            for key, old in self.map.keyframes.items()
            if key in recent
            or keyframe.stamp_ns - old.stamp_ns < gap_ns
            or (c.min_loop_travel_m > 0.0 and (curr_path - self._path_m.get(key, curr_path)) < c.min_loop_travel_m)
        }
        match, tilt_rejected = self._search_with_rejection(
            self.map,
            keyframe,
            desc,
            lambda key: key in excluded,
            lambda key: self._loop_submap(key, excluded),
            apply_tilt_gate=True,
            use_gravity_prior=True,
        )
        if match is None:
            return None, tilt_rejected, ("tilt" if tilt_rejected is not None else None)
        key, result = match
        T_world_new = result.T_target_source
        rejected = self._odometry_inconsistency(key, keyframe, T_world_new)
        if rejected is not None:
            return None, rejected, "odom"
        anchor = min(
            self._loop_window(key, excluded),
            key=lambda k: np.linalg.norm(self.map.poses[k][:3, 3] - T_world_new[:3, 3]),
        )
        loop = c.loop_information()
        self.graph.add_edge(anchor, keyframe.key, invert(self.graph.pose(anchor)) @ T_world_new, loop, robust=True)
        self._last_loop_path_m = curr_path
        self._last_loop_stamp_ns = keyframe.stamp_ns
        return anchor, None, None

    def _loop_window(self, key, excluded: set) -> list:
        """The candidate's temporal neighbours that are not excluded.

        Keyframes from other passes carry the drift the loop is about to
        correct, so they would blur the target; the recent window is excluded
        for the same reason.
        """
        keys = list(self.map.keyframes)
        i = keys.index(key)
        n = self.config.loop_submap_neighbors
        return [k for k in keys[max(0, i - n) : i + n + 1] if k not in excluded]

    def _loop_submap(self, key, excluded: set) -> np.ndarray:
        """World cloud of the candidate's ``_loop_window`` (the ICP target of a loop check)."""
        window = set(self._loop_window(key, excluded))
        others = [k for k in self.map.keyframes if k not in window]
        return self.map.submap(self.map.poses[key][:3, 3], self.config.submap_radius_m, exclude_keys=others)

    # Relocalization mode -------------------------------------------------

    def _relocalize_step(self, keyframe: Keyframe, desc: np.ndarray) -> dict:
        search = None
        T_world_body = None
        if not self._tracking:
            search = "global"
            match = self._search(self.prior, keyframe, desc, None, self._prior_submap)
            T_world_body = None if match is None else match[1].T_target_source
        else:
            self._since_check += 1
            if self._since_check >= self.config.relocalize_every:
                self._since_check = 0
                search = "local"
                T_world_body = self._track(keyframe)
        status = None
        if T_world_body is not None:
            status = self._propose(T_world_body @ invert(keyframe.T_odom_body), keyframe)
        if search == "local":
            self._count_failure(status in (None, "rejected"))
        if status == "adopted":
            self._tracking, self._failures, self._since_check = True, 0, 0
        if self._correction is not None:
            self.map.add(keyframe, self._correction @ keyframe.T_odom_body, desc)
        return {
            "search": search,
            "relocalized": status in ("adopted", "pending"),
            "adopted": status == "adopted",
            "lost": not self._tracking,
        }

    def _track(self, keyframe: Keyframe) -> np.ndarray | None:
        T_init = self._correction @ keyframe.T_odom_body
        target = self.prior.submap(T_init[:3, 3], self.config.submap_radius_m)
        result = self._register(keyframe.points_body(), target, T_init)
        return None if result is None else result.T_target_source

    def _prior_submap(self, key) -> np.ndarray:
        return self.prior.submap(self.prior.poses[key][:3, 3], self.config.submap_radius_m)

    def _count_failure(self, failed: bool) -> None:
        """Relocalization only: after ``lost_after`` local failures in a row, go lost.

        Global search resumes with a fresh admission (no jump reference) while
        the last adopted C stays published. Mapping mode never resets its
        admission, so every graph correction stays behind the jump gate.
        """
        self._failures = self._failures + 1 if failed else 0
        if self._failures >= self.config.lost_after:
            self._tracking, self._failures = False, 0
            self.admission = self._new_admission()

    # Shared ---------------------------------------------------------------

    def _tilt_rejection_reason(self, keyframe: Keyframe, T_world_body: np.ndarray) -> str | None:
        limit = self.config.max_loop_tilt_rad
        if limit is None:
            return None
        up_body = keyframe.T_odom_body[2, :3]
        up_norm = float(np.linalg.norm(up_body))
        if up_norm <= 1e-6:
            return None
        u_world = T_world_body[:3, :3] @ (up_body / up_norm)
        tilt_rad = float(np.arctan2(np.hypot(u_world[0], u_world[1]), u_world[2]))
        if tilt_rad > limit:
            return f"gravity tilt {np.degrees(tilt_rad):.2f} deg exceeds {np.degrees(limit):.2f} deg"
        return None

    def _search_with_rejection(
        self,
        store: MapStore,
        keyframe: Keyframe,
        desc,
        exclude,
        submap,
        *,
        apply_tilt_gate: bool = False,
        use_gravity_prior: bool = False,
    ):
        """Best ICP-verified ScanContext candidate as ``((key, IcpResult) | None, reject_reason)``."""
        candidates = self.index.query(desc, exclude=exclude, candidates=self.config.scan_context_prefilter)
        best = None
        last_tilt_reject = None
        up_body = (
            keyframe.T_odom_body[2, :3]
            if (use_gravity_prior and self.config.icp_gravity_sigma_rad is not None)
            else None
        )
        for key, dist, yaw in candidates[: self.config.icp_candidates]:
            if dist > self.config.max_scan_context_distance:
                break
            T_init = store.poses[key] @ yaw_pose(yaw)
            result = self._register(keyframe.points_body(), submap(key), T_init, source_up_body=up_body)
            if result is None:
                continue
            if apply_tilt_gate:
                tilt_reject = self._tilt_rejection_reason(keyframe, result.T_target_source)
                if tilt_reject is not None:
                    self._stats["loops_rejected_tilt"] += 1
                    last_tilt_reject = tilt_reject
                    continue
            if best is None or result.rmse_m < best[1].rmse_m:
                best = (key, result)
        return best, (None if best is not None else last_tilt_reject)

    def _search(self, store: MapStore, keyframe: Keyframe, desc, exclude, submap):
        """Best ICP-verified ScanContext candidate as ``(key, IcpResult)``, or None.

        ``submap(key)`` builds the world-frame ICP target around a candidate;
        the initial guess is the candidate's pose rotated by the ScanContext
        yaw. Among verified candidates the lowest RMSE wins.
        """
        best, _ = self._search_with_rejection(store, keyframe, desc, exclude, submap)
        return best

    def _register(
        self, source_body, target_world, T_init, *, source_up_body: np.ndarray | None = None
    ) -> IcpResult | None:
        """Coarse-then-fine ICP; the fine result must pass fitness and RMSE gates."""
        c = self.config
        coarse = icp_point_to_plane(
            source_body,
            target_world,
            T_init,
            max_correspondence_m=c.coarse_correspondence_m,
            iterations=c.icp_iterations,
            voxel_m=2 * c.icp_voxel_m,
            source_up_body=source_up_body,
            gravity_sigma_rad=c.icp_gravity_sigma_rad,
        )
        fine = icp_point_to_plane(
            source_body,
            target_world,
            coarse.T_target_source,
            max_correspondence_m=c.fine_correspondence_m,
            iterations=c.icp_iterations,
            voxel_m=c.icp_voxel_m,
            source_up_body=source_up_body,
            gravity_sigma_rad=c.icp_gravity_sigma_rad,
        )
        if fine.converged and fine.fitness >= c.min_fitness and fine.rmse_m <= c.max_rmse_m:
            return fine
        return None

    def _propose(self, C: np.ndarray, keyframe: Keyframe) -> str:
        """Offer C to the admission: ``"adopted"``, ``"rejected"`` (jump) or ``"pending"``."""
        rejections = len(self.admission.rejections)
        ref = self._correction if (self.prior is None and self.config.zupt_enabled) else None
        adopted = self.admission.propose(C, keyframe.T_odom_body, keyframe.stamp_ns, reference_C=ref)
        if adopted is not None:
            self._correction = adopted
            return "adopted"
        return "rejected" if len(self.admission.rejections) > rejections else "pending"

    def _propose_anchored(self, C: np.ndarray, keyframe: Keyframe) -> str:
        """Offer a ZUPT-anchored C through CorrectionAdmission's window and jump gate."""
        rejections = len(self.admission.rejections)
        adopted = self.admission.propose_anchored(
            C, keyframe.T_odom_body, keyframe.stamp_ns, reference_C=self._correction
        )
        if adopted is not None:
            self._correction = adopted
            return "adopted"
        return "rejected" if len(self.admission.rejections) > rejections else "pending"

    def _new_admission(self) -> CorrectionAdmission:
        c = self.config
        mapping = self.prior is None
        return CorrectionAdmission(
            confirmations=c.confirmations,
            window_s=c.window_s,
            agree_translation_m=c.agree_translation_m,
            agree_rotation_rad=c.agree_rotation_rad,
            max_jump_translation_m=c.loop_max_jump_translation_m if mapping else c.max_jump_translation_m,
            max_jump_rotation_rad=c.loop_max_jump_rotation_rad if mapping else c.max_jump_rotation_rad,
        )
