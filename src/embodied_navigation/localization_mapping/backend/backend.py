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

from ...common.geometry import wrap_angle
from .color_scan_context import ColorScanContextIndex
from .correction import CorrectionAdmission
from .floor_plane import FloorPlaneMeasurement, detect_floor_plane, project_coplanar_odom_delta
from .keyframes import Keyframe, transform_points, voxel_downsample
from .map_store import MapStore
from .planar_ba import PlanarBaResult, refine_poses_oriented_planar_ba, refine_window_oriented_planar_ba
from .pose_graph import PoseGraph, invert, pose_difference, project_so3, rotation_angle
from .registration import IcpResult, _correspondences, _normals, icp_point_to_plane
from .scan_context import ScanContextConfig, ScanContextIndex, descriptor
from .submaps import SubmapConfig, build_submap
from .surfaces import SurfaceConfig, compare_surfaces, model_surfaces
from .visual import verify_visual
from .visual_retrieval import VisualIndex, VisualRetrievalConfig


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
    # Wide-span coplanar floor leveling, LiDAR floor-normal gravity, fine-voxel ICP, and saturated X-ICP.
    floor_plane_enabled: bool = False
    floor_plane_max_tilt_deg: float = 14.0
    floor_plane_max_step_dz_m: float = 0.06
    floor_normal_gravity_enabled: bool = False
    floor_normal_graph_sigma_rad: float | None = None
    floor_gravity_world_calibration: bool = False
    gravity_optimize_every: int = 0
    fine_icp_voxel_m: float | None = None
    xicp_enabled: bool = False
    xicp_min_scale: float = 0.15
    xicp_cond_floor: float = 1e-3
    # Robust moving odometry edges and pure-geometric scan-to-scan ICP odometry repair.
    odometry_robust: bool = False
    odometry_icp_repair_enabled: bool = False
    odometry_icp_repair_max_median_m: float = 0.20
    odometry_icp_repair_min_fitness: float = 0.55
    odometry_icp_repair_max_rmse_m: float = 0.12
    # Bounded measured multi-frame submaps, surface coverage gate, and translation degeneracy gate.
    use_measured_submaps: bool = False
    measured_submap: SubmapConfig = field(default_factory=SubmapConfig)
    use_surface_verification: bool = False
    gravity_aligned: bool = False
    surface: SurfaceConfig = field(default_factory=SurfaceConfig)
    min_translation_observability: float | None = None
    # Visual retrieval, Color-ScanContext, and heading-gated hybrid admission.
    use_visual_retrieval: bool = False
    use_visual_verification: bool = False
    visual_agree_translation_m: float = 0.3
    visual_agree_rotation_rad: float = 0.05
    visual_retrieval: VisualRetrievalConfig = field(default_factory=VisualRetrievalConfig)
    color_sc_enabled: bool = False
    color_sc_weight: float = 0.25
    color_sc_min_bins: int = 12
    color_sc_max_distance: float | None = None
    visual_multiview_radius: int = 0
    visual_seq_consistency: int = 1
    hybrid_heading_gate_deg: float | None = None
    reverse_icp_fitness_min: float | None = None
    reverse_icp_rmse_max: float | None = None
    # Oriented-normal 3D planar bundle adjustment (BALM2 point-cluster Schur complement).
    planar_ba_enabled: bool = False
    planar_ba_voxel_m: float = 1.0
    planar_ba_min_frame_pts: int = 8
    planar_ba_plane_ratio_max: float = 0.08
    planar_ba_plane_thickness_max_m: float = 0.08
    planar_ba_min_normal_cos: float = 0.85
    planar_ba_max_offset_m: float = 0.15
    planar_ba_max_frames_per_cluster: int = 20
    planar_ba_sigma_m: float = 0.03
    planar_ba_outer_rounds: int = 2
    planar_ba_iterations: int = 8
    planar_ba_window_size: int = 0
    planar_ba_every: int = 0
    planar_ba_global_every: int = 0
    planar_ba_online_iterations: int = 6

    def __post_init__(self) -> None:
        self.odometry_information()
        self.loop_information()
        if not isinstance(self.measured_submap, SubmapConfig):
            raise ValueError(
                f"measured_submap must be a SubmapConfig, got {type(self.measured_submap).__name__}"
            )
        if not isinstance(self.surface, SurfaceConfig):
            raise ValueError(f"surface must be a SurfaceConfig, got {type(self.surface).__name__}")
        if not isinstance(self.visual_retrieval, VisualRetrievalConfig):
            raise ValueError(
                f"visual_retrieval must be a VisualRetrievalConfig, got {type(self.visual_retrieval).__name__}"
            )
        if self.use_surface_verification and not self.gravity_aligned:
            raise ValueError("use_surface_verification requires gravity_aligned=True")
        if not self.use_visual_retrieval:
            if self.hybrid_heading_gate_deg is not None:
                raise ValueError("hybrid_heading_gate_deg requires use_visual_retrieval=True")
            if self.reverse_icp_fitness_min is not None or self.reverse_icp_rmse_max is not None:
                raise ValueError("reverse_icp_fitness_min/reverse_icp_rmse_max require use_visual_retrieval=True")

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
        if "measured_submap" in values:
            values["measured_submap"] = _nested_config_from_dict("measured_submap", SubmapConfig, values["measured_submap"])
        if "surface" in values:
            values["surface"] = _nested_config_from_dict("surface", SurfaceConfig, values["surface"])
        if "visual_retrieval" in values:
            values["visual_retrieval"] = _nested_config_from_dict(
                "visual_retrieval", VisualRetrievalConfig, values["visual_retrieval"]
            )
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
            "visual_agree_translation_m",
            "visual_agree_rotation_rad",
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
            "color_sc_weight",
        ):
            _nonnegative(name, getattr(self, name))
        for name in ("loop_submap_neighbors", "exclude_recent", "visual_multiview_radius", "gravity_optimize_every"):
            _nonnegative_int(name, getattr(self, name))
        for name in (
            "icp_iterations",
            "scan_context_prefilter",
            "icp_candidates",
            "optimize_iterations",
            "relocalize_every",
            "lost_after",
            "confirmations",
            "color_sc_min_bins",
            "visual_seq_consistency",
        ):
            _count(name, getattr(self, name))
        for name in ("min_loop_travel_m", "min_loop_spacing_m", "min_loop_interval_s", "zupt_full_stop_rot_rad"):
            _nonnegative(name, getattr(self, name))
        for name in ("icp_gravity_sigma_rad", "graph_gravity_sigma_rad", "floor_normal_graph_sigma_rad"):
            value = getattr(self, name)
            if value is not None:
                _inv_variance(name, value)
        for name in ("max_loop_tilt_rad", "fine_icp_voxel_m", "reverse_icp_rmse_max"):
            value = getattr(self, name)
            if value is not None:
                _positive(name, value)
        if self.color_sc_max_distance is not None:
            if _nonnegative("color_sc_max_distance", self.color_sc_max_distance) > 1.0:
                raise ValueError(
                    f"color_sc_max_distance must lie in [0, 1], got {self.color_sc_max_distance!r}"
                )
        if self.hybrid_heading_gate_deg is not None:
            gate = _positive("hybrid_heading_gate_deg", self.hybrid_heading_gate_deg)
            if gate > 180.0:
                raise ValueError(
                    f"hybrid_heading_gate_deg must lie in (0, 180], got {self.hybrid_heading_gate_deg!r}"
                )
        if self.reverse_icp_fitness_min is not None:
            fit = _positive("reverse_icp_fitness_min", self.reverse_icp_fitness_min)
            if fit > 1.0:
                raise ValueError(
                    f"reverse_icp_fitness_min must lie in (0, 1], got {self.reverse_icp_fitness_min!r}"
                )
        if not isinstance(getattr(self, "use_visual_retrieval", False), bool):
            raise ValueError(f"use_visual_retrieval must be a bool, got {self.use_visual_retrieval!r}")
        if not self.use_visual_retrieval:
            if self.hybrid_heading_gate_deg is not None:
                raise ValueError("hybrid_heading_gate_deg requires use_visual_retrieval=True")
            if self.reverse_icp_fitness_min is not None or self.reverse_icp_rmse_max is not None:
                raise ValueError("reverse_icp_fitness_min/reverse_icp_rmse_max require use_visual_retrieval=True")
        if self.min_translation_observability is not None:
            obs = _positive("min_translation_observability", self.min_translation_observability)
            if obs > 1.0:
                raise ValueError(
                    f"min_translation_observability must lie in (0, 1], got {self.min_translation_observability!r}"
                )
        for name in ("zupt_sigma_m", "zupt_sigma_rad"):
            _inv_variance(name, getattr(self, name))
        for name in (
            "zupt_max_odom_xy_m",
            "zupt_max_median_dist_m",
            "zupt_inlier_dist_m",
            "zupt_max_measured_shift_m",
            "floor_plane_max_step_dz_m",
            "xicp_cond_floor",
            "odometry_icp_repair_max_median_m",
            "odometry_icp_repair_max_rmse_m",
            "planar_ba_voxel_m",
            "planar_ba_plane_ratio_max",
            "planar_ba_plane_thickness_max_m",
            "planar_ba_max_offset_m",
            "planar_ba_sigma_m",
        ):
            _positive(name, getattr(self, name))
        if (
            isinstance(self.planar_ba_min_normal_cos, bool)
            or not isinstance(self.planar_ba_min_normal_cos, Real)
            or not np.isfinite(self.planar_ba_min_normal_cos)
            or not (-1.0 < float(self.planar_ba_min_normal_cos) <= 1.0)
        ):
            raise ValueError(
                f"planar_ba_min_normal_cos must lie in (-1, 1], got {self.planar_ba_min_normal_cos!r}"
            )
        if _count("planar_ba_min_frame_pts", self.planar_ba_min_frame_pts) < 3:
            raise ValueError(f"planar_ba_min_frame_pts must be >= 3, got {self.planar_ba_min_frame_pts!r}")
        if _count("planar_ba_max_frames_per_cluster", self.planar_ba_max_frames_per_cluster) < 2:
            raise ValueError(
                f"planar_ba_max_frames_per_cluster must be >= 2, got {self.planar_ba_max_frames_per_cluster!r}"
            )
        for name in ("planar_ba_outer_rounds", "planar_ba_iterations", "planar_ba_online_iterations"):
            _count(name, getattr(self, name))
        for name in ("planar_ba_window_size", "planar_ba_every", "planar_ba_global_every"):
            _nonnegative_int(name, getattr(self, name))
        if self.planar_ba_window_size == 1:
            raise ValueError("planar_ba_window_size must be 0 (disabled) or >= 2, got 1")
        if not 0.0 < _positive("odometry_icp_repair_min_fitness", self.odometry_icp_repair_min_fitness) <= 1.0:
            raise ValueError(
                f"odometry_icp_repair_min_fitness must lie in (0, 1], got {self.odometry_icp_repair_min_fitness!r}"
            )
        if not 0.0 < _positive("zupt_min_fitness", self.zupt_min_fitness) <= 1.0:
            raise ValueError(f"zupt_min_fitness must lie in (0, 1], got {self.zupt_min_fitness!r}")
        if not 0.0 < _positive("floor_plane_max_tilt_deg", self.floor_plane_max_tilt_deg) < 90.0:
            raise ValueError(
                f"floor_plane_max_tilt_deg must lie in (0, 90), got {self.floor_plane_max_tilt_deg!r}"
            )
        if not 0.0 < _positive("xicp_min_scale", self.xicp_min_scale) <= 1.0:
            raise ValueError(f"xicp_min_scale must lie in (0, 1], got {self.xicp_min_scale!r}")
        for name in (
            "graph_gravity_robust",
            "zupt_enabled",
            "zupt_skip_loop_search",
            "floor_plane_enabled",
            "floor_normal_gravity_enabled",
            "floor_gravity_world_calibration",
            "xicp_enabled",
            "odometry_robust",
            "odometry_icp_repair_enabled",
            "use_measured_submaps",
            "use_surface_verification",
            "gravity_aligned",
            "use_visual_retrieval",
            "use_visual_verification",
            "color_sc_enabled",
            "planar_ba_enabled",
        ):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"{name} must be a bool, got {getattr(self, name)!r}")


def _nested_config_from_dict(field_name: str, cls, data: dict):
    """Strict nested dataclass config loader; unknown keys are rejected."""
    if not isinstance(data, dict):
        raise ValueError(f"{field_name} must be a mapping, got {type(data).__name__}")
    unknown = sorted(set(data) - {option.name for option in fields(cls)}, key=str)
    if unknown:
        raise ValueError(f"unknown {field_name} keys: {unknown}")
    return cls(**data)


def _scan_context_from_dict(data: dict) -> ScanContextConfig:
    """Strict nested ScanContextConfig; unknown keys and invalid bounds are rejected."""
    config = _nested_config_from_dict("scan_context", ScanContextConfig, data)
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


def _block_sqrt_confidence(
    block_world: np.ndarray,
    R_world_body: np.ndarray,
    min_scale: float,
    cond_floor: float,
    saturation_ref: float = 50.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Square-root directional confidence 3x3 matrix in body frame from a world Hessian block.

    Axes with Fisher information >= ``saturation_ref`` are well-constrained and retain
    full weight ``1.0`` (preventing dominant floor/ceiling Z counts from falsely attenuating
    well-observed indoor wall XY axes). Only genuinely degenerate axes below ``saturation_ref``
    are attenuated down to ``min_scale``.
    """
    block_body = R_world_body.T @ block_world @ R_world_body
    block_body = 0.5 * (block_body + block_body.T)
    lam, vecs = np.linalg.eigh(block_body)
    lam = np.maximum(lam, 0.0)
    lam_max = float(lam[-1])
    if lam_max <= cond_floor:
        scales = np.full(3, min_scale, dtype=np.float64)
    else:
        denom = min(lam_max, saturation_ref)
        scales = np.clip(lam / max(denom, 1e-12), min_scale, 1.0)
    sqrt_block = (vecs * np.sqrt(scales)) @ vecs.T
    return 0.5 * (sqrt_block + sqrt_block.T), scales


class SlamBackend:
    def __init__(self, config: BackendConfig, *, prior_map: MapStore | None = None):
        self.config = config
        self.prior = prior_map
        store_voxel_m = (
            config.map_voxel_m
            if config.fine_icp_voxel_m is None
            else min(config.map_voxel_m, config.fine_icp_voxel_m)
        )
        self.map = MapStore(store_voxel_m)
        self.graph = PoseGraph(cauchy_scale=config.cauchy_scale)
        self.index = ScanContextIndex(config.scan_context)
        self.visual_index = (
            VisualIndex(config.visual_retrieval, seq_consistency=config.visual_seq_consistency)
            if config.use_visual_retrieval
            else None
        )
        self.color_index = (
            ColorScanContextIndex(
                config.scan_context,
                min_color_bins=config.color_sc_min_bins,
                color_weight=config.color_sc_weight,
                max_color_distance=config.color_sc_max_distance,
                max_time_delta_s=config.visual_retrieval.max_time_delta_s,
            )
            if config.color_sc_enabled
            else None
        )
        self.admission = self._new_admission()
        self.loop_count = 0
        self.visual_candidates_proposed = 0
        self.visual_verified_count = 0
        self.visual_fallback_count = 0
        self.visual_disagreement_count = 0
        self.visual_loops_accepted = 0
        self.visual_only_loops_accepted = 0
        self.color_sc_rescored = 0
        self.color_sc_rejected = 0
        self.hybrid_same_heading_visual_accepted = 0
        self.hybrid_same_heading_visual_rejected = 0
        self.hybrid_reverse_lidar_accepted = 0
        self._last_match_visual = False
        self._last_match_visual_only = False
        self._current_visual_pair = None
        self._current_color_desc = None
        self._last_loop_query_keys: tuple = ()
        self._last_loop_target_keys: tuple = ()
        self._candidate_target_keys: dict = {}
        self.measured_query_submaps = 0
        self.measured_target_submaps = 0
        self.measured_query_frames_total = 0
        self.measured_target_frames_total = 0
        self.surface_checks = 0
        self.surface_accepted = 0
        self.surface_rejected_by_reason: dict[str, int] = {}
        self.last_error: str | None = None
        self._previous: Keyframe | None = None
        self._source_frames: list[Keyframe] = []
        self._up_body: dict = {}
        self._floor_normal_body: dict = {}
        self._floor_measurements: dict[object, FloorPlaneMeasurement] = {}
        self._gravity_world: np.ndarray = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        self._gravity_world_calibrated: bool = False
        self._last_gravity_opt_count: int = 0
        self._last_window_ba_nodes: int = 0
        self._last_global_ba_nodes: int = 0
        self._prev_odom_check_pts: tuple[object, np.ndarray] | None = None
        self._zupt_anchor_kf: Keyframe | None = None
        self._zupt_recovery_anchor: Keyframe | None = None
        self._path_m: dict = {}  # key -> cumulative odom path length
        self._last_loop_path_m: float | None = None
        self._last_loop_stamp_ns: int | None = None
        self._stats = {
            "loops_skipped_spacing": 0,
            "loops_skipped_zupt": 0,
            "loops_rejected_tilt": 0,
            "loops_rejected_degenerate": 0,
            "zupt_full_stop_count": 0,
            "zupt_pivot_count": 0,
            "floor_planes_detected": 0,
            "floor_coplanar_edges": 0,
            "floor_gravity_priors_added": 0,
            "xicp_degenerate_loops": 0,
            "odometry_icp_checked": 0,
            "odometry_icp_repaired": 0,
            "planar_ba_runs": 0,
            "planar_ba_window_runs": 0,
            "planar_ba_global_runs": 0,
            "planar_ba_clusters": 0,
            "planar_ba_obs": 0,
            "planar_ba_wall_s": 0.0,
        }
        self._correction: np.ndarray | None = None
        self._tracking = False
        self._failures = 0
        self._since_check = 0
        if prior_map is not None:
            for key, desc in prior_map.descriptors.items():
                self.index.add(key, desc)
                prior_kf = prior_map.keyframes[key]
                if self.visual_index is not None:
                    self.visual_index.add(prior_kf)
                if self.color_index is not None:
                    self.color_index.add_keyframe(prior_kf)

    @property
    def T_world_odom(self) -> np.ndarray | None:
        return self._correction

    def observability_stats(self) -> dict:
        stats = dict(self._stats)
        if self.prior is not None:
            travel_m = None
            travel_since_last_loop_m = None
            last_loop_age_s = None
        else:
            raw_travel = 0.0 if self._previous is None else float(self._path_m.get(self._previous.key, 0.0))
            travel_m = raw_travel if np.isfinite(raw_travel) else None
            if travel_m is None:
                travel_since_last_loop_m = None
            elif self._last_loop_path_m is None:
                travel_since_last_loop_m = travel_m
            else:
                delta_travel = float(travel_m - self._last_loop_path_m)
                travel_since_last_loop_m = max(0.0, delta_travel) if np.isfinite(delta_travel) else None
            if self._previous is None or self._last_loop_stamp_ns is None:
                last_loop_age_s = None
            elif self._previous.stamp_ns < self._last_loop_stamp_ns:
                last_loop_age_s = None
            else:
                age = float((self._previous.stamp_ns - self._last_loop_stamp_ns) * 1e-9)
                last_loop_age_s = age if np.isfinite(age) else None
        stats.update(
            {
                "travel_m": travel_m,
                "travel_since_last_loop_m": travel_since_last_loop_m,
                "last_loop_age_s": last_loop_age_s,
                "zupt_active": self._zupt_anchor_kf is not None,
                "measured_query_submaps": int(self.measured_query_submaps),
                "measured_query_frames_total": int(self.measured_query_frames_total),
                "measured_target_submaps": int(self.measured_target_submaps),
                "measured_target_frames_total": int(self.measured_target_frames_total),
                "surface_checks": int(self.surface_checks),
                "surface_accepted": int(self.surface_accepted),
                "surface_rejected_by_reason": dict(self.surface_rejected_by_reason),
                "visual_candidates_proposed": int(self.visual_candidates_proposed),
                "visual_verified_count": int(self.visual_verified_count),
                "visual_fallback_count": int(self.visual_fallback_count),
                "visual_disagreement_count": int(self.visual_disagreement_count),
                "visual_loops_accepted": int(self.visual_loops_accepted),
                "visual_only_loops_accepted": int(self.visual_only_loops_accepted),
                "visual_seq_rejected": int(
                    0 if self.visual_index is None else self.visual_index.seq_rejected_count
                ),
                "color_sc_rescored": int(self.color_sc_rescored),
                "color_sc_rejected": int(self.color_sc_rejected),
                "hybrid_same_heading_visual_accepted": int(self.hybrid_same_heading_visual_accepted),
                "hybrid_same_heading_visual_rejected": int(self.hybrid_same_heading_visual_rejected),
                "hybrid_reverse_lidar_accepted": int(self.hybrid_reverse_lidar_accepted),
            }
        )
        return stats

    def _graph_points_body_list(self) -> list[np.ndarray]:
        return [
            self.map.keyframes[key].points_body()
            if key in self.map.keyframes
            else np.empty((0, 3), dtype=np.float64)
            for key in self.graph.keys
        ]

    def refine_planar_ba(
        self,
        *,
        outer_rounds: int | None = None,
        inner_iterations: int | None = None,
        update_correction: bool = True,
    ) -> PlanarBaResult:
        """Run oriented-normal 3D planar BA on the full pose graph and update poses."""
        c = self.config
        result = refine_poses_oriented_planar_ba(
            self.graph,
            self._graph_points_body_list(),
            voxel_m=c.planar_ba_voxel_m,
            min_frame_pts=c.planar_ba_min_frame_pts,
            plane_ratio_max=c.planar_ba_plane_ratio_max,
            plane_thickness_max_m=c.planar_ba_plane_thickness_max_m,
            min_normal_cos=c.planar_ba_min_normal_cos,
            max_plane_offset_m=c.planar_ba_max_offset_m,
            max_frames_per_cluster=c.planar_ba_max_frames_per_cluster,
            plane_sigma_m=c.planar_ba_sigma_m,
            cauchy_scale=c.cauchy_scale,
            outer_rounds=c.planar_ba_outer_rounds if outer_rounds is None else int(outer_rounds),
            inner_iterations=c.planar_ba_iterations if inner_iterations is None else int(inner_iterations),
        )
        self._stats["planar_ba_runs"] += 1
        self._stats["planar_ba_global_runs"] += 1
        self._stats["planar_ba_clusters"] = int(result.clusters_used)
        self._stats["planar_ba_obs"] = int(result.obs_used)
        self._stats["planar_ba_wall_s"] = float(self._stats["planar_ba_wall_s"]) + float(result.wall_s)
        if result.used_cpp and result.clusters_used > 0 and len(result.poses) == len(self.graph.poses):
            self.graph.poses = [p.copy() for p in result.poses]
            self.map.update_poses({key: self.graph.pose(key) for key in self.graph.keys})
            if update_correction and self._previous is not None and self._previous.key in self.graph.index:
                T_world_body = self.graph.pose(self._previous.key)
                self._correction = T_world_body @ invert(self._previous.T_odom_body)
        return result

    def refine_window_planar_ba(
        self,
        window_size: int | None = None,
        *,
        outer_rounds: int = 1,
        inner_iterations: int | None = None,
    ) -> PlanarBaResult:
        """Run fast online sliding-window oriented planar BA on the newest keyframes."""
        c = self.config
        win = c.planar_ba_window_size if window_size is None else int(window_size)
        updated_map, result = refine_window_oriented_planar_ba(
            self.graph,
            self._graph_points_body_list(),
            win,
            voxel_m=c.planar_ba_voxel_m,
            min_frame_pts=c.planar_ba_min_frame_pts,
            plane_ratio_max=c.planar_ba_plane_ratio_max,
            plane_thickness_max_m=c.planar_ba_plane_thickness_max_m,
            min_normal_cos=c.planar_ba_min_normal_cos,
            max_plane_offset_m=c.planar_ba_max_offset_m,
            max_frames_per_cluster=c.planar_ba_max_frames_per_cluster,
            plane_sigma_m=c.planar_ba_sigma_m,
            cauchy_scale=c.cauchy_scale,
            outer_rounds=int(outer_rounds),
            inner_iterations=c.planar_ba_online_iterations if inner_iterations is None else int(inner_iterations),
        )
        self._stats["planar_ba_runs"] += 1
        self._stats["planar_ba_window_runs"] += 1
        self._stats["planar_ba_clusters"] = int(result.clusters_used)
        self._stats["planar_ba_obs"] = int(result.obs_used)
        self._stats["planar_ba_wall_s"] = float(self._stats["planar_ba_wall_s"]) + float(result.wall_s)
        if result.used_cpp and result.clusters_used > 0 and updated_map:
            pose_updates = {}
            for node_idx, pose in updated_map.items():
                self.graph.poses[node_idx] = pose.copy()
                pose_updates[self.graph.keys[node_idx]] = pose
            self.map.update_poses(pose_updates)
        return result

    def finalize(self) -> dict:
        """Run optional end-of-session refinement (oriented planar BA when enabled)."""
        if self.prior is not None or not self.config.planar_ba_enabled or len(self.graph.keys) <= 1:
            return {"ran": False}
        result = self.refine_planar_ba()
        return {
            "ran": True,
            "used_cpp": result.used_cpp,
            "converged": result.converged,
            "clusters_used": result.clusters_used,
            "obs_used": result.obs_used,
            "error_before": result.error_before,
            "error_after": result.error_after,
            "wall_s": result.wall_s,
        }

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
            if self.config.use_measured_submaps and self.config.measured_submap.use_query_submap:
                sub_cfg = self.config.measured_submap
                keep_history = max(sub_cfg.max_frames - 1, sub_cfg.submap_window or 0)
                if keep_history > 0:
                    self._source_frames.append(keyframe)
                    if len(self._source_frames) > keep_history:
                        self._source_frames = self._source_frames[-keep_history:]
        except Exception as error:  # one bad keyframe must not stop the backend
            self.last_error = f"{type(error).__name__}: {error}"
            event["error"] = self.last_error
        event["T_world_odom"] = None if self._correction is None else self._correction.tolist()
        return event

    # Mapping mode -------------------------------------------------------

    def _map_step(self, keyframe: Keyframe, desc: np.ndarray) -> dict:
        zupt_mode, _ = self._add_node(keyframe)
        self._current_visual_pair = self.visual_index.extract(keyframe) if self.visual_index is not None else None
        self._current_color_desc = self.color_index.compute(keyframe) if self.color_index is not None else None
        try:
            loop_key, rejected, rejected_kind = self._find_loop(keyframe, desc, is_zupt=zupt_mode is not None)
            self.map.add(keyframe, self.graph.pose(keyframe.key), desc)
            self.index.add(keyframe.key, desc)
            if self.visual_index is not None:
                self.visual_index.add(self.map.keyframes[keyframe.key], precomputed=self._current_visual_pair)
            if self.color_index is not None:
                self.color_index.add(keyframe.key, self._current_color_desc)
        finally:
            self._current_visual_pair = None
            self._current_color_desc = None
        optimized = False
        if loop_key is not None:
            self.loop_count += 1
            if self._last_match_visual:
                self.visual_loops_accepted += 1
                if self._last_match_visual_only:
                    self.visual_only_loops_accepted += 1
            self.graph.optimize(self.config.optimize_iterations)
            self.map.update_poses({key: self.graph.pose(key) for key in self.graph.keys})
            self._last_gravity_opt_count = self._stats["floor_gravity_priors_added"]
            optimized = True
        elif (
            self.config.gravity_optimize_every > 0
            and (self._stats["floor_gravity_priors_added"] - self._last_gravity_opt_count)
            >= self.config.gravity_optimize_every
        ):
            self.graph.optimize(self.config.optimize_iterations)
            self.map.update_poses({key: self.graph.pose(key) for key in self.graph.keys})
            self._last_gravity_opt_count = self._stats["floor_gravity_priors_added"]
            optimized = True
        if self.config.planar_ba_enabled and len(self.graph.keys) >= 4:
            n_nodes = len(self.graph.keys)
            c = self.config
            run_global_ba = c.planar_ba_global_every > 0 and (
                (n_nodes - self._last_global_ba_nodes) >= c.planar_ba_global_every
                or (
                    loop_key is not None
                    and (n_nodes - self._last_global_ba_nodes) >= max(1, c.planar_ba_global_every // 2)
                )
            )
            if run_global_ba:
                ba_res = self.refine_planar_ba(
                    outer_rounds=1,
                    inner_iterations=c.planar_ba_online_iterations,
                    update_correction=False,
                )
                self._last_global_ba_nodes = n_nodes
                self._last_window_ba_nodes = n_nodes
                if ba_res.used_cpp and ba_res.clusters_used > 0:
                    optimized = True
            elif (
                c.planar_ba_every > 0
                and c.planar_ba_window_size > 1
                and zupt_mode is None
                and (n_nodes - self._last_window_ba_nodes) >= c.planar_ba_every
            ):
                ba_res = self.refine_window_planar_ba(
                    c.planar_ba_window_size,
                    outer_rounds=1,
                    inner_iterations=c.planar_ba_online_iterations,
                )
                self._last_window_ba_nodes = n_nodes
                if ba_res.used_cpp and ba_res.clusters_used > 0:
                    optimized = True
        if self._correction is None:
            self._correction = np.eye(4)
        C_graph = self.graph.pose(keyframe.key) @ invert(keyframe.T_odom_body)
        if zupt_mode is not None and not optimized:
            status = self._propose_anchored(C_graph, keyframe)
        else:
            status = self._propose(C_graph, keyframe)
            if optimized and status == "pending":
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
        _, res_final, _ = _correspondences(p_rot + shift, prev_pts, normals, tree, inlier_dist_m)
        if len(res_final) < 50:
            return None
        rms = float(np.sqrt(np.mean(res_final * res_final)))
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
        zupt_voxel_m = max(c.map_voxel_m, 0.2)
        anchor_pts = voxel_downsample(anchor.points_body(), zupt_voxel_m)
        curr_pts = voxel_downsample(keyframe.points_body(), zupt_voxel_m)
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
        zupt_voxel_m = max(c.map_voxel_m, 0.2)
        anchor_pts = voxel_downsample(anchor.points_body(), zupt_voxel_m)
        curr_pts = voxel_downsample(keyframe.points_body(), zupt_voxel_m)
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

    def _repair_odometry_delta(
        self,
        previous: Keyframe,
        keyframe: Keyframe,
        delta_use: np.ndarray,
        prev_floor: FloorPlaneMeasurement | None,
        cur_floor: FloorPlaneMeasurement | None,
    ) -> tuple[np.ndarray, bool]:
        """Pure-geometric scan-to-scan consistency check and point-to-plane ICP repair.

        When measured LiDAR geometry disagrees with the frontend odometry step
        (either overall scan-to-scan median distance exceeds threshold or both
        frames measure wide floor slabs whose height disagrees with ``delta_use``),
        runs point-to-plane ICP from both ``delta_use`` and a zero-vertical-jump
        seed and adopts the result only when it strictly and materially improves
        full-scan alignment without introducing rotational jump.
        """
        c = self.config
        check_voxel = max(c.map_voxel_m, 0.25)
        if self._prev_odom_check_pts is not None and self._prev_odom_check_pts[0] == previous.key:
            prev_pts = self._prev_odom_check_pts[1]
        else:
            prev_pts = voxel_downsample(previous.points_body(), check_voxel)
        curr_pts = voxel_downsample(keyframe.points_body(), check_voxel)
        self._prev_odom_check_pts = (keyframe.key, curr_pts)
        if len(prev_pts) < 50 or len(curr_pts) < 50:
            return delta_use, False

        tree = cKDTree(prev_pts)
        d_odom, _ = tree.query(transform_points(delta_use, curr_pts))
        med_odom = float(np.median(d_odom))

        floor_disagreement = False
        if prev_floor is not None and cur_floor is not None:
            dz_odom = float(prev_floor.normal_body @ delta_use[:3, 3])
            dz_floor = float(cur_floor.height_below_body_m - prev_floor.height_below_body_m)
            floor_disagreement = abs(dz_odom - dz_floor) > c.floor_plane_max_step_dz_m

        if med_odom <= c.odometry_icp_repair_max_median_m and not floor_disagreement:
            return delta_use, False

        self._stats["odometry_icp_checked"] += 1
        icp_voxel = c.fine_icp_voxel_m if c.fine_icp_voxel_m is not None else c.icp_voxel_m
        iters = min(c.icp_iterations, 20)
        p_prev_full = previous.points_body()
        p_curr_full = keyframe.points_body()

        init_level = np.array(delta_use, dtype=np.float64, copy=True)
        if prev_floor is not None and cur_floor is not None:
            n_a = prev_floor.normal_body
            dz_odom = float(n_a @ init_level[:3, 3])
            dz_floor = float(cur_floor.height_below_body_m - prev_floor.height_below_body_m)
            init_level[:3, 3] = init_level[:3, 3] + (dz_floor - dz_odom) * n_a
        else:
            init_level[2, 3] = 0.0

        candidates = [
            icp_point_to_plane(
                p_curr_full,
                p_prev_full,
                init_level,
                voxel_m=icp_voxel,
                max_correspondence_m=0.8,
                iterations=iters,
            ),
            icp_point_to_plane(
                p_curr_full,
                p_prev_full,
                delta_use,
                voxel_m=icp_voxel,
                max_correspondence_m=0.8,
                iterations=iters,
            ),
        ]
        valid = [
            r
            for r in candidates
            if r.converged
            and r.fitness >= c.odometry_icp_repair_min_fitness
            and r.rmse_m <= c.odometry_icp_repair_max_rmse_m
            and rotation_angle(delta_use[:3, :3], r.T_target_source[:3, :3]) <= 0.15
        ]
        if not valid:
            return delta_use, False

        # Preserve short-horizon IMU/odometry rotation R (avoids 1-2 deg single-scan
        # pitch/roll jitter) and adopt ICP translation only when it materially fixes
        # a genuine >= 5 cm translation slip in the 3D scan-to-scan alignment.
        best_T = None
        best_med = med_odom
        for r in valid:
            T_cand = np.array(delta_use, dtype=np.float64, copy=True)
            T_cand[:3, 3] = r.T_target_source[:3, 3]
            if float(np.linalg.norm(T_cand[:3, 3] - delta_use[:3, 3])) < 0.05:
                continue
            d_cand, _ = tree.query(transform_points(T_cand, curr_pts))
            med_cand = float(np.median(d_cand))
            if med_cand < best_med:
                best_med = med_cand
                best_T = T_cand

        if best_T is None or best_med + 0.04 >= med_odom:
            return delta_use, False
        self._stats["odometry_icp_repaired"] += 1
        return best_T, True

    def _add_node(self, keyframe: Keyframe) -> tuple[str | None, np.ndarray]:
        """Node at the previous optimised pose composed with the odometry (or ZUPT) delta."""
        c = self.config
        previous = self._previous
        zupt_mode: str | None = None
        delta_use = np.eye(4, dtype=np.float64)
        floor_meas: FloorPlaneMeasurement | None = None
        if (
            c.floor_plane_enabled
            or c.floor_normal_gravity_enabled
            or c.floor_normal_graph_sigma_rad is not None
            or c.floor_gravity_world_calibration
        ):
            if previous is None:
                R_floor_hint = keyframe.T_odom_body[:3, :3]
            else:
                R_step = project_so3((invert(previous.T_odom_body) @ keyframe.T_odom_body)[:3, :3])
                R_floor_hint = project_so3(self.graph.pose(previous.key)[:3, :3] @ R_step)
            floor_meas = detect_floor_plane(
                keyframe.points_body(),
                R_floor_hint,
                max_body_tilt_deg=c.floor_plane_max_tilt_deg,
            )
            if floor_meas is not None:
                self._stats["floor_planes_detected"] += 1
                self._floor_measurements[keyframe.key] = floor_meas
                if c.floor_normal_gravity_enabled:
                    self._floor_normal_body[keyframe.key] = floor_meas.normal_body
        if previous is None:
            self.graph.add_node(keyframe.key, keyframe.T_odom_body)
            self._path_m[keyframe.key] = 0.0
            if c.floor_gravity_world_calibration and floor_meas is not None:
                g_w = project_so3(keyframe.T_odom_body[:3, :3]) @ floor_meas.normal_body
                g_norm = float(np.linalg.norm(g_w))
                if g_norm > 1e-6:
                    self._gravity_world = g_w / g_norm
                    self._gravity_world_calibrated = True
                    self.graph.set_gravity_world(self._gravity_world)
                up_body = floor_meas.normal_body.copy()
            else:
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
                if c.odometry_icp_repair_enabled and parent_key == previous.key:
                    parent_floor = self._floor_measurements.get(parent_key)
                    delta_use, repaired = self._repair_odometry_delta(
                        previous,
                        keyframe,
                        delta_use,
                        parent_floor,
                        floor_meas,
                    )
                    if repaired:
                        step_dist = float(np.linalg.norm(delta_use[:3, 3]))
                if c.floor_plane_enabled and floor_meas is not None and parent_key == previous.key:
                    parent_floor = self._floor_measurements.get(parent_key)
                    if parent_floor is not None:
                        delta_use, leveled = project_coplanar_odom_delta(
                            delta_use,
                            parent_floor,
                            floor_meas,
                            max_step_dz_m=c.floor_plane_max_step_dz_m,
                        )
                        if leveled:
                            self._stats["floor_coplanar_edges"] += 1
            if zupt_mode is not None:
                self._zupt_recovery_anchor = None
                if self._zupt_anchor_kf is None:
                    self._zupt_anchor_kf = previous
            else:
                self._zupt_anchor_kf = None
            new_pose = self.graph.pose(parent_key) @ delta_use
            self.graph.add_node(keyframe.key, new_pose)
            self.graph.add_edge(
                parent_key,
                keyframe.key,
                delta_use,
                odometry,
                robust=(c.odometry_robust and zupt_mode is None),
            )
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
            if (
                c.floor_gravity_world_calibration
                and not self._gravity_world_calibrated
                and self.loop_count == 0
                and floor_meas is not None
            ):
                g_w = project_so3(new_pose[:3, :3]) @ floor_meas.normal_body
                g_norm = float(np.linalg.norm(g_w))
                if g_norm > 1e-6:
                    self._gravity_world = g_w / g_norm
                    self._gravity_world_calibrated = True
                    self.graph.set_gravity_world(self._gravity_world)
                    up_body = floor_meas.normal_body.copy()
        if c.graph_gravity_sigma_rad is not None:
            self.graph.add_gravity_prior(
                keyframe.key,
                up_body,
                c.graph_gravity_sigma_rad,
                robust=c.graph_gravity_robust,
            )
        if c.floor_normal_graph_sigma_rad is not None and floor_meas is not None and zupt_mode is None:
            self.graph.add_gravity_prior(
                keyframe.key,
                floor_meas.normal_body,
                c.floor_normal_graph_sigma_rad,
                robust=True,
            )
            self._stats["floor_gravity_priors_added"] += 1
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

        recent = set(list(self.map.keyframes)[-c.exclude_recent :]) if c.exclude_recent > 0 else set()
        gap_ns = c.min_loop_gap_s * 1e9
        excluded = {
            key
            for key, old in self.map.keyframes.items()
            if key in recent
            or keyframe.stamp_ns - old.stamp_ns < gap_ns
            or (c.min_loop_travel_m > 0.0 and (curr_path - self._path_m.get(key, curr_path)) < c.min_loop_travel_m)
        }
        self._candidate_target_keys = {}
        match, rejected, rejected_kind = self._search_with_rejection(
            self.map,
            keyframe,
            desc,
            lambda key: key in excluded,
            lambda key, exclude_keys=None: self._loop_submap(key, excluded, exclude_keys=exclude_keys),
            apply_tilt_gate=True,
            apply_degeneracy_gate=True,
            apply_odom_gate=True,
            use_gravity_prior=True,
            return_hessian=(c.xicp_enabled or c.min_translation_observability is not None),
        )
        if match is None:
            return None, rejected, rejected_kind
        key, result = match
        self._last_loop_target_keys = tuple(self._candidate_target_keys.get(key, ()))
        T_world_new = result.T_target_source
        if c.use_measured_submaps and c.measured_submap.anchor_mode == "candidate":
            anchor = key
        else:
            anchor_ex = excluded | set(self._last_loop_query_keys)
            anchor = min(
                self._loop_window(key, anchor_ex),
                key=lambda k: np.linalg.norm(self.map.poses[k][:3, 3] - T_world_new[:3, 3]),
            )
        loop = c.loop_information()
        if c.xicp_enabled:
            loop, is_degenerate = self._modulate_loop_information(
                loop, result.hessian, T_world_new[:3, :3]
            )
            if is_degenerate:
                self._stats["xicp_degenerate_loops"] += 1
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

    def _loop_submap(self, key, excluded: set, *, exclude_keys: set | None = None) -> np.ndarray:
        """World cloud of the candidate's ``_loop_window`` or bounded measured submap."""
        effective_excluded = excluded if not exclude_keys else (excluded | exclude_keys)
        if self.config.use_measured_submaps and self.config.measured_submap.use_history_submap:
            anchor = self.map.keyframes[key]
            measured = build_submap(
                self.map.keyframes.values(),
                anchor,
                self.config.measured_submap,
                excluded=excluded,
                exclude_keys=exclude_keys,
                track_sources=False,
            )
            self._candidate_target_keys[key] = tuple(measured.source_keys)
            self.measured_target_submaps += 1
            self.measured_target_frames_total += len(measured.source_keys)
            return transform_points(self.map.poses[key], measured.points_body)
        window = set(self._loop_window(key, effective_excluded))
        self._candidate_target_keys[key] = tuple(k for k in self.map.keyframes if k in window)
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

    def _prior_submap(self, key, *, exclude_keys: set | None = None) -> np.ndarray:
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

    def _verified_up_body(self, keyframe: Keyframe) -> np.ndarray:
        """Consistent body-frame gravity-up unit vector shared by ICP, tilt gate, and graph prior."""
        vec = self._up_body.get(keyframe.key, keyframe.T_odom_body[2, :3])
        norm = float(np.linalg.norm(vec))
        return (vec / norm) if norm > 1e-6 else np.array([0.0, 0.0, 1.0], dtype=np.float64)

    def _tilt_rejection_reason(
        self,
        keyframe: Keyframe,
        T_world_body: np.ndarray,
        *,
        up_body: np.ndarray | None = None,
    ) -> str | None:
        limit = self.config.max_loop_tilt_rad
        if limit is None:
            return None
        if up_body is not None:
            vec = np.asarray(up_body, dtype=np.float64).reshape(3)
            norm = float(np.linalg.norm(vec))
            resolved_up = (vec / norm) if norm > 1e-6 else self._verified_up_body(keyframe)
        else:
            resolved_up = self._verified_up_body(keyframe)
        u_world = T_world_body[:3, :3] @ resolved_up
        g_w = self._gravity_world
        tilt_rad = float(np.arctan2(float(np.linalg.norm(np.cross(u_world, g_w))), float(u_world @ g_w)))
        if tilt_rad > limit:
            return f"gravity tilt {np.degrees(tilt_rad):.2f} deg exceeds {np.degrees(limit):.2f} deg"
        return None

    def _degeneracy_rejection_reason(self, result: IcpResult) -> str | None:
        limit = self.config.min_translation_observability
        if limit is None or result.hessian is None:
            return None
        H_t = 0.5 * (result.hessian[:3, :3] + result.hessian[:3, :3].T)
        eigvals = np.linalg.eigvalsh(H_t)
        lam_max = float(eigvals[-1])
        lam_min = max(0.0, float(eigvals[0]))
        ratio = (lam_min / lam_max) if lam_max > 0.0 else 0.0
        if ratio < limit:
            return f"translation observability {ratio:.4f} below {limit:.4f}"
        return None

    def _modulate_loop_information(
        self,
        I_base: np.ndarray,
        hessian: np.ndarray | None,
        R_world_body: np.ndarray,
    ) -> tuple[np.ndarray, bool]:
        """Symmetrically attenuate degenerate ICP axes in the loop edge's 6x6 information matrix."""
        if hessian is None or hessian.shape != (6, 6) or not np.isfinite(hessian).all():
            return I_base, False
        c = self.config
        st_half, s_t = _block_sqrt_confidence(
            hessian[:3, :3], R_world_body, c.xicp_min_scale, c.xicp_cond_floor
        )
        sr_half, _ = _block_sqrt_confidence(
            hessian[3:, 3:], R_world_body, c.xicp_min_scale, c.xicp_cond_floor
        )
        s_half = np.zeros((6, 6), dtype=np.float64)
        s_half[:3, :3] = st_half
        s_half[3:, 3:] = sr_half
        I_eff = s_half @ I_base @ s_half
        I_eff = 0.5 * (I_eff + I_eff.T)
        return I_eff, bool(np.min(s_t) < 0.35)

    def _search_with_rejection(
        self,
        store: MapStore,
        keyframe: Keyframe,
        desc,
        exclude,
        submap,
        *,
        apply_tilt_gate: bool = False,
        apply_degeneracy_gate: bool = False,
        apply_odom_gate: bool = False,
        use_gravity_prior: bool = False,
        return_hessian: bool = False,
    ):
        """Best admissible ICP-verified candidate as ``((key, IcpResult) | None, reject_reason, reject_kind)``."""
        candidates = self.index.query(desc, exclude=exclude, candidates=self.config.scan_context_prefilter)
        source_body = None
        query_source_keys: set = {keyframe.key}
        self._last_loop_query_keys = (keyframe.key,)
        self._last_loop_target_keys = ()
        target_cache: dict = {}

        def _get_source() -> np.ndarray:
            nonlocal source_body, query_source_keys
            if source_body is None:
                if self.config.use_measured_submaps and self.config.measured_submap.use_query_submap:
                    measured = build_submap(
                        [*self._source_frames, keyframe],
                        keyframe,
                        self.config.measured_submap,
                        track_sources=False,
                    )
                    source_body = measured.points_body
                    query_source_keys = set(measured.source_keys)
                    self._last_loop_query_keys = tuple(measured.source_keys)
                    self.measured_query_submaps += 1
                    self.measured_query_frames_total += len(measured.source_keys)
                else:
                    source_body = keyframe.points_body()
                    query_source_keys = {keyframe.key}
                    self._last_loop_query_keys = (keyframe.key,)
            return source_body

        def _get_target(key) -> np.ndarray:
            _get_source()
            if key not in target_cache:
                target_cache[key] = submap(key, exclude_keys=query_source_keys)
            return target_cache[key]

        best = None
        best_visual = False
        best_from_sc = False
        best_hybrid_mode: str | None = None
        last_reject = None
        last_reject_kind = None
        up_body = None
        if use_gravity_prior and self.config.icp_gravity_sigma_rad is not None:
            if self.config.floor_normal_gravity_enabled and keyframe.key in self._floor_normal_body:
                up_body = self._floor_normal_body[keyframe.key]
            else:
                up_body = self._verified_up_body(keyframe)

        def _check_gates(key, result: IcpResult | None, target_world: np.ndarray) -> IcpResult | None:
            nonlocal last_reject, last_reject_kind
            if result is None:
                return None
            if apply_tilt_gate:
                tilt_reject = self._tilt_rejection_reason(
                    keyframe, result.T_target_source, up_body=up_body
                )
                if tilt_reject is not None:
                    self._stats["loops_rejected_tilt"] += 1
                    last_reject = tilt_reject
                    last_reject_kind = "tilt"
                    return None
            if apply_degeneracy_gate:
                degen_reject = self._degeneracy_rejection_reason(result)
                if degen_reject is not None:
                    self._stats["loops_rejected_degenerate"] += 1
                    last_reject = degen_reject
                    last_reject_kind = "degenerate"
                    return None
            if apply_odom_gate:
                odom_reject = self._odometry_inconsistency(key, keyframe, result.T_target_source)
                if odom_reject is not None:
                    last_reject = odom_reject
                    last_reject_kind = "odom"
                    return None
            if self.config.use_surface_verification:
                self.surface_checks += 1
                aligned_source = transform_points(result.T_target_source, _get_source())
                check = compare_surfaces(
                    model_surfaces(aligned_source, config=self.config.surface),
                    model_surfaces(target_world, config=self.config.surface),
                    self.config.surface,
                )
                if check.accepted:
                    self.surface_accepted += 1
                else:
                    self.surface_rejected_by_reason[check.reason] = (
                        self.surface_rejected_by_reason.get(check.reason, 0) + 1
                    )
                    last_reject = f"surface {check.reason}"
                    last_reject_kind = "surface"
                    return None
            return result

        # Fast path when visual retrieval, visual verification, and Color-SC are all disabled.
        if (
            not self.config.use_visual_retrieval
            and not self.config.use_visual_verification
            and self.color_index is None
        ):
            for key, dist, yaw in candidates[: self.config.icp_candidates]:
                if dist > self.config.max_scan_context_distance:
                    break
                src_cloud = _get_source()
                if key in query_source_keys:
                    continue
                T_init = store.poses[key] @ yaw_pose(yaw)
                target = _get_target(key)
                result = _check_gates(
                    key,
                    self._register(
                        src_cloud,
                        target,
                        T_init,
                        source_up_body=up_body,
                        return_hessian=return_hessian,
                    ),
                    target,
                )
                if result is not None and (best is None or result.rmse_m < best[1].rmse_m):
                    best = (key, result)
            self._last_match_visual = False
            self._last_match_visual_only = False
            if best is not None:
                return best, None, None
            return None, last_reject, last_reject_kind

        eligible_sc = [
            (key, dist, yaw) for key, dist, yaw in candidates if dist <= self.config.max_scan_context_distance
        ]
        if self.color_index is not None and eligible_sc:
            query_cdesc = (
                self._current_color_desc
                if self._current_color_desc is not None
                else self.color_index.compute(keyframe)
            )
            rescored, n_rescored, n_rejected = self.color_index.rescore(query_cdesc, eligible_sc)
            self.color_sc_rescored += n_rescored
            self.color_sc_rejected += n_rejected
            sc_pool = [(k, d, y) for k, d, y, _ in rescored[: self.config.icp_candidates]]
        else:
            sc_pool = eligible_sc[: self.config.icp_candidates]

        vr = self.config.visual_retrieval
        query_vdesc = None
        query_vfeat = None
        if self._current_visual_pair is not None:
            query_vdesc, query_vfeat = self._current_visual_pair
        elif self.visual_index is not None:
            query_vdesc, query_vfeat = self.visual_index.extract(keyframe)

        mv_radius = self.config.visual_multiview_radius
        store_keys = list(store.keyframes) if mv_radius > 0 else []
        store_key_pos = {k: idx for idx, k in enumerate(store_keys)} if mv_radius > 0 else {}
        hybrid_gate = self.config.hybrid_heading_gate_deg
        hybrid_active = hybrid_gate is not None and self.config.use_visual_retrieval
        reverse_sc_keys: set = set()

        def _final_heading_deg(target_key, T_world_query: np.ndarray) -> float:
            T_rel = invert(store.poses[target_key]) @ T_world_query
            return abs(float(np.degrees(wrap_angle(np.arctan2(T_rel[1, 0], T_rel[0, 0])))))

        def _window_keys(cand_key) -> list:
            idx = store_key_pos.get(cand_key)
            if idx is None:
                return [cand_key]
            epoch = cand_key[0] if isinstance(cand_key, tuple) else None
            lo = max(0, idx - mv_radius)
            hi = min(len(store_keys), idx + mv_radius + 1)
            return [
                k
                for k in store_keys[lo:hi]
                if (epoch is None or not isinstance(k, tuple) or k[0] == epoch)
                and (exclude is None or not exclude(k))
                and k not in query_source_keys
                and (not hybrid_active or k not in reverse_sc_keys)
            ]

        def _build_multiview_target_body(cand_key) -> np.ndarray | None:
            wkeys = _window_keys(cand_key)
            if len(wkeys) <= 1:
                return None
            T_cand_world = invert(store.poses[cand_key])
            chunks: list[np.ndarray] = []
            for k in wkeys:
                pts_b = store.keyframes[k].points_body()
                if len(pts_b) == 0:
                    continue
                if k == cand_key:
                    chunks.append(pts_b)
                else:
                    chunks.append(transform_points(T_cand_world @ store.poses[k], pts_b))
            return np.vstack(chunks) if chunks else None

        def _try_visual_candidate(key) -> tuple[object, IcpResult | None]:
            src_cloud = _get_source()
            if key in query_source_keys:
                return key, None
            target_key = key
            if mv_radius > 0 and self.visual_index is not None:
                wkeys = _window_keys(key)
                if not wkeys:
                    return key, None
                target_key = self.visual_index.select_best_window_view(query_vdesc, wkeys, key)
            if target_key in query_source_keys or (hybrid_active and target_key in reverse_sc_keys):
                return target_key, None
            target_pts_body = _build_multiview_target_body(target_key) if mv_radius > 0 else None
            target_vfeat = (
                None
                if (target_pts_body is not None or self.visual_index is None)
                else self.visual_index.local_features.get(target_key)
            )
            visual = verify_visual(
                keyframe,
                store.keyframes[target_key],
                feature_type=vr.feature_type,
                max_time_delta_s=vr.max_time_delta_s,
                inlier_m=vr.verify_inlier_m,
                min_inliers=vr.verify_min_inliers,
                ratio_threshold=vr.verify_ratio_threshold,
                max_features=vr.max_features,
                lidar_px_radius=vr.lidar_px_radius,
                source_features=query_vfeat,
                target_features=target_vfeat,
                target_points_body=target_pts_body,
            )
            if visual.status != "verified":
                return target_key, None
            self.visual_verified_count += 1
            T_init = store.poses[target_key] @ visual.T_target_source
            target = _get_target(target_key)
            result = self._register(
                src_cloud,
                target,
                T_init,
                source_up_body=up_body,
                return_hessian=return_hessian,
            )
            if result is None:
                return target_key, None
            translation, rotation = pose_difference(T_init, result.T_target_source)
            if (
                translation > self.config.visual_agree_translation_m
                or rotation > self.config.visual_agree_rotation_rad
            ):
                self.visual_disagreement_count += 1
                return target_key, None
            return target_key, _check_gates(target_key, result, target)

        def _try_lidar_candidate(
            key, yaw: float, *, is_reverse_hybrid: bool = False
        ) -> tuple[IcpResult | None, bool]:
            src_cloud = _get_source()
            if key in query_source_keys:
                return None, False
            T_init = store.poses[key] @ yaw_pose(yaw)
            target = _get_target(key)
            result = self._register(
                src_cloud,
                target,
                T_init,
                source_up_body=up_body,
                return_hessian=return_hessian,
            )
            if result is None:
                return None, False
            final_deg = _final_heading_deg(key, result.T_target_source) if hybrid_active else 0.0
            is_rev_final = bool(is_reverse_hybrid or (hybrid_active and final_deg >= hybrid_gate))
            if is_rev_final:
                if (
                    self.config.reverse_icp_fitness_min is not None
                    and result.fitness < self.config.reverse_icp_fitness_min
                ):
                    return None, True
                if (
                    self.config.reverse_icp_rmse_max is not None
                    and result.rmse_m > self.config.reverse_icp_rmse_max
                ):
                    return None, True
            checked = _check_gates(key, result, target)
            return checked, bool(hybrid_active and final_deg >= hybrid_gate)

        sc_pool_keys: set = set()
        tried_visual_keys: set = set()
        for key, _dist, yaw in sc_pool:
            sc_pool_keys.add(key)
            if hybrid_active:
                heading_deg = abs(float(np.degrees(wrap_angle(yaw))))
                has_visual_pair = keyframe.visual is not None and store.keyframes[key].visual is not None
                if heading_deg < hybrid_gate and has_visual_pair:
                    tried_visual_keys.add(key)
                    matched_key, result = _try_visual_candidate(key)
                    if result is None:
                        self.hybrid_same_heading_visual_rejected += 1
                    else:
                        final_deg = _final_heading_deg(matched_key, result.T_target_source)
                        if matched_key in reverse_sc_keys or final_deg >= hybrid_gate:
                            self.hybrid_same_heading_visual_rejected += 1
                        elif best is None or result.rmse_m < best[1].rmse_m:
                            best = (matched_key, result)
                            best_visual = True
                            best_from_sc = True
                            best_hybrid_mode = "same_heading_visual"
                else:
                    is_rev = heading_deg >= hybrid_gate
                    if is_rev:
                        reverse_sc_keys.add(key)
                    result, is_rev_final = _try_lidar_candidate(key, yaw, is_reverse_hybrid=is_rev)
                    if result is not None and (best is None or result.rmse_m < best[1].rmse_m):
                        best = (key, result)
                        best_visual = False
                        best_from_sc = True
                        best_hybrid_mode = "reverse_lidar" if is_rev_final else None
            elif self.config.use_visual_verification:
                tried_visual_keys.add(key)
                matched_key, result = _try_visual_candidate(key)
                if result is not None and (best is None or result.rmse_m < best[1].rmse_m):
                    best = (matched_key, result)
                    best_visual = True
                    best_from_sc = True
            else:
                result, _ = _try_lidar_candidate(key, yaw)
                if result is not None and (best is None or result.rmse_m < best[1].rmse_m):
                    best = (key, result)
                    best_visual = False
                    best_from_sc = True

        if self.visual_index is not None:
            same_epoch_only = self.prior is None and isinstance(keyframe.key, tuple)

            def _visual_exclude(key) -> bool:
                if exclude is not None and exclude(key):
                    return True
                if same_epoch_only and isinstance(key, tuple) and key[0] != keyframe.key[0]:
                    return True
                if hybrid_active and key in reverse_sc_keys:
                    return True
                return False

            visual_candidates = self.visual_index.query(
                keyframe, exclude=_visual_exclude, precomputed_desc=query_vdesc
            )
            self.visual_candidates_proposed += len(visual_candidates)
            for key, _distance in visual_candidates:
                if key in tried_visual_keys or (hybrid_active and key in reverse_sc_keys):
                    continue
                tried_visual_keys.add(key)
                matched_key, result = _try_visual_candidate(key)
                if result is None:
                    if (
                        not self.config.use_visual_verification
                        and not hybrid_active
                        and key in sc_pool_keys
                    ):
                        self.visual_fallback_count += 1
                    continue
                if hybrid_active:
                    final_deg = _final_heading_deg(matched_key, result.T_target_source)
                    if matched_key in reverse_sc_keys or final_deg >= hybrid_gate:
                        self.hybrid_same_heading_visual_rejected += 1
                        continue
                if (
                    best is None
                    or result.rmse_m < best[1].rmse_m
                    or (not best_visual and result.rmse_m <= best[1].rmse_m + 1e-9)
                ):
                    best = (matched_key, result)
                    best_visual = True
                    best_from_sc = key in sc_pool_keys
                    if hybrid_active:
                        best_hybrid_mode = "same_heading_visual"

        self._last_match_visual = best is not None and best_visual
        self._last_match_visual_only = self._last_match_visual and not best_from_sc
        if best is not None:
            if hybrid_active:
                if best_hybrid_mode == "same_heading_visual":
                    self.hybrid_same_heading_visual_accepted += 1
                elif best_hybrid_mode == "reverse_lidar":
                    self.hybrid_reverse_lidar_accepted += 1
            return best, None, None
        return None, last_reject, last_reject_kind

    def _search(self, store: MapStore, keyframe: Keyframe, desc, exclude, submap):
        """Best ICP-verified ScanContext candidate as ``(key, IcpResult)``, or None.

        ``submap(key)`` builds the world-frame ICP target around a candidate;
        the initial guess is the candidate's pose rotated by the ScanContext
        yaw. Among verified candidates the lowest RMSE wins.
        """
        best, _, _ = self._search_with_rejection(store, keyframe, desc, exclude, submap)
        return best

    def _register(
        self,
        source_body,
        target_world,
        T_init,
        *,
        source_up_body: np.ndarray | None = None,
        return_hessian: bool = False,
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
            target_up_world=self._gravity_world,
            gravity_sigma_rad=c.icp_gravity_sigma_rad,
        )
        fine_voxel_m = c.icp_voxel_m if c.fine_icp_voxel_m is None else c.fine_icp_voxel_m
        need_hessian = return_hessian or (c.min_translation_observability is not None)
        fine = icp_point_to_plane(
            source_body,
            target_world,
            coarse.T_target_source,
            max_correspondence_m=c.fine_correspondence_m,
            iterations=c.icp_iterations,
            voxel_m=fine_voxel_m,
            source_up_body=source_up_body,
            target_up_world=self._gravity_world,
            gravity_sigma_rad=c.icp_gravity_sigma_rad,
            return_hessian=need_hessian,
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
