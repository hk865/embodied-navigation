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

from .correction import CorrectionAdmission
from .keyframes import Keyframe, transform_points
from .map_store import MapStore
from .pose_graph import PoseGraph, invert, pose_difference
from .registration import IcpResult, icp_point_to_plane
from .scan_context import ScanContextConfig, ScanContextIndex, descriptor
from .surfaces import SurfaceConfig, compare_surfaces, model_surfaces


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
    # 表面一致性门控实验分支专用开关与参数（默认关闭，需配合重力对齐坐标系使用）。
    use_surface_verification: bool = False
    gravity_aligned: bool = False
    surface: SurfaceConfig = field(default_factory=SurfaceConfig)

    def __post_init__(self) -> None:
        self.odometry_information()
        self.loop_information()
        if not isinstance(self.surface, SurfaceConfig):
            raise ValueError(f"surface 必须为 SurfaceConfig 实例，当前收到 {type(self.surface).__name__}")
        if self.use_surface_verification and not self.gravity_aligned:
            raise ValueError("启用 use_surface_verification 时必须声明 gravity_aligned=True")

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
        if "surface" in values:
            values["surface"] = _surface_from_dict(values["surface"])
        for flag in ("use_surface_verification", "gravity_aligned"):
            if flag in values and not isinstance(values[flag], bool):
                raise ValueError(f"{flag} 必须为布尔值，当前收到 {values[flag]!r}")
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


def _surface_from_dict(data: dict) -> SurfaceConfig:
    """严格解析嵌套的 SurfaceConfig，拒绝未知键与非法边界。"""
    if not isinstance(data, dict):
        raise ValueError(f"surface 必须为映射对象，当前收到 {type(data).__name__}")
    unknown = sorted(set(data) - {option.name for option in fields(SurfaceConfig)}, key=str)
    if unknown:
        raise ValueError(f"未知的 surface 配置键: {unknown}")
    return SurfaceConfig(**data)


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
        self.surface_checks = 0
        self.surface_accepted = 0
        self.surface_rejected_by_reason: dict[str, int] = {}
        self.last_error: str | None = None
        self._previous: Keyframe | None = None
        self._path_m: dict = {}  # key -> cumulative odom path length
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
        self._add_node(keyframe)
        loop_key, rejected = self._find_loop(keyframe, desc)
        self.map.add(keyframe, self.graph.pose(keyframe.key), desc)
        self.index.add(keyframe.key, desc)
        if loop_key is not None:
            self.loop_count += 1
            self.graph.optimize(self.config.optimize_iterations)
            self.map.update_poses({key: self.graph.pose(key) for key in self.graph.keys})
        if self._correction is None:
            self._correction = np.eye(4)
        status = self._propose(self.graph.pose(keyframe.key) @ invert(keyframe.T_odom_body), keyframe)
        event = {"loop": loop_key is not None, "loop_key": loop_key, "adopted": status == "adopted"}
        if rejected is not None:
            event["loop_rejected"] = rejected
        return event

    def _add_node(self, keyframe: Keyframe) -> None:
        """Node at the previous optimised pose composed with the odometry delta."""
        previous = self._previous
        if previous is None:
            self.graph.add_node(keyframe.key, keyframe.T_odom_body)
            self._path_m[keyframe.key] = 0.0
        else:
            delta = invert(previous.T_odom_body) @ keyframe.T_odom_body
            self.graph.add_node(keyframe.key, self.graph.pose(previous.key) @ delta)
            odometry = self.config.odometry_information()
            self.graph.add_edge(previous.key, keyframe.key, delta, odometry)
            self._path_m[keyframe.key] = self._path_m[previous.key] + float(np.linalg.norm(delta[:3, 3]))
        self._previous = keyframe

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

    def _find_loop(self, keyframe: Keyframe, desc: np.ndarray):
        """Add one verified loop edge to the newest keyframe.

        Returns ``(partner key, None)`` for an added loop, ``(None, reason)``
        when a geometric match failed the odometry check, else ``(None, None)``.
        """
        recent = set(list(self.map.keyframes)[-self.config.exclude_recent :])
        gap_ns = self.config.min_loop_gap_s * 1e9
        excluded = {
            key
            for key, old in self.map.keyframes.items()
            if key in recent or keyframe.stamp_ns - old.stamp_ns < gap_ns
        }
        match = self._search(
            self.map, keyframe, desc, lambda key: key in excluded, lambda key: self._loop_submap(key, excluded)
        )
        if match is None:
            return None, None
        key, result = match
        T_world_new = result.T_target_source
        rejected = self._odometry_inconsistency(key, keyframe, T_world_new)
        if rejected is not None:
            return None, rejected
        anchor = min(
            self._loop_window(key, excluded),
            key=lambda k: np.linalg.norm(self.map.poses[k][:3, 3] - T_world_new[:3, 3]),
        )
        loop = self.config.loop_information()
        self.graph.add_edge(anchor, keyframe.key, invert(self.graph.pose(anchor)) @ T_world_new, loop, robust=True)
        return anchor, None

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

    def _search(self, store: MapStore, keyframe: Keyframe, desc, exclude, submap):
        """Best ICP-verified ScanContext candidate as ``(key, IcpResult)``, or None.

        ``submap(key)`` builds the world-frame ICP target around a candidate;
        the initial guess is the candidate's pose rotated by the ScanContext
        yaw. Among verified candidates the lowest RMSE wins.
        """
        candidates = self.index.query(desc, exclude=exclude, candidates=self.config.scan_context_prefilter)
        source_body = None
        best = None
        for key, dist, yaw in candidates[: self.config.icp_candidates]:
            if dist > self.config.max_scan_context_distance:
                break
            if source_body is None:
                # 仅在首个合格候选进入 ICP 时才计算查询帧机体系点云，默认关闭或无候选时严格零开销。
                source_body = keyframe.points_body()
            T_init = store.poses[key] @ yaw_pose(yaw)
            target = submap(key)
            result = self._register(source_body, target, T_init)
            if result is not None and self.config.use_surface_verification:
                # 在重力对齐的世界系下校验查询点云与目标子图的局部实测平面覆盖一致性。
                self.surface_checks += 1
                aligned_source = transform_points(result.T_target_source, source_body)
                check = compare_surfaces(
                    model_surfaces(aligned_source, config=self.config.surface),
                    model_surfaces(target, config=self.config.surface),
                    self.config.surface,
                )
                if check.accepted:
                    self.surface_accepted += 1
                else:
                    self.surface_rejected_by_reason[check.reason] = (
                        self.surface_rejected_by_reason.get(check.reason, 0) + 1
                    )
                    result = None
            if result is not None and (best is None or result.rmse_m < best[1].rmse_m):
                best = (key, result)
        return best

    def _register(self, source_body, target_world, T_init) -> IcpResult | None:
        """Coarse-then-fine ICP; the fine result must pass fitness and RMSE gates."""
        c = self.config
        coarse = icp_point_to_plane(
            source_body, target_world, T_init,
            max_correspondence_m=c.coarse_correspondence_m, iterations=c.icp_iterations, voxel_m=2 * c.icp_voxel_m,
        )
        fine = icp_point_to_plane(
            source_body, target_world, coarse.T_target_source,
            max_correspondence_m=c.fine_correspondence_m, iterations=c.icp_iterations, voxel_m=c.icp_voxel_m,
        )
        if fine.converged and fine.fitness >= c.min_fitness and fine.rmse_m <= c.max_rmse_m:
            return fine
        return None

    def _propose(self, C: np.ndarray, keyframe: Keyframe) -> str:
        """Offer C to the admission: ``"adopted"``, ``"rejected"`` (jump) or ``"pending"``."""
        rejections = len(self.admission.rejections)
        adopted = self.admission.propose(C, keyframe.T_odom_body, keyframe.stamp_ns)
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
