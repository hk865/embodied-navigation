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

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .correction import CorrectionAdmission
from .keyframes import Keyframe, transform_points
from .submaps import SubmapConfig, build_submap
from .surfaces import SurfaceConfig
from .map_store import MapStore
from .pose_graph import PoseGraph, invert, pose_difference
from .registration import IcpResult, icp_point_to_plane
from .scan_context import ScanContextConfig, ScanContextIndex, descriptor
from .trace import graph_snapshot


@dataclass
class BackendConfig:
    use_measured_submaps: bool = False
    use_visual_verification: bool = False
    use_surface_verification: bool = False
    gravity_aligned: bool = False
    visual_agree_translation_m: float = 0.3
    visual_agree_rotation_rad: float = 0.05
    surface: SurfaceConfig = field(default_factory=SurfaceConfig)
    measured_submap: SubmapConfig = field(default_factory=SubmapConfig)
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


def information(sigma_m: float, sigma_rad: float) -> np.ndarray:
    """Diagonal 6x6 information in the pose graph's (translation, rotation) order."""
    return np.diag([sigma_m**-2] * 3 + [sigma_rad**-2] * 3)


def yaw_pose(yaw: float) -> np.ndarray:
    T = np.eye(4)
    T[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    return T


class SlamBackend:
    def __init__(self, config: BackendConfig, *, prior_map: MapStore | None = None, trace=None):
        if config.use_surface_verification and not config.gravity_aligned:
            raise ValueError("surface verification requires an explicitly gravity-aligned odom frame")
        self.config = config
        self._source_frames = []
        self.trace = trace
        self.trace_error = None
        self._trace_context = {}
        self._trace_candidate = None
        self.prior = prior_map
        self.map = MapStore(config.map_voxel_m)
        self.graph = PoseGraph(cauchy_scale=config.cauchy_scale)
        self.index = ScanContextIndex(config.scan_context)
        self.admission = self._new_admission()
        self.loop_count = 0
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

    def _record(self, event_name, **fields):
        if self.trace is not None:
            try:
                self.trace.emit(event_name, **self._trace_context, **fields)
            except Exception as error:
                # 诊断失败必须可见，但不能提前结束正在进行的算法步骤。
                self.trace_error = f"{type(error).__name__}: {error}"

    @property
    def T_world_odom(self) -> np.ndarray | None:
        return self._correction

    def save_map(self, directory: Path) -> None:
        self.map.save(directory)

    def ingest(self, keyframe: Keyframe) -> dict:
        """Process one keyframe; failures are reported in ``event["error"]``."""
        self._trace_context = {"key": keyframe.key, "stamp_ns": keyframe.stamp_ns}
        event = {"key": keyframe.key, "stamp_ns": keyframe.stamp_ns}
        try:
            if len(keyframe.points_lidar) == 0:
                raise ValueError(f"keyframe {keyframe.key} has no points")
            if self.config.use_measured_submaps:
                self._source_frames.append(keyframe)
                self._source_frames = self._source_frames[-self.config.measured_submap.max_frames:]
            if self.config.use_surface_verification:
                from .surfaces import model_surfaces
                model = model_surfaces(transform_points(keyframe.T_odom_body, keyframe.points_body()),
                                       [keyframe.key] * len(keyframe.points_lidar), self.config.surface)
                event["surfaces"] = {"unknown_points": model.unknown_points,
                                     "patches": [{"kind": p.kind, "height_m": p.height_m,
                                                  "support_count": p.support_count}
                                                 for p in model.patches]}
            desc = descriptor(keyframe.points_body(), self.config.scan_context)
            if self.prior is None:
                event.update(self._map_step(keyframe, desc))
            else:
                event.update(self._relocalize_step(keyframe, desc))
        except Exception as error:  # one bad keyframe must not stop the backend
            self.last_error = f"{type(error).__name__}: {error}"
            event["error"] = self.last_error
        event["T_world_odom"] = None if self._correction is None else self._correction.tolist()
        self._record("keyframe", result=event)
        return event

    # Mapping mode -------------------------------------------------------

    def _map_step(self, keyframe: Keyframe, desc: np.ndarray) -> dict:
        self._add_node(keyframe)
        loop_key, rejected = self._find_loop(keyframe, desc)
        self.map.add(keyframe, self.graph.pose(keyframe.key), desc)
        self.index.add(keyframe.key, desc)
        if loop_key is not None:
            self.loop_count += 1
            if self.trace is not None:
                self._record("graph_before", graph=graph_snapshot(self.graph))
            try:
                optimization = self.graph.optimize(self.config.optimize_iterations)
            except Exception as error:
                self._record("optimization_failed", error=f"{type(error).__name__}: {error}")
                raise
            if self.trace is not None:
                self._record("graph_after", graph=graph_snapshot(self.graph), optimization=optimization)
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
            odometry = information(self.config.odometry_sigma_m, self.config.odometry_sigma_rad)
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
        if self.trace is not None:
            self._record("loop_exclusions", excluded=[{
                "candidate_key": key, "recent": key in recent,
                "time_gap": keyframe.stamp_ns - self.map.keyframes[key].stamp_ns < gap_ns,
            } for key in excluded])
        match = self._search(
            self.map, keyframe, desc, lambda key: key in excluded, lambda key: self._loop_submap(key, excluded)
        )
        if match is None:
            self._record("loop_decision", accepted=False, reason="no_verified_candidate")
            return None, None
        key, result = match
        T_world_new = result.T_target_source
        rejected = self._odometry_inconsistency(key, keyframe, T_world_new)
        if rejected is not None:
            self._record("loop_decision", accepted=False, candidate_key=key, reason=rejected)
            return None, rejected
        anchor = key if self.config.use_measured_submaps else min(
            self._loop_window(key, excluded),
            key=lambda k: np.linalg.norm(self.map.poses[k][:3, 3] - T_world_new[:3, 3]),
        )
        loop = information(self.config.loop_sigma_m, self.config.loop_sigma_rad)
        self.graph.add_edge(anchor, keyframe.key, invert(self.graph.pose(anchor)) @ T_world_new, loop, robust=True)
        self._record("loop_decision", accepted=True, candidate_key=key, anchor_key=anchor, T_world_body=T_world_new, information=loop)
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
        if self.config.use_measured_submaps:
            anchor = self.map.keyframes[key]
            frames = [self.map.keyframes[k] for k in self._loop_window(key, excluded)]
            measured = build_submap(frames, anchor, self.config.measured_submap)
            self._target_point_sources = measured.point_sources
            self._record("measured_target", source_keys=measured.source_keys, points=len(measured.points_body))
            return transform_points(self.map.poses[key], measured.points_body)
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
        self._record("search_candidates", query_key=keyframe.key,
                     candidates=[{"candidate_key": k, "distance": d, "yaw_rad": y}
                                 for k, d, y in candidates], icp_limit=self.config.icp_candidates)
        best = None
        source = keyframe.points_body()
        source_keys = [keyframe.key] * len(source)
        if self.config.use_measured_submaps:
            measured = build_submap(self._source_frames, keyframe, self.config.measured_submap)
            source = measured.points_body
            source_keys = measured.point_sources
            self._record("measured_source", source_keys=measured.source_keys, points=len(source))
        attempted, verified = [], []
        for key, dist, yaw in candidates[: self.config.icp_candidates]:
            self._trace_candidate = [keyframe.key, key]
            if dist > self.config.max_scan_context_distance:
                self._record("candidate_skip", candidate_id=self._trace_candidate, reason="scan_context_distance", distance=dist)
                break
            attempted.append(key)
            T_init = store.poses[key] @ yaw_pose(yaw)
            if self.config.use_visual_verification:
                from .visual import verify_visual
                visual = verify_visual(keyframe, store.keyframes[key])
                self._record("visual_verification", candidate_key=key, status=visual.status)
                if visual.status != "verified":
                    continue
                T_init = store.poses[key] @ visual.T_target_source
            self._target_point_sources = None
            target = submap(key)
            result = self._register(source, target, T_init)
            if result is not None and self.config.use_visual_verification:
                translation, rotation = pose_difference(T_init, result.T_target_source)
                if translation > self.config.visual_agree_translation_m or rotation > self.config.visual_agree_rotation_rad:
                    self._record("visual_icp_disagreement", candidate_key=key,
                                 translation_m=translation, rotation_rad=rotation)
                    result = None
            if result is not None and self.config.use_surface_verification:
                from .surfaces import model_surfaces, compare_surfaces
                surfaces = compare_surfaces(
                    model_surfaces(transform_points(result.T_target_source, source), source_keys, self.config.surface),
                    model_surfaces(target, self._target_point_sources, self.config.surface), self.config.surface)
                self._record("surface_verification", candidate_key=key, accepted=surfaces.accepted,
                             reason=surfaces.reason, rmse_m=surfaces.rmse_m)
                if not surfaces.accepted:
                    result = None
            self._record("candidate_verified", candidate_id=self._trace_candidate,
                         accepted=result is not None,
                         reason="icp_verified" if result is not None else "icp_verification_failed")
            if result is not None:
                verified.append(key)
            if result is not None and (best is None or result.rmse_m < best[1].rmse_m):
                best = (key, result)
        self._record("search_outcome", candidates=[{
            "candidate_key": key,
            "status": ("selected_best" if best is not None and key == best[0]
                       else "verified_not_selected" if key in verified
                       else "icp_rejected" if key in attempted
                       else "not_run_icp_limit" if i >= self.config.icp_candidates
                       else "scan_context_distance"),
        } for i, (key, _, _) in enumerate(candidates)])
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
        self._record("icp", candidate_id=self._trace_candidate,
                     T_init=T_init, source_points=len(source_body), target_points=len(target_world),
                     coarse={"T_init": T_init, "T_result": coarse.T_target_source, "fitness": coarse.fitness,
                             "rmse_m": coarse.rmse_m, "converged": coarse.converged},
                     fine={"T_init": coarse.T_target_source, "T_result": fine.T_target_source, "fitness": fine.fitness,
                           "rmse_m": fine.rmse_m, "converged": fine.converged},
                     thresholds={"min_fitness": c.min_fitness, "max_rmse_m": c.max_rmse_m})
        if fine.converged and fine.fitness >= c.min_fitness and fine.rmse_m <= c.max_rmse_m:
            return fine
        return None

    def _propose(self, C: np.ndarray, keyframe: Keyframe) -> str:
        """Offer C to the admission: ``"adopted"``, ``"rejected"`` (jump) or ``"pending"``."""
        rejections = len(self.admission.rejections)
        adopted = self.admission.propose(C, keyframe.T_odom_body, keyframe.stamp_ns)
        status = "adopted" if adopted is not None else (
            "rejected" if len(self.admission.rejections) > rejections else "pending")
        if adopted is not None:
            self._correction = adopted
        self._record("correction", status=status, proposed=C,
                     adopted=self._correction, T_odom_body=keyframe.T_odom_body,
                     rejection=self.admission.rejections[-1] if status == "rejected" else None)
        return status

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
