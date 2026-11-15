"""Point-cloud SLAM backend core: loop closure, prior-map relocalization, map->odom.

Frames: ``odom`` is the frontend odometry frame, ``body`` the estimated body
frame, ``world`` the map frame. The output is the correction
``C = T_world_odom`` so that ``T_world_body = C @ T_odom_body``.

Mapping mode (no prior map): world equals the first keyframe's odom frame and
C starts as identity. Every keyframe becomes a pose-graph node linked to its
predecessor by the odometry delta. ScanContext proposes loop candidates outside
the recent window, each with a yaw guess; coarse-then-fine ICP against the
candidate's world submap verifies them. A verified loop adds a robust edge and
re-optimises the graph; ``C = T_world_body(latest) @ inv(T_odom_body(latest))``
is then offered to ``CorrectionAdmission`` on every keyframe. ``lost_after``
jump rejections in a row reset the admission so a persistent correction is
still adopted.

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
from .keyframes import Keyframe
from .map_store import MapStore
from .pose_graph import PoseGraph, invert
from .registration import IcpResult, icp_point_to_plane
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
        loop_key = self._find_loop(keyframe, desc)
        self.map.add(keyframe, self.graph.pose(keyframe.key), desc)
        self.index.add(keyframe.key, desc)
        if loop_key is not None:
            self.loop_count += 1
            self.graph.optimize(self.config.optimize_iterations)
            self.map.update_poses({key: self.graph.pose(key) for key in self.graph.keys})
        if self._correction is None:
            self._correction = np.eye(4)
        status = self._propose(self.graph.pose(keyframe.key) @ invert(keyframe.T_odom_body), keyframe)
        self._count_failure(status == "rejected")
        return {"loop": loop_key is not None, "loop_key": loop_key, "adopted": status == "adopted"}

    def _add_node(self, keyframe: Keyframe) -> None:
        """Node at the previous optimised pose composed with the odometry delta."""
        previous = self._previous
        if previous is None:
            self.graph.add_node(keyframe.key, keyframe.T_odom_body)
        else:
            delta = invert(previous.T_odom_body) @ keyframe.T_odom_body
            self.graph.add_node(keyframe.key, self.graph.pose(previous.key) @ delta)
            odometry = information(self.config.odometry_sigma_m, self.config.odometry_sigma_rad)
            self.graph.add_edge(previous.key, keyframe.key, delta, odometry)
        self._previous = keyframe

    def _find_loop(self, keyframe: Keyframe, desc: np.ndarray):
        """Add one verified loop edge to the newest keyframe; return its partner key."""
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
            return None
        key, result = match
        T_world_new = result.T_target_source
        anchor = min(
            self._loop_window(key, excluded),
            key=lambda k: np.linalg.norm(self.map.poses[k][:3, 3] - T_world_new[:3, 3]),
        )
        loop = information(self.config.loop_sigma_m, self.config.loop_sigma_rad)
        self.graph.add_edge(anchor, keyframe.key, invert(self.graph.pose(anchor)) @ T_world_new, loop, robust=True)
        return anchor

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
        """World cloud of ``_loop_window``; the loop edge anchors to its keyframe nearest the match."""
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
        """After ``lost_after`` failures in a row, drop tracking and the jump reference.

        A fresh admission lets a persistently proposed correction be confirmed
        again (relocalization: through global search) while the last adopted
        C stays published until then.
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
        best = None
        for key, dist, yaw in candidates[: self.config.icp_candidates]:
            if dist > self.config.max_scan_context_distance:
                break
            T_init = store.poses[key] @ yaw_pose(yaw)
            result = self._register(keyframe.points_body(), submap(key), T_init)
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
