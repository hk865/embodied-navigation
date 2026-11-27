"""Self-updating Habitat-Sim WorldDaemon bridging sensors into layered spatial memory."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np

from embodied_navigation.agent.tools.embodied_tools import _format_agent_candidates
from embodied_navigation.common.geometry import wrap_angle
from embodied_navigation.localization_mapping.semantic_mapping import (
    ROOM_PROTOTYPE_PROMPTS,
    GlobalRegionMemory,
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
    RegionArchiveEntry,
    TextureKeyframeSelector,
    extract_rgbd_surface_voxels,
)
from embodied_navigation.localization_mapping.semantic_mapping.hierarchical_memory import (
    _label_strict_match,
)
from embodied_navigation.perception.vision_tools import (
    GroundedDetection,
    VisionToolClient,
    build_vision_prompts,
    ground_detections_with_depth,
    render_visual_enhancement_overlay,
)
from embodied_navigation.planning.exploration.habitat_memory_runner import (
    DepthOccupancyGrid,
    _pose_matrix_zup,
    hab_quat_to_mem_yaw,
    hab_to_mem,
)


@dataclass(frozen=True)
class MotionFrameIndex:
    """Lightweight trajectory index for high-frequency frames between keyframes."""

    step_seq: int
    xyz_mem: tuple[float, float, float]
    yaw_rad: float
    median_depth_m: float
    zone_id: str
    promoted_kf_id: str | None = None


class HabitatWorldDaemon:
    """Self-updating Habitat perception, occupancy, surface-visibility keyframes, and global region archive."""

    def __init__(
        self,
        *,
        sim: Any,
        output_dir: Path,
        stop_radius_m: float = 1.8,
        vision_client: VisionToolClient | None = None,
        vision_enhance_overlay: bool = True,
        vision_score_threshold: float = 0.25,
        vision_fuse_threshold: float = 0.35,
        vision_max_prompts: int = 5,
        region_summarizer_fn: Callable[
            [RegionArchiveEntry, Sequence[str]], tuple[str | None, str | None] | None
        ]
        | None = None,
    ) -> None:
        self.sim = sim
        self.output_dir = Path(output_dir)
        self.frames_dir = self.output_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.stop_radius_m = float(stop_radius_m)

        self.vision_client = vision_client
        self.vision_health: dict[str, Any] = (
            vision_client.health()
            if vision_client is not None
            else {"ok": False, "backend": "none", "embed_model": ""}
        )
        self.vision_ok = bool(self.vision_health.get("ok"))
        self.vision_backend = str(self.vision_health.get("backend", "none"))
        self.vision_has_seg = self.vision_ok and self.vision_backend != "none"
        self.vision_has_embed = self.vision_ok and bool(
            str(self.vision_health.get("embed_model", "")).strip()
        )
        self.vision_enhance_overlay = bool(vision_enhance_overlay)
        self.vision_score_threshold = float(vision_score_threshold)
        self.vision_fuse_threshold = float(vision_fuse_threshold)
        self.vision_max_prompts = max(1, int(vision_max_prompts))
        self.region_summarizer_fn = region_summarizer_fn

        self.memory = HierarchicalMemoryStore(
            KeyframeSceneGraph(), default_stop_radius_m=self.stop_radius_m
        )
        self.occ_grid = DepthOccupancyGrid()
        self.texture_selector = TextureKeyframeSelector()
        self.region_memory = GlobalRegionMemory(self.memory, self.texture_selector)
        self._init_room_prototypes()
        self.motion_index: list[MotionFrameIndex] = []

        self.current_kf_id: str | None = None
        self.current_raw_rgb_path: Path | None = None
        self.current_vis_rgb_path: Path | None = None
        self.current_floorplan_path: Path | None = None
        self.current_zone_id: str | None = None
        self.current_detections: list[GroundedDetection] = []
        self.latest_disambiguated_ok: bool = False
        self.latest_disambiguated_kf: str | None = None

        self._last_motion_xyz: np.ndarray | None = None
        self._last_motion_yaw: float = 0.0
        self._last_kf_xyz: np.ndarray | None = None
        self._last_kf_yaw: float = 0.0
        self._last_floorplan_xyz: np.ndarray | None = None
        self._last_floorplan_zone: str | None = None

        self.total_vision_dets = 0
        self.total_vision_fused = 0
        self.total_deferred_hits = 0
        self.total_refutations = 0
        self.total_vlm_region_summaries = 0
        self._file_seq = 0
        self._vis_seq = 0
        self._map_seq = 0

    def _init_room_prototypes(self) -> None:
        if self.vision_client is None or not self.vision_has_embed:
            return
        protos: dict[str, tuple[float, ...]] = {}
        for rtype, prompt in ROOM_PROTOTYPE_PROMPTS.items():
            vec = self.vision_client.embed_text(prompt)
            if vec is not None:
                protos[rtype] = vec
        if protos:
            self.region_memory.set_prototype_embeddings(protos)

    def is_within_observed_map(
        self, goal_mem: Sequence[float], *, max_margin_m: float = 1.2
    ) -> bool:
        """True iff ``goal_mem`` lies inside or adjacent to observed cells (including stair-linked floors)."""
        rx, ry, rz = float(goal_mem[0]), float(goal_mem[1]), float(goal_mem[2])
        cell_margin = max(1, int(math.ceil(float(max_margin_m) / self.occ_grid.resolution_m)))
        gx, gy = self.occ_grid._to_cell(rx, ry)
        fk = self.occ_grid._floor_key(rz)
        for dfk in (0, -1, 1, -2, 2):
            grid = self.occ_grid._grids.get(fk + dfk)
            if not grid:
                continue
            for dx in range(-cell_margin, cell_margin + 1):
                for dy in range(-cell_margin, cell_margin + 1):
                    if (gx + dx, gy + dy) in grid:
                        return True
        return False

    def reset_world(self, *, keep_memory: bool = True, keep_occupancy: bool = True) -> None:
        if not keep_memory:
            self.memory = HierarchicalMemoryStore(
                KeyframeSceneGraph(), default_stop_radius_m=self.stop_radius_m
            )
            self.texture_selector = TextureKeyframeSelector()
            self.region_memory = GlobalRegionMemory(self.memory, self.texture_selector)
            self._init_room_prototypes()
            self._last_kf_xyz = None
        if not keep_occupancy:
            self.occ_grid = DepthOccupancyGrid()
            self.motion_index.clear()
            self._last_motion_xyz = None
        self.current_kf_id = None
        self.current_raw_rgb_path = None
        self.current_vis_rgb_path = None
        self.current_floorplan_path = None
        self.current_zone_id = None
        self.current_detections = []
        self.latest_disambiguated_ok = False
        self.latest_disambiguated_kf = None

    def agent_pose(self) -> tuple[list[float], float, np.ndarray]:
        st = self.sim.agents[0].get_state()
        p_hab = np.asarray(st.position, dtype=np.float64)
        q = st.rotation
        yaw_mem = hab_quat_to_mem_yaw((q.w, q.x, q.y, q.z))
        return list(hab_to_mem(p_hab)), yaw_mem, p_hab

    @staticmethod
    def _median_valid_depth(depth_m: np.ndarray) -> float:
        arr = depth_m[:, :, 0] if depth_m.ndim == 3 else depth_m
        h, w = arr.shape[:2]
        band = arr[int(0.25 * h) : int(0.75 * h), int(0.25 * w) : int(0.75 * w)]
        valid = band[np.isfinite(band) & (band >= 0.25) & (band <= 10.0)]
        if valid.size == 0:
            return 3.0
        return float(np.median(valid))

    def _fuse_detections(
        self,
        detections: Sequence[GroundedDetection],
        kf_id: str,
        kf_seq: int,
        *,
        min_score: float,
    ) -> int:
        fused = 0
        for gd in detections:
            if gd.abstain:
                continue
            if (
                gd.range_source == "vision_mask_depth_backproj"
                and gd.score >= float(min_score)
                and gd.valid_pixels >= 30
                and gd.surface_distance_m <= 8.0
            ):
                node, _ = self.memory.add_or_fuse_object(
                    label=gd.label,
                    keyframe_id=kf_id,
                    local_xyz=list(gd.local_xyz),
                    local_extent_xyz=gd.extent_xyz_m,
                    confidence=gd.score,
                    producer=f"{gd.backend}_depth_grounded",
                    range_source=gd.range_source,
                    evidence_frame="habitat_world",
                    evidence_stamp_s=float(kf_seq),
                    embedding=gd.embedding,
                )
                if len(gd.candidate_scores) > 1:
                    meta = self.memory._object_meta.get(node.object_id)
                    if meta is not None:
                        meta.label_scores = dict(gd.candidate_scores)
                fused += 1
                self.total_vision_fused += 1
        return fused

    def _refresh_recent_keyframe_clearances(self, max_recent: int = 8) -> None:
        """Re-estimate 360-deg occupancy clearance for recent keyframes as walls are revealed."""
        kf_ids = list(self.memory.graph.keyframe_poses.keys())[-max(1, int(max_recent)) :]
        for kid in kf_ids:
            pose = self.memory.graph.keyframe_poses.get(kid)
            if pose is None:
                continue
            p_xyz = [float(pose[0, 3]), float(pose[1, 3]), float(pose[2, 3])]
            clr_m, sec_clr = self.occ_grid.estimate_clearance_profile(
                p_xyz, num_sectors=16, max_range_m=4.5
            )
            self.memory.graph.keyframe_clearances[kid] = clr_m
            self.memory.graph.keyframe_sector_clearances[kid] = sec_clr

    def ingest_sensor_frame(
        self,
        *,
        force_keyframe: bool = False,
        run_baseline_vision: bool = False,
        active_target: str = "",
    ) -> dict[str, Any]:
        """Synchronize occupancy, surface-visibility keyframes, real clearance topology, and region summaries."""
        from PIL import Image

        robot_mem, yaw_rad, _ = self.agent_pose()
        obs = self.sim.get_sensor_observations()
        rgb = obs["color_sensor"][:, :, :3]
        depth = obs["depth_sensor"]
        self.occ_grid.update(depth, robot_mem, yaw_rad)

        pos_arr = np.asarray(robot_mem, dtype=np.float64)
        med_depth = self._median_valid_depth(depth)
        seq = self.memory.allocate_step_seq()
        prev_zone = self.current_zone_id
        fallback_zone_id = self.memory.record_zone_observation(
            robot_mem, median_depth_m=med_depth, step_seq=seq
        )

        # Surface-visibility texture coverage admission (replaces naive fixed-distance/angle triggers)
        T_world_body = _pose_matrix_zup(robot_mem, yaw_rad)
        vox_q = extract_rgbd_surface_voxels(
            depth,
            T_world_body,
            voxel_m=self.texture_selector.voxel_m,
            hfov_deg=90.0,
            camera_height_m=self.occ_grid.camera_height_m,
            stride=8,
        )
        tex_dec = self.texture_selector.evaluate_voxels(vox_q, force=False)
        promote = bool(force_keyframe or tex_dec.admit_texture)

        clr_m, sec_clr = self.occ_grid.estimate_clearance_profile(
            robot_mem, num_sectors=16, max_range_m=4.5
        )
        promoted_kf_id: str | None = None
        if promote or (run_baseline_vision and self.current_kf_id is None):
            self._file_seq += 1
            kf_id = f"kf_{seq:04d}"
            raw_path = self.frames_dir / f"frame_{self._file_seq:05d}_{kf_id}_rgb.png"
            Image.fromarray(rgb).save(raw_path)
            self.memory.graph.add_keyframe(
                kf_id,
                T_world_body,
                clearance_m=clr_m,
                rgb_path=str(raw_path),
                sector_clearances=sec_clr,
            )
            self.region_memory.record_keyframe_depth(kf_id, med_depth)
            # Admit to minimal texture keyframe candidate pool whenever surface visibility gained
            if tex_dec.admit_texture or (not self.texture_selector.records and len(vox_q) >= 8):
                self.texture_selector.commit_candidate(
                    kf_id,
                    vox_q,
                    pose_xyz=robot_mem,
                    region_hint=prev_zone or "",
                )
            if self.vision_client is not None and self.vision_has_embed:
                emb = self.vision_client.embed_image(rgb)
                if emb is not None:
                    self.memory.annotate_keyframe(kf_id, embedding=emb, wall_s=float(seq))
            self.current_kf_id = kf_id
            self.current_raw_rgb_path = raw_path
            self._last_kf_xyz = pos_arr.copy()
            self._last_kf_yaw = float(yaw_rad)
            promoted_kf_id = kf_id

        # Refresh occupancy clearances of recent keyframes so newly mapped walls expose doorways cleanly
        if promoted_kf_id is not None or run_baseline_vision:
            self._refresh_recent_keyframe_clearances(max_recent=8)

        # Record lightweight motion index for frames between texture keyframes
        motion_dist = (
            float("inf")
            if self._last_motion_xyz is None
            else float(np.linalg.norm(pos_arr - self._last_motion_xyz))
        )
        motion_yaw = (
            float("inf")
            if self._last_motion_xyz is None
            else abs(float(wrap_angle(yaw_rad - self._last_motion_yaw)))
        )
        if promoted_kf_id is not None or motion_dist >= 0.65 or motion_yaw >= math.radians(30.0):
            self.motion_index.append(
                MotionFrameIndex(
                    step_seq=seq,
                    xyz_mem=(round(robot_mem[0], 3), round(robot_mem[1], 3), round(robot_mem[2], 3)),
                    yaw_rad=round(yaw_rad, 3),
                    median_depth_m=round(med_depth, 2),
                    zone_id=self.current_zone_id or fallback_zone_id,
                    promoted_kf_id=promoted_kf_id,
                )
            )
            self._last_motion_xyz = pos_arr.copy()
            self._last_motion_yaw = float(yaw_rad)

        if run_baseline_vision:
            self.latest_disambiguated_ok = False
            self.latest_disambiguated_kf = None
            grounded_vdets: list[GroundedDetection] = []
            vis_path: Path | None = self.current_raw_rgb_path
            if (
                self.vision_client is not None
                and self.vision_has_seg
                and self.current_kf_id is not None
            ):
                prompts = build_vision_prompts(
                    active_target or "chair", max_prompts=self.vision_max_prompts
                )
                raw_dets = self.vision_client.segment_text(
                    rgb,
                    prompts,
                    score_threshold=self.vision_score_threshold,
                    max_detections=10,
                    include_masks=True,
                    include_crop_embeddings=self.vision_has_embed,
                )
                grounded_vdets = ground_detections_with_depth(raw_dets, depth)
                self.total_vision_dets += len(grounded_vdets)
                self._fuse_detections(
                    grounded_vdets,
                    self.current_kf_id,
                    seq,
                    min_score=self.vision_fuse_threshold,
                )
                if self.vision_enhance_overlay and grounded_vdets:
                    self._vis_seq += 1
                    vis_path = render_visual_enhancement_overlay(
                        rgb,
                        grounded_vdets,
                        self.frames_dir / f"vis_{self._vis_seq:04d}.png",
                        target_label=active_target,
                    )
            self.current_detections = grounded_vdets
            self.current_vis_rgb_path = vis_path

        # Sync geometric+semantic region archive, delayed Set-Cover pruning, and event-driven VLM summaries
        frontiers = self.occ_grid.extract_frontiers(robot_mem, max_candidates=6)
        reg_sync = self.region_memory.sync_from_scene_graph(
            robot_mem,
            active_frontiers=frontiers,
            summarizer_fn=(self.region_summarizer_fn if run_baseline_vision else None),
        )
        self.total_vlm_region_summaries += len(reg_sync.get("summarized_regions") or ())
        active_rid = reg_sync.get("current_region_id") or fallback_zone_id
        self.current_zone_id = active_rid
        cur_reg = self.region_memory.regions.get(str(active_rid))
        cur_reg_label = cur_reg.primary_label if cur_reg is not None else "open_area"

        return {
            "seq": seq,
            "robot_mem": [round(v, 3) for v in robot_mem],
            "yaw_deg": round(math.degrees(yaw_rad), 1),
            "zone_id": active_rid,
            "region_label": cur_reg_label,
            "zone_changed": bool(reg_sync.get("region_changed") or (prev_zone != active_rid)),
            "clearance_m": clr_m,
            "texture_admitted": tex_dec.admit_texture,
            "texture_reason": tex_dec.reason,
            "keyframe_id": self.current_kf_id,
            "promoted_keyframe": promoted_kf_id,
        }

    def current_candidates(self, target_label: str) -> list[dict[str, Any]]:
        robot_mem, _, _ = self.agent_pose()
        f_cands = self.occ_grid.extract_frontiers(robot_mem, max_candidates=6)
        s_cands = self.occ_grid.extract_stair_portals(robot_mem, max_portals=3)
        m_cands = (
            self.memory.memory_goal_candidates(
                robot_mem,
                target_label=target_label,
                standoff_m=min(1.0, self.stop_radius_m * 0.5),
            )
            if target_label
            else []
        )
        allowed, _blocked = self.memory.filter_candidates(
            [*m_cands, *s_cands, *f_cands], default_z=robot_mem[2]
        )
        return _format_agent_candidates(allowed)

    def render_floorplan_if_needed(
        self, target_label: str, *, force: bool = False
    ) -> Path | None:
        robot_mem, yaw_rad, _ = self.agent_pose()
        pos_arr = np.asarray(robot_mem, dtype=np.float64)
        moved = (
            float("inf")
            if self._last_floorplan_xyz is None
            else float(np.linalg.norm(pos_arr - self._last_floorplan_xyz))
        )
        zone_changed = self._last_floorplan_zone != self.current_zone_id
        if not force and not zone_changed and moved < 0.8:
            return None
        self._map_seq += 1
        cands = self.current_candidates(target_label)
        zones = (
            self.region_memory.region_summaries(robot_mem)
            if self.region_memory.regions
            else self.memory.zone_coverage_summary(robot_mem)
        )
        objs = self.memory.l2_submap_entities(include_stale=False)["objects"]
        out_path = self.occ_grid.render_global_floorplan(
            self.frames_dir / f"floorplan_{self._map_seq:04d}.png",
            robot_mem,
            yaw_rad,
            candidates=cands,
            zones=zones,
            objects=objs,
        )
        self._last_floorplan_xyz = pos_arr.copy()
        self._last_floorplan_zone = self.current_zone_id
        self.current_floorplan_path = out_path
        return out_path

    def run_contrastive_vision(
        self,
        prompts: Sequence[str],
        *,
        active_target: str = "",
        mode: str = "disambiguate",
    ) -> tuple[list[GroundedDetection], Path | None]:
        """Run agent-chosen open-vocabulary prompts with multi-label clustering & abstain gate."""
        if (
            self.vision_client is None
            or not self.vision_has_seg
            or self.current_kf_id is None
        ):
            return [], self.current_vis_rgb_path

        clean_prompts = [
            " ".join(str(p).strip().lower().split())
            for p in prompts
            if str(p or "").strip()
        ][:12]
        if not clean_prompts:
            return [], self.current_vis_rgb_path

        obs = self.sim.get_sensor_observations()
        rgb = obs["color_sensor"][:, :, :3]
        depth = obs["depth_sensor"]
        low_thr = 0.15 if mode == "disambiguate" else self.vision_score_threshold
        raw_dets = self.vision_client.segment_text(
            rgb,
            clean_prompts,
            score_threshold=low_thr,
            max_detections=16,
            include_masks=True,
            include_crop_embeddings=self.vision_has_embed,
        )
        grounded = ground_detections_with_depth(
            raw_dets,
            depth,
            multilabel_cluster=(mode == "disambiguate"),
            abstain_threshold=self.vision_score_threshold,
        )
        self.total_vision_dets += len(grounded)
        seq = self.memory.allocate_step_seq()
        self._fuse_detections(
            grounded,
            self.current_kf_id,
            seq,
            min_score=self.vision_score_threshold,
        )
        vis_path = self.current_raw_rgb_path
        if self.vision_enhance_overlay and grounded:
            self._vis_seq += 1
            vis_path = render_visual_enhancement_overlay(
                rgb,
                grounded,
                self.frames_dir / f"vis_{self._vis_seq:04d}.png",
                target_label=active_target,
            )
        self.current_detections = grounded
        self.current_vis_rgb_path = vis_path
        if active_target:
            matched_ok = any(
                not g.abstain
                and (
                    _label_strict_match(active_target, g.label)
                    or any(
                        _label_strict_match(active_target, clbl)
                        and sc >= self.vision_score_threshold
                        for clbl, sc in g.candidate_scores.items()
                    )
                )
                for g in grounded
            )
            if matched_ok:
                self.latest_disambiguated_ok = True
                self.latest_disambiguated_kf = self.current_kf_id
        return grounded, vis_path

    def deferred_scan_keyframes(
        self,
        query: str,
        top_keyframes: Sequence[dict[str, Any]],
        *,
        max_kfs: int = 2,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        """Scan Top-K historical clean L3 keyframes locally with zero VLM token overhead."""
        if self.vision_client is None or not self.vision_has_seg:
            return [], []
        out_root = self.output_dir.resolve()
        hits: list[dict[str, Any]] = []
        hit_rgbs: list[str] = []
        scanned = 0
        for kf in top_keyframes:
            if scanned >= max(1, int(max_kfs)):
                break
            rp = kf.get("rgb_path")
            if not isinstance(rp, str) or not rp:
                continue
            try:
                resolved = Path(rp).resolve()
                resolved.relative_to(out_root)
            except (OSError, ValueError):
                continue
            if not resolved.is_file():
                continue
            scanned += 1
            dets = self.vision_client.segment_text(
                resolved,
                [query],
                score_threshold=self.vision_fuse_threshold,
                max_detections=3,
                include_masks=False,
            )
            if not dets:
                continue
            best = max(dets, key=lambda d: float(d.score))
            hits.append(
                {
                    "keyframe_id": str(kf.get("keyframe_id", "")),
                    "pose_xyz": kf.get("pose_xyz"),
                    "label": best.label,
                    "score": round(float(best.score), 2),
                    "box_px": [int(round(v)) for v in best.box_xyxy],
                    "backend": best.backend,
                }
            )
            if str(resolved) not in hit_rgbs:
                hit_rgbs.append(str(resolved))
        self.total_deferred_hits += len(hits)
        return hits, hit_rgbs

    def milestone_summary(self, active_target: str) -> str:
        """Build a compact global semantic-region milestone for lazy context compaction."""
        region_digest = self.region_memory.format_global_digest(max_regions=10)
        cands = self.current_candidates(active_target)
        refuted = [
            f"{oid}:was={meta.refuted_labels}"
            for oid, meta in self.memory._object_meta.items()
            if meta.epistemic_status == "refuted"
        ]
        tex_m = self.texture_selector.diagnostic_metrics()
        return (
            f"Active goal: {active_target or 'unspecified'} | map_rev={self.memory.map_revision} | "
            f"texture_kfs={tex_m['retained_texture_keyframes']}/{tex_m['total_candidates_admitted']} "
            f"({tex_m['global_textured_voxels']} voxels)\n"
            f"Global Region Memory:\n{region_digest}\n"
            f"Current candidates: {[c.get('id') for c in cands]}\n"
            f"Refuted objects: {refuted[:8]}"
        )
