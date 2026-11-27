"""Decoupled Habitat-Sim World Daemon & Task-Agnostic EmbodiedAgentRuntime Adapter.

Separates:
1. ``HabitatWorldDaemon`` — self-updating world/perception/mapping subsystem
   (occupancy grid, motion-frame index, dual-trigger keyframes, Zone coverage,
   3D object fusion, 2.5D semantic floorplan rendering, SigLIP & SAM 3 grounding).
2. Tool handlers + pluggable ``BeforeFinishHook`` verifiers (`ObjectNavFinishVerifier`,
   `ReportFinishVerifier`) registered onto ``EmbodiedAgentRuntime``.

Strictly preserves zero ground-truth leakage: ``semantic_sensor`` is touched ONLY
by offline evaluation setup/scoring and is never exposed to tools, images, or prompts.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Any, Callable, Sequence

import numpy as np

from embodied_navigation.agent_runtime import (
    AgentRunResult,
    EmbodiedAgentRuntime,
    FinishCheckResult,
    ImageAttachment,
    OpenCodeToolChatClient,
    PrefixStableContextManager,
    ToolCall,
    ToolResult,
    ToolSpec,
)
from embodied_navigation.common.geometry import wrap_angle
from embodied_navigation.common.runtime_io import atomic_write_json
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
from embodied_navigation.planning.exploration.llm_agent_client import (
    LlmClientConfig,
    _encode_image_part,
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
    HabitatObjectGT,
    HabitatSubtaskSpec,
    _pose_matrix_zup,
    build_private_hm3d_gt,
    compute_spl,
    depth_bearing_to_local_xy,
    hab_quat_to_mem_yaw,
    hab_to_mem,
    is_geodesic_target_reached,
    measure_depth_bearing_patch,
    mem_to_hab,
)


def _format_agent_candidates(candidates: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    compact: list[dict[str, Any]] = []
    for c in list(candidates)[:12]:
        if not isinstance(c, dict) or not c.get("id"):
            continue
        row: dict[str, Any] = {
            "id": c["id"],
            "kind": c.get("kind"),
            "xy": c.get("xy"),
            "z": c.get("z"),
            "dist_m": c.get("dist_m"),
        }
        if c.get("label"):
            row["label"] = c["label"]
        if c.get("object_id"):
            row["object_id"] = c["object_id"]
        if c.get("description"):
            row["description"] = c["description"]
        if c.get("observation_count") is not None:
            row["obs"] = c["observation_count"]
        if c.get("blacklisted"):
            row["blacklisted"] = True
        compact.append(row)
    return compact

AGENT_RUNTIME_SYSTEM_PROMPT = (
    "## [ROLE]\n"
    "You are an autonomous embodied indoor robot powered by a persistent multi-turn memory "
    "runtime, a self-updating 3D world model, and on-demand local vision/spatial tools.\n"
    "Maintain coherent multi-turn reasoning: remember which rooms/zones you entered, what "
    "stairwells or doorways you saw on earlier frames, and hypotheses you already tested.\n\n"
    "## [OBSERVATIONS]\n"
    "Attached images when present:\n"
    "- Front RGB (when local vision segmentation is active, colored masks/boxes + metric "
    "distances like '[V1] bed 1.1m (0.89)' are overlaid directly on the image).\n"
    "- Global 2.5D Floorplan (when moved between regions or requested via query_world): shows "
    "explored floor free/occupied geometry, robot trail, geometric+semantic Region IDs "
    "(R1*:living_room, R2~:kitchen, R3*:hallway) with doorway/passage links, remembered "
    "landmarks, M* memory targets, S* staircase portals (magenta diamonds), and F* unexplored frontiers.\n"
    "- Global Region Memory: structured index of explored regions with primary_label, "
    "detailed_description, connected_regions, and Set-Cover minimal texture_keyframe_ids "
    "(inspect any region visually via get_evidence(region_id='R1')).\n\n"
    "## [COGNITIVE & TOOL DISCIPLINE]\n"
    "1. Exploration budget is strictly measured in Habitat atomic physical steps (500 per subtask, "
    "shown in the status footer); information tools (query_world, inspect_vision, get_evidence, "
    "update_belief) cost 0 atomic steps.\n"
    "2. Explore systematically via navigate_to(target_id='F*') toward unexplored doorways and "
    "glimpsed zones (avoid repeatedly revisiting zones marked 'inspected_closeup').\n"
    "3. Multi-floor traversal: if you see a staircase leading up or down in RGB or an S* stair "
    "portal on the floorplan, and the current floor lacks the target room/object, call "
    "navigate_to(target_id='S1') or traverse_stairs(direction='down'|'up') to move onto the "
    "adjacent floor! Do NOT stay trapped on one floor.\n"
    "4. Check memory first via query_world(query=...) or M* candidates when switching tasks "
    "or wondering if an area was already inspected.\n"
    "5. Contrastive disambiguation BEFORE trusting a confusable object (e.g. table vs desk vs "
    "nightstand vs dresser vs cabinet vs stair railing): call inspect_vision(prompts=[...], "
    "mode='disambiguate') with BOTH the target category AND plausible confusors! For 'table', "
    "target genuine tables (dining table, coffee table, desk, kitchen/office table) — do NOT "
    "stop at bedroom nightstands, dressers, cabinets, vanity units, or kitchen counters.\n"
    "6. Active belief revision: if you approach or inspect an M* candidate and see it is NOT the "
    "target (e.g. clothing/bedskirt mistaken for chair, or nightstand/stair rail mistaken for "
    "table), call update_belief(object_id=..., status='refuted', new_label=..., note=...) "
    "IMMEDIATELY so it is purged from M* candidates.\n"
    "7. Zero-step guard: if navigate_to reports already_at_goal=true (0 steps), do NOT repeat "
    "the same navigate_to; inspect/refute/finish_task or choose a different F*/S*.\n"
    "8. Always invoke registered tools to act or answer; never hallucinate coordinates."
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


def _build_vlm_region_summarizer(
    base_cfg: LlmClientConfig,
    output_root: Path,
    *,
    summary_model: str = "qwen3.8-flash",
) -> Callable[[RegionArchiveEntry, Sequence[str]], tuple[str | None, str | None] | None] | None:
    """Create a fast non-thinking background VLM region summarizer for event-driven archive updates."""
    if not base_cfg.provider or base_cfg.provider == "mock":
        return None
    fast_cfg = LlmClientConfig(
        provider=base_cfg.provider,
        base_url=base_cfg.base_url,
        model=summary_model if base_cfg.provider == "opencode" else base_cfg.model,
        api_key=base_cfg.api_key,
        timeout_s=22.0,
        max_tokens=220,
        temperature=0.1,
        supports_vision=True,
        stream=False,
        extra_body={"enable_thinking": False} if "qwen" in summary_model.lower() else None,
    )
    if not fast_cfg.api_key:
        return None
    client = OpenCodeToolChatClient(fast_cfg)
    root_resolved = output_root.resolve()

    def _summarize(
        region: RegionArchiveEntry, rgb_paths: Sequence[str]
    ) -> tuple[str | None, str | None] | None:
        import json as _json

        parts: list[dict[str, Any]] = [
            {
                "type": "text",
                "text": (
                    "Summarize this indoor room/region from its minimal Set-Cover keyframes.\n"
                    f"Region ID: {region.region_id} | Floor: {region.floor_id} | "
                    f"Initial label: {region.primary_label} | Clearance: {region.clearance_m:.2f}m\n"
                    f"Detected objects: {list(region.objects[:8])}\n"
                    f"Connected regions: {[c['to'] + '(' + c['to_label'] + ') via ' + c['via'] for c in region.connected_regions[:5]]}\n"
                    "Return ONLY strict JSON: "
                    '{"primary_label": "living_room|bedroom|kitchen|dining_room|bathroom|office|hallway|stairwell|lobby|closet|laundry", '
                    '"detailed_description": "2 concise factual sentences describing layout, key furniture/landmarks, visual style, and visible doorways/exits."}'
                ),
            }
        ]
        attached = 0
        for rp in rgb_paths[:3]:
            try:
                resolved = Path(rp).resolve()
                resolved.relative_to(root_resolved)
            except (OSError, ValueError):
                continue
            if not resolved.is_file():
                continue
            part = _encode_image_part(resolved, allowed_roots=[root_resolved])
            if part is not None:
                parts.append(part)
                attached += 1
        if attached == 0:
            return None
        turn = client.complete_turn(
            [
                {
                    "role": "system",
                    "content": "You are a concise spatial-memory indexer. Output ONLY valid JSON.",
                },
                {"role": "user", "content": parts},
            ],
            tools=(),
        )
        if turn.fallback or not turn.content.strip():
            return None
        txt = turn.content.strip()
        start = txt.find("{")
        end = txt.rfind("}")
        if start < 0 or end <= start:
            return None
        try:
            parsed = _json.loads(txt[start : end + 1])
        except ValueError:
            return None
        if not isinstance(parsed, dict):
            return None
        lbl = str(parsed.get("primary_label") or "").strip() or None
        desc = str(parsed.get("detailed_description") or "").strip() or None
        return (lbl, desc) if (lbl or desc) else None

    return _summarize


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


def build_agent_tools(
    *,
    daemon: HabitatWorldDaemon,
    active_target_getter: Callable[[], str],
    step_goal_fn: Callable[[Sequence[float]], dict[str, Any]],
    step_primitives_fn: Callable[[str, int], dict[str, Any]],
    step_stairs_fn: Callable[[str, Sequence[float] | None], dict[str, Any]] | None = None,
    record_zone_annotation_fn: Callable[[str, str], None] | None = None,
) -> list[tuple[ToolSpec, Callable[[ToolCall], ToolResult]]]:
    """Assemble generic embodied tool specs + handlers around ``HabitatWorldDaemon``."""
    out_root = daemon.output_dir.resolve()

    def _obs_images(include_floorplan: bool, target_label: str) -> tuple[ImageAttachment, ...]:
        imgs: list[ImageAttachment] = []
        if daemon.current_vis_rgb_path is not None and daemon.current_vis_rgb_path.is_file():
            imgs.append(ImageAttachment(daemon.current_vis_rgb_path, "front RGB camera"))
        elif daemon.current_raw_rgb_path is not None and daemon.current_raw_rgb_path.is_file():
            imgs.append(ImageAttachment(daemon.current_raw_rgb_path, "front RGB camera"))
        if include_floorplan:
            fp = daemon.render_floorplan_if_needed(target_label, force=False)
            if fp is not None and fp.is_file():
                imgs.append(ImageAttachment(fp, "global 2.5D floorplan"))
        return tuple(imgs)

    def _handle_navigate_to(call: ToolCall) -> ToolResult:
        target_label = active_target_getter()
        robot_mem, yaw_rad, _ = daemon.agent_pose()
        tid = call.arguments.get("target_id")
        raw_xyz = call.arguments.get("xyz")
        goal_mem: list[float] | None = None

        cands = daemon.current_candidates(target_label)
        selected_stair_dir: str | None = None
        selected_stair_xy: list[float] | None = None
        if isinstance(tid, str) and tid.strip():
            for c in cands:
                if c.get("id") == tid.strip():
                    xy = c.get("xy")
                    if isinstance(xy, (list, tuple)) and len(xy) >= 2:
                        goal_mem = [float(xy[0]), float(xy[1]), float(c.get("z", robot_mem[2]))]
                        kind = str(c.get("kind") or "")
                        if kind in ("stair_down", "stair_up"):
                            selected_stair_dir = "down" if kind == "stair_down" else "up"
                            selected_stair_xy = [float(xy[0]), float(xy[1])]
                    break
        elif isinstance(raw_xyz, (list, tuple)) and len(raw_xyz) == 3:
            try:
                vals = [float(v) for v in raw_xyz]
                if all(math.isfinite(v) for v in vals):
                    if not daemon.is_within_observed_map(vals, max_margin_m=1.2):
                        return ToolResult(
                            call_id=call.call_id,
                            name=call.name,
                            payload={
                                "status": "outside_observed_map",
                                "reason": "xyz_not_in_explored_region",
                                "available_candidates": cands,
                            },
                            is_error=False,
                        )
                    goal_mem = vals
            except (TypeError, ValueError):
                goal_mem = None

        if goal_mem is None:
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={
                    "status": "error",
                    "reason": "unresolved_target_id_or_xyz",
                    "available_candidates": cands,
                },
                is_error=True,
            )

        if daemon.memory.match_blacklist(goal_mem):
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={
                    "status": "blocked_by_blacklist",
                    "available_candidates": cands,
                },
                is_error=False,
            )

        if selected_stair_dir is not None and step_stairs_fn is not None:
            exec_info = step_stairs_fn(selected_stair_dir, selected_stair_xy)
        else:
            exec_info = step_goal_fn(goal_mem)
        daemon.memory.record_attempt(
            "goto",
            goal_mem,
            str(exec_info.get("status", "unknown")),
            reason=str(exec_info.get("reason", "")),
            keyframe_id=daemon.current_kf_id,
        )
        ing = daemon.ingest_sensor_frame(
            force_keyframe=True,
            run_baseline_vision=True,
            active_target=target_label,
        )
        new_cands = daemon.current_candidates(target_label)
        zero_step = (
            exec_info.get("status") == "completed"
            and int(exec_info.get("primitives", 0)) == 0
            and float(exec_info.get("traveled_m", 0.0)) < 0.05
        )
        payload: dict[str, Any] = {
            **exec_info,
            "robot_xyz": ing["robot_mem"],
            "yaw_deg": ing["yaw_deg"],
            "region_id": ing["zone_id"],
            "region_label": ing.get("region_label", "open_area"),
            "keyframe_id": ing["keyframe_id"],
            "map_revision": daemon.memory.map_revision,
            "candidates": new_cands,
            "detections": [g.to_prompt_dict() for g in daemon.current_detections[:8]],
        }
        if ing.get("zone_changed"):
            payload["global_regions_digest"] = daemon.region_memory.format_global_digest()
        if zero_step:
            payload["already_at_goal"] = True
            payload["hint"] = (
                "Already at goal (0 steps). Do NOT repeat this navigate_to; inspect/finish_task "
                "if visible or choose another frontier F*."
            )
        return ToolResult(
            call_id=call.call_id,
            name=call.name,
            payload=payload,
            images=_obs_images(include_floorplan=True, target_label=target_label),
        )

    def _handle_scan_surroundings(call: ToolCall) -> ToolResult:
        target_label = active_target_getter()
        direction = str(call.arguments.get("direction", "left")).strip().lower()
        act = "turn_right" if direction == "right" else "turn_left"
        try:
            angle = float(call.arguments.get("angle_deg", 60.0))
        except (TypeError, ValueError):
            angle = 60.0
        steps = max(1, min(8, int(round(abs(angle) / 15.0))))
        exec_info = step_primitives_fn(act, steps)
        ing = daemon.ingest_sensor_frame(
            force_keyframe=True,
            run_baseline_vision=True,
            active_target=target_label,
        )
        return ToolResult(
            call_id=call.call_id,
            name=call.name,
            payload={
                **exec_info,
                "robot_xyz": ing["robot_mem"],
                "yaw_deg": ing["yaw_deg"],
                "zone_id": ing["zone_id"],
                "keyframe_id": ing["keyframe_id"],
                "candidates": daemon.current_candidates(target_label),
                "detections": [g.to_prompt_dict() for g in daemon.current_detections[:8]],
            },
            images=_obs_images(include_floorplan=bool(ing["zone_changed"]), target_label=target_label),
        )

    def _handle_maneuver(call: ToolCall) -> ToolResult:
        target_label = active_target_getter()
        motion_raw = call.arguments.get("motion")
        steps_raw = call.arguments.get("steps")
        if not isinstance(motion_raw, str) or motion_raw.strip().lower() not in (
            "forward",
            "left",
            "right",
            "turn_left",
            "turn_right",
        ):
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"error": "invalid_or_missing_motion"},
                is_error=True,
            )
        try:
            if steps_raw is None or isinstance(steps_raw, bool):
                raise ValueError("missing_steps")
            steps = max(1, min(16, int(steps_raw)))
        except (TypeError, ValueError):
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"error": "invalid_or_missing_steps"},
                is_error=True,
            )
        motion = motion_raw.strip().lower()
        act = "move_forward"
        if motion in ("left", "turn_left"):
            act = "turn_left"
        elif motion in ("right", "turn_right"):
            act = "turn_right"
        exec_info = step_primitives_fn(act, steps)
        # Always bind maneuver completion to a fresh keyframe ID so old-view masks cannot vouch
        ing = daemon.ingest_sensor_frame(
            force_keyframe=True,
            run_baseline_vision=True,
            active_target=target_label,
        )
        return ToolResult(
            call_id=call.call_id,
            name=call.name,
            payload={
                **exec_info,
                "robot_xyz": ing["robot_mem"],
                "yaw_deg": ing["yaw_deg"],
                "zone_id": ing["zone_id"],
                "keyframe_id": ing["keyframe_id"],
                "candidates": daemon.current_candidates(target_label),
                "detections": [g.to_prompt_dict() for g in daemon.current_detections[:8]],
            },
            images=_obs_images(include_floorplan=False, target_label=target_label),
        )

    def _handle_inspect_vision(call: ToolCall) -> ToolResult:
        target_label = active_target_getter()
        raw_prompts = call.arguments.get("prompts") or [target_label or "chair"]
        if not isinstance(raw_prompts, (list, tuple)):
            raw_prompts = [str(raw_prompts)]
        mode = str(call.arguments.get("mode", "disambiguate")).strip().lower()
        mode = "segment" if mode == "segment" else "disambiguate"
        grounded, vis_path = daemon.run_contrastive_vision(
            [str(p) for p in raw_prompts],
            active_target=target_label,
            mode=mode,
        )
        imgs: tuple[ImageAttachment, ...] = ()
        if vis_path is not None and vis_path.is_file():
            imgs = (ImageAttachment(vis_path, "contrastive vision overlay"),)
        return ToolResult(
            call_id=call.call_id,
            name=call.name,
            payload={
                "backend": daemon.vision_backend,
                "mode": mode,
                "keyframe_id": daemon.current_kf_id,
                "detections": [g.to_prompt_dict() for g in grounded[:10]],
                "candidates": daemon.current_candidates(target_label),
            },
            images=imgs,
        )

    def _handle_query_world(call: ToolCall) -> ToolResult:
        target_label = active_target_getter()
        q = str(call.arguments.get("query") or target_label or "").strip()
        include_fp = bool(call.arguments.get("include_floorplan", False))
        robot_mem, _, _ = daemon.agent_pose()
        q_emb = (
            daemon.vision_client.embed_text(f"a photo of a {q}")
            if (daemon.vision_client is not None and daemon.vision_has_embed and q)
            else None
        )
        c2f = daemon.memory.coarse_to_fine_search(
            q,
            robot_xyz=robot_mem,
            query_embedding=q_emb,
            top_k_objects=5,
            top_k_keyframes=3,
            max_l3_images=2,
        )
        def_hits, def_rgbs = daemon.deferred_scan_keyframes(q, c2f["top_keyframes"], max_kfs=2)
        safe_objs = [
            {k: v for k, v in o.items() if k != "rgb_path"}
            for o in c2f["top_objects"]
        ]
        safe_kfs = [
            {k: v for k, v in kf.items() if k != "rgb_path"}
            for kf in c2f["top_keyframes"]
        ]
        imgs: list[ImageAttachment] = []
        if include_fp:
            fp = daemon.render_floorplan_if_needed(target_label, force=True)
            if fp is not None and fp.is_file():
                imgs.append(ImageAttachment(fp, "global 2.5D floorplan"))
        if c2f["needs_l3_verification"] or def_rgbs:
            merged_l3 = list(dict.fromkeys([*def_rgbs, *c2f["l3_verify_rgb_paths"]]))[:2]
            for rp in merged_l3:
                try:
                    resolved = Path(rp).resolve()
                    resolved.relative_to(out_root)
                except (OSError, ValueError):
                    continue
                if resolved.is_file():
                    imgs.append(ImageAttachment(resolved, f"recalled L3 frame ({resolved.stem})"))
        return ToolResult(
            call_id=call.call_id,
            name=call.name,
            payload={
                "query": q,
                "map_revision": c2f["map_revision"],
                "top1_prob": c2f["top1_prob"],
                "confidence_margin": c2f["confidence_margin"],
                "needs_l3_verification": c2f["needs_l3_verification"],
                "top_objects": safe_objs,
                "top_keyframes": safe_kfs,
                "deferred_vision_hits": def_hits,
                "matched_regions": daemon.region_memory.search_regions(q, top_k=4),
                "global_regions": daemon.region_memory.region_summaries(robot_mem),
                "zone_coverage": daemon.region_memory.region_summaries(robot_mem) or c2f["zone_coverage"],
                "candidates": daemon.current_candidates(target_label),
            },
            images=tuple(imgs),
        )

    def _handle_get_evidence(call: ToolCall) -> ToolResult:
        kf_id = str(call.arguments.get("keyframe_id", "")).strip()
        reg_id = str(call.arguments.get("region_id", "")).strip()
        if reg_id and not kf_id:
            reg = daemon.region_memory.regions.get(reg_id)
            if reg is None:
                return ToolResult(
                    call_id=call.call_id,
                    name=call.name,
                    payload={
                        "error": "unknown_region_id",
                        "region_id": reg_id,
                        "available_regions": list(daemon.region_memory.regions.keys()),
                    },
                    is_error=True,
                )
            imgs: list[ImageAttachment] = []
            attached_kfs: list[str] = []
            for kid in reg.texture_keyframe_ids[:3]:
                rp_k = daemon.memory.graph.keyframe_rgb_paths.get(kid)
                if not rp_k:
                    continue
                try:
                    resolved_k = Path(rp_k).resolve()
                    resolved_k.relative_to(out_root)
                except (OSError, ValueError):
                    continue
                if resolved_k.is_file():
                    imgs.append(ImageAttachment(resolved_k, f"{reg.region_id}({reg.primary_label}) texture KF {kid}"))
                    attached_kfs.append(kid)
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={
                    **reg.to_dict(),
                    "attached_texture_keyframe_ids": attached_kfs,
                },
                images=tuple(imgs),
            )
        rp = daemon.memory.graph.keyframe_rgb_paths.get(kf_id)
        if not rp:
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"error": "unknown_keyframe_id", "keyframe_id": kf_id},
                is_error=True,
            )
        try:
            resolved = Path(rp).resolve()
            resolved.relative_to(out_root)
        except (OSError, ValueError):
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"error": "evidence_outside_output_root", "keyframe_id": kf_id},
                is_error=True,
            )
        if not resolved.is_file():
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"error": "evidence_file_missing", "keyframe_id": kf_id},
                is_error=True,
            )
        pose = daemon.memory.graph.keyframe_poses.get(kf_id)
        pos_list = [round(float(v), 2) for v in pose[:3, 3]] if pose is not None else None
        return ToolResult(
            call_id=call.call_id,
            name=call.name,
            payload={"keyframe_id": kf_id, "pose_xyz": pos_list},
            images=(ImageAttachment(resolved, f"L3 keyframe {kf_id}"),),
        )

    def _handle_update_belief(call: ToolCall) -> ToolResult:
        target_label = active_target_getter()
        oid = str(call.arguments.get("object_id", "")).strip()
        status = str(call.arguments.get("status", "refuted")).strip().lower()
        new_lbl = str(call.arguments.get("new_label", "")).strip()
        note = str(call.arguments.get("note", "")).strip()
        zone_type = str(call.arguments.get("zone_type", "")).strip()
        if zone_type and record_zone_annotation_fn is not None:
            record_zone_annotation_fn(zone_type, note)
        if not oid and zone_type:
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"updated_zone_type": zone_type, "map_revision": daemon.memory.map_revision},
            )
        try:
            res = daemon.memory.update_object_belief(
                oid, status=status, new_label=(new_lbl or None), note=note
            )
        except ValueError as exc:
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"error": str(exc)},
                is_error=True,
            )
        if status == "refuted":
            daemon.total_refutations += 1
        res["candidates"] = daemon.current_candidates(target_label)
        return ToolResult(call_id=call.call_id, name=call.name, payload=res)

    def _handle_observe_landmark(call: ToolCall) -> ToolResult:
        label = str(call.arguments.get("label") or "").strip()
        if not label or daemon.current_kf_id is None:
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"error": "missing_label_or_keyframe"},
                is_error=True,
            )
        try:
            bearing = float(call.arguments.get("bearing_deg", 0.0))
        except (TypeError, ValueError):
            bearing = 0.0
        obs = daemon.sim.get_sensor_observations()
        patch = measure_depth_bearing_patch(obs["depth_sensor"], bearing)
        if patch is None:
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"grounded": False, "reason": "no_valid_depth_at_bearing"},
            )
        d_meas, ext_xyz = patch
        lx, ly = depth_bearing_to_local_xy(d_meas, bearing)
        seq = daemon.memory.allocate_step_seq()
        node, _ = daemon.memory.add_or_fuse_object(
            label=label,
            keyframe_id=daemon.current_kf_id,
            local_xyz=[lx, ly, 0.0],
            local_extent_xyz=ext_xyz,
            confidence=0.8,
            producer="vlm_depth_grounded",
            range_source="habitat_depth_raycast",
            evidence_frame="habitat_world",
            evidence_stamp_s=float(seq),
        )
        return ToolResult(
            call_id=call.call_id,
            name=call.name,
            payload={
                "grounded": True,
                "object_id": node.object_id,
                "label": node.label,
                "surface_dist_m": round(d_meas, 2),
                "keyframe_id": daemon.current_kf_id,
            },
        )

    def _handle_traverse_stairs(call: ToolCall) -> ToolResult:
        if step_stairs_fn is None:
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"status": "unsupported", "reason": "stair_traversal_unavailable"},
                is_error=True,
            )
        target_label = active_target_getter()
        direction = str(call.arguments.get("direction", "down")).strip().lower()
        if direction not in ("up", "down"):
            return ToolResult(
                call_id=call.call_id,
                name=call.name,
                payload={"error": "invalid_direction", "expected": ["up", "down"]},
                is_error=True,
            )
        raw_xy = call.arguments.get("xy")
        anchor_xy: list[float] | None = None
        if isinstance(raw_xy, (list, tuple)) and len(raw_xy) >= 2:
            try:
                vx, vy = float(raw_xy[0]), float(raw_xy[1])
                if math.isfinite(vx) and math.isfinite(vy):
                    anchor_xy = [vx, vy]
            except (TypeError, ValueError):
                anchor_xy = None
        exec_info = step_stairs_fn(direction, anchor_xy)
        ing = daemon.ingest_sensor_frame(
            force_keyframe=True,
            run_baseline_vision=True,
            active_target=target_label,
        )
        new_cands = daemon.current_candidates(target_label)
        payload: dict[str, Any] = {
            **exec_info,
            "robot_xyz": ing["robot_mem"],
            "yaw_deg": ing["yaw_deg"],
            "region_id": ing["zone_id"],
            "region_label": ing.get("region_label", "open_area"),
            "keyframe_id": ing["keyframe_id"],
            "map_revision": daemon.memory.map_revision,
            "candidates": new_cands,
            "detections": [g.to_prompt_dict() for g in daemon.current_detections[:8]],
        }
        if ing.get("zone_changed"):
            payload["global_regions_digest"] = daemon.region_memory.format_global_digest()
        return ToolResult(
            call_id=call.call_id,
            name=call.name,
            payload=payload,
            images=_obs_images(include_floorplan=True, target_label=target_label),
        )

    def _handle_finish_task(call: ToolCall) -> ToolResult:
        # Gated by BeforeFinishHook inside EmbodiedAgentRuntime; handler is a safety fallback.
        return ToolResult(
            call_id=call.call_id,
            name=call.name,
            payload={"allowed": True, "reason": "default_finish", **call.arguments},
        )

    tools_list: list[tuple[ToolSpec, Callable[[ToolCall], ToolResult]]] = [
        (
            ToolSpec(
                name="navigate_to",
                description=(
                    "Navigate toward a frontier candidate (F1..F*), memory target (M1..M*), "
                    "staircase portal (S1..S*), or [x,y,z] coordinate along the walkable NavMesh."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "target_id": {
                            "type": "string",
                            "description": "Candidate ID like 'F1', 'M1', or stair portal 'S1'.",
                        },
                        "xyz": {
                            "type": "array",
                            "items": {"type": "number"},
                            "description": "Optional [x, y, z] coordinates if not using target_id.",
                        },
                        "reason": {"type": "string"},
                    },
                },
            ),
            _handle_navigate_to,
        ),
        (
            ToolSpec(
                name="scan_surroundings",
                description="Turn in place (15..120 deg) to reveal hidden objects, doorways, or stairs.",
                parameters={
                    "type": "object",
                    "properties": {
                        "direction": {"type": "string", "enum": ["left", "right"]},
                        "angle_deg": {"type": "number", "default": 60},
                        "reason": {"type": "string"},
                    },
                    "required": ["direction"],
                },
            ),
            _handle_scan_surroundings,
        ),
        (
            ToolSpec(
                name="maneuver",
                description="Take 1..16 atomic steps forward or turn to align in doorways, corners, or staircases.",
                parameters={
                    "type": "object",
                    "properties": {
                        "motion": {"type": "string", "enum": ["forward", "left", "right"]},
                        "steps": {"type": "integer", "minimum": 1, "maximum": 16},
                        "reason": {"type": "string"},
                    },
                    "required": ["motion", "steps"],
                },
            ),
            _handle_maneuver,
        ),
        (
            ToolSpec(
                name="inspect_vision",
                description=(
                    "Run local open-vocabulary SAM 3 pixel segmentation + depth grounding on the "
                    "current front RGB with custom prompts (target + confusable classes + distractors) "
                    "returning candidate_scores distributions and 3D distances."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "prompts": {
                            "type": "array",
                            "items": {"type": "string"},
                            "description": "Contrastive concepts to test simultaneously (2..10 short phrases).",
                        },
                        "mode": {"type": "string", "enum": ["disambiguate", "segment"], "default": "disambiguate"},
                    },
                    "required": ["prompts"],
                },
            ),
            _handle_inspect_vision,
        ),
        (
            ToolSpec(
                name="query_world",
                description=(
                    "Query the L0-L3 spatial memory (SigLIP + text + negative zone coverage) for "
                    "remembered objects/rooms/L3 frames, and optionally attach the 2.5D global floorplan."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "query": {"type": "string"},
                        "include_floorplan": {"type": "boolean", "default": False},
                    },
                    "required": ["query"],
                },
            ),
            _handle_query_world,
        ),
        (
            ToolSpec(
                name="get_evidence",
                description=(
                    "Attach raw historical L3 keyframe RGB(s) by keyframe_id OR by region_id "
                    "(returns that region's Set-Cover minimal texture keyframes + detailed summary)."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "keyframe_id": {"type": "string"},
                        "region_id": {"type": "string", "description": "Region ID like 'R1' to inspect its texture keyframes."},
                    },
                },
            ),
            _handle_get_evidence,
        ),
        (
            ToolSpec(
                name="update_belief",
                description=(
                    "Confirm or refute a memory object (removing false M* candidates) or label the "
                    "current functional Zone."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "object_id": {"type": "string"},
                        "status": {"type": "string", "enum": ["confirmed", "refuted", "tentative"]},
                        "new_label": {"type": "string"},
                        "note": {"type": "string"},
                        "zone_type": {"type": "string"},
                    },
                    "required": ["status"],
                },
            ),
            _handle_update_belief,
        ),
        (
            ToolSpec(
                name="finish_task",
                description=(
                    "Declare task completion (for ObjectNav, call ONLY when directly visible within "
                    "stop_radius_m on the same floor; for map_report/eqa, provide summary & evidence_ids)."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "result_type": {
                            "type": "string",
                            "enum": ["arrived", "map_report", "eqa_answer"],
                            "default": "arrived",
                        },
                        "target_label": {"type": "string"},
                        "summary": {"type": "string"},
                        "evidence_ids": {"type": "array", "items": {"type": "string"}},
                    },
                },
            ),
            _handle_finish_task,
        ),
    ]
    if step_stairs_fn is not None:
        tools_list.insert(
            1,
            (
                ToolSpec(
                    name="traverse_stairs",
                    description=(
                        "Ascend or descend a nearby staircase to reach an adjacent floor along "
                        "the walkable NavMesh. Call when you see stairs or want to explore another floor."
                    ),
                    parameters={
                        "type": "object",
                        "properties": {
                            "direction": {
                                "type": "string",
                                "enum": ["down", "up"],
                                "description": "Whether to go 'down' or 'up' the staircase.",
                            },
                            "xy": {
                                "type": "array",
                                "items": {"type": "number"},
                                "description": "Optional [x, y] stair anchor near the visible stairwell.",
                            },
                            "reason": {"type": "string"},
                        },
                        "required": ["direction"],
                    },
                ),
                _handle_traverse_stairs,
            ),
        )
    if not (daemon.vision_has_seg and daemon.vision_backend == "sam3"):
        tools_list.append(
            (
                ToolSpec(
                    name="observe_landmark",
                    description=(
                        "Ground a visible object along a camera-relative horizontal bearing (0=ahead, "
                        "+left, -right) using live depth raycast into 3D memory."
                    ),
                    parameters={
                        "type": "object",
                        "properties": {
                            "label": {"type": "string"},
                            "bearing_deg": {"type": "number", "default": 0.0},
                        },
                        "required": ["label"],
                    },
                ),
                _handle_observe_landmark,
            )
        )
    return tools_list


class ObjectNavFinishVerifier:
    """Sensor-only BeforeFinishHook enforcing current-frame proximity and mask consensus."""

    def __init__(
        self,
        *,
        daemon: HabitatWorldDaemon,
        target_category: str,
        stop_radius_m: float,
        on_attempt_cb: Callable[[FinishCheckResult], None] | None = None,
    ) -> None:
        self.daemon = daemon
        self.target_category = str(target_category).strip()
        self.stop_radius_m = float(stop_radius_m)
        self.on_attempt_cb = on_attempt_cb
        self.first_stop_attempted = False
        self.first_stop_success = False

    def __call__(self, call: ToolCall) -> FinishCheckResult:
        claimed = call.arguments.get("target_label")
        if (
            isinstance(claimed, str)
            and claimed.strip()
            and not _label_strict_match(self.target_category, claimed)
        ):
            res = FinishCheckResult(
                allowed=False,
                reason="target_label_mismatch",
                metadata={"claimed": claimed, "expected": self.target_category},
            )
            if self.on_attempt_cb:
                self.on_attempt_cb(res)
            return res

        robot_mem, _, _ = self.daemon.agent_pose()
        kf_id = self.daemon.current_kf_id

        # Confusable-class & low-score stop guard on current frame detections (prevents
        # bedroom nightstands/dressers/consoles from slipping past a 'table' stop gate).
        _CONFUSABLE_FOR_TARGET: dict[str, frozenset[str]] = {
            "table": frozenset(
                {
                    "nightstand",
                    "bedside table",
                    "dresser",
                    "chest of drawers",
                    "cabinet",
                    "vanity",
                    "counter",
                    "kitchen counter",
                    "shelf",
                    "bookshelf",
                    "console",
                    "stair railing",
                    "railing",
                }
            ),
            "chair": frozenset({"bed", "couch", "sofa", "table", "cabinet", "toilet"}),
            "couch": frozenset({"bed", "chair", "table", "ottoman", "bench"}),
            "bed": frozenset({"couch", "sofa", "table", "dresser"}),
        }
        _MIN_TARGET_SCORE: dict[str, float] = {"table": 0.45}

        if self.daemon.vision_has_seg and self.daemon.current_detections:
            conf_set = _CONFUSABLE_FOR_TARGET.get(self.target_category, frozenset())
            min_req = _MIN_TARGET_SCORE.get(self.target_category, 0.35)
            near_matches = [
                g
                for g in self.daemon.current_detections
                if _label_strict_match(self.target_category, g.label)
                and float(g.surface_distance_m) <= (self.stop_radius_m + 0.6)
            ]
            if near_matches:
                best_det = max(near_matches, key=lambda d: float(d.score))
                conf_winner: tuple[str, float] | None = None
                for clbl, csc in (best_det.candidate_scores or {}).items():
                    cl_norm = str(clbl).strip().lower()
                    if (
                        any(cf in cl_norm for cf in conf_set)
                        and float(csc) >= float(best_det.score) - 0.08
                    ):
                        if conf_winner is None or float(csc) > conf_winner[1]:
                            conf_winner = (cl_norm, float(csc))
                # Also check spatially overlapping stronger confusable detection on the same frame
                if conf_winner is None and conf_set:
                    for other in self.daemon.current_detections:
                        ol_norm = str(other.label).strip().lower()
                        if not any(cf in ol_norm for cf in conf_set):
                            continue
                        dx = float(other.local_xyz[0]) - float(best_det.local_xyz[0])
                        dy = float(other.local_xyz[1]) - float(best_det.local_xyz[1])
                        if math.hypot(dx, dy) <= 0.65 and float(other.score) >= float(best_det.score) - 0.05:
                            conf_winner = (ol_norm, float(other.score))
                            break
                if best_det.abstain or conf_winner is not None:
                    cname = conf_winner[0] if conf_winner is not None else "ambiguous_confusor"
                    res = FinishCheckResult(
                        allowed=False,
                        reason="confusable_class_override",
                        metadata={
                            "target_score": round(float(best_det.score), 3),
                            "confusable_label": cname,
                            "hint": (
                                f"STOP rejected: this object looks more like '{cname}' than a genuine "
                                f"'{self.target_category}'. Refute it with update_belief and keep searching."
                            ),
                        },
                    )
                    if self.on_attempt_cb:
                        self.on_attempt_cb(res)
                    return res
                if float(best_det.score) < min_req and not (
                    self.daemon.latest_disambiguated_ok
                    and self.daemon.latest_disambiguated_kf == kf_id
                ):
                    res = FinishCheckResult(
                        allowed=False,
                        reason="low_confidence_target_detection",
                        metadata={
                            "target_score": round(float(best_det.score), 3),
                            "min_required": min_req,
                            "hint": (
                                f"STOP rejected: '{self.target_category}' score={best_det.score:.2f} < {min_req:.2f}. "
                                f"Run inspect_vision(prompts=['{self.target_category}', ...], mode='disambiguate') "
                                f"with confusors to verify before stopping."
                            ),
                        },
                    )
                    if self.on_attempt_cb:
                        self.on_attempt_cb(res)
                    return res

        # When an exact pixel-mask backend (SAM 3) is active, require vision-mask consensus
        # on the current keyframe so ungrounded VLM guesses or stair-rail hallucinations fail.
        excluded: tuple[str, ...] = (
            ("opencode_vlm", "opencode_vlm_unverified", "vlm_depth_grounded", "agent_relabeled")
            if (self.daemon.vision_has_seg and self.daemon.vision_backend == "sam3")
            else ("opencode_vlm", "opencode_vlm_unverified", "agent_relabeled")
        )
        ver = self.daemon.memory.verify_stop_claim(
            robot_mem,
            target_label=self.target_category,
            stop_radius_m=self.stop_radius_m,
            require_current_keyframe_id=kf_id,
            excluded_producers=excluded,
        )
        if not ver.allowed and self.daemon.vision_has_seg and self.daemon.vision_backend == "sam3":
            hint = (
                f"STOP rejected ({ver.reason}, dist={ver.distance_m}): no verified SAM 3 pixel mask "
                f"consensus for '{self.target_category}' on current keyframe '{kf_id}' within "
                f"{self.stop_radius_m:.1f} m. Run inspect_vision or move closer with direct LOS."
            )
        elif not ver.allowed:
            hint = f"STOP rejected ({ver.reason}, dist={ver.distance_m}). Move closer or inspect."
        else:
            hint = "Verified target proximity within stop_radius."
        res = FinishCheckResult(
            allowed=ver.allowed,
            reason=ver.reason,
            metadata={**ver.to_dict(), "hint": hint},
        )
        if self.on_attempt_cb:
            self.on_attempt_cb(res)
        return res


class ReportFinishVerifier:
    """Task-agnostic BeforeFinishHook for EQA / mapping-report delivery tasks."""

    def __init__(self, daemon: HabitatWorldDaemon) -> None:
        self.daemon = daemon

    def __call__(self, call: ToolCall) -> FinishCheckResult:
        summary = str(call.arguments.get("summary") or "").strip()
        if not summary:
            return FinishCheckResult(allowed=False, reason="missing_summary")
        ev_ids = call.arguments.get("evidence_ids") or []
        valid_ev = []
        if isinstance(ev_ids, (list, tuple)):
            for eid in ev_ids:
                s = str(eid).strip()
                if s in self.daemon.memory.graph.keyframe_poses or s in self.daemon.memory.graph.objects:
                    valid_ev.append(s)
        return FinishCheckResult(
            allowed=True,
            reason="report_verified",
            metadata={"summary": summary, "validated_evidence_ids": valid_ev},
        )


class HabitatAgentRunner:
    """Closed-loop Habitat-Sim HM3D evaluator driven by standalone EmbodiedAgentRuntime."""

    def __init__(
        self,
        *,
        scene_dataset_config: str | Path,
        scene_id: str | Path,
        chat_client: OpenCodeToolChatClient,
        output_dir: str | Path,
        stop_radius_m: float = 1.8,
        memory_carryover: bool = True,
        reset_occupancy_per_subtask: bool = False,
        vision_client: VisionToolClient | None = None,
        vision_enhance_overlay: bool = True,
        vision_score_threshold: float = 0.25,
        vision_fuse_threshold: float = 0.35,
        max_context_tokens: int = 200_000,
        soft_compaction_tokens: int = 155_000,
        enable_vlm_region_summary: bool = True,
        gpu_device_id: int = 0,
        seed: int = 42,
    ) -> None:
        self.scene_dataset_config = str(scene_dataset_config)
        self.scene_id = str(scene_id)
        self.chat_client = chat_client
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.stop_radius_m = float(stop_radius_m)
        self.memory_carryover = bool(memory_carryover)
        self.reset_occupancy_per_subtask = bool(reset_occupancy_per_subtask)
        self.vision_client = vision_client
        self.vision_enhance_overlay = bool(vision_enhance_overlay)
        self.vision_score_threshold = float(vision_score_threshold)
        self.vision_fuse_threshold = float(vision_fuse_threshold)
        self.max_context_tokens = int(max_context_tokens)
        self.soft_compaction_tokens = int(soft_compaction_tokens)
        self.enable_vlm_region_summary = bool(enable_vlm_region_summary)
        self.gpu_device_id = int(gpu_device_id)
        self.seed = int(seed)

    def _create_sim(self) -> Any:
        import habitat_sim

        sim_cfg = habitat_sim.SimulatorConfiguration()
        sim_cfg.scene_dataset_config_file = self.scene_dataset_config
        sim_cfg.scene_id = self.scene_id
        sim_cfg.enable_physics = True
        sim_cfg.gpu_device_id = self.gpu_device_id

        specs = []
        for uuid_name, stype in (
            ("color_sensor", habitat_sim.SensorType.COLOR),
            ("depth_sensor", habitat_sim.SensorType.DEPTH),
            ("semantic_sensor", habitat_sim.SensorType.SEMANTIC),
        ):
            sp = habitat_sim.CameraSensorSpec()
            sp.uuid = uuid_name
            sp.sensor_type = stype
            sp.resolution = [512, 512]
            sp.position = [0.0, 0.88, 0.0]
            specs.append(sp)

        agent_cfg = habitat_sim.agent.AgentConfiguration()
        agent_cfg.sensor_specifications = specs
        agent_cfg.action_space = {
            "move_forward": habitat_sim.agent.ActionSpec(
                "move_forward", habitat_sim.agent.ActuationSpec(amount=0.25)
            ),
            "turn_left": habitat_sim.agent.ActionSpec(
                "turn_left", habitat_sim.agent.ActuationSpec(amount=15.0)
            ),
            "turn_right": habitat_sim.agent.ActionSpec(
                "turn_right", habitat_sim.agent.ActuationSpec(amount=15.0)
            ),
        }
        return habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg]))

    @staticmethod
    def _geodesic_dist(sim: Any, start_hab: Sequence[float], end_hab: Sequence[float]) -> float:
        import habitat_sim

        sp = habitat_sim.ShortestPath()
        sp.requested_start = np.asarray(start_hab, dtype=np.float32)
        sp.requested_end = np.asarray(end_hab, dtype=np.float32)
        if sim.pathfinder.find_path(sp) and math.isfinite(float(sp.geodesic_distance)):
            return float(sp.geodesic_distance)
        return float("inf")

    def _eval_gt_proximity(
        self,
        sim: Any,
        robot_hab: Sequence[float],
        gt_list: Sequence[HabitatObjectGT],
        stop_radius_m: float,
    ) -> tuple[float, float, bool, HabitatObjectGT | None]:
        best_euc = float("inf")
        best_geo = float("inf")
        best_gt: HabitatObjectGT | None = None
        reached = False
        r_mem = np.asarray(hab_to_mem(robot_hab), dtype=np.float64)
        for gt in gt_list:
            if gt.surface_points_mem:
                surf_arr = np.asarray(gt.surface_points_mem, dtype=np.float64)
                d_euc = float(np.min(np.linalg.norm(surf_arr - r_mem[None, :], axis=1)))
            else:
                d_euc = float(np.linalg.norm(r_mem - np.asarray(gt.center_mem, dtype=np.float64)))
            obj_best_geo = float("inf")
            for vp in gt.viewpoints_hab:
                d_geo = self._geodesic_dist(sim, robot_hab, vp)
                if d_geo < obj_best_geo:
                    obj_best_geo = d_geo
            if obj_best_geo < best_geo:
                best_geo = obj_best_geo
                best_gt = gt
            if d_euc < best_euc:
                best_euc = d_euc
            if is_geodesic_target_reached(obj_best_geo, d_euc, stop_radius_m):
                reached = True
        return best_euc, best_geo, reached, best_gt

    def run_sequential_episode(
        self,
        subtasks: Sequence[HabitatSubtaskSpec],
        *,
        start_hab: Sequence[float] | None = None,
        independent_subtasks: bool = False,
    ) -> dict[str, Any]:
        import habitat_sim
        import quaternion

        t_ep = time.perf_counter()
        sim = self._create_sim()
        cot_log_path = self.output_dir / "agent_cot_traces.jsonl"
        cot_log_path.write_text("", encoding="utf-8")

        try:
            target_cats = sorted({st.target_category.strip().lower() for st in subtasks})
            max_stop_r = max((float(st.stop_radius_m) for st in subtasks), default=self.stop_radius_m)
            private_gt = build_private_hm3d_gt(
                sim,
                target_categories=target_cats,
                num_samples=260,
                seed=self.seed,
                max_view_dist_m=max_stop_r,
            )

            sim.pathfinder.seed(self.seed + 7)
            if start_hab is not None:
                init_pt = np.asarray(start_hab, dtype=np.float32)
            else:
                avail_cats = [c for c in target_cats if private_gt.get(c)]
                init_pt = np.asarray(sim.pathfinder.get_random_navigable_point(), dtype=np.float32)
                for _ in range(40):
                    cand_pt = np.asarray(sim.pathfinder.get_random_navigable_point(), dtype=np.float32)
                    if all(
                        math.isfinite(self._eval_gt_proximity(sim, cand_pt, private_gt.get(c, []), max_stop_r)[1])
                        for c in avail_cats
                    ):
                        init_pt = cand_pt
                        break

            st0 = sim.agents[0].get_state()
            st0.position = init_pt
            st0.rotation = quaternion.from_rotation_vector([0.0, 0.0, 0.0])
            sim.agents[0].set_state(st0)

            reg_summarizer = (
                _build_vlm_region_summarizer(self.chat_client.config, self.output_dir)
                if self.enable_vlm_region_summary
                else None
            )
            daemon = HabitatWorldDaemon(
                sim=sim,
                output_dir=self.output_dir,
                stop_radius_m=self.stop_radius_m,
                vision_client=self.vision_client,
                vision_enhance_overlay=self.vision_enhance_overlay,
                vision_score_threshold=self.vision_score_threshold,
                vision_fuse_threshold=self.vision_fuse_threshold,
                region_summarizer_fn=reg_summarizer,
            )

            shared_context: PrefixStableContextManager | None = None
            runtime: EmbodiedAgentRuntime | None = None

            active_state: dict[str, Any] = {
                "target": "",
                "gt_list": [],
                "stop_radius_m": self.stop_radius_m,
                "max_prim_budget": 500,
                "traveled_m": 0.0,
                "primitives_used": 0,
                "oracle_reached": False,
                "first_stop_attempted": False,
                "first_stop_success": False,
                "false_stops": 0,
                "stop_verified_gt": False,
            }

            def _step_goal(goal_mem: Sequence[float]) -> dict[str, Any]:
                goal_hab = np.asarray(mem_to_hab(goal_mem), dtype=np.float32)
                snapped = np.asarray(sim.pathfinder.snap_point(goal_hab), dtype=np.float32)
                if not np.all(np.isfinite(snapped)) or not sim.pathfinder.is_navigable(snapped):
                    return {"status": "blocked", "reason": "unreachable_goal", "traveled_m": 0.0, "primitives": 0}
                _, _, start_h = daemon.agent_pose()
                rem_prims = max(0, int(active_state["max_prim_budget"]) - int(active_state["primitives_used"]))
                max_prims = min(24, rem_prims)
                if max_prims <= 0:
                    return {"status": "failed", "reason": "primitive_budget_exhausted", "traveled_m": 0.0, "primitives": 0}
                follower = habitat_sim.GreedyGeodesicFollower(sim.pathfinder, sim.agents[0], goal_radius=0.45)
                traveled = 0.0
                prev_h = start_h
                taken = 0
                arrived = False
                ferr: str | None = None
                for idx in range(max_prims):
                    try:
                        act = follower.next_action_along(snapped)
                    except habitat_sim.errors.GreedyFollowerError:
                        ferr = "follower_blocked"
                        break
                    if act is None or act == "stop":
                        arrived = True
                        break
                    sim.step(act)
                    taken += 1
                    active_state["primitives_used"] += 1
                    _, _, cur_h = daemon.agent_pose()
                    traveled += float(np.linalg.norm(cur_h - prev_h))
                    prev_h = cur_h
                    if not active_state["oracle_reached"]:
                        _, _, r_pt, _ = self._eval_gt_proximity(
                            sim, cur_h, active_state["gt_list"], active_state["stop_radius_m"]
                        )
                        if r_pt:
                            active_state["oracle_reached"] = True
                    if idx % 4 == 3:
                        daemon.ingest_sensor_frame(force_keyframe=False)

                active_state["traveled_m"] += traveled
                end_mem, _, _ = daemon.agent_pose()
                straight_rem = math.hypot(
                    float(goal_mem[0]) - float(end_mem[0]),
                    float(goal_mem[1]) - float(end_mem[1]),
                )
                if arrived or straight_rem <= 0.65:
                    st_str, reason = "completed", "waypoint_reached"
                elif taken == 0:
                    st_str, reason = "blocked", (ferr or "follower_no_action")
                elif ferr is not None and traveled < 0.25:
                    st_str, reason = "blocked", ferr
                elif traveled < 0.25 and taken >= 4:
                    st_str, reason = "stuck", "no_forward_progress"
                else:
                    st_str, reason = "completed", "partial_progress"
                return {
                    "status": st_str,
                    "reason": reason,
                    "traveled_m": round(traveled, 3),
                    "primitives": taken,
                    "straight_line_to_goal_m": round(straight_rem, 2),
                    "atomic_steps_used": active_state["primitives_used"],
                    "atomic_steps_budget": active_state["max_prim_budget"],
                }

            def _step_prims(act_name: str, count: int) -> dict[str, Any]:
                rem = max(0, int(active_state["max_prim_budget"]) - int(active_state["primitives_used"]))
                n_run = min(max(1, int(count)), rem)
                traveled = 0.0
                _, _, prev_h = daemon.agent_pose()
                for idx in range(n_run):
                    sim.step(act_name)
                    active_state["primitives_used"] += 1
                    _, _, cur_h = daemon.agent_pose()
                    traveled += float(np.linalg.norm(cur_h - prev_h))
                    prev_h = cur_h
                    if not active_state["oracle_reached"]:
                        _, _, r_pt, _ = self._eval_gt_proximity(
                            sim, cur_h, active_state["gt_list"], active_state["stop_radius_m"]
                        )
                        if r_pt:
                            active_state["oracle_reached"] = True
                    if idx % 3 == 2:
                        daemon.ingest_sensor_frame(force_keyframe=False)
                active_state["traveled_m"] += traveled
                return {
                    "status": "completed",
                    "action": act_name,
                    "primitives": n_run,
                    "traveled_m": round(traveled, 3),
                    "atomic_steps_used": active_state["primitives_used"],
                    "atomic_steps_budget": active_state["max_prim_budget"],
                }

            def _step_stairs(
                direction: str, anchor_xy: Sequence[float] | None = None
            ) -> dict[str, Any]:
                rem_prims = max(
                    0, int(active_state["max_prim_budget"]) - int(active_state["primitives_used"])
                )
                if rem_prims <= 0:
                    return {
                        "status": "failed",
                        "reason": "primitive_budget_exhausted",
                        "traveled_m": 0.0,
                        "primitives": 0,
                    }
                start_mem, _, start_h = daemon.agent_pose()
                ax = float(anchor_xy[0]) if (anchor_xy and len(anchor_xy) >= 2) else float(start_mem[0])
                ay = float(anchor_xy[1]) if (anchor_xy and len(anchor_xy) >= 2) else float(start_mem[1])
                cur_y = float(start_h[1])

                # Local NavMesh landing discovery around observed staircase (zero semantic/GT access)
                best_landing: np.ndarray | None = None
                best_score = float("inf")
                for radius_m in (4.5, 7.5, 11.0):
                    for _ in range(180):
                        cand = np.asarray(
                            sim.pathfinder.get_random_navigable_point(), dtype=np.float32
                        )
                        if not np.all(np.isfinite(cand)):
                            continue
                        dy = float(cand[1]) - cur_y
                        if direction == "down" and not (-4.5 <= dy <= -1.2):
                            continue
                        if direction == "up" and not (1.2 <= dy <= 4.5):
                            continue
                        c_mem = hab_to_mem(cand)
                        hdist = math.hypot(float(c_mem[0]) - ax, float(c_mem[1]) - ay)
                        if hdist > radius_m:
                            continue
                        sp = habitat_sim.ShortestPath()
                        sp.requested_start = start_h
                        sp.requested_end = cand
                        if not sim.pathfinder.find_path(sp) or not math.isfinite(
                            float(sp.geodesic_distance)
                        ):
                            continue
                        geo = float(sp.geodesic_distance)
                        if geo > 20.0:
                            continue
                        sc_val = geo + 0.35 * hdist
                        if sc_val < best_score:
                            best_score = sc_val
                            best_landing = cand
                    if best_landing is not None:
                        break

                if best_landing is None:
                    return {
                        "status": "blocked",
                        "reason": "no_connected_stair_landing",
                        "direction": direction,
                        "traveled_m": 0.0,
                        "primitives": 0,
                        "atomic_steps_used": active_state["primitives_used"],
                        "atomic_steps_budget": active_state["max_prim_budget"],
                    }

                max_prims = min(48, rem_prims)
                follower = habitat_sim.GreedyGeodesicFollower(
                    sim.pathfinder, sim.agents[0], goal_radius=0.50
                )
                traveled = 0.0
                prev_h = start_h
                taken = 0
                arrived = False
                ferr: str | None = None
                for idx in range(max_prims):
                    try:
                        act = follower.next_action_along(best_landing)
                    except habitat_sim.errors.GreedyFollowerError:
                        ferr = "follower_blocked"
                        break
                    if act is None or act == "stop":
                        arrived = True
                        break
                    sim.step(act)
                    taken += 1
                    active_state["primitives_used"] += 1
                    _, _, cur_h = daemon.agent_pose()
                    traveled += float(np.linalg.norm(cur_h - prev_h))
                    prev_h = cur_h
                    if not active_state["oracle_reached"]:
                        _, _, r_pt, _ = self._eval_gt_proximity(
                            sim, cur_h, active_state["gt_list"], active_state["stop_radius_m"]
                        )
                        if r_pt:
                            active_state["oracle_reached"] = True
                    if idx % 4 == 3:
                        daemon.ingest_sensor_frame(force_keyframe=False)

                active_state["traveled_m"] += traveled
                end_mem, _, _ = daemon.agent_pose()
                dz = float(end_mem[2]) - float(start_mem[2])
                if abs(dz) >= 1.2:
                    st_str, reason = "completed", "floor_changed"
                elif abs(dz) >= 0.35 or (taken > 0 and traveled >= 0.5):
                    st_str, reason = "completed", "partial_stair_progress"
                elif arrived:
                    st_str, reason = "completed", "stair_landing_reached"
                else:
                    st_str, reason = "blocked", (ferr or "stair_no_progress")
                return {
                    "status": st_str,
                    "reason": reason,
                    "direction": direction,
                    "dz_m": round(dz, 3),
                    "traveled_m": round(traveled, 3),
                    "primitives": taken,
                    "atomic_steps_used": active_state["primitives_used"],
                    "atomic_steps_budget": active_state["max_prim_budget"],
                }

            def _record_zone_note(zone_type: str, note: str) -> None:
                robot_mem, _, _ = daemon.agent_pose()
                daemon.memory.record_zone_observation(
                    robot_mem, zone_type=zone_type, summary=note
                )
                if daemon.region_memory.current_region_id:
                    daemon.region_memory.annotate_region(
                        daemon.region_memory.current_region_id,
                        primary_label=zone_type,
                        detailed_description=note,
                        source="agent_annotated",
                    )

            tools = build_agent_tools(
                daemon=daemon,
                active_target_getter=lambda: str(active_state["target"]),
                step_goal_fn=_step_goal,
                step_primitives_fn=_step_prims,
                step_stairs_fn=_step_stairs,
                record_zone_annotation_fn=_record_zone_note,
            )

            subtask_reports: list[dict[str, Any]] = []
            for sub_idx, subtask in enumerate(subtasks, start=1):
                if independent_subtasks and sub_idx > 1:
                    daemon.reset_world(keep_memory=False, keep_occupancy=False)
                    shared_context = None
                    runtime = None
                    cat_gts = private_gt.get(subtask.target_category.strip().lower(), [])
                    sub_init_pt = init_pt
                    for _ in range(40):
                        cand_pt = np.asarray(sim.pathfinder.get_random_navigable_point(), dtype=np.float32)
                        if not cat_gts:
                            sub_init_pt = cand_pt
                            break
                        _, dg, _, _ = self._eval_gt_proximity(sim, cand_pt, cat_gts, subtask.stop_radius_m)
                        if math.isfinite(dg) and dg >= 1.5:
                            sub_init_pt = cand_pt
                            break
                    st_r = sim.agents[0].get_state()
                    st_r.position = sub_init_pt
                    st_r.rotation = quaternion.from_rotation_vector([0.0, 0.0, 0.0])
                    sim.agents[0].set_state(st_r)
                elif sub_idx > 1:
                    if not self.memory_carryover:
                        daemon.reset_world(keep_memory=False, keep_occupancy=not self.reset_occupancy_per_subtask)
                        shared_context = None
                        runtime = None
                    elif self.reset_occupancy_per_subtask:
                        daemon.reset_world(keep_memory=True, keep_occupancy=False)

                gt_list = private_gt.get(subtask.target_category.strip().lower(), [])
                _, _, sub_start_h = daemon.agent_pose()
                _, optimal_geo_m, reached_init, _ = self._eval_gt_proximity(
                    sim, sub_start_h, gt_list, subtask.stop_radius_m
                )
                if not gt_list or not math.isfinite(optimal_geo_m):
                    subtask_reports.append(
                        {
                            "subtask_id": subtask.subtask_id,
                            "target_category": subtask.target_category,
                            "valid_gt": False,
                            "skipped_reason": "no_reachable_gt",
                            "sr": 0.0,
                            "first_stop_sr": 0.0,
                            "osr": 0.0,
                            "spl": 0.0,
                            "optimal_geodesic_m": None,
                            "traveled_m": 0.0,
                            "steps_used": 0,
                            "memory_diagnostics": daemon.memory.diagnostic_metrics(),
                        }
                    )
                    continue

                active_state.update(
                    {
                        "target": subtask.target_category,
                        "gt_list": gt_list,
                        "stop_radius_m": float(subtask.stop_radius_m),
                        "max_prim_budget": max(1, int(subtask.max_primitive_steps)),
                        "traveled_m": 0.0,
                        "primitives_used": 0,
                        "oracle_reached": bool(reached_init),
                        "first_stop_attempted": False,
                        "first_stop_success": False,
                        "false_stops": 0,
                        "stop_verified_gt": False,
                    }
                )

                daemon.stop_radius_m = float(subtask.stop_radius_m)
                daemon.memory.default_stop_radius_m = float(subtask.stop_radius_m)
                ing0 = daemon.ingest_sensor_frame(
                    force_keyframe=True,
                    run_baseline_vision=True,
                    active_target=subtask.target_category,
                )
                fp0 = daemon.render_floorplan_if_needed(subtask.target_category, force=True)
                init_imgs: list[ImageAttachment] = []
                if daemon.current_vis_rgb_path is not None:
                    init_imgs.append(ImageAttachment(daemon.current_vis_rgb_path, "front RGB camera"))
                if fp0 is not None:
                    init_imgs.append(ImageAttachment(fp0, "global 2.5D floorplan"))

                cands0 = daemon.current_candidates(subtask.target_category)
                init_obs = {
                    "subtask": subtask.subtask_id,
                    "target_category": subtask.target_category,
                    "robot_xyz": ing0["robot_mem"],
                    "yaw_deg": ing0["yaw_deg"],
                    "region_id": ing0["zone_id"],
                    "region_label": ing0.get("region_label", "open_area"),
                    "keyframe_id": ing0["keyframe_id"],
                    "map_revision": daemon.memory.map_revision,
                    "candidates": cands0,
                    "detections": [g.to_prompt_dict() for g in daemon.current_detections[:8]],
                    "global_region_memory": daemon.region_memory.region_summaries(ing0["robot_mem"]),
                }

                # Create persistent PrefixStableContextManager on first valid subtask (or reset)
                is_transition = shared_context is not None
                if shared_context is None:
                    shared_context = PrefixStableContextManager(
                        system_prompt=AGENT_RUNTIME_SYSTEM_PROMPT,
                        initial_task_prompt=(
                            f"## [TASK {sub_idx}]\n"
                            f"{subtask.instruction}\n"
                            f"Target: '{subtask.target_category}', stop_radius_m={subtask.stop_radius_m:.1f}, "
                            f"atomic_budget={subtask.max_primitive_steps}."
                        ),
                        allowed_image_roots=[self.output_dir.resolve()],
                        max_context_tokens=self.max_context_tokens,
                        soft_compaction_tokens=self.soft_compaction_tokens,
                        keep_recent_groups=4,
                    )

                def _on_stop_attempt(ver: FinishCheckResult) -> None:
                    is_first = not active_state["first_stop_attempted"]
                    active_state["first_stop_attempted"] = True
                    if ver.allowed:
                        _, _, r_hab = daemon.agent_pose()
                        _, _, reached_now, _ = self._eval_gt_proximity(
                            sim, r_hab, gt_list, subtask.stop_radius_m
                        )
                        if reached_now:
                            active_state["stop_verified_gt"] = True
                            active_state["oracle_reached"] = True
                            if is_first:
                                active_state["first_stop_success"] = True
                        else:
                            active_state["false_stops"] += 1

                verifier = ObjectNavFinishVerifier(
                    daemon=daemon,
                    target_category=subtask.target_category,
                    stop_radius_m=subtask.stop_radius_m,
                    on_attempt_cb=_on_stop_attempt,
                )

                def _tail_status_hook(_t_idx: int) -> str:
                    rid = daemon.region_memory.current_region_id or daemon.current_zone_id or "R?"
                    reg_obj = daemon.region_memory.regions.get(str(rid))
                    rlbl = reg_obj.primary_label if reg_obj is not None else "open_area"
                    tex_m = daemon.texture_selector.diagnostic_metrics()
                    return (
                        f"[map_rev={daemon.memory.map_revision} | region={rid}({rlbl}) | "
                        f"regions={len(daemon.region_memory.regions)} | "
                        f"tex_kfs={tex_m['retained_texture_keyframes']}/{tex_m['total_candidates_admitted']} | "
                        f"atomic={active_state['primitives_used']}/{active_state['max_prim_budget']}]"
                    )

                runtime = EmbodiedAgentRuntime(
                    chat_client=self.chat_client,
                    context=shared_context,
                    before_turn_hook=_tail_status_hook,
                    before_finish_hook=verifier,
                    world_milestone_fn=daemon.milestone_summary,
                    budget_exhausted_fn=lambda: int(active_state["primitives_used"]) >= int(active_state["max_prim_budget"]),
                    motion_progress_fn=lambda: int(active_state["primitives_used"]),
                    max_consecutive_zero_motion_turns=12,
                )
                for spec, handler in tools:
                    runtime.register_tool(spec, handler)

                import json as _json

                res: AgentRunResult = runtime.run_task(
                    task_label=subtask.target_category,
                    initial_observation_text=_json.dumps(init_obs, ensure_ascii=False, separators=(",", ":")),
                    initial_images=tuple(init_imgs),
                    max_turns=subtask.max_steps,
                    append_transition=is_transition,
                    transition_prompt=(
                        f"## [NEXT_TASK {sub_idx}/{len(subtasks)}]\n"
                        f"{subtask.instruction}\n"
                        f"Remember to check query_world('{subtask.target_category}') or M* candidates from earlier zones!"
                    ),
                )

                with cot_log_path.open("a", encoding="utf-8") as cf:
                    for tr in res.traces:
                        cf.write(
                            _json.dumps(
                                {
                                    "subtask_id": subtask.subtask_id,
                                    "target": subtask.target_category,
                                    **tr,
                                },
                                ensure_ascii=False,
                            )
                            + "\n"
                        )

                _, _, end_h = daemon.agent_pose()
                end_euc, end_geo, _, _ = self._eval_gt_proximity(
                    sim, end_h, gt_list, subtask.stop_radius_m
                )
                sr = 1.0 if active_state["stop_verified_gt"] else 0.0
                first_sr = 1.0 if active_state["first_stop_success"] else 0.0
                osr = 1.0 if active_state["oracle_reached"] else 0.0
                spl = compute_spl(
                    bool(active_state["stop_verified_gt"]),
                    optimal_geo_m,
                    float(active_state["traveled_m"]),
                )
                print(
                    f"[AgentRT sub {sub_idx}:{subtask.target_category}] sr={sr} osr={osr} spl={spl:.3f} "
                    f"turns={res.turns_used} prims={active_state['primitives_used']} "
                    f"false_stops={active_state['false_stops']} end_geo={end_geo:.2f}m "
                    f"term={res.termination_reason} cache_hit={res.context_metrics.get('cache_hit_rate')}",
                    flush=True,
                )
                subtask_reports.append(
                    {
                        "subtask_id": subtask.subtask_id,
                        "target_category": subtask.target_category,
                        "valid_gt": True,
                        "sr": sr,
                        "first_stop_sr": first_sr,
                        "osr": osr,
                        "spl": round(spl, 4),
                        "optimal_geodesic_m": round(optimal_geo_m, 2),
                        "traveled_m": round(float(active_state["traveled_m"]), 2),
                        "steps_used": res.turns_used,
                        "primitives_used": int(active_state["primitives_used"]),
                        "max_primitive_steps": int(active_state["max_prim_budget"]),
                        "false_stops": int(active_state["false_stops"]),
                        "end_gt_euc_m": (None if not math.isfinite(end_euc) else round(end_euc, 3)),
                        "end_gt_geo_m": (None if not math.isfinite(end_geo) else round(end_geo, 3)),
                        "termination_reason": res.termination_reason,
                        "context_metrics": res.context_metrics,
                        "memory_diagnostics": daemon.memory.diagnostic_metrics(),
                    }
                )

            valid_reports = [r for r in subtask_reports if r.get("valid_gt", True)]
            denom = max(1, len(valid_reports))
            ctx_final = shared_context.metrics_dict() if shared_context is not None else {}
            summary = {
                "scene_id": self.scene_id,
                "runtime": "standalone_embodied_agent_runtime_v1",
                "eval_mode": ("independent_cold_start" if independent_subtasks else "sequential_multi_task"),
                "memory_carryover": (False if independent_subtasks else self.memory_carryover),
                "occupancy_carryover": (False if independent_subtasks else not self.reset_occupancy_per_subtask),
                "vision_tools": {
                    "enabled": bool(daemon.vision_client is not None and daemon.vision_ok),
                    "backend": daemon.vision_backend,
                    "embed_model": str(daemon.vision_health.get("embed_model", "")),
                    "total_detections": daemon.total_vision_dets,
                    "total_fused": daemon.total_vision_fused,
                    "total_deferred_hits": daemon.total_deferred_hits,
                    "total_refutations": daemon.total_refutations,
                },
                "runtime_metrics": ctx_final,
                "planner_config": self.chat_client.config.to_safe_dict(),
                "subtasks_count": len(subtask_reports),
                "valid_subtasks_count": len(valid_reports),
                "invalid_subtasks_count": len(subtask_reports) - len(valid_reports),
                "sr": round(sum(r["sr"] for r in valid_reports) / denom, 4) if valid_reports else 0.0,
                "first_stop_sr": (
                    round(sum(r["first_stop_sr"] for r in valid_reports) / denom, 4)
                    if valid_reports
                    else 0.0
                ),
                "osr": round(sum(r["osr"] for r in valid_reports) / denom, 4) if valid_reports else 0.0,
                "spl": round(sum(r["spl"] for r in valid_reports) / denom, 4) if valid_reports else 0.0,
                "false_stops": sum(int(r.get("false_stops", 0)) for r in valid_reports),
                "total_traveled_m": round(sum(r["traveled_m"] for r in valid_reports), 2),
                "total_steps": sum(r["steps_used"] for r in valid_reports),
                "total_primitives": sum(int(r.get("primitives_used", 0)) for r in valid_reports),
                "motion_index_frames": len(daemon.motion_index),
                "keyframes_total": len(daemon.memory.graph.keyframe_poses),
                "scene_graph_rooms": len(daemon.memory.graph.rooms),
                "scene_graph_doorways": len(daemon.memory.graph.doorways),
                "scene_graph_stairs": len(daemon.memory.graph.stairs),
                "texture_keyframe_metrics": daemon.texture_selector.diagnostic_metrics(),
                "vlm_region_summaries": daemon.total_vlm_region_summaries,
                "global_regions": daemon.region_memory.region_summaries(),
                "wall_s": round(time.perf_counter() - t_ep, 2),
                "private_gt_counts": {k: len(v) for k, v in private_gt.items()},
                "subtasks": subtask_reports,
            }
            daemon.memory.save(self.output_dir / "habitat_hierarchical_memory.json")
            atomic_write_json(self.output_dir / "habitat_eval_report.json", summary)
            return summary
        finally:
            sim.close()
