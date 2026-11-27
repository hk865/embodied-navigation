"""Generic embodied tool contracts and handlers registered onto EmbodiedAgentRuntime."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Callable, Sequence

from embodied_navigation.agent.runtime import (
    ImageAttachment,
    ToolCall,
    ToolResult,
    ToolSpec,
)
from embodied_navigation.planning.exploration.habitat_memory_runner import (
    depth_bearing_to_local_xy,
    measure_depth_bearing_patch,
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

format_agent_candidates = _format_agent_candidates


def build_agent_tools(
    *,
    daemon: Any,
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
