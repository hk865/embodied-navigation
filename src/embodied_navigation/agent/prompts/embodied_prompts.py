"""System prompts and event-driven VLM region summarization helpers for the Agent layer."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Sequence

from embodied_navigation.agent.runtime import OpenCodeToolChatClient
from embodied_navigation.common.llm_config import (
    LlmClientConfig,
    encode_image_part as _encode_image_part,
)
from embodied_navigation.localization_mapping.semantic_mapping import RegionArchiveEntry


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

build_vlm_region_summarizer = _build_vlm_region_summarizer
