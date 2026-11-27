#!/usr/bin/env python3
"""Thin CLI for evaluating Habitat-Sim HM3D with the standalone EmbodiedAgentRuntime."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from embodied_navigation.agent import (
    HabitatAgentRunner,
    OpenCodeToolChatClient,
    resolve_agent_thinking,
)
from embodied_navigation.common.llm_config import LlmClientConfig
from embodied_navigation.perception.vision_tools import VisionToolClient
from embodied_navigation.planning.exploration.habitat_memory_runner import (
    HabitatSubtaskSpec,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run closed-loop Habitat-Sim HM3D evaluation using standalone EmbodiedAgentRuntime."
    )
    parser.add_argument("--scene-dataset", required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--targets", nargs="+", default=["chair", "table", "couch", "bed"])
    parser.add_argument("--provider", default="opencode")
    parser.add_argument("--model", default="deepseek-v4.1-flash")
    parser.add_argument(
        "--thinking-mode",
        choices=("disabled", "default", "medium", "high", "enabled"),
        default="default",
    )
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--stop-radius-m", type=float, default=1.8)
    parser.add_argument(
        "--max-turns-per-subtask",
        type=int,
        default=0,
        help="Optional LLM turn cap per subtask (0 = unlimited, strictly governed by --max-primitive-steps 500).",
    )
    parser.add_argument("--max-primitive-steps", type=int, default=500)
    parser.add_argument("--max-context-tokens", type=int, default=200_000)
    parser.add_argument("--soft-compaction-tokens", type=int, default=155_000)
    parser.add_argument("--eval-mode", choices=("sequential", "independent"), default="sequential")
    parser.add_argument("--no-memory-carryover", action="store_true", default=False)
    parser.add_argument("--reset-occupancy-per-subtask", action="store_true", default=False)
    parser.add_argument("--vision-endpoint", default="")
    parser.add_argument("--no-vision-overlay", action="store_true", default=False)
    parser.add_argument("--vision-score-threshold", type=float, default=0.25)
    parser.add_argument("--vision-fuse-threshold", type=float, default=0.35)
    parser.add_argument("--no-vlm-region-summary", action="store_true", default=False)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    extra_body, timeout_s, max_tokens = resolve_agent_thinking(
        args.model,
        args.thinking_mode,
        base_timeout_s=args.timeout_s,
    )
    cfg = LlmClientConfig(
        provider=args.provider,
        model=args.model,
        timeout_s=timeout_s,
        max_tokens=max_tokens,
        temperature=0.1,
        supports_vision=True,
        stream=True,
        extra_body=extra_body,
    )
    chat_client = OpenCodeToolChatClient(cfg)
    vision_client = (
        VisionToolClient(args.vision_endpoint, timeout_s=10.0)
        if str(args.vision_endpoint or "").strip()
        else None
    )
    out_dir = Path(args.output_dir)
    runner = HabitatAgentRunner(
        scene_dataset_config=args.scene_dataset,
        scene_id=args.scene_id,
        chat_client=chat_client,
        output_dir=out_dir,
        stop_radius_m=args.stop_radius_m,
        memory_carryover=not args.no_memory_carryover,
        reset_occupancy_per_subtask=bool(args.reset_occupancy_per_subtask),
        vision_client=vision_client,
        vision_enhance_overlay=not bool(args.no_vision_overlay),
        vision_score_threshold=float(args.vision_score_threshold),
        vision_fuse_threshold=float(args.vision_fuse_threshold),
        max_context_tokens=int(args.max_context_tokens),
        soft_compaction_tokens=int(args.soft_compaction_tokens),
        enable_vlm_region_summary=not bool(args.no_vlm_region_summary),
        seed=args.seed,
    )
    subtasks = [
        HabitatSubtaskSpec(
            subtask_id=f"goal_{idx:02d}",
            target_category=cat,
            instruction=(
                f"Find a '{cat}' in this indoor house, inspect contrastively if uncertain, "
                f"get within {args.stop_radius_m:.1f} m with direct line of sight on the same floor, "
                f"and call finish_task(result_type='arrived', target_label='{cat}')."
            ),
            stop_radius_m=args.stop_radius_m,
            max_steps=(int(args.max_turns_per_subtask) if int(args.max_turns_per_subtask) > 0 else None),
            max_primitive_steps=args.max_primitive_steps,
        )
        for idx, cat in enumerate(args.targets, start=1)
    ]
    report = runner.run_sequential_episode(
        subtasks,
        independent_subtasks=(args.eval_mode == "independent"),
    )
    print(
        json.dumps(
            {
                "scene_id": report["scene_id"],
                "runtime": report["runtime"],
                "model": args.model,
                "sr": report["sr"],
                "first_stop_sr": report["first_stop_sr"],
                "osr": report["osr"],
                "spl": report["spl"],
                "false_stops": report["false_stops"],
                "runtime_metrics": report["runtime_metrics"],
                "wall_s": report["wall_s"],
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
