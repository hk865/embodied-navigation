#!/usr/bin/env python3
"""Thin CLI for running GOAT-style sequential multi-task evaluation in Habitat-Sim HM3D."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from embodied_navigation.planning.exploration.habitat_memory_runner import (
    HabitatMemoryRunner,
    HabitatSubtaskSpec,
)
from embodied_navigation.planning.exploration.llm_agent_client import LlmClientConfig
from embodied_navigation.planning.exploration.streaming_task_agent import (
    OpenCodeTaskPlanner,
    resolve_thinking_options,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run closed-loop sequential multi-task navigation in Habitat-Sim HM3D."
    )
    parser.add_argument("--scene-dataset", required=True, help="HM3D scene_dataset_config.json path.")
    parser.add_argument("--scene-id", required=True, help="Scene .basis.glb path.")
    parser.add_argument("--output-dir", required=True, help="Directory for RGB/map logs and report JSON.")
    parser.add_argument(
        "--targets",
        nargs="+",
        default=["chair", "table", "couch", "bed"],
        help="Sequential target object categories for GOAT-style carryover evaluation.",
    )
    parser.add_argument("--provider", default="opencode")
    parser.add_argument("--model", default="qwen3.8-flash")
    parser.add_argument(
        "--thinking-mode",
        choices=("disabled", "default", "medium", "high"),
        default="disabled",
    )
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--stop-radius-m", type=float, default=1.8)
    parser.add_argument("--max-steps-per-subtask", type=int, default=10)
    parser.add_argument("--no-memory-carryover", action="store_true", default=False)
    parser.add_argument(
        "--reset-occupancy-per-subtask",
        action="store_true",
        default=False,
        help="Also reset the 2D depth occupancy grid between subtasks (default keeps occupancy constant for pure L0-L3 memory ablation).",
    )
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    extra_body, timeout_s, max_tokens = resolve_thinking_options(
        args.model,
        args.thinking_mode,
        base_timeout_s=args.timeout_s,
        base_max_tokens=560,
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
    out_dir = Path(args.output_dir)
    planner = OpenCodeTaskPlanner(cfg, allowed_image_roots=[out_dir])
    runner = HabitatMemoryRunner(
        scene_dataset_config=args.scene_dataset,
        scene_id=args.scene_id,
        planner=planner,
        output_dir=out_dir,
        stop_radius_m=args.stop_radius_m,
        memory_carryover=not args.no_memory_carryover,
        reset_occupancy_per_subtask=bool(args.reset_occupancy_per_subtask),
        seed=args.seed,
    )
    subtasks = [
        HabitatSubtaskSpec(
            subtask_id=f"goal_{idx:02d}",
            target_category=cat,
            instruction=(
                f"Explore this real indoor house to find a '{cat}', move within "
                f"{args.stop_radius_m:.1f} m of it with direct line of sight, and call stop_claim "
                f"with target_label='{cat}'. Remember other salient furniture you see along the way!"
            ),
            stop_radius_m=args.stop_radius_m,
            max_steps=args.max_steps_per_subtask,
        )
        for idx, cat in enumerate(args.targets, start=1)
    ]
    report = runner.run_sequential_episode(subtasks)
    print(
        json.dumps(
            {
                "scene_id": report["scene_id"],
                "memory_carryover": report["memory_carryover"],
                "occupancy_carryover": report["occupancy_carryover"],
                "model": args.model,
                "valid_subtasks": report["valid_subtasks_count"],
                "invalid_subtasks": report["invalid_subtasks_count"],
                "sr": report["sr"],
                "first_stop_sr": report["first_stop_sr"],
                "osr": report["osr"],
                "spl": report["spl"],
                "total_traveled_m": report["total_traveled_m"],
                "total_steps": report["total_steps"],
                "wall_s": report["wall_s"],
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
