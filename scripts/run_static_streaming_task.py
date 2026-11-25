#!/usr/bin/env python3
"""Thin CLI runner for closed-loop static embodied tasks using OpenCode Go streaming."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    build_scene_graph_from_map_dir,
)
from embodied_navigation.planning.exploration.llm_agent_client import LlmClientConfig
from embodied_navigation.planning.exploration.streaming_task_agent import (
    OpenCodeTaskPlanner,
    StaticStreamingTaskRunner,
    VisionLandmarkObserver,
)


def _thinking_extra_body(model_id: str, mode: str) -> dict[str, object] | None:
    m = model_id.lower()
    if mode == "high":
        return {"reasoning_effort": "high"}
    if mode == "disabled":
        if "qwen" in m:
            return {"enable_thinking": False}
        return {"thinking": {"type": "disabled"}}
    return None


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a closed-loop static task with L0-L3 memory and OpenCode Go SSE streaming."
    )
    parser.add_argument("--run-dir", required=True, help="Active DynaNav run directory.")
    parser.add_argument("--task", required=True, help="Natural language task instruction.")
    parser.add_argument("--target-label", default=None, help="Target label for STOP verification.")
    parser.add_argument(
        "--target-alias",
        action="append",
        default=[],
        help="Optional explicit alias accepted for frozen target STOP verification.",
    )
    parser.add_argument("--provider", default="opencode", help="LLM provider preset.")
    parser.add_argument("--planner-model", default="qwen3.8-flash", help="Main planner model ID.")
    parser.add_argument(
        "--sentinel-model",
        default="qwen3.8-flash",
        help="In-motion async sentinel model ID (empty to disable sentinel).",
    )
    parser.add_argument(
        "--vision-observer-model",
        default="qwen3.8-flash",
        help="Vision landmark observer model used automatically when planner is text-only (empty to disable).",
    )
    parser.add_argument(
        "--thinking-mode",
        choices=("disabled", "default", "high"),
        default="disabled",
        help="Reasoning mode for planner ('high' supports DeepSeek high reasoning).",
    )
    parser.add_argument("--planner-timeout-s", type=float, default=25.0)
    parser.add_argument("--sentinel-timeout-s", type=float, default=8.0)
    parser.add_argument("--max-steps", type=int, default=10)
    parser.add_argument("--step-timeout-s", type=float, default=180.0)
    parser.add_argument("--stop-radius-m", type=float, default=1.5)
    parser.add_argument("--pose-source", default="backend_corrected")
    parser.add_argument("--prior-memory", default=None, help="Optional saved hierarchical_memory.json.")
    parser.add_argument(
        "--prior-evidence-root",
        action="append",
        default=[],
        help="Explicitly allowed directory root(s) for prior-memory L3 RGB files.",
    )
    parser.add_argument(
        "--continue-episode",
        action="store_true",
        default=False,
        help="Keep transient attempt ledger from --prior-memory instead of starting a fresh episode.",
    )
    parser.add_argument("--prior-map-dir", default=None, help="Optional saved MapStore dir for initial L0-L3 memory.")
    parser.add_argument("--no-stop-on-exit", action="store_true", default=False)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    planner_extra = _thinking_extra_body(args.planner_model, args.thinking_mode)
    planner_timeout = (
        max(90.0, args.planner_timeout_s) if args.thinking_mode == "high" else args.planner_timeout_s
    )
    planner_max_tokens = 2048 if args.thinking_mode == "high" else 380
    planner_vision = "deepseek" not in args.planner_model.lower()

    planner_cfg = LlmClientConfig(
        provider=args.provider,
        model=args.planner_model,
        timeout_s=planner_timeout,
        max_tokens=planner_max_tokens,
        temperature=0.1,
        supports_vision=planner_vision,
        stream=True,
        extra_body=planner_extra,
    )
    sentinel_cfg = None
    if args.sentinel_model and args.sentinel_model.strip():
        s_model = args.sentinel_model.strip()
        sentinel_cfg = LlmClientConfig(
            provider=args.provider,
            model=s_model,
            timeout_s=args.sentinel_timeout_s,
            max_tokens=96,
            temperature=0.1,
            supports_vision=("deepseek" not in s_model.lower()),
            stream=True,
            extra_body=_thinking_extra_body(s_model, "disabled"),
        )

    allowed_roots = [Path(args.run_dir), *(Path(p) for p in args.prior_evidence_root)]
    vision_observer = None
    if (
        not planner_vision
        and args.vision_observer_model
        and args.vision_observer_model.strip()
    ):
        v_model = args.vision_observer_model.strip()
        v_cfg = LlmClientConfig(
            provider=args.provider,
            model=v_model,
            timeout_s=max(20.0, args.sentinel_timeout_s),
            max_tokens=256,
            temperature=0.1,
            supports_vision=True,
            stream=True,
            extra_body=_thinking_extra_body(v_model, "disabled"),
        )
        vision_observer = VisionLandmarkObserver(v_cfg, allowed_image_roots=allowed_roots)

    memory_store = None
    if args.prior_memory:
        memory_store = HierarchicalMemoryStore.load(Path(args.prior_memory))
        if not args.continue_episode:
            memory_store.clear_episode_attempts()
    elif args.prior_map_dir:
        graph = build_scene_graph_from_map_dir(Path(args.prior_map_dir))
        memory_store = HierarchicalMemoryStore(graph, default_stop_radius_m=args.stop_radius_m)

    runner = StaticStreamingTaskRunner(
        args.run_dir,
        task_instruction=args.task,
        target_label=args.target_label,
        target_aliases=args.target_alias,
        planner=OpenCodeTaskPlanner(planner_cfg, allowed_image_roots=allowed_roots),
        vision_observer=vision_observer,
        memory_store=memory_store,
        pose_source=args.pose_source,
        sentinel_config=sentinel_cfg,
        stop_radius_m=args.stop_radius_m,
        step_timeout_s=args.step_timeout_s,
        stop_simulator_on_exit=not args.no_stop_on_exit,
        allowed_image_roots=allowed_roots,
    )
    report = runner.run(max_steps=args.max_steps)
    print(
        json.dumps(
            {
                "task_succeeded": report["task_succeeded"],
                "terminal_status": report["terminal_status"],
                "steps_executed": report["steps_executed"],
                "wall_s": report["wall_s"],
                "estimated_travel_m": report["estimated_travel_m"],
                "memory_diagnostics": report["memory_diagnostics"],
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0 if report["task_succeeded"] else 1


if __name__ == "__main__":
    sys.exit(main())
