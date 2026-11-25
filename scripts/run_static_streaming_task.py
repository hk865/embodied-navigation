#!/usr/bin/env python3
"""Thin CLI runner for closed-loop static embodied tasks using OpenCode Go streaming."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
)
from embodied_navigation.planning.exploration.llm_agent_client import LlmClientConfig
from embodied_navigation.planning.exploration.streaming_task_agent import (
    OpenCodeTaskPlanner,
    StaticStreamingTaskRunner,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a closed-loop static task with L0-L3 memory and OpenCode Go SSE streaming."
    )
    parser.add_argument("--run-dir", required=True, help="Active DynaNav run directory.")
    parser.add_argument("--task", required=True, help="Natural language task instruction.")
    parser.add_argument("--target-label", default=None, help="Target label for STOP verification.")
    parser.add_argument("--provider", default="opencode", help="LLM provider preset.")
    parser.add_argument("--planner-model", default="glm-5.3-flash", help="Main planner model ID.")
    parser.add_argument(
        "--sentinel-model",
        default="glm-5.3-flash",
        help="In-motion async sentinel model ID (empty to disable sentinel).",
    )
    parser.add_argument("--disable-thinking", action="store_true", default=True)
    parser.add_argument("--planner-timeout-s", type=float, default=18.0)
    parser.add_argument("--sentinel-timeout-s", type=float, default=6.0)
    parser.add_argument("--max-steps", type=int, default=10)
    parser.add_argument("--step-timeout-s", type=float, default=180.0)
    parser.add_argument("--stop-radius-m", type=float, default=1.5)
    parser.add_argument("--pose-source", default="backend_corrected")
    parser.add_argument("--prior-memory", default=None, help="Optional saved hierarchical_memory.json.")
    parser.add_argument("--no-stop-on-exit", action="store_true", default=False)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    extra_body = {"thinking": {"type": "disabled"}} if args.disable_thinking else None

    planner_cfg = LlmClientConfig(
        provider=args.provider,
        model=args.planner_model,
        timeout_s=args.planner_timeout_s,
        max_tokens=360,
        temperature=0.1,
        stream=True,
        extra_body=extra_body,
    )
    sentinel_cfg = None
    if args.sentinel_model and args.sentinel_model.strip():
        sentinel_cfg = LlmClientConfig(
            provider=args.provider,
            model=args.sentinel_model.strip(),
            timeout_s=args.sentinel_timeout_s,
            max_tokens=96,
            temperature=0.1,
            stream=True,
            extra_body=extra_body,
        )

    memory_store = None
    if args.prior_memory:
        memory_store = HierarchicalMemoryStore.load(Path(args.prior_memory))

    runner = StaticStreamingTaskRunner(
        args.run_dir,
        task_instruction=args.task,
        target_label=args.target_label,
        planner=OpenCodeTaskPlanner(planner_cfg),
        memory_store=memory_store,
        pose_source=args.pose_source,
        sentinel_config=sentinel_cfg,
        stop_radius_m=args.stop_radius_m,
        step_timeout_s=args.step_timeout_s,
        stop_simulator_on_exit=not args.no_stop_on_exit,
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
