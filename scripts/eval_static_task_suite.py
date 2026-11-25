#!/usr/bin/env python3
"""Run the 30-task static memory & navigation benchmark over a saved MapStore."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from pathlib import Path
import sys
import time
from typing import Any

from embodied_navigation.localization_mapping.semantic_mapping import (
    build_scene_graph_from_map_dir,
)
from embodied_navigation.planning.exploration.llm_agent_client import LlmClientConfig
from embodied_navigation.planning.exploration.static_task_suite import (
    aggregate_suite_results,
    build_default_30_task_suite,
    evaluate_single_task_offline,
)
from embodied_navigation.planning.exploration.streaming_task_agent import (
    OpenCodeTaskPlanner,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate the 30-task static memory/navigation suite with OpenCode Go."
    )
    parser.add_argument("--map-dir", required=True, help="Saved MapStore directory (e.g. D-Indoor-2F/map).")
    parser.add_argument("--output", required=True, help="Output JSON report path.")
    parser.add_argument("--provider", default="opencode")
    parser.add_argument("--model", default="qwen3.8-flash")
    parser.add_argument("--thinking-mode", choices=("disabled", "default", "high"), default="disabled")
    parser.add_argument(
        "--modes",
        nargs="+",
        default=["hierarchical_6zone", "no_memory_targets", "stale_unfiltered"],
        help="Memory modes to evaluate.",
    )
    parser.add_argument("--workers", type=int, default=4, help="Parallel episode workers.")
    parser.add_argument("--limit", type=int, default=30, help="Limit number of tasks (default: all 30).")
    return parser.parse_args(argv)


def _extra_body(model_id: str, mode: str) -> dict[str, Any] | None:
    if mode == "high":
        return {"reasoning_effort": "high"}
    if mode == "disabled":
        return {"enable_thinking": False} if "qwen" in model_id.lower() else {"thinking": {"type": "disabled"}}
    return None


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    t0 = time.monotonic()
    graph = build_scene_graph_from_map_dir(Path(args.map_dir))
    tasks = build_default_30_task_suite(graph)[: max(1, int(args.limit))]

    timeout_s = 60.0 if args.thinking_mode == "high" else 25.0
    max_tokens = 1024 if args.thinking_mode == "high" else 360
    extra = _extra_body(args.model, args.thinking_mode)

    all_modes: dict[str, Any] = {}
    for mode in args.modes:
        mode_t0 = time.monotonic()
        rows: list[dict[str, Any]] = []

        def _run_one(spec):
            cfg = LlmClientConfig(
                provider=args.provider,
                model=args.model,
                timeout_s=timeout_s,
                max_tokens=max_tokens,
                temperature=0.1,
                supports_vision=False,
                stream=True,
                extra_body=extra,
            )
            return evaluate_single_task_offline(
                graph, spec, OpenCodeTaskPlanner(cfg), memory_mode=mode
            )

        with ThreadPoolExecutor(max_workers=max(1, int(args.workers))) as pool:
            futs = {pool.submit(_run_one, spec): spec.task_id for spec in tasks}
            for fut in as_completed(futs):
                res = fut.result()
                rows.append(res)
                print(
                    f"[{mode}] {res['task_id']} ({res['tier']}) SR={int(res['sr'])} "
                    f"SPL={res['spl']:.2f} steps={res['steps_used']} recalls={res['memory_recall_calls']} "
                    f"stale_err={res['stale_flip_errors']} lat={res['mean_latency_ms']:.0f}ms",
                    flush=True,
                )

        rows.sort(key=lambda r: r["task_id"])
        summary = aggregate_suite_results(rows)
        all_modes[mode] = {
            "wall_s": round(time.monotonic() - mode_t0, 2),
            "summary": summary,
            "tasks": rows,
        }
        print(f"=== MODE {mode} SUMMARY ({all_modes[mode]['wall_s']}s) ===", flush=True)
        print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "model": args.model,
        "provider": args.provider,
        "thinking_mode": args.thinking_mode,
        "num_tasks": len(tasks),
        "total_wall_s": round(time.monotonic() - t0, 2),
        "modes": all_modes,
    }
    out_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Saved full 30-task benchmark report to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
