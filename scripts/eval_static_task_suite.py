#!/usr/bin/env python3
"""Run the 30-task static memory & navigation benchmark over a saved MapStore."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

from embodied_navigation.localization_mapping.semantic_mapping import (
    build_scene_graph_from_map_dir,
)
from embodied_navigation.planning.exploration.llm_agent_client import LlmClientConfig
from embodied_navigation.planning.exploration.static_task_suite import (
    DeterministicGreedyM1Planner,
    DeterministicRecallGotoXyzPlanner,
    VALID_MEMORY_MODES,
    aggregate_suite_results,
    build_30_static_tasks,
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
        default=list(VALID_MEMORY_MODES),
        help=f"Memory modes to evaluate ({', '.join(VALID_MEMORY_MODES)}).",
    )
    parser.add_argument(
        "--include-deterministic-baselines",
        dest="include_deterministic_baselines",
        action="store_true",
        default=True,
        help="Run model-free deterministic baselines (det_m1_greedy, det_recall_goto_xyz).",
    )
    parser.add_argument(
        "--no-deterministic-baselines",
        "--no-include-deterministic-baselines",
        dest="include_deterministic_baselines",
        action="store_false",
        help="Disable model-free deterministic baselines.",
    )
    parser.add_argument("--workers", type=int, default=4, help="Parallel episode workers.")
    parser.add_argument("--limit", type=int, default=30, help="Limit number of tasks (default: all 30).")
    args = parser.parse_args(argv)
    invalid_modes = [m for m in args.modes if m not in VALID_MEMORY_MODES]
    if invalid_modes:
        parser.error(
            f"invalid --modes {invalid_modes!r}; supported modes: {', '.join(VALID_MEMORY_MODES)}"
        )
    return args


def _extra_body(model_id: str, mode: str) -> dict[str, Any] | None:
    if mode == "high":
        return {"reasoning_effort": "high"}
    if mode == "disabled":
        return {"enable_thinking": False} if "qwen" in model_id.lower() else {"thinking": {"type": "disabled"}}
    return None


def _resolve_git_commit() -> str:
    repo_root = Path(__file__).resolve().parents[1]
    try:
        res = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
        if res.returncode == 0 and res.stdout.strip():
            return res.stdout.strip()
    except OSError:
        pass
    return "unknown"


def _map_json_sha256(map_dir: Path) -> str | None:
    map_json = map_dir / "map.json"
    if not map_json.is_file():
        return None
    return hashlib.sha256(map_json.read_bytes()).hexdigest()


def _print_summary_table(
    all_modes: dict[str, Any], deterministic_baselines: dict[str, Any]
) -> None:
    entries: list[tuple[str, dict[str, Any]]] = []
    for name, payload in deterministic_baselines.items():
        entries.append((f"baseline:{name}", payload["summary"]["overall"]))
    for mode, payload in all_modes.items():
        entries.append((f"mode:{mode}", payload["summary"]["overall"]))
    if not entries:
        return
    print("\n=== BENCHMARK SUMMARY TABLE ===", flush=True)
    header = (
        f"{'Run':<32} {'SR':>6} {'NF-SR':>6} {'OSR':>6} {'sOSR':>6} "
        f"{'SPL':>6} {'Stale':>5} {'Unrch':>5} {'Fb':>4}"
    )
    print(header, flush=True)
    print("-" * len(header), flush=True)
    for label, ov in entries:
        print(
            f"{label:<32} "
            f"{ov.get('sr', 0.0):>6.3f} "
            f"{ov.get('no_fallback_sr', 0.0):>6.3f} "
            f"{ov.get('osr', 0.0):>6.3f} "
            f"{ov.get('avg_subgoal_osr', 0.0):>6.3f} "
            f"{ov.get('spl', 0.0):>6.3f} "
            f"{ov.get('stale_flip_errors', 0):>5d} "
            f"{ov.get('total_unreachable_moves', 0):>5d} "
            f"{ov.get('total_fallbacks', 0):>4d}",
            flush=True,
        )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    t0 = time.monotonic()
    map_dir = Path(args.map_dir)
    graph = build_scene_graph_from_map_dir(map_dir)
    tasks = build_30_static_tasks(graph)[: max(1, int(args.limit))]

    timeout_s = 60.0 if args.thinking_mode == "high" else 25.0
    max_tokens = 1024 if args.thinking_mode == "high" else 360
    extra = _extra_body(args.model, args.thinking_mode)

    deterministic_baselines: dict[str, Any] = {}
    if args.include_deterministic_baselines:
        for b_name, b_cls in (
            ("det_m1_greedy", DeterministicGreedyM1Planner),
            ("det_recall_goto_xyz", DeterministicRecallGotoXyzPlanner),
        ):
            b_t0 = time.monotonic()
            b_rows = [
                evaluate_single_task_offline(
                    graph, spec, b_cls(), memory_mode="hierarchical_6zone"
                )
                for spec in tasks
            ]
            b_rows.sort(key=lambda r: r["task_id"])
            b_summary = aggregate_suite_results(b_rows)
            deterministic_baselines[b_name] = {
                "wall_s": round(time.monotonic() - b_t0, 2),
                "summary": b_summary,
                "tasks": b_rows,
            }
            print(
                f"=== BASELINE {b_name} SUMMARY ({deterministic_baselines[b_name]['wall_s']}s) ===",
                flush=True,
            )
            print(json.dumps(b_summary, indent=2, ensure_ascii=False), flush=True)

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

    _print_summary_table(all_modes, deterministic_baselines)

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    report = {
        "git_commit": _resolve_git_commit(),
        "map_json_sha256": _map_json_sha256(map_dir),
        "model": args.model,
        "provider": args.provider,
        "thinking_mode": args.thinking_mode,
        "num_tasks": len(tasks),
        "total_wall_s": round(time.monotonic() - t0, 2),
        "deterministic_baselines": deterministic_baselines,
        "modes": all_modes,
    }
    out_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Saved full 30-task benchmark report to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
