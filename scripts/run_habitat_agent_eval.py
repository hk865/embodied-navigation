#!/usr/bin/env python3
"""Thin CLI for evaluating Habitat-Sim HM3D with the standalone EmbodiedAgentRuntime."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

from embodied_navigation.agent import (
    HabitatAgentRunner,
    OpenCodeToolChatClient,
    build_default_deepseek_fallback,
    resolve_agent_thinking,
    run_concurrent_scene_sessions,
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
        "--fallback-provider",
        choices=("auto", "none", "deepseek_official"),
        default="auto",
        help="Failover provider on gateway timeout/5xx/429 (auto = deepseek_official for opencode+deepseek).",
    )
    parser.add_argument("--fallback-model", default="deepseek-flash")
    parser.add_argument("--circuit-open-s", type=float, default=120.0)
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
    parser.add_argument(
        "--eval-mode",
        choices=("sequential", "independent", "concurrent_independent"),
        default="sequential",
    )
    parser.add_argument("--concurrent-workers", type=int, default=3)
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
        stream=True,
        extra_body=extra_body,
    )
    fallbacks: list[LlmClientConfig] = []
    want_ds_fb = (
        args.fallback_provider == "deepseek_official"
        or (
            args.fallback_provider == "auto"
            and args.provider in ("opencode", "opencode_go")
            and "deepseek" in str(args.model or "").lower()
        )
    )
    if want_ds_fb:
        fb = build_default_deepseek_fallback(
            cfg,
            thinking_mode=args.thinking_mode,
            model=args.fallback_model,
            timeout_s=timeout_s,
        )
        if fb is not None:
            fallbacks.append(fb)
        elif args.fallback_provider == "deepseek_official":
            raise SystemExit(
                "ERROR: --fallback-provider deepseek_official requested, but no DeepSeek API key "
                "was found in DEEPSEEK_API_KEY or opencode auth.json."
            )
        else:
            print(
                "[WARN] --fallback-provider auto could not find DeepSeek official credentials; "
                "running primary route only.",
                file=sys.stderr,
                flush=True,
            )

    chat_client = OpenCodeToolChatClient(
        cfg,
        fallback_configs=fallbacks,
        circuit_open_s=float(args.circuit_open_s),
    )
    vision_endpoint = str(args.vision_endpoint or "").strip()
    vision_client = (
        VisionToolClient(vision_endpoint, timeout_s=10.0) if vision_endpoint else None
    )
    common_runner_kwargs = {
        "stop_radius_m": float(args.stop_radius_m),
        "vision_enhance_overlay": not bool(args.no_vision_overlay),
        "vision_score_threshold": float(args.vision_score_threshold),
        "vision_fuse_threshold": float(args.vision_fuse_threshold),
        "max_context_tokens": int(args.max_context_tokens),
        "soft_compaction_tokens": int(args.soft_compaction_tokens),
        "enable_vlm_region_summary": not bool(args.no_vlm_region_summary),
    }
    out_dir = Path(args.output_dir)
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

    if args.eval_mode == "concurrent_independent":
        session_specs = []
        for idx, st in enumerate(subtasks, start=1):
            slug = re.sub(r"[^A-Za-z0-9._-]+", "_", st.target_category).strip("_.-") or "target"
            session_specs.append((f"sess_{idx:02d}_{slug}", [st], int(args.seed) + idx * 17))
        reports = run_concurrent_scene_sessions(
            scene_dataset_config=args.scene_dataset,
            scene_id=args.scene_id,
            output_root=out_dir,
            session_specs=session_specs,
            chat_client=chat_client,
            vision_endpoint=vision_endpoint,
            max_workers=int(args.concurrent_workers),
            gt_seed=int(args.seed),
            **common_runner_kwargs,
        )
        print(
            json.dumps(
                {
                    "scene_id": args.scene_id,
                    "eval_mode": "concurrent_independent",
                    "sessions": {
                        k: {
                            "status": v.get("status", "ok"),
                            "sr": v.get("sr", 0.0),
                            "spl": v.get("spl", 0.0),
                            "false_stops": v.get("false_stops", 0),
                            "wall_s": v.get("wall_s", 0.0),
                            "fallback_switches": v.get("runtime_metrics", {}).get(
                                "provider_fallback_switches", 0
                            ),
                            "fallback_served_turns": v.get("runtime_metrics", {}).get(
                                "provider_fallback_served_turns", 0
                            ),
                            **({"error": v["error"]} if "error" in v else {}),
                        }
                        for k, v in sorted(reports.items())
                    },
                },
                indent=2,
                ensure_ascii=False,
            )
        )
        return 0

    runner = HabitatAgentRunner(
        scene_dataset_config=args.scene_dataset,
        scene_id=args.scene_id,
        chat_client=chat_client,
        output_dir=out_dir,
        memory_carryover=not args.no_memory_carryover,
        reset_occupancy_per_subtask=bool(args.reset_occupancy_per_subtask),
        vision_client=vision_client,
        seed=args.seed,
        **common_runner_kwargs,
    )
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
