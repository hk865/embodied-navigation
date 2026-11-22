#!/usr/bin/env python3
"""Build a keyframe-anchored semantic scene graph from a saved MapStore directory."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from embodied_navigation.localization_mapping.semantic_mapping import (
    build_scene_graph_from_map_dir,
    load_rgb_index_report,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, required=True, help="MapStore directory containing map.json")
    parser.add_argument("--rgb-index-jsonl", type=Path, default=None, help="Optional JSONL mapping timestamp to RGB image path")
    parser.add_argument("--output", type=Path, default=None, help="Output scene_graph.json path")
    parser.add_argument("--summary", action="store_true", help="Print compact agent summary JSON to stdout")
    args = parser.parse_args(argv)

    rgb_index = None
    if args.rgb_index_jsonl is not None:
        rgb_report = load_rgb_index_report(args.rgb_index_jsonl)
        rgb_index = rgb_report.index
        if rgb_report.skipped_count:
            line_nums = [entry["line"] for entry in rgb_report.skipped_lines]
            sys.stderr.write(
                f"rgb-index: skipped {rgb_report.skipped_count} invalid record(s) at lines {line_nums}\n"
            )

    scene_graph = build_scene_graph_from_map_dir(args.map_dir, rgb_index=rgb_index)

    if args.output is not None:
        scene_graph.save(args.output)
    if args.summary or args.output is None:
        sys.stdout.write(
            json.dumps(
                scene_graph.compact_agent_summary(),
                indent=2,
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
