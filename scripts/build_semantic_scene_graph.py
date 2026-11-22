#!/usr/bin/env python3
"""Build a keyframe-anchored semantic scene graph from a saved MapStore."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from embodied_navigation.localization_mapping.backend.map_store import MapStore
from embodied_navigation.localization_mapping.semantic_mapping import (
    build_scene_graph_from_map_store,
)


def _load_rgb_index(jsonl_path: Path) -> dict[int, str]:
    index: dict[int, str] = {}
    for line in jsonl_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        row = json.loads(line)
        stamp_ns = int(row["stamp_ns"])
        rgb_path = row.get("rgb_path") or row.get("path") or row.get("image_path")
        if rgb_path:
            index[stamp_ns] = str(rgb_path)
    return index


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, required=True, help="MapStore directory containing map.json")
    parser.add_argument("--rgb-index-jsonl", type=Path, default=None, help="Optional JSONL mapping stamp_ns to RGB image path")
    parser.add_argument("--output", type=Path, default=None, help="Output scene_graph.json path")
    parser.add_argument("--summary", action="store_true", help="Print compact agent summary JSON to stdout")
    args = parser.parse_args(argv)

    map_store = MapStore.load(args.map_dir)
    rgb_index = _load_rgb_index(args.rgb_index_jsonl) if args.rgb_index_jsonl else None
    scene_graph = build_scene_graph_from_map_store(map_store, rgb_index=rgb_index)

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
