#!/usr/bin/env python3
"""Query one observed scene-map snapshot; no model, ROS or simulator is started.

The query/filter logic lives in
``embodied_navigation.localization_mapping.semantic_mapping.query``; this file is
the CLI wrapper (argument parsing, snapshot loading, output formatting).
"""

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from embodied_navigation.localization_mapping.semantic_mapping import read_snapshot
from embodied_navigation.localization_mapping.semantic_mapping.query import (  # noqa: F401 - re-exported for existing importers/tests
    context_text,
    select,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "map", type=Path, help="Committed scene_map.json (full or compact snapshot)"
    )
    parser.add_argument(
        "--label", help="Exact recorded class; no synonyms or inferred labels"
    )
    parser.add_argument("--near", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument(
        "--radius", type=float, help="Nonnegative metres in the recorded map frame"
    )
    parser.add_argument("--include-expired", action="store_true")
    parser.add_argument(
        "--context",
        action="store_true",
        help="Compact deterministic context instead of JSON",
    )
    args = parser.parse_args(argv)
    if (args.near is None) != (args.radius is None):
        parser.error("--near and --radius must be supplied together")
    if args.near is not None and (
        not all(math.isfinite(x) for x in args.near)
        or not math.isfinite(args.radius)
        or args.radius < 0
    ):
        parser.error(
            "--near requires finite XYZ and --radius must be finite and nonnegative"
        )
    try:
        result = select(
            read_snapshot(args.map),
            label=args.label,
            near=args.near,
            radius=args.radius,
            include_expired=args.include_expired,
        )
        output = (
            context_text(result)
            if args.context
            else json.dumps(
                result, sort_keys=True, indent=2, ensure_ascii=False, allow_nan=False
            )
            + "\n"
        )
    except (OSError, ValueError, TypeError, AttributeError) as error:
        parser.error("cannot query map snapshot: " + str(error))
    sys.stdout.write(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
