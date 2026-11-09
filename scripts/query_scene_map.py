#!/usr/bin/env python3
"""Query one observed scene-map snapshot; no model, ROS or simulator is started."""

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from embodied_navigation.localization_mapping.semantic_mapping import read_snapshot


def select(snapshot, *, label=None, near=None, radius=None, include_expired=False):
    """Filter existing records only; never infer missing objects, floors or relations."""
    if not isinstance(snapshot.get("objects"), list):
        raise ValueError("map snapshot must contain an objects list")  # noqa: TRY004 - Preserve the existing ValueError input-validation contract.
    objects = []
    for obj in snapshot["objects"]:
        if not isinstance(obj, dict):
            raise ValueError("object record must be an object")  # noqa: TRY004 - Preserve the existing ValueError input-validation contract.
        if label is not None and obj.get("label") != label:
            continue
        if obj.get("expired") is True and not include_expired:
            continue
        if near is not None:
            position = obj.get("position")
            if (
                not isinstance(position, list)
                or len(position) != 3
                or any(
                    type(x) not in (int, float) or not math.isfinite(x)
                    for x in position
                )
            ):
                raise ValueError("spatial query requires a finite XYZ object position")
            if math.dist(position, near) > radius:
                continue
        objects.append(obj)
    objects.sort(key=lambda x: x.get("object_id", ""))
    return {
        "schema": "observed_scene_map_query/v1",
        "map": {
            k: v for k, v in snapshot.items() if k not in ("objects", "static_points")
        },
        "geometry_payload_omitted": "static_points" in snapshot,
        "filters": {
            "label": label,
            "near": near,
            "radius": radius,
            "include_expired": include_expired,
        },
        "objects": objects,
        "scope": "retained observed snapshot; no inferred floor names, topology, full-scene completion or control authority",
    }


def context_text(result):
    """Deterministic compact context with explicit evidence and completeness scope."""
    encode = lambda value: json.dumps(
        value, sort_keys=True, ensure_ascii=False, allow_nan=False
    )
    meta = result["map"]
    lines = [
        "observed_scene_map_context/v1",
        "map="
        + encode(
            {
                key: meta.get(key)
                for key in (
                    "map_id",
                    "epoch",
                    "frame_id",
                    "revision",
                    "source_ns",
                    "query_clock_ns",
                )
            }
        ),
        "coverage="
        + encode(
            {
                key: meta.get(key)
                for key in (
                    "capacity_complete",
                    "mask_coverage_complete",
                    "semantic_label_coverage_complete",
                    "observation_point_counts",
                    "capacity_rejections",
                    "mask_rejections",
                )
            }
        ),
        "label_coverage_scope=" + encode(meta.get("label_coverage_scope")),
        "sources=" + encode(meta.get("provenance_summary")),
        "scope=" + result["scope"],
        "velocity_caveat=observed visible-centroid difference in revised map coordinates; not rigid-body velocity",
        "filters=" + encode(result["filters"]),
        "matched_objects=" + str(len(result["objects"])),
    ]
    for obj in result["objects"]:
        lines.append(
            "object="
            + encode(
                {
                    key: obj.get(key)
                    for key in (
                        "object_id",
                        "label",
                        "status",
                        "expired",
                        "position",
                        "source_ns",
                        "age_ns",
                        "source_instance_id",
                        "instance_id_scope",
                        "source_epoch",
                        "sensor_frame_id",
                        "calibration_id",
                        "pose_source",
                        "pose_source_ns",
                        "anchor_id",
                        "anchor_revision",
                        "T_map_sensor",
                        "provenance",
                        "velocity",
                        "velocity_semantics",
                    )
                }
            )
        )
    return "\n".join(lines) + "\n"


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
