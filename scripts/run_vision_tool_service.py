#!/usr/bin/env python3
"""Thin CLI launcher for the local vision foundation model HTTP service."""

from __future__ import annotations

import argparse
import sys

from embodied_navigation.perception.vision_tool_server import run_vision_service


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run local vision foundation model service (SAM 3 / YOLOE + CLIP/SigLIP)."
    )
    parser.add_argument(
        "--backend", choices=("sam3", "grounding_dino", "yoloe", "none"), default="sam3"
    )
    parser.add_argument("--sam3-source", default="", help="Path to SAM 3 repository root.")
    parser.add_argument("--sam3-checkpoint", default="", help="Path to sam3.pt checkpoint.")
    parser.add_argument("--gdino-model", default="", help="Path to local GroundingDINO model dir.")
    parser.add_argument("--yoloe-model", default="", help="Path to YOLOE-seg .pt checkpoint.")
    parser.add_argument(
        "--embed-model",
        default="",
        help="Optional local SigLIP model directory or open_clip model name.",
    )
    parser.add_argument("--embed-pretrained", default="openai")
    parser.add_argument("--embed-cache-dir", default="")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8114)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    return run_vision_service(
        backend=args.backend,
        sam3_source=args.sam3_source,
        sam3_checkpoint=args.sam3_checkpoint,
        gdino_model=args.gdino_model,
        yoloe_model=args.yoloe_model,
        embed_model=args.embed_model,
        embed_pretrained=args.embed_pretrained,
        embed_cache_dir=args.embed_cache_dir,
        device=args.device,
        host=args.host,
        port=args.port,
    )


if __name__ == "__main__":
    sys.exit(main())
