#!/usr/bin/env python3
"""Read-only renderer for the exported operator occupancy grid (no ROS, no map mutation).

Usage: render_dynanav_operator_grid.py <grid.npz> <manifest.json> <out.png>
Colors: occupied black, free light grey, unknown grey inside the observed ROI,
white only outside it. Unknown cells are never drawn as free.
"""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

SCHEMA = "operator_grid_export/v1"
FREE_COLOR = (222, 222, 222)
OCCUPIED_COLOR = (0, 0, 0)
UNKNOWN_COLOR = (128, 128, 128)
BACKGROUND = (255, 255, 255)
HEADER_H = 68
FOOTER_H = 26
MARGIN = 8
CELL_PX = 4
MAX_SIDE_PX = 4096
MAX_PIXELS = 4096 * 4096


def _fail(message):
    raise RuntimeError("operator grid render refused: " + message)


def _validate(nodes, manifest):
    if not isinstance(manifest, dict) or manifest.get("schema") != SCHEMA:
        _fail("manifest schema mismatch")
    arrays = {}
    for key in ("ix", "iy", "code"):
        if key not in nodes:
            _fail("npz is missing the %s array" % key)
        value = nodes[key]
        if value.ndim != 1:
            _fail("%s must be one-dimensional" % key)
        if not np.issubdtype(value.dtype, np.integer):
            _fail("%s must be an integer array" % key)
        arrays[key] = value
    sizes = {value.size for value in arrays.values()}
    if len(sizes) != 1:
        _fail("ix/iy/code lengths differ")
    codes = set(int(code) for code in np.unique(arrays["code"]))
    if not codes.issubset({0, 1}):
        _fail("code array contains values outside {0 free, 1 occupied}")
    return arrays


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def cell_pixel(ix_val, iy_val, min_ix, max_ix, min_iy, max_iy, scale=CELL_PX):
    """Top-left pixel of one cell inside the grid area; +x right, +y up."""
    row = HEADER_H + (max_iy - iy_val) * scale
    col = MARGIN + (ix_val - min_ix) * scale
    return int(row), int(col)


def render_grid(npz_path, manifest_path, png_path, *, max_side_px=MAX_SIDE_PX, max_pixels=MAX_PIXELS):
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    expected = manifest.get("npz_sha256")
    actual = _sha256(npz_path)
    if expected is None:
        _fail("manifest has no npz_sha256")
    if str(expected) != actual:
        _fail("npz sha256 does not match the manifest (stale or overwritten pair)")
    with np.load(npz_path) as nodes:
        arrays = _validate(nodes, manifest)
        ix, iy, code = arrays["ix"], arrays["iy"], arrays["code"].copy()
    if ix.size:
        min_ix, max_ix = int(ix.min()), int(ix.max())
        min_iy, max_iy = int(iy.min()), int(iy.max())
        grid_w = (max_ix - min_ix + 1) * CELL_PX
        grid_h = (max_iy - min_iy + 1) * CELL_PX
    else:
        min_ix = max_ix = min_iy = max_iy = 0
        grid_w = grid_h = CELL_PX
    width = max(480, grid_w + 2 * MARGIN)
    height = HEADER_H + grid_h + FOOTER_H + MARGIN
    if width > int(max_side_px) or height > int(max_side_px):
        _fail("image side exceeds %d px (grid %dx%d)" % (int(max_side_px), grid_w, grid_h))
    if width * height > int(max_pixels):
        _fail("image exceeds %d pixels" % int(max_pixels))
    frame_label = manifest.get("odometry_map_frame") or manifest.get("map_frame")
    declared = manifest.get("map_frame")
    image = Image.new("RGB", (width, height), BACKGROUND)
    draw = ImageDraw.Draw(image)
    if ix.size:
        draw.rectangle(
            [MARGIN, HEADER_H, MARGIN + grid_w - 1, HEADER_H + grid_h - 1],
            fill=UNKNOWN_COLOR,
        )
    for cell_ix, cell_iy, cell_code in zip(ix, iy, code):
        row, col = cell_pixel(int(cell_ix), int(cell_iy), min_ix, max_ix, min_iy, max_iy)
        color = OCCUPIED_COLOR if int(cell_code) == 1 else FREE_COLOR
        draw.rectangle([col, row, col + CELL_PX - 1, row + CELL_PX - 1], fill=color)
    lines = [
        str(manifest.get("provenance", "operator occupancy grid")),
        "frame: %s (declared map %s) | src clock %s | written %s"
        % (frame_label, declared, manifest.get("source_clock_used"), manifest.get("written_wall_s")),
        "occupied=black free=light unknown=grey | partial=%s | actionable=%s"
        % (manifest.get("partial"), (manifest.get("frontier") or {}).get("actionable")),
        "cells %s | resolution %s m | bounds %s..%s / %s..%s"
        % (manifest.get("counts", {}).get("total"), manifest.get("resolution_m"),
           min_ix, max_ix, min_iy, max_iy),
    ]
    for line_index, line in enumerate(lines):
        draw.text((MARGIN, 4 + line_index * 15), line, fill=(0, 0, 0))
    if not ix.size:
        draw.text((MARGIN, HEADER_H + 8), "no observed map", fill=(0, 0, 0))
    draw.text(
        (MARGIN, height - FOOTER_H + 4),
        "+x right, +y up | 1 cell = %d px | no robot pose marker" % CELL_PX,
        fill=(0, 0, 0),
    )
    image.save(png_path)
    return {
        "cells": int(ix.size),
        "png": str(png_path),
        "bounds": [min_ix, max_ix, min_iy, max_iy],
        "pixels": [width, height],
        "frame_label": frame_label,
        "declared_map_frame": declared,
        "npz_sha256": actual,
        "note": None if ix.size else "no observed map",
    }


def main(argv):
    if len(argv) != 4:
        print("usage: render_dynanav_operator_grid.py <grid.npz> <manifest.json> <out.png>", file=sys.stderr)
        return 2
    print(json.dumps(render_grid(argv[1], argv[2], argv[3]), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
