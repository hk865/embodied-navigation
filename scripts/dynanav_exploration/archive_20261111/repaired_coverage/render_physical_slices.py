#!/usr/bin/env python3
"""Render explicitly configured physical-region display slices.

Inputs are only this generation's measured_cloud.npz, an existing actual
postVIO trajectory CSV, and a JSON configuration. No occupancy/free fill,
scene geometry, robot controls, collector, or implicit upper-floor model
is used. The configuration supplies every region's bounds and height bands.
"""

import argparse
import csv
import json
import re
import textwrap
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import numpy as np


RUN_ID = "native_exploration_20261111T102621Z"
OUTPUT_ROOT = Path(__file__).resolve().parent
EVIDENCE_ROOT = OUTPUT_ROOT.parent / "repaired_evidence"
CSV_COLUMNS = ("source_time_s", "x_m", "y_m", "z_m", "yaw_rad")


def ordered_band(value, name):
    band = np.asarray(value, dtype=float)
    if band.shape != (2,) or not np.isfinite(band).all() or band[0] >= band[1]:
        raise ValueError(f"{name} must be two finite increasing absolute map-z values")
    return band


def validate_region(region):
    required = ("name", "xy_roi_m", "point_z_band_m", "trace_z_band_m", "evidence")
    missing = [key for key in required if key not in region]
    if missing:
        raise ValueError(f"Region is missing explicit configuration fields: {missing}")
    if not isinstance(region["name"], str) or not region["name"].strip():
        raise ValueError("Region name must be nonempty")
    roi = np.asarray(region["xy_roi_m"], dtype=float)
    if (roi.shape != (4,) or not np.isfinite(roi).all()
            or roi[0] >= roi[1] or roi[2] >= roi[3]):
        raise ValueError("xy_roi_m must be [x_min, x_max, y_min, y_max]")
    # An explicit return band is authoritative. It can represent a spatially
    # varying near-floor cluster without claiming one exact physical floor z.
    floor = None if "floor_z_m" not in region else float(region["floor_z_m"])
    if floor is not None and not np.isfinite(floor):
        raise ValueError("floor_z_m must be finite when supplied")
    if "floor_z_band_m" in region:
        floor_band = ordered_band(region["floor_z_band_m"], "floor_z_band_m")
    else:
        if floor is None or "floor_near_halfwidth_m" not in region:
            raise ValueError("Supply floor_z_band_m, or floor_z_m plus floor_near_halfwidth_m")
        halfwidth = float(region["floor_near_halfwidth_m"])
        if not np.isfinite(halfwidth) or halfwidth <= 0:
            raise ValueError("floor_near_halfwidth_m must be positive and finite")
        floor_band = np.array([floor - halfwidth, floor + halfwidth])
    point_band = ordered_band(region["point_z_band_m"], "point_z_band_m")
    trace_band = ordered_band(region["trace_z_band_m"], "trace_z_band_m")
    evidence = region["evidence"]
    if isinstance(evidence, list):
        evidence = " ".join(str(item) for item in evidence)
    if not isinstance(evidence, str) or not evidence.strip():
        raise ValueError("Region evidence must be nonempty text or a list of text")
    return roi, floor, floor_band, point_band, trace_band, evidence


def load_trace(path):
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != list(CSV_COLUMNS):
            raise ValueError(f"Trajectory CSV must contain exactly {CSV_COLUMNS}")
        trace = np.asarray([[float(row[key]) for key in CSV_COLUMNS] for row in reader],
                           dtype=float)
    if trace.ndim != 2 or trace.shape[1] != 5 or len(trace) < 1:
        raise ValueError("Trajectory CSV must contain measured samples")
    if not np.isfinite(trace).all() or not np.all(np.diff(trace[:, 0]) > 0):
        raise ValueError("Trajectory samples must be finite with distinct increasing source times")
    return trace


def plot_trace(ax, trace, selected):
    # Adjacent actual samples only: excluded height intervals never get joined.
    if len(trace) > 1:
        segments = np.stack((trace[:-1, 1:3], trace[1:, 1:3]), axis=1)
        ax.add_collection(LineCollection(segments, colors="#9b9b9b", linewidths=1.4,
                                         alpha=0.82, zorder=4))
        selected_segments = selected[:-1] & selected[1:]
        ax.add_collection(LineCollection(segments[selected_segments], colors="#e77824",
                                         linewidths=2.1, zorder=5))
    # Points also show isolated selected measurements without inventing a segment.
    ax.scatter(trace[selected, 1], trace[selected, 2], c="#e77824", s=1.5,
               linewidths=0, zorder=5)
    ax.scatter(trace[0, 1], trace[0, 2], c="#337b51", marker="s", s=32,
               edgecolors="white", linewidths=0.6, zorder=6)
    ax.scatter(trace[-1, 1], trace[-1, 2], facecolors="none", edgecolors="#5a5050",
               marker="o", s=48, linewidths=1.5, zorder=6)


def render_region(xyz, trace, region, snapshot, destination):
    roi, floor, floor_band, point_band, trace_band, evidence = validate_region(region)
    finite = np.isfinite(xyz).all(axis=1)
    in_xy = finite & (xyz[:, 0] >= roi[0]) & (xyz[:, 0] <= roi[1])
    in_xy &= (xyz[:, 1] >= roi[2]) & (xyz[:, 1] <= roi[3])
    in_slice = in_xy & (xyz[:, 2] >= point_band[0]) & (xyz[:, 2] <= point_band[1])
    near = in_slice & (xyz[:, 2] >= floor_band[0]) & (xyz[:, 2] <= floor_band[1])
    above = in_slice & (xyz[:, 2] > floor_band[1])
    below = in_slice & (xyz[:, 2] < floor_band[0])
    trace_selected = (trace[:, 3] >= trace_band[0]) & (trace[:, 3] <= trace_band[1])
    trace_xy = (trace[:, 1] >= roi[0]) & (trace[:, 1] <= roi[1])
    trace_xy &= (trace[:, 2] >= roi[2]) & (trace[:, 2] <= roi[3])

    width = roi[1] - roi[0]
    height = roi[3] - roi[2]
    figure_width = 10.8
    figure_height = min(17.0, max(9.3, 8.2 * height / width + 3.8))
    fig, ax = plt.subplots(figsize=(figure_width, figure_height))
    fig.subplots_adjust(left=.09, right=.97, top=.86, bottom=.25)
    ax.set_facecolor("#fbfbfa")
    # Draw measured points directly. Blank pixels are simply unplotted; they
    # are never populated or labeled as free space.
    for mask, color, alpha in ((below, "#b39bc2", .5),
                               (above, "#354b65", .48),
                               (near, "#4aa9bc", .62)):
        points = xyz[mask]
        ax.scatter(points[:, 0], points[:, 1], color=color, s=.42, alpha=alpha,
                   linewidths=0, rasterized=True, zorder=2)
    plot_trace(ax, trace, trace_selected)
    for note in region.get('annotations', []):
        # Optional descriptions refer only to reviewed current-generation
        # evidence; these are labels, not map fill or navigation targets.
        ax.annotate(note['text'], xy=note['xy_m'], xytext=note['text_xy_m'],
                    fontsize=8, ha=note.get('ha', 'left'), va='center', zorder=8,
                    bbox={'boxstyle':'round,pad=.35','fc':'#fff6dc','ec':'#ae8e51','alpha':.94},
                    arrowprops={'arrowstyle':'->','color':'#947439','lw':1.1})
    ax.set(xlim=roi[:2], ylim=roi[2:], xlabel="estimated map x (m)",
           ylabel="estimated map y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(color="#b9b9b9", alpha=.24, linewidth=.6)
    name = region["name"].strip()
    fig.suptitle(f"{name} | {snapshot}\nMeasured-point 2D height slice + actual estimated trajectory",
                 fontsize=14, y=.976)
    floor_reference = "" if floor is None else f"; floor reference {floor:.6f} m"
    subtitle = (f"Point z [{point_band[0]:.3f}, {point_band[1]:.3f}] m; "
                f"near-floor return band [{floor_band[0]:.3f}, {floor_band[1]:.3f}] m{floor_reference}\n"
                f"Orange go2_imu z [{trace_band[0]:.3f}, {trace_band[1]:.3f}] m; "
                f"actual postVIO samples through {trace[-1, 0]:.1f} s")
    fig.text(.5, .919, subtitle, ha="center", va="top", fontsize=9)
    legend = [
        Line2D([0], [0], marker=".", linestyle="none", color="#4aa9bc", markersize=7,
               label="measured returns in explicit near-floor z band"),
        Line2D([0], [0], marker=".", linestyle="none", color="#354b65", markersize=7,
               label="measured returns above floor-near band"),
        Line2D([0], [0], color="#9b9b9b", lw=1.5,
               label="entire actual postVIO trajectory (clipped to XY ROI)"),
        Line2D([0], [0], color="#e77824", lw=2.1,
               label="trajectory samples in this explicit pose-z band"),
        Line2D([0], [0], marker="s", color="#337b51", linestyle="none", markersize=5,
               label="first actual sample"),
        Line2D([0], [0], marker="o", markerfacecolor="none", color="#5a5050",
               linestyle="none", markersize=6, label="latest actual sample"),
    ]
    if below.any():
        legend.insert(2, Line2D([0], [0], marker=".", linestyle="none", color="#b39bc2",
                                markersize=7, label="measured returns below floor-near band"))
    fig.legend(handles=legend, loc="lower left", bbox_to_anchor=(.075, .139),
               fontsize=8, frameon=False, ncol=1)
    evidence_lines = textwrap.wrap("Configured evidence: " + evidence, width=115)
    if len(evidence_lines) > 3:
        evidence_lines = evidence_lines[:2] + ["Full evidence is preserved in the slice JSON."]
    footer = ["Pure display slice: point projections certify neither physical support nor route clearance.",
              "No free-space fill. Blank areas have no plotted return; they are not a free/unknown classification.",
              "Pose origin is go2_imu, not floor height. A z band alone does not establish a physical storey.",
              "Gray/orange lines connect adjacent measured samples; orange gaps are not bridged."]
    fig.text(.085, .129, "\n".join(footer + evidence_lines), fontsize=8,
             va="top", linespacing=1.35)
    fig.savefig(destination, dpi=170, facecolor="white")
    plt.close(fig)

    selected_segments = trace_selected[:-1] & trace_selected[1:]
    segment_distances = np.linalg.norm(np.diff(trace[:, 1:4], axis=0), axis=1)
    return {
        "name": name, "image": str(destination), "configuration": region,
        "effective_floor_return_band_m": floor_band.tolist(),
        "floor_band_source": "explicit floor_z_band_m" if "floor_z_band_m" in region
                             else "floor_z_m +/- floor_near_halfwidth_m",
        "point_counts": {"raw": len(xyz), "nonfinite_not_plotted": int((~finite).sum()),
                         "within_xy_roi": int(in_xy.sum()), "within_point_slice": int(in_slice.sum()),
                         "floor_near": int(near.sum()), "above_floor_near": int(above.sum()),
                         "below_floor_near": int(below.sum())},
        "trajectory": {"source": "livo_post_vio", "map_frame": "dynanav_lio_map",
                       "pose_origin": "go2_imu", "all_actual_samples": len(trace),
                       "samples_in_pose_z_band": int(trace_selected.sum()),
                       "samples_in_pose_z_band_and_xy_roi": int((trace_selected & trace_xy).sum()),
                       "first_source_time_s": float(trace[0, 0]),
                       "last_source_time_s": float(trace[-1, 0]),
                       "selected_adjacent_segment_length_m": float(segment_distances[selected_segments].sum()),
                       "max_source_sample_gap_s": float(np.diff(trace[:, 0]).max()) if len(trace) > 1 else None,
                       "length_is_not_coverage": True},
        "evidence": evidence,
        "limitations": ["Pure display slice; height/XY bounds come only from explicit configuration.",
                        "Measured points are not support, footprint-clearance, traversability or completion proof.",
                        "No free-space fill; blank pixels are not classified.",
                        "Pose-origin z is go2_imu height, distinct from configured floor z.",
                        "A physical upper region is not assigned by height alone.",
                        "Full raw point set is preserved; no point sampling is used for this rendering."]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", required=True, help="Existing current-generation snapshot, e.g. r14")
    parser.add_argument("--config", required=True, type=Path, help="Explicit region configuration JSON")
    parser.add_argument("--out", required=True, type=Path, help="Output directory within repaired_coverage")
    parser.add_argument("--cloud-file", type=Path, help="Optional frozen current-generation NPZ; generation/stage/frame are validated")
    parser.add_argument("--trajectory-file", type=Path, help="Optional previously extracted actual postVIO CSV")
    args = parser.parse_args()
    if not re.fullmatch(r"r(?:[0-9]+[a-z]?|final)", args.snapshot):
        raise ValueError("Snapshot must be an r-number; external input paths are not accepted")
    out = args.out.resolve()
    if not out.is_relative_to(OUTPUT_ROOT):
        raise ValueError("Output must be under repaired_coverage")
    config = json.loads(args.config.read_text())
    if config.get("run_id") != RUN_ID:
        raise ValueError("Configuration must explicitly identify the current repaired generation")
    regions = config.get("regions")
    if not isinstance(regions, list) or not regions:
        raise ValueError("At least one explicitly configured physical region is required")
    for region in regions:
        validate_region(region)

    cloud_path = args.cloud_file or EVIDENCE_ROOT / args.snapshot / "measured_cloud.npz"
    trace_path = args.trajectory_file or OUTPUT_ROOT / args.snapshot / "actual_postvio_trajectory.csv"
    with np.load(cloud_path, allow_pickle=False) as cloud:
        if (str(cloud["run_id"].item()) != RUN_ID
                or str(cloud["cloud_map_frame"].item()) != "dynanav_lio_map"
                or str(cloud["cloud_stage"].item()) != "LIO"):
            raise ValueError("Cloud must be this generation's LIO measurements in dynanav_lio_map")
        xyz = cloud["xyz"].copy()
        source_stamps = cloud["source_stamp_ns"].copy()
    if xyz.ndim != 2 or xyz.shape[1] != 3 or source_stamps.size == 0:
        raise ValueError("Malformed measured cloud")
    trace = load_trace(trace_path)
    out.mkdir(parents=True, exist_ok=True)
    records = []
    used_slugs = set()
    for index, region in enumerate(regions, 1):
        slug = re.sub(r"[^a-zA-Z0-9_-]+", "_", region["name"].strip()).strip("_").lower()
        slug = slug or f"region_{index}"
        if slug in used_slugs:
            raise ValueError(f"Region names produce duplicate output filename {slug}")
        used_slugs.add(slug)
        records.append(render_region(xyz, trace, region, args.snapshot,
                                     out / f"{slug}_physical_slice_2d.png"))
    result = {
        "schema": "physical_region_display_slices/v1", "run_id": RUN_ID,
        "snapshot": args.snapshot, "read_only_inputs": True,
        "sources": {"measured_cloud": str(cloud_path), "actual_postvio_csv": str(trace_path),
                    "configuration": str(args.config.resolve())},
        "cloud_stage": "LIO", "cloud_map_frame": "dynanav_lio_map",
        "cloud_first_source_time_s": float(source_stamps.min() / 1e9),
        "cloud_last_source_time_s": float(source_stamps.max() / 1e9),
        "cloud_and_trajectory_not_same_snapshot_frame": True,
        "input_config": config, "regions": records,
        "ground_truth_or_scene_geometry_read": False,
        "occupancy_or_free_fill_used": False, "source_files_modified": False,
    }
    record_path = out / "physical_slice_record.json"
    record_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"record": str(record_path),
                      "images": [region["image"] for region in records]}, indent=2))


if __name__ == "__main__":
    main()
