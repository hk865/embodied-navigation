#!/usr/bin/env python3
"""Offline ADR-0004 3-layer SLAM backend trajectory evaluation CLI."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.evaluation import (
    evaluate_replay,
    load_input_json_keyframes,
    load_map_keyframes,
    load_truth_stream,
    replay_backend,
    save_evaluation_bundle,
)


def _git_provenance(repo_root: Path) -> dict:
    def _run(args: list[str]) -> str:
        try:
            return subprocess.check_output(args, cwd=repo_root, text=True, stderr=subprocess.DEVNULL).strip()
        except Exception:
            return ""

    return {
        "branch": _run(["git", "rev-parse", "--abbrev-ref", "HEAD"]),
        "commit": _run(["git", "rev-parse", "HEAD"]),
        "dirty": bool(_run(["git", "status", "--porcelain"])),
    }


def _save_plots(output_dir: Path, arrays: dict) -> list[str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return []

    saved: list[str] = []
    t = arrays["stamps_s"] - arrays["stamps_s"][0]

    # Plot 1: Layer 1 ATE & Z & Gravity Tilt over time (frontend vs online vs final)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
    for name, label in (("frontend", "Frontend"), ("online_adopted", "Online Backend"), ("final", "Final Graph")):
        axes[0].plot(t, arrays[f"{name}_init_err_3d_m"], label=label, linewidth=1.2)
        axes[1].plot(t, arrays[f"{name}_init_err_z_signed_m"], label=label, linewidth=1.2)
        axes[2].plot(t, arrays[f"{name}_init_tilt_deg"], label=label, linewidth=1.2)
    axes[0].set_ylabel("3D ATE (m)")
    axes[0].legend(loc="upper left")
    axes[0].grid(True, alpha=0.3)
    axes[1].set_ylabel("Signed Z err (m)")
    axes[1].grid(True, alpha=0.3)
    axes[2].set_ylabel("Gravity Tilt (deg)")
    axes[2].set_xlabel("Elapsed time (s)")
    axes[2].grid(True, alpha=0.3)
    fig.suptitle("Layer 1: Trajectory Errors (Initial-Window SE(3) Aligned)")
    fig.tight_layout()
    p1 = output_dir / "layer1_errors.png"
    fig.savefig(p1, dpi=140)
    plt.close(fig)
    saved.append(p1.name)

    # Plot 2: Layer 2 evolution + Layer 3 per-second revision fan chart
    fig, axes = plt.subplots(2, 1, figsize=(11, 7.0))
    st = arrays["snapshot_stamps_s"] - arrays["stamps_s"][0]
    axes[0].plot(st, arrays["evo_ate_rmse_m"], label="Prefix ATE RMSE", linewidth=1.3)
    axes[0].plot(st, arrays["evo_ate_max_m"], label="Prefix ATE Max", linewidth=1.0, alpha=0.8)
    axes[0].plot(st, arrays["evo_z_rmse_m"], label="Prefix Z RMSE", linewidth=1.1)
    axes[0].set_ylabel("Error (m)")
    axes[0].set_xlabel("Optimization snapshot time (s)")
    axes[0].set_title("Layer 2: Per-Optimization Prefix Trajectory Evolution")
    axes[0].legend(loc="upper left")
    axes[0].grid(True, alpha=0.3)

    bt = arrays["bucket_t_s"]
    axes[1].fill_between(bt, arrays["bucket_err_p05_m"], arrays["bucket_err_p95_m"], alpha=0.22, label="P5-P95")
    axes[1].fill_between(bt, arrays["bucket_err_p25_m"], arrays["bucket_err_p75_m"], alpha=0.45, label="IQR (P25-P75)")
    axes[1].plot(bt, arrays["bucket_err_p50_m"], linewidth=1.3, label="Median")
    axes[1].plot(bt, arrays["bucket_err_max_m"], linewidth=0.9, linestyle="--", label="Max")
    axes[1].set_ylabel("Historical Pose Error (m)")
    axes[1].set_xlabel("Pose timestamp bucket (s)")
    axes[1].set_title("Layer 3: Per-Second Historical Pose Revision Distribution")
    axes[1].legend(loc="upper left")
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    p2 = output_dir / "layer2_3_evolution_revisions.png"
    fig.savefig(p2, dpi=140)
    plt.close(fig)
    saved.append(p2.name)
    return saved


def _print_summary(summary: dict) -> None:
    rt = summary["runtime"]
    print(
        f"KFs={rt['keyframes']} loops={rt['loops_accepted']} (rej_odom={rt['loops_rejected_odom']}) "
        f"errors={rt['errors']} wall={rt['total_wall_s']:.2f}s (p95={rt['step_wall_p95_ms']:.1f}ms)"
    )
    for align_key, label in (("layer1_init_se3", "Init-SE3"), ("layer1_umeyama_se3", "Umeyama-SE3")):
        print(f"  [{label}]")
        for traj in ("frontend", "online_adopted", "final"):
            m = summary[align_key][traj]
            print(
                f"    {traj:14s} | ATE={m['ate_3d_m']['rmse']:.4f}m (max {m['ate_3d_m']['max']:.4f}) "
                f"| Z={m['ate_z_m']['rmse']:.4f}m (max {m['ate_z_m']['max']:.4f}) "
                f"| AOE={m['aoe_deg']['rmse']:.2f}deg | Tilt={m['tilt_deg']['rmse']:.2f}deg (max {m['tilt_deg']['max']:.2f})"
            )
    l2 = summary["layer2_evolution"]
    l3 = summary["layer3_revisions"]
    print(
        f"  [Layer 2] snaps={l2['snapshots']} (loops={l2['loop_snapshots']}) "
        f"peak_ATE_rmse={l2['peak_prefix_ate_rmse_m']:.4f}m peak_tilt={l2['peak_prefix_tilt_max_deg']:.2f}deg "
        f"neg_steps={l2['negative_loop_steps']} pos_steps={l2['positive_loop_steps']}"
    )
    print(
        f"  [Layer 3] median_IQR={l3['median_iqr_m']:.4f}m p95_IQR={l3['p95_iqr_m']:.4f}m "
        f"max_shift={l3['max_shift_m']:.4f}m max_jitter={l3['max_cum_jitter_m']:.4f}m"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Offline 3-layer SLAM backend evaluation (ADR-0004).")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--map-dir", type=Path, help="Saved MapStore directory (map.json + kf_*.npz)")
    src.add_argument("--input-json", type=Path, help="Offline input.json manifest")
    parser.add_argument("--truth", type=Path, required=True, help="capture/truth_scoring_stream.jsonl")
    parser.add_argument("--visual-dir", type=Path, default=None, help="Optional directory of <stamp_ns>.npz visual frames")
    parser.add_argument("--config", type=Path, default=None, help="BackendConfig JSON overlay")
    parser.add_argument("--output-dir", type=Path, required=True, help="Output directory on data disk")
    parser.add_argument("--init-window-s", type=float, default=5.0, help="Initial SE(3) alignment window (s)")
    parser.add_argument("--bucket-s", type=float, default=1.0, help="Layer 3 revision bucket size (s)")
    parser.add_argument("--snapshot-stride", type=int, default=50, help="Periodic snapshot stride in KFs")
    parser.add_argument("--no-tum", action="store_true", help="Skip TUM trajectory export")
    parser.add_argument("--plot", action="store_true", help="Save PNG diagnostic plots if matplotlib is installed")
    args = parser.parse_args(argv)

    keyframes = (
        load_map_keyframes(args.map_dir, visual_dir=args.visual_dir)
        if args.map_dir
        else load_input_json_keyframes(args.input_json, visual_dir=args.visual_dir)
    )
    truth = load_truth_stream(args.truth)
    config = BackendConfig.from_json(args.config) if args.config else BackendConfig()
    backend = SlamBackend(config)

    record = replay_backend(backend, keyframes, snapshot_stride=args.snapshot_stride)
    summary, arrays = evaluate_replay(record, truth, init_window_s=args.init_window_s, bucket_s=args.bucket_s)

    save_evaluation_bundle(args.output_dir, summary, arrays, export_tum=not args.no_tum)
    plots = _save_plots(args.output_dir, arrays) if args.plot else []
    manifest = {
        "provenance": _git_provenance(Path(__file__).resolve().parents[1]),
        "source": str(args.map_dir or args.input_json),
        "truth": str(args.truth),
        "config_path": None if args.config is None else str(args.config),
        "plots": plots,
    }
    (args.output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    _print_summary(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
