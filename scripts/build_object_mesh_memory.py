#!/usr/bin/env python3
"""基于 V4 后端全局地图与 RGB 帧构建物体级 3D Mesh 空间记忆库的薄 CLI。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))

from embodied_navigation.localization_mapping.semantic_mapping.object_mesh_memory import (  # noqa: E402
    ObjectMeshConfig,
    build_object_mesh_memory_from_map,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="从 V4 SLAM MapStore 构建物体级 3D Surface Nets Mesh 记忆库")
    parser.add_argument("--map-dir", required=True, type=Path, help="后端保存的 map 目录（含 map.json + kf_*.npz）")
    parser.add_argument("--output-dir", required=True, type=Path, help="输出记忆库目录（建议在数据卷）")
    parser.add_argument("--rgb-index", type=Path, default=None, help="capture/interactive_frames/index.jsonl 路径")
    parser.add_argument("--preflight-json", type=Path, default=None, help="observer/preflight.json 相机标定路径")
    parser.add_argument("--cloud-voxel-m", type=float, default=0.05, help="全局去重体素分辨率（米）")
    parser.add_argument("--mesh-voxel-m", type=float, default=0.05, help="单物体 Surface Nets 网格分辨率（米）")
    parser.add_argument("--keyframe-stride", type=int, default=1, help="关键帧抽样步长")
    args = parser.parse_args(argv)

    cfg = ObjectMeshConfig(cloud_voxel_m=args.cloud_voxel_m, mesh_voxel_m=args.mesh_voxel_m)
    summary = build_object_mesh_memory_from_map(
        args.map_dir,
        args.output_dir,
        rgb_index_jsonl=args.rgb_index,
        preflight_json=args.preflight_json,
        config=cfg,
        keyframe_stride=args.keyframe_stride,
    )
    brief = {k: v for k, v in summary.items() if k != "objects"}
    print(json.dumps(brief, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
