"""Offline measured-window audit; no ground truth or navigation output is used."""
import argparse
import json
from pathlib import Path

import numpy as np

from ..backend.map_store import MapStore
from ..backend.submaps import SubmapConfig, build_submap


def audit(directory, config=None):
    store = MapStore.load(directory)
    frames = list(store.keyframes.values())
    rows = []
    for i, frame in enumerate(frames):
        window = build_submap(frames[:i+1], frame, config or SubmapConfig())
        selected = [store.keyframes[key] for key in window.source_keys]
        rows.append((len(selected), len(window.points_body),
                     float(np.ptp([f.T_odom_body[2, 3] for f in selected])),
                     sum(len(f.points_lidar) for f in selected)))
    if not rows:
        return {"keyframes": 0}
    values = np.asarray(rows)
    result = {"keyframes": len(rows), "visual_observations": sum(f.visual is not None for f in frames),
              "single_frame_count": int(np.sum(values[:, 0] == 1)),
              "total_input_points": int(values[:, 3].sum()),
              "total_output_voxels": int(values[:, 1].sum())}
    for i, name in enumerate(("frame_count", "point_count", "height_span_m")):
        result[name] = dict(min=float(values[:, i].min()), median=float(np.median(values[:, i])),
                            max=float(values[:, i].max()))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(audit(args.map), indent=2))


if __name__ == "__main__":
    main()
