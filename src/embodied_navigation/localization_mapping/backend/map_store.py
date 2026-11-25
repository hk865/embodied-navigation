"""Keyframe map: lidar clouds plus their ``T_world_body`` poses.

``world`` is the map frame (in mapping mode the first keyframe's odom frame).
Clouds are stored in the lidar frame, voxel-downsampled once on ``add``, and
moved into the world frame on demand with ``T_world_body @ T_body_lidar``, so
pose-graph updates never rewrite point data.

On-disk layout of ``save``/``load``: ``map.json`` (schema
``embodied_navigation_map/v1``) lists keyframes in insertion order with their
key, ``stamp_ns``, ``T_odom_body``, ``T_world_body``, ``T_body_lidar`` and npz
file name; each ``kf_<epoch>_<sequence>.npz`` holds ``points_lidar`` and
``descriptor``.
"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
from typing import Iterator

import numpy as np

from .keyframes import Keyframe, VisualObservation, transform_points, voxel_downsample

SCHEMA = "embodied_navigation_map/v1"


def _resolve_map_file(directory: Path, rel_file: str | Path) -> Path:
    """Resolve a map-relative NPZ path inside ``directory``, preserving subdirectories."""
    if not isinstance(rel_file, (str, Path)) or not str(rel_file).strip():
        raise ValueError(f"{directory}: empty keyframe file path")
    rel = Path(rel_file)
    if rel.is_absolute():
        raise ValueError(f"{directory}: keyframe file path must be relative: {rel_file}")
    root = directory.resolve()
    candidate = (root / rel).resolve()
    if candidate == root or not candidate.is_relative_to(root):
        raise ValueError(f"{directory}: keyframe file escapes map directory: {rel_file}")
    return candidate


class MapStore:
    def __init__(self, voxel_m: float = 0.2):
        self.voxel_m = voxel_m
        self.keyframes: dict = {}
        self.poses: dict = {}  # key -> T_world_body
        self.descriptors: dict = {}

    def add(self, keyframe: Keyframe, T_world_body: np.ndarray, descriptor: np.ndarray) -> None:
        points = voxel_downsample(keyframe.points_lidar, self.voxel_m)
        self.keyframes[keyframe.key] = replace(keyframe, points_lidar=points)
        self.poses[keyframe.key] = np.array(T_world_body, dtype=np.float64)
        self.descriptors[keyframe.key] = descriptor

    def update_poses(self, poses: dict) -> None:
        """Replace ``T_world_body`` of the given keys (e.g. after optimisation)."""
        for key, pose in poses.items():
            self.poses[key] = np.array(pose, dtype=np.float64)

    def points_world(self, key) -> np.ndarray:
        keyframe = self.keyframes[key]
        return transform_points(self.poses[key] @ keyframe.T_body_lidar, keyframe.points_lidar)

    def submap(self, center_xyz: np.ndarray, radius_m: float, *, exclude_keys=()) -> np.ndarray:
        """World points of keyframes whose body origin lies within ``radius_m``."""
        excluded = set(exclude_keys)
        keys = [
            key
            for key, pose in self.poses.items()
            if key not in excluded and np.linalg.norm(pose[:3, 3] - center_xyz) <= radius_m
        ]
        return self._merge(keys, self.voxel_m)

    def global_map(self, voxel_m: float = 0.2) -> np.ndarray:
        return self._merge(list(self.poses), voxel_m)

    def _merge(self, keys, voxel_m) -> np.ndarray:
        if not keys:
            return np.empty((0, 3))
        return voxel_downsample(np.vstack([self.points_world(key) for key in keys]), voxel_m)

    def save(self, directory: Path) -> None:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        rows = []
        for key, keyframe in self.keyframes.items():
            name = f"kf_{key[0]}_{key[1]}.npz"
            np.savez(directory / name, points_lidar=keyframe.points_lidar, descriptor=self.descriptors[key])
            row = {
                "key": list(key),
                "stamp_ns": keyframe.stamp_ns,
                "T_odom_body": keyframe.T_odom_body.tolist(),
                "T_world_body": self.poses[key].tolist(),
                "T_body_lidar": keyframe.T_body_lidar.tolist(),
                "file": name,
            }
            if keyframe.visual is not None:
                vname = f"kf_{key[0]}_{key[1]}_visual.npz"
                save_kwargs = {
                    "gray": keyframe.visual.gray,
                    "K": keyframe.visual.K,
                    "T_body_camera": keyframe.visual.T_body_camera,
                    "stamp_ns": np.int64(keyframe.visual.stamp_ns),
                }
                if keyframe.visual.rgb is not None:
                    save_kwargs["rgb"] = keyframe.visual.rgb
                np.savez_compressed(directory / vname, **save_kwargs)
                row["visual_file"] = vname
            rows.append(row)
        document = {"schema": SCHEMA, "voxel_m": self.voxel_m, "keyframes": rows}
        (directory / "map.json").write_text(json.dumps(document))

    @classmethod
    def iter_keyframes_from_dir(
        cls,
        directory: Path,
        *,
        include_descriptors: bool = False,
    ) -> Iterator[tuple[Keyframe, np.ndarray, np.ndarray | None]]:
        """Stream saved keyframes one at a time in numeric ``(epoch, sequence)`` order."""
        directory = Path(directory)
        document = json.loads((directory / "map.json").read_text(encoding="utf-8"))
        if document.get("schema") != SCHEMA:
            raise ValueError(f"{directory}: schema is not {SCHEMA}")
        rows = sorted(
            document["keyframes"],
            key=lambda r: (int(r["key"][0]), int(r["key"][1])),
        )
        for row in rows:
            key = (int(row["key"][0]), int(row["key"][1]))
            npz_path = _resolve_map_file(directory, row["file"])
            with np.load(npz_path) as arrays:
                points = np.array(arrays["points_lidar"], dtype=np.float64)
                descriptor = (
                    np.array(arrays["descriptor"])
                    if include_descriptors and "descriptor" in arrays
                    else None
                )
            keyframe = Keyframe(
                key,
                int(row["stamp_ns"]),
                np.array(row["T_odom_body"], dtype=np.float64),
                np.array(row["T_body_lidar"], dtype=np.float64),
                points,
            )
            pose = np.array(row["T_world_body"], dtype=np.float64)
            yield keyframe, pose, descriptor

    @classmethod
    def load(cls, directory: Path) -> "MapStore":
        directory = Path(directory)
        document = json.loads((directory / "map.json").read_text(encoding="utf-8"))
        if document.get("schema") != SCHEMA:
            raise ValueError(f"{directory}: schema is not {SCHEMA}")
        store = cls(document["voxel_m"])
        for row in document["keyframes"]:
            key = (int(row["key"][0]), int(row["key"][1]))
            npz_path = _resolve_map_file(directory, row["file"])
            with np.load(npz_path) as arrays:
                points, descriptor = arrays["points_lidar"], arrays["descriptor"]
            visual = None
            vfile = row.get("visual_file")
            if vfile:
                vpath = directory / Path(vfile).name
                if vpath.is_file():
                    with np.load(vpath, allow_pickle=False) as vdata:
                        rgb = np.asarray(vdata["rgb"]) if "rgb" in vdata.files else None
                        visual = VisualObservation(
                            gray=np.asarray(vdata["gray"]),
                            K=np.asarray(vdata["K"], dtype=np.float64),
                            T_body_camera=np.asarray(vdata["T_body_camera"], dtype=np.float64),
                            stamp_ns=int(np.asarray(vdata["stamp_ns"]).item()),
                            rgb=rgb,
                        )
            keyframe = Keyframe(
                key,
                int(row["stamp_ns"]),
                np.array(row["T_odom_body"], dtype=np.float64),
                np.array(row["T_body_lidar"], dtype=np.float64),
                points,
                visual=visual,
            )
            store.keyframes[key] = keyframe
            store.poses[key] = np.array(row["T_world_body"], dtype=np.float64)
            store.descriptors[key] = descriptor
        return store
